#!/usr/bin/env python3
import argparse
import subprocess
import time
import csv
from collections import deque
import sys


def normalize_ssid(raw_ssid: str):
    """SSID 문자열을 정규화 (hex → ASCII 변환, 공백/널 제거)"""
    if not raw_ssid or raw_ssid == "<MISSING>":
        return None
    raw_ssid = raw_ssid.strip().strip("\x00")
    if all(32 <= ord(c) < 127 for c in raw_ssid):
        return raw_ssid
    try:
        if all(c in "0123456789abcdefABCDEF" for c in raw_ssid):
            decoded = bytes.fromhex(raw_ssid).decode("utf-8", errors="ignore")
            return decoded.strip().strip("\x00")
    except:
        pass
    return raw_ssid


def ascii_to_hex(s: str):
    return "".join(f"{ord(c):02x}" for c in s)


def set_channel(iface: str, channel: int):
    try:
        subprocess.check_call(
            ["sudo", "iw", "dev", iface, "set", "channel", str(channel)],
            stderr=subprocess.DEVNULL,
        )
        print(f"[*] 인터페이스 {iface} 채널 {channel} 으로 고정 완료")
        return True
    except subprocess.CalledProcessError:
        print(f"[!] 채널 {channel} 고정 실패")
        return False


def sweep_channels(iface: str, target_ssid: str, candidates):
    """채널 1,6,11 우선 → 나머지 순차 탐색"""
    sweep_order = [1, 6, 11] + [ch for ch in range(1, 14) if ch not in (1, 6, 11)]
    print(f"[*] 스윕 모드 시작 (탐색 순서: {sweep_order})")

    for ch in sweep_order:
        set_channel(iface, ch)
        cmd = [
            "tshark",
            "-i",
            iface,
            "-a",
            "duration:2",
            "-Y",
            "wlan.fc.type_subtype == 0x08",
            "-T",
            "fields",
            "-e",
            "wlan.ssid",
        ]
        try:
            out = subprocess.check_output(cmd, stderr=subprocess.DEVNULL, text=True)
        except:
            continue

        for line in out.splitlines():
            ssid_norm = normalize_ssid(line.strip())
            ssid_raw = line.strip()
            if (ssid_norm and ssid_norm.lower() in candidates) or (
                ssid_raw and ssid_raw.lower() in candidates
            ):
                print(f"[*] 스윕 모드에서 SSID '{target_ssid}' 발견 → 채널 {ch}")
                return ch

    print("[!] 스윕 모드 실패: SSID 찾지 못함")
    return None


def main():
    parser = argparse.ArgumentParser(
        description="Track RSSI of a specific SSID (Manual Enter Trigger Mode)"
    )
    parser.add_argument(
        "--iface", required=True, help="Monitor mode interface (예: wlan0mon)"
    )
    parser.add_argument("--ssid", required=True, help="Target SSID")
    parser.add_argument("--window", type=int, default=10, help="측정 시간 (초)")
    args = parser.parse_args()

    # SSID 후보군 구성
    norm_target = normalize_ssid(args.ssid)
    candidates = set()
    if norm_target:
        candidates.add(norm_target.lower())
        if all(32 <= ord(c) < 127 for c in norm_target):
            candidates.add(ascii_to_hex(norm_target).lower())
    candidates.add(args.ssid.lower())

    # 스윕 모드 먼저 수행
    channel = sweep_channels(args.iface, args.ssid, candidates)
    if channel:
        set_channel(args.iface, channel)
    else:
        print("[!] 스윕 실패 → 기본 채널에서 수집 시도")

    window = args.window

    # tshark 실시간 스트림 시작
    cmd = [
        "tshark",
        "-i",
        args.iface,
        "-l",
        "-Y",
        "wlan.fc.type_subtype == 0x08",
        "-T",
        "fields",
        "-E",
        "separator=,",
        "-E",
        "quote=d",
        "-E",
        "header=n",
        "-e",
        "wlan.ssid",
        "-e",
        "radiotap.dbm_antsignal",
        "-e",
        "frame.time_epoch",
    ]

    print("Running:", " ".join(cmd))
    process = subprocess.Popen(
        cmd, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True
    )
    reader = csv.reader(process.stdout)

    print("\n========================================")
    print(" ENTER를 누르면 즉시 측정을 시작합니다.")
    print(f" 측정 시간: {window}초")
    print("========================================\n")

    try:
        while True:
            input("▶ 엔터를 누르면 측정 시작...\n")

            print("[측정 시작] -------------------------------")
            samples = deque()
            start_t = time.time()

            # window초 동안만 수집
            while time.time() - start_t < window:
                try:
                    row = next(reader)
                except:
                    continue

                if len(row) < 3:
                    continue

                raw_ssid, rssi, ts = row
                ssid_norm = normalize_ssid(raw_ssid)
                ssid_raw = raw_ssid.strip()

                if (ssid_norm and ssid_norm.lower() in candidates) or (
                    ssid_raw and ssid_raw.lower() in candidates
                ):

                    try:
                        val = int(rssi.split(",")[0])
                        samples.append(val)
                    except:
                        pass

            # 결과 출력
            print("\n[측정 결과]")
            if samples:
                n = len(samples)
                if n > 15:
                    sorted_s = sorted(samples)
                    k = max(1, int(n * 0.1))
                    trimmed = sorted_s[k:-k] if n > 2 * k else sorted_s
                    avg_rssi = sum(trimmed) / len(trimmed)
                else:
                    avg_rssi = sum(samples) / n

                print(f"  샘플 수: {n}")
                print(f"  평균 RSSI: {avg_rssi:.1f} dBm\n")
            else:
                print("  SSID 샘플 없음\n")

    except KeyboardInterrupt:
        print("\n사용자 종료")

    finally:
        process.terminate()
        process.wait()
        print("tshark 종료됨.")


if __name__ == "__main__":
    main()
