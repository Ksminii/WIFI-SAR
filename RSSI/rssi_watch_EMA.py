#!/usr/bin/env python3
import argparse
import subprocess
import time
import csv
from collections import deque
import sys

############################################################
# 1) SSID Normalization
############################################################


def normalize_ssid(raw_ssid):
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


def ascii_to_hex(s):
    return "".join(f"{ord(c):02x}" for c in s)


############################################################
# 2) EMA Filter Class
############################################################


class EMAFilter:
    def __init__(self, alpha=0.3):
        self.alpha = alpha
        self.value = None

    def update(self, new_value):
        if self.value is None:
            self.value = new_value
        else:
            self.value = self.alpha * new_value + (1 - self.alpha) * self.value
        return self.value


############################################################
# 3) Channel Sweep (same)
############################################################


def set_channel(iface, channel):
    try:
        subprocess.check_call(
            ["sudo", "iw", "dev", iface, "set", "channel", str(channel)],
            stderr=subprocess.DEVNULL,
        )
        print(f"[*] 채널 {channel} 고정")
        return True
    except:
        print(f"[!] 채널 {channel} 고정 실패")
        return False


def sweep_channels(iface, target_ssid, candidates):
    order = [1, 6, 11] + [ch for ch in range(1, 14) if ch not in (1, 6, 11)]
    print("[*] 스윕 모드 시작")

    for ch in order:
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
                print(f"[*] SSID '{target_ssid}' 발견 → 채널 {ch}")
                return ch

    print("[!] 스윕 실패")
    return None


############################################################
# 4) MAIN
############################################################


def main():
    parser = argparse.ArgumentParser(description="RSSI Tracker with EMA Filter")
    parser.add_argument("--iface", required=True)
    parser.add_argument("--ssid", required=True)
    parser.add_argument("--window", type=int, default=10)
    args = parser.parse_args()

    # SSID 후보
    norm = normalize_ssid(args.ssid)
    candidates = set()
    if norm:
        candidates.add(norm.lower())
        if all(32 <= ord(c) < 127 for c in norm):
            candidates.add(ascii_to_hex(norm).lower())
    candidates.add(args.ssid.lower())

    # 스윕
    ch = sweep_channels(args.iface, args.ssid, candidates)
    if ch:
        set_channel(args.iface, ch)

    # EMA 필터 생성
    ema = EMAFilter(alpha=0.3)

    # tshark 스트리밍
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

    print("\n=== ENTER-triggered RSSI Measurement (EMA-Version) ===\n")

    try:
        while True:
            input("▶ 엔터를 누르면 측정 시작...\n")

            print("[측정 시작]")
            samples = []
            start = time.time()

            while time.time() - start < args.window:
                try:
                    row = next(reader)
                except:
                    continue
                if len(row) < 2:
                    continue

                raw_ssid, rssi, ts = row
                ssid_norm = normalize_ssid(raw_ssid)
                ssid_raw = raw_ssid.strip()

                if (ssid_norm and ssid_norm.lower() in candidates) or (
                    ssid_raw and ssid_raw.lower() in candidates
                ):

                    try:
                        samples.append(int(rssi.split(",")[0]))
                    except:
                        pass

            print("\n[측정 결과]")

            if not samples:
                print("  샘플 없음\n")
                continue

            # 1) Trimmed Mean
            n = len(samples)
            if n > 15:
                sorted_s = sorted(samples)
                k = max(1, int(n * 0.1))
                trimmed = sorted_s[k:-k] if n > 2 * k else sorted_s
                avg = sum(trimmed) / len(trimmed)
            else:
                avg = sum(samples) / n

            # 2) EMA 적용
            filtered = ema.update(avg)

            print(f"  원본 평균: {avg:.1f} dBm")
            print(f"  EMA 필터 적용: {filtered:.1f} dBm\n")

    except KeyboardInterrupt:
        print("\n종료")
    finally:
        process.terminate()
        process.wait()


if __name__ == "__main__":
    main()
