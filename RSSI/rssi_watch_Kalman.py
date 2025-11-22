#!/usr/bin/env python3
import argparse
import subprocess
import time
import csv
from collections import deque
import sys

############################################################
# 1) SSID Normalization / Utility
############################################################


def normalize_ssid(raw_ssid: str):
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


############################################################
# 2) Kalman Filter Class
############################################################


class KalmanFilter:
    def __init__(self, Q=1e-3, R=5):
        self.Q = Q
        self.R = R
        self.x = None
        self.P = 1.0

    def update(self, z):
        if self.x is None:
            self.x = z
            return z

        # Prediction
        self.P = self.P + self.Q

        # Kalman Gain
        K = self.P / (self.P + self.R)

        # Update estimate
        self.x = self.x + K * (z - self.x)

        # Update covariance
        self.P = (1 - K) * self.P

        return self.x


############################################################
# 3) Channel Sweep (same as your original)
############################################################


def set_channel(iface: str, channel: int):
    try:
        subprocess.check_call(
            ["sudo", "iw", "dev", iface, "set", "channel", str(channel)],
            stderr=subprocess.DEVNULL,
        )
        print(f"[*] 인터페이스 {iface} 채널 {channel} 으로 고정 완료")
        return True
    except:
        print(f"[!] 채널 {channel} 고정 실패")
        return False


def sweep_channels(iface: str, target_ssid: str, candidates):
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


############################################################
# 4) MAIN
############################################################


def main():
    parser = argparse.ArgumentParser(description="RSSI Tracker with Kalman Filter")
    parser.add_argument("--iface", required=True)
    parser.add_argument("--ssid", required=True)
    parser.add_argument("--window", type=int, default=10)
    args = parser.parse_args()

    # SSID 후보 구성
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

    # Kalman Filter 생성
    kf = KalmanFilter(Q=1e-3, R=5)

    # tshark 실시간 스트리밍
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

    print("\n=== ENTER-triggered RSSI Measurement (Kalman-Version) ===\n")

    try:
        while True:
            input("▶ 엔터를 누르면 측정 시작...\n")

            print("[측정 시작]")
            samples = []
            start = time.time()

            # window초 동안 수집
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

            # 2) Kalman Filter 적용
            filtered = kf.update(avg)

            print(f"  원본 평균: {avg:.1f} dBm")
            print(f"  칼만 필터 적용: {filtered:.1f} dBm\n")

    except KeyboardInterrupt:
        print("\n종료")
    finally:
        process.terminate()
        process.wait()


if __name__ == "__main__":
    main()
