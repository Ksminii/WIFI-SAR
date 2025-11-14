#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
RSSI 실시간 모니터링 도구

사용법:
  python rssi_monitor.py --iface wlan0mon --ssid EmergencyAP --window 10
"""

import subprocess
import re
import csv
import argparse
import time
from collections import deque


# ========================================
# RSSI 측정 유틸리티 함수
# ========================================

def normalize_ssid(raw_ssid: str):
    """SSID 문자열 정규화 (hex → ASCII 변환, 공백/널 제거)"""
    if not raw_ssid or raw_ssid == "<MISSING>":
        return None
    raw_ssid = raw_ssid.strip().strip("\x00")
    if all(32 <= ord(c) < 127 for c in raw_ssid):
        return raw_ssid
    try:
        if all(c in "0123456789abcdefABCDEF" for c in raw_ssid):
            decoded = bytes.fromhex(raw_ssid).decode("utf-8", errors="ignore")
            return decoded.strip().strip("\x00")
    except Exception:
        pass
    return raw_ssid


def ascii_to_hex(s: str):
    """ASCII 문자열을 hex로 변환"""
    return "".join(f"{ord(c):02x}" for c in s)


def set_channel(iface: str, channel: int):
    """WiFi 인터페이스를 특정 채널로 고정"""
    try:
        subprocess.check_call(
            ["sudo", "iw", "dev", iface, "set", "channel", str(channel)],
            stderr=subprocess.DEVNULL
        )
        return True
    except subprocess.CalledProcessError:
        return False


# ========================================
# 채널 스위핑
# ========================================

def sweep_channels(iface: str, target_ssid: str, candidates):
    """채널 1,6,11 우선 순위 에너지 효율 탐색"""
    sweep_order = [1, 6, 11] + [ch for ch in range(1, 14) if ch not in (1, 6, 11)]
    print(f"[*] 스윕 모드 시작 (탐색 순서: {sweep_order})")

    for ch in sweep_order:
        set_channel(iface, ch)
        # 짧게 tshark 실행 (2초)
        cmd = [
            "tshark",
            "-i", iface,
            "-a", "duration:2",
            "-Y", "wlan.fc.type_subtype == 0x08",
            "-T", "fields",
            "-e", "wlan.ssid",
        ]
        try:
            out = subprocess.check_output(cmd, stderr=subprocess.DEVNULL, text=True)
        except subprocess.CalledProcessError:
            continue

        for line in out.splitlines():
            ssid_norm = normalize_ssid(line.strip())
            ssid_raw = line.strip()
            if ((ssid_norm and ssid_norm.lower() in candidates) or
                (ssid_raw and ssid_raw.lower() in candidates)):
                print(f"[*] 스윕 모드에서 SSID '{target_ssid}' 발견 → 채널 {ch}")
                return ch
    print("[!] 스윕 모드 실패: SSID 찾지 못함")
    return None


# ========================================
# RSSI 모니터링 모드
# ========================================

def rssi_monitor_mode(iface, target_ssid, window=10):
    """
    RSSI 실시간 모니터링 모드

    Args:
        iface: 모니터 모드 인터페이스 (예: wlan0mon)
        target_ssid: 타겟 SSID
        window: 윈도우 크기(초)
    """
    # 후보 SSID 집합
    norm_target = normalize_ssid(target_ssid)
    candidates = set()
    if norm_target:
        candidates.add(norm_target.lower())
        if all(32 <= ord(c) < 127 for c in norm_target):
            candidates.add(ascii_to_hex(norm_target).lower())
    candidates.add(target_ssid.lower())

    # 스윕 모드 바로 실행
    channel = sweep_channels(iface, target_ssid, candidates)
    if channel:
        set_channel(iface, channel)
    else:
        print("[!] 타겟 SSID를 찾지 못했음. 기본 채널에서 재시도 시도")

    cmd = [
        "tshark",
        "-i", iface,
        "-l",
        "-Y", "wlan.fc.type_subtype == 0x08",
        "-T", "fields",
        "-E", "separator=,", "-E", "quote=d", "-E", "header=n",
        "-e", "wlan.ssid",
        "-e", "radiotap.dbm_antsignal",
        "-e", "frame.time_epoch",
    ]

    print("Running:", " ".join(cmd))
    process = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True)

    reader = csv.reader(process.stdout)
    samples = deque()
    start_time = time.time()

    try:
        for row in reader:
            if len(row) < 3:
                continue
            raw_ssid, rssi, ts = row
            ssid_norm = normalize_ssid(raw_ssid)
            ssid_raw = raw_ssid.strip()

            if ((ssid_norm and ssid_norm.lower() in candidates) or
                (ssid_raw and ssid_raw.lower() in candidates)):
                try:
                    rssi_val = int(rssi.split(",")[0])
                    samples.append(rssi_val)
                except ValueError:
                    continue

            now = time.time()
            if now - start_time >= window:
                if samples:
                    n = len(samples)
                    if n > 15:
                        sorted_samples = sorted(samples)
                        k = max(1, int(n * 0.1))  # 상하위 10% 개수
                        trimmed = sorted_samples[k:-k] if n > 2 * k else sorted_samples
                        avg_rssi = sum(trimmed) / len(trimmed)
                    else:
                        avg_rssi = sum(samples) / n

                    print(f"[{time.strftime('%Y-%m-%d %H:%M:%S')}] "
                          f'SSID="{target_ssid}": count={n}, avg_rssi={avg_rssi:.1f} dBm')
                else:
                    print(f"[{time.strftime('%Y-%m-%d %H:%M:%S')}] "
                          f'SSID="{target_ssid}": count=0 (no samples)')
                samples.clear()
                start_time = now

    except KeyboardInterrupt:
        print("\nStopping...")
    finally:
        process.terminate()
        process.wait()


# ========================================
# 메인 실행
# ========================================

if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="RSSI 실시간 모니터링 도구",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
사용 예시:
  python rssi_monitor.py --iface wlan0mon --ssid EmergencyAP --window 10
        """
    )

    parser.add_argument("--iface", default="wlan0mon",
                       help="모니터 모드 인터페이스 (기본: wlan0mon)")
    parser.add_argument("--ssid", required=True,
                       help="타겟 SSID (필수)")
    parser.add_argument("--window", type=int, default=10,
                       help="RSSI 측정 윈도우 크기(초, 기본: 10)")

    args = parser.parse_args()

    print("\n" + "="*60)
    print("RSSI 실시간 모니터링 모드")
    print(f"타겟 SSID: {args.ssid}")
    print(f"인터페이스: {args.iface}")
    print(f"윈도우: {args.window}초")
    print("="*60 + "\n")

    rssi_monitor_mode(args.iface, args.ssid, args.window)
