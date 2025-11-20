#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
========================================
드론 신호 탐색 시스템 (속도 기반 5방향 탐색)
========================================

[실행 환경]
- 하드웨어: 라즈베리파이 + 픽스호크
- 연결: /dev/ttyAMA0 (시리얼 연결)
- Python 3.12 호환

[주요 기능]
1. 5방향 신호 탐색 (중앙, 북, 동, 남, 서)
2. 모든 이동을 속도 기반(BODY_NED)으로 수행 (GPS/Compass/Guided 불필요)
3. 각 방향으로 일정 시간 이동 후 그 지점에서 RSSI 측정
4. 중앙이 가장 강하면 탐색 종료 (정점 도달로 판단)
5. 최적 방향으로 추가 이동 후 라운드 반복

[사용 방법]
1. 조종기로 드론 수동 이륙 및 고도 유지 (ALT HOLD 추천)
2. python3 simple.py 실행
3. Enter를 누르면 5방향 탐색 시작

[설정 변수]
- MOVE_SPEED         : 속도 (m/s)
- MOVE_DURATION      : 각 방향으로 이동 시간 (초)
- MOVE_MULTIPLIER    : 최적 방향 추가 이동 시간 배수
- MAX_ROUNDS         : 탐색 라운드 수
- TARGET_SSID        : 타겟 Wi-Fi SSID
- MONITOR_INTERFACE  : 모니터 인터페이스 (wlan0mon)
========================================
"""

# Python 3.12 호환성 패치
import collections
import collections.abc
collections.MutableMapping = collections.abc.MutableMapping

from dronekit import connect, VehicleMode
from pymavlink import mavutil
import time
import subprocess


# ========================================
# 설정
# ========================================

TARGET_SSID = "Victim_Phone_WiFi"
MONITOR_INTERFACE = "wlan0mon"

MOVE_SPEED = 1.0        # m/s
MOVE_DURATION = 10      # 초
MOVE_MULTIPLIER = 2.0   # 최적 방향으로 추가 이동 시 시간 배수
MAX_ROUNDS = 5
RSSI_THRESHOLD_STOP = -20
TSHARK_DURATION = 3


# ========================================
# RSSI 측정 유틸
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


def get_signal_strength(target_ssid, iface="wlan0mon", duration=3):
    """
    tshark를 사용한 정밀 RSSI 측정

    Returns:
        평균 RSSI (dBm) 또는 None
    """
    if not target_ssid:
        print("[오류] get_signal_strength: target_ssid가 지정되지 않았습니다.")
        return None

    try:
        norm_target = normalize_ssid(target_ssid)
        candidates = set()
        if norm_target:
            candidates.add(norm_target.lower())
            if all(32 <= ord(c) < 127 for c in norm_target):
                candidates.add(ascii_to_hex(norm_target).lower())
        candidates.add(target_ssid.lower())

        cmd = [
            #"sudo",
            "tshark",
            "-i", iface,
            "-a", f"duration:{duration}",
            "-Y", "wlan.fc.type_subtype == 0x08",  # 비컨 프레임
            "-T", "fields",
            "-E", "separator=,",
            "-e", "wlan.ssid",
            "-e", "radiotap.dbm_antsignal",
        ]

        output = subprocess.check_output(cmd, stderr=subprocess.DEVNULL, text=True)

        samples = []
        for line in output.splitlines():
            parts = line.strip().split(',')
            if len(parts) < 2:
                continue

            raw_ssid, rssi = parts[0], parts[1]
            ssid_norm = normalize_ssid(raw_ssid)
            ssid_raw = raw_ssid.strip()

            if ((ssid_norm and ssid_norm.lower() in candidates) or
                (ssid_raw and ssid_raw.lower() in candidates)):
                try:
                    rssi_val = int(rssi.split(",")[0])
                    samples.append(rssi_val)
                except ValueError:
                    continue

        if samples:
            n = len(samples)
            if n > 15:
                sorted_samples = sorted(samples)
                k = max(1, int(n * 0.1))
                trimmed = sorted_samples[k:-k] if n > 2 * k else sorted_samples
                avg_rssi = sum(trimmed) / len(trimmed)
            else:
                avg_rssi = sum(samples) / n
            return avg_rssi
        else:
            print(f"[신호 측정] SSID '{target_ssid}'에 대한 샘플 없음.")
            return None

    except Exception as e:
        print(f"[오류] tshark 실행 실패: {e}")
        return None


# ========================================
# 속도 기반 이동
# ========================================

def send_velocity_body(vehicle, vx, vy, vz, duration):
    """
    BODY_NED 기준 속도 명령을 duration 초 동안 반복 전송
    """
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,
        0, 0,
        mavutil.mavlink.MAV_FRAME_BODY_NED,
        0b0000111111000111,  # 속도만 사용
        0, 0, 0,
        vx, vy, vz,
        0, 0, 0,
        0, 0
    )

    print(f"속도명령: vx={vx}, vy={vy}, vz={vz}, duration={duration}s")
    for _ in range(int(duration * 10)):  # 10Hz
        vehicle.send_mavlink(msg)
        vehicle.flush()
        time.sleep(0.1)

    # 정지 명령
    stop_msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,
        0, 0,
        mavutil.mavlink.MAV_FRAME_BODY_NED,
        0b0000111111000111,
        0, 0, 0,
        0, 0, 0,
        0, 0, 0,
        0, 0
    )
    vehicle.send_mavlink(stop_msg)
    vehicle.flush()


# ========================================
# 5방향 탐색 (중앙 + 4방향)
# ========================================

def search_five_directions(vehicle, target_ssid, iface):
    """
    중앙/북/동/남/서 5방향에서 각각 RSSI를 측정하고
    가장 강한 방향을 반환
    """
    print("\n" + "="*60)
    print(f"5방향 신호 탐색 시작 (속도={MOVE_SPEED} m/s, 시간={MOVE_DURATION}s)")
    print("="*60 + "\n")

    # 이름 → (vx, vy) 매핑 (BODY_NED)
    dir_vectors = {
        '중앙': (0.0, 0.0),
        '북쪽': (+MOVE_SPEED, 0.0),
        '동쪽': (0.0, +MOVE_SPEED),
        '남쪽': (-MOVE_SPEED, 0.0),
        '서쪽': (0.0, -MOVE_SPEED)
    }

    order = ['중앙', '북쪽', '남쪽', '동쪽', '서쪽']
    signal_results = {}

    def measure_signal(point_name):
        print(f"\n{'─'*50}")
        print(f"[{point_name}] 지점 신호 측정 중... (tshark {TSHARK_DURATION}초)")
        print(f"{'─'*50}")
        time.sleep(2)

        signals = []
        for i in range(3):
            sig = get_signal_strength(target_ssid=target_ssid, iface=iface, duration=TSHARK_DURATION)
            if sig is not None:
                signals.append(sig)
                print(f"  신호 {i+1}: {sig:.2f} dBm")
            else:
                print(f"  신호 {i+1}: 측정 실패 (None)")
            time.sleep(0.5)

        if signals:
            avg_signal = sum(signals) / len(signals)
            print(f"→ 평균: {avg_signal:.2f} dBm")
        else:
            print(f"⚠️ {point_name} 신호 측정 실패 (모두 None)")
            avg_signal = -100
        signal_results[point_name] = avg_signal
        return avg_signal

    # 중앙: 이동 없이 측정만
    print("▶ 중앙 지점 신호 측정")
    measure_signal('중앙')

    # 나머지 4방향: 이동 → 측정 → 반대 방향으로 복귀 시도
    for name in order[1:]:
        vx, vy = dir_vectors[name]
        print(f"\n▶ {name} 방향으로 이동")
        send_velocity_body(vehicle, vx, vy, 0, MOVE_DURATION)
        measure_signal(name)
        print(f"◀ {name} 반대 방향으로 복귀 시도")
        send_velocity_body(vehicle, -vx, -vy, 0, MOVE_DURATION)
        time.sleep(2)

    # 결과 표시
    print(f"\n{'='*60}")
    print("신호 측정 결과:")
    print(f"{'='*60}")
    for direction, signal in signal_results.items():
        bar = '█' * max(0, int((signal + 100) / 2))
        print(f"  {direction:4s}: {signal:6.2f} dBm  {bar}")

    best_direction = max(signal_results, key=signal_results.get)
    best_signal = signal_results[best_direction]

    print(f"\n✓ 최고 신호: {best_direction} ({best_signal:.2f} dBm)")
    print(f"{'='*60}\n")

    vx_best, vy_best = dir_vectors[best_direction]
    return best_direction, best_signal, (vx_best, vy_best)


# ========================================
# 드론 연결 및 착륙
# ========================================

def connect_vehicle():
    print("픽스호크 연결 중... (/dev/ttyAMA0, 57600)")
    vehicle = connect('/dev/ttyAMA0', wait_ready=True, baud=57600) #라파
    #vehicle = connect('/dev/ttyACM0', baud=115200, wait_ready=True) #VM
    #vehicle = connect('COM5', baud=115200, wait_ready=False) #VScode
    print("\n=== 연결 완료 ===")
    print(f"Firmware Version: {vehicle.version}")
    print(f"Battery: {vehicle.battery.level}%")
    print(f"Current Mode: {vehicle.mode.name}")
    print("==================\n")

    return vehicle


def land(vehicle):
    print("\n=== 착륙 시작 ===")
    vehicle.mode = VehicleMode("LAND")
    while vehicle.armed:
        alt = vehicle.location.global_relative_frame.alt
        if alt is not None:
            print(f"착륙 중... 고도: {alt:.1f}m")
        time.sleep(1)
    print("✓ 착륙 완료!\n")


# ========================================
# 메인
# ========================================

def main():
    vehicle = None

    try:
        vehicle = connect_vehicle()
        time.sleep(2)

        print("=" * 60)
        print("드론이 이륙한 상태인지 확인하세요!")
        print("수동으로 이륙 후 ALT HOLD 등으로 호버링 유지.")
        print("Enter를 눌러 5방향 신호 탐색을 시작하세요.")
        print(f"목표 SSID: '{TARGET_SSID}'")
        print("=" * 60)
        input()

        print(f"현재 모드: {vehicle.mode.name}")
        print("모드 변경 없이 속도 기반 탐색을 진행합니다.\n")

        for round_num in range(1, MAX_ROUNDS + 1):
            print("\n" + "#" * 60)
            print(f"###  탐색 라운드 {round_num}/{MAX_ROUNDS}  ###")
            print("#" * 60)

            best_dir, best_sig, (vx_best, vy_best) = search_five_directions(
                vehicle,
                target_ssid=TARGET_SSID,
                iface=MONITOR_INTERFACE
            )

            if best_sig is not None and best_sig > RSSI_THRESHOLD_STOP:
                print(f"\n신호 강도 충분: {best_sig:.2f} dBm")
                print("목표 지점 근처 도달로 판단, 탐색을 종료합니다.")
                break

            if best_dir == '중앙':
                print(f"\n'중앙' 지점 신호({best_sig:.2f} dBm)가 가장 강합니다.")
                print("신호 정점에 도달한 것으로 판단, 탐색을 종료합니다.")
                break

            extra_time = MOVE_DURATION * MOVE_MULTIPLIER
            print(f"\n{best_dir} 방향으로 추가 이동 ({extra_time:.1f}초)...")
            send_velocity_body(vehicle, vx_best, vy_best, 0, extra_time)
            print(f"✓ {best_dir} 방향 추가 이동 완료\n")

            if round_num < MAX_ROUNDS:
                wait_time = 3
                print(f"다음 라운드까지 {wait_time}초 대기...\n")
                time.sleep(wait_time)
            else:
                print("최대 탐색 라운드 도달. 미션을 종료합니다.")

        print("\n" + "=" * 60)
        print("모든 탐색 완료!")
        print("=" * 60)

        land(vehicle)
        print("\n✓✓✓ 미션 완료! ✓✓✓\n")

    except KeyboardInterrupt:
        print("\n\n⚠️ 사용자 중단!")
        if vehicle and vehicle.armed:
            print("긴급 착륙!")
            vehicle.mode = VehicleMode("LAND")
            time.sleep(10)

    except Exception as e:
        print(f"\n오류 발생: {e}")
        import traceback
        traceback.print_exc()
        if vehicle and vehicle.armed:
            print("\n긴급 착륙!")
            vehicle.mode = VehicleMode("LAND")
            time.sleep(10)

    finally:
        if vehicle:
            print("\n연결 종료...")
            vehicle.close()
            print("완료.\n")


if __name__ == "__main__":
    main()
