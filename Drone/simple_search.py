#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
========================================
드론 4방향 신호 탐색 및 이동
========================================

[실행 환경]
- 하드웨어: 라즈베리파이 + 픽스호크
- 연결: /dev/ttyAMA0 (시리얼 연결)
- Python 3.12 호환

[주요 기능]
1. 4방향 신호 탐색 (북, 동, 남, 서)
2. tshark를 이용한 정밀 RSSI 측정
3. 각 방향 측정 후 원위치 복귀 (효율적)
4. 최고 신호 방향으로 자동 이동
5. 반복 탐색

[simple.py와의 차이점]
- simple.py: 5방향 (중앙 포함), 6회 이동
- simple_search.py: 4방향, 8회 이동 (각 방향마다 복귀)

[사용 방법]
1. 드론 수동 이륙 (조종기 사용)
2. GUIDED 모드로 변경
3. python3 simple_search.py 실행
4. 자동 탐색 시작

[설정 변수]
- TARGET_SSID: 탐색할 Wi-Fi SSID
- MONITOR_INTERFACE: Wi-Fi 모니터 인터페이스 (wlan0mon)
- SEARCH_DISTANCE: 탐색 거리 (기본 10m)
- MOVE_MULTIPLIER: 이동 배수 (기본 2.0)
- MAX_ROUNDS: 최대 탐색 라운드 (기본 5회)

[참고]
- 5방향 탐색을 원하면 simple.py 사용 권장
========================================
"""

# Python 3.12 호환성 패치
import collections
import collections.abc
collections.MutableMapping = collections.abc.MutableMapping

from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil
import time
import math
import subprocess


# ========================================
# 스크립트 설정
# ========================================
# [필수] 탐색할 실종자 디바이스의 Wi-Fi SSID
TARGET_SSID = "Victim_Phone_WiFi"

# [필수] tshark가 사용할 모니터 모드 인터페이스
MONITOR_INTERFACE = "wlan0mon"

# 탐색 및 이동 관련 설정
SEARCH_DISTANCE = 10.0  # 4방향 탐색 시 이동할 거리 (미터)
MOVE_MULTIPLIER = 2.0   # 최적 방향으로 이동 시, 탐색 거리의 배수
MAX_ROUNDS = 5          # 최대 탐색 라운드
RSSI_THRESHOLD_STOP = -20  # 이 신호 강도(dBm) 이상이면 탐색 중지
TSHARK_DURATION = 3     # tshark 측정 시간 (초)


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


# ========================================
# 신호 측정 함수 (tshark 기반 정밀 측정)
# ========================================

def get_signal_strength(target_ssid, iface="wlan0mon", duration=3):
    """
    tshark를 사용한 정밀 RSSI 측정

    Args:
        target_ssid: (필수) 타겟 SSID
        iface: 모니터 모드 인터페이스
        duration: 측정 시간(초)

    Returns:
        평균 RSSI 값 (dBm) 또는 None (측정 실패)
    """
    if not target_ssid:
        print("[오류] get_signal_strength: target_ssid가 지정되지 않았습니다.")
        return None

    # tshark로 정밀 측정
    try:
        # SSID 후보 생성
        norm_target = normalize_ssid(target_ssid)
        candidates = set()
        if norm_target:
            candidates.add(norm_target.lower())
            if all(32 <= ord(c) < 127 for c in norm_target):
                candidates.add(ascii_to_hex(norm_target).lower())
        candidates.add(target_ssid.lower())

        # tshark 명령
        cmd = [
            "tshark",
            "-i", iface,
            "-a", f"duration:{duration}",
            "-Y", "wlan.fc.type_subtype == 0x08", # 비컨 프레임
            "-T", "fields",
            "-E", "separator=,",
            "-e", "wlan.ssid",
            "-e", "radiotap.dbm_antsignal",
        ]

        output = subprocess.check_output(cmd, stderr=subprocess.DEVNULL, text=True)

        # RSSI 샘플 수집
        samples = []
        for line in output.splitlines():
            parts = line.strip().split(',')
            if len(parts) < 2:
                continue

            raw_ssid, rssi = parts[0], parts[1]
            ssid_norm = normalize_ssid(raw_ssid)
            ssid_raw = raw_ssid.strip()

            # 타겟 SSID 매칭
            if ((ssid_norm and ssid_norm.lower() in candidates) or
                (ssid_raw and ssid_raw.lower() in candidates)):
                try:
                    rssi_val = int(rssi.split(",")[0])
                    samples.append(rssi_val)
                except ValueError:
                    continue

        # 평균 계산 (outlier 제거)
        if samples:
            n = len(samples)
            if n > 15:
                sorted_samples = sorted(samples)
                k = max(1, int(n * 0.1))  # 상하위 10% 제거
                trimmed = sorted_samples[k:-k] if n > 2 * k else sorted_samples
                avg_rssi = sum(trimmed) / len(trimmed)
            else:
                avg_rssi = sum(samples) / n
            return avg_rssi
        else:
            return None

    except Exception as e:
        print(f"[오류] tshark 실행 실패: {e}")
        return None


# ========================================
# NED 좌표 기반 이동
# ========================================

def goto_position_ned(vehicle, north, east, down, timeout=60):
    """
    NED 로컬 좌표로 이동 (원점 기준 상대좌표)
    MAVLink 메시지를 2Hz(0.5초 간격)로 지속적으로 전송
    """
    # MAVLink 메시지 생성
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 1,    # target system, target component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,  # frame
        0b0000111111111000,  # type_mask (only positions enabled)
        north, east, down,   # x, y, z positions (in meters)
        0, 0, 0,  # x, y, z velocity (not used)
        0, 0, 0,  # x, y, z acceleration (not used)
        0, 0)     # yaw, yaw_rate (not used)

    start_time = time.time()

    while time.time() - start_time < timeout:
        # 메시지를 루프 내에서 지속적으로 전송
        vehicle.send_mavlink(msg)

        current_north = vehicle.location.local_frame.north
        current_east = vehicle.location.local_frame.east
        current_down = vehicle.location.local_frame.down

        if current_north is None or current_east is None:
            print("로컬 좌표 대기 중...")
            time.sleep(0.5)
            continue

        # 목표까지 거리 계산
        distance = math.sqrt((north - current_north)**2 + (east - current_east)**2)
        print(f" 목표까지 {distance:.2f}m | 속도: {vehicle.groundspeed:.2f}m/s | NED: ({current_north:.1f}, {current_east:.1f}, {current_down:.1f})")

        # 2m 이내 도착으로 판단
        if distance < 2:
            print("도착")
            return True

        # 2Hz 전송을 위한 0.5초 대기
        time.sleep(0.5)

    print(" 타임아웃")
    return False


# ========================================
# 4방향 신호 탐색
# ========================================

def search_four_directions(vehicle, target_ssid, iface, search_distance=10):
    """
    북/동/남/서 4방향 탐색 후 최적 방향 반환
    """
    print("\n" + "="*60)
    print(f"4방향 신호 탐색 시작 (탐색 거리: {search_distance}m)")
    print("="*60 + "\n")

    # EKF(로컬 좌표)가 준비될 때까지 안전 대기
    print("로컬 좌표계(EKF) 준비 대기 중...")
    ekf_timeout = 30  # 30초
    ekf_start_time = time.time()
    while vehicle.location.local_frame.north is None:
        if time.time() - ekf_start_time > ekf_timeout:
            print("[경고] EKF 초기화 타임아웃! 긴급 착륙")
            vehicle.mode = VehicleMode("LAND")
            raise Exception("EKF 초기화 타임아웃")
        time.sleep(1)
        print("... EKF 대기 ...")

    # 시작 위치 저장 (NED 로컬 좌표)
    home_north = vehicle.location.local_frame.north
    home_east = vehicle.location.local_frame.east
    home_down = vehicle.location.local_frame.down

    print(f"시작 위치 (NED):")
    print(f"  North: {home_north:.2f}m")
    print(f"  East: {home_east:.2f}m")
    print(f"  Down: {home_down:.2f}m\n")

    signal_results = {}

    # 신호 측정 헬퍼 함수
    def measure_signal(point_name):
        print(f"\n{'─'*50}")
        print(f"[{point_name}] 지점 신호 측정 중... (tshark {TSHARK_DURATION}초)")
        print(f"{'─'*50}")
        time.sleep(2) # 안정화 대기
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
            avg_signal = -100 # 실패 시 최저값 할당

        signal_results[point_name] = avg_signal
        return avg_signal

    # 4방향 정의: (이름, dNorth, dEast)
    directions = [
        ("북쪽", search_distance, 0),
        ("동쪽", 0, search_distance),
        ("남쪽", -search_distance, 0),
        ("서쪽", 0, -search_distance)
    ]

    # 각 방향 탐색
    for dir_name, dN, dE in directions:
        print(f"\n{dir_name}으로 이동")
        target_north = home_north + dN
        target_east = home_east + dE
        goto_position_ned(vehicle, target_north, target_east, home_down, timeout=60)
        measure_signal(dir_name)

        # 원위치 복귀
        print(f"원위치 복귀")
        goto_position_ned(vehicle, home_north, home_east, home_down, timeout=60)
        time.sleep(2)

    # 결과 출력
    print(f"\n{'='*60}")
    print("신호 측정 결과:")
    print(f"{'='*60}")

    for direction, signal in sorted(signal_results.items()):
        bar = '█' * int((signal + 100) / 2)  # 시각화
        print(f"  {direction:6s}: {signal:6.2f} dBm  {bar}")

    # 최적 방향
    best_direction = max(signal_results, key=signal_results.get)
    best_signal = signal_results[best_direction]

    print(f"\n✓ 최고 신호: {best_direction} ({best_signal:.2f} dBm)")
    print(f"{'='*60}\n")

    # 다음 이동을 위한 방향 벡터
    direction_vectors = {
        '북쪽': (search_distance, 0),
        '동쪽': (0, search_distance),
        '남쪽': (-search_distance, 0),
        '서쪽': (0, -search_distance)
    }

    # dNorth, dEast 반환
    (dN, dE) = direction_vectors[best_direction]

    return best_direction, best_signal, (dN, dE)


# ========================================
# 드론 연결
# ========================================

def connect_vehicle():
    """픽스호크에 연결"""
    print("픽스호크 연결 중...")
    print("포트: /dev/ttyAMA0, Baud: 57600")

    vehicle = connect('/dev/ttyAMA0', wait_ready=True, baud=57600)

    print("\n=== 연결 완료 ===")
    print(f"Firmware Version: {vehicle.version}")
    print(f"GPS: {vehicle.gps_0.fix_type} (위성 {vehicle.gps_0.satellites_visible}개)")
    print(f"Battery: {vehicle.battery.level}%")
    print(f"Current Mode: {vehicle.mode.name}")
    print("==================\n")

    return vehicle


def land(vehicle):
    """착륙"""
    print("\n=== 착륙 시작 ===")
    vehicle.mode = VehicleMode("LAND")

    land_timeout = 60  # 60초
    land_start_time = time.time()
    while vehicle.armed:
        if time.time() - land_start_time > land_timeout:
            print("[경고] 착륙 타임아웃! (드론이 여전히 armed 상태)")
            break
        alt = vehicle.location.global_relative_frame.alt
        if alt is not None:
            print(f"착륙 중... 고도: {alt:.1f}m")
        time.sleep(1)

    print("✓ 착륙 완료!\n")


# ========================================
# 메인 실행
# ========================================

def main():
    """드론 탐색 메인 함수"""
    vehicle = None

    try:
        # 1. 드론 연결
        vehicle = connect_vehicle()
        time.sleep(2)

        print("=" * 60)
        print("드론이 이륙한 상태인지 확인하세요!")
        print("수동으로 이륙 후 'GUIDED' 모드로 변경하고")
        print("Enter를 눌러 탐색을 시작하세요.")
        print(f"목표 SSID: '{TARGET_SSID}'")
        print("=" * 60)
        input()

        # 가이드 모드 확인
        if vehicle.mode.name != "GUIDED":
            print("모드를 GUIDED로 변경합니다...")
            vehicle.mode = VehicleMode("GUIDED")
            time.sleep(2)
            if vehicle.mode.name != "GUIDED":
                print("GUIDED 모드 변경 실패! 착륙합니다.")
                raise Exception("GUIDED 모드 설정 실패")

        # 2. 신호 탐색 반복
        for round_num in range(1, MAX_ROUNDS + 1):
            print("\n" + "#" * 60)
            print(f"###  탐색 라운드 {round_num}/{MAX_ROUNDS}  ###")
            print("#" * 60)

            # 4방향 탐색
            best_dir, best_sig, (dN, dE) = search_four_directions(
                vehicle,
                target_ssid=TARGET_SSID,
                iface=MONITOR_INTERFACE,
                search_distance=SEARCH_DISTANCE
            )

            # 신호가 매우 강하면 종료
            if best_sig > RSSI_THRESHOLD_STOP:
                print(f"\n신호 강도 충분: {best_sig:.2f} dBm")
                print("목표 지점 근처 도달! 탐색을 종료합니다.")
                break

            # 최적 방향으로 이동 (N배 거리) - NED 좌표 사용
            move_distance = math.sqrt(dN**2 + dE**2) * MOVE_MULTIPLIER

            print(f"\n{best_dir}으로 {move_distance:.1f}m 이동 중...")

            # 현재 NED 좌표 (이동의 기준점이 됨)
            current_north = vehicle.location.local_frame.north
            current_east = vehicle.location.local_frame.east
            current_down = vehicle.location.local_frame.down

            if current_north is None: # 안전장치
                 print("이동 전 로컬 좌표 손실! 라운드 중단")
                 continue

            # 목표 NED 좌표 계산 (현재 위치 + 이동 벡터 * 배수)
            target_north = current_north + dN * MOVE_MULTIPLIER
            target_east = current_east + dE * MOVE_MULTIPLIER

            goto_position_ned(vehicle, target_north, target_east, current_down, timeout=60)
            print(f"✓ {best_dir} 방향 이동 완료\n")

            # 다음 라운드 전 대기
            if round_num < MAX_ROUNDS:
                wait_time = 3
                print(f"다음 라운드까지 {wait_time}초 대기...\n")
                time.sleep(wait_time)

            if round_num == MAX_ROUNDS:
                print("최대 탐색 라운드 도달. 미션을 종료합니다.")


        print("\n" + "=" * 60)
        print("모든 탐색 완료!")
        print("=" * 60)

        # 자동 착륙
        land(vehicle)

        print("\n✓✓✓ 미션 완료! ✓✓✓\n")

    except KeyboardInterrupt:
        print("\n\n⚠️ 사용자 중단!")
        if vehicle and vehicle.armed:
            print("긴급 착륙!")
            vehicle.mode = VehicleMode("LAND")
            time.sleep(10)

    except Exception as e:
        print(f"\n❌ 오류 발생: {e}")
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
