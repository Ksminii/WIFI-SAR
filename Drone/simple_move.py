#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
========================================
드론 4방향 이동 테스트 (신호 측정 없음)
========================================

[실행 환경]
- 하드웨어: 라즈베리파이 + 픽스호크
- 연결: /dev/ttyAMA0 (시리얼 연결)
- Python 3.12 호환

[주요 기능]
1. 신호 측정 없이 4방향 이동만 테스트
2. 북 → 동 → 남 → 서 순서로 이동
3. 각 방향 이동 후 원위치 복귀
4. NED 로컬 좌표계 사용

[사용 목적]
- 드론 이동 기능만 검증
- 신호 측정 없이 간단한 테스트
- 개발/디버깅 단계에서 사용

[사용 방법]
1. 드론 수동 이륙 (조종기 사용)
2. GUIDED 모드로 변경
3. python3 simple_move.py 실행
4. 자동 이동 시작

[설정 변수]
- MOVE_DISTANCE: 각 방향 이동 거리 (기본 10m)
- MAX_ROUNDS: 최대 이동 라운드 (기본 3회)

[참고]
- 실제 탐색은 simple.py 또는 simple_search.py 사용
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


# ========================================
# 스크립트 설정
# ========================================
# 이동 관련 설정
MOVE_DISTANCE = 10.0  # 각 방향으로 이동할 거리 (미터)
MAX_ROUNDS = 3        # 최대 이동 라운드


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


def move_four_directions(vehicle, move_distance=10):
    """
    4방향(북, 동, 남, 서)으로 순서대로 이동
    신호 측정 없이 이동만 수행
    """
    print("\n" + "="*60)
    print(f"4방향 이동 시작 (이동 거리: {move_distance}m)")
    print("="*60 + "\n")

    # EKF(로컬 좌표)가 준비될 때까지 안전 대기
    print("로컬 좌표계(EKF) 준비 대기 중...")
    while vehicle.location.local_frame.north is None:
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

    # 4방향 정의: (이름, dNorth, dEast)
    directions = [
        ("북쪽", move_distance, 0),
        ("동쪽", 0, move_distance),
        ("남쪽", -move_distance, 0),
        ("서쪽", 0, -move_distance)
    ]

    # 각 방향으로 이동
    for dir_name, dN, dE in directions:
        print(f"\n{'─'*50}")
        print(f"[{dir_name}] 이동 시작")
        print(f"{'─'*50}")

        # 현재 위치에서 상대적으로 이동
        current_north = vehicle.location.local_frame.north
        current_east = vehicle.location.local_frame.east
        current_down = vehicle.location.local_frame.down

        target_north = current_north + dN
        target_east = current_east + dE

        print(f"목표: North={target_north:.2f}m, East={target_east:.2f}m")

        success = goto_position_ned(vehicle, target_north, target_east, current_down, timeout=60)

        if success:
            print(f"✓ {dir_name} 이동 완료")
        else:
            print(f"⚠️ {dir_name} 이동 실패 (타임아웃)")

        # 다음 이동 전 잠시 대기
        time.sleep(2)

    # 원위치 복귀
    print(f"\n{'='*60}")
    print("원위치로 복귀")
    print(f"{'='*60}")
    goto_position_ned(vehicle, home_north, home_east, home_down, timeout=60)
    print("✓ 복귀 완료\n")


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

    while vehicle.armed:
        alt = vehicle.location.global_relative_frame.alt
        if alt is not None:
            print(f"착륙 중... 고도: {alt:.1f}m")
        time.sleep(1)

    print("✓ 착륙 완료!\n")


# ========================================
# 메인 실행
# ========================================

def main():
    """드론 이동 테스트 메인 함수"""
    vehicle = None

    try:
        # 1. 드론 연결
        vehicle = connect_vehicle()
        time.sleep(2)

        print("=" * 60)
        print("드론이 이륙한 상태인지 확인하세요!")
        print("수동으로 이륙 후 'GUIDED' 모드로 변경하고")
        print("Enter를 눌러 이동 테스트를 시작하세요.")
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

        # 2. 4방향 이동 반복
        for round_num in range(1, MAX_ROUNDS + 1):
            print("\n" + "#" * 60)
            print(f"###  이동 라운드 {round_num}/{MAX_ROUNDS}  ###")
            print("#" * 60)

            move_four_directions(vehicle, move_distance=MOVE_DISTANCE)

            # 다음 라운드 전 대기
            if round_num < MAX_ROUNDS:
                wait_time = 5
                print(f"다음 라운드까지 {wait_time}초 대기...\n")
                time.sleep(wait_time)

        print("\n" + "=" * 60)
        print("모든 이동 테스트 완료!")
        print("=" * 60)

        # 자동 착륙
        land(vehicle)

        print("\n✓✓✓ 테스트 완료! ✓✓✓\n")

    except KeyboardInterrupt:
        print("\n\n⚠️ 사용자 중단!")
        if vehicle and vehicle.armed:
            print("긴급 착륙!")
            vehicle.mode = VehicleMode("LAND")
            time.sleep(10)

    except Exception as e:
        print(f"\n 오류 발생: {e}")
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
