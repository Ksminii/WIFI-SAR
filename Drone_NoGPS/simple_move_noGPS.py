    #!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
========================================
드론 4방향 이동 테스트 (속도 기반, 신호 측정 없음)
========================================

[실행 환경]
- 하드웨어: 라즈베리파이 + 픽스호크
- 연결: /dev/ttyAMA0 (시리얼 연결)
- Python 3.12 호환

[특징]
- GPS / Compass / GUIDED 모드 필요 없음
- 현재 모드(예: ALT HOLD) 그대로 사용
- 속도 기반 BODY_NED 프레임으로 앞/뒤/좌/우 이동
- 4방향 이동을 여러 라운드 반복

[사용 방법]
1. 조종기로 드론 수동 이륙 및 고도 유지 (ALT HOLD 추천)
2. 드론이 공중에 안정적으로 떠 있는지 확인
3. python3 simple_move.py 실행
4. Enter를 누르면 4방향 속도 이동 시작

[설정 변수]
- MOVE_SPEED    : 속도 (m/s)
- MOVE_DURATION : 한 방향으로 밀어주는 시간 (초) → 대략 이동 거리 결정
- MAX_ROUNDS    : 4방향 세트를 몇 번 반복할지
========================================
"""

# Python 3.12 호환성 패치
import collections
import collections.abc
collections.MutableMapping = collections.abc.MutableMapping

import time
from dronekit import connect, VehicleMode
from pymavlink import mavutil

# 이동 관련 설정
MOVE_SPEED = 1.0       # m/s
MOVE_DURATION = 10     # 초 (대략 10m 이동)
MAX_ROUNDS = 2         # 4방향 세트를 몇 번 반복할지


# ==========================================================
#  속도 기반 이동 함수 (GPS 필요 없음)
# ==========================================================

def send_velocity_body(vehicle, vx, vy, vz, duration):
    """
    BODY_NED 기준 속도 명령을 duration 초 동안 반복 전송
    - vx: 앞(+)/뒤(-)  [m/s]
    - vy: 오른쪽(+)/왼쪽(-) [m/s]
    - vz: 위(-)/아래(+) [m/s]  (NED 기준이라 아래가 +)
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
    for _ in range(int(duration * 10)):   # 10Hz로 전송
        vehicle.send_mavlink(msg)
        vehicle.flush()
        time.sleep(0.1)

    # 멈추는 명령 한 번 더
    stop_msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,
        0, 0,
        mavutil.mavlink.MAV_FRAME_BODY_NED,
        0b0000111111000111,
        0, 0, 0,
        0, 0, 0,   # 속도 0
        0, 0, 0,
        0, 0
    )
    vehicle.send_mavlink(stop_msg)
    vehicle.flush()


# ==========================================================
#  4방향 이동 함수
# ==========================================================

def move_four_directions(vehicle):
    print("\n======================")
    print(f" 4방향 속도 이동 시작 (속도={MOVE_SPEED} m/s, 시간={MOVE_DURATION}s)")
    print("======================\n")

    directions = [
        ("북쪽(앞)",  +MOVE_SPEED, 0),
        ("동쪽(우)",   0, +MOVE_SPEED),
        ("남쪽(뒤)",  -MOVE_SPEED, 0),
        ("서쪽(좌)",   0, -MOVE_SPEED)
    ]

    for name, vx, vy in directions:
        print(f"\n▶ {name}으로 이동 시작")
        send_velocity_body(vehicle, vx, vy, 0, MOVE_DURATION)
        print(f"✓ {name} 이동 완료")
        time.sleep(2)

    print("\n======================")
    print(" 원위치 복귀 시도 (정확한 GPS 기준 복귀는 아님, 단순 멈춤)")
    print("======================\n")

    send_velocity_body(vehicle, 0, 0, 0, 1)


# ==========================================================
#  드론 연결
# ==========================================================

def connect_vehicle():
    print("픽스호크 연결 중... (/dev/ttyAMA0, 57600)")
    vehicle = connect('/dev/ttyAMA0', wait_ready=True, baud=57600)
    #vehicle = connect('COM5', baud=115200, wait_ready=False)
    #컴퓨터 연결용
    print("연결 완료!")
    print(f"현재 모드: {vehicle.mode.name}")
    return vehicle


# ==========================================================
#  메인 함수
# ==========================================================

def main():
    vehicle = connect_vehicle()
    time.sleep(2)

    print("\n==============================")
    print(" 드론이 공중에 떠 있어야 합니다!")
    print(" ALT HOLD 같은 모드에서 고도 유지 중인지 확인하세요.")
    print(" Enter 누르면 4방향 이동 테스트를 시작합니다.")
    print("==============================\n")
    input()

    print(f"현재 모드: {vehicle.mode.name}")
    print("모드 변경 없이 속도 기반 이동을 시작합니다.\n")

    for i in range(MAX_ROUNDS):
        print(f"\n====== 라운드 {i+1}/{MAX_ROUNDS} ======")
        move_four_directions(vehicle)
        time.sleep(3)

    print("\n=== 이동 테스트 완료 ===")
    vehicle.close()


if __name__ == "__main__":
    main()
