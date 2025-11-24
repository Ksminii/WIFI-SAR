
##  파일 구조

#### Python 버전 
```
simple.py               # 5방향 탐색 (중앙, 북, 동, 남, 서) 
simple_search.py        # 4방향 탐색 (북, 동, 남, 서)
simple_move.py          # 4방향 이동 테스트 (신호 측정 없음)
```

#### C++ 버전
```
simple.cpp              # 5방향 탐색 
simple_search.cpp       # 4방향 탐색
simple_move.cpp         # 4방향 이동 테스트  (신호 측정 x)
```


---


###  시스템 구성

```
실종자 휴대폰 (Wi-Fi 신호)
         ↓
    ┌─────────────────────────────────┐
    │         드론 시스템                |
    │                                 │
    │  ┌──────────────────────────┐   │
    │  │  라즈베리파이               │    │
    │  │  - Python/C++ 코드 실행    │   │
    │  │  - Wi-Fi RSSI 측정        │   │
    │  │  - 알고리즘 실행            │   │
    │  └──────────┬───────────────┘   │
    │             │ 시리얼(/dev/ttyAMA0)│
    │             ↓                    │
    │  ┌──────────────────────────┐   │
    │  │  픽스호크 (비행 제어)         │   │
    │  │  - MAVLink 명령 수신        │   │
    │  │  - 모터/GPS/센서 제어        │   │
    │  └──────────────────────────┘   │
    └─────────────────────────────────┘
```

### 2️ 라즈베리파이 환경 설정

```bash
# 1. Python 환경 설정
sudo apt-get update
sudo apt-get install python3 python3-pip python3-venv

# 2. 가상환경 생성
python3 -m venv ~/drone_venv
source ~/drone_venv/bin/activate

# 3. 필요 패키지 설치
pip install dronekit pymavlink

# 4. Wi-Fi 도구 설치 (RSSI 측정용)
sudo apt-get install tshark wireless-tools
```

### 3️ 모니터 모드 설정

```bash
# Wi-Fi 인터페이스를 모니터 모드로 설정
sudo ip link set wlan0 down
sudo iw dev wlan0 set monitor none
sudo ip link set wlan0 up
sudo ip link set wlan0 name wlan0mon
```

---

##  사용 방법

### Python 버전 
#### simple.py - 5방향 탐색 (메인 사용)

```bash
# 1. 코드 설정 수정 (파일 상단)
TARGET_SSID = "실종자_휴대폰_SSID"  # 탐색할 Wi-Fi SSID
MONITOR_INTERFACE = "wlan0mon"     # Wi-Fi 모니터 인터페이스
SEARCH_DISTANCE = 10.0             # 탐색 거리 (m)
MAX_ROUNDS = 5                     # 최대 라운드 수

# 2. 드론 수동 이륙 (조종기 사용)

# 3. GUIDED 모드로 변경 (조종기)

# 4. 스크립트 실행
source ~/drone_venv/bin/activate
python3 simple.py

# 5. 자동 탐색 시작
# - 5방향 신호 측정
# - 최고 신호 방향으로 이동
# - 반복 탐색
# - 중앙이 최고 신호면 종료
```

---

### C++ 버전

#### 컴파일

```bash
# MAVSDK 설치 (라즈베리파이에서)
sudo apt-get install cmake git
git clone https://github.com/mavlink/MAVSDK.git
cd MAVSDK
git submodule update --init --recursive
cmake -Bbuild -DCMAKE_BUILD_TYPE=Release
cmake --build build -j4
sudo cmake --build build --target install

# 코드 컴파일
g++ -std=c++17 simple.cpp -o simple -lmavsdk
g++ -std=c++17 simple_search.cpp -o simple_search -lmavsdk
g++ -std=c++17 simple_move.cpp -o simple_move -lmavsdk
```

#### 실행

```bash
# 실제 드론
./simple serial:///dev/ttyAMA0:57600
```

---

## 설정 변수 상세

### 공통 설정

| 변수 | 기본값 | 설명 |
|------|--------|------|
| `TARGET_SSID` | "Victim_Phone_WiFi" | 탐색할 Wi-Fi SSID |
| `MONITOR_INTERFACE` | "wlan0mon" | Wi-Fi 모니터 인터페이스 |
| `SEARCH_DISTANCE` | 10.0 | 탐색 시 이동 거리 (m) |
| `MOVE_MULTIPLIER` | 2.0 | 최적 방향 이동 배수 |
| `MAX_ROUNDS` | 5 | 최대 탐색 라운드 |
| `RSSI_THRESHOLD_STOP` | -20 | 탐색 중지 신호 강도 (dBm) |
| `TSHARK_DURATION` | 3 | tshark 측정 시간 (초) |

### Python 연결 설정

```python
# 실제 드론 (라즈베리파이 + 픽스호크)
vehicle = connect('/dev/ttyAMA0', wait_ready=True, baud=57600)
```

### C++ 연결 설정

```cpp
// 실제 드론
./simple serial:///dev/ttyAMA0:57600
```


### Wi-Fi 모니터 모드 확인

```bash
# 인터페이스 상태 확인
iwconfig

# wlan0mon이 보이면 정상
```

### 시리얼 포트 권한

```bash
# 권한 부여
sudo chmod 666 /dev/ttyAMA0

# 또는 사용자를 dialout 그룹에 추가
sudo usermod -a -G dialout $USER
# 재로그인 필요
```

### DroneKit 연결 실패

```bash
# 1. 픽스호크 전원 확인
# 2. 시리얼 연결 확인
# 3. Baud rate 확인 (57600)

# 연결 테스트
python3 -c "from dronekit import connect; vehicle = connect('/dev/ttyAMA0', wait_ready=True, baud=57600); print('Success')"
```

---






