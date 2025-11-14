# 드론 긴급 구조 시스템 - 코드 사용 가이드
---


# 파라미터는 적당히 수정하세요

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


###  하드웨어 구성

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

#### simple_move.py - 이동 기능 테스트

```bash
# 신호 측정 없이 이동만 테스트
python3 simple_move.py

# 북 → 동 → 남 → 서 순서로 이동
# 각 방향 10m씩 이동 후 복귀
```

#### simple_search.py - 4방향 탐색

```bash
# 4방향 탐색 (각 방향마다 원위치 복귀)
python3 simple_search.py

# 5방향보다 이동 횟수 많음 (8회 vs 6회)
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



##  문제 해결

### Python 3.12 호환성


```python
# Python 3.12 호환성 패치
import collections
import collections.abc
collections.MutableMapping = collections.abc.MutableMapping
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



## 프로젝트 구조

```
Drone/
├── README.md                   
│
├── Python 버전 (권장)
│   ├── simple.py               # 5방향 탐색 (메인)
│   ├── simple_search.py        # 4방향 탐색
│   └── simple_move.py          # 이동 테스트
│
└── C++ 버전
    ├── simple.cpp              # 5방향 탐색 (메인)
    ├── simple_search.cpp       # 4방향 탐색
    └── simple_move.cpp         # 이동 테스트
```

---

##  주요 특징

### 1. 5방향 탐색 알고리즘 (simple.py/cpp)

- 중앙 지점 포함으로 정점 감지 가능
- 6회 이동으로 효율적 탐색
- 중앙이 최고 신호면 탐색 종료

### 2. NED 로컬 좌표계

- EKF 준비 대기 로직
- 안전한 위치 제어
- 정밀한 상대 위치 이동

### 3. 정밀 RSSI 측정

- tshark 기반 비컨 프레임 분석
- 3회 측정 후 평균
- Outlier 제거 (15개 이상 샘플 시)

### 4. 적응형 종료 조건

- 신호 강도 임계값
- 중앙 정점 감지
- 최대 라운드 제한

---

##  실제 사용 시나리오

```
1. 라즈베리파이 부팅 및 SSH 접속
   ssh pi@raspberrypi.local

2. 코드 디렉토리 이동
   cd ~/drone_project/Drone

3. 설정 확인 (SSID, 거리 등)
   nano simple.py

4. 드론 수동 이륙 (조종기)
   - 10m 정도 이륙
   - GUIDED 모드로 변경

5. Python 스크립트 실행
   source ~/drone_venv/bin/activate
   python3 simple.py

6. 자동 탐색 시작
   - 5방향 신호 측정
   - 최고 신호 방향으로 이동
   - 반복

7. 종료 조건 만족 시 자동 착륙
   - 신호 강도 -20dBm 이상
   - 중앙이 최고 신호
   - 최대 5라운드 완료
```

---

## 주의사항

1. **실제 비행 전 반드시 테스트**
   - simple_move.py로 이동 기능 확인
   - 안전한 환경에서 알고리즘 검증

2. **안전 거리 유지**
   - SEARCH_DISTANCE 초기값 작게 시작
   - 장애물 확인

3. **배터리 모니터링**
   - 충분한 배터리 용량 확인
   - 귀환 예비 배터리 고려

4. **Wi-Fi 모니터 모드**
   - 모니터 모드 설정 필수
   - 테스트로 RSSI 측정 확인

5. **픽스호크 펌웨어**
   - ArduCopter 또는 PX4 설치
   - MAVLink 통신 활성화

---


