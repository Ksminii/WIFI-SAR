## 드론 신호 탐색 시스템 (No GPS)

GPS/Compass 없이 속도 기반으로 Wi-Fi 신호원을 탐색하는 드론 제어 스크립트

## 파일 설명

```
simple_noGPS.py           # 5방향 탐색 (중앙, 북, 동, 남, 서)
simple_search_noGPS.py    # 4방향 탐색 (북, 동, 남, 서)
simple_move_noGPS.py      # 4방향 이동 테스트 (신호 측정 없음)
```

## 실행 환경

- 라즈베리파이 + 픽스호크
- 연결: `/dev/ttyAMA0` (57600 baud)
- Python 3.12
- Monitor mode 무선 인터페이스 (wlan0mon)

## 사용법

```bash
python3 simple_noGPS.py
# Enter 누르면 탐색 시작
```

## 설정 변수

```python
TARGET_SSID = "Victim_Phone_WiFi"   # 타겟 Wi-Fi SSID
MONITOR_INTERFACE = "wlan0mon"      

MOVE_SPEED = 1.0        # 이동 속도 (m/s)
MOVE_DURATION = 10      # 각 방향 이동 시간 (초)
MOVE_MULTIPLIER = 2.0   # 최적 방향 추가 이동 배수
MAX_ROUNDS = 5          # 탐색 라운드 수
```

## 동작 방식

1. 각 방향으로 이동 후 RSSI 측정 (tshark)
2. 가장 강한 신호 방향 선택
3. 해당 방향으로 추가 이동
4. 중앙이 최고 신호면 탐색 종료 (정점 도달)
