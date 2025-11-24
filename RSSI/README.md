## RSSI 측정 도구

RSSI 값을 실시간으로 측정하는 Python 스크립트 모음

## 파일 설명

```
rssi_watch.py           # 기본 RSSI 측정 (Trimmed Mean)
rssi_watch_EMA.py       # EMA(지수이동평균) 필터 적용 버전
rssi_watch_Kalman.py    # 칼만 필터 적용 버전
```

## 사용법

```bash
sudo python3 rssi_watch.py --iface wlan0mon --ssid "YOUR_SSID" --window 10
```

### 옵션

| 옵션 | 설명 | 기본값 |
|------|------|--------|
| `--iface` | Monitor mode 인터페이스 | (필수) |
| `--ssid` | 측정 대상 SSID | (필수) |
| `--window` | 측정 시간 (초) | 10 |

## 동작 방식

1. 채널 스윕: 1, 6, 11 채널 우선 탐색 후 나머지 채널 순차 탐색
2. 대기: Enter 키 입력 대기
3. 측정: 지정된 시간 동안 Beacon 프레임에서 RSSI 수집
4. 결과 출력: Trimmed Mean (상하위 10% 제거) 평균값 출력

## 필터 비교

| 필터 | 특징 |
|------|------|
| Trimmed Mean | 이상치 제거, 단일 측정에 적합 |
| EMA (α=0.3) | 연속 측정 시 변화 추적에 적합 |
| Kalman | 노이즈 제거, 안정적인 추정값 제공 |

## 측정 예시 (EMA 필터)

```
[측정 결과]
  원본 평균: -58.4 dBm
  EMA 필터 적용: -64.1 dBm

[측정 결과]
  원본 평균: -51.9 dBm
  EMA 필터 적용: -60.4 dBm

[측정 결과]
  원본 평균: -65.4 dBm
  EMA 필터 적용: -61.3 dBm

[측정 결과]
  원본 평균: -58.0 dBm
  EMA 필터 적용: -62.0 dBm

```

