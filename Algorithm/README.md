# Algorithm - WiFi-SAR 탐색 알고리즘

WiFi 신호(Hotspot) 기반 드론 탐색 알고리즘 구현

## 디렉토리 구조

```
Algorithm/
├── algorithm_ewma_v1.1.py           # EWMA 기반 알고리즘
├── algorithm_ewma_fading_v1.2.py    # EWMA + Momentum 알고리즘
└── algorithm_kalman_v1.0.py         # Kalman Filter 알고리즘
```

## 알고리즘 설명

### algorithm_ewma_v1.1.py
지수이동평균(EWMA)을 이용한 RSSI 신호 평활화 기반 알고리즘
- 나선형 상승 탐색 (신호 약할 때)
- 적응형 상승 (신호 강도 기반)
- Stuck detection 갇힘 탈출

### algorithm_ewma_fading_v1.2.py
EWMA + Momentum 메커니즘
- v1.1의 모든 기능 포함
- 연속 성공 시 탐색 속도 동적 증가
- Rician/Rayleigh Fading 지원

### algorithm_kalman_v1.0.py
Kalman Filter 기반 최적 상태 추정 알고리즘
- RSSI 예측-보정 사이클로 노이즈 감소
- Rician Fading 모델 (K-factor 조정 가능)
- 노이즈가 많은 환경에서 가장 우수한 성능

## 실행 방법

```bash
python algorithm_ewma_v1.1.py
python algorithm_ewma_fading_v1.2.py
python algorithm_kalman_v1.0.py
```
