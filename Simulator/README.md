# Simulator 

## 디렉토리 구조

```
Simulator/
├── simulator_base_v2.0.py          # 기본 시뮬레이터
├── simulator_fading_v2.1.py        # 페이딩 모델 추가
├── tempsim1.py                     # 개발
└── final_simulator.py              # 최종 시뮬레이터

```

## 버전별 변경사항

### v2.0 (simulator_base_v2.0.py)
- 3가지 알고리즘 비교 (Greedy / Kalman / Hybrid)
- 기본 신호 모델 (Path Loss + Gaussian Noise)
- 시각화 / 성능 테스트 모드

```bash
python simulator_base_v2.0.py <알고리즘> <모드>
```

### v2.1 (simulator_fading_v2.1.py)
v2.0 대비 추가:
- Rician/Rayleigh Fading 모델
- GPS Drift 오차 시뮬레이션
- EWMA + Momentum 알고리즘
- Shadow Fading, Sensor Delay

```bash
python simulator_fading_v2.1.py
```

### final
- 시각화 업그레이드
- GPS_ERROR_STD: 8.0m → 3.0m (드론용 RTK GPS 정확도)
- DIST_PINPOINT: 1.2m → 0.8m (목표 근접 거리 감소)
