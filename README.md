# WIFI-SAR: WiFi-Based Search and Rescue Drone System

드론 기반 WiFi 신호 추적을 통한 실종자 탐색 시스템

##  프로젝트 개요

◯ 본 프로젝트는 실종자의 스마트폰 WiFi 신호(핫스팟 신호)를 기반으로 조난자의 위치를 신속하게 탐지하는 드론 기술 개발을 목표로 함.
◯ 드론에 탑재된 네트워크 탐지 모듈이 실시간으로 실종자 스마트폰의 신호 세기(RSSI)를 측정하고, 수집된 데이터를 바탕으로 탐색 경로 생성 알고리즘이 최적의 비행 경로(Waypoint)를 동적으로 생성함.
◯ 최종적으로 드론은 생성된 경로에 따라 실종자의 위치를 특정할 수 있음.

## Repository 구조

```
WIFI-SAR/
│
├── README.md                               # 프로젝트 전체 개요 
│
├── Simulator/                              # 시뮬레이션 환경
│   ├── README.md                           시뮬레이터 상세 가이드
│   ├── simulator_base_v2.0.py             기본 시뮬레이터
│   ├── simulator_fading_v2.1.py           페이딩 모델 시뮬레이터
│   └── tempsim1.py                        실험용
│
├── Algorithm/                              # 알고리즘 구현
│   ├── README.md                           알고리즘 상세 설명
│   ├── algorithm_kalman_v1.0.py           Kalman Filter
│   ├── algorithm_ewma_v1.1.py             EWMA 평활화
│   └── algorithm_ewma_fading_v1.2.py      EWMA + Fading (최신)
│
└── Drone/                                  # 하드웨어 구현
    ├── README.md                           하드웨어 설정 가이드
    ├── simple.py                          5방향 탐색 (Python)
    ├── simple_search.py                   4방향 탐색 (Python)
    ├── rssi_monitor.py                    RSSI 모니터링
    ├── simple.cpp                         C++ 구현
    └── CMakeLists.txt                     빌드 설정
```

