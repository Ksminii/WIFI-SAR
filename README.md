# WIFI-SAR

드론 기반 WiFi 신호 추적을 통한 실종자 탐색 시스템

##  프로젝트 개요

본 프로젝트는 실종자의 스마트폰 WiFi 신호(핫스팟 신호)를 기반으로 조난자의 위치를 신속하게 탐지하는 드론 기술 개발을 목표로 한다.
드론에 탑재된 네트워크 탐지 모듈이 실시간으로 실종자 스마트폰의 신호 세기(RSSI)를 측정하고, 수집된 데이터를 바탕으로 탐색 경로 생성 알고리즘이 최적의 비행 경로(Waypoint)를 동적으로 생성하면 최종적으로 드론은 생성된 경로에 따라 실종자의 위치를 특정할 수 있다.

## Repository 구조

```
WIFI-SAR/
│
├── README.md                               # 프로젝트 전체 개요
│
├── Algorithm/                              # 탐색 알고리즘 구현
│   ├── README.md                           알고리즘 상세 설명
│   ├── algorithm_ver1.py                  논문 기반 알고리즘
│   ├── algorithm_ver2_simple.py           상하좌우 단순 이동
│   ├── algorithm_ver3_upgrade.py          상하좌우 업그레이드 알고리즘
│   ├── algorithm_kalman_v1.0.py           Kalman Filter 적용
│   ├── algorithm_ewma_v1.1.py             EWMA 평활화
│   └── algorithm_ewma_fading_v1.2.py      EWMA + 페이딩 모델
│
├── Simulator/                              # 시뮬레이션 환경
│   ├── README.md                           시뮬레이터 상세 가이드
│   ├── simulator_base_v2.0.py             기본 시뮬레이터
│   ├── simulator_fading_v2.1.py           페이딩 모델 시뮬레이터
│   └── final_simulator.py                 최종 시뮬레이터
│
├── Drone/                                  # 드론 제어 (GPS 기반)
│   ├── README.md                           하드웨어 설정 가이드
│   ├── cpp/                               C++ 구현
│   │   ├── simple.cpp                     기본 드론 제어
│   │   ├── simple_move.cpp                이동 제어
│   │   ├── simple_search.cpp              탐색 제어
│   │   └── CMakeLists.txt                 C++ 빌드 설정
│   └── python/                            Python 구현
│       ├── simple.py                      기본 드론 제어
│       ├── simple_move.py                 이동 제어
│       ├── simple_search.py               탐색 제어
│       └── rssi_monitor.py                RSSI 모니터링
│
├── Drone_NoGPS/                            # 드론 제어 (GPS 미사용)
│   ├── README.md                          상세 설명
│   ├── simple_noGPS.py                    기본 제어 (No GPS)
│   ├── simple_move_noGPS.py               이동 제어 (No GPS)
│   └── simple_search_noGPS.py             탐색 제어 (No GPS)
│
├── GPS_Simulator/                          # 도보 시뮬레이터
│   ├── README.md                          시뮬레이터 사용 가이드
│   └── index.html                         4방향 전수 조사
│
├── object_detection/                       # 객체 탐지 (YOLOv8)
│   ├── README.txt                         상세 설명
│   ├── detection.py                       객체 탐지 스크립트
│   └── small_model/                       경량화 모델
│
└── RSSI/                                   # RSSI 신호 처리
    ├── README.md                          상세 설명
    ├── rssi_watch.py                      기본 RSSI 모니터링
    ├── rssi_watch_EMA.py                  EMA 필터 적용
    └── rssi_watch_Kalman.py               Kalman 필터 적용
```

