# GPS Simulator (도보 시뮬레이터)

스마트폰 GPS와 RSSI 입력을 기반으로 하는 드론 탐색 알고리즘을 실제 환경에서 도보로 검증하기 위한 시뮬레이터
사용자가 직접 걸어다니며 RSSI 값을 입력하면, 알고리즘이 다음 이동 지점(Waypoint)을 생성해준다. 

## 실행 방법

```bash
# 로컬 서버 실행 (Python)
python -m http.server 8080

# 또는 Node.js
npx serve -p 8080
```

브라우저에서 `http://localhost:8080/index.html` 접속

**중요**: HTTPS 또는 localhost에서만 GPS/나침반 기능이 작동합니다.

## 기능

### 지도 및 UI
- 실시간 GPS 추적: 현재 위치 및 이동 경로 표시
- 나침반: 목표 지점 방향 표시
- 경로 기록: 이동 궤적 시각화

### 탐색 알고리즘 (4방향 Wide Scan)

| 상태 | 설명 |
|------|------|
| IDLE | 대기 상태 |
| INIT | 초기화, 4방향 스캔 준비 |
| SCANNING | 상/하/좌/우 4방향 순차 탐색 |
| MOVING | 최적 방향으로 직진 |
| BACKTRACK | 신호 약화 시 복귀 |
| FINISHED | 목표 RSSI 도달 |

### 파라미터

```javascript
ARRIVAL_DIST: 2.5m    // 도착 판정 거리
TARGET_RSSI: -55dBm   // 목표 신호 세기
DEAD_ZONE: -80dBm     // 신호 없음 판정

// 직진 보폭 (MOVING)
STEP: { FAR: 30m, MID: 15m, NEAR: 5m }

// 4방향 스캔 반경 (SCANNING)
SCAN: { FAR: 50m, MID: 25m, NEAR: 12m }
```

## 사용 방법

1. GPS 수신 확인 후 테스트 시작 버튼 클릭
2. 현재 위치에서 측정한 RSSI 값 입력
3. 빨간 점(목표 지점)으로 이동
4. 도착 후 다시 RSSI 입력
5. `특정 dBm` 이상 도달 시 탐색 완료 


## 요구사항

- GPS 지원 브라우저 (Chrome/Safari 권장)
- 위치 권한 허용
- iOS: 나침반 권한 별도 승인 필요
