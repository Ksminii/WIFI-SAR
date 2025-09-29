"""
핫스팟 신호원을 RSSI 신호로 찾는 시뮬레이터 코드

주요 구성 요소는 다음과 같음

데이터 클래스
시뮬레이션에 사용되는 파라미터와 결과값 등 변수 보관하는 곳

SimulationEnvironment
가상 2D 환경을 생성 및 특정 위치에서의 RSSI 신호를 모델링, 추후 수정해야할 부분

HomingAlgorithm(가장 중요함)
RSSI 신호 변화에 따라 다음 행동(웨이포인트)을 결정하는 알고리즘으로 구현
IDLE, ADAPTIVE_ASCENT, ESCAPING 세 가지 상태를 가지는데, 실제 알고리즘에서는 이 부분은 필요없을듯

SimulationVisualizer
 matplotlib을 사용하여 시뮬레이션 과정을 시각화

SimulationRunner
단일 또는 다중 시뮬레이션을 실행하고, 그 결과를 통계적으로 분석하려고 만듦

  [1] 단일 시뮬레이션 (시각화)
  [2] 다중 시뮬레이션 (1000회 실행 및 통계 분석)
  두 가지 모드 지원함
"""
import numpy as np
import math
import time
import matplotlib.pyplot as plt
from dataclasses import dataclass, field

# --------------------------------------------------------------------------
# 1. 상수 및 데이터 클래스 정의
# --------------------------------------------------------------------------
@dataclass
class Constants:
    """
    시뮬레이션 전체에서 변하지 않는 물리적 또는 환경적 상수를 정의
    """
    INITIAL_DISTANCE: float = 90.0          # 드론의 시작점과 핫스팟 사이의 초기 거리 (m)
    MIN_SIGNAL_STRENGTH: float = -100.0     # RSSI 신호의 최솟값 (dBm)
    MIN_DISTANCE_TO_HOTSPOT: float = 1.0    # 신호 계산 시 핫스팟과의 최소 거리 (0으로 나누기 방지)
    ROTATION_ANGLE_RAD: float = np.pi / 4   # 방향 전환 시 회전 각도 (라디안)

@dataclass
class SimParams:
    """
    시뮬레이션 및 알고리즘의 동작을 제어하는 파라미터를 관리
    """
    # 튜닝된 파라미터: 이 값들은 시뮬레이션 성능에 직접적인 영향을 미침
    DIST_FAR: float = 18.35                 # 신호가 매우 약할 때의 전진 거리
    DIST_MID: float = 6.20                  # 신호가 중간일 때의 전진 거리
    DIST_NEAR: float = 3.15                 # 신호가 강할 때의 전진 거리
    DIST_PINPOINT: float = 1.25             # 신호가 매우 강할 때의 전진 거리
    STUCK_THRESHOLD: int = 3                # 'Stuck' 상태로 판단하기까지 신호 개선이 없는 횟수
    ESCAPE_DISTANCE: float = 25.5           # 'ESCAPING' 상태에서 탈출을 위해 이동하는 거리
    SIGNAL_MID: float = -66.5               # '중간 신호'로 판단하는 RSSI 임계값
    SIGNAL_NEAR: float = -59.8              # '강한 신호'로 판단하는 RSSI 임계값
    SIGNAL_PINPOINT: float = -54.5          # '매우 강한 신호'로 판단하는 RSSI 임계값
    
    # 고정 파라미터: 시뮬레이션의 기본 환경을 정의
    FAILURE_THRESHOLD: float = -90.0        # 이 신호보다 약하면 시작 실패로 간주
    ASCENT_THRESHOLD: float = -80.0         # 이 신호보다 강해지면 본격적인 탐색(ASCENT) 시작
    TARGET_THRESHOLD: float = -50.0         # 이 신호보다 강해지면 목표 지점 도착으로 간주
    DRONE_SPEED: float = 8.0               # 드론의 이동 속도 (m/s)
    RSSI_SCAN_TIME: float = 2.0             # RSSI 스캔에 소요되는 시간 (s)
    TIME_LIMIT: float = 300.0               # 시뮬레이션 최대 제한 시간 (s)
    GPS_ERROR_STD: float = 1.5              # GPS 오차의 표준편차 (m)
    RSSI_SHADOW_STD: float = 2.0            # RSSI 섀도잉(장주기 페이딩) 오차의 표준편차 (dBm)

@dataclass
class SimResult:
    """단일 시뮬레이션의 결과를 저장하는 구조체 역할을 합니다."""
    success: bool
    final_distance: float
    total_travel: float
    search_time: float
    reason: str

# --------------------------------------------------------------------------
# 2. Simulation Environment Class
# --------------------------------------------------------------------------
class SimulationEnvironment:
    """
    가상 Wi-Fi 핫스팟 환경을 관리
    핫스팟의 위치를 생성하고, 특정 좌표의 RSSI 신호 강도를 계산하는 역할을 담당
    """
    def __init__(self, params: SimParams):
        if not isinstance(params, SimParams):
            raise TypeError("params는 SimParams의 인스턴스여야 합니다.")
        self.params = params
        
        # 원점(0,0)에서 INITIAL_DISTANCE 만큼 떨어진 곳에 핫스팟 무작위 생성
        angle = np.random.uniform(0, 2 * np.pi)
        self.hotspot_pos = np.array([
            Constants.INITIAL_DISTANCE * np.cos(angle),
            Constants.INITIAL_DISTANCE * np.sin(angle)
        ])

    def get_signal(self, pos: np.ndarray, add_noise: bool = True) -> float:
        """
        주어진 위치(pos)에서 핫스팟까지의 거리를 기반으로 RSSI 신호 강도를 계산
        현실성을 위해 경로 손실 모델과 무작위 노이즈를 적용

        Args:
            pos (np.ndarray): 신호를 측정할 2D 위치 좌표
            add_noise (bool): 신호에 가우시안 노이즈를 추가할지 여부

        Returns:
            float: 계산된 RSSI 신호 강도 (dBm)
        """
        if not isinstance(pos, np.ndarray) or pos.shape != (2,):
            raise ValueError("위치는 2D numpy 배열이어야 합니다.")

        distance = np.linalg.norm(pos - self.hotspot_pos)
        distance = max(distance, Constants.MIN_DISTANCE_TO_HOTSPOT)
        
        # Log-distance path loss 모델: 신호는 거리에 로그 스케일로 반비례하여 감소
        signal = -30 - 10 * 2 * np.log10(distance)

        if add_noise:
            # 현실적인 신호 변동성을 위해 두 종류의 노이즈 추가
            # 노이즈를 더 추가해야할 수도 있음
            shadow_noise = np.random.normal(0, self.params.RSSI_SHADOW_STD) # 큰 장애물에 의한 신호 감쇠
            small_noise = np.random.randn() * 0.5                          # 다중 경로 페이딩 등 작은 규모의 신호 변동
            signal += shadow_noise + small_noise
            
        return max(Constants.MIN_SIGNAL_STRENGTH, signal)

# --------------------------------------------------------------------------
# 3. HomingAlgorithm Class
# --------------------------------------------------------------------------
class HomingAlgorithm:
    """
    RSSI 신호를 기반으로 목표물을 찾아가는 알고리즘에 대한 정의
    이 부분이 가장 중요하며, 수정도 가장 많이 해야할듯
    """
    def __init__(self, start_pos: np.ndarray, params: SimParams):
        # --- 상태 변수 초기화 (가독성을 위해 여러 줄로 분리) ---
        self.pos: np.ndarray = np.array(start_pos, dtype=float)
        self.waypoint: np.ndarray = self.pos.copy()     # 다음 목표 지점
        self.path: list[np.ndarray] = [start_pos.copy()]# 이동 경로 기록
        self.params: SimParams = params
        
        self.is_finished: bool = False                 # 임무 완료 여부
        self.state: str = "IDLE"                       # 현재 알고리즘 상태
        self.last_signal: float = Constants.MIN_SIGNAL_STRENGTH
        self.stuck_counter: int = 0                    # 신호 개선이 없는 연속 횟수
        
        self.best_known_pos: np.ndarray = self.pos.copy() # 가장 신호가 강했던 위치
        self.best_known_signal: float = Constants.MIN_SIGNAL_STRENGTH
        self.ascent_direction: np.ndarray = np.array([1.0, 0.0]) # 현재 탐색 방향 벡터

    def decide_action(self, rssi: float):
        """현재 RSSI 값을 기반으로 다음 행동을 결정하고 상태를 업데이트함"""
        if self.is_finished:
            return

        # 가장 강한 신호 정보 업데이트
        if rssi > self.best_known_signal:
            self.best_known_signal = rssi
            self.best_known_pos = self.pos.copy()

        # 목표 달성 조건 확인
        if rssi >= self.params.TARGET_THRESHOLD:
            self.state = "FINISHED"
            self.is_finished = True
            return

        # 현재 상태에 따라 적절한 행동 실행 (상태 패턴)
        state_handler = {
            "IDLE": self._execute_idle,
            "ADAPTIVE_ASCENT": self._execute_adaptive_ascent,
            "ESCAPING": self._execute_escaping
        }
        handler = state_handler.get(self.state)
        if handler:
            handler(rssi)
        
        self.last_signal = rssi

    def _get_adaptive_distance(self, signal: float) -> float:
        """신호 강도에 따라 전진할 거리를 동적으로 결정"""
        if signal > self.params.SIGNAL_PINPOINT: return self.params.DIST_PINPOINT
        if signal > self.params.SIGNAL_NEAR: return self.params.DIST_NEAR
        if signal > self.params.SIGNAL_MID: return self.params.DIST_MID
        return self.params.DIST_FAR

    def _execute_idle(self, rssi: float):
        """IDLE 상태: 유의미한 신호를 찾을 때까지 대기"""
        if rssi > self.params.ASCENT_THRESHOLD:
            # 탐색을 시작할 만큼 강한 신호를 찾으면, 무작위 방향으로 탐색 시작
            self.state = "ADAPTIVE_ASCENT"
            angle = np.random.uniform(0, 2 * np.pi)
            self.ascent_direction = np.array([np.cos(angle), np.sin(angle)])
            self._execute_adaptive_ascent(rssi)
        else:
            # 신호가 너무 약하면 제자리에 머묾
            self.waypoint = self.pos.copy()

    def _execute_adaptive_ascent(self, rssi: float):
        """ADAPTIVE_ASCENT 상태: 신호가 강해지는 방향으로 전진"""
        if rssi > self.last_signal:
            # 신호가 이전보다 강해졌으면, 같은 방향으로 계속 전진
            self.stuck_counter = 0
            step_distance = self._get_adaptive_distance(rssi)
            self.waypoint = self.pos + self.ascent_direction * step_distance
        else:
            # 신호가 약해졌거나 정체되면, stuck 카운터를 올리고 방향 전환
            self.stuck_counter += 1
            if self.stuck_counter > self.params.STUCK_THRESHOLD:
                # 너무 오래 막혀 있으면 ESCAPING 상태로 전환하여 멀리 벗어남
                self.state = "ESCAPING"
                self.waypoint = self.best_known_pos.copy()
                return
            
            # 현재 위치에서 대기하며 탐색 방향을 일정 각도 회전
            self.waypoint = self.pos.copy()
            rot_matrix = np.array([
                [np.cos(Constants.ROTATION_ANGLE_RAD), -np.sin(Constants.ROTATION_ANGLE_RAD)],
                [np.sin(Constants.ROTATION_ANGLE_RAD), np.cos(Constants.ROTATION_ANGLE_RAD)]
            ])
            self.ascent_direction = np.dot(rot_matrix, self.ascent_direction)

    def _execute_escaping(self, rssi: float):
        """ESCAPING : local maximum에서 벗어나기 위해 무작위 방향으로 멀리 이동"""
        angle = np.random.uniform(0, 2 * np.pi)
        self.ascent_direction = np.array([np.cos(angle), np.sin(angle)])
        self.waypoint = self.best_known_pos + self.ascent_direction * self.params.ESCAPE_DISTANCE
        
        # 탈출 후 다시 탐색 시작
        self.state = "ADAPTIVE_ASCENT"
        self.stuck_counter = 0
        self.last_signal = Constants.MIN_SIGNAL_STRENGTH

    def update_position(self, new_pos: np.ndarray):
        """알고리즘의 현재 위치를 갱신하고 경로에 추가"""
        self.pos = new_pos
        self.path.append(new_pos.copy())

    def get_total_distance(self) -> float:
        """지금까지 이동한 총 거리를 계산"""
        if len(self.path) < 2:
            return 0.0
        path_arr = np.array(self.path)
        return np.sum(np.linalg.norm(np.diff(path_arr, axis=0), axis=1))

# --------------------------------------------------------------------------
# 4. 시각화 클래스 (관심사 분리)
# --------------------------------------------------------------------------
class SimulationVisualizer:
    """
    시뮬레이션 과정을 matplotlib을 사용하여 시각화하는 역할을 담당
    시뮬레이션 로직과 분리되어 코드의 복잡도를 낮춤
    """
    def __init__(self):
        plt.ion()  # 대화형 모드 활성화
        self.fig, self.ax = plt.subplots(figsize=(10, 10))

    def update(self, env: SimulationEnvironment, algo: HomingAlgorithm, true_pos: np.ndarray,
               reported_pos: np.ndarray, simulation_time: float, current_rssi: float):
        """매 시뮬레이션 스텝마다 차트를 업데이트"""
        self.ax.clear()
        path_arr = np.array(algo.path)
        
        self.ax.plot(path_arr[:, 0], path_arr[:, 1], 'o-', color='lightcoral', ms=2, label='Reported Path (GPS)')
        self.ax.plot(true_pos[0], true_pos[1], 'go', markersize=10, label='True Position')
        self.ax.plot(reported_pos[0], reported_pos[1], 'o', color='gold', ms=10, mfc='none', mew=2, label='Reported Position (GPS)')
        self.ax.plot(env.hotspot_pos[0], env.hotspot_pos[1], 'rX', markersize=12, label='Hotspot (Goal)')
        self.ax.plot(algo.waypoint[0], algo.waypoint[1], 'bo', markersize=8, mfc='none', label='Next Waypoint')
        
        self.ax.set_title(f"Time: {simulation_time:.1f}s | State: {algo.state} | RSSI: {current_rssi:.1f} dBm")
        self.ax.set_xlabel("X (meters)")
        self.ax.set_ylabel("Y (meters)")
        self.ax.grid(True)
        self.ax.legend(loc='upper right')
        self.ax.axis('equal')
        
        view_range = max(100, np.max(np.abs(path_arr)) + 20) if len(algo.path) > 0 else 100
        self.ax.set_xlim(-view_range, view_range)
        self.ax.set_ylim(-view_range, view_range)
        
        plt.pause(0.1)

    def close(self):
        """시뮬레이션 종료 후 창을 유지"""
        plt.ioff()
        plt.show()

# --------------------------------------------------------------------------
# 5. 시뮬레이션 실행 클래스 (관심사 분리)
# --------------------------------------------------------------------------
class SimulationRunner:
    """
    시뮬레이션 실행 및 결과 분석을 담당
    단일 실행과 다중 실행 모드를 있음, 단일 실행은  시각화 가능
    1000회 다중 실행 후 통계 분석 기능 포함인데, 이 부분은 아직 timeout밖에 못봄
    """
    def __init__(self, params: SimParams):
        self.params = params

    def run_single(self, visualizer: SimulationVisualizer = None) -> SimResult:
        """단일 시뮬레이션을 실행하고 그 결과를 반환."""
        env = SimulationEnvironment(self.params)
        algo = HomingAlgorithm(start_pos=np.array([0.0, 0.0]), params=self.params)
        true_pos = np.array([0.0, 0.0])

        initial_signal = env.get_signal(true_pos, add_noise=False)
        if initial_signal < self.params.FAILURE_THRESHOLD:
            return SimResult(False, np.linalg.norm(true_pos - env.hotspot_pos), 0, 0, "Initial signal too weak")

        if visualizer:
            print(f"Hotspot is at: {env.hotspot_pos.round(1)}")

        simulation_time = 0.0
        while simulation_time < self.params.TIME_LIMIT:
            # 1. 시간 경과 및 RSSI 스캔
            simulation_time += self.params.RSSI_SCAN_TIME
            current_rssi = env.get_signal(true_pos)
            
            # 2. GPS 오차를 포함한 위치 업데이트
            gps_error = np.random.normal(0, self.params.GPS_ERROR_STD, 2)
            reported_pos = true_pos + gps_error
            algo.update_position(reported_pos)
            
            # 3. 알고리즘 실행
            algo.decide_action(current_rssi)
            if algo.is_finished:
                break

            # 4. 다음 웨이포인트로 이동
            distance_to_travel = np.linalg.norm(algo.waypoint - true_pos)
            time_to_travel = distance_to_travel / self.params.DRONE_SPEED
            simulation_time += time_to_travel
            true_pos = algo.waypoint.copy()

            if visualizer:
                visualizer.update(env, algo, true_pos, reported_pos, simulation_time, current_rssi)
        
        reason = "Success" if algo.is_finished else "Timeout"
        
        return SimResult(
            success=algo.is_finished,
            final_distance=np.linalg.norm(true_pos - env.hotspot_pos),
            total_travel=algo.get_total_distance(),
            search_time=simulation_time,
            reason=reason
        )

    def run_multiple(self, num_simulations: int = 1000):
        """지정된 횟수만큼 시뮬레이션을 반복 실행하고 결과 분석."""
        print(f"Starting {num_simulations} simulations...")
        results = []
        start_time = time.time()
        
        for i in range(num_simulations):
            if (i + 1) % 10 == 0:
                print(f"\rProgress: {i + 1}/{num_simulations}", end="")
            results.append(self.run_single())
            
        end_time = time.time()
        print(f"\nTotal execution time: {end_time - start_time:.2f} seconds")
        self._analyze_results(results)

    def _analyze_results(self, results: list[SimResult]):
        """시뮬레이션 결과 리스트를 받아 통계 정보를 출력"""
        successful_runs = [r for r in results if r.success]
        success_count = len(successful_runs)
        num_simulations = len(results)

        print("\n--- Final Statistical Analysis ---")
        print(f"Success Rate: {success_count / num_simulations * 100:.1f}%")
        
        if successful_runs:
            final_distances = [r.final_distance for r in successful_runs]
            total_travels = [r.total_travel for r in successful_runs]
            search_times = [r.search_time for r in successful_runs]
            
            print(f"Average Final Distance Error (on success): {np.mean(final_distances):.2f} m (Std: {np.std(final_distances):.2f} m)")
            print(f"Average Total Distance Traveled (on success): {np.mean(total_travels):.2f} m (Std: {np.std(total_travels):.2f} m)")
            print(f"Average Search Time (on success): {np.mean(search_times):.2f} s (Std: {np.std(search_times):.2f} s)")

        failure_reasons = {}
        for r in results:
            if not r.success:
                failure_reasons[r.reason] = failure_reasons.get(r.reason, 0) + 1
                
        print("\n--- Failure Analysis ---")
        if not failure_reasons:
            print("No failures recorded.")
        else:
            total_failures = num_simulations - success_count
            if total_failures > 0:
                for reason, count in failure_reasons.items():
                    print(f"- {reason}: {count} times ({count / total_failures * 100:.1f}%)")

# --------------------------------------------------------------------------
# 6. Main Execution Block
# --------------------------------------------------------------------------
def main():
    params = SimParams()
    runner = SimulationRunner(params)

    mode = input("Select mode:\n [1] Single Simulation (Visualized)\n [2] Multi-Simulation (1000 runs & stats)\n >> ")

    if mode == '1':
        print("Starting single simulation...")
        visualizer = SimulationVisualizer()
        result = runner.run_single(visualizer=visualizer)
        visualizer.close()
        
        print("\n--- Simulation Result ---")
        print(f"Success: {result.success} (Reason: {result.reason})")
        print(f"Final Distance Error: {result.final_distance:.2f} m")
        print(f"Total Distance Traveled: {result.total_travel:.2f} m")
        print(f"Total Search Time: {result.search_time:.2f} s")
        
    elif mode == '2':
        runner.run_multiple(1000)
        
    else:
        print("Invalid input.")

if __name__ == "__main__":
    main()
