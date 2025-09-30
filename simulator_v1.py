"""
핫스팟 신호원을 RSSI 신호로 찾는 시뮬레이터 코드

주요 구성 요소는 다음과 같음

데이터 클래스
시뮬레이션에 사용되는 파라미터와 결과값 등 변수 보관하는 곳

SimulationEnvironment
가상 2D 환경을 생성 및 특정 위치에서의 RSSI 신호를 모델링, 추후 수정해야할 부분

HomingAlgorithm(가장 중요함)
RSSI 신호 변화에 따라 다음 행동(웨이포인트)을 결정하는 알고리즘으로 구현

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
    # --- 이동 거리 (신호 강도별) ---
    DIST_FAR: float = 15.0      # 신호가 매우 약할 때의 전진 거리 (기존 18.35 → 15.0)
    DIST_MID: float = 5.0       # 신호가 중간일 때의 전진 거리 (기존 6.20 → 5.5)
    DIST_NEAR: float = 3.0      # 신호가 강할 때의 전진 거리 (기존 3.15 → 2.5)
    DIST_PINPOINT: float = 1.2  # 신호가 매우 강할 때의 전진 거리 (기존 1.25 → 1.0)

    STUCK_THRESHOLD: int = 2
    ESCAPE_DISTANCE: float = 21.0  # 탈출 거리도 약간 줄임 (기존 25.5 → 20.0)

    # --- 신호 임계값 (PATHLOSS_EXPONENT=2.7 기준) ---
    SIGNAL_MID: float = -75.0      # 중간 신호 (기존 -66.5 → -70.0)
    SIGNAL_NEAR: float = -67.0     # 강한 신호 (기존 -59.8 → -62.0)
    SIGNAL_PINPOINT: float = -60.0 # 매우 강한 신호 (기존 -54.5 → -56.0)

    PROBE_DISTANCE: float = 8.0    # 탐사 거리도 약간 줄임 (기존 8.0 → 7.0)
    GPS_DRIFT_FACTOR: float = 0.8        # 드리프트 계수 - 더 낮게
    ROTATION_PENALTY_TIME: float = 1.5

    FAILURE_THRESHOLD: float = -90.0
    ASCENT_THRESHOLD: float = -80.0
    TARGET_THRESHOLD: float = -60.0  # 성공 기준 완화
    DRONE_SPEED: float = 8.0
    RSSI_SCAN_TIME: float = 2.0
    TIME_LIMIT: float = 300.0

    GPS_ERROR_STD: float = 8.0           # GPS 오차 표준편차 (m) - 더 크게
    RSSI_SHADOW_STD: float = 2.2
    SENSOR_DELAY_MEAN: float = 0.12
    SENSOR_DELAY_STD: float = 0.02
    SENSOR_ERROR_STD: float = 1.2
    PATHLOSS_EXPONENT: float = 2.7

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
        
        # [수정] path loss exponent 파라미터 적용
        n = self.params.PATHLOSS_EXPONENT
        signal = -30 - 10 * n * np.log10(distance)

        if add_noise:
            shadow_noise = np.random.normal(0, self.params.RSSI_SHADOW_STD)
            small_noise = np.random.randn() * 0.5
            signal += shadow_noise + small_noise
            
        return max(Constants.MIN_SIGNAL_STRENGTH, signal)

# --------------------------------------------------------------------------
# 3. HomingAlgorithm Class
# --------------------------------------------------------------------------
class HomingAlgorithm:
    """
    RSSI 신호를 기반으로 목표물을 찾아가는 알고리즘 (주변 탐사 로직 적용 버전)
    """
    def __init__(self, start_pos: np.ndarray, params: SimParams):
        # --- 상태 변수 초기화 ---
        self.pos: np.ndarray = np.array(start_pos, dtype=float)
        self.waypoint: np.ndarray = self.pos.copy()
        self.path: list[np.ndarray] = [start_pos.copy()]
        self.params: SimParams = params
        
        self.is_finished: bool = False
        # ### [수정] 상태에 'PROBING' 추가 ###
        self.state: str = "ADAPTIVE_ASCENT"  # 초기 상태를 'ADAPTIVE_ASCENT'로 설정                 
        self.last_signal: float = Constants.MIN_SIGNAL_STRENGTH
        self.stuck_counter: int = 0
        
        self.best_known_pos: np.ndarray = self.pos.copy()
        self.best_known_signal: float = Constants.MIN_SIGNAL_STRENGTH
        self.ascent_direction: np.ndarray = np.array([1.0, 0.0])

        # ### [신규] 주변 탐사(Probing)를 위한 변수들 ###
        self.probe_points: list[np.ndarray] = []    # 탐사할 지점들의 목록
        self.probe_results: list[dict] = []         # 탐사 후 각 지점의 신호 결과 저장
        self.probe_index: int = 0                   # 현재 몇 번째 지점을 탐사 중인지
        self.pre_probe_pos: np.ndarray = self.pos.copy() # 탐사 시작 전 위치
        self.pre_probe_signal: float = Constants.MIN_SIGNAL_STRENGTH # 탐사 시작 전 신호

    def decide_action(self, rssi: float):
        """현재 RSSI 값을 기반으로 다음 행동을 결정하고 상태를 업데이트합니다."""
        if self.is_finished:
            return

        if rssi > self.best_known_signal:
            self.best_known_signal = rssi
            self.best_known_pos = self.pos.copy()

        if rssi >= self.params.TARGET_THRESHOLD:
            self.state = "FINISHED"
            self.is_finished = True
            return

        # ### [수정] 상태 핸들러에 'PROBING' 추가 ###
        state_handler = {
            "ADAPTIVE_ASCENT": self._execute_adaptive_ascent,
            "PROBING": self._execute_probing,
            "ESCAPING": self._execute_escaping
        }
        handler = state_handler.get(self.state)
        if handler:
            handler(rssi)
        
        # PROBING 상태가 아닐 때만 last_signal을 업데이트하여 탐사 전 신호를 기억
        if self.state != "PROBING":
            self.last_signal = rssi

    def _get_adaptive_distance(self, signal: float) -> float:
        """신호 강도에 따라 전진할 거리를 동적으로 결정합니다."""
        if signal > self.params.SIGNAL_PINPOINT: return self.params.DIST_PINPOINT
        if signal > self.params.SIGNAL_NEAR: return self.params.DIST_NEAR
        if signal > self.params.SIGNAL_MID: return self.params.DIST_MID
        return self.params.DIST_FAR

    def _execute_adaptive_ascent(self, rssi: float):
        """ADAPTIVE_ASCENT 상태: 신호가 강해지는 방향으로 전진합니다."""
        if rssi > self.last_signal:
            # 신호가 강해졌으면 같은 방향으로 계속 전진
            self.stuck_counter = 0
            step_distance = self._get_adaptive_distance(rssi)
            self.waypoint = self.pos + self.ascent_direction * step_distance
        else:
            # ### [수정] 신호가 약해지면 제자리 회전 대신 '주변 탐사' 시작 ###
            self.state = "PROBING"
            self.pre_probe_pos = self.pos.copy()
            self.pre_probe_signal = self.last_signal # 탐사 전 마지막 신호를 기억
            
            # 현재 위치 기준 동서남북 4방향으로 탐사 지점 생성
            offsets = np.array([
                [0, self.params.PROBE_DISTANCE], [self.params.PROBE_DISTANCE, 0],
                [0, -self.params.PROBE_DISTANCE], [-self.params.PROBE_DISTANCE, 0]
            ])
            self.probe_points = [self.pre_probe_pos + offset for offset in offsets]
            
            # 탐사 변수 초기화 및 첫 번째 탐사 지점으로 이동 시작
            self.probe_index = 0
            self.probe_results = []
            self.waypoint = self.probe_points[self.probe_index]

    # ### [신규] 주변 탐사를 수행하는 상태 로직 ###
    def _execute_probing(self, rssi: float):
        """PROBING 상태: 주변 4개 지점을 방문하며 신호를 측정합니다."""
        # 방금 도착한 탐사 지점의 결과를 기록
        self.probe_results.append({'pos': self.pos.copy(), 'rssi': rssi})
        self.probe_index += 1

        if self.probe_index < len(self.probe_points):
            # 아직 탐사할 지점이 남았다면 다음 지점으로 이동
            self.waypoint = self.probe_points[self.probe_index]
        else:
            # 4방향 탐사가 모두 끝났으면 결과를 분석
            best_probe_result = max(self.probe_results, key=lambda x: x['rssi'])

            # 탐사 결과, 기존보다 더 나은 지점을 찾았는지 확인
            if best_probe_result['rssi'] > self.pre_probe_signal:
                # 더 나은 방향을 찾았음!
                self.stuck_counter = 0
                
                # 탐사 시작점에서 가장 신호가 좋았던 지점으로의 방향을 새로운 전진 방향으로 설정
                new_direction_vector = best_probe_result['pos'] - self.pre_probe_pos
                self.ascent_direction = new_direction_vector / np.linalg.norm(new_direction_vector)
                
                # 가장 신호가 좋았던 지점에서부터 새로운 방향으로 전진 시작
                self.waypoint = best_probe_result['pos'] + self.ascent_direction * self._get_adaptive_distance(best_probe_result['rssi'])
                self.state = "ADAPTIVE_ASCENT"
            else:
                # 주변을 다 둘러봐도 더 나은 곳이 없음. 'Stuck'으로 판단.
                self.stuck_counter += 1
                if self.stuck_counter > self.params.STUCK_THRESHOLD:
                    # 너무 많이 막혔으므로 ESCAPING 상태로 전환
                    self.state = "ESCAPING"
                    self.waypoint = self.best_known_pos.copy()
                else:
                    # 아직 기회가 남았으므로 탐사 시작 지점으로 복귀 후 다시 시도
                    self.waypoint = self.pre_probe_pos
                    self.state = "ADAPTIVE_ASCENT"

    def _execute_escaping(self, rssi: float):
        """ESCAPING 상태: 국소 최댓값에서 벗어나기 위해 여러 번 무작위 방향으로 멀리 이동합니다."""
        escape_steps = 3  # 탈출 시 반복 횟수 (원하는 만큼 조정 가능)
        escape_distance = self.params.ESCAPE_DISTANCE
        pos = self.pos.copy()
        for _ in range(escape_steps):
            angle = np.random.uniform(0, 2 * np.pi)
            direction = np.array([np.cos(angle), np.sin(angle)])
            pos += direction * escape_distance
        self.ascent_direction = direction
        self.waypoint = pos
        self.state = "ADAPTIVE_ASCENT"
        self.stuck_counter = 0
        self.last_signal = Constants.MIN_SIGNAL_STRENGTH

    def update_position(self, new_pos: np.ndarray):
        """알고리즘의 현재 위치를 갱신하고 경로에 추가합니다."""
        self.pos = new_pos
        self.path.append(new_pos.copy())

    def get_total_distance(self) -> float:
        """지금까지 이동한 총 거리를 계산합니다."""
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
        plt.ion()
        self.fig, self.ax = plt.subplots(figsize=(10, 10))
        
    def update(self, env: SimulationEnvironment, algo: HomingAlgorithm, 
               reported_pos: np.ndarray, simulation_time: float, current_rssi: float):
        self.ax.clear()
        reported_path_arr = np.array(algo.path)
        self.ax.plot(reported_path_arr[:, 0], reported_path_arr[:, 1], 'o--', color='lightcoral', ms=2, label='Drone Path)')
        self.ax.plot(reported_pos[0], reported_pos[1], 'o', color='gold', ms=10, mfc='none', mew=2, label='pos')
        self.ax.plot(env.hotspot_pos[0], env.hotspot_pos[1], 'rX', markersize=12, label='Hotspot (Goal)')
        self.ax.plot(algo.waypoint[0], algo.waypoint[1], 'bo', markersize=8, mfc='none', label='Next Waypoint')

        self.ax.set_title(f"Time: {simulation_time:.1f}s | State: {algo.state} | RSSI: {current_rssi:.1f} dBm")
        self.ax.set_xlabel("X (meters)")
        self.ax.set_ylabel("Y (meters)")
        self.ax.grid(True, which='both', linestyle='--', linewidth=0.5, alpha=0.7)
        self.ax.legend(loc='upper right')
        self.ax.axis('equal')
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
        """단일 시뮬레이션을 실행하고 그 결과를 반환합니다. (오차 누적 모델 최종 수정 버전)"""
        env = SimulationEnvironment(self.params)
        algo = HomingAlgorithm(start_pos=np.array([0.0, 0.0]), params=self.params)
        
        # --- 초기화 ---
        true_pos = np.array([0.0, 0.0])
        previous_gps_error = np.zeros(2)
        previous_direction = None

        # --- 시뮬레이션 루프 ---
        simulation_time = 0.0
        while simulation_time < self.params.TIME_LIMIT:
            # 1. 인식(Perceive): 현재 실제 위치에 GPS 오차를 더해, 드론이 인식하는 자신의 위치를 계산합니다.
            new_random_error = np.random.normal(0, self.params.GPS_ERROR_STD, 2)
            drift_factor = self.params.GPS_DRIFT_FACTOR
            gps_error = drift_factor * previous_gps_error + (1 - drift_factor) * new_random_error
            previous_gps_error = gps_error

            reported_pos = true_pos + gps_error
            algo.update_position(reported_pos)

            # 2. 판단(Decide): 현재 신호를 측정하고, 인식된 위치를 기반으로 다음 목표(waypoint)를 결정합니다.
            # 센서 지연 및 오차 적용
            sensor_delay = max(0.0, np.random.normal(self.params.SENSOR_DELAY_MEAN, self.params.SENSOR_DELAY_STD))
            sensor_error = np.random.normal(0, self.params.SENSOR_ERROR_STD)
            current_rssi = env.get_signal(true_pos) + sensor_error
            simulation_time += sensor_delay

            algo.decide_action(current_rssi)

            # --- 현실적 이동 방식 적용 ---
            # 드론은 reported_pos에서 웨이포인트까지 이동한다고 인식함
            move_vector = algo.waypoint - reported_pos
            move_distance = np.linalg.norm(move_vector)
            move_direction = move_vector / move_distance if move_distance > 0 else None

            rotation_penalty = 0.0
            if previous_direction is not None and move_distance > 0:
                angle_change = np.arccos(np.clip(np.dot(previous_direction, move_direction), -1.0, 1.0))
                if angle_change > np.deg2rad(10):
                    rotation_penalty = self.params.ROTATION_PENALTY_TIME
            previous_direction = move_direction

            # 실제 위치를 reported_pos에서 웨이포인트까지 이동시키는 것처럼 업데이트
            true_pos += move_vector
            time_to_travel = move_distance / self.params.DRONE_SPEED
            simulation_time += time_to_travel + self.params.RSSI_SCAN_TIME + rotation_penalty

            if visualizer:
                visualizer.update(env, algo, reported_pos, simulation_time, current_rssi)

            # --- 성공 판정 후 웨이포인트까지 이동 ---
            if algo.is_finished:
                break
        
        # --- 결과 정리 ---
        reason = "Success" if algo.is_finished else "Timeout"
        final_distance = np.linalg.norm(true_pos - env.hotspot_pos)
        waypoint_distance = np.linalg.norm(true_pos - algo.waypoint)
        print(f"\n[종료] 실제 위치와 핫스팟 거리: {final_distance:.2f} m")
        print(f"[종료] 실제 위치와 웨이포인트 거리: {waypoint_distance:.2f} m")
        print(f"[종료] GPS 오차: {previous_gps_error}")

        return SimResult(
            success=algo.is_finished,
            final_distance=final_distance,
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
