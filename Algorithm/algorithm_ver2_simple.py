import numpy as np
import math
import time
import matplotlib.pyplot as plt
from dataclasses import dataclass, field
import warnings

warnings.filterwarnings('ignore')  # 모든 경고 차단


# ==========================================================================
# 상하좌우 탐색
# ==========================================================================


# --------------------------------------------------------------------------
# 1. 상수 및 데이터 클래스 정의
# --------------------------------------------------------------------------
@dataclass
class Constants:
    INITIAL_DISTANCE: float = 100.0
    MIN_SIGNAL_STRENGTH: float = -100.0
    MIN_DISTANCE_TO_HOTSPOT: float = 1.0
    ROTATION_ANGLE_RAD: float = np.pi / 4


@dataclass
class SimParams:
    TRANSMIT_POWER_DBM: float = 15.0

    # --- [알고리즘 파라미터] ---
    # 신호 강도에 따라 적용할 이동 보폭 (Step Size)
    STEP_FAR: float = 30.0  # 신호 약함 (< -55): 30m 이동
    STEP_MID: float = 15.0  # 신호 중간: 15m 이동
    STEP_NEAR: float = 5.0  # 신호 강함 (> -45): 5m 이동

    # 탐색을 위해 실제로 갔다 오는 거리 (Radius)
    SCAN_RADIUS_FAR: float = 20.0  # 멀 때는 20m씩 왕복
    SCAN_RADIUS_MID: float = 10.0  # 중간엔 10m씩 왕복
    SCAN_RADIUS_NEAR: float = 5.0  # 가까우면 5m씩 왕복
    # ------------------------

    # 신호 강도 기준점
    SIGNAL_MID: float = -50.0
    SIGNAL_NEAR: float = -40.0
    SIGNAL_PINPOINT: float = -35.0

    DIST_FAR: float = 15.0
    DIST_MID: float = 5.0
    DIST_NEAR: float = 3.0
    DIST_PINPOINT: float = 0.8

    STUCK_THRESHOLD: int = 3
    ESCAPE_DISTANCE: float = 25.0
    ASCENT_THRESHOLD: float = -60.0
    PROBE_DISTANCE: float = 20.0  # (사용 안 함, 위 SCAN_RADIUS로 대체됨)

    GPS_DRIFT_FACTOR: float = 0.8
    ROTATION_PENALTY_TIME: float = 1.5
    DRONE_SPEED: float = 15.0  # 속도 상향
    RSSI_SCAN_TIME: float = 0.5
    TIME_LIMIT: float = 100000.0
    GPS_ERROR_STD: float = 3.0
    RSSI_SHADOW_STD: float = 1.0
    SENSOR_DELAY_MEAN: float = 0.12
    SENSOR_DELAY_STD: float = 0.02
    SENSOR_ERROR_STD: float = 1.2
    NUM_ESCAPE_SAMPLES: int = 8
    ESCAPE_SAMPLE_RADIUS: float = 20.0

    MOMENTUM_FACTOR: float = 0.1
    MAX_MOMENTUM_BOOST: float = 2.5
    RSSI_SMOOTHING_FACTOR: float = 0.3

    ENABLE_FADING: bool = True
    RICIAN_K_FACTOR: float = 6.0


@dataclass
class SimResult:
    """단일 시뮬레이션의 결과를 저장하는 구조체 역할"""
    success: bool
    final_distance: float
    total_travel: float
    search_time: float
    reason: str
    waypoint_count: int
    waypoints_at_threshold: int
    rssi_at_success: float
    true_total_travel: float


# --------------------------------------------------------------------------
# 2. Simulation Environment Class
# --------------------------------------------------------------------------
class SimulationEnvironment:
    def __init__(self, params: SimParams):
        self.params = params
        angle = np.random.uniform(0, 2 * np.pi)
        self.hotspot_pos = np.array([
            Constants.INITIAL_DISTANCE * np.cos(angle),
            Constants.INITIAL_DISTANCE * np.sin(angle)
        ])

    def apply_rician_fading(self, signal_db: float) -> float:
        if not self.params.ENABLE_FADING: return signal_db
        signal_linear = 10 ** (signal_db / 10)
        K_linear = 10 ** (self.params.RICIAN_K_FACTOR / 10)
        los_component = np.sqrt(K_linear / (K_linear + 1))
        scatter_scale = 1 / np.sqrt(2 * (K_linear + 1))
        scatter_real = np.random.normal(0, scatter_scale)
        scatter_imag = np.random.normal(0, scatter_scale)
        amplitude = np.abs(los_component + scatter_real + 1j * scatter_imag)
        faded_signal_linear = signal_linear * (amplitude ** 2)
        return 10 * np.log10(faded_signal_linear) if faded_signal_linear > 0 else Constants.MIN_SIGNAL_STRENGTH

    def get_signal(self, pos: np.ndarray, add_noise: bool = True) -> float:
        distance = np.linalg.norm(pos - self.hotspot_pos)
        distance = max(distance, Constants.MIN_DISTANCE_TO_HOTSPOT)
        path_loss_db = 30.0 + 20 * np.log10(distance)
        signal = self.params.TRANSMIT_POWER_DBM - path_loss_db
        signal = self.apply_rician_fading(signal)
        if add_noise:
            signal += np.random.normal(0, self.params.RSSI_SHADOW_STD)
            signal += np.random.randn() * 0.5
        return max(Constants.MIN_SIGNAL_STRENGTH, signal)


# --------------------------------------------------------------------------
# 3. HomingAlgorithm Class (십자 탐색)
# --------------------------------------------------------------------------
class HomingAlgorithm:
    def __init__(self, start_pos: np.ndarray, params: SimParams):
        self.pos = np.array(start_pos, dtype=float)
        self.waypoint = np.array(start_pos, dtype=float)
        self.path = [start_pos.copy()]
        self.params = params
        self.is_finished = False

        # 상태 머신: INIT -> OUT -> IN -> DECIDE -> MOVE -> INIT
        self.state = "INIT"
        self.center_pos = start_pos.copy()

        # 현재 단계에서 사용할 보폭과 탐색 반경
        self.current_step = self.params.STEP_FAR
        self.current_radius = self.params.SCAN_RADIUS_FAR

        self.directions = [
            np.array([1.0, 0.0]), np.array([-1.0, 0.0]),
            np.array([0.0, -1.0]), np.array([0.0, 1.0])
        ]
        self.dir_idx = 0
        self.scan_results = {}

        # 통계 및 호환성 변수
        self.waypoint_count = 0
        self.smoothed_rssi = Constants.MIN_SIGNAL_STRENGTH
        self.true_path = [start_pos.copy()]
        self.current_true_pos = start_pos.copy()
        self.last_signal = Constants.MIN_SIGNAL_STRENGTH
        self.stuck_counter = 0
        self.best_known_pos = self.pos.copy()
        self.best_known_signal = Constants.MIN_SIGNAL_STRENGTH
        self.ascent_direction = np.array([1.0, 0.0])
        self.success_streak = 0

    def update_true_position(self, new_true_pos: np.ndarray):
        self.current_true_pos = new_true_pos
        self.true_path.append(new_true_pos.copy())

    def get_true_total_distance(self) -> float:
        return np.sum(np.linalg.norm(np.diff(np.array(self.true_path), axis=0), axis=1)) if len(
            self.true_path) > 1 else 0.0

    def update_position(self, new_pos: np.ndarray):
        self.pos = new_pos
        self.path.append(new_pos.copy())

    def get_total_distance(self) -> float:
        return np.sum(np.linalg.norm(np.diff(np.array(self.path), axis=0), axis=1)) if len(self.path) > 1 else 0.0

    def decide_action(self, rssi: float):
        if self.is_finished: return
        self.smoothed_rssi = rssi

        # 도착 판정 (GPS 오차 고려 2.5m)
        dist = np.linalg.norm(self.pos - self.waypoint)
        arrived = dist < 2.5

        # --- 상태 머신 ---
        if self.state == "INIT":
            self.center_pos = self.pos.copy()
            self.dir_idx = 0
            self.scan_results = {}

            # [핵심] 신호 세기에 따라 보폭(Step)과 탐색 반경(Radius) 결정
            if rssi < self.params.SIGNAL_MID:
                self.current_step = self.params.STEP_FAR  # 30m
                self.current_radius = self.params.SCAN_RADIUS_FAR  # 20m
            elif rssi < self.params.SIGNAL_NEAR:
                self.current_step = self.params.STEP_MID  # 15m
                self.current_radius = self.params.SCAN_RADIUS_MID  # 10m
            else:
                self.current_step = self.params.STEP_NEAR  # 5m
                self.current_radius = self.params.SCAN_RADIUS_NEAR  # 5m

            # 첫 방향(동쪽) 탐색 출발
            target = self.center_pos + self.directions[self.dir_idx] * self.current_radius
            self.waypoint = target
            self.state = "OUTBOUND"
            self.waypoint_count += 1

        elif self.state == "OUTBOUND":
            if arrived:
                # 끝점 도착 -> 측정값 저장
                self.scan_results[self.dir_idx] = rssi
                # 중심으로 복귀 명령
                self.waypoint = self.center_pos
                self.state = "INBOUND"
                self.waypoint_count += 1

        elif self.state == "INBOUND":
            if arrived:
                # 중심 복귀 완료 -> 다음 방향 설정
                self.dir_idx += 1
                if self.dir_idx < 4:
                    target = self.center_pos + self.directions[self.dir_idx] * self.current_radius
                    self.waypoint = target
                    self.state = "OUTBOUND"
                    self.waypoint_count += 1
                else:
                    self.state = "DECIDING"

        if self.state == "DECIDING":
            best_idx = -1
            best_rssi = -9999.0

            for idx, val in self.scan_results.items():
                if val > best_rssi:
                    best_rssi = val
                    best_idx = idx

            if best_idx != -1:
                # 가장 좋은 방향으로 '보폭(Step)'만큼 실제 이동
                best_dir = self.directions[best_idx]
                new_center = self.center_pos + best_dir * self.current_step

                self.waypoint = new_center
                self.center_pos = new_center
                self.state = "MOVING_CENTER"
                self.waypoint_count += 1
            else:
                self.state = "INIT"  # 예외 처리

        elif self.state == "MOVING_CENTER":
            if arrived:
                self.state = "INIT"  # 도착했으면 다시 탐색 시작

# --------------------------------------------------------------------------
# 4. 시각화 클래스
# --------------------------------------------------------------------------
class SimulationVisualizer:
    def __init__(self, time_scale: float = 1.0, view_radius_m: float = 50.0):
        plt.ion()
        self.fig, self.ax = plt.subplots(figsize=(10, 10))
        self.start_real_time = None
        self.start_simulation_time = None
        self.time_scale = time_scale
        self.view_radius = view_radius_m
        self.path_history = []

    def update(self, env, true_pos, reported_pos, simulation_time, current_rssi, status="Moving",
               algo_state: str = "INIT", next_waypoint: np.ndarray = None):
        if self.start_real_time is None:
            self.start_real_time = time.time()
            self.start_simulation_time = simulation_time

        sim_elapsed = simulation_time - self.start_simulation_time
        real_elapsed = time.time() - self.start_real_time
        target_real = sim_elapsed / self.time_scale
        if target_real > real_elapsed:
            time.sleep(target_real - real_elapsed)

        if len(self.path_history) == 0 or np.linalg.norm(true_pos - np.array(self.path_history[-1])) > 0.1:
            self.path_history.append(true_pos.copy())

        self.ax.clear()
        if len(self.path_history) > 1:
            path = np.array(self.path_history)
            self.ax.plot(path[:, 0], path[:, 1], 'c-', alpha=0.6, linewidth=1, label='Path')

        self.ax.plot(true_pos[0], true_pos[1], 'bo', markersize=14, label='Drone', zorder=5)
        if reported_pos is not None:
            self.ax.plot(reported_pos[0], reported_pos[1], 'bx', markersize=10, alpha=0.7, label='GPS Reading',
                         zorder=4)

        self.ax.plot(env.hotspot_pos[0], env.hotspot_pos[1], 'r*', markersize=20, label='Hotspot', zorder=4)

        if next_waypoint is not None:
            self.ax.plot(next_waypoint[0], next_waypoint[1], 'go', markersize=10, mfc='none', mew=2,
                         label='Next Waypoint', zorder=3)
            self.ax.plot([true_pos[0], next_waypoint[0]], [true_pos[1], next_waypoint[1]], 'y--', alpha=0.7,
                         linewidth=1.5)

        dist = np.linalg.norm(true_pos - env.hotspot_pos)

        display_state = algo_state
        if algo_state == "OUTBOUND":
            display_state = "SCAN (GO)"
        elif algo_state == "INBOUND":
            display_state = "SCAN (BACK)"
        elif algo_state == "MOVING_CENTER":
            display_state = "MOVING >>"

        title_line_1 = f"[{display_state}] Time: {simulation_time:.1f}s"
        title_line_2 = f"Dist: {dist:.1f}m | RSSI: {current_rssi:.1f} dBm"

        self.ax.set_title(title_line_1 + " | " + title_line_2)
        self.ax.set_xlabel("X (m)");
        self.ax.set_ylabel("Y (m)")
        self.ax.grid(True, linestyle='--', alpha=0.3)
        self.ax.legend(loc='upper right')

        if status == "SUCCESS" and len(self.path_history) > 0:
            path_arr = np.array(self.path_history)
            all_x = np.append(path_arr[:, 0], [env.hotspot_pos[0]])
            all_y = np.append(path_arr[:, 1], [env.hotspot_pos[1]])
            margin = 30.0
            self.ax.set_xlim(np.min(all_x) - margin, np.max(all_x) + margin)
            self.ax.set_ylim(np.min(all_y) - margin, np.max(all_y) + margin)
        else:
            self.ax.set_xlim(true_pos[0] - self.view_radius, true_pos[0] + self.view_radius)
            self.ax.set_ylim(true_pos[1] - self.view_radius, true_pos[1] + self.view_radius)

        plt.pause(0.00001)

    def close(self):
        plt.ioff();
        plt.show()


# --------------------------------------------------------------------------
# 5. 시뮬레이션 실행 클래스
# --------------------------------------------------------------------------
class SimulationRunner:
    def __init__(self, params: SimParams):
        self.params = params

    def run_single(self, visualizer: SimulationVisualizer = None) -> SimResult:
        env = SimulationEnvironment(self.params)
        algo = HomingAlgorithm(start_pos=np.array([0.0, 0.0]), params=self.params)

        true_pos = np.array([0.0, 0.0])
        reported_pos = true_pos.copy()
        previous_gps_error, previous_direction = np.zeros(2), None
        waypoints_at_threshold_pass, threshold_passed = 0, False  # 복구됨

        rssi_at_success: float = 0.0

        current_std = self.params.RSSI_SHADOW_STD
        if self.params.RICIAN_K_FACTOR == 0.0:
            threshold_map = {1.0: -26.7, 3.0: -24.0, 5.0: -20.2}
        else:
            threshold_map = {1.0: -28.8, 3.0: -25.7, 5.0: -21.2}
        if current_std in threshold_map:
            success_threshold = threshold_map[current_std]
        else:
            success_threshold = -28.0 if current_std <= 1.0 else -21.0

        if visualizer:
            print(f"\nSimulation with RSSI_SHADOW_STD = {current_std:.1f}")
            print(f"Success Threshold: {success_threshold:.2f} dBm")

        simulation_time = 0.0
        loop_count = 0

        while simulation_time < self.params.TIME_LIMIT:
            loop_count += 1

            new_err = np.random.normal(0, self.params.GPS_ERROR_STD, 2)
            gps_err = self.params.GPS_DRIFT_FACTOR * previous_gps_error + (1 - self.params.GPS_DRIFT_FACTOR) * new_err
            previous_gps_error = gps_err
            reported_pos = true_pos + gps_err

            algo.update_position(reported_pos)

            rssi = env.get_signal(true_pos)
            simulation_time += max(0.0, np.random.normal(self.params.SENSOR_DELAY_MEAN, self.params.SENSOR_DELAY_STD))

            algo.decide_action(rssi)

            if visualizer and loop_count <= 5:
                print(f"[Loop {loop_count}] State: {algo.state} | RSSI: {rssi:.2f} | WP: {algo.waypoint}")

            if visualizer:
                visualizer.update(env, true_pos, reported_pos, simulation_time, rssi, "Active", algo.state,
                                  algo.waypoint)

            if not algo.is_finished and rssi >= success_threshold:
                algo.is_finished = True
                rssi_at_success = rssi
                algo.state = "SUCCESS"

            move_vec = algo.waypoint - reported_pos
            dist = np.linalg.norm(move_vec)

            if dist > 0:
                direction = move_vec / dist
                penalty = 0.0
                if previous_direction is not None and np.dot(previous_direction, direction) < 0.9:
                    penalty = self.params.ROTATION_PENALTY_TIME
                previous_direction = direction

                travel_time = dist / self.params.DRONE_SPEED
                num_steps = max(1, int(travel_time / 0.05))
                step_vec = move_vec / num_steps
                step_time = travel_time / num_steps

                for _ in range(num_steps):
                    true_pos += step_vec
                    algo.update_true_position(true_pos)

                    new_err = np.random.normal(0, self.params.GPS_ERROR_STD, 2)
                    gps_err = self.params.GPS_DRIFT_FACTOR * previous_gps_error + (
                                1 - self.params.GPS_DRIFT_FACTOR) * new_err
                    previous_gps_error = gps_err
                    reported_pos = true_pos + gps_err

                    algo.update_position(reported_pos)
                    simulation_time += step_time

                    if visualizer:
                        visualizer.update(env, true_pos, reported_pos, simulation_time, rssi, "Moving", algo.state,
                                          algo.waypoint)

                    if algo.is_finished: break

                simulation_time += penalty

            if algo.is_finished: break

        if visualizer and algo.is_finished:
            visualizer.update(env, true_pos, reported_pos, simulation_time, rssi, "SUCCESS", "FINISHED", None)
            time.sleep(2)

        return SimResult(
            algo.is_finished, np.linalg.norm(true_pos - env.hotspot_pos),
            algo.get_total_distance(), simulation_time, "Success" if algo.is_finished else "Timeout",
            algo.waypoint_count, waypoints_at_threshold_pass, rssi_at_success, algo.get_true_total_distance()
        )

    def run_multiple(self, num_simulations: int = 1000):
        print(f"Starting {num_simulations} simulations...")
        results = []
        start_time = time.time()
        for i in range(num_simulations):
            if (i + 1) % 10 == 0:
                if (i + 1) % 100 == 0: print(f"\rProgress: {i + 1}/{num_simulations}", end="")
            results.append(self.run_single())
        end_time = time.time()
        print(f"\nTotal execution time: {end_time - start_time:.2f} seconds")
        self._analyze_results(results)

    # ✅ [복구 완료] 원하시던 상세 통계 출력 (Mean + Std)
    def _analyze_results(self, results: list[SimResult]):
        successful_runs = [r for r in results if r.success]
        success_count = len(successful_runs)
        print(f"\n--- Final Statistical Analysis ---")
        print(f"Success Rate: {success_count / len(results) * 100:.1f}%")

        if successful_runs:
            success_waypoints = [r.waypoint_count for r in successful_runs]
            success_rssi = [r.rssi_at_success for r in successful_runs if r.rssi_at_success != 0.0]
            total_travels = [r.total_travel for r in successful_runs]
            true_total_travels = [r.true_total_travel for r in successful_runs]
            search_times = [r.search_time for r in successful_runs]

            print(f"\n--- Stats on Successful Runs ({len(successful_runs)} runs) ---")
            print(f"Avg Waypoints Generated: {np.mean(success_waypoints):.1f} (Std: {np.std(success_waypoints):.1f})")
            if success_rssi:
                print(
                    f"Avg RSSI at Success Moment: {np.mean(success_rssi):.2f} dBm (Std: {np.std(success_rssi):.2f} dBm)")
            print(f"Avg Total Distance Traveled: {np.mean(total_travels):.2f} m (Std: {np.std(total_travels):.2f} m)")
            print(
                f"Avg Total Distance Traveled (True Pos): {np.mean(true_total_travels):.2f} m (Std: {np.std(true_total_travels):.2f} m)")
            print(f"Avg Search Time: {np.mean(search_times):.2f} s (Std: {np.std(search_times):.2f} s)")

        failure_reasons = {}
        for r in results:
            if not r.success: failure_reasons[r.reason] = failure_reasons.get(r.reason, 0) + 1
        print("\n--- Failure Analysis ---")
        if not failure_reasons:
            print("No failures recorded.")
        else:
            for r, c in failure_reasons.items(): print(f"- {r}: {c} times ({c / len(results) * 100:.1f}%)")


# --------------------------------------------------------------------------
# 6. Main Execution Block
# --------------------------------------------------------------------------
def main():
    mode = input(
        "Select mode:\n"
        " [1] Single Simulation (Visualized)\n"
        " [2] Multi-Simulation (Statistical Analysis for RSSI_SHADOW_STD = 1, 3, 5)\n"
        " >> "
    )

    if mode == '1':
        print("\nStarting single simulation...")
        print("\n배속을 선택하세요:")
        print(" [1] 0.5배속 (느림)")
        print(" [2] 1.0배속 (실시간)")
        print(" [3] 최대속 (가장 빠름)")
        speed_choice = input("선택 [기본값: 2]: ").strip()

        speed_map = {'1': 0.5, '2': 1.0, '3': 100.0}  # 100배 = 최대한 빠르게
        time_scale = speed_map.get(speed_choice, 1.0)

        params = SimParams()
        runner = SimulationRunner(params)
        visualizer = SimulationVisualizer(time_scale=time_scale, view_radius_m=100.0)
        result = runner.run_single(visualizer=visualizer)
        visualizer.close()

        print("\n--- Simulation Result ---")
        print(f"Success: {result.success} (Reason: {result.reason})")
        if result.success:
            print("\n--- On Success ---")
            print(f"Waypoints Generated: {result.waypoint_count}")
            print(f"RSSI at Success Moment: {result.rssi_at_success:.2f} dBm")
            print(f"Total Distance Traveled: {result.total_travel:.2f} m")
            print(f"Total Distance Traveled (True Pos): {result.true_total_travel:.2f} m")

        print("\n--- General Info ---")
        print(f"Final Distance to Hotspot: {result.final_distance:.2f} m")
        print(f"Total Search Time: {result.search_time:.2f} s")


    elif mode == '2':

        shadow_std_values = [1.0, 3.0, 5.0]

        for std_val in shadow_std_values:
            print("\n" + "=" * 50)
            print(f"### Starting Simulation for RSSI_SHADOW_STD = {std_val:.1f} ###")
            print("=" * 50)
            params = SimParams()
            params.RSSI_SHADOW_STD = std_val
            runner = SimulationRunner(params)
            runner.run_multiple(1000)
    else:
        print("Invalid input. Please enter 1 or 2.")


if __name__ == "__main__":
    main()
