import numpy as np
import math
import time
import matplotlib.pyplot as plt
from dataclasses import dataclass, field
import warnings

warnings.filterwarnings('ignore')  # 모든 경고 차단

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
    DIST_FAR: float = 15.0
    DIST_MID: float = 5.0
    DIST_NEAR: float = 3.0
    DIST_PINPOINT: float = 0.8  # [최적화] 1.2 → 0.8m

    STUCK_THRESHOLD: int = 3
    ESCAPE_DISTANCE: float = 25.0

    SIGNAL_MID: float = -51.0
    SIGNAL_NEAR: float = -45.0
    SIGNAL_PINPOINT: float = -35.0
    ASCENT_THRESHOLD: float = -60.0

    PROBE_DISTANCE: float = 8.0

    GPS_DRIFT_FACTOR: float = 0.8  # (0~1) GPS 오차의 표류 강도. 0이면 매번 독립적인 오차(표류 없음), 1에 가까울수록 오차가 천천히 변함.

    ROTATION_PENALTY_TIME: float = 1.5
    DRONE_SPEED: float = 8.0
    RSSI_SCAN_TIME: float = 2.0
    TIME_LIMIT: float = 100000.0
    GPS_ERROR_STD: float = 3.0  # [최적화] 8.0 → 3.0m (드론용 RTK GPS)
    RSSI_SHADOW_STD: float = 1.0
    SENSOR_DELAY_MEAN: float = 0.12
    SENSOR_DELAY_STD: float = 0.02
    SENSOR_ERROR_STD: float = 1.2
    NUM_ESCAPE_SAMPLES: int = 8
    ESCAPE_SAMPLE_RADIUS: float = 20.0

    ### 관성(Momentum) 파라미터 ###
    MOMENTUM_FACTOR: float = 0.1  # 성공 스트리크 당 이동 속도 증가량
    MAX_MOMENTUM_BOOST: float = 2.5  # 최대 가속 배율
    ##############################

    ### 지수이동평균_평활상수 ###
    """(0~1 사이 값) 신호 필터링 강도. 높을수록 현재 값에 민감, 낮을수록 둔감."""
    RSSI_SMOOTHING_FACTOR: float = 0.3
    ########################

    # --- 페이딩 모델 파라미터 ---
    ENABLE_FADING: bool = True
    RICIAN_K_FACTOR: float = 6.0  # 라이시안 K 팩터 (dB)
    # K > 10: LOS (페이딩 거의 없음)
    # K = 3~10 dB: LOS + 산란 혼합
    # K ≈ 0.01 dB: 레이리 페이딩 (NLOS)


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
    true_total_travel: float  # [추가] True Pos 기반 실제 이동 거리


# --------------------------------------------------------------------------
# 2. Simulation Environment Class
# --------------------------------------------------------------------------
class SimulationEnvironment:
    def __init__(self, params: SimParams):
        if not isinstance(params, SimParams):
            raise TypeError("params는 SimParams의 인스턴스여야 합니다.")
        self.params = params
        angle = np.random.uniform(0, 2 * np.pi)
        self.hotspot_pos = np.array([
            Constants.INITIAL_DISTANCE * np.cos(angle),
            Constants.INITIAL_DISTANCE * np.sin(angle)
        ])

    def apply_rician_fading(self, signal_db: float) -> float:
        """
        라이시안 페이딩을 신호에 적용

        라이시안 페이딩 모델:
        수신 신호 진폭 r은 다음 확률 밀도 함수를 따름:

        f(r) = (r/σ²) * exp(-(r² + s²)/(2σ²)) * I₀(rs/σ²)

        K-factor 조절로 다양한 환경 표현:
        - K > 10 dB: 순수 LOS (페이딩 거의 없음)
        - K = 3~10 dB: LOS + 산란 혼합 환경
        - K → 0 dB: 레이리 페이딩 (순수 NLOS)
        """
        if not self.params.ENABLE_FADING:
            return signal_db

        # dB를 선형 스케일로 변환
        signal_linear = 10 ** (signal_db / 10)

        # K-factor를 선형 스케일로 변환
        K_linear = 10 ** (self.params.RICIAN_K_FACTOR / 10)

        # LOS 성분
        los_component = np.sqrt(K_linear / (K_linear + 1))

        # 산란 성분의 표준편차
        scatter_scale = 1 / np.sqrt(2 * (K_linear + 1))

        # 산란 성분 생성
        scatter_real = np.random.normal(0, scatter_scale)  # N₁
        scatter_imag = np.random.normal(0, scatter_scale)  # N₂

        # 복소 진폭의 크기
        amplitude = np.abs(los_component + scatter_real + 1j * scatter_imag)

        # 수신 전력 = 진폭²
        faded_signal_linear = signal_linear * (amplitude ** 2)

        # 선형 스케일을 다시 dB로 변환
        if faded_signal_linear > 0:
            return 10 * np.log10(faded_signal_linear)
        else:
            return Constants.MIN_SIGNAL_STRENGTH

    def get_signal(self, pos: np.ndarray, add_noise: bool = True) -> float:
        if not isinstance(pos, np.ndarray) or pos.shape != (2,):
            raise ValueError("위치는 2D numpy 배열이어야 합니다.")

        distance = np.linalg.norm(pos - self.hotspot_pos)
        distance = max(distance, Constants.MIN_DISTANCE_TO_HOTSPOT)
        p_tx = self.params.TRANSMIT_POWER_DBM
        path_loss_db = 30.0 + 20 * np.log10(distance)
        signal = p_tx - path_loss_db

        # 라이시안 페이딩 적용
        signal = self.apply_rician_fading(signal)

        if add_noise:
            shadow_fading = np.random.normal(0, self.params.RSSI_SHADOW_STD)
            signal += shadow_fading
            small_scale_fading = np.random.randn() * 0.5
            signal += small_scale_fading

        return max(Constants.MIN_SIGNAL_STRENGTH, signal)


# --------------------------------------------------------------------------
# 3. HomingAlgorithm Class
# --------------------------------------------------------------------------

class HomingAlgorithm:
    """
    관성(Momentum) 적용 하이브리드 알고리즘
    Ttm
    """

    def __init__(self, start_pos: np.ndarray, params: SimParams):
        # --- 공통 변수 초기화 ---
        self.pos, self.waypoint = np.array(start_pos, dtype=float), np.array(start_pos, dtype=float)
        self.path, self.params, self.is_finished = [start_pos.copy()], params, False
        self.state = "SPIRAL"
        self.last_signal, self.stuck_counter = Constants.MIN_SIGNAL_STRENGTH, 0
        self.best_known_pos, self.best_known_signal = self.pos.copy(), Constants.MIN_SIGNAL_STRENGTH
        self.ascent_direction = np.array([1.0, 0.0])
        self.waypoint_count: int = 0
        self.raw_rssi = Constants.MIN_SIGNAL_STRENGTH

        # --- 실제 이동 궤적 기록 변수 ---
        self.true_path = [start_pos.copy()]  # 실제 위치 (GPS 오차 없음)
        self.current_true_pos = start_pos.copy()

        # --- 나선형 탐색 변수 ---
        self.spiral_waypoint = self.pos.copy()
        self.spiral_leg_length, self.spiral_steps_taken, self.spiral_legs_completed = 1, 0, 0
        self.spiral_direction = np.array([1.0, 0.0])

        # --- 샘플링 탈출 변수 ---
        self.escape_points, self.escape_results = [], {}
        self.current_escape_point_index, self.stuck_signal_baseline = 0, Constants.MIN_SIGNAL_STRENGTH

        # --- 지수이동평균(EWMA) 변수 ---
        self.smoothed_rssi = Constants.MIN_SIGNAL_STRENGTH

        # --- 관성(Momentum) 변수 ---
        self.success_streak = 0

    def update_true_position(self, new_true_pos: np.ndarray):
        """드론의 실제 위치 기록, 내부 변수 업데이트"""
        self.current_true_pos = new_true_pos
        self.true_path.append(new_true_pos.copy())

    def get_true_total_distance(self) -> float:
        """실제 이동한 총 거리 (true_path 기반) 계산"""
        return np.sum(np.linalg.norm(np.diff(np.array(self.true_path), axis=0), axis=1)) if len(
            self.true_path) > 1 else 0.0

    def decide_action(self, rssi: float):
        if self.is_finished: return
        self.waypoint_count += 1
        self.raw_rssi = rssi

        if self.smoothed_rssi == Constants.MIN_SIGNAL_STRENGTH:
            self.smoothed_rssi = rssi
        else:
            alpha = self.params.RSSI_SMOOTHING_FACTOR
            self.smoothed_rssi = (alpha * rssi) + ((1 - alpha) * self.smoothed_rssi)
        if self.smoothed_rssi > self.best_known_signal:
            self.best_known_signal, self.best_known_pos = self.smoothed_rssi, self.pos.copy()

        if self.state == "SPIRAL":
            self._execute_spiral(self.raw_rssi)
        elif self.state == "ADAPTIVE_ASCENT":
            self._execute_adaptive_ascent(self.smoothed_rssi)
        elif self.state == "ESCAPING":
            self._execute_escaping(self.smoothed_rssi)

        self.last_signal = self.smoothed_rssi

    def _execute_spiral(self, raw_rssi: float):
        if raw_rssi > self.params.ASCENT_THRESHOLD:
            self.state = "ADAPTIVE_ASCENT"
            self._execute_adaptive_ascent(raw_rssi)
            return
        step_distance = self.params.DIST_FAR
        self.waypoint = self.spiral_waypoint + self.spiral_direction * step_distance
        self.spiral_steps_taken += 1
        if self.spiral_steps_taken >= self.spiral_leg_length:
            self.spiral_steps_taken, self.spiral_legs_completed = 0, self.spiral_legs_completed + 1
            self.spiral_direction = np.array([-self.spiral_direction[1], self.spiral_direction[0]])
            if self.spiral_legs_completed % 2 == 0: self.spiral_leg_length += 1
        self.spiral_waypoint = self.waypoint

    def _execute_adaptive_ascent(self, rssi: float):
        # --- 관성(Momentum) 로직 적용 ---
        if rssi > self.last_signal:
            # 성공 시: 스트리크 증가, stuck 카운터 초기화
            self.success_streak += 1
            self.stuck_counter = 0
        else:
            # 실패 시: 스트리크 초기화, stuck 카운터 증가 및 방향 전환
            self.success_streak = 0
            self.stuck_counter += 1
            rot_matrix = np.array([[np.cos(Constants.ROTATION_ANGLE_RAD), -np.sin(Constants.ROTATION_ANGLE_RAD)],
                                   [np.sin(Constants.ROTATION_ANGLE_RAD), np.cos(Constants.ROTATION_ANGLE_RAD)]])
            self.ascent_direction = np.dot(rot_matrix, self.ascent_direction)

        if self.stuck_counter > self.params.STUCK_THRESHOLD:
            self.success_streak = 0  # 탈출 시 관성 초기화
            self._initiate_escaping(rssi)
            self.state = "ESCAPING"
            return

        # 관성을 이용한 동적 이동 거리 계산
        base_step = self._get_adaptive_distance(rssi)
        momentum_boost = 1.0 + (self.success_streak * self.params.MOMENTUM_FACTOR)
        # 최대 가속 배율 제한
        momentum_boost = min(momentum_boost, self.params.MAX_MOMENTUM_BOOST)

        final_step = base_step * momentum_boost
        self.waypoint = self.pos + self.ascent_direction * final_step

    def _initiate_escaping(self, current_rssi: float):
        self.stuck_signal_baseline = current_rssi
        self.escape_points, self.escape_results = [], {}
        self.current_escape_point_index = 0
        num_samples, radius = self.params.NUM_ESCAPE_SAMPLES, self.params.ESCAPE_SAMPLE_RADIUS

        mean_angle = np.arctan2(self.ascent_direction[1], self.ascent_direction[0])

        angle_std_dev = np.pi / 4

        for _ in range(num_samples):
            angle = np.random.normal(loc=mean_angle, scale=angle_std_dev)

            r = np.random.uniform(0, radius)
            point = self.best_known_pos + np.array([r * np.cos(angle), r * np.sin(angle)])
            self.escape_points.append(point)

        self.waypoint = self.escape_points[0]

    def _execute_escaping(self, rssi: float):
        # 현재 목표(샘플 지점)에 거의 도달했는지 확인
        if np.linalg.norm(self.pos - self.waypoint) < 2.0:

            # 1. 즉각적인 탈출 조건 (현재 샘플이 기준보다 훨씬 좋음)
            if rssi > self.stuck_signal_baseline + 1.0:
                self.ascent_direction = (self.waypoint - self.best_known_pos)
                norm = np.linalg.norm(self.ascent_direction)
                if norm > 0: self.ascent_direction /= norm

                # 현재 위치에서 새 방향으로 즉시 다음 스텝 계산
                step_dist = self._get_adaptive_distance(rssi)
                self.waypoint = self.pos + self.ascent_direction * step_dist

                self.state, self.stuck_counter = "ADAPTIVE_ASCENT", 0
                return

            # 2. 마지막 샘플 지점 도달 시 (모든 샘플 방문 완료)
            if self.current_escape_point_index >= len(self.escape_points) - 1:
                # 마지막 지점 결과 저장
                self.escape_results[self.current_escape_point_index] = rssi

                best_point_idx = -1
                best_rssi = Constants.MIN_SIGNAL_STRENGTH

                if self.escape_results:
                    # 방문했던 모든 샘플 중 최고의 지점 인덱스와 RSSI 찾기
                    best_point_idx = max(self.escape_results, key=self.escape_results.get)
                    best_rssi = self.escape_results[best_point_idx]

                # 3. 샘플링 결과 분석 및 새 방향 설정
                # 찾은 최고 신호가 기존 신호(stuck_signal_baseline)보다 유의미하게 좋다면
                if best_point_idx != -1 and best_rssi > self.stuck_signal_baseline + 0.5:
                    # 가장 좋았던 지점의 위치
                    best_point_pos = self.escape_points[best_point_idx]

                    # '가장 잘 알려진 위치'에서 '가장 좋았던 샘플 지점'으로의 방향을 새 방향으로 설정
                    self.ascent_direction = (best_point_pos - self.best_known_pos)
                    norm = np.linalg.norm(self.ascent_direction)
                    if norm > 0:
                        self.ascent_direction /= norm

                    # 현재 위치(self.pos)에서 새 방향으로 즉시 다음 waypoint 설정
                    step_dist = self._get_adaptive_distance(best_rssi)  # 가장 좋았던 신호 기준으로 스텝 결정
                    self.waypoint = self.pos + self.ascent_direction * step_dist

                else:
                    # 탈출 실패: 샘플링에서 더 나은 지점을 찾지 못함
                    # 최선책으로, 이전에 가장 신호가 좋았던 'best_known_pos'로 복귀
                    self.waypoint = self.best_known_pos

                self.state, self.stuck_counter = "ADAPTIVE_ASCENT", 0
                return

            # 4. 다음 샘플 지점으로 이동 (아직 샘플링 진행 중)
            self.escape_results[self.current_escape_point_index] = rssi  # 현재 지점 결과 저장
            self.current_escape_point_index += 1
            self.waypoint = self.escape_points[self.current_escape_point_index]

    def _get_adaptive_distance(self, signal: float) -> float:
        if signal > self.params.SIGNAL_PINPOINT: return self.params.DIST_PINPOINT
        if signal > self.params.SIGNAL_NEAR: return self.params.DIST_NEAR
        if signal > self.params.SIGNAL_MID: return self.params.DIST_MID
        return self.params.DIST_FAR

    def update_position(self, new_pos: np.ndarray):
        self.pos = new_pos
        self.path.append(new_pos.copy())

    def get_total_distance(self) -> float:
        return np.sum(np.linalg.norm(np.diff(np.array(self.path), axis=0), axis=1)) if len(self.path) > 1 else 0.0


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

        # 배속 선택
        print("\n배속을 선택하세요:")
        print(" [1] 0.5배속 (느림)")
        print(" [2] 1.0배속 (실시간)")
        print(" [3] 최대속 (가장 빠름)")
        speed_choice = input("선택 [기본값: 2]: ").strip()

        speed_map = {'1': 0.5, '2': 1.0, '3': 100.0}  # 100배 = 최대한 빠르게
        time_scale = speed_map.get(speed_choice, 1.0)

        speed_names = {'0.5': '0.5배속', '1.0': '1.0배속', '100.0': '최대속'}
        print(f"{speed_names.get(str(time_scale), '1.0배속')}로 시작합니다...")

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
