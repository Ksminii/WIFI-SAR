import numpy as np
import math
import time
import matplotlib.pyplot as plt
from dataclasses import dataclass, field
import warnings

warnings.filterwarnings('ignore')  # 모든 경고 차단


# ==========================================================================
# algo_ver3_upgrade
# 상하좌우 탐색 + 신호 좋으면 그쪽으로 더 감 (네 방향 탐색 생략)
# ==========================================================================


# --------------------------------------------------------------------------
# 1. 상수 및 데이터 클래스 정의
# --------------------------------------------------------------------------
@dataclass
class Constants:
    INITIAL_DISTANCE: float = 100.0
    MIN_SIGNAL_STRENGTH: float = -100.0
    MIN_DISTANCE_TO_HOTSPOT: float = 1.0

@dataclass
class SimParams:
    TRANSMIT_POWER_DBM: float = 15.0

    # --- [알고리즘 파라미터] ---
    # 신호 강도에 따라 적용할 이동 보폭 (Step Size)
    STEP_FAR: float = 30.0  # 신호 약함 (< -55): 30m 이동
    STEP_MID: float = 15.0  # 신호 중간: 15m 이동
    STEP_NEAR: float = 5.0  # 신호 강함 (> -45): 5m 이동

    # 탐색을 위해 실제로 갔다 오는 거리 (Radius)
    SCAN_RADIUS_FAR: float = 30.0  # 멀 때는 20m씩 왕복
    SCAN_RADIUS_MID: float = 15.0  # 중간엔 10m씩 왕복
    SCAN_RADIUS_NEAR: float = 8.0  # 가까우면 5m씩 왕복

    # 신호 강도 기준점
    SIGNAL_MID: float = -50.0
    SIGNAL_NEAR: float = -40.0
    SIGNAL_PINPOINT: float = -35.0
    # ------------------------

    # --- 시뮬레이터 ---
    GPS_DRIFT_FACTOR: float = 0.8  # (0~1) GPS 오차의 표류 강도. 0이면 매번 독립적인 오차(표류 없음), 1에 가까울수록 오차가 천천히 변함.
    ROTATION_PENALTY_TIME: float = 1.5
    DRONE_SPEED: float = 8.0
    RSSI_SCAN_TIME: float = 0.0
    TIME_LIMIT: float = 100000.0
    GPS_ERROR_STD: float = 3.0  # [최적화] 8.0 → 3.0m (드론용 RTK GPS)
    RSSI_SHADOW_STD: float = 1.0
    SENSOR_DELAY_MEAN: float = 0.12
    SENSOR_DELAY_STD: float = 0.02
    SENSOR_ERROR_STD: float = 1.2
    # ------------------------


    # --- 페이딩 모델 파라미터 ---
    ENABLE_FADING: bool = True
    RICIAN_K_FACTOR: float = 6.0  # 라이시안 K 팩터 (dB)
    # K > 10: LOS (페이딩 거의 없음)
    # K = 3~10 dB: LOS + 산란 혼합
    # K ≈ 0.01 dB: 레이리 페이딩 (NLOS)
    # ------------------------


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
# 3. HomingAlgorithm Class (Backtracking: 실패 시 원점으로 복귀)
# --------------------------------------------------------------------------
class HomingAlgorithm:
    def __init__(self, start_pos: np.ndarray, params: SimParams):
        self.pos = np.array(start_pos, dtype=float)
        self.waypoint = np.array(start_pos, dtype=float)
        self.path = [start_pos.copy()]
        self.params = params
        self.is_finished = False
        # [New] 실패한 지점 데이터 기억용
        self.fail_rssi = None  # 실패한 곳의 RSSI
        self.fail_dir_name = None  # 실패한 방향 이름

        # 상태: INIT -> SCANNING -> MOVING_TO_NEXT -> (RETURNING_TO_BEST) -> ...
        self.state = "INIT"
        self.center_pos = start_pos.copy()

        # 큐 & 결과 저장
        self.scan_queue = []
        self.scan_results = {}

        # 이전 정보
        self.skip_dir = None
        self.prev_center_rssi = -999.0
        self.last_move_dir_name = None

        # 벡터
        self.dirs = {
            'right': np.array([1.0, 0.0]),
            'left': np.array([-1.0, 0.0]),
            'up': np.array([0.0, 1.0]),
            'down': np.array([0.0, -1.0])
        }

        # 통계
        self.waypoint_count = 0
        self.smoothed_rssi = Constants.MIN_SIGNAL_STRENGTH
        self.true_path = [start_pos.copy()]
        self.current_true_pos = start_pos.copy()

        # 파라미터
        self.current_step = self.params.STEP_FAR
        self.current_radius = self.params.SCAN_RADIUS_FAR

    # --- 기본 메서드 ---
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

    # ------------------

    def decide_action(self, rssi: float):
        if self.is_finished: return
        self.smoothed_rssi = rssi

        dist = np.linalg.norm(self.pos - self.waypoint)
        arrived = dist < 2.5

        # --- 로직 ---

        # [1] 초기 탐색 준비
        if self.state == "INIT":
            self.prev_center_rssi = rssi
            self.center_pos = self.pos.copy()
            self.scan_results = {}

            # 거리 설정
            if rssi < self.params.SIGNAL_MID:
                self.current_step = self.params.STEP_FAR
                self.current_radius = self.params.SCAN_RADIUS_FAR
            elif rssi < self.params.SIGNAL_NEAR:
                self.current_step = self.params.STEP_MID
                self.current_radius = self.params.SCAN_RADIUS_MID
            else:
                self.current_step = self.params.STEP_NEAR
                self.current_radius = self.params.SCAN_RADIUS_NEAR

            # [수정됨] 큐 생성 로직
            full_order = ['right', 'left', 'up', 'down']
            self.scan_queue = []

            for d in full_order:
                if d == self.skip_dir:
                    self.scan_results[d] = -999.0
                elif d == self.fail_dir_name:
                    # 몸은 가지 않고, 아까 잰 값(fail_rssi)을 그대로 재활용
                    self.scan_results[d] = self.fail_rssi
                else:
                    self.scan_queue.append(d)

            self.fail_dir_name = None
            self.fail_rssi = None

            if self.scan_queue:
                first_dir = self.scan_queue[0]
                target = self.center_pos + self.dirs[first_dir] * self.current_radius
                self.waypoint = target
                self.state = "SCANNING"
                self.waypoint_count += 1
            else:
                self.is_finished = True


        # [2] 주변 찌르기
        elif self.state == "SCANNING":
            if arrived:
                if self.scan_queue:
                    done_dir = self.scan_queue.pop(0)
                    self.scan_results[done_dir] = rssi

                if self.scan_queue:
                    next_dir = self.scan_queue[0]
                    target = self.center_pos + self.dirs[next_dir] * self.current_radius
                    self.waypoint = target
                    self.waypoint_count += 1
                else:
                    best_dir_name = max(self.scan_results, key=self.scan_results.get)
                    best_vec = self.dirs[best_dir_name]
                    next_center = self.center_pos + best_vec * self.current_step

                    self.waypoint = next_center

                    reverse_map = {'right': 'left', 'left': 'right', 'up': 'down', 'down': 'up'}
                    self.skip_dir = reverse_map[best_dir_name]
                    self.last_move_dir_name = best_dir_name

                    self.state = "MOVING_TO_NEXT"
                    self.waypoint_count += 1


        # [3] 중심으로 이동 중 (직진 구간)
        elif self.state == "MOVING_TO_NEXT":
            if arrived:
                # Trust Check: 신호가 좋아졌는가?
                # 노이즈 고려 (-1.0dB 까지는 봐줌)
                if rssi >= self.prev_center_rssi - 1.0:
                    # [성공] 신호 좋음 -> 계속 직진 (탐색 생략)
                    self.prev_center_rssi = rssi
                    self.center_pos = self.pos.copy()  # 현재 위치를 새 베이스캠프로 확정

                    move_vec = self.dirs[self.last_move_dir_name]

                    # 보폭 설정
                    if rssi > self.params.SIGNAL_PINPOINT:
                        step = self.params.STEP_NEAR
                    elif rssi > self.params.SIGNAL_NEAR:
                        step = self.params.STEP_MID
                    else:
                        step = self.params.STEP_FAR

                    next_center = self.center_pos + move_vec * (step * 1.2)
                    self.waypoint = next_center
                    self.state = "MOVING_TO_NEXT"
                    self.waypoint_count += 1

                else:
                    # [실패] 신호가 나빠짐
                    # 현재 위치(나쁜 곳)에서 탐색하지 않고,
                    # 저장해둔 이전 베이스캠프(self.center_pos)로 복귀 명령
                    self.fail_rssi = rssi
                    self.fail_dir_name = self.last_move_dir_name  # 방금 온 방향
                    # 복귀 웨이포인트 설정 (이전 중심점)
                    # self.center_pos는 아직 업데이트되지 않았으므로 '이전 좋은 위치'임
                    self.waypoint = self.center_pos

                    self.state = "RETURNING_TO_BEST"
                    self.waypoint_count += 1


        # [4] 복귀 중 (Backtracking)
        elif self.state == "RETURNING_TO_BEST":
            if arrived:
                # 안전하게 베이스캠프 복귀 완료
                # 이제 여기서부터 다시 정석대로 4방향 탐색 시작
                self.state = "INIT"
                self.skip_dir = None

# --------------------------------------------------------------------------
# 4. 시각화 클래스
# --------------------------------------------------------------------------
class SimulationVisualizer:
    def __init__(self, time_scale: float = 1.0, view_radius_m: float = 50.0):
        """
        time_scale: 시뮬레이션 속도 조절 (1.0 = 실시간, 0.5 = 절반 속도, 2.0 = 2배 속도, 5.0 = 5배 속도)
        """
        plt.ion()
        self.fig, self.ax = plt.subplots(figsize=(10, 10))
        self.start_real_time = None
        self.start_simulation_time = None
        self.time_scale = time_scale
        self.view_radius = view_radius_m
        print(f"Visualizer initialized with time_scale = {self.time_scale}")  # 디버그용
        self.path_history = []  # 경로 히스토리 누적
        self.last_real_time = None
        self.last_sim_time = None

    def update(self, env: SimulationEnvironment, true_pos: np.ndarray, reported_pos: np.ndarray, simulation_time: float,
               current_rssi: float, status: str = "Moving", algo_state: str = "INIT", next_waypoint: np.ndarray = None):
        # 첫 업데이트일 때 시작 시간 초기화
        if self.start_real_time is None:
            self.start_real_time = time.time()
            self.start_simulation_time = simulation_time

        # 배속 적용: 시뮬레이션 시작 이후 경과 시간 기반으로 계산
        sim_elapsed_total = simulation_time - self.start_simulation_time
        target_real_elapsed_total = sim_elapsed_total / self.time_scale  # ← 배속 적용

        # 현재까지 실제로 경과한 시간
        current_real_time = time.time()
        actual_real_elapsed_total = current_real_time - self.start_real_time

        # 필요한 만큼 대기 (전체 기준으로 동기화)
        sleep_time = target_real_elapsed_total - actual_real_elapsed_total
        if sleep_time > 0.001:  # 1ms 이상만 sleep
            time.sleep(sleep_time)

        # 경로 히스토리에 현재 위치 추가
        if len(self.path_history) == 0 or np.linalg.norm(true_pos - np.array(self.path_history[-1])) > 0.1:
            self.path_history.append(true_pos.copy())

        # 그래프 업데이트 (간단하고 빠르게)
        self.ax.clear()

        # 누적된 경로 표시
        if len(self.path_history) > 1:
            path_array = np.array(self.path_history)
            self.ax.plot(path_array[:, 0], path_array[:, 1], 'c-', alpha=0.6, linewidth=1, label='Path')

        # 드론 현재 위치 (실제 위치)
        self.ax.plot(true_pos[0], true_pos[1], 'bo', markersize=14, label='Drone (True Pos)', zorder=5)

        # 드론 인식 위치 (GPS 오차 포함) 표시
        if reported_pos is not None:
            self.ax.plot(reported_pos[0], reported_pos[1], 'bx', markersize=10, alpha=0.7, label='Drone (Reported GPS)',
                         zorder=4)

        # Next Waypoint 표시
        if next_waypoint is not None:
            self.ax.plot(next_waypoint[0], next_waypoint[1], 'go', markersize=10, mfc='none', mew=2,
                         label='Next Waypoint', zorder=3)
            self.ax.plot([true_pos[0], next_waypoint[0]], [true_pos[1], next_waypoint[1]], 'y--', alpha=0.7,
                         linewidth=1.5)

        # 목표 (hotspot)
        self.ax.plot(env.hotspot_pos[0], env.hotspot_pos[1], 'r*', markersize=20, label='Hotspot (Goal)', zorder=4)

        # 드론에서 목표까지 선 그리기
        self.ax.plot([true_pos[0], env.hotspot_pos[0]], [true_pos[1], env.hotspot_pos[1]], 'g--', alpha=0.3,
                     linewidth=1)

        distance_to_goal = np.linalg.norm(true_pos - env.hotspot_pos)
        speed_display = f"x{self.time_scale:.1f}" if self.time_scale != 1.0 else "1x"

        title_line_1 = f"[{algo_state} | {status}]"
        title_line_2 = f"Time: {simulation_time:.1f}s | Dist: {distance_to_goal:.1f}m | RSSI: {current_rssi:.1f} dBm | Speed: {speed_display}"

        self.ax.set_title(title_line_1 + "\n" + title_line_2)
        self.ax.set_xlabel("X (meters)")
        self.ax.set_ylabel("Y (meters)")
        self.ax.grid(True, which='both', linestyle='--', linewidth=0.5, alpha=0.3)
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

        # 최소한의 pause - 화면 업데이트만 하고 지연 최소화
        plt.pause(0.00001)  # 거의 무시할 수 있는 수준

    def close(self):
        plt.ioff()
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
        waypoints_at_threshold_pass, threshold_passed = 0, False

        # --- 성공 시점의 RSSI 기록 ---
        rssi_at_success: float = 0.0

        # --- 99.5% percentile 기반 동적 성공 임계값 계산
        # Monte Carlo 검증된 임계값 사용 (10,000 samples)
        current_std = self.params.RSSI_SHADOW_STD

        # 99.5% percentile 임계값 매핑 (Monte Carlo 재계산 결과)
        if self.params.RICIAN_K_FACTOR == 0.0:  # Rayleigh fading (K=0)
            threshold_map = {
                1.0: -26.7,
                3.0: -24.0,
                5.0: -20.2
            }
        else:  # Rician fading (K=6)
            threshold_map = {
                1.0: -28.8,
                3.0: -25.7,
                5.0: -21.2
            }

        # 정확한 매핑이 있으면 사용, 없으면 보간
        if current_std in threshold_map:
            success_threshold_dbm = threshold_map[current_std]
        else:
            # 선형 보간
            keys = sorted(threshold_map.keys())
            if current_std < keys[0]:
                success_threshold_dbm = threshold_map[keys[0]]
            elif current_std > keys[-1]:
                success_threshold_dbm = threshold_map[keys[-1]]
            else:
                for i in range(len(keys) - 1):
                    if keys[i] <= current_std <= keys[i + 1]:
                        # 선형 보간
                        ratio = (current_std - keys[i]) / (keys[i + 1] - keys[i])
                        success_threshold_dbm = (threshold_map[keys[i]] +
                                                 ratio * (threshold_map[keys[i + 1]] - threshold_map[keys[i]]))
                        break

        # 시각화 모드일 때 계산된 임계값 출력 (확인용)
        if visualizer:
            print(f"\nSimulation with RSSI_SHADOW_STD = {current_std:.1f}")
            # Rician/Rayleigh 상태 표시 추가
            k_type = "Rayleigh (K=0)" if self.params.RICIAN_K_FACTOR == 0.0 else f"Rician (K={self.params.RICIAN_K_FACTOR})"
            print(f"Fading Model: {k_type}")
            print(f"Success Threshold set to: {success_threshold_dbm:.2f} dBm")

        simulation_time = 0.0
        loop_count = 0

        while simulation_time < self.params.TIME_LIMIT:
            loop_count += 1
            new_random_error = np.random.normal(0, self.params.GPS_ERROR_STD, 2)
            drift_factor = self.params.GPS_DRIFT_FACTOR
            gps_error = drift_factor * previous_gps_error + (1 - drift_factor) * new_random_error
            previous_gps_error = gps_error

            reported_pos = true_pos + gps_error
            algo.update_position(reported_pos)

            sensor_delay = max(0.0, np.random.normal(self.params.SENSOR_DELAY_MEAN, self.params.SENSOR_DELAY_STD))
            sensor_error = np.random.normal(0, self.params.SENSOR_ERROR_STD)
            current_rssi = env.get_signal(true_pos) + sensor_error
            simulation_time += sensor_delay

            algo.decide_action(current_rssi)
            # 알고리즘의 현재 상태를 기반으로 시각화에 표시할 상세 상태 문자열 생성
            current_algo_state_str = algo.state
            current_algo_state_str = algo.state

            if algo.state == "FINISHED_SUCCESS":
                current_algo_state_str = "SUCCESS"

            #  알고리즘 상태 디버그 정보
            if visualizer and loop_count <= 10:  # 'visualizer'일 때만 출력하도록 조건 추가
                print(
                    f"[Loop {loop_count}] State: {current_algo_state_str} | RSSI: {current_rssi:.2f} dBm (Smoothed: {algo.smoothed_rssi:.2f}) | Waypoint: ({algo.waypoint[0]:.1f}, {algo.waypoint[1]:.1f})")

            # waypoint 계산 중 상태 표시
            if visualizer:
                # [수정] reported_pos를 visualizer에 전달
                visualizer.update(env, true_pos, reported_pos, simulation_time, current_rssi,
                                  status="Computing Waypoint",
                                  algo_state=current_algo_state_str, next_waypoint=algo.waypoint)

            if not algo.is_finished and algo.smoothed_rssi >= success_threshold_dbm:
                algo.is_finished = True
                algo.state = "FINISHED_SUCCESS"
                # 성공 시점의 RSSI 값 체크 (평활화된 값 기준)
                rssi_at_success = algo.smoothed_rssi

            # 이동 벡터 계산 (알고리즘이 인식하는 reported_pos 기준)
            move_vector = algo.waypoint - reported_pos
            move_distance = np.linalg.norm(move_vector)

            if visualizer and loop_count <= 10:
                print(
                    f"[Loop {loop_count}] reported_pos: ({reported_pos[0]:.1f}, {reported_pos[1]:.1f}) | waypoint: ({algo.waypoint[0]:.1f}, {algo.waypoint[1]:.1f}) | move_vector: ({move_vector[0]:.1f}, {move_vector[1]:.1f})")

            move_direction = move_vector / move_distance if move_distance > 0 else None

            rotation_penalty = 0.0
            if previous_direction is not None and move_distance > 0:
                angle_change = np.arccos(np.clip(np.dot(previous_direction, move_direction), -1.0, 1.0))
                if angle_change > np.deg2rad(10):
                    rotation_penalty = self.params.ROTATION_PENALTY_TIME
            previous_direction = move_direction

            time_to_travel = move_distance / self.params.DRONE_SPEED
            num_steps = max(1, int(time_to_travel / 0.05))  # 50ms마다 업데이트
            step_size = move_distance / num_steps

            for step in range(num_steps):
                if move_distance > 0:
                    true_pos += (move_vector / move_distance) * step_size
                else:
                    true_pos += move_vector

                algo.update_true_position(true_pos)

                new_random_error = np.random.normal(0, self.params.GPS_ERROR_STD, 2)
                drift_factor = self.params.GPS_DRIFT_FACTOR
                gps_error = drift_factor * previous_gps_error + (1 - drift_factor) * new_random_error
                previous_gps_error = gps_error
                reported_pos = true_pos + gps_error

                simulation_time += time_to_travel / num_steps

                if visualizer:
                    visualizer.update(env, true_pos, reported_pos, simulation_time, current_rssi, status="Moving",
                                      algo_state=current_algo_state_str, next_waypoint=algo.waypoint)

                if algo.is_finished:
                    break

            # RSSI_SCAN_TIME과 rotation_penalty를 프레임으로 나누어 표시
            pause_time = self.params.RSSI_SCAN_TIME + rotation_penalty
            if pause_time > 0:
                num_pause_frames = max(1, int(pause_time / 0.05))
                frame_time = pause_time / num_pause_frames
                for _ in range(num_pause_frames):
                    simulation_time += frame_time
                    if visualizer:
                        visualizer.update(env, true_pos, reported_pos, simulation_time, current_rssi,
                                          status="Scanning RSSI",
                                          algo_state=current_algo_state_str, next_waypoint=algo.waypoint)

            if algo.is_finished:
                break

        # 탐색 완료 시 최종 화면 표시
        if visualizer and algo.is_finished:
            visualizer.update(env, true_pos, reported_pos, simulation_time, current_rssi, status="SUCCESS",
                              algo_state=current_algo_state_str, next_waypoint=algo.waypoint)
            import time as time_module
            time_module.sleep(2)  # 2초간 최종 화면 표시

        reason = "Success" if algo.is_finished else "Timeout"
        final_distance = np.linalg.norm(true_pos - env.hotspot_pos)

        return SimResult(
            success=algo.is_finished,
            final_distance=final_distance,
            total_travel=algo.get_total_distance(),
            true_total_travel=algo.get_true_total_distance(),
            search_time=simulation_time,
            reason=reason,
            waypoint_count=algo.waypoint_count,
            waypoints_at_threshold=waypoints_at_threshold_pass,
            rssi_at_success=rssi_at_success
        )

    def run_multiple(self, num_simulations: int = 1000):
        print(f"Starting {num_simulations} simulations...")
        results = []
        start_time = time.time()

        for i in range(num_simulations):
            if (i + 1) % 10 == 0:
                if (i + 1) % 100 == 0:
                    print(f"\rProgress: {i + 1}/{num_simulations}", end="")
            results.append(self.run_single())

        end_time = time.time()
        print(f"\nTotal execution time: {end_time - start_time:.2f} seconds")
        self._analyze_results(results)

    def _analyze_results(self, results: list[SimResult]):
        successful_runs = [r for r in results if r.success]
        success_count = len(successful_runs)
        num_simulations = len(results)

        print("\n--- Final Statistical Analysis ---")
        print(f"Success Rate: {success_count / num_simulations * 100:.1f}%")

        if successful_runs:
            # 데이터 추출
            success_waypoint_counts = [r.waypoint_count for r in successful_runs]
            success_rssi_values = [r.rssi_at_success for r in successful_runs if r.rssi_at_success != 0.0]  # 0.0은 초기값
            total_travels = [r.total_travel for r in successful_runs]
            true_total_travels = [r.true_total_travel for r in successful_runs]
            search_times = [r.search_time for r in successful_runs]

            # 통계 출력
            print(f"\n--- Stats on Successful Runs ({len(successful_runs)} runs) ---")
            print(
                f"Avg Waypoints Generated: {np.mean(success_waypoint_counts):.1f} (Std: {np.std(success_waypoint_counts):.1f})")
            if success_rssi_values:
                print(
                    f"Avg RSSI at Success Moment: {np.mean(success_rssi_values):.2f} dBm (Std: {np.std(success_rssi_values):.2f} dBm)")
            else:
                print("Avg RSSI at Success Moment: N/A (No successful runs recorded RSSI)")
            print(f"Avg Total Distance Traveled: {np.mean(total_travels):.2f} m (Std: {np.std(total_travels):.2f} m)")
            print(f"Avg Total Distance Traveled (True Pos): {np.mean(true_total_travels):.2f} m (Std: {np.std(true_total_travels):.2f} m)")
            print(f"Avg Search Time: {np.mean(search_times):.2f} s (Std: {np.std(search_times):.2f} s)")

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
        visualizer = SimulationVisualizer(time_scale=time_scale, view_radius_m=120.0)
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
