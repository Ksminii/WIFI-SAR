/**
 * ========================================
 * 드론 4방향 신호 탐색 및 이동 (C++ 버전)
 * ========================================
 *
 * [실행 환경]
 * - 하드웨어: 라즈베리파이 + 픽스호크
 * - 연결: serial:///dev/ttyAMA0:57600
 * - MAVSDK 라이브러리 필요
 *
 * [주요 기능]
 * 1. 4방향 신호 탐색 (북, 동, 남, 서)
 * 2. tshark를 이용한 정밀 RSSI 측정
 * 3. 각 방향 측정 후 원위치 복귀 (효율적)
 * 4. 최고 신호 방향으로 자동 이동
 * 5. 반복 탐색
 *
 * [simple.cpp와의 차이점]
 * - simple.cpp: 5방향 (중앙 포함), 6회 이동
 * - simple_search.cpp: 4방향, 8회 이동 (각 방향마다 복귀)
 *
 * [컴파일]
 *   g++ -std=c++17 simple_search.cpp -o simple_search -lmavsdk
 *
 * [사용법]
 *   ./simple_search serial:///dev/ttyAMA0:57600  (실제 드론)
 *   ./simple_search udp://:14540                 (SITL 테스트)
 *
 * [설정 변수]
 * - TARGET_SSID: 탐색할 Wi-Fi SSID
 * - MONITOR_INTERFACE: Wi-Fi 모니터 인터페이스
 * - SEARCH_DISTANCE: 탐색 거리 (기본 10m)
 * - MOVE_MULTIPLIER: 이동 배수 (기본 2.0)
 * - MAX_ROUNDS: 최대 탐색 라운드 (기본 5회)
 *
 * [참고]
 * - Python 버전: simple_search.py
 * - 5방향 탐색을 원하면 simple.cpp 사용 권장
 * ========================================
 */

#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include <mavsdk/plugins/offboard/offboard.h>
#include <mavsdk/plugins/action/action.h>
#include <iostream>
#include <string>
#include <vector>
#include <thread>
#include <chrono>
#include <cmath>
#include <sstream>
#include <cstdlib>
#include <algorithm>
#include <numeric>
#include <map>

using namespace mavsdk;
using namespace std::chrono;
using namespace std::this_thread;

// ========================================
// 스크립트 설정
// ========================================
// [필수] 탐색할 실종자 디바이스의 Wi-Fi SSID
const std::string TARGET_SSID = "Victim_Phone_WiFi";

// [필수] tshark가 사용할 모니터 모드 인터페이스
const std::string MONITOR_INTERFACE = "wlan0mon";

// 탐색 및 이동 관련 설정
const float SEARCH_DISTANCE = 10.0f;  // 4방향 탐색 시 이동할 거리 (미터)
const float MOVE_MULTIPLIER = 2.0f;   // 최적 방향으로 이동 시, 탐색 거리의 배수
const int MAX_ROUNDS = 5;             // 최대 탐색 라운드
const float RSSI_THRESHOLD_STOP = -20.0f;  // 이 신호 강도(dBm) 이상이면 탐색 중지
const int TSHARK_DURATION = 3;        // tshark 측정 시간 (초)

// ========================================
// RSSI 측정 유틸리티
// ========================================

/**
 * tshark를 사용한 정밀 RSSI 측정
 */
float get_signal_strength(const std::string& target_ssid,
                          const std::string& iface = "wlan0mon",
                          int duration = 3) {
    if (target_ssid.empty()) {
        std::cerr << "[오류] get_signal_strength: target_ssid가 지정되지 않았습니다." << std::endl;
        return -999.0f;
    }

    // tshark로 정밀 측정
    std::stringstream cmd;
    cmd << "tshark -i " << iface
        << " -a duration:" << duration
        << " -Y 'wlan.fc.type_subtype == 0x08'"  // 비컨 프레임
        << " -T fields -E separator=, -e wlan.ssid -e radiotap.dbm_antsignal"
        << " 2>/dev/null";

    FILE* pipe = popen(cmd.str().c_str(), "r");
    if (!pipe) {
        std::cerr << "[오류] tshark 실행 실패" << std::endl;
        return -999.0f;
    }

    std::vector<float> samples;
    char buffer[256];

    // SSID 후보 생성 (대소문자 무시)
    std::string target_lower = target_ssid;
    std::transform(target_lower.begin(), target_lower.end(), target_lower.begin(), ::tolower);

    while (fgets(buffer, sizeof(buffer), pipe) != nullptr) {
        std::string line(buffer);
        size_t comma_pos = line.find(',');

        if (comma_pos != std::string::npos) {
            std::string ssid = line.substr(0, comma_pos);
            std::string rssi_str = line.substr(comma_pos + 1);

            // SSID 정규화 및 매칭 (공백/널 제거, 대소문자 무시)
            ssid.erase(0, ssid.find_first_not_of(" \t\n\r\0"));
            ssid.erase(ssid.find_last_not_of(" \t\n\r\0") + 1);
            std::transform(ssid.begin(), ssid.end(), ssid.begin(), ::tolower);

            if (ssid == target_lower || ssid.find(target_lower) != std::string::npos) {
                try {
                    float rssi = std::stof(rssi_str);
                    samples.push_back(rssi);
                } catch (...) {
                    continue;
                }
            }
        }
    }
    pclose(pipe);

    // 평균 계산 (outlier 제거)
    if (samples.empty()) {
        return -999.0f;
    }

    int n = samples.size();
    if (n > 15) {
        // 상하위 10% 제거
        std::sort(samples.begin(), samples.end());
        int k = std::max(1, static_cast<int>(n * 0.1));
        float sum = 0.0f;
        int count = 0;
        for (int i = k; i < n - k; i++) {
            sum += samples[i];
            count++;
        }
        return count > 0 ? sum / count : -999.0f;
    } else {
        return std::accumulate(samples.begin(), samples.end(), 0.0f) / n;
    }
}

// ========================================
// NED 좌표 기반 이동
// ========================================

/**
 * NED 로컬 좌표로 이동
 * MAVSDK Offboard는 자동으로 위치 명령을 지속 전송 (2Hz 이상)
 */
bool goto_position_ned(std::shared_ptr<Offboard> offboard,
                       std::shared_ptr<Telemetry> telemetry,
                       float north, float east, float down,
                       int timeout_sec = 60) {

    std::cout << "목표 NED: (" << north << ", " << east << ", " << down << ")" << std::endl;

    // Offboard 모드로 위치 명령 전송 (MAVSDK가 자동으로 지속 전송)
    Offboard::PositionNedYaw target{};
    target.north_m = north;
    target.east_m = east;
    target.down_m = down;
    target.yaw_deg = 0.0f;  // 방향 유지

    offboard->set_position_ned(target);

    auto start_time = steady_clock::now();

    while (duration_cast<seconds>(steady_clock::now() - start_time).count() < timeout_sec) {
        // 현재 NED 좌표
        auto position = telemetry->position_velocity_ned();
        float current_north = position.position.north_m;
        float current_east = position.position.east_m;
        float current_down = position.position.down_m;

        // 목표까지 거리 계산
        float distance = std::sqrt(std::pow(north - current_north, 2) +
                                   std::pow(east - current_east, 2));

        std::cout << " 목표까지 " << distance << "m | 속도: " << telemetry->velocity_ned().forward_m_s << "m/s | "
                  << "NED: (" << current_north << ", " << current_east << ", " << current_down << ")"
                  << std::endl;

        // 2m 이내 도착
        if (distance < 2.0f) {
            std::cout << "도착" << std::endl;
            return true;
        }

        sleep_for(milliseconds(500));  // 0.5초 간격 체크
    }

    std::cout << " 타임아웃" << std::endl;
    return false;
}

// ========================================
// 4방향 신호 탐색
// ========================================

struct SearchResult {
    std::string direction;
    float signal;
    float dNorth;
    float dEast;
};

/**
 * 북/동/남/서 4방향 탐색
 */
SearchResult search_four_directions(std::shared_ptr<Offboard> offboard,
                                    std::shared_ptr<Telemetry> telemetry,
                                    float search_distance = 10.0f,
                                    const std::string& target_ssid = "Victim_Phone_WiFi") {

    std::cout << "\n============================================================" << std::endl;
    std::cout << "4방향 신호 탐색 시작 (탐색 거리: " << search_distance << "m)" << std::endl;
    std::cout << "============================================================\n" << std::endl;

    // EKF(로컬 좌표)가 준비될 때까지 안전 대기
    std::cout << "로컬 좌표계(EKF) 준비 대기 중..." << std::endl;
    auto pos = telemetry->position_velocity_ned();
    while (std::isnan(pos.position.north_m) || std::isnan(pos.position.east_m)) {
        sleep_for(seconds(1));
        std::cout << "... EKF 대기 ..." << std::endl;
        pos = telemetry->position_velocity_ned();
    }

    // 시작 위치 저장 (NED 로컬 좌표)
    auto home_pos = telemetry->position_velocity_ned();
    float home_north = home_pos.position.north_m;
    float home_east = home_pos.position.east_m;
    float home_down = home_pos.position.down_m;

    std::cout << "시작 위치 (NED):" << std::endl;
    std::cout << "  North: " << home_north << "m" << std::endl;
    std::cout << "  East: " << home_east << "m" << std::endl;
    std::cout << "  Down: " << home_down << "m\n" << std::endl;

    std::map<std::string, float> signal_results;

    // 신호 측정 헬퍼 함수
    auto measure_signal = [&](const std::string& point_name) {
        std::cout << "\n--------------------------------------------------" << std::endl;
        std::cout << "[" << point_name << "] 지점 신호 측정 중... (tshark " << TSHARK_DURATION << "초)" << std::endl;
        std::cout << "--------------------------------------------------" << std::endl;

        sleep_for(seconds(2));  // 안정화 대기

        std::vector<float> signals;
        for (int i = 0; i < 3; i++) {
            float sig = get_signal_strength(target_ssid, MONITOR_INTERFACE, TSHARK_DURATION);
            if (sig > -999.0f) {
                signals.push_back(sig);
                std::cout << "  신호 " << (i+1) << ": " << sig << " dBm" << std::endl;
            } else {
                std::cout << "  신호 " << (i+1) << ": 측정 실패 (None)" << std::endl;
            }
            sleep_for(milliseconds(500));
        }

        float avg_signal;
        if (!signals.empty()) {
            avg_signal = std::accumulate(signals.begin(), signals.end(), 0.0f) / signals.size();
            std::cout << "→ 평균: " << avg_signal << " dBm" << std::endl;
        } else {
            std::cout << "⚠️ " << point_name << " 신호 측정 실패 (모두 None)" << std::endl;
            avg_signal = -100.0f;  // 실패 시 최저값 할당
        }

        signal_results[point_name] = avg_signal;
        return avg_signal;
    };

    // 4방향 정의
    struct Direction {
        std::string name;
        float dNorth;
        float dEast;
    };

    std::vector<Direction> directions = {
        {"북쪽", search_distance, 0.0f},
        {"동쪽", 0.0f, search_distance},
        {"남쪽", -search_distance, 0.0f},
        {"서쪽", 0.0f, -search_distance}
    };

    // 각 방향 탐색
    for (const auto& dir : directions) {
        std::cout << "\n" << dir.name << "으로 이동" << std::endl;

        // 목표 위치 계산
        float target_north = home_north + dir.dNorth;
        float target_east = home_east + dir.dEast;

        // 이동
        goto_position_ned(offboard, telemetry, target_north, target_east, home_down, 60);
        measure_signal(dir.name);

        // 원위치 복귀
        std::cout << "원위치 복귀" << std::endl;
        goto_position_ned(offboard, telemetry, home_north, home_east, home_down, 60);
        sleep_for(seconds(2));
    }

    // 결과 출력
    std::cout << "\n============================================================" << std::endl;
    std::cout << "신호 측정 결과:" << std::endl;
    std::cout << "============================================================" << std::endl;

    for (const auto& [direction, signal] : signal_results) {
        int bar_len = static_cast<int>((signal + 100) / 2);
        std::string bar(std::max(0, bar_len), '█');
        std::cout << "  " << direction << ": " << signal << " dBm  " << bar << std::endl;
    }

    // 최적 방향 찾기
    auto best = std::max_element(signal_results.begin(), signal_results.end(),
                                 [](const auto& a, const auto& b) {
                                     return a.second < b.second;
                                 });

    std::string best_direction = best->first;
    float best_signal = best->second;

    std::cout << "\n✓ 최고 신호: " << best_direction << " (" << best_signal << " dBm)" << std::endl;
    std::cout << "============================================================\n" << std::endl;

    // 다음 이동을 위한 방향 벡터
    std::map<std::string, std::pair<float, float>> direction_vectors = {
        {"북쪽", {search_distance, 0.0f}},
        {"동쪽", {0.0f, search_distance}},
        {"남쪽", {-search_distance, 0.0f}},
        {"서쪽", {0.0f, -search_distance}}
    };

    auto [dN, dE] = direction_vectors[best_direction];

    return {best_direction, best_signal, dN, dE};
}

// ========================================
// 메인 실행
// ========================================

int main(int argc, char* argv[]) {
    // 연결 URL (기본값: 시리얼 포트)
    std::string connection_url = "serial:///dev/ttyAMA0:57600";

    if (argc >= 2) {
        connection_url = argv[1];
    }

    Mavsdk mavsdk{Mavsdk::Configuration{ComponentType::GroundStation}};

    std::cout << "연결 중: " << connection_url << std::endl;
    ConnectionResult connection_result = mavsdk.add_any_connection(connection_url);

    if (connection_result != ConnectionResult::Success) {
        std::cerr << "❌ 연결 실패: " << connection_result << std::endl;
        return 1;
    }

    // 시스템 발견 대기
    std::cout << "드론 발견 대기 중..." << std::endl;
    auto prom = std::promise<std::shared_ptr<System>>{};
    auto fut = prom.get_future();

    mavsdk.subscribe_on_new_system([&mavsdk, &prom]() {
        auto system = mavsdk.systems().back();
        if (system->has_autopilot()) {
            prom.set_value(system);
        }
    });

    if (fut.wait_for(seconds(10)) == std::future_status::timeout) {
        std::cerr << "❌ 드론 발견 실패 (타임아웃)" << std::endl;
        return 1;
    }

    auto system = fut.get();
    std::cout << "✓ 드론 발견!" << std::endl;

    // 플러그인 초기화
    auto telemetry = std::make_shared<Telemetry>(system);
    auto offboard = std::make_shared<Offboard>(system);
    auto action = std::make_shared<Action>(system);

    // 드론 정보 출력
    sleep_for(seconds(2));
    std::cout << "\n=== 연결 완료 ===" << std::endl;
    std::cout << "Battery: " << telemetry->battery().remaining_percent * 100 << "%" << std::endl;
    std::cout << "GPS: " << (int)telemetry->gps_info().fix_type << " (위성 "
              << telemetry->gps_info().num_satellites << "개)" << std::endl;
    std::cout << "==================\n" << std::endl;

    std::cout << "============================================================" << std::endl;
    std::cout << "드론이 이륙한 상태인지 확인하세요!" << std::endl;
    std::cout << "수동으로 이륙 후 'GUIDED' 모드로 변경하고" << std::endl;
    std::cout << "Enter를 눌러 탐색을 시작하세요." << std::endl;
    std::cout << "목표 SSID: '" << TARGET_SSID << "'" << std::endl;
    std::cout << "============================================================" << std::endl;
    std::cin.get();

    // Offboard 모드 시작
    std::cout << "Offboard 모드 시작..." << std::endl;
    Offboard::Result offboard_result = offboard->start();
    if (offboard_result != Offboard::Result::Success) {
        std::cerr << "⚠️ Offboard 모드 시작 실패: " << offboard_result << std::endl;
        std::cerr << "GUIDED 모드로 전환하거나 권한을 확인하세요." << std::endl;
    }

    // 신호 탐색 반복
    for (int round_num = 1; round_num <= MAX_ROUNDS; round_num++) {
        std::cout << "\n############################################################" << std::endl;
        std::cout << "###  탐색 라운드 " << round_num << "/" << MAX_ROUNDS << "  ###" << std::endl;
        std::cout << "############################################################" << std::endl;

        // 4방향 탐색
        auto best_result = search_four_directions(offboard, telemetry, SEARCH_DISTANCE, TARGET_SSID);

        // 신호가 매우 강하면 종료
        if (best_result.signal > RSSI_THRESHOLD_STOP) {
            std::cout << "\n신호 강도 충분: " << best_result.signal << " dBm" << std::endl;
            std::cout << "목표 지점 근처 도달! 탐색을 종료합니다." << std::endl;
            break;
        }

        // 최적 방향으로 이동 (N배 거리) - NED 좌표 사용
        float move_distance = std::sqrt(std::pow(best_result.dNorth, 2) +
                                       std::pow(best_result.dEast, 2)) * MOVE_MULTIPLIER;

        std::cout << "\n" << best_result.direction << "으로 " << move_distance << "m 이동 중..." << std::endl;

        // 현재 NED 좌표 (이동의 기준점이 됨)
        auto current_pos = telemetry->position_velocity_ned();
        float current_north = current_pos.position.north_m;
        float current_east = current_pos.position.east_m;
        float current_down = current_pos.position.down_m;

        if (std::isnan(current_north)) {  // 안전장치
            std::cout << "이동 전 로컬 좌표 손실! 라운드 중단" << std::endl;
            continue;
        }

        // 목표 NED 좌표 계산 (현재 위치 + 이동 벡터 * 배수)
        float target_north = current_north + best_result.dNorth * MOVE_MULTIPLIER;
        float target_east = current_east + best_result.dEast * MOVE_MULTIPLIER;

        goto_position_ned(offboard, telemetry, target_north, target_east, current_down, 60);
        std::cout << "✓ " << best_result.direction << " 방향 이동 완료\n" << std::endl;

        // 다음 라운드 전 대기
        if (round_num < MAX_ROUNDS) {
            int wait_time = 3;
            std::cout << "다음 라운드까지 " << wait_time << "초 대기...\n" << std::endl;
            sleep_for(seconds(wait_time));
        }

        if (round_num == MAX_ROUNDS) {
            std::cout << "최대 탐색 라운드 도달. 미션을 종료합니다." << std::endl;
        }
    }

    std::cout << "\n============================================================" << std::endl;
    std::cout << "모든 탐색 완료!" << std::endl;
    std::cout << "============================================================" << std::endl;

    // 자동 착륙
    std::cout << "\n=== 착륙 시작 ===" << std::endl;
    Action::Result land_result = action->land();
    if (land_result != Action::Result::Success) {
        std::cerr << "⚠️ 착륙 명령 실패: " << land_result << std::endl;
    }

    // 착륙 완료 대기
    while (telemetry->armed()) {
        float alt = telemetry->position().relative_altitude_m;
        std::cout << "착륙 중... 고도: " << alt << "m" << std::endl;
        sleep_for(seconds(1));
    }

    std::cout << "✓ 착륙 완료!\n" << std::endl;
    std::cout << "\n✓✓✓ 미션 완료! ✓✓✓\n" << std::endl;

    return 0;
}
