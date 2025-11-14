/**
 * ========================================
 * 드론 4방향 이동 테스트 (C++ 버전 - 신호 측정 없음)
 * ========================================
 *
 * [실행 환경]
 * - 하드웨어: 라즈베리파이 + 픽스호크
 * - 연결: serial:///dev/ttyAMA0:57600
 * - MAVSDK 라이브러리 필요
 *
 * [주요 기능]
 * 1. 신호 측정 없이 4방향 이동만 테스트
 * 2. 북 → 동 → 남 → 서 순서로 이동
 * 3. 각 방향 이동 후 원위치 복귀
 * 4. NED 로컬 좌표계 사용
 *
 * [사용 목적]
 * - 드론 이동 기능만 검증
 * - 신호 측정 없이 간단한 테스트
 * - 개발/디버깅 단계에서 사용
 *
 * [컴파일]
 *   g++ -std=c++17 simple_move.cpp -o simple_move -lmavsdk
 *
 * [사용법]
 *   ./simple_move serial:///dev/ttyAMA0:57600  (실제 드론)
 *   ./simple_move udp://:14540                 (SITL 테스트)
 *
 * [설정 변수]
 * - MOVE_DISTANCE: 각 방향 이동 거리 (기본 10m)
 * - MAX_ROUNDS: 최대 이동 라운드 (기본 3회)
 *
 * [참고]
 * - Python 버전: simple_move.py
 * - 실제 탐색은 simple.cpp 또는 simple_search.cpp 사용
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

using namespace mavsdk;
using namespace std::chrono;
using namespace std::this_thread;

// ========================================
// 스크립트 설정
// ========================================
const float MOVE_DISTANCE = 10.0f;  // 각 방향으로 이동할 거리 (미터)
const int MAX_ROUNDS = 3;           // 최대 이동 라운드

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
// 4방향 이동
// ========================================

/**
 * 4방향(북, 동, 남, 서)으로 순서대로 이동
 * 신호 측정 없이 이동만 수행
 */
void move_four_directions(std::shared_ptr<Offboard> offboard,
                         std::shared_ptr<Telemetry> telemetry,
                         float move_distance = 10.0f) {

    std::cout << "\n============================================================" << std::endl;
    std::cout << "4방향 이동 시작 (이동 거리: " << move_distance << "m)" << std::endl;
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

    // 4방향 정의
    struct Direction {
        std::string name;
        float dNorth;
        float dEast;
    };

    std::vector<Direction> directions = {
        {"북쪽", move_distance, 0.0f},
        {"동쪽", 0.0f, move_distance},
        {"남쪽", -move_distance, 0.0f},
        {"서쪽", 0.0f, -move_distance}
    };

    // 각 방향으로 이동
    for (const auto& dir : directions) {
        std::cout << "\n--------------------------------------------------" << std::endl;
        std::cout << "[" << dir.name << "] 이동 시작" << std::endl;
        std::cout << "--------------------------------------------------" << std::endl;

        // 현재 위치에서 상대적으로 이동
        auto current_pos = telemetry->position_velocity_ned();
        float current_north = current_pos.position.north_m;
        float current_east = current_pos.position.east_m;
        float current_down = current_pos.position.down_m;

        float target_north = current_north + dir.dNorth;
        float target_east = current_east + dir.dEast;

        std::cout << "목표: North=" << target_north << "m, East=" << target_east << "m" << std::endl;

        bool success = goto_position_ned(offboard, telemetry, target_north, target_east, current_down, 60);

        if (success) {
            std::cout << "✓ " << dir.name << " 이동 완료" << std::endl;
        } else {
            std::cout << "⚠️ " << dir.name << " 이동 실패 (타임아웃)" << std::endl;
        }

        // 다음 이동 전 잠시 대기
        sleep_for(seconds(2));
    }

    // 원위치 복귀
    std::cout << "\n============================================================" << std::endl;
    std::cout << "원위치로 복귀" << std::endl;
    std::cout << "============================================================" << std::endl;
    goto_position_ned(offboard, telemetry, home_north, home_east, home_down, 60);
    std::cout << "✓ 복귀 완료\n" << std::endl;
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
    std::cout << "수동으로 이륙 후 Enter를 눌러 이동 테스트를 시작하세요." << std::endl;
    std::cout << "============================================================" << std::endl;
    std::cin.get();

    // Offboard 모드 시작
    std::cout << "Offboard 모드 시작..." << std::endl;
    Offboard::Result offboard_result = offboard->start();
    if (offboard_result != Offboard::Result::Success) {
        std::cerr << "⚠️ Offboard 모드 시작 실패: " << offboard_result << std::endl;
        std::cerr << "GUIDED 모드로 전환하거나 권한을 확인하세요." << std::endl;
    }

    // 4방향 이동 반복
    for (int round_num = 1; round_num <= MAX_ROUNDS; round_num++) {
        std::cout << "\n############################################################" << std::endl;
        std::cout << "###  이동 라운드 " << round_num << "/" << MAX_ROUNDS << "  ###" << std::endl;
        std::cout << "############################################################" << std::endl;

        move_four_directions(offboard, telemetry, MOVE_DISTANCE);

        // 다음 라운드 전 대기
        if (round_num < MAX_ROUNDS) {
            int wait_time = 5;
            std::cout << "다음 라운드까지 " << wait_time << "초 대기...\n" << std::endl;
            sleep_for(seconds(wait_time));
        }
    }

    std::cout << "\n============================================================" << std::endl;
    std::cout << "모든 이동 테스트 완료!" << std::endl;
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
    std::cout << "\n✓✓✓ 테스트 완료! ✓✓✓\n" << std::endl;

    return 0;
}
