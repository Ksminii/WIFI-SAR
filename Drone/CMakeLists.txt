cmake_minimum_required(VERSION 3.10)
project(drone_search)

# C++17 표준 사용
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 컴파일 옵션
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra")

# MAVSDK 찾기
find_package(MAVSDK REQUIRED)

# 실행 파일 생성
add_executable(simple simple.cpp)

# MAVSDK 라이브러리 링크
target_link_libraries(simple
    MAVSDK::mavsdk
)

# 설치 규칙
install(TARGETS simple
    RUNTIME DESTINATION bin
)

message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")
message(STATUS "C++ standard: ${CMAKE_CXX_STANDARD}")
