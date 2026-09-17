cmake_minimum_required(VERSION 3.20)
project(TacticalDefenseSDN VERSION 1.0.0 LANGUAGES CXX)

# Require Modern C++20 standard
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Compiler optimization and diagnostic flags
if(MSVC)
    add_compile_options(/W4 /permissive- /O2)
else()
    add_compile_options(-Wall -Wextra -Wpedantic -O3)
endif()

# Threading support
find_package(Threads REQUIRED)

# Core SDN & Simulation Library
add_library(SDNCore STATIC
    src/Math3D.cpp
    src/ShipDataBus.cpp
    src/Entities.cpp
    src/SimulationEngine.cpp
    src/WebSocketBridge.cpp
    src/ScenarioConfig.cpp
    src/ScenarioManager.cpp
)

target_include_directories(SDNCore PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(SDNCore PUBLIC Threads::Threads)
if(WIN32)
    target_link_libraries(SDNCore PUBLIC ws2_32)
endif()

# Main Real-Time Simulation Executable
add_executable(tactical_sim
    src/main.cpp
)
target_link_libraries(tactical_sim PRIVATE SDNCore)

# Comprehensive Test Runner Executable
add_executable(run_tests
    tests/test_suite.cpp
)
target_link_libraries(run_tests PRIVATE SDNCore)

# Phase-Specific Unit Test Targets (Optional)
add_executable(test_phase1 tests/test_phase1.cpp)
target_link_libraries(test_phase1 PRIVATE SDNCore)

add_executable(test_phase2 tests/test_phase2.cpp)
target_link_libraries(test_phase2 PRIVATE SDNCore)

# CTest Integration
enable_testing()
add_test(NAME UnifiedTestSuite COMMAND run_tests)
add_test(NAME Phase1UnitTests COMMAND test_phase1)
add_test(NAME Phase2UnitTests COMMAND test_phase2)
