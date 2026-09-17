/**
 * @file test_suite.cpp
 * @brief Unified Test Suite validating Kinematic Convergence, Radar Inverse 4th-Power Physics,
 *        True Proportional Navigation (TPN) Intercepts, and ShipDataBus Throughput Under Load.
 */

#include "Entities.hpp"
#include "Math3D.hpp"
#include "ShipDataBus.hpp"
#include "SimulationEngine.hpp"

#include <atomic>
#include <chrono>
#include <cmath>
#include <iostream>
#include <memory>
#include <thread>
#include <vector>

namespace {

int g_totalTests = 0;
int g_passedTests = 0;
int g_failedTests = 0;

#define TEST_ASSERT(condition, msg) \
    do { \
        ++g_totalTests; \
        if (condition) { \
            ++g_passedTests; \
        } else { \
            ++g_failedTests; \
            std::cerr << "  [FAIL] " << msg << " (" << __FILE__ << ":" << __LINE__ << ")\n"; \
        } \
    } while (0)

#define TEST_SECTION(name) \
    std::cout << "\n=================================================================\n" \
              << " RUNNING: " << name << "\n" \
              << "=================================================================\n"

} // anonymous namespace

using namespace SDN::Math;
using namespace SDN::Network;
using namespace SDN::Entities;
using namespace SDN::Engine;

// ============================================================================
// Test 1: Kinematic Convergence Under Constant Acceleration
// ============================================================================
void test_kinematic_convergence() {
    TEST_SECTION("Test 1: Kinematic Position Convergence vs Analytical Ground Truth");

    // Analytical initial conditions:
    // s(t) = s0 + v0*t + 0.5*a*t^2
    const Vector3D initialPos{100.0, -50.0, 20.0};
    const Vector3D initialVel{30.0, 40.0, -10.0};
    const Vector3D constantAccel{5.0, -2.5, 9.80665};
    const double totalTimeSec = 10.0;

    const Vector3D analyticalFinalPos = initialPos + (initialVel * totalTimeSec) + 
                                        (constantAccel * (0.5 * totalTimeSec * totalTimeSec));

    // Numerical integration using Drone entity under constant acceleration
    Drone testDrone("TEST-KINEMATICS", Classification::FRIENDLY, initialPos, initialVel.magnitude());
    testDrone.setVelocity(initialVel);
    testDrone.setAcceleration(constantAccel);

    const double dt = 0.001; // 1 kHz fine numerical step
    const int steps = static_cast<int>(totalTimeSec / dt);

    for (int i = 0; i < steps; ++i) {
        // Integrate step
        testDrone.setPosition(testDrone.getPosition() + (testDrone.getVelocity() * dt));
        testDrone.setVelocity(testDrone.getVelocity() + (constantAccel * dt));
    }

    const Vector3D numericalFinalPos = testDrone.getPosition();
    const double errorDistance = numericalFinalPos.distanceTo(analyticalFinalPos);
    const double relativeError = errorDistance / analyticalFinalPos.magnitude();

    std::cout << "  Analytical Final Pos: " << analyticalFinalPos << "\n"
              << "  Numerical Final Pos:  " << numericalFinalPos << "\n"
              << "  Position Error:       " << errorDistance << " m (Relative: " << (relativeError * 100.0) << "%)\n";

    TEST_ASSERT(errorDistance < 0.25, "Kinematic position converges within 0.25m over 10s flight");
    TEST_ASSERT(relativeError < 0.001, "Relative integration error is below 0.1%");
}

// ============================================================================
// Test 2: Radar Range Equation Inverse 4th-Power Decay Physics
// ============================================================================
void test_radar_inverse_fourth_power() {
    TEST_SECTION("Test 2: Radar Range Equation Inverse 4th-Power (1/R^4) Decay");

    RadarSystem radar("TEST-RDR", Classification::FRIENDLY, Vector3D{0.0, 0.0, 0.0},
                      50000.0, // 50 kW
                      35.0,    // 35 dBi
                      9.4e9,   // 9.4 GHz
                      1e-13,   // Sensitivity
                      5.0,     // 5 deg Azimuth Beamwidth
                      30.0,    // 30 deg Elevation Beamwidth
                      60.0);
    radar.setCurrentAzimuthDeg(0.0);

    // Test across multiple slant ranges: 5 km, 10 km (2x), 20 km (4x)
    Drone tgt5k("TGT-5k", Classification::HOSTILE, Vector3D{5000.0, 0.0, 0.0}, 0.0, 1.0);
    Drone tgt10k("TGT-10k", Classification::HOSTILE, Vector3D{10000.0, 0.0, 0.0}, 0.0, 1.0);
    Drone tgt20k("TGT-20k", Classification::HOSTILE, Vector3D{20000.0, 0.0, 0.0}, 0.0, 1.0);

    RadarDetection det5k = radar.interrogateTarget(tgt5k, 1.0);
    RadarDetection det10k = radar.interrogateTarget(tgt10k, 1.0);
    RadarDetection det20k = radar.interrogateTarget(tgt20k, 1.0);

    TEST_ASSERT(det5k.receivedPowerWatts > 0.0, "5km target received power is positive");
    TEST_ASSERT(det10k.receivedPowerWatts > 0.0, "10km target received power is positive");
    TEST_ASSERT(det20k.receivedPowerWatts > 0.0, "20km target received power is positive");

    // Theoretical ratios:
    // (R2 / R1)^4 = (10000 / 5000)^4 = 2^4 = 16
    // (R3 / R1)^4 = (20000 / 5000)^4 = 4^4 = 256
    const double ratio2x = det5k.receivedPowerWatts / det10k.receivedPowerWatts;
    const double ratio4x = det5k.receivedPowerWatts / det20k.receivedPowerWatts;

    std::cout << "  Power @ 5km:  " << det5k.receivedPowerWatts << " W (SNR: " << det5k.snrDb << " dB)\n"
              << "  Power @ 10km: " << det10k.receivedPowerWatts << " W (SNR: " << det10k.snrDb << " dB)\n"
              << "  Power @ 20km: " << det20k.receivedPowerWatts << " W (SNR: " << det20k.snrDb << " dB)\n"
              << "  2x Range Power Decay Ratio (Theoretical 16.0):  " << ratio2x << "\n"
              << "  4x Range Power Decay Ratio (Theoretical 256.0): " << ratio4x << "\n";

    TEST_ASSERT(std::abs(ratio2x - 16.0) < 0.05, "2x range yields exactly 16x power reduction");
    TEST_ASSERT(std::abs(ratio4x - 256.0) < 0.5, "4x range yields exactly 256x power reduction");
}

// ============================================================================
// Test 3: True Proportional Navigation Intercept Against Maneuvering Target
// ============================================================================
void test_proportional_navigation_maneuvering_target() {
    TEST_SECTION("Test 3: True Proportional Navigation (TPN) Against 4.5G Weaving Target");

    // Target Drone flying at 160 m/s with aggressive 4.5G sinusoidal evasive weave
    Vector3D targetPos{6000.0, 3000.0, -1200.0};
    auto target = std::make_shared<Drone>("TGT-EVASIVE", Classification::HOSTILE, targetPos, 160.0);
    target->setEvasiveMode(true);
    target->setEvasiveParameters(0.5, 4.5); // 0.5 Hz, 4.5G lateral weave

    // High-G Interceptor Missile
    Vector3D missilePos{0.0, 0.0, 0.0};
    Vector3D missileVel{250.0, 125.0, -50.0}; // Initial boost vector
    GuidedMissile missile("MSL-INTERCEPTOR", Classification::FRIENDLY,
                          missilePos, missileVel,
                          "TGT-EVASIVE",
                          6.0,     // 6.0 sec burn time
                          16000.0, // 16 kN thrust
                          40.0,    // 40 kg dry mass
                          35.0,    // 35 kg propellant
                          3.5,     // Navigation ratio N = 3.5
                          35.0,    // 35G structural limit
                          10.0);   // 10m proximity fuse radius

    const double dt = 0.005; // 200 Hz guidance loop
    bool intercepted = false;
    double minDistanceReached = 1e9;

    for (int step = 0; step < 2400; ++step) { // Max 12 seconds
        target->update(dt);

        missile.setTargetKinematics(target->getPosition(), target->getVelocity());
        missile.update(dt);

        double dist = missile.getPosition().distanceTo(target->getPosition());
        if (dist < minDistanceReached) {
            minDistanceReached = dist;
        }

        if (missile.hasDetonated() && missile.isTargetIntercepted()) {
            intercepted = true;
            break;
        }
    }

    std::cout << "  Minimum Miss Distance: " << minDistanceReached << " m\n"
              << "  Proximity Detonation:  " << (intercepted ? "TRIGGERED" : "MISSED") << "\n";

    TEST_ASSERT(intercepted, "Missile successfully intercepted high-G weaving drone");
    TEST_ASSERT(minDistanceReached <= 15.0, "Miss distance is strictly within 15m tolerance");
    TEST_ASSERT(minDistanceReached <= 10.0, "Proximity fuse detonated within 10m radius");
}

// ============================================================================
// Test 4: ShipDataBus Queue Saturation, Throughput & Dropped Counter
// ============================================================================
void test_ship_data_bus_queue_saturation() {
    TEST_SECTION("Test 4: ShipDataBus Queue Saturation & Dropped Counter Metrics Under Load");

    // Create a bus with a intentionally small queue capacity of 50 messages
    constexpr size_t CAPACITY = 50;
    ShipDataBus smallBus(CAPACITY);
    // Do not start worker thread yet to force immediate buffer saturation

    std::atomic<int> receivedCount{0};
    smallBus.subscribe(Talkers::WILDCARD_ALL, [&](const NetworkMessage&) {
        receivedCount++;
    });

    // Burst 250 messages into the 50-capacity queue
    constexpr int BURST_MESSAGES = 250;
    int acceptedCount = 0;
    int rejectedCount = 0;

    for (int i = 0; i < BURST_MESSAGES; ++i) {
        std::string sentence = formatSentence("GP", "GGA", {std::to_string(i), "1234.56", "N"});
        if (smallBus.publishRaw(sentence)) {
            acceptedCount++;
        } else {
            rejectedCount++;
        }
    }

    std::cout << "  Queue Capacity:    " << CAPACITY << "\n"
              << "  Burst Messages:    " << BURST_MESSAGES << "\n"
              << "  Accepted Messages: " << acceptedCount << "\n"
              << "  Rejected (Full):   " << rejectedCount << "\n"
              << "  Bus Dropped Count: " << smallBus.droppedCount() << "\n";

    TEST_ASSERT(acceptedCount == static_cast<int>(CAPACITY), "Bus accepted exactly capacity (50) messages");
    TEST_ASSERT(rejectedCount == (BURST_MESSAGES - static_cast<int>(CAPACITY)), "Bus rejected 200 excess messages");
    TEST_ASSERT(smallBus.droppedCount() == static_cast<uint64_t>(rejectedCount), "Bus dropped count accurately tracks rejected packets");

    // Now start the worker thread to drain accepted queue
    smallBus.start();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    smallBus.stop(true);

    TEST_ASSERT(receivedCount.load() == static_cast<int>(CAPACITY), "Worker drained all 50 accepted messages without corruption");
}

// ============================================================================
// Main Entry Point
// ============================================================================
int main() {
    std::cout << "=================================================================\n"
              << "  DISTRIBUTED TACTICAL DEFENSE & SHIP DATA NETWORK (SDN)        \n"
              << "  Unified Verification & Validation Test Suite                   \n"
              << "=================================================================\n";

    test_kinematic_convergence();
    test_radar_inverse_fourth_power();
    test_proportional_navigation_maneuvering_target();
    test_ship_data_bus_queue_saturation();

    std::cout << "\n=================================================================\n"
              << " TEST SUITE SUMMARY\n"
              << " Total Assertions: " << g_totalTests << "\n"
              << " Passed:           " << g_passedTests << "\n"
              << " Failed:           " << g_failedTests << "\n"
              << "=================================================================\n";

    if (g_failedTests == 0) {
        std::cout << "\n>>> ALL VERIFICATION AND VALIDATION TESTS PASSED (100%)! <<<\n\n";
        return 0;
    } else {
        std::cerr << "\n>>> TEST SUITE FAILED WITH " << g_failedTests << " ERRORS! <<<\n\n";
        return 1;
    }
}
