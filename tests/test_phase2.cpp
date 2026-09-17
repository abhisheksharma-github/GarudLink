/**
 * @file test_phase2.cpp
 * @brief Comprehensive Verification Test Suite for Phase 2 Tactical Entities: Drone, RadarSystem, and GuidedMissile.
 */

#include "Entities.hpp"
#include "Math3D.hpp"
#include "ShipDataBus.hpp"

#include <cmath>
#include <iostream>
#include <memory>
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
    std::cout << "\n========================================\n" \
              << " RUNNING: " << name << "\n" \
              << "========================================\n"

} // anonymous namespace

using namespace SDN::Math;
using namespace SDN::Network;
using namespace SDN::Entities;

// ============================================================================
// Test 1: Drone Waypoint Navigation, Turn Limiting, and Evasive Weave
// ============================================================================
void test_drone_kinematics() {
    TEST_SECTION("Drone Kinematics, Waypoints, Evasive Weave & Telemetry");

    // 1. Initial State & Waypoint Navigation
    Drone drone("UAV-01", Classification::HOSTILE, Vector3D{0.0, 0.0, -500.0}, 50.0, 0.2);
    drone.addWaypoint(Vector3D{500.0, 0.0, -500.0});
    drone.addWaypoint(Vector3D{500.0, 500.0, -500.0});
    drone.setWaypointAcceptanceRadius(30.0);

    TEST_ASSERT(drone.getWaypoints().size() == 2, "Drone has 2 waypoints");
    TEST_ASSERT(drone.getCurrentWaypointIndex() == 0, "Drone starts at waypoint 0");

    // Simulate 12 seconds of flight (50 m/s * 12s = 600m -> reaches WP 0 at 500m)
    for (int i = 0; i < 120; ++i) {
        drone.update(0.1);
    }

    TEST_ASSERT(drone.getCurrentWaypointIndex() == 1, "Drone reached WP 0 and advanced to WP 1");
    TEST_ASSERT(drone.getPosition().x > 450.0, "Drone advanced along +X axis towards 500m");

    // 2. Evasive Weave Mode
    Drone evasiveDrone("UAV-02", Classification::HOSTILE, Vector3D{0.0, 0.0, -500.0}, 60.0);
    evasiveDrone.setEvasiveMode(true);
    evasiveDrone.setEvasiveParameters(1.0, 5.0); // 1 Hz, 5G lateral weave

    evasiveDrone.update(0.25); // At t=0.25s (quarter cycle of 1Hz), sin(2*pi*1*0.25) = sin(pi/2) = 1.0 (Peak lateral acceleration)
    const Vector3D lateralAccel = evasiveDrone.getAcceleration();
    const double accelMag = lateralAccel.magnitude();
    TEST_ASSERT(std::abs(accelMag - (5.0 * 9.80665)) < 0.5, "Evasive lateral acceleration peaks at ~5G");

    // 3. Telemetry Serialization
    std::string telemetry = drone.serializeTelemetry();
    TEST_ASSERT(validateChecksum(telemetry), "Drone telemetry has valid NMEA checksum");
    TEST_ASSERT(telemetry.find("$UAVPOS,UAV-01,") == 0, "Drone telemetry prefix matches $UAVPOS,UAV-01");
}

// ============================================================================
// Test 2: Physical Radar Range Equation, Beam Cone Filtering & Track Output
// ============================================================================
void test_radar_range_equation_and_scanning() {
    TEST_SECTION("Radar Range Equation, Spatial Beam Cone & Detection");

    // Naval X-band Radar at Origin
    RadarSystem radar("RDR-01", Classification::FRIENDLY, Vector3D{0.0, 0.0, -25.0},
                      50000.0,   // 50 kW Peak Power
                      35.0,      // 35 dBi Gain
                      9.4e9,     // 9.4 GHz (lambda ~ 0.0319 m)
                      1e-13,     // -100 dBm sensitivity
                      2.0,       // 2 deg Azimuth Beamwidth
                      30.0,      // 30 deg Elevation Beamwidth
                      60.0);     // 60 deg/sec Scan Rate

    // 1. Target Inside Beam Cone at 15 km (North)
    radar.setCurrentAzimuthDeg(0.0); // Pointing North
    Drone target1("TGT-01", Classification::HOSTILE, Vector3D{15000.0, 0.0, -500.0}, 0.0, 2.0); // 2 m^2 RCS

    RadarDetection det1 = radar.interrogateTarget(target1, 2.0);
    TEST_ASSERT(det1.detected, "Target inside beam cone at 15km is detected");
    TEST_ASSERT(det1.rangeM > 14900.0 && det1.rangeM < 15100.0, "Slant range is ~15km");
    TEST_ASSERT(det1.snrDb > 0.0, "SNR is positive (> 0 dB)");

    // 2. Inverse 4th Power Law Verification (R^4 roll-off)
    // Moving target from 10km to 20km (2x range) should decrease power by 2^4 = 16 (-12.04 dB)
    Drone target10k("TGT-10k", Classification::HOSTILE, Vector3D{10000.0, 0.0, 0.0}, 0.0, 1.0);
    Drone target20k("TGT-20k", Classification::HOSTILE, Vector3D{20000.0, 0.0, 0.0}, 0.0, 1.0);

    RadarDetection det10k = radar.interrogateTarget(target10k, 1.0);
    RadarDetection det20k = radar.interrogateTarget(target20k, 1.0);

    const double powerRatio = det10k.receivedPowerWatts / det20k.receivedPowerWatts;
    TEST_ASSERT(std::abs(powerRatio - 16.0) < 0.2, "Received power follows exact R^4 law (ratio ~ 16)");
    const double deltaSnrDb = det10k.snrDb - det20k.snrDb;
    TEST_ASSERT(std::abs(deltaSnrDb - 12.04) < 0.2, "SNR difference across 2x range is ~12.04 dB");

    // 3. Target Outside Azimuth Beam Cone
    // Target is at East (90 deg), while Radar points North (0 deg)
    Drone targetOffBeam("TGT-OFF", Classification::HOSTILE, Vector3D{0.0, 5000.0, -500.0}, 0.0, 5.0);
    RadarDetection detOff = radar.interrogateTarget(targetOffBeam, 5.0);
    TEST_ASSERT(!detOff.detected, "Target outside azimuth beam cone is not detected");

    // 4. Track NMEA Sentence Formatting
    std::string trackSentence = radar.formatTrackSentence(det1);
    TEST_ASSERT(validateChecksum(trackSentence), "Track sentence has valid NMEA checksum");
    TEST_ASSERT(trackSentence.find("$RDRTRK,TGT-01,") == 0, "Track sentence prefix matches $RDRTRK,TGT-01");
}

// ============================================================================
// Test 3: Guided Missile True Proportional Navigation (TPN) & Intercept
// ============================================================================
void test_guided_missile_guidance_and_intercept() {
    TEST_SECTION("Guided Missile True Proportional Navigation (TPN) & Intercept");

    // 1. Target Drone Flying East at 100 m/s at 2000m Altitude
    Vector3D targetInitialPos{4000.0, 1000.0, -2000.0};
    Vector3D targetVelocity{0.0, 100.0, 0.0};
    auto target = std::make_shared<Drone>("TGT-INTRUDER", Classification::HOSTILE, targetInitialPos, 100.0);
    target->setVelocity(targetVelocity);

    // 2. Interceptor Missile Initialized
    Vector3D missileInitialPos{0.0, 0.0, 0.0};
    Vector3D missileInitialVel{300.0, 75.0, -150.0}; // Boosted initial velocity towards target
    GuidedMissile missile("MSL-01", Classification::FRIENDLY,
                          missileInitialPos, missileInitialVel,
                          "TGT-INTRUDER",
                          5.0,     // 5.0 sec burn time
                          15000.0, // 15 kN thrust
                          40.0,    // 40 kg dry mass
                          35.0,    // 35 kg fuel mass
                          3.5,     // N = 3.5 Navigation ratio
                          35.0,    // 35G structural limit
                          10.0);   // 10m Proximity fuse radius

    // 3. Closed-Loop Tactical Guidance Simulation Loop
    const double dt = 0.01; // 100 Hz GNC guidance cycle
    bool interceptOccurred = false;

    for (int step = 0; step < 1200; ++step) { // Max 12 seconds
        target->update(dt);

        // Feed updated target state into missile seeker
        missile.setTargetKinematics(target->getPosition(), target->getVelocity());
        missile.update(dt);

        if (missile.hasDetonated() && missile.isTargetIntercepted()) {
            interceptOccurred = true;
            break;
        }
    }

    TEST_ASSERT(interceptOccurred, "Missile successfully intercepted maneuvering target using TPN");
    TEST_ASSERT(missile.getRangeToGo() <= 10.0, "Detonation occurred within 10m proximity fuse radius");
    TEST_ASSERT(!missile.isActive(), "Missile is deactivated following proximity detonation");

    // 4. Status Telemetry Verification
    std::string missileTelemetry = missile.serializeTelemetry();
    TEST_ASSERT(validateChecksum(missileTelemetry), "Missile status telemetry has valid NMEA checksum");
    TEST_ASSERT(missileTelemetry.find("$MSLSTATUS,MSL-01,TGT-INTRUDER,") == 0, "Missile telemetry header matches");
}

// ============================================================================
// Main Entry Point
// ============================================================================
int main() {
    std::cout << "========================================================\n"
              << " Distributed Tactical Defense Simulator - Phase 2\n"
              << " Kinematic & Physical Entity Models Verification\n"
              << "========================================================\n";

    test_drone_kinematics();
    test_radar_range_equation_and_scanning();
    test_guided_missile_guidance_and_intercept();

    std::cout << "\n========================================================\n"
              << " TEST SUMMARY\n"
              << " Total Assertions: " << g_totalTests << "\n"
              << " Passed:           " << g_passedTests << "\n"
              << " Failed:           " << g_failedTests << "\n"
              << "========================================================\n";

    if (g_failedTests == 0) {
        std::cout << "\n>>> ALL PHASE 2 VERIFICATION TESTS PASSED SUCCESSFULLY! <<<\n\n";
        return 0;
    } else {
        std::cerr << "\n>>> VERIFICATION FAILED WITH " << g_failedTests << " ERRORS! <<<\n\n";
        return 1;
    }
}
