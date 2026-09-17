/**
 * @file test_phase1.cpp
 * @brief Comprehensive Verification Test Suite for Math3D and ShipDataBus.
 * 
 * Tests:
 * 1. Vector3D operations, norms, products, normalization, zero-division, and NaN resilience.
 * 2. EulerAngles degree/radian conversions, angular wrapping, and tactical LOS kinematics.
 * 3. IEC 61162-1 / NMEA 0183 XOR checksum calculation, validation, formatting, and corrupted frame rejection.
 * 4. Multi-threaded asynchronous ShipDataBus producer-consumer routing, subscription filtering, and shutdown.
 */

#include "Math3D.hpp"
#include "ShipDataBus.hpp"

#include <atomic>
#include <cassert>
#include <chrono>
#include <cmath>
#include <iostream>
#include <numeric>
#include <string>
#include <thread>
#include <vector>

// ============================================================================
// Minimalist Zero-Dependency Test Harness
// ============================================================================

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

// ============================================================================
// Test 1: 3D Vector Math, Edge Cases, & Numerical Safety
// ============================================================================
void test_vector_basics_and_operations() {
    TEST_SECTION("Vector3D Basic Operations & Vector Algebra");

    // 1. Constructors & Arithmetic
    Vector3D v1{1.0, 2.0, 3.0};
    Vector3D v2{4.0, 5.0, 6.0};

    Vector3D sum = v1 + v2;
    TEST_ASSERT(sum.equals(Vector3D{5.0, 7.0, 9.0}), "Vector3D addition");

    Vector3D diff = v2 - v1;
    TEST_ASSERT(diff.equals(Vector3D{3.0, 3.0, 3.0}), "Vector3D subtraction");

    Vector3D scaled = v1 * 2.5;
    TEST_ASSERT(scaled.equals(Vector3D{2.5, 5.0, 7.5}), "Vector3D scalar multiplication");

    Vector3D leftScaled = 2.5 * v1;
    TEST_ASSERT(leftScaled.equals(Vector3D{2.5, 5.0, 7.5}), "Vector3D left scalar multiplication");

    Vector3D div = v2 / 2.0;
    TEST_ASSERT(div.equals(Vector3D{2.0, 2.5, 3.0}), "Vector3D scalar division");

    Vector3D neg = -v1;
    TEST_ASSERT(neg.equals(Vector3D{-1.0, -2.0, -3.0}), "Vector3D unary negation");

    // 2. Magnitudes & Distances
    Vector3D v3{3.0, 4.0, 0.0};
    TEST_ASSERT(std::abs(v3.magnitude() - 5.0) < 1e-9, "Vector3D magnitude (3-4-5 triangle)");
    TEST_ASSERT(std::abs(v3.magnitudeSquared() - 25.0) < 1e-9, "Vector3D magnitudeSquared");

    Vector3D pA{0.0, 0.0, 0.0};
    Vector3D pB{1.0, 2.0, 2.0};
    TEST_ASSERT(std::abs(pA.distanceTo(pB) - 3.0) < 1e-9, "Vector3D distanceTo (1-2-2 -> 3)");

    // 3. Dot and Cross Products
    Vector3D unitX{1.0, 0.0, 0.0};
    Vector3D unitY{0.0, 1.0, 0.0};
    Vector3D unitZ{0.0, 0.0, 1.0};

    TEST_ASSERT(std::abs(unitX.dot(unitY)) < 1e-9, "Dot product of orthogonal vectors is 0");
    TEST_ASSERT(std::abs(unitX.dot(unitX) - 1.0) < 1e-9, "Dot product of unit vector with itself is 1");

    Vector3D crossXY = unitX.cross(unitY);
    TEST_ASSERT(crossXY.equals(unitZ), "Cross product X x Y = Z (Right-hand rule)");

    Vector3D crossYX = unitY.cross(unitX);
    TEST_ASSERT(crossYX.equals(-unitZ), "Cross product Y x X = -Z (Anti-commutativity)");

    Vector3D crossSelf = unitX.cross(unitX);
    TEST_ASSERT(crossSelf.isZero(), "Cross product of parallel vector is zero vector");
}

void test_vector_numerical_edge_cases() {
    TEST_SECTION("Vector3D Numerical Edge Cases & Zero Protection");

    // 1. Safe Normalization of Normal Vector
    Vector3D v{0.0, 3.0, 4.0};
    Vector3D vNorm = v.normalized();
    TEST_ASSERT(std::abs(vNorm.magnitude() - 1.0) < 1e-9, "Normalized vector has unit magnitude");
    TEST_ASSERT(vNorm.equals(Vector3D{0.0, 0.6, 0.8}), "Normalized vector components match");

    // 2. Safe Normalization of Zero Vector
    Vector3D zeroVec{0.0, 0.0, 0.0};
    Vector3D zeroNorm = zeroVec.normalized();
    TEST_ASSERT(zeroNorm.isZero(), "Normalization of zero vector safely returns zero vector");
    TEST_ASSERT(zeroNorm.isValid(), "Normalized zero vector components remain finite/valid");

    // 3. Sub-epsilon vector normalization
    Vector3D subEpsVec{1e-15, 1e-15, 1e-15};
    Vector3D subEpsNorm = subEpsVec.normalized(1e-12);
    TEST_ASSERT(subEpsNorm.isZero(), "Normalization of sub-epsilon vector safely returns zero vector");

    // 4. Safe division by zero scalar
    Vector3D divZero = v / 0.0;
    TEST_ASSERT(divZero.isZero(), "Division by 0.0 returns safe zero vector");

    // 5. Compound operator division by zero
    Vector3D divZeroInPlace{10.0, 20.0, 30.0};
    divZeroInPlace /= 0.0;
    TEST_ASSERT(divZeroInPlace.isZero(), "/= by 0.0 safely resets vector to zero");

    // 6. Validity & NaN checks
    Vector3D validVec{100.5, -200.3, 50.0};
    TEST_ASSERT(validVec.isValid(), "Standard vector is valid");

    Vector3D nanVec{std::numeric_limits<double>::quiet_NaN(), 1.0, 2.0};
    TEST_ASSERT(!nanVec.isValid(), "Vector containing NaN is invalid");

    Vector3D infVec{1.0, std::numeric_limits<double>::infinity(), 2.0};
    TEST_ASSERT(!infVec.isValid(), "Vector containing Infinity is invalid");
}

// ============================================================================
// Test 2: Euler Angles & Tactical Line-Of-Sight (LOS) Kinematics
// ============================================================================
void test_euler_and_los_kinematics() {
    TEST_SECTION("Euler Angles & Tactical LOS Kinematics");

    // 1. Euler Angles Conversion
    EulerAngles angles = EulerAngles::fromDegrees(45.0, 30.0, 90.0);
    TEST_ASSERT(std::abs(angles.rollDeg() - 45.0) < 1e-6, "Euler roll degree conversion");
    TEST_ASSERT(std::abs(angles.pitchDeg() - 30.0) < 1e-6, "Euler pitch degree conversion");
    TEST_ASSERT(std::abs(angles.yawDeg() - 90.0) < 1e-6, "Euler yaw degree conversion");

    // 2. Normalization of out-of-bounds angles
    EulerAngles rawAngles = EulerAngles::fromDegrees(370.0, 110.0, -45.0);
    EulerAngles normAngles = rawAngles.normalized();
    TEST_ASSERT(std::abs(normAngles.rollDeg() - 10.0) < 1e-6, "Euler roll normalization (370 -> 10 deg)");
    TEST_ASSERT(std::abs(normAngles.pitchDeg() - 90.0) < 1e-6, "Euler pitch clamping to 90 deg");
    TEST_ASSERT(std::abs(normAngles.yawDeg() - 315.0) < 1e-6, "Euler yaw normalization (-45 -> 315 deg)");

    // 3. Tactical LOS Calculation (Observer at Origin, Target at 10km North)
    Vector3D observerPos{0.0, 0.0, 0.0};
    Vector3D observerVel{0.0, 0.0, 0.0};
    Vector3D targetPos{10000.0, 0.0, 0.0}; // 10,000 m along North (X-axis)
    Vector3D targetVel{-250.0, 50.0, 0.0};  // Flying towards observer at 250 m/s with 50 m/s Eastward drift

    LOSResult los = calculateLOS(observerPos, observerVel, targetPos, targetVel);
    TEST_ASSERT(!los.isSingular, "LOS is not singular at 10km range");
    TEST_ASSERT(std::abs(los.range - 10000.0) < 1e-3, "LOS range equals 10,000 m");
    TEST_ASSERT(los.losUnitVector.equals(Vector3D{1.0, 0.0, 0.0}, 1e-6), "LOS unit vector points North (+X)");
    TEST_ASSERT(std::abs(los.closingVelocity - 250.0) < 1e-3, "Closing velocity V_c = 250 m/s");
    
    // Cross product: R x V = (10000, 0, 0) x (-250, 50, 0) = (0, 0, 500000)
    // LOS Rate: Omega = (0, 0, 500000) / (10000^2) = (0, 0, 0.005) rad/s
    TEST_ASSERT(std::abs(los.losRateVector.z - 0.005) < 1e-6, "LOS angular rate in Yaw = 0.005 rad/s");

    // 4. Singularity / Zero-Range Protection
    LOSResult singularLos = calculateLOS(observerPos, observerVel, observerPos, targetVel);
    TEST_ASSERT(singularLos.isSingular, "Co-located observer and target is marked singular");
    TEST_ASSERT(singularLos.losUnitVector.isZero(), "Singular LOS unit vector is safe zero vector");
    TEST_ASSERT(singularLos.losRateVector.isZero(), "Singular LOS rate vector is safe zero vector");
}

// ============================================================================
// Test 3: NMEA 0183 / IEC 61162-1 Checksum & Message Parsing
// ============================================================================
void test_nmea_checksum_and_parsing() {
    TEST_SECTION("NMEA 0183 / IEC 61162-1 Protocol & Checksum Validation");

    // 1. Checksum calculation on standard NMEA sentences
    // "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47"
    std::string_view ggaPayload = "GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,";
    uint8_t ggaChecksum = calculateChecksum(ggaPayload);
    TEST_ASSERT(ggaChecksum == 0x47, "GGA Checksum matches 0x47");

    std::string_view hdtPayload = "HCHDT,215.3,T";
    uint8_t hdtChecksum = calculateChecksum(hdtPayload);
    TEST_ASSERT(hdtChecksum == 0x2B, "HDT Checksum matches 0x2B");

    // 2. Validate valid sentences
    std::string validGGA = "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47\r\n";
    TEST_ASSERT(validateChecksum(validGGA), "Valid GGA sentence validates successfully");

    std::string validHDT = "$HCHDT,215.3,T*2B\r\n";
    TEST_ASSERT(validateChecksum(validHDT), "Valid HDT sentence validates successfully");

    // 3. Rejection of corrupted sentences
    std::string badChecksumSentence = "$HCHDT,215.3,T*2C\r\n";
    TEST_ASSERT(!validateChecksum(badChecksumSentence), "Mismatched checksum is rejected");

    std::string bitFlipPayload = "$HCHDT,215.4,T*2B\r\n";
    TEST_ASSERT(!validateChecksum(bitFlipPayload), "Payload bit-flip is detected and rejected");

    std::string missingPrefix = "HCHDT,215.3,T*2B\r\n";
    TEST_ASSERT(!validateChecksum(missingPrefix), "Missing '$' prefix is rejected");

    std::string missingStar = "$HCHDT,215.3,T2B\r\n";
    TEST_ASSERT(!validateChecksum(missingStar), "Missing '*' delimiter is rejected");

    std::string truncatedHex = "$HCHDT,215.3,T*2\r\n";
    TEST_ASSERT(!validateChecksum(truncatedHex), "Truncated checksum hex is rejected");

    std::string invalidHex = "$HCHDT,215.3,T*ZZ\r\n";
    TEST_ASSERT(!validateChecksum(invalidHex), "Non-hex checksum characters are rejected");

    // 4. Sentence Formatter
    std::vector<std::string> radarFields{"01", "12.4", "045.2", "T", "18.5", "N"};
    std::string formattedRadar = formatSentence("RA", "TTM", radarFields);
    TEST_ASSERT(validateChecksum(formattedRadar), "Formatted radar sentence has valid checksum");
    TEST_ASSERT(formattedRadar.find("$RATTM,01,12.4,045.2,T,18.5,N*") == 0, "Formatted radar prefix matches");

    // 5. Message Parsing
    NetworkMessage parsedRadar = NetworkMessage::parse(formattedRadar);
    TEST_ASSERT(parsedRadar.isValid, "Parsed formatted sentence is valid");
    TEST_ASSERT(parsedRadar.talkerId == "RA", "Talker ID extracted as RA");
    TEST_ASSERT(parsedRadar.sentenceType == "TTM", "Sentence type extracted as TTM");
    TEST_ASSERT(parsedRadar.fields.size() == 6, "Extracted 6 field arguments");
    TEST_ASSERT(parsedRadar.fields[0] == "01", "Field 0 is target 01");
    TEST_ASSERT(parsedRadar.fields[1] == "12.4", "Field 1 is range 12.4");
}

// ============================================================================
// Test 4: Multi-Threaded Asynchronous ShipDataBus
// ============================================================================
void test_ship_data_bus_concurrency() {
    TEST_SECTION("ShipDataBus Multi-Threaded Asynchronous Pub/Sub Routing");

    ShipDataBus bus(5000);
    bus.start();

    // 1. Topic/Talker Subscription Filtering
    std::atomic<int> gpsCount{0};
    std::atomic<int> gyroCount{0};
    std::atomic<int> wildcardCount{0};

    auto gpsSubId = bus.subscribe(Talkers::GPS_GNSS, [&](const NetworkMessage& msg) {
        if (msg.talkerId == Talkers::GPS_GNSS) {
            gpsCount++;
        }
    });

    auto gyroSubId = bus.subscribe(Talkers::GYRO_COMPASS, [&](const NetworkMessage& msg) {
        if (msg.talkerId == Talkers::GYRO_COMPASS) {
            gyroCount++;
        }
    });

    auto wildcardSubId = bus.subscribe(Talkers::WILDCARD_ALL, [&](const NetworkMessage& msg) {
        (void)msg;
        wildcardCount++;
    });

    // 2. Multi-threaded Producers
    constexpr int MESSAGES_PER_PRODUCER = 250;
    
    auto gpsPublisher = [&bus]() {
        for (int i = 0; i < MESSAGES_PER_PRODUCER; ++i) {
            std::string sentence = formatSentence("GP", "GGA", {"123456", "1300.00", "N", "08000.00", "E", "1", "08"});
            bus.publishRaw(sentence);
            std::this_thread::yield();
        }
    };

    auto gyroPublisher = [&bus]() {
        for (int i = 0; i < MESSAGES_PER_PRODUCER; ++i) {
            std::string sentence = formatSentence("HE", "HDT", {std::to_string(i % 360), "T"});
            bus.publishRaw(sentence);
            std::this_thread::yield();
        }
    };

    auto radarPublisher = [&bus]() {
        for (int i = 0; i < MESSAGES_PER_PRODUCER; ++i) {
            std::string sentence = formatSentence("RA", "TTM", {std::to_string(i), "5.2", "120.0", "T"});
            bus.publishRaw(sentence);
            std::this_thread::yield();
        }
    };

    auto weatherPublisher = [&bus]() {
        for (int i = 0; i < MESSAGES_PER_PRODUCER; ++i) {
            std::string sentence = formatSentence("WI", "MWV", {"045.0", "R", "12.5", "N", "A"});
            bus.publishRaw(sentence);
            std::this_thread::yield();
        }
    };

    // Launch 4 concurrent producers
    std::thread t1(gpsPublisher);
    std::thread t2(gyroPublisher);
    std::thread t3(radarPublisher);
    std::thread t4(weatherPublisher);

    t1.join();
    t2.join();
    t3.join();
    t4.join();

    // Allow background worker thread to process queued messages
    std::this_thread::sleep_for(std::chrono::milliseconds(250));

    // Verify correct counts
    TEST_ASSERT(gpsCount.load() == MESSAGES_PER_PRODUCER, "GPS subscriber received all GPS messages");
    TEST_ASSERT(gyroCount.load() == MESSAGES_PER_PRODUCER, "Gyro subscriber received all Gyro messages");
    TEST_ASSERT(wildcardCount.load() == (4 * MESSAGES_PER_PRODUCER), "Wildcard subscriber received all 1,000 messages");

    // 3. Test Unsubscription
    bus.unsubscribe(gpsSubId);
    bus.publishRaw(formatSentence("GP", "GGA", {"999999", "1300.00", "N"}));
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    TEST_ASSERT(gpsCount.load() == MESSAGES_PER_PRODUCER, "Unsubscribed GPS listener did not receive new message");
    TEST_ASSERT(wildcardCount.load() == (4 * MESSAGES_PER_PRODUCER) + 1, "Wildcard listener received post-unsub message");

    // 4. Graceful Shutdown & Queue Drain
    bus.stop(true);
    TEST_ASSERT(!bus.isRunning(), "Bus stopped successfully");
    TEST_ASSERT(bus.queueSize() == 0, "Bus queue fully drained upon stop");
}

// ============================================================================
// Main Entry Point
// ============================================================================
int main() {
    std::cout << "========================================================\n"
              << " Distributed Tactical Defense & SDN Simulator - Phase 1\n"
              << " Modern C++20 Test Suite & Numerical Verification\n"
              << "========================================================\n";

    test_vector_basics_and_operations();
    test_vector_numerical_edge_cases();
    test_euler_and_los_kinematics();
    test_nmea_checksum_and_parsing();
    test_ship_data_bus_concurrency();

    std::cout << "\n========================================================\n"
              << " TEST SUMMARY\n"
              << " Total Assertions: " << g_totalTests << "\n"
              << " Passed:           " << g_passedTests << "\n"
              << " Failed:           " << g_failedTests << "\n"
              << "========================================================\n";

    if (g_failedTests == 0) {
        std::cout << "\n>>> ALL PHASE 1 VERIFICATION TESTS PASSED SUCCESSFULLY! <<<\n\n";
        return 0;
    } else {
        std::cerr << "\n>>> VERIFICATION FAILED WITH " << g_failedTests << " ERRORS! <<<\n\n";
        return 1;
    }
}
