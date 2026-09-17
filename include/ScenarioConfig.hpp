/**
 * @file ScenarioConfig.hpp
 * @brief Declarative scenario configuration schema, parameters, and JSON ingestion parser.
 */

#pragma once

#include "Math3D.hpp"

#include <string>
#include <vector>

namespace SDN::Config {

/**
 * @struct SimulationParams
 * @brief Global execution parameters.
 */
struct SimulationParams {
    std::string name{"Default Naval Defense Scenario"};
    int tickRateHz{100};
    double durationS{300.0};
    unsigned int seed{42};
};

/**
 * @struct RadarConfig
 * @brief Specification of ownship phased array radar sensor.
 */
struct RadarConfig {
    std::string id{"RDR-01"};
    double transmitterPowerW{60000.0};
    double frequencyHz{9.4e9};
    double antennaGainDbi{38.0};
    double noiseFloorDbm{-100.0};
    std::string scanMode{"SURVEILLANCE_360"};
    double scanRateDegS{90.0};
    double azimuthBeamwidthDeg{2.0};
    double elevationBeamwidthDeg{30.0};
};

/**
 * @struct OwnshipConfig
 * @brief Ownship platform initial conditions and installed combat suite.
 */
struct OwnshipConfig {
    std::string id{"DDG-1000"};
    Math::Vector3D position{0.0, 0.0, -25.0};
    Math::Vector3D velocity{0.0, 0.0, 0.0};
    RadarConfig radar{};
};

/**
 * @struct TargetConfig
 * @brief Threat entity instantiation template.
 */
struct TargetConfig {
    std::string id{"TGT-01"};
    std::string type{"UAV"}; // UAV, CRUISE_MISSILE, BALLISTIC
    double spawnTimeS{0.0};
    Math::Vector3D initialPosition{14142.0, 14142.0, -1500.0};
    Math::Vector3D initialVelocity{-127.3, -127.3, 0.0};
    double speedMps{180.0};
    double rcsM2{0.5};
    std::string evasionProfile{"NONE"}; // NONE, WEAVE_6G, BARREL_ROLL
    double evasionFrequencyHz{0.5};
    double evasionAmplitudeG{6.0};
    bool activeJamming{false};
    double jammerPowerW{0.0};
};

/**
 * @struct DefenseDoctrineConfig
 * @brief Automated Combat Management System rules of engagement.
 */
struct DefenseDoctrineConfig {
    int interceptorCount{8};
    std::string doctrineMode{"AUTO_DEFENSE"}; // AUTO_DEFENSE or MANUAL_AUTH
    double maxEngagementRangeM{15000.0};
    double proportionalNavGain{4.0};
    double maxGLimit{35.0};
    double proximityFuseRadiusM{12.0};
};

/**
 * @struct ScenarioConfig
 * @brief Master composite scenario descriptor.
 */
struct ScenarioConfig {
    SimulationParams simulation{};
    OwnshipConfig ownship{};
    DefenseDoctrineConfig defenseDoctrine{};
    std::vector<TargetConfig> targets{};

    [[nodiscard]] static ScenarioConfig loadFromFile(const std::string& filepath, std::string& outError);
    [[nodiscard]] static ScenarioConfig loadFromJsonString(const std::string& json, std::string& outError);
    [[nodiscard]] static ScenarioConfig createDefault();
    [[nodiscard]] std::string toJsonString() const;
};

} // namespace SDN::Config
