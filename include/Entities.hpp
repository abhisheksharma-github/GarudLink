/**
 * @file Entities.hpp
 * @brief Production-grade kinematic, physical, sensor, and GNC tactical entity models.
 * 
 * Features:
 * - Multi-Threat Targets with Swerling Fluctuation & ECM Noise Jamming
 * - 4/3 Earth Curvature Horizon & CFAR Radar Detection
 * - Multi-Mode Phased Array Radar (SURVEILLANCE_360, SECTOR_CUE, STT_TRACK_LOCK)
 * - Three-Phase Rocket Interceptor (Booster, Sustainer, Coast) with Seeker Gimbal Constraints (FOV +-35 deg)
 * - Proportional Navigation (PN) Guidance with G-Saturation and Proximity Blast Lethality
 */

#pragma once

#include "Math3D.hpp"
#include "ShipDataBus.hpp"

#include <cmath>
#include <memory>
#include <string>
#include <string_view>
#include <vector>

namespace SDN::Entities {

/**
 * @enum Classification
 * @brief Tactical identification friend-or-foe (IFF) classification.
 */
enum class Classification {
    FRIENDLY,
    HOSTILE,
    UNKNOWN
};

[[nodiscard]] std::string classificationToString(Classification classification);
[[nodiscard]] Classification stringToClassification(const std::string& str);

/**
 * @enum SwerlingModel
 * @brief Statistical radar target fluctuation models.
 */
enum class SwerlingModel {
    NONE_0,      // Non-fluctuating target
    SWERLING_1,  // Slow Rayleigh fluctuation (scan-to-scan constant, slow drones)
    SWERLING_2,  // Fast Rayleigh fluctuation (pulse-to-pulse)
    SWERLING_3,  // Slow 4-DOF Chi-Square fluctuation (missiles with dominant scatterers)
    SWERLING_4   // Fast 4-DOF Chi-Square fluctuation
};

/**
 * @enum MissileMotorPhase
 * @brief Propulsion stages of multi-phase solid rocket motor.
 */
enum class MissileMotorPhase {
    BOOSTER,   // 0.0s - 3.0s: High thrust acceleration
    SUSTAINER, // 3.0s - 7.0s: Sustaining thrust
    COAST      // > 7.0s: Motor burnt out, aerodynamic velocity bleed
};

[[nodiscard]] inline std::string motorPhaseToString(MissileMotorPhase phase) {
    switch (phase) {
        case MissileMotorPhase::BOOSTER:   return "BOOSTER";
        case MissileMotorPhase::SUSTAINER: return "SUSTAINER";
        case MissileMotorPhase::COAST:     return "COAST";
    }
    return "COAST";
}

/**
 * @class Entity
 * @brief Abstract base class for all physical simulated tactical entities.
 */
class Entity {
public:
    Entity(std::string id, Classification classification, 
           Math::Vector3D position = {}, Math::Vector3D velocity = {}, 
           double massKg = 100.0);

    virtual ~Entity() = default;

    Entity(const Entity&) = default;
    Entity& operator=(const Entity&) = default;
    Entity(Entity&&) noexcept = default;
    Entity& operator=(Entity&&) noexcept = default;

    virtual void update(double dt) = 0;
    [[nodiscard]] virtual std::string serializeTelemetry() const = 0;
    [[nodiscard]] virtual std::string getTalkerId() const = 0;

    // Kinematic & State Accessors
    [[nodiscard]] const std::string& getId() const noexcept { return id_; }
    [[nodiscard]] Classification getClassification() const noexcept { return classification_; }
    void setClassification(Classification c) noexcept { classification_ = c; }

    [[nodiscard]] const Math::Vector3D& getPosition() const noexcept { return position_; }
    void setPosition(const Math::Vector3D& pos) noexcept { position_ = pos; }

    [[nodiscard]] const Math::Vector3D& getVelocity() const noexcept { return velocity_; }
    void setVelocity(const Math::Vector3D& vel) noexcept { velocity_ = vel; }

    [[nodiscard]] const Math::Vector3D& getAcceleration() const noexcept { return acceleration_; }
    void setAcceleration(const Math::Vector3D& acc) noexcept { acceleration_ = acc; }

    [[nodiscard]] double getMass() const noexcept { return massKg_; }
    void setMass(double massKg) noexcept { massKg_ = massKg; }

    [[nodiscard]] bool isActive() const noexcept { return isActive_; }
    void setActive(bool active) noexcept { isActive_ = active; }

    [[nodiscard]] double getSpeed() const noexcept { return velocity_.magnitude(); }
    [[nodiscard]] double getHeadingDeg() const noexcept;
    [[nodiscard]] double getAltitudeM() const noexcept { return -position_.z; } // NED: Z-down

protected:
    void integrateKinematics(double dt) noexcept;

    std::string id_;
    Classification classification_{Classification::UNKNOWN};
    Math::Vector3D position_{0.0, 0.0, 0.0};
    Math::Vector3D velocity_{0.0, 0.0, 0.0};
    Math::Vector3D acceleration_{0.0, 0.0, 0.0};
    double massKg_{100.0};
    bool isActive_{true};
};

// ============================================================================
// Enhanced Drone / Threat Entity
// ============================================================================

/**
 * @class Drone
 * @brief Tactical UAV, Kamikaze Drone, or Anti-Ship Missile with Waypoint & EW logic.
 */
class Drone : public Entity {
public:
    Drone(std::string id, Classification classification,
          Math::Vector3D initialPos, double cruiseSpeedMps = 60.0,
          double rcsM2 = 0.5, double massKg = 250.0,
          std::string threatType = "GENERIC_UAV",
          SwerlingModel swerling = SwerlingModel::SWERLING_1);

    void update(double dt) override;
    [[nodiscard]] std::string serializeTelemetry() const override;
    [[nodiscard]] std::string getTalkerId() const override { return "UAV"; }

    // Threat & Classification Attributes
    [[nodiscard]] const std::string& getThreatType() const noexcept { return threatType_; }
    void setThreatType(std::string type) { threatType_ = std::move(type); }

    [[nodiscard]] SwerlingModel getSwerlingModel() const noexcept { return swerlingModel_; }
    void setSwerlingModel(SwerlingModel sm) noexcept { swerlingModel_ = sm; }

    // Waypoint Navigation
    void setWaypoints(std::vector<Math::Vector3D> waypoints) {
        waypoints_ = std::move(waypoints);
        currentWaypointIdx_ = 0;
    }
    void addWaypoint(const Math::Vector3D& wp) { waypoints_.push_back(wp); }
    void clearWaypoints() { waypoints_.clear(); currentWaypointIdx_ = 0; }
    [[nodiscard]] const std::vector<Math::Vector3D>& getWaypoints() const noexcept { return waypoints_; }
    [[nodiscard]] size_t getCurrentWaypointIndex() const noexcept { return currentWaypointIdx_; }

    // Kinematic Configuration
    void setCruiseSpeed(double speedMps) noexcept { cruiseSpeedMps_ = speedMps; }
    [[nodiscard]] double getCruiseSpeed() const noexcept { return cruiseSpeedMps_; }

    void setTurnRateLimitDegPerSec(double limitDeg) noexcept { turnRateLimitDegPerSec_ = limitDeg; }
    [[nodiscard]] double getTurnRateLimitDegPerSec() const noexcept { return turnRateLimitDegPerSec_; }

    void setWaypointAcceptanceRadius(double radiusM) noexcept { waypointRadiusM_ = radiusM; }
    [[nodiscard]] double getWaypointAcceptanceRadius() const noexcept { return waypointRadiusM_; }

    void setRcs(double rcsM2) noexcept { rcsM2_ = rcsM2; }
    [[nodiscard]] double getRcs() const noexcept { return rcsM2_; }

    // Evasive Maneuvers (Sinusoidal Weave / High-G Barrel Roll)
    void setEvasiveMode(bool enable) noexcept { evasiveMode_ = enable; }
    [[nodiscard]] bool isEvasiveModeEnabled() const noexcept { return evasiveMode_; }
    void setEvasiveParameters(double frequencyHz, double amplitudeG) noexcept {
        evasiveFrequencyHz_ = frequencyHz;
        evasiveAmplitudeG_ = amplitudeG;
    }

    // Electronic Countermeasures (ECM / Active Noise Jamming)
    void setJammingEnabled(bool enable) noexcept { isJamming_ = enable; }
    [[nodiscard]] bool isJammingEnabled() const noexcept { return isJamming_; }
    void setJammerPowerWatts(double watts) noexcept { jammerPowerWatts_ = watts; }
    [[nodiscard]] double getJammerPowerWatts() const noexcept { return jammerPowerWatts_; }

    // Fluctuating instantaneous RCS calculation
    [[nodiscard]] double getInstantaneousRcs() const;

private:
    std::string threatType_{"GENERIC_UAV"};
    SwerlingModel swerlingModel_{SwerlingModel::SWERLING_1};
    double rcsM2_{0.5};
    std::vector<Math::Vector3D> waypoints_;
    size_t currentWaypointIdx_{0};
    double waypointRadiusM_{50.0};
    double cruiseSpeedMps_{60.0};
    double turnRateLimitDegPerSec_{30.0};

    // Evasive settings
    bool evasiveMode_{false};
    double evasiveFrequencyHz_{0.5};
    double evasiveAmplitudeG_{4.0};
    double evasiveTimer_{0.0};

    // ECM Jamming settings
    bool isJamming_{false};
    double jammerPowerWatts_{500.0}; // 500W active RF barrage jamming
};

// ============================================================================
// Physical Multi-Mode Radar System with 4/3 Earth Horizon & CFAR
// ============================================================================

/**
 * @enum RadarOperationalMode
 * @brief Sensor sweep modes.
 */
enum class RadarOperationalMode {
    SURVEILLANCE_360,
    SECTOR_CUE,
    STT_TRACK_LOCK
};

/**
 * @struct RadarDetection
 * @brief Registered contact metadata from the radar beam.
 */
struct RadarDetection {
    std::string targetId;
    double azimuthDeg{0.0};
    double elevationDeg{0.0};
    double rangeM{0.0};
    double snrDb{0.0};
    double receivedPowerWatts{0.0};
    bool detected{false};
    bool horizonMasked{false};
    bool jammed{false};
};

/**
 * @class RadarSystem
 * @brief Production 3D phased array radar with 4/3 Earth Horizon, CFAR, and Swerling target modeling.
 */
class RadarSystem : public Entity {
public:
    RadarSystem(std::string id, Classification classification,
                Math::Vector3D position,
                double txPowerWatts = 60000.0,       // 60 kW Peak Power
                double antennaGainDbi = 38.0,        // 38 dBi Gain
                double frequencyHz = 9.4e9,          // 9.4 GHz (X-Band)
                double minDetectableSignalWatts = 1e-13, // -100 dBm Sensitivity
                double azBeamwidthDeg = 2.0,         // 2.0 deg Azimuth Beam
                double elBeamwidthDeg = 30.0,        // 30.0 deg Fan Coverage
                double scanRateDegPerSec = 90.0);    // 15 RPM (90 deg/s)

    void update(double dt) override;
    [[nodiscard]] std::string serializeTelemetry() const override;
    [[nodiscard]] std::string getTalkerId() const override { return "RDR"; }

    // Radar Mode Tasking
    void setMode(RadarOperationalMode mode) noexcept { mode_ = mode; }
    [[nodiscard]] RadarOperationalMode getMode() const noexcept { return mode_; }

    void setSectorCue(double cueAzimuthDeg, double sectorWidthDeg = 45.0) noexcept {
        mode_ = RadarOperationalMode::SECTOR_CUE;
        cueAzimuthDeg_ = cueAzimuthDeg;
        sectorWidthDeg_ = sectorWidthDeg;
    }

    void setSttLock(std::string targetId) noexcept {
        mode_ = RadarOperationalMode::STT_TRACK_LOCK;
        lockedTargetId_ = std::move(targetId);
    }

    [[nodiscard]] double getCueAzimuthDeg() const noexcept { return cueAzimuthDeg_; }
    [[nodiscard]] double getSectorWidthDeg() const noexcept { return sectorWidthDeg_; }
    [[nodiscard]] const std::string& getLockedTargetId() const noexcept { return lockedTargetId_; }

    // 4/3 Earth Curvature Horizon
    [[nodiscard]] double calculateRadarHorizonM(double targetAltitudeM) const noexcept;
    [[nodiscard]] bool isTargetAboveHorizon(double targetAltitudeM, double rangeM) const noexcept;

    // Target Interrogation & CFAR
    [[nodiscard]] RadarDetection interrogateTarget(const Entity& target, double targetRcsM2, 
                                                   bool targetJamming = false, double jamPowerW = 0.0) const noexcept;

    [[nodiscard]] std::vector<std::string> scanTargets(
        const std::vector<std::shared_ptr<Entity>>& targets
    );

    [[nodiscard]] std::string formatTrackSentence(const RadarDetection& det) const;

    // Accessors
    [[nodiscard]] double getCurrentAzimuthDeg() const noexcept { return currentAzimuthDeg_; }
    void setCurrentAzimuthDeg(double azDeg) noexcept;
    [[nodiscard]] double getTxPowerWatts() const noexcept { return txPowerWatts_; }
    [[nodiscard]] double getAntennaGainDbi() const noexcept { return antennaGainDbi_; }
    [[nodiscard]] double getWavelengthM() const noexcept { return wavelengthM_; }
    [[nodiscard]] double getCfarThresholdDb() const noexcept { return cfarThresholdDb_; }
    void setCfarThresholdDb(double db) noexcept { cfarThresholdDb_ = db; }

private:
    double txPowerWatts_{60000.0};
    double antennaGainDbi_{38.0};
    double antennaGainLinear_{6309.57};
    double frequencyHz_{9.4e9};
    double wavelengthM_{0.03189};
    double minDetectableSignalWatts_{1e-13};
    double azBeamwidthDeg_{2.0};
    double elBeamwidthDeg_{30.0};
    double scanRateDegPerSec_{90.0};
    double currentAzimuthDeg_{0.0};

    // Mode state
    RadarOperationalMode mode_{RadarOperationalMode::SURVEILLANCE_360};
    double cueAzimuthDeg_{0.0};
    double sectorWidthDeg_{45.0};
    double sectorScanDirection_{1.0}; // +1 or -1 for oscillating sweep
    std::string lockedTargetId_;

    // CFAR & Noise Parameters
    double cfarThresholdDb_{10.0}; // 10 dB SNR detection threshold
    static constexpr double EFFECTIVE_EARTH_RADIUS_M = 8494700.0; // 4/3 * 6371 km
};

// ============================================================================
// Multi-Stage Guided Interceptor Missile with Seeker Gimbal Constraints
// ============================================================================

/**
 * @struct FlightDataPoint
 * @brief Telemetry point recorded for After-Action Review (AAR).
 */
struct FlightDataPoint {
    double timestampSec{0.0};
    std::string targetId;
    std::string interceptorId;
    double slantRangeM{0.0};
    double closingVelocityMps{0.0};
    double losRateRadS{0.0};
    double seekerLookAngleDeg{0.0};
    double missileSpeedMps{0.0};
    std::string motorStage;
    std::string outcome;
};

/**
 * @class GuidedMissile
 * @brief High-G Surface-to-Air Interceptor with 3-Stage Motor, Seeker FOV, and TPN Guidance.
 */
class GuidedMissile : public Entity {
public:
    GuidedMissile(std::string id, Classification classification,
                  Math::Vector3D initialPos, Math::Vector3D initialVel,
                  std::string targetId,
                  double boosterTimeSec = 3.0,
                  double sustainerTimeSec = 4.0,
                  double boosterThrustN = 24000.0,
                  double sustainerThrustN = 6000.0,
                  double dryMassKg = 40.0,
                  double propellantMassKg = 45.0,
                  double navRatioN = 4.0,
                  double maxGForceLimit = 35.0,
                  double proximityFuseRadiusM = 12.0);

    void update(double dt) override;
    [[nodiscard]] std::string serializeTelemetry() const override;
    [[nodiscard]] std::string getTalkerId() const override { return "MSL"; }

    void setTargetKinematics(const Math::Vector3D& targetPos, const Math::Vector3D& targetVel) noexcept;

    // Guidance & Intercept Queries
    [[nodiscard]] const std::string& getTargetId() const noexcept { return targetId_; }
    [[nodiscard]] bool hasTargetLock() const noexcept { return hasTargetLock_; }
    [[nodiscard]] bool hasDetonated() const noexcept { return hasDetonated_; }
    [[nodiscard]] bool isTargetIntercepted() const noexcept { return targetIntercepted_; }
    [[nodiscard]] double getRangeToGo() const noexcept { return lastRangeToGo_; }
    [[nodiscard]] double getClosingVelocity() const noexcept { return lastClosingVel_; }
    [[nodiscard]] double getLosRateRadS() const noexcept { return lastLosRateRadS_; }
    [[nodiscard]] double getSeekerLookAngleDeg() const noexcept { return lastSeekerLookAngleDeg_; }
    [[nodiscard]] MissileMotorPhase getMotorPhase() const noexcept { return motorPhase_; }
    [[nodiscard]] double getMissDistanceM() const noexcept { return missDistanceM_; }
    [[nodiscard]] const std::string& getEngagementOutcome() const noexcept { return engagementOutcome_; }

    [[nodiscard]] const std::vector<FlightDataPoint>& getFlightRecorder() const noexcept { return flightHistory_; }

    // Configuration
    void setNavRatio(double n) noexcept { navRatioN_ = n; }
    void setMaxGForceLimit(double gLimit) noexcept { maxGForceLimit_ = gLimit; }
    void setProximityFuseRadius(double radiusM) noexcept { proximityFuseRadiusM_ = radiusM; }

private:
    std::string targetId_;
    Math::Vector3D targetPosition_{0.0, 0.0, 0.0};
    Math::Vector3D targetVelocity_{0.0, 0.0, 0.0};
    bool hasTargetLock_{false};

    // 3-Phase Solid Rocket Propulsion
    double boosterTimeSec_{3.0};
    double sustainerTimeSec_{4.0};
    double boosterThrustN_{24000.0};
    double sustainerThrustN_{6000.0};
    double dryMassKg_{40.0};
    double boosterPropellantKg_{30.0};
    double sustainerPropellantKg_{15.0};
    double flightTimer_{0.0};
    MissileMotorPhase motorPhase_{MissileMotorPhase::BOOSTER};

    // Guidance & Seeker Constraints
    double navRatioN_{4.0};
    double maxGForceLimit_{35.0};
    double proximityFuseRadiusM_{12.0};
    double seekerFovDeg_{35.0}; // Conical Field-of-View (+-35 deg)

    // Flight Telemetry
    bool hasDetonated_{false};
    bool targetIntercepted_{false};
    double lastRangeToGo_{0.0};
    double lastClosingVel_{0.0};
    double lastLosRateRadS_{0.0};
    double lastSeekerLookAngleDeg_{0.0};
    double closestDistanceM_{1e9};
    double missDistanceM_{0.0};
    std::string engagementOutcome_{"IN_FLIGHT"};

    // AAR Recorder
    std::vector<FlightDataPoint> flightHistory_;
};

} // namespace SDN::Entities
