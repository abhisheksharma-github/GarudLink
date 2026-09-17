/**
 * @file Entities.cpp
 * @brief Implementation of high-precision tactical entities, radar physics, and multi-stage GNC interceptor.
 */

#include "Entities.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <random>
#include <sstream>

namespace SDN::Entities {

namespace {
    constexpr double PI_VAL = 3.14159265358979323846;
    constexpr double DEG_TO_RAD = PI_VAL / 180.0;
    constexpr double RAD_TO_DEG = 180.0 / PI_VAL;
    constexpr double GRAVITY_G = 9.80665;
    constexpr double AIR_DENSITY_SEA_LEVEL = 1.225; // kg/m^3
}

std::string classificationToString(Classification classification) {
    switch (classification) {
        case Classification::FRIENDLY: return "FRIENDLY";
        case Classification::HOSTILE:  return "HOSTILE";
        case Classification::UNKNOWN:  return "UNKNOWN";
    }
    return "UNKNOWN";
}

Classification stringToClassification(const std::string& str) {
    if (str == "FRIENDLY") return Classification::FRIENDLY;
    if (str == "HOSTILE")  return Classification::HOSTILE;
    return Classification::UNKNOWN;
}

// ============================================================================
// Base Entity Implementation
// ============================================================================

Entity::Entity(std::string id, Classification classification,
               Math::Vector3D position, Math::Vector3D velocity,
               double massKg)
    : id_(std::move(id)), classification_(classification),
      position_(position), velocity_(velocity),
      massKg_(std::max(0.1, massKg)) {}

void Entity::integrateKinematics(double dt) noexcept {
    // Second-order Velocity-Verlet / kinematic step
    position_ += (velocity_ * dt) + (acceleration_ * (0.5 * dt * dt));
    velocity_ += acceleration_ * dt;
}

double Entity::getHeadingDeg() const noexcept {
    // In NED frame, Azimuth/Heading = atan2(East/Y, North/X) in degrees
    const double headingRad = std::atan2(velocity_.y, velocity_.x);
    double headingDeg = headingRad * RAD_TO_DEG;
    if (headingDeg < 0.0) {
        headingDeg += 360.0;
    }
    return headingDeg;
}

// ============================================================================
// Enhanced Drone & Threat Entity Implementation
// ============================================================================

Drone::Drone(std::string id, Classification classification,
             Math::Vector3D initialPos, double cruiseSpeedMps,
             double rcsM2, double massKg,
             std::string threatType, SwerlingModel swerling)
    : Entity(std::move(id), classification, initialPos, {}, massKg),
      threatType_(std::move(threatType)), swerlingModel_(swerling),
      rcsM2_(rcsM2), cruiseSpeedMps_(cruiseSpeedMps) {
    // Initial velocity towards origin or forward
    velocity_ = Math::Vector3D{-cruiseSpeedMps, 0.0, 0.0};
}

double Drone::getInstantaneousRcs() const {
    if (rcsM2_ <= 0.0) return 0.001;

    static thread_local std::mt19937 rng(1337);
    std::uniform_real_distribution<double> dist(0.001, 0.999);

    switch (swerlingModel_) {
        case SwerlingModel::NONE_0:
            return rcsM2_;

        case SwerlingModel::SWERLING_1:
        case SwerlingModel::SWERLING_2: {
            // Exponential / Rayleigh fading: sigma = -avg * ln(u)
            double u = dist(rng);
            return std::max(0.001, -rcsM2_ * std::log(u));
        }

        case SwerlingModel::SWERLING_3:
        case SwerlingModel::SWERLING_4: {
            // Chi-Square 4-DOF: sigma = avg * (-ln(u1 * u2)) / 2
            double u1 = dist(rng);
            double u2 = dist(rng);
            return std::max(0.001, rcsM2_ * (-std::log(u1 * u2)) * 0.5);
        }
    }
    return rcsM2_;
}

void Drone::update(double dt) {
    if (!isActive_) return;

    Math::Vector3D targetPos = position_;
    bool hasTargetWaypoint = false;

    if (!waypoints_.empty() && currentWaypointIdx_ < waypoints_.size()) {
        targetPos = waypoints_[currentWaypointIdx_];
        hasTargetWaypoint = true;

        // Check if reached waypoint
        if (position_.distanceTo(targetPos) <= waypointRadiusM_) {
            currentWaypointIdx_ = (currentWaypointIdx_ + 1) % waypoints_.size();
        }
    }

    // Desired steering vector
    Math::Vector3D desiredDir = (hasTargetWaypoint) ? (targetPos - position_).normalized() 
                                                    : velocity_.normalized();
    if (desiredDir.isZero()) {
        desiredDir = Math::Vector3D{-1.0, 0.0, 0.0};
    }

    Math::Vector3D desiredVel = desiredDir * cruiseSpeedMps_;

    // Heading steering control
    Math::Vector3D velDiff = desiredVel - velocity_;
    double maxSpeedChange = (turnRateLimitDegPerSec_ * DEG_TO_RAD * cruiseSpeedMps_) * dt;
    if (velDiff.magnitude() > maxSpeedChange && maxSpeedChange > 0.0) {
        velDiff = velDiff.normalized() * maxSpeedChange;
    }
    velocity_ += velDiff;

    // Normalizing speed to cruise
    if (velocity_.magnitude() > 0.1) {
        velocity_ = velocity_.normalized() * cruiseSpeedMps_;
    }

    // High-G Evasive Weave / Barrel Roll Acceleration
    acceleration_ = Math::Vector3D{0.0, 0.0, 0.0};
    if (evasiveMode_) {
        evasiveTimer_ += dt;
        double lateralFactor = std::sin(2.0 * PI_VAL * evasiveFrequencyHz_ * evasiveTimer_);
        double evasiveAccelMag = evasiveAmplitudeG_ * GRAVITY_G * lateralFactor;

        // Vector orthogonal to velocity in horizontal plane
        Math::Vector3D lateralUnit{-velocity_.y, velocity_.x, 0.0};
        if (!lateralUnit.isZero()) {
            lateralUnit = lateralUnit.normalized();
            acceleration_ += lateralUnit * evasiveAccelMag;
        }
    }

    integrateKinematics(dt);
}

std::string Drone::serializeTelemetry() const {
    const double speed = getSpeed();
    const double heading = getHeadingDeg();

    std::ostringstream oss;
    oss << std::fixed << std::setprecision(1);
    oss << id_ << ","
        << classificationToString(classification_) << ","
        << position_.x << "," << position_.y << "," << position_.z << ","
        << velocity_.x << "," << velocity_.y << "," << velocity_.z << ","
        << speed << "," << heading << ","
        << (evasiveMode_ ? "1" : "0") << ","
        << (isJamming_ ? "1" : "0");

    return Network::formatSentence(getTalkerId(), "POS", {
        id_,
        classificationToString(classification_),
        std::to_string(static_cast<int>(position_.x)),
        std::to_string(static_cast<int>(position_.y)),
        std::to_string(static_cast<int>(position_.z)),
        std::to_string(static_cast<int>(speed)),
        std::to_string(static_cast<int>(heading))
    });
}

// ============================================================================
// Multi-Mode Radar System Implementation with 4/3 Earth Horizon & CFAR
// ============================================================================

RadarSystem::RadarSystem(std::string id, Classification classification,
                         Math::Vector3D position,
                         double txPowerWatts, double antennaGainDbi,
                         double frequencyHz, double minDetectableSignalWatts,
                         double azBeamwidthDeg, double elBeamwidthDeg,
                         double scanRateDegPerSec)
    : Entity(std::move(id), classification, position, {}, 5000.0),
      txPowerWatts_(txPowerWatts), antennaGainDbi_(antennaGainDbi),
      frequencyHz_(frequencyHz), minDetectableSignalWatts_(minDetectableSignalWatts),
      azBeamwidthDeg_(azBeamwidthDeg), elBeamwidthDeg_(elBeamwidthDeg),
      scanRateDegPerSec_(scanRateDegPerSec) {
    antennaGainLinear_ = std::pow(10.0, antennaGainDbi_ / 10.0);
    wavelengthM_ = 299792458.0 / frequencyHz_;
}

void RadarSystem::setCurrentAzimuthDeg(double azDeg) noexcept {
    currentAzimuthDeg_ = std::fmod(azDeg, 360.0);
    if (currentAzimuthDeg_ < 0.0) {
        currentAzimuthDeg_ += 360.0;
    }
}

void RadarSystem::update(double dt) {
    if (!isActive_) return;

    switch (mode_) {
        case RadarOperationalMode::SURVEILLANCE_360:
            // Continuous 360-degree rotation
            setCurrentAzimuthDeg(currentAzimuthDeg_ + (scanRateDegPerSec_ * dt));
            break;

        case RadarOperationalMode::SECTOR_CUE: {
            // High-rate 45-degree sector sweep centered at cueAzimuthDeg_
            double sweepSpeed = scanRateDegPerSec_ * 3.0; // 3x faster sector search
            currentAzimuthDeg_ += sectorScanDirection_ * sweepSpeed * dt;

            double halfSector = sectorWidthDeg_ * 0.5;
            double minAz = cueAzimuthDeg_ - halfSector;
            double maxAz = cueAzimuthDeg_ + halfSector;

            if (currentAzimuthDeg_ >= maxAz) {
                currentAzimuthDeg_ = maxAz;
                sectorScanDirection_ = -1.0;
            } else if (currentAzimuthDeg_ <= minAz) {
                currentAzimuthDeg_ = minAz;
                sectorScanDirection_ = 1.0;
            }
            break;
        }

        case RadarOperationalMode::STT_TRACK_LOCK:
            // Beam remains locked on dedicated target (azimuth updated during scan)
            break;
    }
}

double RadarSystem::calculateRadarHorizonM(double targetAltitudeM) const noexcept {
    double radarHeightM = std::max(25.0, -position_.z); // Minimum 25m mast height
    double tgtHeightM = std::max(10.0, targetAltitudeM); // Minimum 10m target altitude

    // 4/3 Earth Curvature Horizon: D = sqrt(2 * Re * h_radar) + sqrt(2 * Re * h_tgt)
    double dRadar = std::sqrt(2.0 * EFFECTIVE_EARTH_RADIUS_M * radarHeightM);
    double dTgt   = std::sqrt(2.0 * EFFECTIVE_EARTH_RADIUS_M * tgtHeightM);
    return dRadar + dTgt;
}

bool RadarSystem::isTargetAboveHorizon(double targetAltitudeM, double rangeM) const noexcept {
    double horizonLimitM = calculateRadarHorizonM(targetAltitudeM);
    return rangeM <= horizonLimitM;
}

RadarDetection RadarSystem::interrogateTarget(const Entity& target, double targetRcsM2,
                                             bool targetJamming, double jamPowerW) const noexcept {
    RadarDetection det;
    det.targetId = target.getId();

    const Math::Vector3D relPos = target.getPosition() - position_;
    det.rangeM = relPos.magnitude();

    if (det.rangeM < 1.0) {
        return det;
    }

    // Azimuth and Elevation to target
    double azRad = std::atan2(relPos.y, relPos.x);
    det.azimuthDeg = azRad * RAD_TO_DEG;
    if (det.azimuthDeg < 0.0) det.azimuthDeg += 360.0;

    double horizDist = std::sqrt(relPos.x * relPos.x + relPos.y * relPos.y);
    double elRad = std::atan2(-relPos.z, horizDist);
    det.elevationDeg = elRad * RAD_TO_DEG;

    // 1. Check 4/3 Earth Curvature Horizon
    double targetAltM = target.getAltitudeM();
    if (!isTargetAboveHorizon(targetAltM, det.rangeM)) {
        det.horizonMasked = true;
        det.detected = false;
        return det;
    }

    // 2. Beam Alignment Check
    bool inBeam = false;
    if (mode_ == RadarOperationalMode::STT_TRACK_LOCK && target.getId() == lockedTargetId_) {
        inBeam = true; // Dedicated lock beam
    } else {
        double deltaAz = std::abs(det.azimuthDeg - currentAzimuthDeg_);
        if (deltaAz > 180.0) deltaAz = 360.0 - deltaAz;
        inBeam = (deltaAz <= (azBeamwidthDeg_ * 1.5)) && (std::abs(det.elevationDeg) <= elBeamwidthDeg_);
    }

    if (!inBeam) {
        return det;
    }

    // 3. Physical Radar Range Equation: Pr = (Pt * G^2 * lambda^2 * sigma) / ((4*pi)^3 * R^4)
    const double fourPi = 4.0 * PI_VAL;
    const double fourPiCubed = fourPi * fourPi * fourPi;
    const double range4 = det.rangeM * det.rangeM * det.rangeM * det.rangeM;

    const double numerator = txPowerWatts_ * (antennaGainLinear_ * antennaGainLinear_) *
                             (wavelengthM_ * wavelengthM_) * std::max(0.001, targetRcsM2);
    det.receivedPowerWatts = numerator / (fourPiCubed * range4);

    // 4. Noise & Electronic Countermeasures (ECM) Jamming Calculation
    double noisePowerWatts = minDetectableSignalWatts_;
    if (targetJamming && jamPowerW > 0.0) {
        // Active Barrage Jamming received power: Pjam_rx = (Pjam * Gjam * Gradar * lambda^2) / ((4*pi*R)^2)
        double jamGainLinear = 2.0; // 3 dBi jammer horn
        double fourPiR = fourPi * det.rangeM;
        double jammerRxWatts = (jamPowerW * jamGainLinear * antennaGainLinear_ * wavelengthM_ * wavelengthM_) / 
                               (fourPiR * fourPiR);
        noisePowerWatts += jammerRxWatts;
        det.jammed = true;
    }

    det.snrDb = 10.0 * std::log10(std::max(1e-20, det.receivedPowerWatts) / std::max(1e-20, noisePowerWatts));

    // 5. CFAR Detection Threshold
    if (det.snrDb >= cfarThresholdDb_) {
        det.detected = true;
    }

    return det;
}

std::vector<std::string> RadarSystem::scanTargets(const std::vector<std::shared_ptr<Entity>>& targets) {
    std::vector<std::string> nmeaTracks;

    for (const auto& target : targets) {
        if (!target || !target->isActive() || target->getId() == id_) {
            continue;
        }

        double rcs = 0.5;
        bool jamming = false;
        double jamPower = 0.0;

        if (auto drone = std::dynamic_pointer_cast<Drone>(target)) {
            rcs = drone->getInstantaneousRcs();
            jamming = drone->isJammingEnabled();
            jamPower = drone->getJammerPowerWatts();
        }

        // Interrogate target
        RadarDetection det = interrogateTarget(*target, rcs, jamming, jamPower);
        if (det.detected) {
            if (mode_ == RadarOperationalMode::STT_TRACK_LOCK && target->getId() == lockedTargetId_) {
                currentAzimuthDeg_ = det.azimuthDeg;
            }
            nmeaTracks.push_back(formatTrackSentence(det));
        }
    }

    return nmeaTracks;
}

std::string RadarSystem::formatTrackSentence(const RadarDetection& det) const {
    std::ostringstream azStr, elStr, rngStr, snrStr;
    azStr << std::fixed << std::setprecision(1) << det.azimuthDeg;
    elStr << std::fixed << std::setprecision(1) << det.elevationDeg;
    rngStr << std::fixed << std::setprecision(0) << det.rangeM;
    snrStr << std::fixed << std::setprecision(1) << det.snrDb;

    return Network::formatSentence(getTalkerId(), "TRK", {
        det.targetId,
        azStr.str(),
        elStr.str(),
        rngStr.str(),
        snrStr.str()
    });
}

std::string RadarSystem::serializeTelemetry() const {
    std::ostringstream azStr;
    azStr << std::fixed << std::setprecision(1) << currentAzimuthDeg_;

    return Network::formatSentence(getTalkerId(), "STATUS", {
        id_,
        (mode_ == RadarOperationalMode::SURVEILLANCE_360 ? "SURV_360" :
         (mode_ == RadarOperationalMode::SECTOR_CUE ? "SECTOR" : "STT_LOCK")),
        azStr.str()
    });
}

// ============================================================================
// Multi-Stage Interceptor Missile Implementation (TPN Guidance + Gimbal FOV)
// ============================================================================

GuidedMissile::GuidedMissile(std::string id, Classification classification,
                             Math::Vector3D initialPos, Math::Vector3D initialVel,
                             std::string targetId,
                             double boosterTimeSec, double sustainerTimeSec,
                             double boosterThrustN, double sustainerThrustN,
                             double dryMassKg, double propellantMassKg,
                             double navRatioN, double maxGForceLimit,
                             double proximityFuseRadiusM)
    : Entity(std::move(id), classification, initialPos, initialVel, dryMassKg + propellantMassKg),
      targetId_(std::move(targetId)),
      boosterTimeSec_(boosterTimeSec), sustainerTimeSec_(sustainerTimeSec),
      boosterThrustN_(boosterThrustN), sustainerThrustN_(sustainerThrustN),
      dryMassKg_(dryMassKg),
      boosterPropellantKg_(propellantMassKg * 0.65),
      sustainerPropellantKg_(propellantMassKg * 0.35),
      navRatioN_(navRatioN), maxGForceLimit_(maxGForceLimit),
      proximityFuseRadiusM_(proximityFuseRadiusM) {}

void GuidedMissile::setTargetKinematics(const Math::Vector3D& targetPos, const Math::Vector3D& targetVel) noexcept {
    targetPosition_ = targetPos;
    targetVelocity_ = targetVel;
    hasTargetLock_ = true;
}

void GuidedMissile::update(double dt) {
    if (!isActive_ || hasDetonated_) return;

    flightTimer_ += dt;

    // 1. Propulsion Stage Determination & Mass Depletion
    double thrustForceN = 0.0;
    if (flightTimer_ <= boosterTimeSec_) {
        motorPhase_ = MissileMotorPhase::BOOSTER;
        thrustForceN = boosterThrustN_;
        double burnRatio = dt / boosterTimeSec_;
        massKg_ = std::max(dryMassKg_ + sustainerPropellantKg_, massKg_ - (boosterPropellantKg_ * burnRatio));
    } else if (flightTimer_ <= (boosterTimeSec_ + sustainerTimeSec_)) {
        motorPhase_ = MissileMotorPhase::SUSTAINER;
        thrustForceN = sustainerThrustN_;
        double burnRatio = dt / sustainerTimeSec_;
        massKg_ = std::max(dryMassKg_, massKg_ - (sustainerPropellantKg_ * burnRatio));
    } else {
        motorPhase_ = MissileMotorPhase::COAST;
        thrustForceN = 0.0;
        massKg_ = dryMassKg_;
    }

    // 2. Relative Line-of-Sight Kinematics
    const Math::Vector3D losVec = targetPosition_ - position_;
    const double slantRangeM = losVec.magnitude();
    lastRangeToGo_ = slantRangeM;

    if (slantRangeM < closestDistanceM_) {
        closestDistanceM_ = slantRangeM;
    }

    const Math::Vector3D relVel = targetVelocity_ - velocity_;
    const double currentSpeed = velocity_.magnitude();

    // Seeker Gimbal Constraints (+-35 deg Conical Field-of-View)
    if (currentSpeed > 10.0 && slantRangeM > 1.0) {
        double dotVal = velocity_.normalized().dot(losVec.normalized());
        dotVal = std::clamp(dotVal, -1.0, 1.0);
        lastSeekerLookAngleDeg_ = std::acos(dotVal) * RAD_TO_DEG;

        if (lastSeekerLookAngleDeg_ > seekerFovDeg_) {
            hasTargetLock_ = false; // Broken seeker lock due to gimbal limits
        }
    }

    // Closing Velocity (Vc = - dR/dt)
    if (slantRangeM > 1e-3) {
        lastClosingVel_ = -losVec.normalized().dot(relVel);
    } else {
        lastClosingVel_ = currentSpeed;
    }

    // 3. Proportional Navigation (PN) Guidance Law
    Math::Vector3D guidanceAccel{0.0, 0.0, 0.0};

    if (hasTargetLock_ && slantRangeM > 1.0 && currentSpeed > 10.0) {
        // Line-Of-Sight Rotation Rate Vector: Omega = (R x V_rel) / |R|^2
        const Math::Vector3D losRateVec = losVec.cross(relVel) / (slantRangeM * slantRangeM);
        lastLosRateRadS_ = losRateVec.magnitude();

        // True Proportional Navigation Command: a_cmd = N * Vc * (Omega x v_unit)
        const Math::Vector3D mslHeadingUnit = velocity_.normalized();
        guidanceAccel = (losRateVec.cross(mslHeadingUnit)) * (navRatioN_ * std::max(0.0, lastClosingVel_));

        // Structural G-limit Acceleration Saturation
        const double maxAccelMps2 = maxGForceLimit_ * GRAVITY_G;
        if (guidanceAccel.magnitude() > maxAccelMps2) {
            guidanceAccel = guidanceAccel.normalized() * maxAccelMps2;
        }
    }

    // 4. Longitudinal Thrust & Aerodynamic Drag
    Math::Vector3D forwardUnit = (currentSpeed > 1.0) ? velocity_.normalized() : Math::Vector3D{1.0, 0.0, 0.0};
    Math::Vector3D thrustAccel = forwardUnit * (thrustForceN / massKg_);

    // Aerodynamic Drag Deceleration: a_drag = - 0.5 * rho * v^2 * Cd * A / m
    double dragAccelMag = (0.5 * AIR_DENSITY_SEA_LEVEL * (currentSpeed * currentSpeed) * 0.25 * 0.03) / massKg_;
    Math::Vector3D dragAccel = -forwardUnit * dragAccelMag;

    acceleration_ = thrustAccel + dragAccel + guidanceAccel;

    integrateKinematics(dt);

    // 5. Proximity Fuse Detonation & Lethal Blast Radius Calculation
    bool passClosestPoint = (slantRangeM > closestDistanceM_ + 2.0);
    if (slantRangeM <= proximityFuseRadiusM_ || (passClosestPoint && closestDistanceM_ <= proximityFuseRadiusM_ * 1.5)) {
        hasDetonated_ = true;
        missDistanceM_ = closestDistanceM_;

        if (closestDistanceM_ <= proximityFuseRadiusM_) {
            targetIntercepted_ = true;
            engagementOutcome_ = "HIT_KILL";
        } else {
            targetIntercepted_ = false;
            engagementOutcome_ = "PROXIMITY_MISS";
        }
        setActive(false);
    }

    // 6. Record AAR Telemetry Point
    flightHistory_.push_back({
        flightTimer_,
        targetId_,
        id_,
        lastRangeToGo_,
        lastClosingVel_,
        lastLosRateRadS_,
        lastSeekerLookAngleDeg_,
        velocity_.magnitude(),
        motorPhaseToString(motorPhase_),
        engagementOutcome_
    });
}

std::string GuidedMissile::serializeTelemetry() const {
    return Network::formatSentence(getTalkerId(), "GNC", {
        id_,
        targetId_,
        std::to_string(static_cast<int>(lastRangeToGo_)),
        std::to_string(static_cast<int>(lastClosingVel_)),
        motorPhaseToString(motorPhase_),
        (hasTargetLock_ ? "LOCKED" : "LOST")
    });
}

} // namespace SDN::Entities
