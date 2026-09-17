/**
 * @file Math3D.cpp
 * @brief Implementation of 3D Vector Math, Euler Angles, and Tactical Line-Of-Sight (LOS) Kinematics.
 */

#include "Math3D.hpp"

#include <cmath>
#include <iomanip>
#include <limits>
#include <sstream>

namespace SDN::Math {

namespace {
    constexpr double PI_VAL = 3.14159265358979323846;
    constexpr double DEG_TO_RAD = PI_VAL / 180.0;
    constexpr double RAD_TO_DEG = 180.0 / PI_VAL;
    constexpr double TWO_PI = 2.0 * PI_VAL;
}

// ============================================================================
// Vector3D Implementation
// ============================================================================

bool Vector3D::isValid() const noexcept {
    return std::isfinite(x) && std::isfinite(y) && std::isfinite(z);
}

bool Vector3D::isZero(double epsilon) const noexcept {
    return (std::abs(x) <= epsilon) && 
           (std::abs(y) <= epsilon) && 
           (std::abs(z) <= epsilon);
}

double Vector3D::magnitudeSquared() const noexcept {
    return (x * x) + (y * y) + (z * z);
}

double Vector3D::magnitude() const noexcept {
    return std::sqrt(magnitudeSquared());
}

double Vector3D::distanceTo(const Vector3D& other) const noexcept {
    return (*this - other).magnitude();
}

double Vector3D::distanceSquaredTo(const Vector3D& other) const noexcept {
    return (*this - other).magnitudeSquared();
}

Vector3D Vector3D::normalized(double epsilon) const noexcept {
    const double magSq = magnitudeSquared();
    if (!std::isfinite(magSq) || magSq < (epsilon * epsilon)) {
        // Safe numerical fallback for zero or sub-epsilon magnitude
        return Vector3D{0.0, 0.0, 0.0};
    }
    const double invMag = 1.0 / std::sqrt(magSq);
    return Vector3D{x * invMag, y * invMag, z * invMag};
}

double Vector3D::dot(const Vector3D& other) const noexcept {
    return (x * other.x) + (y * other.y) + (z * other.z);
}

Vector3D Vector3D::cross(const Vector3D& other) const noexcept {
    return Vector3D{
        (y * other.z) - (z * other.y),
        (z * other.x) - (x * other.z),
        (x * other.y) - (y * other.x)
    };
}

Vector3D Vector3D::operator/(double scalar) const noexcept {
    if (std::abs(scalar) <= std::numeric_limits<double>::epsilon() || !std::isfinite(scalar)) {
        // Defensive division by zero/subnormal
        return Vector3D{0.0, 0.0, 0.0};
    }
    const double invScalar = 1.0 / scalar;
    return Vector3D{x * invScalar, y * invScalar, z * invScalar};
}

Vector3D& Vector3D::operator/=(double scalar) noexcept {
    if (std::abs(scalar) <= std::numeric_limits<double>::epsilon() || !std::isfinite(scalar)) {
        x = 0.0;
        y = 0.0;
        z = 0.0;
        return *this;
    }
    const double invScalar = 1.0 / scalar;
    x *= invScalar;
    y *= invScalar;
    z *= invScalar;
    return *this;
}

bool Vector3D::equals(const Vector3D& other, double epsilon) const noexcept {
    return (std::abs(x - other.x) <= epsilon) &&
           (std::abs(y - other.y) <= epsilon) &&
           (std::abs(z - other.z) <= epsilon);
}

bool Vector3D::operator==(const Vector3D& other) const noexcept {
    return equals(other, 1e-9);
}

bool Vector3D::operator!=(const Vector3D& other) const noexcept {
    return !(*this == other);
}

std::string Vector3D::toString(int precision) const {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(precision);
    oss << "[" << x << ", " << y << ", " << z << "]";
    return oss.str();
}

std::ostream& operator<<(std::ostream& os, const Vector3D& vec) {
    os << vec.toString();
    return os;
}

// ============================================================================
// Angle Utility Functions
// ============================================================================

double normalizeAnglePi(double angleRad) noexcept {
    if (!std::isfinite(angleRad)) {
        return 0.0;
    }
    while (angleRad > PI_VAL) {
        angleRad -= TWO_PI;
    }
    while (angleRad < -PI_VAL) {
        angleRad += TWO_PI;
    }
    return angleRad;
}

double normalizeAngle360(double angleDeg) noexcept {
    if (!std::isfinite(angleDeg)) {
        return 0.0;
    }
    double normalized = std::fmod(angleDeg, 360.0);
    if (normalized < 0.0) {
        normalized += 360.0;
    }
    return normalized;
}

// ============================================================================
// EulerAngles Implementation
// ============================================================================

EulerAngles EulerAngles::fromDegrees(double rollDeg, double pitchDeg, double yawDeg) noexcept {
    return EulerAngles{
        rollDeg * DEG_TO_RAD,
        pitchDeg * DEG_TO_RAD,
        yawDeg * DEG_TO_RAD
    };
}

double EulerAngles::rollDeg() const noexcept {
    return rollRad * RAD_TO_DEG;
}

double EulerAngles::pitchDeg() const noexcept {
    return pitchRad * RAD_TO_DEG;
}

double EulerAngles::yawDeg() const noexcept {
    return yawRad * RAD_TO_DEG;
}

EulerAngles EulerAngles::normalized() const noexcept {
    // Roll: [-pi, +pi]
    const double nRoll = normalizeAnglePi(rollRad);
    
    // Pitch: clamped to [-pi/2, +pi/2] to prevent gimbal lock singularities
    double nPitch = pitchRad;
    if (pitchRad > (PI_VAL / 2.0)) {
        nPitch = PI_VAL / 2.0;
    } else if (pitchRad < (-PI_VAL / 2.0)) {
        nPitch = -PI_VAL / 2.0;
    } else {
        nPitch = pitchRad;
    }

    // Yaw: [0, 2*pi) standard navigation heading
    double nYaw = std::fmod(yawRad, TWO_PI);
    if (nYaw < 0.0) {
        nYaw += TWO_PI;
    }

    return EulerAngles{nRoll, nPitch, nYaw};
}

bool EulerAngles::isValid() const noexcept {
    return std::isfinite(rollRad) && std::isfinite(pitchRad) && std::isfinite(yawRad);
}

bool EulerAngles::equals(const EulerAngles& other, double epsilon) const noexcept {
    return (std::abs(rollRad - other.rollRad) <= epsilon) &&
           (std::abs(pitchRad - other.pitchRad) <= epsilon) &&
           (std::abs(yawRad - other.yawRad) <= epsilon);
}

// ============================================================================
// Line-of-Sight (LOS) Kinematics Calculation
// ============================================================================

LOSResult calculateLOS(
    const Vector3D& observerPos,
    const Vector3D& observerVel,
    const Vector3D& targetPos,
    const Vector3D& targetVel,
    double singularityThreshold
) noexcept {
    LOSResult result{};

    // 1. Relative displacement vector R = P_target - P_observer
    const Vector3D relPos = targetPos - observerPos;
    const double rangeSq = relPos.magnitudeSquared();
    const double range = std::sqrt(rangeSq);

    result.losVector = relPos;
    result.range = range;

    // Numerical singularity guard
    if (!std::isfinite(range) || range <= singularityThreshold) {
        result.isSingular = true;
        result.losUnitVector = Vector3D{0.0, 0.0, 0.0};
        result.relativeVelocity = targetVel - observerVel;
        result.losRateVector = Vector3D{0.0, 0.0, 0.0};
        result.closingVelocity = 0.0;
        result.azimuthRad = 0.0;
        result.elevationRad = 0.0;
        return result;
    }

    result.isSingular = false;

    // 2. Unit Line-Of-Sight Vector: R_hat = R / |R|
    const double invRange = 1.0 / range;
    result.losUnitVector = Vector3D{relPos.x * invRange, relPos.y * invRange, relPos.z * invRange};

    // 3. Relative Velocity: V_rel = V_target - V_observer
    const Vector3D relVel = targetVel - observerVel;
    result.relativeVelocity = relVel;

    // 4. LOS Angular Rate Vector: Omega_LOS = (R x V_rel) / |R|^2
    const Vector3D rCrossV = relPos.cross(relVel);
    result.losRateVector = rCrossV / rangeSq;

    // 5. Closing Velocity: V_c = - d/dt(range) = - (V_rel . R_hat)
    result.closingVelocity = -(relVel.dot(result.losUnitVector));

    // 6. Azimuth in Horizontal Frame (NED Frame: X=North, Y=East)
    // Azimuth = atan2(East, North) = atan2(relPos.y, relPos.x)
    result.azimuthRad = std::atan2(relPos.y, relPos.x);
    if (result.azimuthRad < 0.0) {
        result.azimuthRad += TWO_PI; // [0, 2*pi)
    }

    // 7. Elevation angle: In NED frame, Z is Down, so -Z is Altitude above horizontal
    // sin(elevation) = -relPos.z / range
    const double rawSin = -relPos.z * invRange;
    const double sinElevation = (rawSin < -1.0) ? -1.0 : ((rawSin > 1.0) ? 1.0 : rawSin);
    result.elevationRad = std::asin(sinElevation);

    return result;
}

} // namespace SDN::Math
