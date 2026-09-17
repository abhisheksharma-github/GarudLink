/**
 * @file Math3D.hpp
 * @brief High-precision 3D Vector Math, Euler Orientations, and Tactical Line-Of-Sight (LOS) Kinematics.
 * 
 * Coordinate System Conventions:
 * 1. North-East-Down (NED) Navigation Frame:
 *    - X-axis: Points North (True North reference)
 *    - Y-axis: Points East
 *    - Z-axis: Points Downwards towards the Earth's center
 * 2. Body-Fixed Tactical Frame:
 *    - X-axis: Longitudinal axis pointing forward towards the Ship's Bow / Aircraft Nose
 *    - Y-axis: Lateral axis pointing towards Starboard (Right side)
 *    - Z-axis: Normal axis pointing downwards through the Keel / Bottom
 * 3. Euler Rotation Conventions (Tait-Bryan Z-Y-X sequence / Aerospace Standard):
 *    - Roll  (phi)   : Rotation about X-axis (Starboard wing/keel dip is positive)
 *    - Pitch (theta) : Rotation about Y-axis (Bow/Nose up is positive)
 *    - Yaw   (psi)   : Rotation about Z-axis (Clockwise heading from above is positive)
 */

#pragma once

#include <cmath>
#include <ostream>
#include <string>

namespace SDN::Math {

/**
 * @struct Vector3D
 * @brief 3D Euclidean vector with double-precision floating-point components.
 */
struct Vector3D {
    double x{0.0};
    double y{0.0};
    double z{0.0};

    // Constructors
    constexpr Vector3D() noexcept = default;
    constexpr Vector3D(double xVal, double yVal, double zVal) noexcept 
        : x(xVal), y(yVal), z(zVal) {}

    // Validation & Numerical Sanity
    [[nodiscard]] bool isValid() const noexcept;
    [[nodiscard]] bool isZero(double epsilon = 1e-12) const noexcept;

    // Vector Norms and Distances
    [[nodiscard]] double magnitudeSquared() const noexcept;
    [[nodiscard]] double magnitude() const noexcept;
    [[nodiscard]] double distanceTo(const Vector3D& other) const noexcept;
    [[nodiscard]] double distanceSquaredTo(const Vector3D& other) const noexcept;

    // Normalization with safe handling of zero/infinite vectors
    [[nodiscard]] Vector3D normalized(double epsilon = 1e-12) const noexcept;

    // Vector Products
    [[nodiscard]] double dot(const Vector3D& other) const noexcept;
    [[nodiscard]] Vector3D cross(const Vector3D& other) const noexcept;

    // Arithmetic Operators
    constexpr Vector3D operator+(const Vector3D& other) const noexcept {
        return Vector3D{x + other.x, y + other.y, z + other.z};
    }

    constexpr Vector3D operator-(const Vector3D& other) const noexcept {
        return Vector3D{x - other.x, y - other.y, z - other.z};
    }

    constexpr Vector3D operator-() const noexcept {
        return Vector3D{-x, -y, -z};
    }

    constexpr Vector3D operator*(double scalar) const noexcept {
        return Vector3D{x * scalar, y * scalar, z * scalar};
    }

    Vector3D operator/(double scalar) const noexcept;

    constexpr Vector3D& operator+=(const Vector3D& other) noexcept {
        x += other.x;
        y += other.y;
        z += other.z;
        return *this;
    }

    constexpr Vector3D& operator-=(const Vector3D& other) noexcept {
        x -= other.x;
        y -= other.y;
        z -= other.z;
        return *this;
    }

    constexpr Vector3D& operator*=(double scalar) noexcept {
        x *= scalar;
        y *= scalar;
        z *= scalar;
        return *this;
    }

    Vector3D& operator/=(double scalar) noexcept;

    // Equality with floating-point tolerance
    [[nodiscard]] bool equals(const Vector3D& other, double epsilon = 1e-9) const noexcept;
    [[nodiscard]] bool operator==(const Vector3D& other) const noexcept;
    [[nodiscard]] bool operator!=(const Vector3D& other) const noexcept;

    // Formatted string representation
    [[nodiscard]] std::string toString(int precision = 3) const;
};

// Left-hand scalar multiplication: scalar * Vector3D
constexpr Vector3D operator*(double scalar, const Vector3D& vec) noexcept {
    return vec * scalar;
}

// Stream insertion operator
std::ostream& operator<<(std::ostream& os, const Vector3D& vec);

/**
 * @struct EulerAngles
 * @brief Representation of 3D orientations (Roll, Pitch, Yaw) in degrees and radians.
 */
struct EulerAngles {
    double rollRad{0.0};   ///< Bank angle around X-axis (-pi to +pi)
    double pitchRad{0.0};  ///< Elevation angle around Y-axis (-pi/2 to +pi/2)
    double yawRad{0.0};    ///< Heading/Azimuth angle around Z-axis (-pi to +pi or 0 to 2*pi)

    constexpr EulerAngles() noexcept = default;
    constexpr EulerAngles(double rRad, double pRad, double yRad) noexcept
        : rollRad(rRad), pitchRad(pRad), yawRad(yRad) {}

    // Factory methods
    [[nodiscard]] static EulerAngles fromDegrees(double rollDeg, double pitchDeg, double yawDeg) noexcept;
    [[nodiscard]] static constexpr EulerAngles fromRadians(double rollRad, double pitchRad, double yawRad) noexcept {
        return EulerAngles{rollRad, pitchRad, yawRad};
    }

    // Degree conversions
    [[nodiscard]] double rollDeg() const noexcept;
    [[nodiscard]] double pitchDeg() const noexcept;
    [[nodiscard]] double yawDeg() const noexcept;

    // Normalization of angles into standard intervals
    [[nodiscard]] EulerAngles normalized() const noexcept;

    [[nodiscard]] bool isValid() const noexcept;
    [[nodiscard]] bool equals(const EulerAngles& other, double epsilon = 1e-6) const noexcept;
};

/**
 * @struct LOSResult
 * @brief Tactical Line-of-Sight kinematics between an observer (e.g., naval radar/ship) and a target.
 */
struct LOSResult {
    double range{0.0};                  ///< Euclidean slant range |R| (meters)
    Vector3D losVector{};               ///< True relative displacement vector R = P_target - P_observer
    Vector3D losUnitVector{};           ///< Normalized LOS unit vector R_hat = R / |R|
    Vector3D relativeVelocity{};        ///< Relative velocity V_rel = V_target - V_observer
    Vector3D losRateVector{};           ///< True 3D LOS angular rate vector Omega = (R x V_rel) / |R|^2 (rad/s)
    double closingVelocity{0.0};        ///< Closing velocity V_c = - d/dt(|R|) = -(V_rel . R_hat) (m/s)
    double azimuthRad{0.0};             ///< LOS Azimuth in horizontal plane relative to reference X (rad)
    double elevationRad{0.0};           ///< LOS Elevation angle above horizontal plane (rad)
    bool isSingular{false};             ///< True if range is zero or below singularity threshold
};

/**
 * @brief Computes tactical Line-Of-Sight (LOS) geometry and rate vector between observer and target.
 * 
 * Mathematical Formulation:
 * - Relative Position: R = P_t - P_o
 * - Slant Range: r = |R|
 * - LOS Unit Vector: R_hat = R / r
 * - Relative Velocity: V_rel = V_t - V_o
 * - LOS Angular Rate Vector: Omega_LOS = (R x V_rel) / r^2
 * - Closing Velocity: V_c = - (V_rel . R_hat)
 * 
 * @param observerPos 3D Position of observer/ship (meters)
 * @param observerVel 3D Velocity of observer/ship (m/s)
 * @param targetPos 3D Position of target/missile (meters)
 * @param targetVel 3D Velocity of target/missile (m/s)
 * @param singularityThreshold Minimum slant range to prevent zero division (default 1e-6 m)
 * @return LOSResult Complete tactical LOS packet
 */
[[nodiscard]] LOSResult calculateLOS(
    const Vector3D& observerPos,
    const Vector3D& observerVel,
    const Vector3D& targetPos,
    const Vector3D& targetVel,
    double singularityThreshold = 1e-6
) noexcept;

/**
 * @brief Normalizes an angle in radians to [-pi, +pi].
 */
[[nodiscard]] double normalizeAnglePi(double angleRad) noexcept;

/**
 * @brief Normalizes an angle in degrees to [0, 360).
 */
[[nodiscard]] double normalizeAngle360(double angleDeg) noexcept;

} // namespace SDN::Math
