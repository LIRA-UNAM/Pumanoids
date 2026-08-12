#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace fall_recovery_policy {

// Freeze before the motor-recovery threshold so the first part of a fall is
// not integrated into the field pose.
constexpr double kLocalizationPreHoldTiltRad = 0.48;
constexpr double kLocalizationPreHoldReleaseTiltRad = 0.30;
constexpr std::int64_t kLocalizationPreHoldReleaseConfirmUs = 500000;

// A falling body can cross upright for a few IMU samples. Require a stable,
// stationary interval before treating that as an implicit recovery.
constexpr std::int64_t kUprightRecoveryConfirmUs = 1000000;

struct PostGetupOdomSettleInput
{
    double elapsedMs = 0.0;
    double minimumDwellMs = 0.0;
    double maximumHoldMs = 0.0;
    bool odomWindowReady = false;
    bool odomFinite = false;
    bool imuUpright = false;
    bool gyroQuiet = false;
    double odomXRangeM = 0.0;
    double odomYRangeM = 0.0;
    double odomThetaRangeRad = 0.0;
    double maximumXyRangeM = 0.0;
    double maximumThetaRangeRad = 0.0;
};

struct PostGetupOdomSettleDecision
{
    bool active = true;
    bool stable = false;
    bool timedOut = false;
    double remainingMs = 0.0;
    const char *reason = "minimum_dwell";
};

// The SDK odometer can look quiet and then make a delayed support-frame
// adjustment after get-up. Cross that plateau first, then require a complete
// rolling quiet window before odometry is allowed to move the field pose.
inline PostGetupOdomSettleDecision evaluatePostGetupOdomSettling(
    const PostGetupOdomSettleInput &input)
{
    if (!std::isfinite(input.elapsedMs) || input.elapsedMs < 0.0) {
        return {true, false, false, 0.0, "invalid_elapsed"};
    }

    const double minimumMs = std::max(0.0, input.minimumDwellMs);
    const double maximumMs = std::max(
        minimumMs, std::max(0.0, input.maximumHoldMs));
    const double remainingMs = std::max(0.0, maximumMs - input.elapsedMs);
    if (input.elapsedMs < minimumMs) {
        return {true, false, false, remainingMs, "minimum_dwell"};
    }
    if (!input.imuUpright) {
        return {true, false, false, remainingMs, "imu_not_upright"};
    }
    if (!input.gyroQuiet) {
        return {true, false, false, remainingMs, "imu_still_moving"};
    }
    // Timeout may bypass noisy odometry, but never a tilted or moving body.
    if (input.elapsedMs >= maximumMs) {
        return {false, false, true, 0.0, "maximum_timeout"};
    }
    if (!input.odomWindowReady) {
        return {true, false, false, remainingMs, "odom_window_incomplete"};
    }
    if (!input.odomFinite ||
        !std::isfinite(input.odomXRangeM) ||
        !std::isfinite(input.odomYRangeM) ||
        !std::isfinite(input.odomThetaRangeRad)) {
        return {true, false, false, remainingMs, "odom_window_invalid"};
    }

    const double maximumXyRangeM = std::max(0.0, input.maximumXyRangeM);
    const double maximumThetaRangeRad =
        std::max(0.0, input.maximumThetaRangeRad);
    if (input.odomXRangeM > maximumXyRangeM ||
        input.odomYRangeM > maximumXyRangeM) {
        return {true, false, false, remainingMs, "odom_translation_unstable"};
    }
    if (input.odomThetaRangeRad > maximumThetaRangeRad) {
        return {true, false, false, remainingMs, "odom_heading_unstable"};
    }
    return {false, true, false, 0.0, "stable_window_confirmed"};
}

// Support-frame translation during get-up is not field motion, while the net
// SDK odometer heading remains useful. Freeze x/y and carry only that heading.
inline double fieldHeadingWithHeldTranslation(
    double fieldHeadingAtHoldStart,
    double odomHeadingAtHoldStart,
    double currentOdomHeading)
{
    if (!std::isfinite(fieldHeadingAtHoldStart) ||
        !std::isfinite(odomHeadingAtHoldStart) ||
        !std::isfinite(currentOdomHeading)) {
        return fieldHeadingAtHoldStart;
    }
    const double odomDelta = std::atan2(
        std::sin(currentOdomHeading - odomHeadingAtHoldStart),
        std::cos(currentOdomHeading - odomHeadingAtHoldStart));
    return std::atan2(
        std::sin(fieldHeadingAtHoldStart + odomDelta),
        std::cos(fieldHeadingAtHoldStart + odomDelta));
}

inline double bodyTilt(double roll, double pitch)
{
    if (!std::isfinite(roll) || !std::isfinite(pitch)) return 0.0;
    return std::max(std::fabs(roll), std::fabs(pitch));
}

inline bool shouldEnterLocalizationPreHold(double roll, double pitch)
{
    return std::isfinite(roll) && std::isfinite(pitch) &&
           bodyTilt(roll, pitch) >= kLocalizationPreHoldTiltRad;
}

inline bool isLocalizationPreHoldReleaseTilt(double roll, double pitch)
{
    return std::isfinite(roll) && std::isfinite(pitch) &&
           bodyTilt(roll, pitch) <= kLocalizationPreHoldReleaseTiltRad;
}

inline bool isFiniteGyro(double gyroX, double gyroY, double gyroZ)
{
    return std::isfinite(gyroX) && std::isfinite(gyroY) &&
           std::isfinite(gyroZ);
}

inline bool isStationaryUprightSample(
    double roll,
    double pitch,
    double gyroX,
    double gyroY,
    double gyroZ,
    double movementThresholdRadPerSec)
{
    if (!isLocalizationPreHoldReleaseTilt(roll, pitch) ||
        !isFiniteGyro(gyroX, gyroY, gyroZ) ||
        !std::isfinite(movementThresholdRadPerSec)) {
        return false;
    }
    const double gyroMagnitude = std::sqrt(
        gyroX * gyroX + gyroY * gyroY + gyroZ * gyroZ);
    return gyroMagnitude <= std::max(0.0, movementThresholdRadPerSec);
}

inline bool updateUprightRecoveryConfirmation(
    bool notUpright,
    bool moving,
    std::int64_t nowUs,
    std::int64_t &stableSinceUs)
{
    if (notUpright || moving || nowUs <= 0) {
        stableSinceUs = 0;
        return false;
    }
    if (stableSinceUs <= 0 || nowUs < stableSinceUs) {
        stableSinceUs = nowUs;
        return false;
    }
    return nowUs - stableSinceUs >= kUprightRecoveryConfirmUs;
}

inline double normalizeAngle(double angle)
{
    constexpr double kPi = 3.14159265358979323846;
    constexpr double kTwoPi = 2.0 * kPi;
    angle = std::fmod(angle + kPi, kTwoPi);
    if (angle < 0.0) angle += kTwoPi;
    return angle - kPi;
}

inline bool areHeadingCandidatesConsistent(
    double lhs, double rhs, double toleranceRad)
{
    return std::isfinite(lhs) && std::isfinite(rhs) &&
           std::isfinite(toleranceRad) &&
           std::fabs(normalizeAngle(lhs - rhs)) <=
               std::max(0.0, toleranceRad);
}

inline int fieldHalf(double x)
{
    if (!std::isfinite(x) || std::fabs(x) <= 0.15) return 0;
    return x < 0.0 ? -1 : 1;
}

inline bool isSafeHeadingRealignCandidate(
    double trustedX,
    double currentX,
    double currentY,
    double candidateX,
    double candidateY,
    double maxPositionDelta)
{
    if (!std::isfinite(trustedX) || !std::isfinite(currentX) ||
        !std::isfinite(currentY) || !std::isfinite(candidateX) ||
        !std::isfinite(candidateY) || !std::isfinite(maxPositionDelta)) {
        return false;
    }
    const int trustedHalf = fieldHalf(trustedX);
    const int candidateHalf = fieldHalf(candidateX);
    if (trustedHalf != 0 && candidateHalf != 0 &&
        trustedHalf != candidateHalf) {
        return false;
    }
    return std::hypot(candidateX - currentX, candidateY - currentY) <=
           std::max(0.0, maxPositionDelta);
}

} // namespace fall_recovery_policy
