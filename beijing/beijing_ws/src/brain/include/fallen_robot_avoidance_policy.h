#pragma once

#include <algorithm>
#include <cmath>

namespace fallen_robot_avoidance_policy {

constexpr double kFallenRobotRadius = 0.40;

inline double normalizeAngle(double angle)
{
    constexpr double kPi = 3.14159265358979323846;
    while (angle > kPi) angle -= 2.0 * kPi;
    while (angle <= -kPi) angle += 2.0 * kPi;
    return angle;
}

struct PathIntersection
{
    bool intersects = false;
    double entryDistance = 0.0;
    double lateralDistance = 0.0;
};

struct TemporalFallenState
{
    bool fallen = false;
    double uprightScore = 0.0;
};

struct AvoidanceVelocity
{
    double x = 0.0;
    double y = 0.0;
};

inline AvoidanceVelocity makeAvoidanceVelocity(
    double pathAngle,
    double forwardSpeed,
    double lateralSpeed,
    double side,
    double vxLimit,
    double vyLimit)
{
    if (!std::isfinite(pathAngle) || !std::isfinite(forwardSpeed) ||
        !std::isfinite(lateralSpeed) || !std::isfinite(side) ||
        !std::isfinite(vxLimit) || !std::isfinite(vyLimit) ||
        forwardSpeed < 0.0 || lateralSpeed < 0.0 ||
        vxLimit < 0.0 || vyLimit < 0.0) {
        return {};
    }

    const double normalizedSide = side >= 0.0 ? 1.0 : -1.0;
    const double pathX = std::cos(pathAngle);
    const double pathY = std::sin(pathAngle);
    return {
        std::clamp(
            forwardSpeed * pathX - lateralSpeed * pathY * normalizedSide,
            -vxLimit,
            vxLimit),
        std::clamp(
            forwardSpeed * pathY + lateralSpeed * pathX * normalizedSide,
            -vyLimit,
            vyLimit)
    };
}

inline TemporalFallenState updateTemporalFallenState(
    double previousScore,
    bool previousFallen,
    bool fallenEvidence,
    double fallenConfidence,
    double fallenConfirmScore = -2.0,
    double uprightConfirmScore = 1.5)
{
    const double boundedConfidence = std::clamp(fallenConfidence, 0.0, 1.0);
    const double evidence = fallenEvidence
        ? -(1.0 + boundedConfidence)
        : 1.0;
    const double score = std::clamp(previousScore + evidence, -3.0, 3.0);

    bool fallen = previousFallen;
    if (score <= fallenConfirmScore) {
        fallen = true;
    } else if (score >= uprightConfirmScore) {
        fallen = false;
    }
    return {fallen, score};
}

inline double selectAvoidanceSide(
    double preferredSide,
    double lockedSide,
    bool lockActive,
    double positiveClearance,
    double negativeClearance,
    double minimumClearance,
    double switchMargin = 0.2)
{
    const auto clearanceFor = [&](double side) {
        return side >= 0.0 ? positiveClearance : negativeClearance;
    };

    double selectedSide = preferredSide >= 0.0 ? 1.0 : -1.0;
    if (lockActive && std::fabs(lockedSide) > 0.1) {
        selectedSide = lockedSide >= 0.0 ? 1.0 : -1.0;
        const double lockedClearance = clearanceFor(selectedSide);
        const double alternateClearance = clearanceFor(-selectedSide);
        if (lockedClearance < minimumClearance &&
            alternateClearance > lockedClearance + switchMargin) {
            selectedSide = -selectedSide;
        }
        return selectedSide;
    }

    if (clearanceFor(-selectedSide) >
        clearanceFor(selectedSide) + switchMargin) {
        selectedSide = -selectedSide;
    }
    return selectedSide;
}

inline PathIntersection intersectsPath(
    double x,
    double y,
    double pathAngle,
    double maxDistance,
    double robotClearance,
    double regionRadius = kFallenRobotRadius)
{
    PathIntersection result;
    if (!std::isfinite(x) || !std::isfinite(y) ||
        !std::isfinite(pathAngle) || !std::isfinite(maxDistance) ||
        !std::isfinite(robotClearance) || !std::isfinite(regionRadius) ||
        maxDistance <= 0.0 || robotClearance < 0.0 || regionRadius < 0.0) {
        return result;
    }

    const double centerDistance = std::hypot(x, y);
    if (centerDistance - regionRadius > maxDistance) return result;

    const double pathX = std::cos(pathAngle);
    const double pathY = std::sin(pathAngle);
    const double alongTrack = x * pathX + y * pathY;
    const double lateral = x * pathY - y * pathX;
    const double effectiveRadius = robotClearance + regionRadius;
    result.lateralDistance = std::fabs(lateral);
    if (alongTrack + effectiveRadius <= 0.0 ||
        result.lateralDistance > effectiveRadius) {
        return result;
    }

    const double halfChord = std::sqrt(std::max(
        0.0,
        effectiveRadius * effectiveRadius - lateral * lateral));
    result.intersects = true;
    result.entryDistance = std::max(0.0, alongTrack - halfChord);
    return result;
}

inline bool intersectsForwardSector(
    double x,
    double y,
    double maxDistance,
    double halfAngle,
    double regionRadius = kFallenRobotRadius)
{
    constexpr double kPi = 3.14159265358979323846;
    if (!std::isfinite(x) || !std::isfinite(y) ||
        !std::isfinite(maxDistance) || !std::isfinite(halfAngle) ||
        !std::isfinite(regionRadius) || maxDistance <= 0.0 ||
        halfAngle < 0.0 || regionRadius < 0.0) {
        return false;
    }

    const double centerDistance = std::hypot(x, y);
    if (centerDistance - regionRadius > maxDistance) return false;
    if (centerDistance <= regionRadius) return true;

    const double angularRadius = std::asin(std::clamp(
        regionRadius / centerDistance, 0.0, 1.0));
    const double allowedAngle = std::min(kPi, halfAngle + angularRadius);
    return std::fabs(normalizeAngle(std::atan2(y, x))) <= allowedAngle;
}

} // namespace fallen_robot_avoidance_policy
