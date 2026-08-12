#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>

namespace head_scan_policy {

struct Pose {
    double pitch = 0.0;
    double yaw = 0.0;
};

inline double wrapPhase(double phase)
{
    if (!std::isfinite(phase)) return 0.0;
    phase = std::fmod(phase, 1.0);
    return phase < 0.0 ? phase + 1.0 : phase;
}

inline Pose fieldScanPose(
    double phase,
    double highPitch,
    double lowPitch,
    double rightYaw,
    double leftYaw)
{
    constexpr double kPi = 3.14159265358979323846;
    const double angle = 2.0 * kPi * wrapPhase(phase);
    const double pitchCenter = 0.5 * (highPitch + lowPitch);
    const double pitchAmplitude = 0.5 * (lowPitch - highPitch);
    const double yawCenter = 0.5 * (rightYaw + leftYaw);
    const double yawAmplitude = 0.5 * (leftYaw - rightYaw);

    // A closed figure-eight covers both near and far field while keeping the
    // commanded position and velocity continuous at the cycle boundary.
    return {
        pitchCenter + pitchAmplitude * std::sin(2.0 * angle),
        yawCenter - yawAmplitude * std::cos(angle),
    };
}

template <std::size_t N>
inline Pose closedWaypointPose(
    const std::array<Pose, N> &waypoints,
    double phase)
{
    static_assert(N >= 2, "a closed scan needs at least two waypoints");
    const double segmentPosition = wrapPhase(phase) * static_cast<double>(N);
    const std::size_t first = std::min(
        static_cast<std::size_t>(segmentPosition), N - 1);
    const std::size_t second = (first + 1) % N;
    const double progress = std::clamp(
        segmentPosition - static_cast<double>(first), 0.0, 1.0);
    constexpr double kPi = 3.14159265358979323846;
    const double eased = 0.5 - 0.5 * std::cos(kPi * progress);

    return {
        waypoints[first].pitch +
            (waypoints[second].pitch - waypoints[first].pitch) * eased,
        waypoints[first].yaw +
            (waypoints[second].yaw - waypoints[first].yaw) * eased,
    };
}

template <typename Evaluator>
inline double nearestPhase(
    Evaluator evaluate,
    const Pose &current,
    double pitchScale,
    double yawScale,
    std::size_t samples = 192)
{
    if (!std::isfinite(current.pitch) || !std::isfinite(current.yaw)) {
        return 0.0;
    }

    pitchScale = std::max(std::fabs(pitchScale), 0.05);
    yawScale = std::max(std::fabs(yawScale), 0.05);
    samples = std::max<std::size_t>(samples, 2);
    double bestPhase = 0.0;
    double bestDistance = std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i < samples; ++i) {
        const double phase = static_cast<double>(i) /
            static_cast<double>(samples);
        const Pose candidate = evaluate(phase);
        if (!std::isfinite(candidate.pitch) || !std::isfinite(candidate.yaw)) {
            continue;
        }
        const double pitchError =
            (candidate.pitch - current.pitch) / pitchScale;
        const double yawError = (candidate.yaw - current.yaw) / yawScale;
        const double distance = std::hypot(pitchError, yawError);
        if (distance < bestDistance) {
            bestDistance = distance;
            bestPhase = phase;
        }
    }
    return bestPhase;
}

inline Pose rateLimitedPose(
    const Pose &previous,
    const Pose &target,
    double dtSeconds,
    double maxPitchRate,
    double maxYawRate)
{
    if (!std::isfinite(target.pitch) || !std::isfinite(target.yaw)) {
        return previous;
    }
    if (!std::isfinite(previous.pitch) || !std::isfinite(previous.yaw)) {
        return target;
    }

    const double dt = std::max(0.0, dtSeconds);
    const double pitchStep = dt * std::max(0.0, maxPitchRate);
    const double yawStep = dt * std::max(0.0, maxYawRate);
    return {
        previous.pitch + std::clamp(
            target.pitch - previous.pitch, -pitchStep, pitchStep),
        previous.yaw + std::clamp(
            target.yaw - previous.yaw, -yawStep, yawStep),
    };
}

} // namespace head_scan_policy
