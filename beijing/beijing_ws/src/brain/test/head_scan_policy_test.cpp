#include <array>
#include <cmath>
#include <cstdlib>
#include <iostream>

#include "head_scan_policy.h"

namespace policy = head_scan_policy;

namespace {

void require(bool condition, const char *message)
{
    if (condition) return;
    std::cerr << "FAILED: " << message << '\n';
    std::exit(EXIT_FAILURE);
}

double distance(const policy::Pose &lhs, const policy::Pose &rhs)
{
    return std::hypot(lhs.pitch - rhs.pitch, lhs.yaw - rhs.yaw);
}

void checkFieldScanCycleIsContinuous()
{
    const auto evaluate = [](double phase) {
        return policy::fieldScanPose(phase, 0.15, 0.35, -0.8, 0.8);
    };
    require(distance(evaluate(0.0), evaluate(1.0)) < 1e-12,
            "field scan position jumps at cycle boundary");

    constexpr double epsilon = 1e-5;
    const auto before = evaluate(1.0 - epsilon);
    const auto boundary = evaluate(1.0);
    const auto after = evaluate(epsilon);
    const policy::Pose incoming{
        (boundary.pitch - before.pitch) / epsilon,
        (boundary.yaw - before.yaw) / epsilon,
    };
    const policy::Pose outgoing{
        (after.pitch - boundary.pitch) / epsilon,
        (after.yaw - boundary.yaw) / epsilon,
    };
    require(distance(incoming, outgoing) < 0.01,
            "field scan velocity jumps at cycle boundary");
}

void checkBallScanClosingEdgeIsContinuous()
{
    static constexpr std::array<policy::Pose, 6> waypoints{{
        {1.0, 1.1}, {1.0, 0.0}, {1.0, -1.1},
        {0.2, -1.1}, {0.2, 0.0}, {0.2, 1.1},
    }};
    const auto evaluate = [](double phase) {
        return policy::closedWaypointPose(waypoints, phase);
    };

    require(distance(evaluate(0.0), evaluate(1.0)) < 1e-12,
            "ball scan position jumps at cycle boundary");
    constexpr double epsilon = 1e-5;
    require(distance(evaluate(1.0 - epsilon), evaluate(epsilon)) < 1e-6,
            "ball scan closing edge is not eased at the cycle boundary");
}

void checkRateLimit()
{
    const policy::Pose command = policy::rateLimitedPose(
        {0.0, 0.0}, {1.0, -1.0}, 0.02, 0.5, 1.0);
    require(std::fabs(command.pitch - 0.01) < 1e-12,
            "pitch rate limit was not applied");
    require(std::fabs(command.yaw + 0.02) < 1e-12,
            "yaw rate limit was not applied");
}

void checkPhaseAlignment()
{
    const auto evaluate = [](double phase) {
        return policy::fieldScanPose(phase, 0.15, 0.35, -0.8, 0.8);
    };
    const auto current = evaluate(0.37);
    const double phase = policy::nearestPhase(
        evaluate, current, 0.2, 1.6, 1000);
    require(distance(current, evaluate(phase)) < 0.01,
            "scan phase did not align to the current head pose");
}

} // namespace

int main()
{
    checkFieldScanCycleIsContinuous();
    checkBallScanClosingEdgeIsContinuous();
    checkRateLimit();
    checkPhaseAlignment();
    std::cout << "head scan policy tests passed\n";
    return 0;
}
