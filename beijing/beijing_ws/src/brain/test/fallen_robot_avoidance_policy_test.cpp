#include "fallen_robot_avoidance_policy.h"

#include <cmath>
#include <iostream>
#include <stdexcept>

namespace policy = fallen_robot_avoidance_policy;

void expect(bool condition, const char *message)
{
    if (!condition) throw std::runtime_error(message);
}

int main()
{
    constexpr double kPi = 3.14159265358979323846;

    expect(policy::intersectsPath(2.8, 0.0, 0.0, 3.0, 0.3).intersects,
           "fallen region on the route was not detected");
    expect(!policy::intersectsPath(2.8, 0.9, 0.0, 3.0, 0.3).intersects,
           "fallen region outside the route corridor was detected");
    expect(!policy::intersectsPath(-1.0, 0.0, 0.0, 3.0, 0.3).intersects,
           "fallen region behind the robot was detected on a forward route");
    expect(policy::intersectsPath(3.3, 0.0, 0.0, 3.0, 0.3).intersects,
           "region edge inside the configured distance was not detected");
    expect(policy::intersectsPath(2.8, 0.75, 0.0, 3.0, 0.3, 0.85).intersects,
           "configured fallen footprint radius was not applied");
    expect(!policy::intersectsPath(2.8, 0.75, 0.0, 3.0, 0.3, 0.40).intersects,
           "upright-sized footprint was expanded unexpectedly");

    expect(policy::intersectsForwardSector(2.0, 0.0, 3.0, kPi / 4.0),
           "centered fallen region was not detected in the visual-kick sector");
    expect(policy::intersectsForwardSector(
               2.0 * std::cos(50.0 * kPi / 180.0),
               2.0 * std::sin(50.0 * kPi / 180.0),
               3.0,
               kPi / 4.0),
           "fallen region overlapping the sector edge was not detected");
    expect(!policy::intersectsForwardSector(
               2.0 * std::cos(70.0 * kPi / 180.0),
               2.0 * std::sin(70.0 * kPi / 180.0),
               3.0,
               kPi / 4.0),
           "fallen region outside the visual-kick sector was detected");

    auto temporal = policy::updateTemporalFallenState(0.0, false, true, 0.8);
    expect(!temporal.fallen,
           "a single fallen frame should not confirm a fallen robot");
    temporal = policy::updateTemporalFallenState(
        temporal.uprightScore, temporal.fallen, true, 0.8);
    expect(temporal.fallen,
           "repeated fallen evidence did not confirm a fallen robot");
    for (int i = 0; i < 4; ++i) {
        temporal = policy::updateTemporalFallenState(
            temporal.uprightScore, temporal.fallen, false, 0.0);
    }
    expect(temporal.fallen,
           "fallen state was cleared before upright hysteresis was satisfied");
    temporal = policy::updateTemporalFallenState(
        temporal.uprightScore, temporal.fallen, false, 0.0);
    expect(!temporal.fallen,
           "upright evidence did not clear the fallen state");

    expect(policy::selectAvoidanceSide(-1.0, 0.0, false, 3.0, 3.0, 1.0) < 0.0,
           "preferred avoidance side was not selected with equal clearance");
    expect(policy::selectAvoidanceSide(-1.0, 0.0, false, 3.0, 0.5, 1.0) > 0.0,
           "clear alternate avoidance side was not selected");
    expect(policy::selectAvoidanceSide(-1.0, -1.0, true, 3.0, 1.2, 1.0) < 0.0,
           "safe locked avoidance side was not preserved");
    expect(policy::selectAvoidanceSide(-1.0, -1.0, true, 3.0, 0.5, 1.0) > 0.0,
           "unsafe locked avoidance side was not released");

    const auto diagonalVelocity = policy::makeAvoidanceVelocity(
        0.0, 0.25, 0.35, 1.0, 1.2, 0.4);
    expect(std::fabs(diagonalVelocity.x - 0.25) < 1e-9 &&
               std::fabs(diagonalVelocity.y - 0.35) < 1e-9,
           "avoidance velocity did not preserve the checked diagonal direction");
    const auto limitedVelocity = policy::makeAvoidanceVelocity(
        kPi / 4.0, 0.25, 0.40, 1.0, 1.2, 0.4);
    expect(std::fabs(limitedVelocity.y) <= 0.4 + 1e-9,
           "avoidance velocity exceeded the final robot-frame limit");

    std::cout << "fallen robot avoidance policy tests passed\n";
    return 0;
}
