#include "robot_path_planner_policy.h"

#include <cmath>
#include <iostream>
#include <stdexcept>
#include <vector>

namespace policy = robot_path_planner_policy;

void expect(bool condition, const char *message)
{
    if (!condition) throw std::runtime_error(message);
}

int main()
{
    const policy::Position2D goal{4.0, 0.0};
    const auto direct = policy::planPath(goal, {});
    expect(direct.direct && direct.hasPath, "clear path was not kept direct");

    const std::vector<policy::CircleObstacle> one{{{2.0, 0.0}, 0.55}};
    const auto detour = policy::planPath(goal, one, 0.12);
    expect(!detour.direct && detour.hasPath, "single obstacle was not routed");
    expect(std::fabs(detour.waypoint.y) > 0.35,
           "detour waypoint did not pass a side of the obstacle");
    expect(detour.pathLength > detour.directLength,
           "detour did not have a longer route than the direct path");
    expect(detour.minimumClearance >= -1e-4,
           "detour first segment intersects the inflated obstacle");

    const std::vector<policy::CircleObstacle> gate{
        {{2.0, 0.0}, 0.65}, {{3.0, 0.0}, 0.65}};
    const auto multi = policy::planPath(goal, gate, 0.12);
    expect(!multi.direct && multi.hasPath,
           "multiple obstacles did not produce a connected route");
    expect(multi.pathLength > multi.directLength,
           "multi-obstacle route was unexpectedly direct");

    const auto locked = policy::planPath(goal, one, 0.12, detour.avoidanceSide,
                                         true, 4.0);
    expect(locked.avoidanceSide == detour.avoidanceSide,
           "previous avoidance side was not retained");
    const auto hardLocked = policy::planPath(
        goal, one, 0.12, detour.avoidanceSide, true, 0.0, 0.25, true);
    expect(hardLocked.hasPath &&
               hardLocked.avoidanceSide == detour.avoidanceSide,
           "hard avoidance-side lock allowed an early side switch");

    const std::vector<policy::CircleObstacle> originInside{
        {{0.40, 0.0}, 0.95}};
    const auto escape = policy::planPath(goal, originInside, 0.12);
    expect(!escape.direct && escape.hasPath && escape.escapingOverlap,
           "origin overlap did not produce an escape route");
    expect(escape.overlappingObstacleCount == 1,
           "origin overlap count was not reported");
    expect(escape.escapeClearanceGain > 0.1,
           "escape route did not increase obstacle clearance");
    expect(escape.waypoint.x < -0.1,
           "single-circle escape did not move away from its centre");

    const auto rearEscape = policy::planPath(
        goal, {{{-0.40, 0.0}, 0.95}}, 0.12);
    expect(rearEscape.hasPath && rearEscape.escapingOverlap &&
               rearEscape.waypoint.x > 0.1,
           "rear overlap was not allowed to escape forwards");

    const std::vector<policy::CircleObstacle> overlappingPair{
        {{0.35, 0.35}, 0.90}, {{0.35, -0.35}, 0.90}};
    const auto pairEscape = policy::planPath(goal, overlappingPair, 0.12);
    expect(pairEscape.hasPath && pairEscape.escapingOverlap,
           "overlapping circles did not produce an escape route");
    expect(pairEscape.overlappingObstacleCount == 2,
           "overlapping-circle count was not reported");
    expect(pairEscape.waypoint.x < -0.1,
           "overlapping-circle escape did not move away from both centres");

    const std::vector<policy::CircleObstacle> opposingPair{
        {{0.35, 0.0}, 0.80}, {{-0.35, 0.0}, 0.80}};
    const auto lateralEscape = policy::planPath(
        goal, opposingPair, 0.12, 1.0, true, 4.0);
    expect(lateralEscape.hasPath && lateralEscape.escapingOverlap,
           "opposing overlaps did not find a lateral escape");
    expect(lateralEscape.waypoint.y > 0.1,
           "escape-side lock was not applied to an overlap escape");

    const double speed = policy::velocityForPath(
        0.35, 1.7, 1.2, 3.0, 0.45, 0.70, 0.32);
    expect(speed > 0.5 && speed <= 1.2,
           "normal avoidance speed remained at the old crawl speed");
    const double hardLimitedSpeed = policy::velocityForPath(
        0.1, 2.0, 2.0, 3.0, 0.5, 0.70, 0.32, 0.40);
    expect(hardLimitedSpeed <= 0.700001,
           "preferred avoidance speed was not enforced as a magnitude limit");
    const double lateralAngle = policy::angleWithMinimumLateralSpeed(
        0.05, 0.70, 0.40, 1.0);
    expect(0.70 * std::sin(lateralAngle) >= 0.399,
           "avoidance angle did not clear the lateral dead zone");
    const double lockedLateralAngle = policy::angleWithMinimumLateralSpeed(
        0.05, 0.70, 0.40, -1.0);
    expect(lockedLateralAngle < 0.0 &&
               0.70 * std::fabs(std::sin(lockedLateralAngle)) >= 0.399,
           "minimum-vy adjustment did not preserve the selected side");
    const auto limitedVelocity = policy::limitVelocityMagnitude(
        {0.9, 1.2}, 0.70);
    expect(std::hypot(limitedVelocity.x, limitedVelocity.y) <= 0.700001,
           "avoidance vector magnitude was not limited");
    const double deadZoneAwareAngle = policy::angleWithMinimumLateralSpeed(
        3.14159265358979323846 / 6.0, 0.70, 0.40, 1.0);
    const double deadZoneAwareSpeed = policy::velocityForPath(
        deadZoneAwareAngle,
        0.9,
        0.4,
        3.0,
        0.45,
        0.70,
        0.32,
        0.40);
    expect(deadZoneAwareSpeed * std::sin(deadZoneAwareAngle) >=
               0.399,
           "avoidance path did not clear the configured lateral dead zone");
    expect(policy::velocityForPath(0.0, 0.0, 1.0, 1.0, 1.0) == 0.0,
           "zero forward limit produced a nonzero command");
    const double escapeAngle = std::atan2(
        escape.waypoint.y, escape.waypoint.x);
    expect(policy::velocityForPath(
               escapeAngle, 1.0, 1.0, 2.0, 0.0, 0.70, 0.32) > 0.3,
           "escape path produced a zero or dead-zone command");

    std::cout << "robot path planner policy tests passed\n";
    return 0;
}
