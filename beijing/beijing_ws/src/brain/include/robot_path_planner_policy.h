#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>

// Small, dependency-free visibility-graph policy for the visual robot
// obstacles.  It deliberately plans only the few recognized robots; the
// depth grid remains the local veto in Brain::distToObstacle().
namespace robot_path_planner_policy {

struct Position2D
{
    double x = 0.0;
    double y = 0.0;
};

struct CircleObstacle
{
    Position2D center;
    double radius = 0.0;
};

struct PathPlan
{
    bool direct = true;
    bool hasPath = true;
    bool escapingOverlap = false;
    Position2D waypoint;
    double pathLength = 0.0;
    double directLength = 0.0;
    double minimumClearance = std::numeric_limits<double>::infinity();
    double startClearance = std::numeric_limits<double>::infinity();
    double escapeClearanceGain = 0.0;
    double avoidanceSide = 0.0;
    std::size_t overlappingObstacleCount = 0;
};

inline bool finite(const Position2D &p)
{
    return std::isfinite(p.x) && std::isfinite(p.y);
}

inline double distance(const Position2D &a, const Position2D &b)
{
    return std::hypot(a.x - b.x, a.y - b.y);
}

inline double dot(const Position2D &a, const Position2D &b)
{
    return a.x * b.x + a.y * b.y;
}

inline Position2D operator+(const Position2D &a, const Position2D &b)
{
    return {a.x + b.x, a.y + b.y};
}

inline Position2D operator-(const Position2D &a, const Position2D &b)
{
    return {a.x - b.x, a.y - b.y};
}

inline Position2D operator*(const Position2D &a, double scale)
{
    return {a.x * scale, a.y * scale};
}

inline double pointToSegmentDistance(
    const Position2D &point,
    const Position2D &start,
    const Position2D &end)
{
    const Position2D segment = end - start;
    const double lengthSquared = dot(segment, segment);
    if (!std::isfinite(lengthSquared) || lengthSquared <= 1e-12) {
        return distance(point, start);
    }
    const double projection = std::clamp(
        dot(point - start, segment) / lengthSquared, 0.0, 1.0);
    return distance(point, start + segment * projection);
}

inline bool segmentClear(
    const Position2D &start,
    const Position2D &end,
    const CircleObstacle &obstacle,
    double epsilon = 1e-5)
{
    return pointToSegmentDistance(obstacle.center, start, end) >=
        std::max(0.0, obstacle.radius) - epsilon;
}

inline double segmentClearance(
    const Position2D &start,
    const Position2D &end,
    const std::vector<CircleObstacle> &obstacles)
{
    double clearance = std::numeric_limits<double>::infinity();
    for (const auto &obstacle : obstacles) {
        if (!finite(obstacle.center) || !std::isfinite(obstacle.radius)) {
            continue;
        }
        clearance = std::min(
            clearance,
            pointToSegmentDistance(obstacle.center, start, end) -
                std::max(0.0, obstacle.radius));
    }
    return clearance;
}

inline PathPlan planPath(
    Position2D goal,
    const std::vector<CircleObstacle> &inputObstacles,
    double extraClearance = 0.12,
    double previousSide = 0.0,
    bool lockPreviousSide = false,
    double switchPenalty = 0.8,
    double initialTurnPenalty = 0.25,
    bool hardLockPreviousSide = false)
{
    PathPlan result;
    result.waypoint = goal;
    result.directLength = finite(goal) ? distance({}, goal) : 0.0;
    result.pathLength = result.directLength;
    if (!finite(goal) || result.directLength <= 1e-6) return result;

    std::vector<CircleObstacle> obstacles;
    obstacles.reserve(inputObstacles.size());
    for (const auto &input : inputObstacles) {
        if (!finite(input.center) || !std::isfinite(input.radius) ||
            input.radius <= 0.0) {
            continue;
        }
        CircleObstacle obstacle = input;
        obstacle.radius += std::max(0.0, extraClearance);
        obstacles.push_back(obstacle);
    }

    bool directClear = true;
    for (const auto &obstacle : obstacles) {
        if (!segmentClear({}, goal, obstacle)) {
            directClear = false;
            break;
        }
    }
    result.direct = directClear;
    result.startClearance = segmentClearance({}, {}, obstacles);
    result.minimumClearance = segmentClearance({}, goal, obstacles);
    if (directClear || obstacles.empty()) return result;

    std::vector<std::size_t> overlappingObstacles;
    for (std::size_t i = 0; i < obstacles.size(); ++i) {
        if (distance({}, obstacles[i].center) < obstacles[i].radius - 1e-5) {
            overlappingObstacles.push_back(i);
        }
    }

    if (!overlappingObstacles.empty()) {
        // A conventional visibility graph cannot leave an obstacle containing
        // its origin: every first edge intersects that obstacle. Sample an
        // outward ray and return a short local escape waypoint. Replanning a
        // bounded step avoids rejecting every ray only because a different,
        // more distant obstacle intersects the full route out. The strict
        // candidate condition makes distance from every overlapping centre
        // non-decreasing along the ray.
        constexpr int kEscapeDirectionSamples = 72;
        constexpr double kPi = 3.14159265358979323846;
        const double goalAngle = std::atan2(goal.y, goal.x);
        bool foundStrictCandidate = false;
        bool foundProgressCandidate = false;
        double bestScore = -std::numeric_limits<double>::infinity();
        Position2D bestWaypoint{};
        double bestEndClearance = result.startClearance;

        for (int sample = 0; sample < kEscapeDirectionSamples; ++sample) {
            const double angle = -kPi +
                2.0 * kPi * sample / kEscapeDirectionSamples;
            const Position2D direction{std::cos(angle), std::sin(angle)};
            bool monotonic = true;
            double exitDistance = 0.0;
            double minimumIndividualGain =
                std::numeric_limits<double>::infinity();

            for (const std::size_t index : overlappingObstacles) {
                const auto &obstacle = obstacles[index];
                const double along = dot(obstacle.center, direction);
                // Positive means the first motion approaches this centre.
                if (along > 1e-5) monotonic = false;
                const double centerDistanceSquared =
                    dot(obstacle.center, obstacle.center);
                const double discriminant = std::max(
                    0.0,
                    along * along + obstacle.radius * obstacle.radius -
                        centerDistanceSquared);
                exitDistance = std::max(
                    exitDistance, along + std::sqrt(discriminant));
            }

            const double waypointDistance = std::min(
                0.60, std::max(0.20, exitDistance + 0.15));
            const Position2D waypoint = direction * waypointDistance;
            bool clearOfOtherObstacles = true;
            for (std::size_t i = 0; i < obstacles.size(); ++i) {
                const bool wasOverlapping = std::find(
                    overlappingObstacles.begin(),
                    overlappingObstacles.end(),
                    i) != overlappingObstacles.end();
                if (!wasOverlapping &&
                    !segmentClear({}, waypoint, obstacles[i])) {
                    clearOfOtherObstacles = false;
                    break;
                }
                if (wasOverlapping) {
                    const double initialClearance =
                        distance({}, obstacles[i].center) - obstacles[i].radius;
                    const double endpointClearance =
                        distance(waypoint, obstacles[i].center) -
                            obstacles[i].radius;
                    minimumIndividualGain = std::min(
                        minimumIndividualGain,
                        endpointClearance - initialClearance);
                }
            }
            if (!clearOfOtherObstacles) continue;

            const double endpointClearance = segmentClearance(
                waypoint, waypoint, obstacles);
            const bool makesProgress = minimumIndividualGain > 1e-4 &&
                endpointClearance > result.startClearance + 1e-4;
            if (!makesProgress) continue;
            if (foundStrictCandidate && !monotonic) continue;
            if (monotonic && !foundStrictCandidate) {
                foundStrictCandidate = true;
                bestScore = -std::numeric_limits<double>::infinity();
            }

            const double angleToGoal = std::fabs(std::atan2(
                std::sin(angle - goalAngle), std::cos(angle - goalAngle)));
            const double sidePenalty = lockPreviousSide &&
                    std::fabs(previousSide) > 0.1 &&
                    waypoint.y * previousSide < -0.05
                ? std::max(0.0, switchPenalty)
                : 0.0;
            if (hardLockPreviousSide &&
                std::fabs(previousSide) > 0.1 &&
                waypoint.y * previousSide < -0.05) {
                continue;
            }
            const double score = 8.0 * minimumIndividualGain +
                2.0 * endpointClearance - waypointDistance -
                std::max(0.0, initialTurnPenalty) * angleToGoal - sidePenalty;
            if (score > bestScore) {
                bestScore = score;
                bestWaypoint = waypoint;
                bestEndClearance = endpointClearance;
                foundProgressCandidate = true;
            }
        }

        if (foundProgressCandidate) {
            result.direct = false;
            result.hasPath = true;
            result.escapingOverlap = true;
            result.waypoint = bestWaypoint;
            result.pathLength = distance({}, bestWaypoint);
            result.minimumClearance = result.startClearance;
            result.escapeClearanceGain =
                bestEndClearance - result.startClearance;
            result.avoidanceSide = bestWaypoint.y > 0.05
                ? 1.0 : (bestWaypoint.y < -0.05 ? -1.0 : 0.0);
            result.overlappingObstacleCount = overlappingObstacles.size();
            return result;
        }

        result.direct = false;
        result.hasPath = false;
        result.pathLength = std::numeric_limits<double>::infinity();
        result.overlappingObstacleCount = overlappingObstacles.size();
        return result;
    }

    struct Node
    {
        Position2D point;
        std::size_t obstacleIndex = std::numeric_limits<std::size_t>::max();
        int sampleIndex = -1;
    };
    constexpr int kSamplesPerCircle = 16;
    constexpr double kPi = 3.14159265358979323846;
    std::vector<Node> nodes;
    nodes.push_back({{}, std::numeric_limits<std::size_t>::max(), -1});
    nodes.push_back({goal, std::numeric_limits<std::size_t>::max(), -1});
    for (std::size_t i = 0; i < obstacles.size(); ++i) {
        const auto &obstacle = obstacles[i];
        // A slight outward offset keeps floating point tangent contacts from
        // being classified as a collision by the next control cycle.
        const double radius = obstacle.radius + 0.03;
        for (int sample = 0; sample < kSamplesPerCircle; ++sample) {
            const double angle = 2.0 * kPi * sample / kSamplesPerCircle;
            nodes.push_back({
                {obstacle.center.x + radius * std::cos(angle),
                 obstacle.center.y + radius * std::sin(angle)},
                i,
                sample});
        }
    }

    const std::size_t nodeCount = nodes.size();
    const double infinity = std::numeric_limits<double>::infinity();
    std::vector<double> cost(nodeCount, infinity);
    std::vector<std::size_t> previous(nodeCount, nodeCount);
    std::vector<bool> visited(nodeCount, false);
    cost[0] = 0.0;
    for (std::size_t iteration = 0; iteration < nodeCount; ++iteration) {
        std::size_t current = nodeCount;
        double currentCost = infinity;
        for (std::size_t i = 0; i < nodeCount; ++i) {
            if (!visited[i] && cost[i] < currentCost) {
                current = i;
                currentCost = cost[i];
            }
        }
        if (current == nodeCount || current == 1) break;
        visited[current] = true;
        for (std::size_t next = 1; next < nodeCount; ++next) {
            if (visited[next] || next == current) continue;
            if (current == 0 && hardLockPreviousSide &&
                std::fabs(previousSide) > 0.1 &&
                nodes[next].point.y * previousSide < -0.05) {
                continue;
            }
            bool clear = true;
            for (const auto &obstacle : obstacles) {
                if (!segmentClear(nodes[current].point, nodes[next].point, obstacle)) {
                    clear = false;
                    break;
                }
            }
            if (!clear) {
                // Consecutive boundary samples represent a safe circular arc,
                // even though their straight chord crosses the inflated disk.
                const bool sameObstacle =
                    nodes[current].obstacleIndex !=
                        std::numeric_limits<std::size_t>::max() &&
                    nodes[current].obstacleIndex == nodes[next].obstacleIndex;
                const int sampleDelta = std::abs(
                    nodes[current].sampleIndex - nodes[next].sampleIndex);
                const bool adjacentArc = sameObstacle &&
                    (sampleDelta == 1 || sampleDelta == kSamplesPerCircle - 1);
                if (!adjacentArc) continue;
            }

            double edgeLength = distance(nodes[current].point, nodes[next].point);
            if (nodes[current].obstacleIndex == nodes[next].obstacleIndex &&
                nodes[current].obstacleIndex !=
                    std::numeric_limits<std::size_t>::max()) {
                edgeLength = 2.0 * (obstacles[nodes[current].obstacleIndex].radius +
                    0.03) * std::sin(3.14159265358979323846 /
                        kSamplesPerCircle);
            }
            if (!std::isfinite(edgeLength) || edgeLength <= 1e-6) continue;
            double edgeCost = edgeLength;
            if (current == 0) {
                edgeCost += std::max(0.0, initialTurnPenalty) * std::fabs(
                    std::atan2(nodes[next].point.y, nodes[next].point.x));
                if (lockPreviousSide && std::fabs(previousSide) > 0.1 &&
                    nodes[next].point.y * previousSide < -0.05) {
                    edgeCost += std::max(0.0, switchPenalty);
                }
            }
            const double candidate = cost[current] + edgeCost;
            if (candidate < cost[next]) {
                cost[next] = candidate;
                previous[next] = current;
            }
        }
    }

    if (!std::isfinite(cost[1])) {
        result.direct = false;
        result.hasPath = false;
        result.pathLength = infinity;
        return result;
    }

    std::vector<std::size_t> route;
    for (std::size_t node = 1; node != nodeCount; node = previous[node]) {
        route.push_back(node);
        if (node == 0 || previous[node] == nodeCount) break;
    }
    if (route.empty() || route.back() != 0) {
        result.direct = false;
        result.hasPath = false;
        result.pathLength = infinity;
        return result;
    }
    std::reverse(route.begin(), route.end());
    if (route.size() > 1) {
        result.waypoint = nodes[route[1]].point;
        result.avoidanceSide = result.waypoint.y > 0.05
            ? 1.0 : (result.waypoint.y < -0.05 ? -1.0 : 0.0);
    }
    result.pathLength = cost[1];
    result.minimumClearance = segmentClearance({}, result.waypoint, obstacles);
    result.direct = false;
    return result;
}

inline double velocityForPath(
    double pathAngle,
    double vxLimit,
    double vyLimit,
    double distanceToGoal,
    double minimumClearance,
    double preferredSpeed = 0.70,
    double minimumSpeed = 0.32,
    double minimumLateralSpeed = -1.0)
{
    if (!std::isfinite(pathAngle) || !std::isfinite(vxLimit) ||
        !std::isfinite(vyLimit) || !std::isfinite(distanceToGoal) ||
        !std::isfinite(minimumClearance)) {
        return 0.0;
    }
    vxLimit = std::max(0.0, vxLimit);
    vyLimit = std::max(0.0, vyLimit);
    const double c = std::fabs(std::cos(pathAngle));
    const double s = std::fabs(std::sin(pathAngle));
    double componentLimit = std::numeric_limits<double>::infinity();
    if (c > 1e-6) componentLimit = std::min(componentLimit, vxLimit / c);
    if (s > 1e-6) componentLimit = std::min(componentLimit, vyLimit / s);
    if (!std::isfinite(componentLimit)) componentLimit = std::max(vxLimit, vyLimit);
    componentLimit = std::min(
        componentLimit, std::max(0.0, preferredSpeed));

    const double turnScale = 1.0 - 0.35 * std::clamp(
        std::fabs(pathAngle) / (3.14159265358979323846 / 2.0), 0.0, 1.0);
    const double clearanceScale = std::clamp(
        (minimumClearance + 0.10) / 0.60, 0.60, 1.0);
    const double goalScale = std::clamp(distanceToGoal / 0.8, 0.45, 1.0);
    const double forwardRequested = std::max(0.0, preferredSpeed) * turnScale *
        clearanceScale * goalScale;
    double requested = forwardRequested;

    // A small lateral component can be lost in the robot's command dead zone.
    // Raise the scalar only when necessary to reach the configured minimum y
    // component; componentLimit still enforces both physical axis limits.
    if (std::isfinite(minimumLateralSpeed) &&
        minimumLateralSpeed > 1e-6 && s > 1e-6) {
        requested = std::max(requested, minimumLateralSpeed / s);
    }
    if (requested < 1e-6 || componentLimit <= 1e-6) return 0.0;
    return std::min(componentLimit, std::max(
        requested, std::max(0.0, minimumSpeed)));
}

inline double angleWithMinimumLateralSpeed(
    double pathAngle,
    double maximumSpeed,
    double minimumLateralSpeed,
    double preferredSide = 0.0)
{
    constexpr double kHalfPi = 1.57079632679489661923;
    if (!std::isfinite(pathAngle) || !std::isfinite(maximumSpeed) ||
        !std::isfinite(minimumLateralSpeed) || maximumSpeed <= 1e-6 ||
        minimumLateralSpeed <= 1e-6) {
        return pathAngle;
    }
    const double ratio = std::clamp(
        minimumLateralSpeed / maximumSpeed, 0.0, 1.0);
    const double minimumAngle = std::asin(ratio);
    if (std::fabs(std::sin(pathAngle)) + 1e-6 >= ratio) return pathAngle;

    double side = std::fabs(preferredSide) > 0.1
        ? (preferredSide >= 0.0 ? 1.0 : -1.0)
        : (pathAngle > 1e-6 ? 1.0 : (pathAngle < -1e-6 ? -1.0 : 0.0));
    if (side == 0.0) side = preferredSide >= 0.0 ? 1.0 : -1.0;
    const double forwardSign = std::cos(pathAngle) >= 0.0 ? 1.0 : -1.0;
    return forwardSign >= 0.0
        ? side * minimumAngle
        : side * (kHalfPi + (kHalfPi - minimumAngle));
}

inline Position2D limitVelocityMagnitude(
    Position2D velocity,
    double maximumSpeed)
{
    if (!finite(velocity) || !std::isfinite(maximumSpeed) ||
        maximumSpeed <= 0.0) {
        return {};
    }
    const double speed = std::hypot(velocity.x, velocity.y);
    if (speed <= maximumSpeed || speed <= 1e-9) return velocity;
    return velocity * (maximumSpeed / speed);
}

} // namespace robot_path_planner_policy
