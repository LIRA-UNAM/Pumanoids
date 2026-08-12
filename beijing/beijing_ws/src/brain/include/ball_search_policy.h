#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>

namespace ball_search_policy {

enum class SetPlay {
    None,
    GoalKick,
    CornerKick,
};

struct Field {
    double length = 0.0;
    double width = 0.0;
    double penaltyAreaLength = 0.0;
    double penaltyAreaWidth = 0.0;
    double goalAreaLength = 0.0;
    double goalAreaWidth = 0.0;
};

struct Point {
    double x = 0.0;
    double y = 0.0;
};

struct Waypoint {
    double x = 0.0;
    double y = 0.0;
    double theta = 0.0;
};

struct Context {
    Field field;
    SetPlay setPlay = SetPlay::None;
    bool ourSetPlay = false;
    int searcherIndex = 0;
    int searcherCount = 1;
    bool prioritizeLastObservation = false;
    bool lastObservationValid = false;
    Point lastObservation;
};

inline double fieldMargin(const Field &field)
{
    return std::clamp(0.06 * std::min(field.length, field.width), 0.55, 0.9);
}

inline Point clampToField(Point point, const Field &field)
{
    const double margin = fieldMargin(field);
    const double xLimit = std::max(0.0, field.length / 2.0 - margin);
    const double yLimit = std::max(0.0, field.width / 2.0 - margin);
    point.x = std::clamp(point.x, -xLimit, xLimit);
    point.y = std::clamp(point.y, -yLimit, yLimit);
    return point;
}

inline Waypoint makeWaypoint(Point point, const Field &field)
{
    point = clampToField(point, field);
    return {point.x, point.y, std::atan2(-point.y, -point.x)};
}

inline std::vector<Waypoint> distribute(
    const std::vector<Point> &candidates,
    const Context &context)
{
    std::vector<Waypoint> plan;
    if (candidates.empty()) return plan;

    const std::size_t count = candidates.size();
    const int searcherCount = std::max(1, context.searcherCount);
    const int normalizedIndex =
        ((context.searcherIndex % searcherCount) + searcherCount) % searcherCount;
    std::vector<bool> used(count, false);
    plan.reserve(count);

    // Prime-sized candidate sets make this a complete traversal for the usual
    // one-to-five searchers. The fallback below also keeps arbitrary inputs safe.
    for (std::size_t step = 0; step < count; ++step) {
        const std::size_t index =
            (static_cast<std::size_t>(normalizedIndex) +
             step * static_cast<std::size_t>(searcherCount)) % count;
        if (used[index]) continue;
        used[index] = true;
        plan.push_back(makeWaypoint(candidates[index], context.field));
    }
    for (std::size_t step = 0; step < count; ++step) {
        const std::size_t index =
            (static_cast<std::size_t>(normalizedIndex) + step) % count;
        if (used[index]) continue;
        used[index] = true;
        plan.push_back(makeWaypoint(candidates[index], context.field));
    }
    return plan;
}

inline std::vector<Point> normalCandidates(const Field &field)
{
    const double margin = fieldMargin(field);
    const double xEdge = std::max(0.0, field.length / 2.0 - margin);
    const double yEdge = std::max(0.0, field.width / 2.0 - margin);
    const double xInner = 0.35 * xEdge;
    return {
        {-xEdge, yEdge},
        {-xEdge, 0.0},
        {-xEdge, -yEdge},
        {-xInner, -yEdge},
        {-xInner, yEdge},
        {0.0, 0.0},
        {xInner, -yEdge},
        {xInner, yEdge},
        {xEdge, yEdge},
        {xEdge, 0.0},
        {xEdge, -yEdge},
    };
}

inline std::vector<Point> goalKickCandidates(const Context &context)
{
    const Field &field = context.field;
    const double endSign = context.ourSetPlay ? -1.0 : 1.0;
    const double halfLength = field.length / 2.0;
    const double halfWidth = field.width / 2.0;
    const double margin = fieldMargin(field);
    const double innerY = std::min(
        halfWidth - margin,
        std::max(0.7, 0.32 * field.goalAreaWidth));
    const double outerY = std::min(
        halfWidth - margin,
        std::max(innerY, 0.34 * field.penaltyAreaWidth));

    // Attackers may inspect their own goal area. For an opponent goal kick all
    // points remain outside the opponent penalty area and act as observation posts.
    const double xMagnitude = context.ourSetPlay
        ? halfLength - std::max(0.5, 0.55 * field.goalAreaLength)
        : halfLength - field.penaltyAreaLength - margin;
    const double x = endSign * std::max(0.0, xMagnitude);
    const double supportX = endSign * std::max(
        0.0,
        context.ourSetPlay
            ? halfLength - field.goalAreaLength - margin
            : halfLength - field.penaltyAreaLength - 2.0 * margin);

    return {
        {x, 0.0},
        {x, innerY},
        {x, -innerY},
        {x, outerY},
        {x, -outerY},
        {supportX, 0.55 * outerY},
        {supportX, -0.55 * outerY},
    };
}

inline std::vector<Point> cornerKickCandidates(const Context &context)
{
    const Field &field = context.field;
    const double endSign = context.ourSetPlay ? 1.0 : -1.0;
    const double halfLength = field.length / 2.0;
    const double halfWidth = field.width / 2.0;
    const double margin = fieldMargin(field);
    const double edgeY = std::max(0.0, halfWidth - margin);
    const double boxY = std::min(
        edgeY,
        std::max(1.0, field.penaltyAreaWidth / 2.0 + margin));

    // Opponent corners are observed from outside our penalty area. Own corners
    // can be inspected near either corner because the controller has no side bit.
    const double xMagnitude = context.ourSetPlay
        ? halfLength - margin
        : halfLength - field.penaltyAreaLength - margin;
    const double x = endSign * std::max(0.0, xMagnitude);
    const double supportX = endSign * std::max(
        0.0,
        xMagnitude - (context.ourSetPlay ? 1.5 * margin : margin));

    return {
        {x, edgeY},
        {x, -edgeY},
        {x, boxY},
        {x, -boxY},
        {supportX, 0.0},
        {supportX, 0.55 * boxY},
        {supportX, -0.55 * boxY},
    };
}

inline std::vector<Waypoint> makePlan(const Context &context)
{
    std::vector<Point> candidates;
    switch (context.setPlay) {
    case SetPlay::GoalKick:
        candidates = goalKickCandidates(context);
        break;
    case SetPlay::CornerKick:
        candidates = cornerKickCandidates(context);
        break;
    case SetPlay::None:
    default:
        candidates = normalCandidates(context.field);
        break;
    }

    auto plan = distribute(candidates, context);
    if (context.setPlay == SetPlay::None &&
        context.prioritizeLastObservation &&
        context.lastObservationValid) {
        const Waypoint last = makeWaypoint(
            context.lastObservation, context.field);
        const auto duplicate = std::find_if(
            plan.begin(), plan.end(), [&](const Waypoint &waypoint) {
                return std::hypot(
                    waypoint.x - last.x,
                    waypoint.y - last.y) < 0.75;
            });
        if (duplicate == plan.end()) {
            plan.insert(plan.begin(), last);
        } else {
            std::rotate(plan.begin(), duplicate, duplicate + 1);
        }
    }
    return plan;
}

} // namespace ball_search_policy
