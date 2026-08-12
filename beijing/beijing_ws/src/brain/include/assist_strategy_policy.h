#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>

namespace assist_strategy_policy {

struct Point {
    double x = 0.0;
    double y = 0.0;
};

struct Field {
    double length = 0.0;
    double width = 0.0;
    double penaltyAreaLength = 0.0;
    double goalAreaLength = 0.0;
};

enum Slot : std::size_t {
    NONE = 0,
    COVER_MID = 1,
    SHADOW_SUPPORT = 2,
    ANCHOR_COVER = 3,
    WIDE_OUTLET = 4,
};

using Targets = std::array<Point, 5>;

inline Point operator+(const Point &lhs, const Point &rhs)
{
    return {lhs.x + rhs.x, lhs.y + rhs.y};
}

inline Point operator-(const Point &lhs, const Point &rhs)
{
    return {lhs.x - rhs.x, lhs.y - rhs.y};
}

inline Point operator*(const Point &point, double scale)
{
    return {point.x * scale, point.y * scale};
}

inline double norm(const Point &point)
{
    return std::hypot(point.x, point.y);
}

inline Point normalizedOr(const Point &point,
                          const Point &fallback = {1.0, 0.0})
{
    const double length = norm(point);
    if (length < 1e-9 || !std::isfinite(length)) return fallback;
    return point * (1.0 / length);
}

inline double kickLaneClearance(const Point &candidate, const Point &ball,
                                double kickDir)
{
    const double safeKickDir = std::isfinite(kickDir) ? kickDir : 0.0;
    const Point kickUnit{std::cos(safeKickDir), std::sin(safeKickDir)};
    const Point relative = candidate - ball;
    constexpr double protectedKickLength = 4.5;
    const double projection = std::clamp(
        relative.x * kickUnit.x + relative.y * kickUnit.y,
        0.0,
        protectedKickLength);
    return norm(relative - kickUnit * projection);
}

inline Point clampToAssistField(Point point, const Field &field)
{
    const double xMin = -field.length / 2.0 + field.goalAreaLength + 0.3;
    const double xMax = field.length / 2.0 - field.penaltyAreaLength - 0.3;
    const double yMax = field.width / 2.0 - 0.7;
    point.x = std::clamp(point.x, xMin, xMax);
    point.y = std::clamp(point.y, -yMax, yMax);
    return point;
}

inline double minimumSlotSpacing(const Field &field)
{
    // 1.5 m on the 22 m RoboLeague field, scaled down conservatively.
    return std::clamp(1.5 * field.length / 22.0, 0.8, 1.6);
}

inline double defensiveDangerBoundary(const Field &field)
{
    const double margin = std::max(0.6, 1.5 * field.length / 22.0);
    return -field.length / 2.0 + field.penaltyAreaLength + margin;
}

inline Point rawShadowTarget(const Point &ball, int side, const Field &field)
{
    const double ownGoalX = -field.length / 2.0;
    const Point ownGoal{ownGoalX, 0.0};
    const Point goalToBall = normalizedOr(ball - ownGoal);
    const Point goalNormal{-goalToBall.y, goalToBall.x};
    const int safeSide = side >= 0 ? 1 : -1;

    const double lateral = std::clamp(0.23 * field.width, 1.1, 3.4);
    const double dangerBoundary = defensiveDangerBoundary(field);
    const double attackProgress = std::clamp(
        (ball.x - dangerBoundary) /
            std::max(field.length / 2.0 - dangerBoundary, 0.5),
        0.0,
        1.0);
    const double forwardLead = field.length *
        (0.005 + 0.015 * attackProgress);
    const Point wideSupport{
        ball.x + forwardLead,
        ball.y + safeSide * lateral,
    };

    if (ball.x >= dangerBoundary) {
        return clampToAssistField(wideSupport, field);
    }

    const double transitionDepth = std::max(1.0, 0.08 * field.length);
    const double dangerBlend = std::clamp(
        (dangerBoundary - ball.x) / transitionDepth,
        0.0,
        1.0);
    const double retreatDistance = std::clamp(
        field.length * (0.05 + 0.03 * dangerBlend),
        0.8,
        1.8);
    const Point retreatSupport =
        ball - goalToBall * retreatDistance +
        goalNormal * (safeSide * lateral * 0.85);
    return clampToAssistField(
        wideSupport * (1.0 - dangerBlend) + retreatSupport * dangerBlend,
        field);
}

inline int preferredShadowSide(const Point &ball, double leaderKickDir,
                               int leaderId, const Field &field)
{
    const Point left = rawShadowTarget(ball, 1, field);
    const Point right = rawShadowTarget(ball, -1, field);
    const double leftKickClearance = kickLaneClearance(
        left, ball, leaderKickDir);
    const double rightKickClearance = kickLaneClearance(
        right, ball, leaderKickDir);
    if (std::fabs(leftKickClearance - rightKickClearance) > 0.15) {
        return leftKickClearance > rightKickClearance ? 1 : -1;
    }
    if (std::fabs(std::fabs(left.y) - std::fabs(right.y)) > 1e-6) {
        return std::fabs(left.y) < std::fabs(right.y) ? 1 : -1;
    }
    return leaderId % 2 == 0 ? 1 : -1;
}

inline Point rawSlotTarget(Slot slot, const Point &ball, int shadowSide,
                           const Field &field)
{
    const double ownGoalX = -field.length / 2.0;
    const Point ownGoal{ownGoalX, 0.0};
    const Point goalToBall = normalizedOr(ball - ownGoal);
    const Point goalNormal{-goalToBall.y, goalToBall.x};
    Point target{ownGoalX + field.goalAreaLength + 0.3, 0.0};

    if (slot == COVER_MID) {
        const Point guardedGoal{
            ownGoalX,
            std::clamp(ball.y, -0.084 * field.width, 0.084 * field.width),
        };
        const Point guardedToBall = ball - guardedGoal;
        const double guardedDistance = norm(guardedToBall);
        const double progress = std::clamp(
            (ball.x + 0.5 * field.length) / (0.722222 * field.length),
            0.0,
            1.0);
        const double ballGap = field.length *
            (0.194444 + 0.055556 * progress);
        const double depth = std::clamp(
            guardedDistance - ballGap,
            field.goalAreaLength,
            0.5 * field.length);
        target = guardedGoal + normalizedOr(guardedToBall) * depth;
        target.y = std::clamp(
            target.y, -0.224 * field.width, 0.224 * field.width);
    } else if (slot == SHADOW_SUPPORT) {
        target = rawShadowTarget(ball, shadowSide, field);
    } else if (slot == ANCHOR_COVER) {
        // Reference COVER: a stable radius from our goal, well behind COVER_MID.
        const double anchorDepth = std::max(
            field.goalAreaLength + 0.3,
            3.0 * field.length / 22.0);
        target = ownGoal + goalToBall * anchorDepth;
        target.y = std::clamp(
            target.y, -0.22 * field.width, 0.22 * field.width);
    } else if (slot == WIDE_OUTLET) {
        const int wideSide = shadowSide >= 0 ? -1 : 1;
        const double outletBack = std::clamp(
            3.8 * field.length / 22.0, 1.6, 3.8);
        target = ball - goalToBall * outletBack +
            goalNormal * (wideSide * std::min(3.4, 0.25 * field.width));
        target.x = std::min(target.x, ball.x - 0.5);
    }

    return clampToAssistField(target, field);
}

inline bool clearsPlacedTargets(
    const Point &candidate,
    const std::array<Point, 4> &placed,
    std::size_t placedCount,
    double spacing)
{
    for (std::size_t i = 0; i < placedCount; ++i) {
        if (norm(candidate - placed[i]) + 1e-6 < spacing) return false;
    }
    return true;
}

inline Point nearestSeparatedTarget(
    const Point &desired,
    const std::array<Point, 4> &placed,
    std::size_t placedCount,
    double spacing,
    const Field &field)
{
    const Point clampedDesired = clampToAssistField(desired, field);
    if (clearsPlacedTargets(
            clampedDesired, placed, placedCount, spacing)) {
        return clampedDesired;
    }

    Point best = clampedDesired;
    double bestScore = std::numeric_limits<double>::infinity();
    auto consider = [&](Point candidate) {
        candidate = clampToAssistField(candidate, field);
        if (!clearsPlacedTargets(candidate, placed, placedCount, spacing)) {
            return;
        }
        const double score = norm(candidate - clampedDesired);
        if (score + 1e-9 < bestScore) {
            best = candidate;
            bestScore = score;
        }
    };

    constexpr int angleSamples = 64;
    constexpr double pi = 3.14159265358979323846;
    const double radialStep = spacing / 8.0;
    for (double radius = radialStep;
         radius <= spacing * 4.0 + 1e-9;
         radius += radialStep) {
        for (int i = 0; i < angleSamples; ++i) {
            const double angle = 2.0 * pi * i / angleSamples;
            consider({
                clampedDesired.x + radius * std::cos(angle),
                clampedDesired.y + radius * std::sin(angle),
            });
        }
        if (std::isfinite(bestScore)) break;
    }

    if (std::isfinite(bestScore)) return best;

    // Bounded deterministic fallback for overlapping circles near a boundary.
    const double xMin = -field.length / 2.0 + field.goalAreaLength + 0.3;
    const double xMax = field.length / 2.0 - field.penaltyAreaLength - 0.3;
    const double yMax = field.width / 2.0 - 0.7;
    const double gridStep = spacing / 2.0;
    for (double x = xMin; x <= xMax + 1e-9; x += gridStep) {
        for (double y = -yMax; y <= yMax + 1e-9; y += gridStep) {
            consider({x, y});
        }
    }
    return best;
}

inline Targets calculateTargets(std::size_t assistCount, const Point &ball,
                                int shadowSide, const Field &field)
{
    Targets targets{};
    targets[COVER_MID] = rawSlotTarget(
        COVER_MID, ball, shadowSide, field);
    targets[SHADOW_SUPPORT] = rawSlotTarget(
        SHADOW_SUPPORT, ball, shadowSide, field);
    targets[ANCHOR_COVER] = rawSlotTarget(
        ANCHOR_COVER, ball, shadowSide, field);
    targets[WIDE_OUTLET] = rawSlotTarget(
        WIDE_OUTLET, ball, shadowSide, field);

    if (assistCount == 0) return targets;

    std::array<Point, 4> placed{};
    std::size_t placedCount = 0;
    const double spacing = minimumSlotSpacing(field);
    auto place = [&](Slot slot) {
        targets[slot] = nearestSeparatedTarget(
            targets[slot], placed, placedCount, spacing, field);
        placed[placedCount++] = targets[slot];
    };

    // Keep the deep anchor stable. Dynamic roles absorb any required offset.
    if (assistCount >= 3) place(ANCHOR_COVER);
    if (assistCount >= 2) place(SHADOW_SUPPORT);
    place(COVER_MID);
    if (assistCount >= 4) place(WIDE_OUTLET);
    return targets;
}

inline int selectShadowSide(int currentSide, int preferredSide,
                            bool ownerChanged, bool holdExpired,
                            bool currentOutside,
                            bool meaningfulImprovement)
{
    const int safePreferred = preferredSide >= 0 ? 1 : -1;
    if (std::abs(currentSide) != 1) return safePreferred;
    if (ownerChanged || currentSide == safePreferred || !holdExpired) {
        return currentSide;
    }
    return currentOutside || meaningfulImprovement
        ? safePreferred
        : currentSide;
}

inline double yieldTimeoutMsecs(double routeDistance, double maxSpeed,
                                double configuredTimeoutMsecs)
{
    const double safeSpeed = std::max(maxSpeed, 0.05);
    const double travelMsecs = std::max(routeDistance, 0.0) /
        safeSpeed * 1000.0;
    return std::max(configuredTimeoutMsecs, travelMsecs + 1000.0);
}

inline int trafficPriority(Slot slot, bool yielding)
{
    if (yielding) return 0;
    switch (slot) {
    case ANCHOR_COVER: return 4;
    case COVER_MID: return 3;
    case SHADOW_SUPPORT: return 2;
    case WIDE_OUTLET: return 1;
    default: return -1;
    }
}

inline bool teammateHasRightOfWay(Slot mySlot, bool myYielding, int myId,
                                  Slot teammateSlot, bool teammateYielding,
                                  int teammateId)
{
    const int myPriority = trafficPriority(mySlot, myYielding);
    const int teammatePriority = trafficPriority(
        teammateSlot, teammateYielding);
    if (teammatePriority < 0) return false;
    return teammatePriority > myPriority ||
           (teammatePriority == myPriority && teammateId < myId);
}

} // namespace assist_strategy_policy
