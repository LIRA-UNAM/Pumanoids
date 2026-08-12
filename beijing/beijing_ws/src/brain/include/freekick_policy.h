#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>

namespace freekick_policy {

enum class Type : uint8_t {
    None = 0,
    Direct = 1,
    Indirect = 2,
};

// Values are ordered so a newer phase can be adopted monotonically over UDP.
enum class Phase : uint8_t {
    Inactive = 0,
    Preparing = 1,
    Released = 2,
    FirstTouchKicking = 3,
    AwaitingSecondTouch = 4,
    SecondTouchKicking = 5,
    Complete = 6,
};

struct Point {
    double x = 0.0;
    double y = 0.0;
};

struct Field {
    double length = 9.0;
    double width = 6.0;
};

struct PlanAuthority {
    uint32_t leaderTerm = 0;
    int kickerId = 0;
    uint32_t eventId = 0;
    uint64_t proposerBootId = 0;
};

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

inline Point normalizedOr(const Point &point, const Point &fallback = {1.0, 0.0})
{
    const double length = norm(point);
    if (!std::isfinite(length) || length < 1e-6) return fallback;
    return point * (1.0 / length);
}

inline Point projectInsideField(Point point, const Field &field,
                                double boundaryMargin = 0.35)
{
    const double xLimit = std::max(0.0, field.length / 2.0 - boundaryMargin);
    const double yLimit = std::max(0.0, field.width / 2.0 - boundaryMargin);
    point.x = std::clamp(point.x, -xLimit, xLimit);
    point.y = std::clamp(point.y, -yLimit, yLimit);
    return point;
}

inline double pointSegmentDistance(const Point &point,
                                   const Point &start,
                                   const Point &end)
{
    const Point segment = end - start;
    const double lengthSquared =
        segment.x * segment.x + segment.y * segment.y;
    if (lengthSquared < 1e-9) return norm(point - start);
    const Point relative = point - start;
    const double t = std::clamp(
        (relative.x * segment.x + relative.y * segment.y) / lengthSquared,
        0.0,
        1.0);
    return norm(point - (start + segment * t));
}

inline Point constrainPassDistance(Point candidate, const Point &ball,
                                   const Field &field)
{
    const double minDistance = std::clamp(0.15 * field.length, 1.25, 2.4);
    const double maxDistance = std::clamp(0.36 * field.length, 3.0, 5.5);
    Point direction = candidate - ball;
    const double distance = norm(direction);
    direction = normalizedOr(direction);
    if (distance < minDistance) return ball + direction * minDistance;
    if (distance > maxDistance) return ball + direction * maxDistance;
    return candidate;
}

inline double nearestOpponentDistance(const Point &point,
                                      const std::vector<Point> &opponents)
{
    double result = 4.0;
    for (const Point &opponent : opponents) {
        if (!std::isfinite(opponent.x) || !std::isfinite(opponent.y)) continue;
        result = std::min(result, norm(point - opponent));
    }
    return result;
}

inline double passLaneClearance(const Point &ball, const Point &target,
                                const std::vector<Point> &opponents)
{
    double result = 4.0;
    for (const Point &opponent : opponents) {
        if (!std::isfinite(opponent.x) || !std::isfinite(opponent.y)) continue;
        result = std::min(
            result, pointSegmentDistance(opponent, ball, target));
    }
    return result;
}

inline double indirectPassScore(const Point &ball, const Point &target,
                                const Point &receiver,
                                const std::vector<Point> &opponents)
{
    const double targetClearance = nearestOpponentDistance(target, opponents);
    const double laneClearance = passLaneClearance(ball, target, opponents);
    const double receiverTravel = norm(target - receiver);
    const double forwardProgress = target.x - ball.x;
    return 1.8 * std::min(laneClearance, 2.5) +
           1.3 * std::min(targetClearance, 2.5) -
           0.75 * receiverTravel + 0.08 * forwardProgress;
}

inline Point chooseIndirectPassTarget(const Point &ball,
                                      const Point &receiver,
                                      const std::vector<Point> &opponents,
                                      const Field &field)
{
    const Point towardGoal = normalizedOr(
        {field.length / 2.0 - receiver.x, -receiver.y});
    const Point ballToReceiver = normalizedOr(receiver - ball, towardGoal);
    const Point lateral{-ballToReceiver.y, ballToReceiver.x};

    std::array<Point, 15> candidates{};
    std::size_t candidateCount = 0;
    for (double lead : {0.0, 0.45, 0.9}) {
        for (double side : {0.0, -0.7, 0.7}) {
            candidates[candidateCount++] =
                receiver + towardGoal * lead + lateral * side;
        }
    }
    for (double side : {-1.2, 1.2}) {
        candidates[candidateCount++] = receiver + lateral * side;
        candidates[candidateCount++] =
            ball + ballToReceiver * std::clamp(
                0.24 * field.length, 1.8, 4.0) + lateral * side;
    }
    candidates[candidateCount++] =
        ball + ballToReceiver * std::clamp(0.24 * field.length, 1.8, 4.0);
    candidates[candidateCount++] =
        receiver + towardGoal * 0.6 - lateral * 1.5;

    Point best = projectInsideField(
        constrainPassDistance(receiver, ball, field), field);
    double bestScore = -std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i < candidateCount; ++i) {
        Point candidate = constrainPassDistance(candidates[i], ball, field);
        candidate = projectInsideField(candidate, field);
        // Clamping at a touchline can shorten the pass, so enforce once more.
        candidate = projectInsideField(
            constrainPassDistance(candidate, ball, field), field);

        const double score = indirectPassScore(
            ball, candidate, receiver, opponents);
        if (score > bestScore + 1e-9) {
            best = candidate;
            bestScore = score;
        }
    }
    return best;
}

inline bool isExecutionActive(Phase phase)
{
    return phase >= Phase::Released && phase < Phase::Complete;
}

inline bool visualKickContextAllowed(bool executionActive,
                                     bool plannedActor,
                                     bool normalPlayEligible)
{
    return executionActive ? plannedActor : normalPlayEligible;
}

inline bool executionExpired(Phase phase, double elapsedMs,
                             double timeoutMs)
{
    return isExecutionActive(phase) &&
        std::isfinite(elapsedMs) && elapsedMs >= 0.0 &&
        std::isfinite(timeoutMs) && timeoutMs >= 0.0 &&
        elapsedMs > timeoutMs;
}

inline bool hasValidReceiver(Type type, int kickerId, int receiverId,
                             int playerCount)
{
    if (type != Type::Indirect) return receiverId == 0;
    return receiverId >= 1 && receiverId <= playerCount &&
           receiverId != kickerId;
}

inline bool authorityBetter(const PlanAuthority &candidate,
                            const PlanAuthority &current)
{
    if (candidate.leaderTerm != current.leaderTerm) {
        return candidate.leaderTerm > current.leaderTerm;
    }
    if (candidate.kickerId != current.kickerId) {
        if (candidate.kickerId <= 0) return false;
        if (current.kickerId <= 0) return true;
        return candidate.kickerId < current.kickerId;
    }
    // Event IDs are local to a process boot. Compare the boot identity first
    // so a restarted kicker cannot lose to an old plan merely because its
    // event counter restarted at a smaller value.
    if (candidate.proposerBootId != current.proposerBootId) {
        return candidate.proposerBootId > current.proposerBootId;
    }
    if (candidate.eventId != current.eventId) {
        return candidate.eventId > current.eventId;
    }
    return false;
}

inline uint8_t packetForwardDistance(uint8_t from, uint8_t to)
{
    return static_cast<uint8_t>(to - from);
}

// GameController packet numbers are uint8 values and wrap at 255. A packet is
// newer when it is ahead in the forward half of the sequence space; the
// half-range rule makes duplicate and reordered UDP packets unambiguous.
inline bool packetNewer(uint8_t candidate, uint8_t current)
{
    const uint8_t distance = packetForwardDistance(current, candidate);
    return distance != 0 && distance < 128;
}

inline bool packetInRestartWindow(uint8_t planPacket, uint8_t stopPacket,
                                  uint8_t currentPacket,
                                  uint8_t earlyTolerance = 3,
                                  uint8_t futureTolerance = 3)
{
    const uint8_t stopToCurrent = packetForwardDistance(
        stopPacket, currentPacket);
    if (stopToCurrent >= 128) return false;

    const uint8_t stopToPlan = packetForwardDistance(stopPacket, planPacket);
    if (stopToPlan <= stopToCurrent) return true;
    if (packetForwardDistance(planPacket, stopPacket) <= earlyTolerance) {
        return true;
    }
    return packetForwardDistance(currentPacket, planPacket) <= futureTolerance;
}

inline bool packetRecent(uint8_t planPacket, uint8_t currentPacket,
                         uint8_t maximumAge = 64,
                         uint8_t futureTolerance = 3)
{
    return packetForwardDistance(planPacket, currentPacket) <= maximumAge ||
           packetForwardDistance(currentPacket, planPacket) <= futureTolerance;
}

inline int actorForPhase(Type type, Phase phase, int kickerId, int receiverId)
{
    if (type == Type::Indirect && receiverId <= 0) return 0;
    if (phase == Phase::Released || phase == Phase::FirstTouchKicking) {
        return kickerId;
    }
    if (type == Type::Indirect &&
        (phase == Phase::AwaitingSecondTouch ||
         phase == Phase::SecondTouchKicking)) {
        return receiverId;
    }
    return 0;
}

inline Phase kickStarted(Type type, Phase phase)
{
    if (phase == Phase::Released) return Phase::FirstTouchKicking;
    if (type == Type::Indirect && phase == Phase::AwaitingSecondTouch) {
        return Phase::SecondTouchKicking;
    }
    return phase;
}

inline Phase kickCompleted(Type type, Phase phase)
{
    if (phase == Phase::FirstTouchKicking) {
        return type == Type::Indirect
            ? Phase::AwaitingSecondTouch
            : Phase::Complete;
    }
    if (type == Type::Indirect && phase == Phase::SecondTouchKicking) {
        return Phase::Complete;
    }
    return phase;
}

} // namespace freekick_policy
