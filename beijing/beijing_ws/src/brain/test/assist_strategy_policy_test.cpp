#include <array>
#include <cmath>
#include <cstdlib>
#include <iostream>

#include "../include/assist_strategy_policy.h"

namespace policy = assist_strategy_policy;

namespace {

void require(bool condition, const char *message)
{
    if (condition) return;
    std::cerr << "FAILED: " << message << '\n';
    std::exit(EXIT_FAILURE);
}

std::array<policy::Slot, 4> activeSlots(std::size_t assistCount)
{
    std::array<policy::Slot, 4> slots{
        policy::COVER_MID,
        policy::NONE,
        policy::NONE,
        policy::NONE,
    };
    if (assistCount >= 2) slots[1] = policy::SHADOW_SUPPORT;
    if (assistCount >= 3) slots[2] = policy::ANCHOR_COVER;
    if (assistCount >= 4) slots[3] = policy::WIDE_OUTLET;
    return slots;
}

void checkSampledGeometry(const policy::Field &field)
{
    const double minSpacing = policy::minimumSlotSpacing(field);
    for (int xSample = 0; xSample <= 12; ++xSample) {
        const double ballX = -field.length / 2.0 +
            field.length * xSample / 12.0;
        for (int ySample = 0; ySample <= 8; ++ySample) {
            const double ballY = -field.width / 2.0 +
                field.width * ySample / 8.0;
            for (int side : {-1, 1}) {
                for (std::size_t assistCount = 1;
                     assistCount <= 4;
                     ++assistCount) {
                    const auto targets = policy::calculateTargets(
                        assistCount, {ballX, ballY}, side, field);
                    const auto slots = activeSlots(assistCount);
                    for (std::size_t i = 0; i < assistCount; ++i) {
                        const auto &target = targets[slots[i]];
                        require(std::isfinite(target.x), "target x is finite");
                        require(std::isfinite(target.y), "target y is finite");
                        for (std::size_t j = i + 1;
                             j < assistCount;
                             ++j) {
                            require(policy::norm(
                                target - targets[slots[j]]) + 1e-6 >=
                                minSpacing,
                                "sampled slots preserve minimum spacing");
                        }
                    }
                }
            }
        }
    }
}

void checkThreeAssistGeometry(const policy::Field &field)
{
    const double ownGoalX = -field.length / 2.0;
    const double dangerBoundary = policy::defensiveDangerBoundary(field);
    const std::array<policy::Point, 7> balls{
        policy::Point{0.0, 0.0},
        policy::Point{-0.2 * field.length, 0.0},
        policy::Point{0.15 * field.length, 0.2 * field.width},
        policy::Point{0.35 * field.length, -0.25 * field.width},
        policy::Point{dangerBoundary - 0.2, 0.0},
        policy::Point{ownGoalX + field.penaltyAreaLength + 0.4,
                      0.18 * field.width},
        policy::Point{0.0, -0.4 * field.width},
    };

    const double minSpacing = policy::minimumSlotSpacing(field);
    for (const auto &ball : balls) {
        for (int side : {-1, 1}) {
            const auto targets = policy::calculateTargets(
                3, ball, side, field);
            const std::array<policy::Slot, 3> slots{
                policy::COVER_MID,
                policy::SHADOW_SUPPORT,
                policy::ANCHOR_COVER,
            };
            for (std::size_t i = 0; i < slots.size(); ++i) {
                const auto &target = targets[slots[i]];
                require(std::isfinite(target.x), "target x is finite");
                require(std::isfinite(target.y), "target y is finite");
                for (std::size_t j = i + 1; j < slots.size(); ++j) {
                    const double distance = policy::norm(
                        target - targets[slots[j]]);
                    require(
                        distance + 1e-6 >= minSpacing,
                        "three-assist slots preserve minimum spacing");
                }
            }

            if (ball.x >= dangerBoundary) {
                const double legalBallDepth = std::min(
                    ball.x,
                    field.length / 2.0 - field.penaltyAreaLength - 0.3);
                if (targets[policy::SHADOW_SUPPORT].x + 1e-6 < legalBallDepth) {
                    std::cerr << "shadow behind ball: field=" << field.length
                              << " ball=(" << ball.x << ", " << ball.y
                              << ") side=" << side << " target=("
                              << targets[policy::SHADOW_SUPPORT].x << ", "
                              << targets[policy::SHADOW_SUPPORT].y << ")\n";
                }
                require(
                    targets[policy::SHADOW_SUPPORT].x + 1e-6 >= legalBallDepth,
                    "shadow stays level with or ahead of a non-danger ball");
            }
        }
    }

    const policy::Point midfieldBall{0.0, 0.0};
    const auto rawAnchor = policy::rawSlotTarget(
        policy::ANCHOR_COVER, midfieldBall, 1, field);
    const double anchorDepth = policy::norm(
        rawAnchor - policy::Point{ownGoalX, 0.0});
    const double expectedDepth = std::max(
        field.goalAreaLength + 0.3,
        3.0 * field.length / 22.0);
    require(
        std::fabs(anchorDepth - expectedDepth) < 1e-6,
        "anchor keeps its field-scaled depth");
}

void checkShadowSidePolicy()
{
    require(
        policy::selectShadowSide(0, -1, false, false, false, false) == -1,
        "invalid side adopts preferred side");
    require(
        policy::selectShadowSide(1, -1, true, true, true, true) == 1,
        "new owner inherits the current side");
    require(
        policy::selectShadowSide(1, -1, false, false, true, true) == 1,
        "side hold prevents early switching");
    require(
        policy::selectShadowSide(1, -1, false, true, false, false) == 1,
        "side stays stable without meaningful benefit");
    require(
        policy::selectShadowSide(1, -1, false, true, false, true) == -1,
        "side switches after hold for meaningful benefit");
    require(
        policy::selectShadowSide(1, -1, false, true, true, false) == -1,
        "blocked kick lane switches after hold");

    constexpr double pi = 3.14159265358979323846;
    const policy::Field field{22.003, 14.126, 5.221, 2.307};
    const policy::Point ball{0.0, 0.0};
    require(
        policy::preferredShadowSide(ball, pi / 2.0, 1, field) == -1,
        "leftward cross selects the opposite shadow side");
    require(
        policy::preferredShadowSide(ball, -pi / 2.0, 1, field) == 1,
        "rightward cross selects the opposite shadow side");
}

void checkYieldTimeoutPolicy()
{
    const double routeDistance = std::hypot(1.0, 1.8);
    const double timeout = policy::yieldTimeoutMsecs(
        routeDistance, 0.5, 4800.0);
    require(
        timeout >= routeDistance / 0.5 * 1000.0 + 1000.0,
        "yield timeout covers route travel plus reserve");
    require(timeout > 5000.0, "long yield route extends configured timeout");
    require(
        policy::yieldTimeoutMsecs(0.2, 0.5, 4800.0) == 4800.0,
        "configured timeout remains the short-route floor");
}

void checkTrafficPriorityPolicy()
{
    require(
        policy::trafficPriority(policy::ANCHOR_COVER, false) >
            policy::trafficPriority(policy::COVER_MID, false),
        "anchor has priority over cover");
    require(
        policy::trafficPriority(policy::COVER_MID, false) >
            policy::trafficPriority(policy::SHADOW_SUPPORT, false),
        "cover has priority over shadow");
    require(
        policy::trafficPriority(policy::SHADOW_SUPPORT, false) >
            policy::trafficPriority(policy::WIDE_OUTLET, false),
        "shadow has priority over wide outlet");
    require(
        policy::trafficPriority(policy::ANCHOR_COVER, true) == 0,
        "yield phase has the lowest traffic priority");
    require(
        policy::teammateHasRightOfWay(
            policy::SHADOW_SUPPORT, false, 2,
            policy::ANCHOR_COVER, false, 5),
        "lower-priority slot yields regardless of player id");
    require(
        !policy::teammateHasRightOfWay(
            policy::ANCHOR_COVER, false, 5,
            policy::SHADOW_SUPPORT, false, 2),
        "higher-priority slot keeps right of way");
    require(
        policy::teammateHasRightOfWay(
            policy::COVER_MID, false, 4,
            policy::COVER_MID, false, 2),
        "lower player id breaks an equal-priority tie");
    require(
        !policy::teammateHasRightOfWay(
            policy::COVER_MID, false, 2,
            policy::COVER_MID, false, 4),
        "equal-priority tie break is asymmetric");
    require(
        policy::teammateHasRightOfWay(
            policy::ANCHOR_COVER, true, 1,
            policy::WIDE_OUTLET, false, 5),
        "yielding former leader yields to every active slot");
}

} // namespace

int main()
{
    checkSampledGeometry({9.0, 6.0, 2.0, 1.0});
    checkSampledGeometry({14.16, 9.22, 3.24, 1.345});
    checkSampledGeometry({22.003, 14.126, 5.221, 2.307});
    checkThreeAssistGeometry({9.0, 6.0, 2.0, 1.0});
    checkThreeAssistGeometry({14.16, 9.22, 3.24, 1.345});
    checkThreeAssistGeometry({22.003, 14.126, 5.221, 2.307});
    checkShadowSidePolicy();
    checkYieldTimeoutPolicy();
    checkTrafficPriorityPolicy();
    std::cout << "assist strategy policy tests passed\n";
    return 0;
}
