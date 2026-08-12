#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <set>
#include <utility>

#include "../include/ball_search_policy.h"

namespace policy = ball_search_policy;

namespace {

void require(bool condition, const char *message)
{
    if (condition) return;
    std::cerr << "FAILED: " << message << '\n';
    std::exit(EXIT_FAILURE);
}

policy::Field roboLeagueField()
{
    return {22.003, 14.126, 5.221, 8.121, 2.307, 5.083};
}

policy::Context context(
    policy::SetPlay setPlay,
    bool ours,
    int searcherIndex = 0,
    int searcherCount = 4)
{
    policy::Context result;
    result.field = roboLeagueField();
    result.setPlay = setPlay;
    result.ourSetPlay = ours;
    result.searcherIndex = searcherIndex;
    result.searcherCount = searcherCount;
    return result;
}

void checkNormalDistribution()
{
    std::set<std::pair<int, int>> firstPoints;
    double minX = 1e9;
    double maxX = -1e9;
    double minY = 1e9;
    double maxY = -1e9;
    for (int index = 0; index < 4; ++index) {
        const auto plan = policy::makePlan(
            context(policy::SetPlay::None, false, index, 4));
        require(plan.size() == 11, "normal search covers all grid points");
        firstPoints.insert({
            static_cast<int>(std::lround(plan.front().x * 100.0)),
            static_cast<int>(std::lround(plan.front().y * 100.0)),
        });
        for (const auto &waypoint : plan) {
            minX = std::min(minX, waypoint.x);
            maxX = std::max(maxX, waypoint.x);
            minY = std::min(minY, waypoint.y);
            maxY = std::max(maxY, waypoint.y);
        }
    }
    require(
        firstPoints.size() == 4,
        "four searchers start in four different normal-play regions");
    const auto field = roboLeagueField();
    require(
        minX <= -field.length / 2.0 + 1.0 &&
        maxX >= field.length / 2.0 - 1.0 &&
        minY <= -field.width / 2.0 + 1.0 &&
        maxY >= field.width / 2.0 - 1.0,
        "normal search covers the large field to within one metre of every edge");
}

void checkLastObservationPriority()
{
    auto primary = context(policy::SetPlay::None, false, 0, 4);
    primary.prioritizeLastObservation = true;
    primary.lastObservationValid = true;
    primary.lastObservation = {2.3, -1.7};
    const auto plan = policy::makePlan(primary);
    require(
        std::hypot(plan.front().x - 2.3, plan.front().y + 1.7) < 1e-6,
        "primary searcher checks the recent observation first");

    auto secondary = primary;
    secondary.searcherIndex = 1;
    secondary.prioritizeLastObservation = false;
    const auto secondaryPlan = policy::makePlan(secondary);
    require(
        std::hypot(
            secondaryPlan.front().x - 2.3,
            secondaryPlan.front().y + 1.7) > 0.75,
        "only one searcher prioritizes the recent observation");
}

void checkSetPlayEnds()
{
    for (int index = 0; index < 4; ++index) {
        const auto ourGoal = policy::makePlan(
            context(policy::SetPlay::GoalKick, true, index, 4));
        const auto opponentGoal = policy::makePlan(
            context(policy::SetPlay::GoalKick, false, index, 4));
        const auto ourCorner = policy::makePlan(
            context(policy::SetPlay::CornerKick, true, index, 4));
        const auto opponentCorner = policy::makePlan(
            context(policy::SetPlay::CornerKick, false, index, 4));
        require(ourGoal.front().x < 0.0, "our goal kick searches our end");
        require(
            opponentGoal.front().x > 0.0,
            "opponent goal kick searches the opponent end");
        require(
            ourCorner.front().x > 0.0,
            "our corner kick searches the opponent end");
        require(
            opponentCorner.front().x < 0.0,
            "opponent corner kick observes our end");

        const double opponentPenaltyBoundary =
            roboLeagueField().length / 2.0 -
            roboLeagueField().penaltyAreaLength;
        for (const auto &waypoint : opponentGoal) {
            require(
                waypoint.x < opponentPenaltyBoundary,
                "all opponent goal kick observations stay outside the penalty area");
        }
        for (const auto &waypoint : opponentCorner) {
            require(
                waypoint.x > -opponentPenaltyBoundary,
                "all opponent corner observations stay outside our penalty area");
        }
    }

    const auto left = policy::makePlan(
        context(policy::SetPlay::CornerKick, true, 0, 4));
    const auto right = policy::makePlan(
        context(policy::SetPlay::CornerKick, true, 1, 4));
    require(
        left.front().y * right.front().y < 0.0,
        "corner searchers initially split between both corners");
}

void checkAllWaypointsAreFiniteAndInside()
{
    const auto field = roboLeagueField();
    for (policy::SetPlay setPlay : {
             policy::SetPlay::None,
             policy::SetPlay::GoalKick,
             policy::SetPlay::CornerKick}) {
        for (bool ours : {false, true}) {
            const auto plan = policy::makePlan(context(setPlay, ours));
            require(!plan.empty(), "search plan is non-empty");
            for (const auto &waypoint : plan) {
                require(
                    std::isfinite(waypoint.x) &&
                    std::isfinite(waypoint.y) &&
                    std::isfinite(waypoint.theta),
                    "search waypoint is finite");
                require(
                    std::fabs(waypoint.x) < field.length / 2.0 &&
                    std::fabs(waypoint.y) < field.width / 2.0,
                    "search waypoint remains inside the field");
            }
        }
    }
}

} // namespace

int main()
{
    checkNormalDistribution();
    checkLastObservationPriority();
    checkSetPlayEnds();
    checkAllWaypointsAreFiniteAndInside();
    std::cout << "ball search policy tests passed\n";
    return 0;
}
