#include <algorithm>
#include <array>
#include <cmath>
#include <chrono>
#include <cstdlib>
#include <limits>
#include <memory> 
#include <mutex>
#include <optional>
#include <utility>
#include "assist_strategy_policy.h"
#include "ball_search_policy.h"
#include "head_scan_policy.h"
#include "brain_tree.h"
#include "brain.h"
#include "utils/math.h"
#include "utils/print.h"
#include "utils/misc.h"
#include "std_msgs/msg/string.hpp"
#include <fstream>
#include <ios>

using namespace std;
using namespace BT;

namespace {

using AssistClock = std::chrono::steady_clock;

double assistElapsedMsecs(const AssistClock::time_point &timePoint)
{
    if (timePoint == AssistClock::time_point{}) {
        return std::numeric_limits<double>::infinity();
    }
    return std::chrono::duration<double, std::milli>(
        AssistClock::now() - timePoint).count();
}

struct AssistPoint {
    double x = 0.0;
    double y = 0.0;
};

AssistPoint operator+(const AssistPoint &lhs, const AssistPoint &rhs)
{
    return {lhs.x + rhs.x, lhs.y + rhs.y};
}

AssistPoint operator-(const AssistPoint &lhs, const AssistPoint &rhs)
{
    return {lhs.x - rhs.x, lhs.y - rhs.y};
}

AssistPoint operator*(const AssistPoint &point, double scale)
{
    return {point.x * scale, point.y * scale};
}

double assistPointNorm(const AssistPoint &point)
{
    return std::hypot(point.x, point.y);
}

AssistPoint assistNormalizedOr(const AssistPoint &point,
                               const AssistPoint &fallback = {1.0, 0.0})
{
    const double length = assistPointNorm(point);
    if (length < 1e-6 || !std::isfinite(length)) return fallback;
    return point * (1.0 / length);
}

double assistCross(const AssistPoint &a, const AssistPoint &b)
{
    return a.x * b.y - a.y * b.x;
}

AssistPoint closestPointOnSegment(const AssistPoint &point,
                                  const AssistPoint &start,
                                  const AssistPoint &end)
{
    const AssistPoint segment = end - start;
    const double lengthSquared = segment.x * segment.x + segment.y * segment.y;
    if (lengthSquared < 1e-9) return start;
    const AssistPoint relative = point - start;
    const double t = std::clamp(
        (relative.x * segment.x + relative.y * segment.y) / lengthSquared,
        0.0,
        1.0);
    return start + segment * t;
}

double assistPointSegmentDistance(const AssistPoint &point,
                                  const AssistPoint &start,
                                  const AssistPoint &end)
{
    return assistPointNorm(point - closestPointOnSegment(point, start, end));
}

int assistOrientation(const AssistPoint &a, const AssistPoint &b,
                      const AssistPoint &c)
{
    const double value = assistCross(b - a, c - a);
    if (std::fabs(value) < 1e-8) return 0;
    return value > 0.0 ? 1 : -1;
}

bool assistSegmentsIntersect(const AssistPoint &a0, const AssistPoint &a1,
                             const AssistPoint &b0, const AssistPoint &b1)
{
    const int o1 = assistOrientation(a0, a1, b0);
    const int o2 = assistOrientation(a0, a1, b1);
    const int o3 = assistOrientation(b0, b1, a0);
    const int o4 = assistOrientation(b0, b1, a1);
    if (o1 != o2 && o3 != o4) return true;
    if (o1 == 0 && assistPointSegmentDistance(b0, a0, a1) < 1e-6) return true;
    if (o2 == 0 && assistPointSegmentDistance(b1, a0, a1) < 1e-6) return true;
    if (o3 == 0 && assistPointSegmentDistance(a0, b0, b1) < 1e-6) return true;
    if (o4 == 0 && assistPointSegmentDistance(a1, b0, b1) < 1e-6) return true;
    return false;
}

double assistSegmentDistance(const AssistPoint &a0, const AssistPoint &a1,
                             const AssistPoint &b0, const AssistPoint &b1)
{
    if (assistSegmentsIntersect(a0, a1, b0, b1)) return 0.0;
    return std::min({
        assistPointSegmentDistance(a0, b0, b1),
        assistPointSegmentDistance(a1, b0, b1),
        assistPointSegmentDistance(b0, a0, a1),
        assistPointSegmentDistance(b1, a0, a1),
    });
}

Pose2D clampAssistWaypoint(Pose2D waypoint, const FieldDimensions &field)
{
    waypoint.x = std::clamp(
        waypoint.x,
        -field.length / 2.0 + 0.8,
        field.length / 2.0 - 0.8);
    waypoint.y = std::clamp(
        waypoint.y,
        -field.width / 2.0 + 0.8,
        field.width / 2.0 - 0.8);
    return waypoint;
}

struct ProtectedLane {
    AssistPoint start;
    AssistPoint end;
    double radius = 0.0;
};

bool segmentSafelyClearsLane(const AssistPoint &start,
                             const AssistPoint &end,
                             const ProtectedLane &lane)
{
    const double startDistance = assistPointSegmentDistance(
        start, lane.start, lane.end);
    const double endDistance = assistPointSegmentDistance(
        end, lane.start, lane.end);
    if (endDistance < lane.radius) return false;

    if (startDistance < lane.radius) {
        if (endDistance <= startDistance + 0.05) return false;
        const AssistPoint closest = closestPointOnSegment(
            start, lane.start, lane.end);
        const AssistPoint outward = start - closest;
        const AssistPoint movement = end - start;
        if (assistPointNorm(outward) > 1e-4) {
            if (outward.x * movement.x + outward.y * movement.y <= 0.0) {
                return false;
            }
        } else {
            const AssistPoint laneVector = lane.end - lane.start;
            const double laneLength = assistPointNorm(laneVector);
            const AssistPoint axis = assistNormalizedOr(laneVector);
            const AssistPoint moveUnit = assistNormalizedOr(movement);
            const double projection = laneLength > 1e-6
                ? std::clamp(
                    ((start.x - lane.start.x) * axis.x +
                     (start.y - lane.start.y) * axis.y) / laneLength,
                    0.0,
                    1.0)
                : 0.5;
            const double axialMotion =
                moveUnit.x * axis.x + moveUnit.y * axis.y;
            const double lateralMotion = std::fabs(
                assistCross(axis, moveUnit));
            const bool exitsPastStart =
                projection < 0.05 && axialMotion < -0.25;
            const bool exitsPastEnd =
                projection > 0.95 && axialMotion > 0.25;
            if (!exitsPastStart && !exitsPastEnd && lateralMotion < 0.5) {
                return false;
            }
        }
        return true;
    }

    return assistSegmentDistance(
        start, end, lane.start, lane.end) >= lane.radius;
}

bool pathSafelyClearsLanes(const AssistPoint &start,
                           const AssistPoint &end,
                           const vector<ProtectedLane> &lanes)
{
    return std::all_of(
        lanes.begin(), lanes.end(),
        [&](const ProtectedLane &lane) {
            return segmentSafelyClearsLane(start, end, lane);
        });
}

bool pointSafelyClearsLanes(const AssistPoint &point,
                            const vector<ProtectedLane> &lanes,
                            double margin = 0.0)
{
    return std::all_of(
        lanes.begin(), lanes.end(),
        [&](const ProtectedLane &lane) {
            return assistPointSegmentDistance(
                point, lane.start, lane.end) >= lane.radius + margin;
        });
}

std::optional<Pose2D> findNearestLegalAssistTarget(
    const Pose2D &desiredTarget,
    const vector<ProtectedLane> &lanes,
    const AssistPoint &kickAxis,
    const AssistPoint &ballPoint,
    int preferredSide,
    const FieldDimensions &field)
{
    constexpr double targetMargin = 0.12;
    const AssistPoint desiredPoint{desiredTarget.x, desiredTarget.y};
    if (pointSafelyClearsLanes(desiredPoint, lanes, targetMargin)) {
        return desiredTarget;
    }

    const AssistPoint referenceAxis = assistNormalizedOr(kickAxis);
    std::optional<Pose2D> bestTarget;
    double bestScore = std::numeric_limits<double>::infinity();
    auto considerCandidate = [&](const AssistPoint &rawPoint) {
        Pose2D candidate = clampAssistWaypoint(
            {rawPoint.x, rawPoint.y, desiredTarget.theta}, field);
        const AssistPoint candidatePoint{candidate.x, candidate.y};
        if (!pointSafelyClearsLanes(
                candidatePoint, lanes, targetMargin)) {
            return;
        }

        const double sideValue = assistCross(
            referenceAxis, candidatePoint - ballPoint);
        const int side = std::fabs(sideValue) < 1e-6
            ? 0
            : (sideValue > 0.0 ? 1 : -1);
        const double score = assistPointNorm(candidatePoint - desiredPoint) +
            (preferredSide != 0 && side != 0 && side != preferredSide
                ? 0.03
                : 0.0);
        if (score < bestScore) {
            bestScore = score;
            bestTarget = candidate;
        }
    };

    // Analytic projections usually find the closest legal boundary point.
    for (const ProtectedLane &lane : lanes) {
        const AssistPoint axis = assistNormalizedOr(
            lane.end - lane.start, referenceAxis);
        const AssistPoint normal{-axis.y, axis.x};
        const AssistPoint closest = closestPointOnSegment(
            desiredPoint, lane.start, lane.end);
        const AssistPoint radial = desiredPoint - closest;
        const double clearance = lane.radius + targetMargin;
        if (assistPointNorm(radial) > 1e-6) {
            considerCandidate(
                closest + assistNormalizedOr(radial) * clearance);
        }
        considerCandidate(closest + normal * clearance);
        considerCandidate(closest - normal * clearance);
        considerCandidate(lane.start - axis * clearance);
        considerCandidate(lane.end + axis * clearance);
    }

    // Overlapping capsules and field clipping can invalidate every direct
    // projection. A bounded radial search supplies nearby combined escapes.
    constexpr int angleSamples = 32;
    constexpr double radialStep = 0.2;
    constexpr double maxSearchRadius = 4.0;
    for (double radius = radialStep;
         radius <= maxSearchRadius + 1e-6;
         radius += radialStep) {
        for (int i = 0; i < angleSamples; i++) {
            const double angle =
                2.0 * M_PI * static_cast<double>(i) / angleSamples;
            considerCandidate({
                desiredPoint.x + std::cos(angle) * radius,
                desiredPoint.y + std::sin(angle) * radius,
            });
        }
    }
    return bestTarget;
}

struct LaneWaypointChoice {
    Pose2D waypoint;
    int kickSide = 0;
};

std::optional<LaneWaypointChoice> chooseLaneWaypoint(
    const Pose2D &robot,
    const Pose2D &target,
    const vector<ProtectedLane> &lanes,
    const AssistPoint &kickAxis,
    const AssistPoint &ballPoint,
    double margin,
    int preferredSide,
    int failureCount,
    const FieldDimensions &field)
{
    const AssistPoint robotPoint{robot.x, robot.y};
    const AssistPoint targetPoint{target.x, target.y};
    const AssistPoint referenceAxis = assistNormalizedOr(kickAxis);

    std::optional<LaneWaypointChoice> bestChoice;
    std::optional<LaneWaypointChoice> bestPreferredChoice;
    double bestScore = std::numeric_limits<double>::infinity();
    double bestPreferredScore = std::numeric_limits<double>::infinity();
    const int boundedFailureCount = std::clamp(failureCount, 0, 6);
    const double failureExpansion = 0.35 * boundedFailureCount;
    const std::array<double, 4> extraClearances{0.0, 0.6, 1.2, 1.8};
    for (const ProtectedLane &sourceLane : lanes) {
        if (segmentSafelyClearsLane(
                robotPoint, targetPoint, sourceLane)) {
            continue;
        }
        const AssistPoint axis = assistNormalizedOr(
            sourceLane.end - sourceLane.start, referenceAxis);
        const AssistPoint normal{-axis.y, axis.x};
        for (double extraClearance : extraClearances) {
            const double clearance = sourceLane.radius + margin +
                failureExpansion + extraClearance;
            const AssistPoint midpoint =
                (robotPoint + targetPoint) * 0.5;
            const std::array<AssistPoint, 5> waypointBases{
                closestPointOnSegment(
                    robotPoint, sourceLane.start, sourceLane.end),
                closestPointOnSegment(
                    targetPoint, sourceLane.start, sourceLane.end),
                closestPointOnSegment(
                    midpoint, sourceLane.start, sourceLane.end),
                sourceLane.start - axis * clearance,
                sourceLane.end + axis * clearance,
            };
            for (int side : {-1, 1}) {
                for (const AssistPoint &base : waypointBases) {
                    const AssistPoint point =
                        base + normal * (side * clearance);
                    Pose2D waypoint = clampAssistWaypoint(
                        {point.x, point.y, 0.0}, field);
                    const AssistPoint waypointPoint{waypoint.x, waypoint.y};
                    if (!pointSafelyClearsLanes(
                            waypointPoint, lanes, 0.02) ||
                        !pathSafelyClearsLanes(
                            robotPoint, waypointPoint, lanes) ||
                        !pathSafelyClearsLanes(
                            waypointPoint, targetPoint, lanes)) {
                        continue;
                    }

                    const double kickSideValue = assistCross(
                        referenceAxis, waypointPoint - ballPoint);
                    const int kickSide = std::fabs(kickSideValue) < 1e-6
                        ? side
                        : (kickSideValue > 0.0 ? 1 : -1);
                    const double boundaryClearance = std::min(
                        field.width / 2.0 - std::fabs(waypoint.y),
                        field.length / 2.0 - std::fabs(waypoint.x));
                    const double sidePenalty =
                        preferredSide != 0 && kickSide != preferredSide
                        ? 5.0
                        : 0.0;
                    const double score =
                        std::hypot(robot.x - waypoint.x, robot.y - waypoint.y) +
                        std::hypot(target.x - waypoint.x, target.y - waypoint.y) +
                        (boundaryClearance < 1.0 ? 10.0 : 0.0) +
                        sidePenalty +
                        0.1 * extraClearance;
                    waypoint.theta = std::atan2(
                        targetPoint.y - waypoint.y,
                        targetPoint.x - waypoint.x);
                    const LaneWaypointChoice choice{waypoint, kickSide};
                    if (score < bestScore) {
                        bestScore = score;
                        bestChoice = choice;
                    }
                    if (preferredSide != 0 && kickSide == preferredSide &&
                        score < bestPreferredScore) {
                        bestPreferredScore = score;
                        bestPreferredChoice = choice;
                    }
                }
            }
        }
    }
    if (failureCount > 0 && bestPreferredChoice.has_value()) {
        return bestPreferredChoice;
    }
    return bestChoice;
}

struct BallSearchRuntime {
    ball_search_policy::SetPlay setPlay = ball_search_policy::SetPlay::None;
    bool ourSetPlay = false;
    bool anyFreshTeammateSeesBall = false;
    int searcherIndex = 0;
    int searcherCount = 1;
};

BallSearchRuntime getBallSearchRuntime(Brain *brain)
{
    BallSearchRuntime runtime;
    if (brain->data->realGameSubState == "GOAL_KICK") {
        runtime.setPlay = ball_search_policy::SetPlay::GoalKick;
    } else if (brain->data->realGameSubState == "CORNER_KICK") {
        runtime.setPlay = ball_search_policy::SetPlay::CornerKick;
    }
    runtime.ourSetPlay =
        brain->tree->getEntry<bool>("gc_is_sub_state_kickoff_side");

    std::array<TMStatus, MAX_NUM_PLAYERS> teamStatuses{};
    int assignedGoalkeeperId = 0;
    {
        // Match handleCooperation's lock order so the role snapshot is coherent.
        std::lock_guard<std::mutex> cooperationLock(
            brain->data->cooperationMutex);
        assignedGoalkeeperId = brain->data->tmAssignedGoalkeeperId;
        std::lock_guard<std::mutex> teamStatusLock(
            brain->data->teamStatusMutex);
        std::copy(
            std::begin(brain->data->tmStatus),
            std::end(brain->data->tmStatus),
            teamStatuses.begin());
    }

    const double packetTimeoutMs = std::max(
        100.0,
        brain->get_parameter(
            "strategy.cooperation.tactical_packet_timeout_ms").as_double());
    const int playerCount = std::clamp(
        brain->config->numOfPlayers, 1, MAX_NUM_PLAYERS);
    const int selfId = brain->config->playerId;
    std::vector<int> searcherIds;
    for (int index = 0; index < playerCount; ++index) {
        const int playerId = index + 1;
        if (playerId == selfId) {
            if (brain->data->penalty[index] == PENALTY_NONE &&
                playerId != assignedGoalkeeperId &&
                brain->tree->getEntry<string>("player_role") == "striker") {
                searcherIds.push_back(playerId);
            }
            continue;
        }

        const TMStatus &status = teamStatuses[index];
        const bool freshAndAlive =
            brain->data->penalty[index] == PENALTY_NONE &&
            status.isAlive &&
            brain->msecsSince(status.timeLastCom) <= packetTimeoutMs;
        if (!freshAndAlive) continue;
        if (status.ballDetected) runtime.anyFreshTeammateSeesBall = true;
        if (playerId != assignedGoalkeeperId && status.role == "striker") {
            searcherIds.push_back(playerId);
        }
    }

    runtime.searcherCount = std::max(1, static_cast<int>(searcherIds.size()));
    const auto self = std::find(
        searcherIds.begin(), searcherIds.end(), selfId);
    runtime.searcherIndex = self == searcherIds.end()
        ? 0
        : static_cast<int>(std::distance(searcherIds.begin(), self));
    return runtime;
}

std::vector<Pose2D> makeRobotBallSearchPlan(
    Brain *brain,
    const BallSearchRuntime &runtime)
{
    const FieldDimensions &field = brain->config->fieldDimensions;
    ball_search_policy::Context context;
    context.field = {
        field.length,
        field.width,
        field.penaltyAreaLength,
        field.penaltyAreaWidth,
        field.goalAreaLength,
        field.goalAreaWidth,
    };
    context.setPlay = runtime.setPlay;
    context.ourSetPlay = runtime.ourSetPlay;
    context.searcherIndex = runtime.searcherIndex;
    context.searcherCount = runtime.searcherCount;

    const double maxObservationAgeSeconds = std::max(
        0.0,
        brain->get_parameter(
            "strategy.search.last_observation_max_age_secs").as_double());
    context.prioritizeLastObservation = runtime.searcherIndex == 0;
    context.lastObservationValid =
        brain->data->ballEverDetected.load() &&
        std::isfinite(brain->data->ball.posToField.x) &&
        std::isfinite(brain->data->ball.posToField.y) &&
        brain->msecsSince(brain->data->ball.timePoint) <=
            maxObservationAgeSeconds * 1000.0;
    context.lastObservation = {
        brain->data->ball.posToField.x,
        brain->data->ball.posToField.y,
    };

    std::vector<Pose2D> result;
    const auto policyPlan = ball_search_policy::makePlan(context);
    result.reserve(policyPlan.size());
    for (const auto &waypoint : policyPlan) {
        result.push_back({waypoint.x, waypoint.y, waypoint.theta});
    }
    return result;
}

bool ballSearchMotionAllowed(Brain *brain)
{
    const string gameState =
        brain->tree->getEntry<string>("gc_game_state");
    if (!gameState.empty() && gameState != "PLAY") return false;
    const bool waitingForOpponent =
        brain->tree->getEntry<bool>("wait_for_opponent_kickoff");
    const bool safeOpponentSetPlaySearch =
        waitingForOpponent &&
        !brain->tree->getEntry<bool>("gc_is_sub_state_kickoff_side") &&
        (brain->data->realGameSubState == "GOAL_KICK" ||
         brain->data->realGameSubState == "CORNER_KICK");
    if (brain->tree->getEntry<string>("gc_game_sub_state") == "STOP" ||
        (waitingForOpponent && !safeOpponentSetPlaySearch) ||
        brain->tree->getEntry<bool>("gc_is_under_penalty") ||
        brain->tree->getEntry<string>("player_role") != "striker") {
        return false;
    }

    const int selfIndex = brain->config->playerId - 1;
    if (selfIndex < 0 || selfIndex >= MAX_NUM_PLAYERS ||
        brain->data->penalty[selfIndex] != PENALTY_NONE) {
        return false;
    }
    if (!gameState.empty() &&
        !brain->tree->getEntry<bool>("odom_calibrated")) {
        return false;
    }
    return !brain->isRecoveryActive() &&
        !brain->isRecoveryLocalizationBlocked() &&
        brain->isRecoveryOperationalMode();
}

} // namespace

/**
 * This macro reduces repetitive RegisterBuilder calls.
 * REGISTER_BUILDER(Test) expands to:
 * factory.registerBuilder<Test>(  \
 * "Test",                    \
 * [this](const string& name, const NodeConfig& config) { return std::make_unique<Test>(name, config, brain); });
 */
#define REGISTER_BUILDER(Name)     \
    factory.registerBuilder<Name>( \
        #Name,                     \
        [this](const string &name, const NodeConfig &config) { return std::make_unique<Name>(name, config, brain); });

void BrainTree::init()
{
    BehaviorTreeFactory factory;

    // Action Nodes

    REGISTER_BUILDER(RobotFindBall)
    REGISTER_BUILDER(Chase)
    REGISTER_BUILDER(SimpleChase)
    REGISTER_BUILDER(Adjust)
    REGISTER_BUILDER(Kick)
    REGISTER_BUILDER(StandStill)
    REGISTER_BUILDER(CalcKickDir)
    REGISTER_BUILDER(StrikerDecide)
    REGISTER_BUILDER(CamTrackBall)
    REGISTER_BUILDER(CamFindBall)
    REGISTER_BUILDER(CamFastScan)
    REGISTER_BUILDER(CamScanField)
    
    // These classes have tick implementations in the header, so registration is safe.
    REGISTER_BUILDER(SelfLocate)
    REGISTER_BUILDER(SelfLocateLine)
    REGISTER_BUILDER(SelfLocateEnterField)
    REGISTER_BUILDER(SelfLocateLocal)
    REGISTER_BUILDER(SelfLocate1P)
    REGISTER_BUILDER(SelfLocate1M)
    REGISTER_BUILDER(SelfLocate2X)
    REGISTER_BUILDER(SelfLocate2T)
    REGISTER_BUILDER(SelfLocateLT)
    REGISTER_BUILDER(SelfLocatePT)
    REGISTER_BUILDER(SelfLocateBorder)

    REGISTER_BUILDER(SetVelocity)
    REGISTER_BUILDER(RobocupWalk)
    REGISTER_BUILDER(StepOnSpot)
    REGISTER_BUILDER(GoToFreekickPosition)
    REGISTER_BUILDER(GoToReadyPosition)
    REGISTER_BUILDER(GoToGoalBlockingPosition)
    REGISTER_BUILDER(GoalkeeperBlockShot)
    REGISTER_BUILDER(TurnOnSpot)
    REGISTER_BUILDER(MoveToPoseOnField)
    REGISTER_BUILDER(GoBackInField)
    REGISTER_BUILDER(GoalieDecide)
    REGISTER_BUILDER(DecideCheckBehind)
    REGISTER_BUILDER(WaveHand)
    REGISTER_BUILDER(MoveHead)
    REGISTER_BUILDER(CheckAndStandUp)
    REGISTER_BUILDER(RLVisionKick)
    REGISTER_BUILDER(Intercept)

    REGISTER_BUILDER(RoleSwitchIfNeeded)

    REGISTER_BUILDER(Assist)

    // Action Nodes for debug
    REGISTER_BUILDER(CrabWalk)
    REGISTER_BUILDER(AutoCalibrateVision)
    REGISTER_BUILDER(CalibrateOdom)
    REGISTER_BUILDER(PrintMsg)
    REGISTER_BUILDER(PlaySound)
    REGISTER_BUILDER(Speak)

    factory.registerBehaviorTreeFromFile(brain->config->treeFilePath);
    tree = factory.createTree("MainTree");

    // Initialize blackboard entries after constructing the tree.
    initEntry();
}

void BrainTree::initEntry()
{
    setEntry<string>("player_role", brain->config->playerRole);
    setEntry<bool>("ball_location_known", false);
    setEntry<bool>("tm_ball_pos_reliable", false);
    setEntry<bool>("ball_out", false);
    setEntry<bool>("track_ball", true);
    setEntry<bool>("odom_calibrated", false);
    setEntry<string>("decision", "");
    setEntry<string>("defend_decision", "chase");
    setEntry<double>("ball_range", 0);

    // True once the opponent moves the ball or the kickoff time limit expires.
    setEntry<bool>("gamecontroller_isKickOff", true);
    setEntry<string>("gc_game_state", "");
    setEntry<string>("gc_game_sub_state_type", "NONE");
    setEntry<string>("gc_game_sub_state", "");
    setEntry<bool>("gc_is_kickoff_side", false);
    setEntry<bool>("gc_is_sub_state_kickoff_side", false);
    setEntry<bool>("freekick_plan_unavailable", false);
    setEntry<bool>("gc_is_under_penalty", false);

    setEntry<bool>("need_check_behind", false);

    // Multi-robot communication
    setEntry<bool>("is_lead", true); // True when controlling the ball; false when supporting a teammate.
    setEntry<string>("goalie_mode", "attack"); // guard, attack
    setEntry<string>("goalie_kick_type", "default");

    setEntry<int>("test_choice", 0);
    setEntry<int>("control_state", 0);
    setEntry<bool>("assist_chase", false);
    setEntry<bool>("assist_kick", false);
    setEntry<bool>("go_manual", false);

    setEntry<bool>("we_just_scored", false);
    setEntry<bool>("wait_for_opponent_kickoff", false);

    // Automatic vision calibration
    setEntry<string>("calibrate_state", "pitch");
    setEntry<double>("calibrate_pitch_center", 0.0);
    setEntry<double>("calibrate_pitch_step", 1.0);
    setEntry<double>("calibrate_yaw_center", 0.0);
    setEntry<double>("calibrate_yaw_step", 1.0);
    setEntry<double>("calibrate_z_center", 0.0);
    setEntry<double>("calibrate_z_step", 0.01);
}

void BrainTree::tick()
{
    tree.tickOnce();
}

NodeStatus SetVelocity::tick()
{
    double x, y, theta;
    vector<double> targetVec;
    getInput("x", x);
    getInput("y", y);
    getInput("theta", theta);

    auto res = brain->client->setVelocity(x, y, theta);
    return NodeStatus::SUCCESS;
}

NodeStatus RobocupWalk::tick()
{
    // Intended to be wrapped by a RunOnce decorator at behavior-tree startup.
    brain->client->changeRobocupMode();
    return NodeStatus::SUCCESS;
}

NodeStatus StepOnSpot::tick()
{
    std::srand(std::time(0));
    double vx = (std::rand() / (RAND_MAX / 0.02)) - 0.01;

    auto res = brain->client->setVelocity(vx, 0, 0);
    return NodeStatus::SUCCESS;
}

NodeStatus CamTrackBall::tick()
{
    const bool goalkeeper =
        brain->tree->getEntry<string>("player_role") == "goal_keeper";
    const bool frontOnly =
    goalkeeper &&
    brain->get_parameter(
        "goalkeeper.camera.front_only_enabled"
    ).as_bool();

const double searchYawLimit =
    goalkeeper
        ? std::clamp(
            brain->get_parameter(
                "goalkeeper.camera.search_yaw_limit"
            ).as_double(),
            0.10,
            M_PI_2 - 0.02)
        : 1.10;

const double trackingYawLimit =
    goalkeeper
        ? std::clamp(
            brain->get_parameter(
                "goalkeeper.camera.tracking_yaw_limit"
            ).as_double(),
            0.10,
            M_PI_2 - 0.02)
        : std::max(
            std::fabs(
                brain->config
                    ->headYawLimitLeft),
            std::fabs(
                brain->config
                    ->headYawLimitRight));

const bool useTeammateHint =
    !goalkeeper ||
    brain->get_parameter(
        "goalkeeper.camera.use_teammate_ball_hint"
    ).as_bool();
        const double kTrackToleranceRatio = goalkeeper
        ? std::clamp(brain->get_parameter(
            "goalkeeper.camera.track_tolerance_ratio").as_double(), 0.02, 0.49)
        : 0.30;
    // Use hysteresis so a noisy detection around the center does not toggle
    // tracking on and off every tick. The entry box is intentionally smaller
    // than the exit box.
    const double centerFactor = goalkeeper
        ? std::clamp(brain->get_parameter(
            "goalkeeper.camera.center_tolerance_factor").as_double(), 0.1, 1.0)
        : 0.80;
    const double kCenterToleranceRatio = centerFactor * kTrackToleranceRatio;
    const double kFilterTimeConstantSec = goalkeeper
        ? std::max(0.005, brain->get_parameter(
            "goalkeeper.camera.filter_time_constant_sec").as_double())
        : 0.12;
    const double kCommandIntervalSec = goalkeeper
        ? std::max(0.01, brain->get_parameter(
            "goalkeeper.camera.command_interval_sec").as_double())
        : 0.04;
    const double kMaxPitchRate = goalkeeper
        ? std::max(0.05, brain->get_parameter(
            "goalkeeper.camera.max_pitch_rate").as_double())
        : 0.70;
    const double kMaxYawRate = goalkeeper
        ? std::max(0.05, brain->get_parameter(
            "goalkeeper.camera.max_yaw_rate").as_double())
        : 1.00;
    const double kMinCommandChange = goalkeeper
        ? std::max(0.0005, brain->get_parameter(
            "goalkeeper.camera.min_command_change").as_double())
        : 0.006;

    const double imageWidth = brain->config->camPixX;
    const double imageHeight = brain->config->camPixY;
    if (!std::isfinite(imageWidth) || !std::isfinite(imageHeight) ||
        imageWidth <= 1.0 || imageHeight <= 1.0) {
        trackingInitialized_ = false;
        return NodeStatus::SUCCESS;
    }

    const double pixToleranceX = imageWidth * kTrackToleranceRatio;
    const double pixToleranceY = imageHeight * kTrackToleranceRatio;
    const double centerToleranceX = imageWidth * kCenterToleranceRatio;
    const double centerToleranceY = imageHeight * kCenterToleranceRatio;
    const double xCenter = imageWidth / 2.0;
    const double yCenter = imageHeight / 2.0;

    const bool logTrackingVisual = brain->log->shouldLog(
        "cam_track_ball_visual", brain->config->rerunLogVisualHz);
    auto logTrackingBox = [=](int color, string label) {
        if (!logTrackingVisual)
            return;
        brain->log->setTimeNow();
        vector<rerun::Vec2D> mins;
        vector<rerun::Vec2D> sizes;
        mins.push_back(rerun::Vec2D{xCenter - pixToleranceX, yCenter - pixToleranceY});
        sizes.push_back(rerun::Vec2D{pixToleranceX * 2, pixToleranceY * 2});
        brain->log->log(
            "image/track_ball",
            rerun::Boxes2D::from_mins_and_sizes(mins, sizes)
                .with_labels({label})
                .with_colors(color)
        );   

    };

    const bool iSeeBall = brain->data->ballDetected;
    const bool iKnowBallPos = brain->tree->getEntry<bool>("ball_location_known");
    const bool tmBallPosReliable = brain->tree->getEntry<bool>("tm_ball_pos_reliable");
    if (!(iKnowBallPos || tmBallPosReliable)) {
        trackingInitialized_ = false;
        return NodeStatus::SUCCESS;
    }

    const auto now = std::chrono::steady_clock::now();
    double tickDt = 0.01;
    bool timingReset = !trackingInitialized_;
    if (trackingInitialized_) {
        tickDt = std::chrono::duration<double>(now - lastTickTime_).count();
        timingReset = !std::isfinite(tickDt) || tickDt <= 0.0 || tickDt > 0.25;
        if (timingReset) tickDt = 0.01;
    }
    tickDt = std::clamp(tickDt, 0.001, 0.10);
    const double filterAlpha = 1.0 - std::exp(-tickDt / kFilterTimeConstantSec);

    const auto finiteHeadPose = [](double pitch, double yaw) {
        return std::isfinite(pitch) && std::isfinite(yaw);
    };
    const double feedbackPitch = finiteHeadPose(
        brain->data->headPitch, brain->data->headYaw)
        ? brain->data->headPitch
        : lastCommandPitch_;
    const double feedbackYaw = finiteHeadPose(
        brain->data->headPitch, brain->data->headYaw)
        ? brain->data->headYaw
        : lastCommandYaw_;

    const auto &box = brain->data->ball.boundingBox;
    const bool validVisionBox = iSeeBall &&
        std::isfinite(box.xmin) && std::isfinite(box.xmax) &&
        std::isfinite(box.ymin) && std::isfinite(box.ymax) &&
        box.xmax > box.xmin && box.ymax > box.ymin &&
        box.xmax >= 0.0 && box.xmin <= imageWidth &&
        box.ymax >= 0.0 && box.ymin <= imageHeight;
    const bool useVision = validVisionBox;
    double targetPitch = 0.0;
    double targetYaw = 0.0;

    if (useVision) {
        const double ballX = std::clamp(
            mean(box.xmax, box.xmin), 0.0, imageWidth);
        const double ballY = std::clamp(
            mean(box.ymax, box.ymin), 0.0, imageHeight);
        const double deltaX = ballX - xCenter;
        const double deltaY = ballY - yCenter;
        const bool outsideExitBox =
            std::fabs(deltaX) >= pixToleranceX ||
            std::fabs(deltaY) >= pixToleranceY;
        const bool insideEntryBox =
            std::fabs(deltaX) <= centerToleranceX &&
            std::fabs(deltaY) <= centerToleranceY;

        if (!trackingInitialized_ || timingReset || !trackingFromVision_) {
            targetInDeadband_ = insideEntryBox;
            filteredDeltaX_ = targetInDeadband_ ? 0.0 : deltaX;
            filteredDeltaY_ = targetInDeadband_ ? 0.0 : deltaY;
        } else {
            if (targetInDeadband_) {
                if (outsideExitBox) targetInDeadband_ = false;
            } else if (insideEntryBox) {
                targetInDeadband_ = true;
            }

            const double effectiveDeltaX = targetInDeadband_ ? 0.0 : deltaX;
            const double effectiveDeltaY = targetInDeadband_ ? 0.0 : deltaY;
            filteredDeltaX_ += filterAlpha * (effectiveDeltaX - filteredDeltaX_);
            filteredDeltaY_ += filterAlpha * (effectiveDeltaY - filteredDeltaY_);
        }

        targetPitch = feedbackPitch +
            filteredDeltaY_ / imageHeight * brain->config->camAngleY;
        targetYaw = feedbackYaw -
            filteredDeltaX_ / imageWidth * brain->config->camAngleX;
        logTrackingBox(
            targetInDeadband_ ? 0x00FF00FF : 0xFF0000FF,
            format(
                "ballX: %.1f, ballY: %.1f, deltaX: %.1f, deltaY: %.1f, "
                "filteredDeltaX: %.1f, filteredDeltaY: %.1f",
                ballX, ballY, deltaX, deltaY, filteredDeltaX_, filteredDeltaY_));
    } else {
    const auto objectInFront =
        [frontOnly,
         trackingYawLimit](
            const GameObject &obj)
    {
        if (!frontOnly)
            return true;

        return
            std::isfinite(
                obj.posToRobot.x) &&
            std::isfinite(
                obj.yawToRobot) &&
            obj.posToRobot.x > 0.0 &&
            std::abs(
                obj.yawToRobot) <=
                trackingYawLimit;
    };

    bool havePositionTarget =
        false;


    // Prefer the locally remembered position if it is still in front.
    if (iKnowBallPos &&
        objectInFront(
            brain->data->ball))
    {
        targetPitch =
            brain->data
                ->ball.pitchToRobot;

        targetYaw =
            brain->data
                ->ball.yawToRobot;

        havePositionTarget =
            true;
    }


    // Otherwise use a teammate hint, but only when it is in the frontal
    // sector allowed for the goalkeeper.
    else if (
        tmBallPosReliable &&
        useTeammateHint &&
        objectInFront(
            brain->data->tmBall))
    {
        targetPitch =
            brain->data
                ->tmBall.pitchToRobot;

        targetYaw =
            brain->data
                ->tmBall.yawToRobot;

        havePositionTarget =
            true;
    }


    if (havePositionTarget)
    {
        fallbackScanInitialized_ =
            false;

        if (!finiteHeadPose(
                targetPitch,
                targetYaw))
        {
            trackingInitialized_ =
                false;

            return NodeStatus::SUCCESS;
        }

        if (!trackingInitialized_ ||
            timingReset ||
            trackingFromVision_)
        {
            filteredTargetPitch_ =
                targetPitch;

            filteredTargetYaw_ =
                targetYaw;
        }
        else
        {
            filteredTargetPitch_ +=
                filterAlpha *
                (targetPitch -
                 filteredTargetPitch_);

            filteredTargetYaw_ +=
                filterAlpha *
                (targetYaw -
                 filteredTargetYaw_);
        }

        targetPitch =
            filteredTargetPitch_;

        targetYaw =
            filteredTargetYaw_;
    }


    // Ball information exists but points behind/outside the frontal sector:
    // scan the front instead of turning toward the rear.
    else if (frontOnly)
    {
        using head_scan_policy::Pose;

        const std::array<Pose, 6>
            frontWaypoints{{
                {1.0, +searchYawLimit},
                {1.0,  0.0},
                {1.0, -searchYawLimit},
                {0.2, -searchYawLimit},
                {0.2,  0.0},
                {0.2, +searchYawLimit},
            }};

        if (!fallbackScanInitialized_)
        {
            fallbackScanInitialized_ =
                true;

            fallbackScanStartTime_ =
                now;
        }

        const double cycleSec =
            std::max(
                0.5,
                static_cast<double>(
                    brain->get_parameter(
                        "goalkeeper.camera.search_cycle_msec"
                    ).as_int()) /
                    1000.0);

        const double elapsedSec =
            std::chrono::duration<double>(
                now -
                fallbackScanStartTime_)
            .count();

        const double phase =
            elapsedSec /
            cycleSec;

        const auto scanTarget =
            head_scan_policy::
                closedWaypointPose(
                    frontWaypoints,
                    phase);

        targetPitch =
            scanTarget.pitch;

        targetYaw =
            scanTarget.yaw;
    }
    else
    {
        trackingInitialized_ =
            false;

        return NodeStatus::SUCCESS;
    }

    targetInDeadband_ =
        false;
}

    if (!finiteHeadPose(targetPitch, targetYaw)) {
        trackingInitialized_ = false;
        return NodeStatus::SUCCESS;
    }
    targetPitch = std::max(targetPitch, brain->config->headPitchLimitUp);
    if (frontOnly)
{
    targetYaw =
        std::clamp(
            targetYaw,
            -trackingYawLimit,
            trackingYawLimit);
}
else
{
    targetYaw =
        cap(
            targetYaw,
            brain->config
                ->headYawLimitLeft,
            brain->config
                ->headYawLimitRight);
}

    if (!trackingInitialized_ || timingReset ||
        !finiteHeadPose(lastCommandPitch_, lastCommandYaw_)) {
        // Keep the internal command state in the same soft-limited range as
        // RobotClient, otherwise a feedback value just outside a limit could
        // cause repeated commands that the client clips away.
        lastCommandPitch_ = std::max(
            feedbackPitch, brain->config->headPitchLimitUp);
        if (frontOnly)
{
    lastCommandYaw_ =
        std::clamp(
            feedbackYaw,
            -trackingYawLimit,
            trackingYawLimit);
}
else
{
    lastCommandYaw_ =
        cap(
            feedbackYaw,
            brain->config
                ->headYawLimitLeft,
            brain->config
                ->headYawLimitRight);
}
        lastCommandTime_ = std::chrono::steady_clock::time_point{};
    }

    const double commandElapsed = lastCommandTime_ ==
            std::chrono::steady_clock::time_point{}
        ? std::numeric_limits<double>::infinity()
        : std::chrono::duration<double>(now - lastCommandTime_).count();
    if (commandElapsed >= kCommandIntervalSec) {
        const auto command = head_scan_policy::rateLimitedPose(
            {lastCommandPitch_, lastCommandYaw_},
            {targetPitch, targetYaw},
            std::clamp(commandElapsed, 0.0, 0.10),
            kMaxPitchRate,
            kMaxYawRate);
        const double commandChange = std::hypot(
            command.pitch - lastCommandPitch_, command.yaw - lastCommandYaw_);
        if (commandChange >= kMinCommandChange) {
            brain->client->moveHead(command.pitch, command.yaw);
            lastCommandPitch_ = command.pitch;
            lastCommandYaw_ = command.yaw;
            lastCommandTime_ = now;
        }
    }

    trackingFromVision_ = useVision;
    trackingInitialized_ = true;
    lastTickTime_ = now;

    return NodeStatus::SUCCESS;
}

CamFindBall::CamFindBall(const string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain)
{
}

NodeStatus CamFindBall::tick()
{
    if (brain->data->ballDetected)
    {
        _scanInitialized = false;
        return NodeStatus::SUCCESS;
    } // All camera nodes return SUCCESS; FAILURE would prevent subsequent nodes from running.

    using head_scan_policy::Pose;

const bool goalkeeper =
    brain->tree->getEntry<string>(
        "player_role") ==
    "goal_keeper";

const double scanYawLimit =
    goalkeeper
        ? std::clamp(
            brain->get_parameter(
                "goalkeeper.camera.search_yaw_limit"
            ).as_double(),
            0.10,
            M_PI_2 - 0.02)
        : 1.10;

const std::array<Pose, 6>
    kWaypoints{{
        {1.0, +scanYawLimit},
        {1.0,  0.0},
        {1.0, -scanYawLimit},
        {0.2, -scanYawLimit},
        {0.2,  0.0},
        {0.2, +scanYawLimit},
    }};

const rclcpp::Time now =
    brain->get_clock()->now();

double tickDt = 0.0;

if (_scanInitialized)
{
    tickDt =
        (now - _lastTickTime)
            .nanoseconds() /
        1e9;
}

const bool restarted =
    !_scanInitialized ||
    !std::isfinite(tickDt) ||
    tickDt <= 0.0 ||
    tickDt > 0.3;

const double cycleMsecs =
    goalkeeper
        ? std::max(
            100.0,
            static_cast<double>(
                brain->get_parameter(
                    "goalkeeper.camera.search_cycle_msec"
                ).as_int()))
        : std::max(
            100.0,
            static_cast<double>(
                getInput<int>(
                    "msec_cycle"
                ).value()));

const auto evaluate =
    [&kWaypoints](
        double phase)
{
    return
        head_scan_policy::
            closedWaypointPose(
                kWaypoints,
                phase);
};

    if (restarted) {
        _scanStartTime = now;
        _scanPhaseOffset = head_scan_policy::nearestPhase(
            evaluate,
            {brain->data->headPitch, brain->data->headYaw},
            0.8,
            2.2);
        const Pose current{
            brain->data->headPitch,
            brain->data->headYaw,
        };
        const Pose initialTarget = evaluate(_scanPhaseOffset);
        const Pose initial = std::isfinite(current.pitch) &&
                std::isfinite(current.yaw)
            ? current
            : initialTarget;
        _lastCommandPitch = initial.pitch;
        _lastCommandYaw = initial.yaw;
        _scanInitialized = true;
        tickDt = 0.01;
    }

    const double elapsedMsecs =
        (now - _scanStartTime).nanoseconds() / 1e6;
    const Pose target = evaluate(
        _scanPhaseOffset + elapsedMsecs / cycleMsecs);
    const double maxPitchRate = goalkeeper
        ? std::max(0.05, std::fabs(brain->get_parameter(
            "goalkeeper.camera.search_max_pitch_rate").as_double()))
        : std::max(0.05, std::fabs(
            getInput<double>("max_pitch_rate").value()));
    const double maxYawRate = goalkeeper
        ? std::max(0.05, std::fabs(brain->get_parameter(
            "goalkeeper.camera.search_max_yaw_rate").as_double()))
        : std::max(0.05, std::fabs(
            getInput<double>("max_yaw_rate").value()));
    const Pose command = head_scan_policy::rateLimitedPose(
        {_lastCommandPitch, _lastCommandYaw},
        target,
        std::clamp(tickDt, 0.0, 0.05),
        maxPitchRate,
        maxYawRate);

    brain->client->moveHead(command.pitch, command.yaw);
    _lastCommandPitch = command.pitch;
    _lastCommandYaw = command.yaw;
    _lastTickTime = now;
    return NodeStatus::SUCCESS;
}

NodeStatus CamScanField::tick()
{
    double lowPitch, highPitch, leftYaw, rightYaw;
    getInput("low_pitch", lowPitch);
    getInput("high_pitch", highPitch);
    getInput("left_yaw", leftYaw);
    getInput("right_yaw", rightYaw);
    int msecCycle;
    getInput("msec_cycle", msecCycle);
    const double cycleMsecs = std::max(
        100.0, static_cast<double>(msecCycle));
    const auto evaluate = [=](double phase) {
        return head_scan_policy::fieldScanPose(
            phase, highPitch, lowPitch, rightYaw, leftYaw);
    };

    const rclcpp::Time now = brain->get_clock()->now();
    double tickDt = 0.0;
    if (_scanInitialized) {
        tickDt = (now - _lastTickTime).nanoseconds() / 1e9;
    }
    const bool restarted = !_scanInitialized || !std::isfinite(tickDt) ||
        tickDt <= 0.0 || tickDt > 0.3;
    if (restarted) {
        _scanStartTime = now;
        _scanPhaseOffset = head_scan_policy::nearestPhase(
            evaluate,
            {brain->data->headPitch, brain->data->headYaw},
            lowPitch - highPitch,
            leftYaw - rightYaw);
        const head_scan_policy::Pose current{
            brain->data->headPitch,
            brain->data->headYaw,
        };
        const auto initialTarget = evaluate(_scanPhaseOffset);
        const auto initial = std::isfinite(current.pitch) &&
                std::isfinite(current.yaw)
            ? current
            : initialTarget;
        _lastCommandPitch = initial.pitch;
        _lastCommandYaw = initial.yaw;
        _scanInitialized = true;
        tickDt = 0.01;
    }

    const double elapsedMsecs =
        (now - _scanStartTime).nanoseconds() / 1e6;
    const auto target = evaluate(
        _scanPhaseOffset + elapsedMsecs / cycleMsecs);
    const double maxPitchRate = std::max(
        0.05, std::fabs(getInput<double>("max_pitch_rate").value()));
    const double maxYawRate = std::max(
        0.05, std::fabs(getInput<double>("max_yaw_rate").value()));
    const auto command = head_scan_policy::rateLimitedPose(
        {_lastCommandPitch, _lastCommandYaw},
        target,
        std::clamp(tickDt, 0.0, 0.05),
        maxPitchRate,
        maxYawRate);

    brain->client->moveHead(command.pitch, command.yaw);
    _lastCommandPitch = command.pitch;
    _lastCommandYaw = command.yaw;
    _lastTickTime = now;
    return NodeStatus::SUCCESS;
}

NodeStatus DecideCheckBehind::tick()
{
    double maxAngle = 0.0;
    double minAngle = 0.0;
    auto fd = brain->config->fieldDimensions;
    double corners[4][2] = {
        {fd.length / 2.0, fd.width / 2.0}, 
        {-fd.length / 2.0, fd.width / 2.0}, 
        {-fd.length / 2.0, -fd.width / 2.0}, 
        {fd.length / 2.0, -fd.width / 2.0}
    };
    for (int i = 0; i < 4; i++) {
        double angle_f = atan2(corners[i][1] - brain->data->robotPoseToField.y, corners[i][0] - brain->data->robotPoseToField.x);
        double angle = toPInPI(angle_f - brain->data->robotPoseToField.theta);
        if (angle > maxAngle) maxAngle = angle;
        if (angle < minAngle) minAngle = angle;
    }
    if (maxAngle < 1.8 && minAngle > -1.8) brain->tree->setEntry<bool>("need_check_behind", false);
    else brain->tree->setEntry<bool>("need_check_behind", true);
    return NodeStatus::SUCCESS;
}


NodeStatus Chase::tick()
{
    const auto now = brain->get_clock()->now();
    if (
        !brain->tree->getEntry<bool>("ball_location_known")
        || brain->isBallOut(3.0, 1.5)
    )
    {
        _avoidActive = false;
        _avoidSide = 0.0;
        brain->client->setVelocity(0, 0, 0);
        return NodeStatus::SUCCESS;
    }
    double vxLimit, vyLimit, vthetaLimit, dist, safeDist;
    getInput("vx_limit", vxLimit);
    getInput("vy_limit", vyLimit);
    getInput("vtheta_limit", vthetaLimit);
    getInput("dist", dist);
    getInput("safe_dist", safeDist);
    if (brain->tree->getEntry<string>("player_role") == "goal_keeper") {
        vxLimit = brain->get_parameter("goalkeeper.chase.vx_limit").as_double();
        vyLimit = brain->get_parameter("goalkeeper.chase.vy_limit").as_double();
        vthetaLimit = brain->get_parameter(
            "goalkeeper.chase.vtheta_limit").as_double();
        dist = brain->get_parameter(
            "goalkeeper.chase.target_distance").as_double();
        safeDist = brain->get_parameter(
            "goalkeeper.chase.safe_distance").as_double();
    }

    bool avoidObstacle = false;
    double oaSafeDist = 2.0;
    brain->get_parameter("obstacle_avoidance.avoid_during_chase", avoidObstacle);
    brain->get_parameter("obstacle_avoidance.chase_ao_safe_dist", oaSafeDist);

    if (
        brain->config->limitNearBallSpeed
        && brain->data->ball.range < brain->config->nearBallRange
    ) {
        vxLimit = min(brain->config->nearBallSpeedLimit, vxLimit);
    }
    const double globalVyLimit = std::max(0.0, brain->config->vyLimit);
    const double configuredMinimumVy = std::max(
        0.0,
        brain->get_parameter(
            "obstacle_avoidance.robot_avoidance_min_vy").as_double());
    const double avoidanceVyLimit = std::min(
        globalVyLimit,
        std::max(std::max(0.0, vyLimit),
                 std::max(std::max(0.0, brain->config->minVy),
                          configuredMinimumVy)));

    double ballRange = brain->data->ball.range;
    double ballYaw = brain->data->ball.yawToRobot;
    double kickDir = brain->data->kickDir;
    double theta_br = atan2(
        brain->data->robotPoseToField.y - brain->data->ball.posToField.y,
        brain->data->robotPoseToField.x - brain->data->ball.posToField.x
    );
    double theta_rb = brain->data->robotBallAngleToField;
    auto ballPos = brain->data->ball.posToField;

    double vx, vy, vtheta;
    bool avoidanceCommand = false;
    Pose2D target_f, target_r;
    static string targetType = "direct";
    static double circleBackDir = 1.0;
    double dirThreshold = M_PI / 2;
    if (targetType == "direct") dirThreshold *= 1.2;

    if (fabs(toPInPI(kickDir - theta_rb)) < dirThreshold) {
        targetType = "direct";
        target_f.x = ballPos.x - dist * cos(kickDir);
        target_f.y = ballPos.y - dist * sin(kickDir);
    } else {
        targetType = "circle_back";
        double cbDirThreshold = -0.2 * circleBackDir;
        circleBackDir = toPInPI(theta_br - kickDir) > cbDirThreshold ? 1.0 : -1.0;
        double tanTheta = theta_br + circleBackDir * acos(min(1.0, safeDist / max(ballRange, 1e-5)));
        target_f.x = ballPos.x + safeDist * cos(tanTheta);
        target_f.y = ballPos.y + safeDist * sin(tanTheta);
    }
    const bool isGoalkeeper =
    brain->tree->getEntry<string>(
        "player_role") ==
    "goal_keeper";

if (isGoalkeeper &&
    brain->get_parameter(
        "goalkeeper.chase.defensive_clamp_enabled"
    ).as_bool())
{
    const auto fd =
        brain->config->fieldDimensions;

    const double ownGoalX =
        -fd.length / 2.0;

    const double maxDepth =
        std::clamp(
            brain->get_parameter(
                "goalkeeper.chase.max_depth_from_goalline"
            ).as_double(),
            0.4,
            fd.penaltyAreaLength);

    // Reuse the same lateral margin as normal goalkeeper coverage.
    const double lateralLimit =
        fd.goalWidth / 2.0 +
        std::max(
            0.0,
            brain->get_parameter(
                "goalkeeper.blocking.lateral_margin"
            ).as_double());

    target_f.x =
        std::clamp(
            target_f.x,
            ownGoalX + 0.20,
            ownGoalX + maxDepth);

    target_f.y =
        std::clamp(
            target_f.y,
            -lateralLimit,
            lateralLimit);
}
    target_r = brain->data->field2robot(target_f);
    double targetDir = atan2(target_r.y, target_r.x);
    double distToObstacle = brain->distToObstacle(targetDir);
    const double robotPathClearance = std::max(
        0.0,
        brain->get_parameter(
            "obstacle_avoidance.robot_path_clearance").as_double());
    const double lockMsecs = std::max(
        0.0,
        brain->get_parameter(
            "obstacle_avoidance.avoidance_direction_lock_msecs").as_double());
    const bool preferPreviousSide = _avoidActive &&
        std::fabs(_avoidSide) > 0.1;
    const bool hardLockActive = preferPreviousSide &&
        now.nanoseconds() < _avoidUntil.nanoseconds();
    auto robotPlan = brain->planRobotPath(
        target_r.x,
        target_r.y,
        robotPathClearance,
        _avoidSide,
        preferPreviousSide,
        hardLockActive);
    if (hardLockActive && !robotPlan.direct && !robotPlan.hasPath) {
        robotPlan = brain->planRobotPath(
            target_r.x,
            target_r.y,
            robotPathClearance,
            _avoidSide,
            true,
            false);
    }
    ObstacleAvoidanceLogRecord avoidanceLog;
    avoidanceLog.source = "chase";
    avoidanceLog.reason = "direct";
    avoidanceLog.targetX = target_f.x;
    avoidanceLog.targetY = target_f.y;
    avoidanceLog.targetRange = std::hypot(target_r.x, target_r.y);
    avoidanceLog.targetAngle = targetDir;
    avoidanceLog.planDirect = robotPlan.direct;
    avoidanceLog.planHasPath = robotPlan.hasPath;
    avoidanceLog.escapingOverlap = robotPlan.escapingOverlap;
    avoidanceLog.overlappingObstacleCount =
        robotPlan.overlappingObstacleCount;
    avoidanceLog.waypointX = robotPlan.waypoint.x;
    avoidanceLog.waypointY = robotPlan.waypoint.y;
    avoidanceLog.pathMinimumClearance = robotPlan.minimumClearance;
    avoidanceLog.pathStartClearance = robotPlan.startClearance;
    avoidanceLog.escapeClearanceGain = robotPlan.escapeClearanceGain;
    avoidanceLog.avoidanceSide = robotPlan.avoidanceSide;
    avoidanceLog.safeDistance = oaSafeDist;
    avoidanceLog.pathClearance = robotPathClearance;
    avoidanceLog.targetClearance = distToObstacle;
    avoidanceLog.selectedAngle = targetDir;
    avoidanceLog.selectedClearance = distToObstacle;
    const auto blockedRecoveryTurn = [&](double selectedAngle) {
        const double configuredRate = std::max(
            0.0,
            brain->get_parameter(
                "obstacle_avoidance.blocked_recovery_turn_rate").as_double());
        const double rate = std::min(
            configuredRate, std::max(0.0, vthetaLimit));
        if (!std::isfinite(rate) || rate <= 1e-4) return 0.0;
        const double delta = toPInPI(selectedAngle - targetDir);
        return (std::fabs(delta) > 0.1 ? std::copysign(rate, delta) : rate);
    };
    if (avoidObstacle &&
        (!robotPlan.direct || distToObstacle < std::min(
            oaSafeDist,
            std::hypot(target_r.x, target_r.y) + 0.15))) {
        _avoidActive = true;
        avoidanceCommand = true;
        double avoidDir = !robotPlan.direct && robotPlan.hasPath
            ? std::atan2(robotPlan.waypoint.y, robotPlan.waypoint.x)
            : brain->calcAvoidDir(targetDir, oaSafeDist);
        const bool visualPathSafe = robotPlan.direct ||
            robotPlan.escapingOverlap ||
            (robotPlan.hasPath && robotPlan.minimumClearance >= -0.01);
        double selectedClearance = robotPlan.escapingOverlap
            ? brain->distToDepthObstacle(avoidDir)
            : brain->distToObstacle(avoidDir);
        bool usedScanFallback = false;
        if (!visualPathSafe || selectedClearance < 0.20) {
            avoidDir = brain->calcAvoidDir(targetDir, oaSafeDist);
            selectedClearance = brain->distToObstacle(avoidDir);
            usedScanFallback = true;
        }
        if (!std::isfinite(avoidDir) || selectedClearance < 0.20) {
            vx = 0.0;
            vy = 0.0;
            avoidanceLog.reason = "blocked_stop";
        } else {
            const double preferredSpeed = std::max(
                0.0,
                brain->get_parameter(
                    "obstacle_avoidance.robot_avoidance_speed").as_double());
            const double reverseSpeed = std::max(
                0.0,
                brain->get_parameter(
                    "obstacle_avoidance.robot_avoidance_reverse_speed")
                    .as_double());
            const double maximumSpeed = std::cos(avoidDir) < 0.0
                ? std::min(preferredSpeed, reverseSpeed)
                : preferredSpeed;
            const double minimumVy = std::min(
                {configuredMinimumVy, avoidanceVyLimit, maximumSpeed});
            const double adjustedDir =
                robot_path_planner_policy::angleWithMinimumLateralSpeed(
                    avoidDir,
                    maximumSpeed,
                    minimumVy,
                    std::fabs(robotPlan.avoidanceSide) > 0.1
                        ? robotPlan.avoidanceSide
                        : _avoidSide);
            if (std::fabs(toPInPI(adjustedDir - avoidDir)) > 1e-4) {
                const double adjustedClearance = robotPlan.escapingOverlap
                    ? brain->distToDepthObstacle(adjustedDir)
                    : brain->distToObstacle(adjustedDir);
                if (adjustedClearance >= 0.20) {
                    avoidDir = adjustedDir;
                    selectedClearance = adjustedClearance;
                }
            }
            const double minimumSpeed = std::max(
                0.0,
                brain->get_parameter(
                    "obstacle_avoidance.robot_avoidance_min_speed").as_double());
            const double clearance = robotPlan.direct
                ? std::max(0.25, std::min(oaSafeDist, 0.60))
                : std::max(0.0, robotPlan.minimumClearance);
            const double speed = robot_path_planner_policy::velocityForPath(
                avoidDir,
                std::max(0.0, vxLimit),
                avoidanceVyLimit,
                ballRange,
                clearance,
                maximumSpeed,
                minimumSpeed,
                minimumVy);
            const auto limitedVelocity =
                robot_path_planner_policy::limitVelocityMagnitude(
                    {speed * std::cos(avoidDir), speed * std::sin(avoidDir)},
                    maximumSpeed);
            vx = limitedVelocity.x;
            vy = limitedVelocity.y;
            avoidanceLog.reason = robotPlan.escapingOverlap
                ? "escape_overlap"
                : (usedScanFallback || !robotPlan.hasPath
                       ? "scan_fallback"
                       : "detour");
            if (std::hypot(vx, vy) < 1e-4) {
                avoidanceLog.reason = "blocked_stop";
            }
        }
        const double selectedSide = std::fabs(std::sin(avoidDir)) > 0.05
            ? std::copysign(1.0, std::sin(avoidDir))
            : robotPlan.avoidanceSide;
        if (std::fabs(selectedSide) > 0.1) {
            const bool sideFirstSelected = std::fabs(_avoidSide) <= 0.1;
            const bool sideChanged = _avoidSide * selectedSide < 0.0;
            if (sideFirstSelected || sideChanged) {
                _avoidUntil = now + rclcpp::Duration::from_seconds(
                    lockMsecs / 1000.0);
            }
            _avoidSide = selectedSide;
            avoidanceLog.avoidanceSide = selectedSide;
        }
        avoidanceLog.selectedAngle = avoidDir;
        avoidanceLog.selectedClearance = selectedClearance;
        vtheta = avoidanceLog.reason == "blocked_stop"
            ? blockedRecoveryTurn(avoidDir)
            : ballYaw;
    } else {
        _avoidActive = false;
        _avoidSide = 0.0;
        vx = min(vxLimit, ballRange);
        vy = 0;
        vtheta = targetDir;
        if (fabs(targetDir) < 0.1 && ballRange > 2.0) vtheta = 0.0;
        vx *= sigmoid(fabs(vtheta), 1, 3);
    }

    vx = cap(vx, vxLimit, -vxLimit);
    vy = cap(vy, avoidanceCommand ? avoidanceVyLimit : vyLimit,
             avoidanceCommand ? -avoidanceVyLimit : -vyLimit);
    vtheta = cap(vtheta, vthetaLimit, -vthetaLimit);

    brain->client->setVelocity(
        vx,
        vy,
        vtheta,
        !avoidanceCommand,
        !avoidanceCommand,
        !avoidanceCommand);
    if (avoidObstacle) {
        const auto sent = brain->client->lastVelocityCommand();
        avoidanceLog.requestedVx = vx;
        avoidanceLog.requestedVy = vy;
        avoidanceLog.requestedVtheta = vtheta;
        avoidanceLog.sentVx = sent.sentX;
        avoidanceLog.sentVy = sent.sentY;
        avoidanceLog.sentVtheta = sent.sentTheta;
        brain->logObstacleAvoidance(std::move(avoidanceLog));
    }
    return NodeStatus::SUCCESS;
}

NodeStatus SimpleChase::tick()
{
    double stopDist, stopAngle, vyLimit, vxLimit;
    getInput("stop_dist", stopDist);
    getInput("stop_angle", stopAngle);
    getInput("vx_limit", vxLimit);
    getInput("vy_limit", vyLimit);

    if (!brain->tree->getEntry<bool>("ball_location_known"))
    {
        brain->client->setVelocity(0, 0, 0);
        return NodeStatus::SUCCESS;
    }

    double vx = brain->data->ball.posToRobot.x;
    double vy = brain->data->ball.posToRobot.y;
    double vtheta = brain->data->ball.yawToRobot * 2.0; // Larger gain turns the robot faster.

    double linearFactor = 1 / (1 + exp(3 * (brain->data->ball.range * fabs(brain->data->ball.yawToRobot)) - 3)); // Prioritize turning at long range.
    vx *= linearFactor;
    vy *= linearFactor;

    vx = cap(vx, vxLimit, -0.1);     // Apply the final forward-speed limit.
    vy = cap(vy, vyLimit, -vyLimit); // Apply the final lateral-speed limit.

    if (brain->data->ball.range < stopDist)
    {
        vx = 0;
        vy = 0;
    }

    brain->client->setVelocity(vx, vy, vtheta, false, false, false);
    return NodeStatus::SUCCESS;
}


NodeStatus GoToFreekickPosition::onStart() {
    // brain->log->log("debug/freekick_position/onStart", rerun::TextLog(format("stage onStart")));
    _isInFinalAdjust = false;
    return NodeStatus::RUNNING;
}

NodeStatus GoToFreekickPosition::onRunning() {
    auto log = [=](string msg) {
        // brain->log->setTimeNow();
        // brain->log->log("debug/GoToFreekickPosition", rerun::TextLog(msg));
    };
    log("running");

    string side;
    getInput("side", side);
    if (side != "attack" && side != "defense") return NodeStatus::SUCCESS;

    Pose2D targetPose = {0.0, 0.0, 0.0};
    const auto fd = brain->config->fieldDimensions;
    const auto robotPose = brain->data->robotPoseToField;
    const double ownGoalX = -fd.length / 2.0;
    const double oppGoalX = fd.length / 2.0;

    FreeKickPlanState freeKickPlan;
    {
        std::lock_guard<std::mutex> cooperationLock(
            brain->data->cooperationMutex);
        freeKickPlan = brain->data->freeKickPlan;
    }
    const bool frozenOurPlan =
        side == "attack" &&
        freeKickPlan.ourRestart &&
        freeKickPlan.phase >= freekick_policy::Phase::Preparing &&
        freeKickPlan.phase < freekick_policy::Phase::Complete &&
        freeKickPlan.kickerId > 0;
    const Point ballPos = frozenOurPlan &&
            std::isfinite(freeKickPlan.ballSnapshot.x) &&
            std::isfinite(freeKickPlan.ballSnapshot.y)
        ? freeKickPlan.ballSnapshot
        : brain->data->ball.posToField;
    double kickDir = brain->data->kickDir;
    if (frozenOurPlan &&
        std::isfinite(freeKickPlan.passTarget.x) &&
        std::isfinite(freeKickPlan.passTarget.y) &&
        std::hypot(
            freeKickPlan.passTarget.x - ballPos.x,
            freeKickPlan.passTarget.y - ballPos.y) > 0.1) {
        kickDir = std::atan2(
            freeKickPlan.passTarget.y - ballPos.y,
            freeKickPlan.passTarget.x - ballPos.x);
    }
    const double defenseDir = atan2(ballPos.y, ballPos.x + fd.length / 2.0);
    // Once a restart plan exists, it is the authority for both the kicker and
    // the frozen semantic slots. Normal-play ownership may continue changing.
    const bool plannedKicker =
        frozenOurPlan && freeKickPlan.kickerId == brain->config->playerId;
    const AssistSlot placementSlot = frozenOurPlan &&
            brain->config->playerId >= 1 &&
            brain->config->playerId <= MAX_NUM_PLAYERS
        ? freeKickPlan.assistSlots[brain->config->playerId - 1]
        : brain->data->tmMyAssistSlot;
    int rank = (frozenOurPlan ? plannedKicker : brain->data->tmImLead) ? 0 : 4;
    if (rank != 0) {
        switch (placementSlot) {
        case AssistSlot::SHADOW_SUPPORT: rank = 1; break;
        case AssistSlot::COVER_MID: rank = 2; break;
        case AssistSlot::ANCHOR_COVER: rank = 3; break;
        case AssistSlot::WIDE_OUTLET: rank = 4; break;
        default: break;
        }
    }
    if (side == "attack") {
        double attackDist = 0.7;
        getInput("attack_dist", attackDist);

        if (rank == 0) {
            targetPose.x = ballPos.x - attackDist * cos(kickDir);
            targetPose.y = ballPos.y - attackDist * sin(kickDir);
            targetPose.theta = kickDir;
        } else if (rank == 1) {
            targetPose.x = ballPos.x - 2.0 * cos(defenseDir);
            targetPose.y = ballPos.y - 2.0 * sin(defenseDir);
            targetPose.theta = defenseDir;
        } else if (rank == 2) {
            targetPose.x = - fd.length / 2.0 + fd.penaltyDist;
            targetPose.y = fd.goalAreaWidth / 2.0;
        } else if (rank == 3) {
            targetPose.x = - fd.length / 2.0 + fd.penaltyDist;
            targetPose.y = - fd.goalAreaWidth / 2.0;
        } else { // Unexpected extra striker: use a central rear position distinct from rank 3.
            targetPose.x = ownGoalX + fd.penaltyAreaLength + 0.5;
            targetPose.y = 0.0;
            log(format("freekick attack extra striker, rank=%d", rank));
        }
    } else if (side == "defense") {
        if (rank == 0) {
            targetPose.x = ballPos.x - 3.0 * cos(defenseDir);
            targetPose.y = ballPos.y - 2.5 * sin(defenseDir);
            targetPose.theta = defenseDir;
        } else if (rank == 1) {
            targetPose.x = ballPos.x - 3.5 * cos(defenseDir);
            targetPose.y = ballPos.y - 4.0 * sin(defenseDir);
            targetPose.theta = defenseDir;
        } else if (rank == 2) {
            targetPose.x = - fd.length / 2.0 + fd.penaltyDist;
            targetPose.y = fd.goalAreaWidth / 2.0;
        } else if (rank == 3) {
            targetPose.x = - fd.length / 2.0 + fd.penaltyDist;
            targetPose.y = - fd.goalAreaWidth / 2.0;
        } else { // Unexpected extra striker: cover the center without overlapping rank 3.
            targetPose.x = ownGoalX + fd.penaltyAreaLength + 0.5;
            targetPose.y = 0.0;
            log(format("freekick defense extra striker, rank=%d", rank));
        }
    }

    // Keep the target inside the feasible field region, consistent with Assist.
    targetPose.x = cap(targetPose.x, oppGoalX - 0.3, ownGoalX + 0.3);
    targetPose.y = cap(targetPose.y, fd.width / 2.0 - 0.6, -fd.width / 2.0 + 0.6);
    // Fixed rear positions at rank >= 2 are more stable when facing the ball.
    if (rank >= 2) {
        targetPose.theta = atan2(ballPos.y - targetPose.y, ballPos.x - targetPose.x);
    }

    const double dist = norm(targetPose.x - robotPose.x, targetPose.y - robotPose.y);
    const double deltaDir = toPInPI(targetPose.theta - robotPose.theta);

    if ( // Target position reached.
        dist < 0.3
        && fabs(deltaDir) < 0.15
    ) {
        brain->client->setVelocity(0, 0, 0);
        return NodeStatus::SUCCESS;
    }

    const bool enableFreekickAvoid = brain->get_parameter(
        "obstacle_avoidance.enable_freekick_avoid").as_bool();
    auto targetPose_r = brain->data->field2robot(targetPose);
    bool finalPathBlocked = false;
    if (enableFreekickAvoid) {
        const double robotPathClearance = std::max(
            0.0,
            brain->get_parameter(
                "obstacle_avoidance.robot_path_clearance").as_double());
        const auto robotPlan = brain->planRobotPath(
            targetPose_r.x,
            targetPose_r.y,
            robotPathClearance);
        const double targetDirection = std::atan2(
            targetPose_r.y, targetPose_r.x);
        finalPathBlocked = !robotPlan.direct ||
            brain->distToObstacle(targetDirection) <
                std::min(
                    std::max(0.3, brain->config->safeDistance),
                    std::hypot(targetPose_r.x, targetPose_r.y) + 0.15);
    }

    // The final adjustment keeps the ball-specific micro correction, but it
    // is entered only after the complete visual/depth route is clear.
    if (!enableFreekickAvoid ||
        (!finalPathBlocked && (dist < 1.5 || _isInFinalAdjust))) {
        _isInFinalAdjust = true; // Enter final fine adjustment.

        double vx = targetPose_r.x;
        double vy = targetPose_r.y;
        const double ballRange = std::hypot(
            ballPos.x - robotPose.x, ballPos.y - robotPose.y);
        const double ballYaw = toPInPI(
            std::atan2(ballPos.y - robotPose.y, ballPos.x - robotPose.x) -
            robotPose.theta);
        double vtheta = ballYaw * 2.0; // Larger gain turns the robot faster.

        double linearFactor = 1 / (1 + exp(3 * (ballRange * fabs(ballYaw)) - 3)); // Prioritize turning at long range.
        vx *= linearFactor;
        vy *= linearFactor;

        // Avoid colliding with the ball.
        Line path = {robotPose.x, robotPose.y, targetPose.x, targetPose.y};
        if (
            pointMinDistToLine(Point2D({ballPos.x, ballPos.y}), path) < 0.7
            && ballRange < 1.2
        ) {
            vx = min(0.0, vx);
            vy = vy >= 0 ? vy + 0.1: vy - 0.1;
        }

        double vxLimit, vyLimit;
        getInput("vx_limit", vxLimit);
        getInput("vy_limit", vyLimit);
        vx = cap(vx, vxLimit, -0.4);         // Apply the final forward-speed limit.
        vy = cap(vy, vyLimit, -vyLimit);     // Apply the final lateral-speed limit.

        brain->client->setVelocity(vx, vy, vtheta, false, false, false);
        return NodeStatus::RUNNING;
    }

    double longRangeThreshold = 1.4;
    double turnThreshold = 0.4;
    double vxLimit, vyLimit;
    getInput("vx_limit", vxLimit);
    getInput("vy_limit", vyLimit);
    double vthetaLimit = 1.5;
    bool avoidObstacle = true;
    brain->client->moveToPoseOnField3(targetPose.x, targetPose.y, targetPose.theta, longRangeThreshold, turnThreshold, vxLimit, vyLimit, vthetaLimit, 0.2, 0.2, 0.1, avoidObstacle);

    return NodeStatus::RUNNING;
}

void GoToFreekickPosition::onHalted() {
    // brain->log->log("debug/freekick_position/onHault", rerun::TextLog(format("stage OnHalted")));
}

NodeStatus GoToGoalBlockingPosition::tick() {
    auto log = [=](string msg) {
        // brain->log->setTimeNow();
        // brain->log->log("debug/GoToGoalBlockingPosition", rerun::TextLog(msg));
    };
    log("GoToGoalBlockingPosition ticked");

    // if (!brain->tree->getEntry<bool>("ball_location_known")) {
    //      brain->client->setVelocity(0, 0, 0);
    //      return NodeStatus::SUCCESS;
    // }
    // brain->log->setTimeNow();
    // brain->log->log("tree/GoToGoalBlockingPosition", rerun::TextLog("GoToGoalBlockingPosition tick"));
     
    double distTolerance = std::max(
        0.01, brain->get_parameter(
            "goalkeeper.blocking.dist_tolerance").as_double());
    double thetaTolerance = std::max(
        0.01, brain->get_parameter(
            "goalkeeper.blocking.theta_tolerance").as_double());
    double distToGoalline = brain->get_parameter(
        "goalkeeper.blocking.dist_to_goalline").as_double();

    const auto fd = brain->config->fieldDimensions;
    const auto ballPos = brain->data->ball.posToField;
    const auto robotPose = brain->data->robotPoseToField;

    const string curRole = brain->tree->getEntry<string>("player_role");

    const double ownGoalX = -fd.length / 2.0;
    // Keep the target inside the goal area even when a malformed XML value is
    // supplied. The normal RoboLeague value is 1.0 m.
    const double safeDistToGoalline = std::clamp(
        distToGoalline,
        0.4,
        std::max(0.4, fd.goalAreaLength - 0.2));
        const bool ballPositionReliable =
        curRole != "goal_keeper" ||
        brain->tree->getEntry<bool>("ball_location_known") ||
        brain->tree->getEntry<bool>("tm_ball_pos_reliable");

    const double safeBallX =
        ballPositionReliable &&
        std::isfinite(ballPos.x)
            ? ballPos.x
            : ownGoalX;

    const double safeBallY =
        ballPositionReliable &&
        std::isfinite(ballPos.y)
            ? ballPos.y
            : 0.0;

    const double safeBallYaw =
        ballPositionReliable &&
        std::isfinite(brain->data->ball.yawToRobot)
            ? brain->data->ball.yawToRobot
            : (
                curRole == "goal_keeper"
                    ? toPInPI(-robotPose.theta)
                    : 0.0
            );
    const double ballDepth = std::max(
        safeDistToGoalline,
        safeBallX - ownGoalX);

    Pose2D targetPose;
    targetPose.x = curRole == "striker"
        ? std::max(ownGoalX + safeDistToGoalline, safeBallX - 1.5)
        : ownGoalX + safeDistToGoalline;
    if (curRole == "goal_keeper") {
        // Project the goal-to-ball ray onto the goalkeeper's line. For a ball
        // at or behind that line, keep the finite ball-side target instead of
        // jumping to goalWidth/4; this is continuous at the goal line and
        // remains inside the goalkeeper's lateral playing area.
        targetPose.y =
    safeBallY *
    safeDistToGoalline /
    ballDepth;

const bool limitToGoalMouth =
    brain->get_parameter(
        "goalkeeper.blocking.limit_to_goal_mouth"
    ).as_bool();

const double lateralLimit =
    limitToGoalMouth
        ? fd.goalWidth / 2.0 +
            std::max(
                0.0,
                brain->get_parameter(
                    "goalkeeper.blocking.lateral_margin"
                ).as_double())
        : fd.penaltyAreaWidth / 2.0;

targetPose.y =
    std::clamp(
        targetPose.y,
        -lateralLimit,
        lateralLimit);
    } else {
        targetPose.y = safeBallY * safeDistToGoalline / ballDepth;
        targetPose.y = std::clamp(
            targetPose.y, -fd.goalWidth / 2.0, fd.goalWidth / 2.0);
    }

    double dist = norm(targetPose.x - robotPose.x, targetPose.y - robotPose.y);
    if ( // Target position reached.
        dist < distTolerance
        && fabs(safeBallYaw) < thetaTolerance
    ) {
        brain->client->setVelocity(0, 0, 0);
        return NodeStatus::SUCCESS;
    }

    auto targetPose_r = brain->data->field2robot(targetPose);
    const double positionGain = std::max(
        0.0, brain->get_parameter(
            "goalkeeper.blocking.position_gain").as_double());
    const double orientationGain = std::max(
        0.0, brain->get_parameter(
            "goalkeeper.blocking.orientation_gain").as_double());
    double vx = targetPose_r.x * positionGain;
    double vy = targetPose_r.y * positionGain;


    double vxLimit = std::max(
        0.0, brain->get_parameter(
            "goalkeeper.blocking.vx_limit").as_double());
    double vyLimit = std::max(
        0.0, brain->get_parameter(
            "goalkeeper.blocking.vy_limit").as_double());
    double vthetaLimit = std::max(
        0.0, brain->get_parameter(
            "goalkeeper.blocking.vtheta_limit").as_double());
    double vtheta = cap(
        safeBallYaw * orientationGain,
        vthetaLimit,
        -vthetaLimit);
    vx = cap(vx, vxLimit, -vxLimit);     // Apply the final forward-speed limit.
    vy = cap(vy, vyLimit, -vyLimit);     // Apply the final lateral-speed limit.
     

    brain->client->setVelocity(vx, vy, vtheta, false, false, false);
    return NodeStatus::SUCCESS;
}

NodeStatus GoalkeeperBlockShot::tick()
{
    if (!brain->get_parameter("goalkeeper.prediction.enabled").as_bool() ||
    !brain->data->ballWillBreach) {
    brain->data->goalkeeperUrgentBlock = false;
    brain->data->goalkeeperForwardInterceptActive = false;
    brain->data->goalkeeperFrontIntercept = false;

    // A completed/expired threat must not leave a stale target available
    // for the next shot.
    cachedTargetValid_ = false;
    brain->data->goalkeeperBlockTargetSource = "none";

    brain->client->setVelocity(
        0.0, 0.0, 0.0,
        false, false, false);

    return NodeStatus::SUCCESS;
}
    const auto fd =
        brain->config->fieldDimensions;

    const double ownGoalX =
        -fd.length / 2.0;

    const double distToGoalLine =
        std::clamp(
            brain->get_parameter(
                "goalkeeper.blocking.dist_to_goalline"
            ).as_double(),
            0.4,
            std::max(
                0.4,
                fd.goalAreaLength - 0.2));

    const double lateralLimit =
        fd.goalWidth / 2.0 +
        std::max(
            0.0,
            brain->get_parameter(
                "goalkeeper.prediction.goal_margin"
            ).as_double());

    Pose2D targetField{
        ownGoalX + distToGoalLine,
        std::clamp(
            brain->data->ballInterceptPoint.y,
            -lateralLimit,
            lateralLimit),
        0.0
    };
    
    // -------------------------------------------------------------------------
    // Predictive interception target management.
    //
    // A threat can remain active for activation_hold_msec after the current
    // trajectory fit has become invalid. During that HOLD window we must not
    // use the noisy/new ball observation to create a completely different
    // interception target.
    // -------------------------------------------------------------------------

    const auto now = brain->get_clock()->now();

    const bool freezeDuringHold =
        brain->get_parameter(
            "goalkeeper.prediction.freeze_target_during_hold"
        ).as_bool();

    const bool currentThreat =
        brain->data->ballPredictionCurrentThreat;

    const bool heldThreat =
        brain->data->ballPredictionHeldThreat;

    double targetTime =
        brain->data->ballTimeToIntercept;

    bool forwardInterceptActive = false;
    bool frontIntercept = false;

    double reachSpeed =
        brain->data->goalkeeperAdaptiveReachSpeed;

    const bool useCachedTarget =
        freezeDuringHold &&
        heldThreat &&
        !currentThreat &&
        cachedTargetValid_ &&
        cachedDeadline_.nanoseconds() > 0 &&
        cachedDeadline_ > now;


    // -------------------------------------------------------------------------
    // CASE 1:
    // The predictor is currently invalid, but the threat is still retained by
    // activation_hold_msec. Keep exactly the last valid target.
    // -------------------------------------------------------------------------
    if (useCachedTarget)
    {
        targetField =
            cachedTargetField_;

        forwardInterceptActive =
            cachedForwardIntercept_;

        frontIntercept =
            cachedFrontIntercept_;

        reachSpeed =
            cachedReachSpeed_;

        targetTime = std::max(
            0.0,
            (cachedDeadline_ - now).seconds());

        brain->data->goalkeeperBlockTargetSource =
            "cached_prediction";
    }


    // -------------------------------------------------------------------------
    // CASE 2:
    // HOLD is active, but no cached target is available.
    // Fall back to the fixed defensive line instead of calculating an
    // aggressive forward interception from an invalid trajectory.
    // -------------------------------------------------------------------------
    else if (
        freezeDuringHold &&
        heldThreat &&
        !currentThreat)
    {
        targetField.x =
            ownGoalX + distToGoalLine;

        targetField.y = std::clamp(
            brain->data->ballInterceptPoint.y,
            -lateralLimit,
            lateralLimit);

        forwardInterceptActive = false;
        frontIntercept = false;

        targetTime = std::max(
            0.0,
            brain->data->ballTimeToIntercept);

        brain->data->goalkeeperBlockTargetSource =
            "defensive_fallback";
    }


    // -------------------------------------------------------------------------
    // CASE 3:
    // Fresh/current trajectory. Only here are we allowed to calculate a new
    // forward or diagonal interception.
    // -------------------------------------------------------------------------
    else
    {
        brain->data->goalkeeperBlockTargetSource =
            "current_prediction";

        const bool forwardInterceptEnabled =
            brain->get_parameter(
                "goalkeeper.prediction.intercept.enabled"
            ).as_bool();

        const double lateralErrorAtLine =
            brain->data->ballInterceptPoint.y -
            brain->data->robotPoseToField.y;

        const double frontThreshold =
            std::max(
                0.0,
                brain->get_parameter(
                    "goalkeeper.prediction.intercept.front_lateral_threshold"
                ).as_double());

        frontIntercept =
            forwardInterceptEnabled &&
            std::abs(lateralErrorAtLine) <=
                frontThreshold;

        const double configuredMaxForward =
            std::max(
                0.0,
                brain->get_parameter(
                    frontIntercept
                        ? "goalkeeper.prediction.intercept.front_max_forward_distance"
                        : "goalkeeper.prediction.intercept.max_forward_distance"
                ).as_double());

        const double ballSeparation =
            std::max(
                0.05,
                brain->get_parameter(
                    "goalkeeper.prediction.intercept.min_ball_separation"
                ).as_double());

        const double searchStep =
            std::max(
                0.02,
                brain->get_parameter(
                    "goalkeeper.prediction.intercept.search_step"
                ).as_double());

        const double speedMin =
            std::max(
                0.05,
                brain->get_parameter(
                    "goalkeeper.prediction.intercept.robot_speed_min"
                ).as_double());

        const double speedMax =
            std::max(
                speedMin,
                brain->get_parameter(
                    "goalkeeper.prediction.intercept.robot_speed_max"
                ).as_double());

        const double measuredSpeedGain =
            std::max(
                0.1,
                brain->get_parameter(
                    "goalkeeper.prediction.intercept.measured_speed_gain"
                ).as_double());

        reachSpeed =
            std::clamp(
                brain->data->goalkeeperMeasuredOdomSpeed *
                    measuredSpeedGain,
                speedMin,
                speedMax);

        const double safetyTime =
            std::max(
                0.0,
                brain->get_parameter(
                    "goalkeeper.prediction.intercept.safety_time_sec"
                ).as_double());

        const double ballX =
            brain->data->ball.posToField.x;

        const double ballY =
            brain->data->ball.posToField.y;

        const double dxToLine =
            targetField.x - ballX;

        const double dyToLine =
            targetField.y - ballY;

        const double distanceToLine =
            std::hypot(
                dxToLine,
                dyToLine);

        const double ballSpeed =
            std::hypot(
                brain->data->ballVelocityX,
                brain->data->ballVelocityY);

        const double deceleration =
            std::max(
                0.0,
                brain->get_parameter(
                    "goalkeeper.prediction.deceleration"
                ).as_double());


        const auto ballTravelTime =
            [ballSpeed, deceleration](double distance)
        {
            if (distance <= 0.0)
                return 0.0;

            if (ballSpeed <= 1e-6)
                return std::numeric_limits<double>::infinity();

            if (deceleration <= 1e-6)
                return distance / ballSpeed;

            const double discriminant =
                ballSpeed * ballSpeed -
                2.0 * deceleration * distance;

            if (discriminant < 0.0)
                return std::numeric_limits<double>::infinity();

            return
                (ballSpeed -
                 std::sqrt(discriminant)) /
                deceleration;
        };


        if (forwardInterceptEnabled &&
            distanceToLine > 1e-6 &&
            ballX >
                targetField.x + ballSeparation)
        {
            const double ballLimitedForward =
                std::max(
                    0.0,
                    ballX -
                    ballSeparation -
                    targetField.x);

            const double maxForward =
                std::min(
                    configuredMaxForward,
                    ballLimitedForward);


            // Search from the most advanced reachable point backwards.
            for (double forward = maxForward;
                 forward >= searchStep * 0.5;
                 forward -= searchStep)
            {
                const double candidateX =
                    targetField.x + forward;

                const double ratio =
                    std::clamp(
                        (candidateX - ballX) /
                            dxToLine,
                        0.0,
                        1.0);

                const double candidateY =
                    ballY +
                    ratio * dyToLine;

                const double candidateDistance =
                    std::hypot(
                        candidateX - ballX,
                        candidateY - ballY);

                const double candidateTime =
                    ballTravelTime(
                        candidateDistance);

                const double robotDistance =
                    std::hypot(
                        candidateX -
                            brain->data
                                ->robotPoseToField.x,
                        candidateY -
                            brain->data
                                ->robotPoseToField.y);

                if (std::isfinite(candidateTime) &&
                    robotDistance / reachSpeed +
                            safetyTime <=
                        candidateTime)
                {
                    targetField.x =
                        candidateX;

                    targetField.y =
                        candidateY;

                    targetTime =
                        candidateTime;

                    forwardInterceptActive =
                        true;

                    break;
                }
            }


            // For a centered shot, do not leave the goalkeeper waiting on the
            // fixed line solely because the conservative reach model did not
            // find a candidate.
            if (frontIntercept &&
                !forwardInterceptActive &&
                maxForward > 0.0)
            {
                const double frontMinimum =
                    std::max(
                        0.0,
                        brain->get_parameter(
                            "goalkeeper.prediction.intercept.front_min_forward_distance"
                        ).as_double());

                const double forward =
                    std::min(
                        maxForward,
                        frontMinimum);

                if (forward > 0.0)
                {
                    const double candidateX =
                        targetField.x +
                        forward;

                    const double ratio =
                        std::clamp(
                            (candidateX - ballX) /
                                dxToLine,
                            0.0,
                            1.0);

                    const double candidateY =
                        ballY +
                        ratio * dyToLine;

                    const double candidateDistance =
                        std::hypot(
                            candidateX - ballX,
                            candidateY - ballY);

                    const double candidateTime =
                        ballTravelTime(
                            candidateDistance);

                    targetField.x =
                        candidateX;

                    targetField.y =
                        candidateY;

                    if (std::isfinite(candidateTime))
                    {
                        targetTime =
                            candidateTime;
                    }

                    forwardInterceptActive =
                        true;
                }
            }
        }


        // ---------------------------------------------------------------------
        // Store the result of the FRESH prediction so it can be reused during
        // a temporary loss of the trajectory fit.
        // ---------------------------------------------------------------------
        if (freezeDuringHold &&
            currentThreat)
        {
            cachedTargetField_ =
                targetField;

            cachedTargetValid_ =
                true;

            cachedForwardIntercept_ =
                forwardInterceptActive;

            cachedFrontIntercept_ =
                forwardInterceptActive &&
                frontIntercept;

            cachedReachSpeed_ =
                reachSpeed;

            cachedDeadline_ =
                now +
                rclcpp::Duration::from_seconds(
                    std::max(
                        0.0,
                        targetTime));
        }
    }
    
    brain->data->goalkeeperForwardInterceptActive = forwardInterceptActive;
    brain->data->goalkeeperFrontIntercept =
        forwardInterceptActive && frontIntercept;
    brain->data->goalkeeperAdaptiveReachSpeed = reachSpeed;
    brain->data->goalkeeperBlockTargetFieldX = targetField.x;
    brain->data->goalkeeperBlockTargetFieldY = targetField.y;
    brain->data->goalkeeperBlockTargetTime = targetTime;

    const Pose2D targetRobot = brain->data->field2robot(targetField);
    brain->data->goalkeeperBlockTargetRobotX = targetRobot.x;
    brain->data->goalkeeperBlockTargetRobotY = targetRobot.y;
    const double gain = std::max(
        0.1, brain->get_parameter(
            "goalkeeper.prediction.block.position_gain").as_double());
    double vxLimit = std::max(
        0.0, brain->get_parameter(
            "goalkeeper.prediction.block.vx_limit").as_double());
    if (forwardInterceptActive) {
        vxLimit = std::max(0.0, brain->get_parameter(
            frontIntercept
                ? "goalkeeper.prediction.intercept.front_vx_limit"
                : "goalkeeper.prediction.intercept.diagonal_vx_limit").as_double());
    }
    const double vyLimit = std::max(
        0.0, brain->get_parameter(
            "goalkeeper.prediction.block.vy_limit").as_double());
    const double vthetaLimit = std::max(
        0.0, brain->get_parameter(
            "goalkeeper.prediction.block.vtheta_limit").as_double());
    const double tolerance = std::max(
        0.02, brain->get_parameter(
            "goalkeeper.prediction.block.target_tolerance").as_double());
    const double reactionMargin = std::max(
        0.0, brain->get_parameter(
            "goalkeeper.prediction.block.reaction_margin_sec").as_double());

    const double distance = std::hypot(targetRobot.x, targetRobot.y);
    double vx = distance < tolerance ? 0.0 :
        std::clamp(gain * targetRobot.x, -vxLimit, vxLimit);
    double vy = distance < tolerance ? 0.0 :
        std::clamp(gain * targetRobot.y, -vyLimit, vyLimit);
    const double ballYaw = std::isfinite(brain->data->ball.yawToRobot)
        ? brain->data->ball.yawToRobot : 0.0;
    double vtheta = std::clamp(2.0 * ballYaw,
                               -vthetaLimit, vthetaLimit);

    // If the available time is shorter than the nominal lateral travel time,
    // saturate the lateral command immediately instead of approaching softly.
    const double availableTime = std::max(
        0.01, targetTime - reactionMargin);
    if (forwardInterceptActive &&
        std::abs(targetRobot.x) / availableTime > std::abs(vx) + 1e-6) {
        vx = std::copysign(vxLimit, targetRobot.x);
    }
    if (forwardInterceptActive && frontIntercept &&
        std::abs(targetRobot.x) >= tolerance) {
        vx = std::copysign(vxLimit, targetRobot.x);
    }
    if (std::abs(targetRobot.y) / availableTime > std::abs(vy) + 1e-6) {
        vy = std::copysign(vyLimit, targetRobot.y);
    }

    const double urgentTime = std::max(
        0.0, brain->get_parameter(
            "goalkeeper.prediction.block.urgent_time_sec").as_double());
    const double urgentLateralError = std::max(
        0.02, brain->get_parameter(
            "goalkeeper.prediction.block.urgent_lateral_error").as_double());
    const bool urgentBlock =
        targetTime <= urgentTime &&
        std::abs(targetRobot.y) >= urgentLateralError;
    brain->data->goalkeeperUrgentBlock = urgentBlock;
    if (urgentBlock) {
        const double urgentVxLimit = std::max(
            0.0, brain->get_parameter(
                "goalkeeper.prediction.block.urgent_vx_limit").as_double());
        const double urgentVyLimit = std::max(
            0.0, brain->get_parameter(
                "goalkeeper.prediction.block.urgent_vy_limit").as_double());
        const double urgentVthetaLimit = std::max(
            0.0, brain->get_parameter(
                "goalkeeper.prediction.block.urgent_vtheta_limit").as_double());
        vx = std::clamp(vx, -urgentVxLimit, urgentVxLimit);
        vy = std::copysign(urgentVyLimit, targetRobot.y);
        vtheta = std::clamp(vtheta, -urgentVthetaLimit, urgentVthetaLimit);
    }

    const bool applyMinimum = brain->get_parameter(
        "goalkeeper.prediction.block.apply_min_velocity").as_bool();
    // The predictive blocker must preserve its longitudinal and yaw limits.
    // Applying the global dead-zone floors to all axes used to turn a small
    // correction (for example urgent vx <= 0.15 m/s) into vx=0.40 m/s and
    // similarly inflated yaw.  That made an intended lateral block diagonal
    // and consumed gait capacity needed by vy.  Keep dead-zone compensation
    // only on the lateral axis, whose command performs the interception.
    brain->client->setVelocity(
        vx, vy, vtheta, false, applyMinimum, false);
    return NodeStatus::SUCCESS;
}

NodeStatus Assist::tick() {
    const bool logAssistDebug = brain->log->shouldLog(
        "assist_debug", brain->config->rerunLogDebugHz);
    auto log = [=](string msg) {
        if (logAssistDebug) {
            brain->log->setTimeNow();
            brain->log->log("debug/Assist", rerun::TextLog(msg));
        }
    };
    log("ticked");

    std::unique_lock<std::mutex> cooperationLock(
        brain->data->cooperationMutex);

    double distTolerance = getInput<double>("dist_tolerance").value();
    double thetaTolerance = getInput<double>("theta_tolerance").value();
    double vxLimit, vyLimit;
    getInput("vx_limit", vxLimit);
    getInput("vy_limit", vyLimit);

    const FreeKickPlanState &freeKickPlan = brain->data->freeKickPlan;
    const bool movingToIndirectReceivePoint =
        freeKickPlan.ourRestart &&
        freeKickPlan.type == freekick_policy::Type::Indirect &&
        (freeKickPlan.phase == freekick_policy::Phase::Released ||
         freeKickPlan.phase == freekick_policy::Phase::FirstTouchKicking) &&
        freeKickPlan.receiverId == brain->config->playerId;
    if (movingToIndirectReceivePoint) {
        Pose2D receiveTarget{
            freeKickPlan.passTarget.x,
            freeKickPlan.passTarget.y,
            std::atan2(
                freeKickPlan.ballSnapshot.y - freeKickPlan.passTarget.y,
                freeKickPlan.ballSnapshot.x - freeKickPlan.passTarget.x),
        };
        const Pose2D robotPose = brain->data->robotPoseToField;
        const double receiveDistance = std::hypot(
            receiveTarget.x - robotPose.x,
            receiveTarget.y - robotPose.y);
        const double receiveAngleError = std::fabs(toPInPI(
            receiveTarget.theta - robotPose.theta));
        cooperationLock.unlock();
        if (receiveDistance < distTolerance &&
            receiveAngleError < thetaTolerance) {
            brain->client->setVelocity(0.0, 0.0, 0.0);
            return NodeStatus::SUCCESS;
        }
        bool avoidObstacle = true;
        brain->get_parameter(
            "strategy.cooperation.assist_avoid_obstacles", avoidObstacle);
        brain->client->moveToPoseOnField3(
            receiveTarget.x,
            receiveTarget.y,
            receiveTarget.theta,
            1.5,
            0.6,
            vxLimit,
            vyLimit,
            brain->config->vthetaLimit,
            0.2,
            0.2,
            thetaTolerance,
            avoidObstacle);
        return NodeStatus::SUCCESS;
    }

    if (!brain->data->tmAssistTargetValid ||
        brain->data->tmMyAssistSlot == AssistSlot::NONE) {
        cooperationLock.unlock();
        brain->client->setVelocity(0.0, 0.0, 0.0);
        log("no valid semantic assist slot");
        return NodeStatus::SUCCESS;
    }

    const auto fd = brain->config->fieldDimensions;
    const auto ballPos = brain->data->tmFormationBall;
    const auto robotPose = brain->data->robotPoseToField;
    const Pose2D slotTarget = brain->data->tmAssistTarget;

    const Pose2D leaderPose = brain->data->tmFormationLeaderPose;
    double leaderKickDir = brain->data->tmFormationKickDir;
    if (!std::isfinite(leaderKickDir)) leaderKickDir = 0.0;

    const AssistPoint robotPoint{robotPose.x, robotPose.y};
    const AssistPoint ballPoint{ballPos.x, ballPos.y};
    const AssistPoint leaderPoint{leaderPose.x, leaderPose.y};
    const AssistPoint kickUnit{
        std::cos(leaderKickDir),
        std::sin(leaderKickDir),
    };
    const AssistPoint kickNormal{-kickUnit.y, kickUnit.x};
    const AssistPoint kickEnd = ballPoint + kickUnit * 4.5;
    vector<ProtectedLane> protectedLanes{
        {
            ballPoint - kickUnit * 0.05,
            ballPoint + kickUnit * 0.05,
            1.3,
        },
        {leaderPoint, ballPoint, 0.9},
        {ballPoint, kickEnd, 0.85},
    };

    const double teammatePositionClearance = std::max(
        0.0,
        brain->get_parameter(
            "strategy.cooperation.assist_teammate_position_clearance")
            .as_double());
    const double teammatePathClearance = std::max(
        0.0,
        brain->get_parameter(
            "strategy.cooperation.assist_teammate_path_clearance")
            .as_double());
    const double tacticalPacketTimeoutMs = std::max(
        100.0,
        brain->get_parameter(
            "strategy.cooperation.tactical_packet_timeout_ms").as_double());
    std::array<TMStatus, MAX_NUM_PLAYERS> teamStatuses{};
    {
        std::lock_guard<std::mutex> teamStatusLock(
            brain->data->teamStatusMutex);
        std::copy(
            std::begin(brain->data->tmStatus),
            std::end(brain->data->tmStatus),
            teamStatuses.begin());
    }
    const auto myPolicySlot = static_cast<assist_strategy_policy::Slot>(
        static_cast<std::size_t>(brain->data->tmMyAssistSlot));
    const bool myYielding =
        brain->data->tmAssistPhase == AssistPhase::YIELD_LANE;
    const int selfId = brain->config->playerId;
    for (int teammateIdx = 0; teammateIdx < MAX_NUM_PLAYERS;
         ++teammateIdx) {
        const int teammateId = teammateIdx + 1;
        if (teammateId == selfId) continue;
        const TMStatus &status = teamStatuses[teammateIdx];
        if (brain->data->penalty[teammateIdx] != PENALTY_NONE ||
            !status.isAlive || status.role != "striker" ||
            brain->msecsSince(status.timeLastCom) > tacticalPacketTimeoutMs) {
            continue;
        }
        const auto teammatePolicySlot =
            static_cast<assist_strategy_policy::Slot>(
                static_cast<std::size_t>(status.assistSlot));
        if (!assist_strategy_policy::teammateHasRightOfWay(
                myPolicySlot,
                myYielding,
                selfId,
                teammatePolicySlot,
                status.assistPhase == AssistPhase::YIELD_LANE,
                teammateId)) {
            continue;
        }

        const AssistPoint teammatePoint{
            status.robotPoseToField.x,
            status.robotPoseToField.y,
        };
        if (!std::isfinite(teammatePoint.x) ||
            !std::isfinite(teammatePoint.y)) {
            continue;
        }
        protectedLanes.push_back({
            teammatePoint,
            teammatePoint,
            teammatePositionClearance,
        });

        const AssistPoint teammateTarget{
            status.assistTarget.x,
            status.assistTarget.y,
        };
        const bool targetInsideField =
            std::isfinite(teammateTarget.x) &&
            std::isfinite(teammateTarget.y) &&
            std::fabs(teammateTarget.x) <= fd.length / 2.0 + 0.5 &&
            std::fabs(teammateTarget.y) <= fd.width / 2.0 + 0.5;
        if (targetInsideField &&
            assistPointNorm(teammateTarget - teammatePoint) > 0.05) {
            protectedLanes.push_back({
                teammatePoint,
                teammateTarget,
                teammatePathClearance,
            });
        }
    }

    const double configuredYieldTimeoutMs = std::max(
        0.0,
        brain->get_parameter(
            "strategy.cooperation.assist_yield_timeout_ms").as_double());
    const double waypointLockMs = std::max(
        0.0,
        brain->get_parameter(
            "strategy.cooperation.assist_waypoint_lock_ms").as_double());
    const double waypointTtlMs = std::max(
        waypointLockMs,
        brain->get_parameter(
            "strategy.cooperation.assist_waypoint_ttl_ms").as_double());
    const double waypointNoProgressMs = std::max(
        100.0,
        brain->get_parameter(
            "strategy.cooperation.assist_waypoint_no_progress_ms").as_double());

    bool yielding =
        brain->data->tmAssistPhase == AssistPhase::YIELD_LANE;
    Pose2D navigationTarget = slotTarget;
    if (yielding) {
        const int side = brain->data->tmAssistYieldSide == 0
            ? (brain->config->playerId % 2 == 0 ? 1 : -1)
            : brain->data->tmAssistYieldSide;
        const AssistPoint yieldPoint =
            ballPoint - kickUnit * 1.0 + kickNormal * (side * 1.8);
        navigationTarget = clampAssistWaypoint(
            {
                yieldPoint.x,
                yieldPoint.y,
                std::atan2(
                    ballPos.y - yieldPoint.y,
                    ballPos.x - yieldPoint.x),
            },
            fd);
        constexpr double yieldClearanceMargin = 0.12;
        const bool laneCleared = pointSafelyClearsLanes(
            robotPoint, protectedLanes, yieldClearanceMargin);
        if (laneCleared) {
            brain->data->tmAssistPhase = AssistPhase::TRANSIT_SLOT;
            brain->data->tmAssistPhaseChangedAt = AssistClock::now();
            brain->data->tmAssistYieldTimeoutMs = 0.0;
            brain->data->tmAssistYieldRouteExtended = false;
            brain->data->tmAssistWaypointValid = false;
            navigationTarget = slotTarget;
            yielding = false;
            log("former leader cleared every protected lane");
        }
    }

    int targetPreferredSide = brain->data->tmAssistYieldSide;
    if (targetPreferredSide == 0) {
        const double robotSide = assistCross(
            kickUnit, robotPoint - ballPoint);
        targetPreferredSide = std::fabs(robotSide) > 1e-6
            ? (robotSide > 0.0 ? 1 : -1)
            : (brain->config->playerId % 2 == 0 ? 1 : -1);
    }
    auto findLegalNavigationTarget = [&](const Pose2D &requestedTarget) {
        return findNearestLegalAssistTarget(
            requestedTarget,
            protectedLanes,
            kickUnit,
            ballPoint,
            targetPreferredSide,
            fd);
    };
    Pose2D requestedNavigationTarget = navigationTarget;
    auto legalNavigationTarget = findLegalNavigationTarget(
        requestedNavigationTarget);

    if (yielding) {
        const Pose2D timeoutTarget = legalNavigationTarget.has_value()
            ? *legalNavigationTarget
            : requestedNavigationTarget;
        const double yieldDistance = std::hypot(
            robotPose.x - timeoutTarget.x,
            robotPose.y - timeoutTarget.y);
        if (!(brain->data->tmAssistYieldTimeoutMs > 0.0) ||
            !std::isfinite(brain->data->tmAssistYieldTimeoutMs)) {
            brain->data->tmAssistPhaseChangedAt = AssistClock::now();
            brain->data->tmAssistYieldRouteExtended = false;
            brain->data->tmAssistYieldTimeoutMs =
                assist_strategy_policy::yieldTimeoutMsecs(
                    yieldDistance, 0.5, configuredYieldTimeoutMs);
        }

        if (assistElapsedMsecs(brain->data->tmAssistPhaseChangedAt) >=
            brain->data->tmAssistYieldTimeoutMs) {
            brain->data->tmAssistPhase = AssistPhase::TRANSIT_SLOT;
            brain->data->tmAssistPhaseChangedAt = AssistClock::now();
            brain->data->tmAssistYieldTimeoutMs = 0.0;
            brain->data->tmAssistYieldRouteExtended = false;
            brain->data->tmAssistWaypointValid = false;
            requestedNavigationTarget = slotTarget;
            legalNavigationTarget = findLegalNavigationTarget(
                requestedNavigationTarget);
            yielding = false;
            log("former leader yield timed out; protected routing remains active");
        }
    }

    if (!legalNavigationTarget.has_value()) {
        brain->data->tmAssistWaypointValid = false;
        brain->data->tmAssistWaypointFailureCount =
            std::min(brain->data->tmAssistWaypointFailureCount, 11) + 1;
        brain->data->tmAssistYieldSide = -targetPreferredSide;
        cooperationLock.unlock();
        brain->client->setVelocity(0.0, 0.0, 0.0);
        log("no legal temporary assist target; holding position");
        return NodeStatus::SUCCESS;
    }
    navigationTarget = *legalNavigationTarget;
    const bool navigationTargetAdjusted = std::hypot(
        navigationTarget.x - requestedNavigationTarget.x,
        navigationTarget.y - requestedNavigationTarget.y) > 1e-4;

    const AssistPoint navigationPoint{
        navigationTarget.x,
        navigationTarget.y,
    };
    const double navigationDistance = assistPointNorm(
        navigationPoint - robotPoint);
    if (!yielding &&
        brain->data->tmAssistPhase == AssistPhase::HOLD &&
        navigationDistance > distTolerance * 1.5) {
        brain->data->tmAssistPhase = AssistPhase::TRANSIT_SLOT;
        brain->data->tmAssistPhaseChangedAt = AssistClock::now();
        brain->data->tmAssistYieldTimeoutMs = 0.0;
        brain->data->tmAssistYieldRouteExtended = false;
    }

    const bool directPathSafe = pathSafelyClearsLanes(
        robotPoint, navigationPoint, protectedLanes);
    if (brain->data->tmAssistWaypointValid) {
        const Pose2D waypoint = brain->data->tmAssistWaypoint;
        const AssistPoint waypointPoint{waypoint.x, waypoint.y};
        const double waypointDistance = std::hypot(
            robotPose.x - waypoint.x,
            robotPose.y - waypoint.y);
        if (waypointDistance + 0.05 <
            brain->data->tmAssistWaypointBestDistance) {
            brain->data->tmAssistWaypointBestDistance = waypointDistance;
            brain->data->tmAssistWaypointLastProgressAt = AssistClock::now();
        }
        const bool waypointExpired =
            assistElapsedMsecs(brain->data->tmAssistWaypointLockedAt) >=
                waypointTtlMs;
        const bool waypointStalled =
            assistElapsedMsecs(brain->data->tmAssistWaypointLockedAt) >=
                waypointLockMs &&
            assistElapsedMsecs(
                brain->data->tmAssistWaypointLastProgressAt) >=
                waypointNoProgressMs;
        const bool waypointStillSafe =
            pathSafelyClearsLanes(
                robotPoint, waypointPoint, protectedLanes) &&
            pathSafelyClearsLanes(
                waypointPoint, navigationPoint, protectedLanes);
        if (directPathSafe) {
            brain->data->tmAssistWaypointValid = false;
            if (!yielding) {
                brain->data->tmAssistWaypointFailureCount = 0;
                brain->data->tmAssistYieldSide = 0;
            }
        } else if (waypointExpired || waypointStalled ||
                   !waypointStillSafe) {
            brain->data->tmAssistWaypointValid = false;
            brain->data->tmAssistWaypointFailureCount =
                std::min(
                    brain->data->tmAssistWaypointFailureCount,
                    11) + 1;
            brain->data->tmAssistYieldSide =
                brain->data->tmAssistYieldSide == 0
                ? (brain->config->playerId % 2 == 0 ? -1 : 1)
                : -brain->data->tmAssistYieldSide;
            log("assist waypoint invalid; alternate route immediately");
        }
    }

    bool pathBlockedWithoutWaypoint = false;
    if (!brain->data->tmAssistWaypointValid && !directPathSafe) {
        int preferredSide = brain->data->tmAssistYieldSide;
        if (preferredSide == 0) {
            const double robotSide = assistCross(
                kickUnit, robotPoint - ballPoint);
            if (std::fabs(robotSide) > 1e-6) {
                preferredSide = robotSide > 0.0 ? 1 : -1;
            } else {
                preferredSide =
                    brain->config->playerId % 2 == 0 ? 1 : -1;
            }
        }
        const auto waypointChoice = chooseLaneWaypoint(
            robotPose,
            navigationTarget,
            protectedLanes,
            kickUnit,
            ballPoint,
            0.6,
            preferredSide,
            brain->data->tmAssistWaypointFailureCount,
            fd);
        if (waypointChoice.has_value()) {
            brain->data->tmAssistWaypoint = waypointChoice->waypoint;
            brain->data->tmAssistYieldSide = waypointChoice->kickSide;
            brain->data->tmAssistWaypointValid = true;
            brain->data->tmAssistWaypointLockedAt = AssistClock::now();
            brain->data->tmAssistWaypointLastProgressAt = AssistClock::now();
            brain->data->tmAssistWaypointBestDistance = std::hypot(
                robotPose.x - brain->data->tmAssistWaypoint.x,
                robotPose.y - brain->data->tmAssistWaypoint.y);
            if (!yielding &&
                brain->data->tmAssistPhase == AssistPhase::HOLD) {
                brain->data->tmAssistPhase = AssistPhase::TRANSIT_SLOT;
                brain->data->tmAssistPhaseChangedAt = AssistClock::now();
                brain->data->tmAssistYieldTimeoutMs = 0.0;
                brain->data->tmAssistYieldRouteExtended = false;
            }
            if (yielding &&
                !brain->data->tmAssistYieldRouteExtended) {
                const double remainingRouteDistance =
                    brain->data->tmAssistWaypointBestDistance +
                    std::hypot(
                        navigationTarget.x - brain->data->tmAssistWaypoint.x,
                        navigationTarget.y - brain->data->tmAssistWaypoint.y);
                const double extendedTimeoutMs =
                    assistElapsedMsecs(
                        brain->data->tmAssistPhaseChangedAt) +
                    assist_strategy_policy::yieldTimeoutMsecs(
                        remainingRouteDistance, 0.5, 0.0);
                brain->data->tmAssistYieldTimeoutMs = std::max(
                    brain->data->tmAssistYieldTimeoutMs,
                    extendedTimeoutMs);
                brain->data->tmAssistYieldRouteExtended = true;
            }
            log("protected all leader/ball lanes with a safe waypoint");
        } else {
            pathBlockedWithoutWaypoint = true;
            brain->data->tmAssistWaypointFailureCount =
                std::min(
                    brain->data->tmAssistWaypointFailureCount,
                    11) + 1;
            brain->data->tmAssistYieldSide = -preferredSide;
            log("no waypoint clears every protected lane; holding position");
        }
    } else if (!brain->data->tmAssistWaypointValid &&
               directPathSafe && !yielding) {
        brain->data->tmAssistWaypointFailureCount = 0;
        brain->data->tmAssistYieldSide = 0;
    }

    Pose2D movementTarget = brain->data->tmAssistWaypointValid
        ? brain->data->tmAssistWaypoint
        : navigationTarget;
    movementTarget.theta = std::atan2(
        ballPos.y - movementTarget.y,
        ballPos.x - movementTarget.x);

    const double targetDistance = navigationDistance;
    const double targetAngleError = std::fabs(toPInPI(
        navigationTarget.theta - robotPose.theta));
    const bool slotReached =
        !yielding &&
        !brain->data->tmAssistWaypointValid &&
        !pathBlockedWithoutWaypoint &&
        targetDistance < distTolerance &&
        targetAngleError < thetaTolerance;
    if (slotReached) {
        brain->data->tmAssistPhase = AssistPhase::HOLD;
        brain->data->tmAssistPhaseChangedAt = AssistClock::now();
        brain->data->tmAssistYieldTimeoutMs = 0.0;
        brain->data->tmAssistYieldRouteExtended = false;
    }

    const bool waypointValidForLog =
        brain->data->tmAssistWaypointValid;
    const Pose2D waypointForLog = brain->data->tmAssistWaypoint;
    const AssistSlot slotForLog = brain->data->tmMyAssistSlot;
    const AssistPhase phaseForLog = brain->data->tmAssistPhase;
    cooperationLock.unlock();

    if (navigationTargetAdjusted) {
        log("assist target projected outside protected lanes");
    }

    if (slotReached || pathBlockedWithoutWaypoint) {
        brain->client->setVelocity(0.0, 0.0, 0.0);
        return NodeStatus::SUCCESS;
    }

    bool avoidObstacle = true;
    brain->get_parameter(
        "strategy.cooperation.assist_avoid_obstacles", avoidObstacle);
    brain->client->moveToPoseOnField3(
        movementTarget.x,
        movementTarget.y,
        movementTarget.theta,
        1.5,
        0.6,
        vxLimit,
        vyLimit,
        brain->config->vthetaLimit,
        0.2,
        0.2,
        thetaTolerance,
        avoidObstacle);

    if (brain->log->shouldLog(
            "assist_waypoint_visual", brain->config->rerunLogVisualHz) &&
        waypointValidForLog) {
        brain->log->setTimeNow();
        brain->log->logRobot(
            "field/assist_waypoint",
            waypointForLog,
            0xFFCC00FF,
            assistPhaseName(phaseForLog));
    }
    log(format(
        "slot=%s phase=%s target=(%.2f, %.2f) waypoint=%d",
        assistSlotName(slotForLog),
        assistPhaseName(phaseForLog),
        slotTarget.x,
        slotTarget.y,
        waypointValidForLog));
    return NodeStatus::SUCCESS;
}


NodeStatus Adjust::tick()
{
    if (!brain->tree->getEntry<bool>("ball_location_known"))
    {
        return NodeStatus::SUCCESS;
    }

    double turnThreshold, vxLimit, vyLimit, vthetaLimit, range;
    double stFar, stNear, vthetaFactor, nearThreshold, noTurnThreshold, turnFirstThreshold;
    getInput("turn_threshold", turnThreshold);
    getInput("vx_limit", vxLimit);
    getInput("vy_limit", vyLimit);
    getInput("vtheta_limit", vthetaLimit);
    getInput("range", range);
    getInput("tangential_speed_far", stFar);
    getInput("tangential_speed_near", stNear);
    getInput("vtheta_factor", vthetaFactor);
    getInput("near_threshold", nearThreshold);
    getInput("no_turn_threshold", noTurnThreshold);
    turnFirstThreshold = turnThreshold;
    getInput("turn_first_threshold", turnFirstThreshold);
    string position;
    getInput("position", position);
    if (brain->tree->getEntry<string>("player_role") == "goal_keeper") {
        turnThreshold = brain->get_parameter(
            "goalkeeper.adjust.turn_threshold").as_double();
        vxLimit = brain->get_parameter(
            "goalkeeper.adjust.vx_limit").as_double();
        vyLimit = brain->get_parameter(
            "goalkeeper.adjust.vy_limit").as_double();
        vthetaLimit = brain->get_parameter(
            "goalkeeper.adjust.vtheta_limit").as_double();
        range = brain->get_parameter("goalkeeper.adjust.range").as_double();
    }

    double vx = 0, vy = 0, vtheta = 0;
    double kickDir = (position == "defense") ?
        atan2(brain->data->ball.posToField.y, brain->data->ball.posToField.x + brain->config->fieldDimensions.length / 2)
        : brain->data->kickDir;
    double dir_rb_f = brain->data->robotBallAngleToField;
    double deltaDir = toPInPI(kickDir - dir_rb_f);
    double ballRange = brain->data->ball.range;
    double ballYaw = brain->data->ball.yawToRobot;

    double st = stFar;
    if (fabs(deltaDir) * ballRange < nearThreshold) {
        st = stNear;
    }

    double thetaRobotField = brain->data->robotPoseToField.theta;
    double tangentialDirRobot = dir_rb_f + M_PI / 2 * (deltaDir > 0 ? -1.0 : 1.0) - thetaRobotField;
    double radialDirRobot = dir_rb_f - thetaRobotField;
    double sr = cap(ballRange - range, 0.5, 0.0);

    vx = st * cos(tangentialDirRobot) + sr * cos(radialDirRobot);
    vy = st * sin(tangentialDirRobot) + sr * sin(radialDirRobot);
    vtheta = ballYaw * vthetaFactor;

    if (fabs(ballYaw) < noTurnThreshold) {
        vtheta = 0.0;
    }
    if (fabs(ballYaw) > turnFirstThreshold && fabs(deltaDir) < M_PI / 4) {
        vx = 0;
        vy = 0;
    }

    vx = cap(vx, vxLimit, -0.0);
    vy = cap(vy, vyLimit, -vyLimit);
    vtheta = cap(vtheta, vthetaLimit, -vthetaLimit);

    brain->client->setVelocity(vx, vy, vtheta, false, true, false);
    return NodeStatus::SUCCESS;
}

NodeStatus CalcKickDir::tick()
{
    // Read and process parameters.
    double crossThreshold;
    getInput("cross_threshold", crossThreshold);

    string lastKickType = brain->data->kickType;
    if (lastKickType == "cross") crossThreshold += 0.1; // Add hysteresis.

    auto gpAngles = brain->getGoalPostAngles(0.0);
    auto thetal = gpAngles[0]; auto thetar = gpAngles[1];
    auto fd = brain->config->fieldDimensions;
    auto color = 0xFFFFFFFF; // for log

    FreeKickPlanState freeKickPlan;
    {
        std::lock_guard<std::mutex> cooperationLock(
            brain->data->cooperationMutex);
        freeKickPlan = brain->data->freeKickPlan;
    }
    const bool plannedActor = brain->isPlannedFreeKickActor();
    const bool frozenFreeKickBall =
        freeKickPlan.ourRestart &&
        freeKickPlan.phase >= freekick_policy::Phase::Preparing &&
        freeKickPlan.phase < freekick_policy::Phase::Complete &&
        std::isfinite(freeKickPlan.ballSnapshot.x) &&
        std::isfinite(freeKickPlan.ballSnapshot.y);
    const auto bPos = frozenFreeKickBall
        ? freeKickPlan.ballSnapshot
        : brain->data->ball.posToField;
    const bool plannedFirstIndirectTouch =
        plannedActor &&
        freeKickPlan.type == freekick_policy::Type::Indirect &&
        (freeKickPlan.phase == freekick_policy::Phase::Released ||
         freeKickPlan.phase == freekick_policy::Phase::FirstTouchKicking);
    const bool plannedShot =
        plannedActor && !plannedFirstIndirectTouch;

    if (plannedFirstIndirectTouch) {
        brain->data->kickType = "cross";
        color = 0xFF00FFFF;
        brain->data->kickDir = std::atan2(
            freeKickPlan.passTarget.y - bPos.y,
            freeKickPlan.passTarget.x - bPos.x);
    } else if (plannedShot) {
        brain->data->kickType = "shoot";
        color = 0x00FF00FF;
        brain->data->kickDir = std::atan2(-bPos.y, fd.length / 2.0 - bPos.x);
    } else if (thetal - thetar < crossThreshold && brain->data->ball.posToField.x > fd.circleRadius) {
        brain->data->kickType = "cross";
        color = 0xFF00FFFF;
        brain->data->kickDir = atan2(
            - bPos.y,
            fd.length/2 - fd.penaltyDist/2 - bPos.x
        );
    } else if (brain->isDefensing()) {
        brain->data->kickType = "block";
        color = 0xFFFF00FF;
        brain->data->kickDir = atan2(
            bPos.y,
            bPos.x + fd.length/2
        );

    } else { // default to shoot
        brain->data->kickType = "shoot";
        color = 0x00FF00FF;
        brain->data->kickDir = atan2(
            - bPos.y,
            fd.length/2 - bPos.x
        );
        if (brain->data->ball.posToField.x > brain->config->fieldDimensions.length / 2) brain->data->kickDir = 0; // Continue forward after crossing the line.
    }

    if (brain->log->shouldLog(
            "kick_direction_visual", brain->config->rerunLogVisualHz)) {
        brain->log->setTimeNow();
        brain->log->log(
            "field/kick_dir",
            rerun::Arrows2D::from_vectors({{10 * cos(brain->data->kickDir), -10 * sin(brain->data->kickDir)}})
                .with_origins({{brain->data->ball.posToField.x, -brain->data->ball.posToField.y}})
                .with_colors({color})
                .with_radii(0.01)
                .with_draw_order(31)
        );
    }

    return NodeStatus::SUCCESS;
}

NodeStatus StrikerDecide::tick() {
    const bool logDecisionDebug = brain->log->shouldLog(
        "striker_decide_debug", brain->config->rerunLogDebugHz);
    auto log = [=](string msg) {
        if (logDecisionDebug) {
            brain->log->setTimeNow();
            brain->log->log("debug/striker_decide", rerun::TextLog(msg));
        }
    };
    // Read and process parameters.
    bool enableBypass, enableShoot, enableDirectionalKick, kickAOUseShoot, enablePowerShoot, usePowerShootForKickoff;
    brain->get_parameter("strategy.power_shoot.enable", enablePowerShoot);
    brain->get_parameter("strategy.power_shoot.use_for_kickoff", usePowerShootForKickoff);
    brain->get_parameter("strategy.enable_shoot", enableShoot);
    brain->get_parameter("strategy.enable_bypass", enableBypass);
    brain->get_parameter("strategy.enable_directional_kick", enableDirectionalKick);
    brain->get_parameter("obstacle_avoidance.kick_ao_use_shoot", kickAOUseShoot);
    double KICK_RANGE = 1.0;
    brain->get_parameter("strategy.kick_range", KICK_RANGE);
    double KICK_THETA_RANGE = 1.5;
    brain->get_parameter("strategy.kick_theta_range", KICK_THETA_RANGE);

    const double bypassThreshold = 0.5;
    double chaseRangeThreshold;
    getInput("chase_threshold", chaseRangeThreshold);
    string lastDecision, position;
    getInput("decision_in", lastDecision);
    getInput("position", position);

    bool enableAutoVisualKick = false;
    brain->get_parameter("strategy.enable_auto_visual_kick", enableAutoVisualKick);
    double autoVisualKickEnableDistMin = 0.2;
    double autoVisualKickEnableDistMax = 4.0;
    double autoVisualKickEnableAngle = 1.2217304763960306;
    brain->get_parameter("strategy.auto_visual_kick_enable_dist_min", autoVisualKickEnableDistMin);
    brain->get_parameter("strategy.auto_visual_kick_enable_dist_max", autoVisualKickEnableDistMax);
    brain->get_parameter("strategy.auto_visual_kick_enable_angle", autoVisualKickEnableAngle);

    double kickDir = brain->data->kickDir;
    double dir_rb_f = brain->data->robotBallAngleToField; // Field-frame robot-to-ball direction
    auto ball = brain->data->ball;
    double ballRange = ball.range;
    double ballYaw = ball.yawToRobot;
    double ballX = ball.posToRobot.x;
    double ballY = ball.posToRobot.y;
     

    const double goalpostMargin = 0.3; // Goalpost clearance used for angle calculation
    bool angleGoodForKick = brain->isAngleGood(goalpostMargin, "kick");
    bool angleGoodForShoot = brain->isAngleGood(goalpostMargin, "shoot");
    bool angleGoodForDirectionalKick = brain->isAngleGoodForDirectionalKick(goalpostMargin);

    const double SHOOT_Y_RANGE = 0.3;
    const double SHOOT_X_MAX = 0.6;
    const double SHOOT_X_MIN = 0.2;
    const double STRONG_SHOOT_X_MIN = 1.0;
    bool shootPossible = angleGoodForShoot && fabs(ballY) < SHOOT_Y_RANGE && ballX < SHOOT_X_MAX && ballX > SHOOT_X_MIN;
    bool directionalKickPossible = angleGoodForDirectionalKick && fabs(ballY) < SHOOT_Y_RANGE && ballX < SHOOT_X_MAX && ballX > SHOOT_X_MIN;
    bool useStrongShoot = shootPossible && fabs(ballX) > STRONG_SHOOT_X_MIN; // TODO now impossible to be true


    bool avoidPushing;
    double kickAoSafeDist;
    brain->get_parameter("obstacle_avoidance.avoid_during_kick", avoidPushing);
    brain->get_parameter("obstacle_avoidance.kick_ao_safe_dist", kickAoSafeDist);
    bool avoidKick = avoidPushing // Whether to avoid collision while kicking
        && brain->data->robotPoseToField.x < brain->config->fieldDimensions.length / 2 - brain->config->fieldDimensions.goalAreaLength
        && brain->distToObstacle(brain->data->ball.yawToRobot) < kickAoSafeDist;

    log(format("ballRange: %.2f, ballYaw: %.2f, ballX:%.2f, ballY: %.2f kickDir: %.2f, dir_rb_f: %.2f, angleGoodForKick: %d, angleGoodForShoot: %d, shootPossible: %d, strongShoot: %d",
        ballRange, ballYaw, ballX, ballY, kickDir, dir_rb_f, angleGoodForKick, angleGoodForShoot, shootPossible, useStrongShoot));

    // Determine whether the robot crossed KickDir.
    double deltaDir = toPInPI(kickDir - dir_rb_f);
    auto now = brain->get_clock()->now();
    auto dt = brain->msecsSince(timeLastTick);
    bool reachedKickDir = 
        deltaDir * lastDeltaDir <= 0 
        && fabs(deltaDir) < M_PI / 6
        && dt < 100;
    reachedKickDir = reachedKickDir || fabs(deltaDir) < 0.1;
    timeLastTick = now;
    lastDeltaDir = deltaDir;

    double kickValue = brain->kickValue(dir_rb_f);
    double threatLevel = brain->threatLevel();
    log(format("kickValue: %.1f, threatLevel: %.1f", kickValue, threatLevel));
     

    string newDecision;
    auto color = 0xFFFFFFFF; // for log
    bool iKnowBallPos = brain->tree->getEntry<bool>("ball_location_known");
    bool tmBallPosReliable = brain->tree->getEntry<bool>("tm_ball_pos_reliable");
    const bool freeKickExecution = brain->isOwnFreeKickExecutionActive();
    const bool plannedFreeKickActor = brain->isPlannedFreeKickActor();
    const bool fallenRobotBlocksVisualKick =
        brain->isFallenRobotVisualKickExitEnabled() &&
        brain->hasFallenRobotInVisualKickZone();
    const bool visualKickNormalPlayRegion =
        ball.posToField.x > brain->config->fieldDimensions.length / 2 - 14.3 &&
        fabs(ball.posToField.y) < 5.0 &&
        brain->data->robotPoseToField.x >
            brain->config->fieldDimensions.length / 2 - 14.3 &&
        fabs(brain->data->robotPoseToField.y) < 5.0;
    const bool visualKickNormalPlayEligible =
        brain->data->tmImLead &&
        visualKickNormalPlayRegion;
    const bool visualKickCostEligible = brain->data->tmMyCost < 7.0;
    const bool visualKickContextAllowed =
        freekick_policy::visualKickContextAllowed(
            freeKickExecution,
            plannedFreeKickActor,
            visualKickNormalPlayEligible);
    log(format(
        "visualKick gates: enabled=%d freeKick=%d actor=%d normalEligible=%d "
        "context=%d lead=%d cost=%.2f costEligible=%d range=%.2f yaw=%.2f "
        "region=%d",
        enableAutoVisualKick,
        freeKickExecution,
        plannedFreeKickActor,
        visualKickNormalPlayEligible,
        visualKickContextAllowed,
        brain->data->tmImLead,
        brain->data->tmMyCost,
        visualKickCostEligible,
        ballRange,
        ballYaw,
        visualKickNormalPlayRegion));
    if (freeKickExecution && !plannedFreeKickActor) {
        // During a synchronized restart, non-actors must not start a ball
        // search and accidentally approach the first/second-touch lane.
        newDecision = "assist";
        color = 0x00FFFFFF;
    } else if (!(iKnowBallPos || tmBallPosReliable))
    {
        newDecision = "find";
        color = 0xFFFFFFFF;
    } else if (
        enableAutoVisualKick &&
        !fallenRobotBlocksVisualKick &&
        visualKickContextAllowed &&
        visualKickCostEligible &&
        !brain->tree->getEntry<bool>("ball_out") &&
        !brain->data->lose_ball &&
        ballRange < autoVisualKickEnableDistMax &&
        ballRange > autoVisualKickEnableDistMin &&
        fabs(ballYaw) < autoVisualKickEnableAngle
    ) {
        newDecision = "auto_visual_kick";
        brain->data->tmImInVisualKick = true;
        color = 0xFF00FFFF;
    } else if (!brain->data->tmImLead) {
        newDecision = "assist";
        color = 0x00FFFFFF;
    }
    else if (ballRange > chaseRangeThreshold * (lastDecision == "chase" ? 0.9 : 1.0))
    {
        newDecision = "chase";
        color = 0x0000FFFF;
    } 
    else if (
        (
            (angleGoodForKick && !freeKickExecution)
            || reachedKickDir
        )
        && !avoidKick
        && brain->data->ballDetected
        && fabs(brain->data->ball.yawToRobot) < KICK_THETA_RANGE
        && ball.range < KICK_RANGE
    )
    {
        if (brain->data->kickType == "cross") newDecision = "cross";
        else if (plannedFreeKickActor) newDecision = "kick";
        else { // kickType == kick
            double threatThreshold;
            brain->get_parameter("strategy.shoot.threat_threshold", threatThreshold);
            if (threatLevel < threatThreshold) newDecision = "safe_shoot";
            else newDecision = "kick";
        }        
        color = 0x00FF00FF;
    }
    else
    {
        newDecision = "adjust";
        color = 0xFFFF00FF;
    }

    if (newDecision != "auto_visual_kick") {
        brain->data->tmImInVisualKick = false;
    }

    setOutput("decision_out", newDecision);
    brain->log->logToScreen(
        "tree/Decide",
        format(
            "Decision: %s ballrange: %.2f ballyaw: %.2f kickDir: %.2f rbDir: %.2f angleGoodForKick: %d angleGoodForShoot: %d lead: %d", 
            newDecision.c_str(), ballRange, ballYaw, kickDir, dir_rb_f, angleGoodForKick, angleGoodForShoot, brain->data->tmImLead
        ),
        color
    );

    color = 0xFFFFFFFF;
    if (threatLevel >= 2.0) color = 0xFF0000FF;
    else if (threatLevel >= 1.0) color = 0xFFCC00FF;
    brain->log->logToScreen("tree/value_threat", format("Threat Level: %.1f, Kick Value: %.1f", threatLevel, kickValue), color, 60);
    return NodeStatus::SUCCESS;
}

NodeStatus GoalieDecide::tick()
{
    // Read and process parameters.
    double chaseRangeThreshold = std::max(
        0.0, brain->get_parameter("goalkeeper.chase.threshold").as_double());
    string lastDecision;
    getInput("decision_in", lastDecision);

    double kickDir = atan2(brain->data->ball.posToField.y, brain->data->ball.posToField.x + brain->config->fieldDimensions.length / 2);
    brain->data->kickDir = kickDir;
    double dir_rb_f = brain->data->robotBallAngleToField; // Field-frame robot-to-ball direction
    const double alignmentTolerance = std::clamp(
        brain->get_parameter(
            "goalkeeper.kick.alignment_tolerance").as_double(),
        0.02, M_PI);
    const double alignmentError = toPInPI(kickDir - dir_rb_f);
    bool angleIsGood = std::abs(alignmentError) < alignmentTolerance;
    double ballRange = brain->data->ball.range;
    double ballYaw = brain->data->ball.yawToRobot;
    const bool goalieMayClaimBall = brain->canGoalkeeperClaimBall(
        brain->data->ballDetected,
        ballRange,
        brain->data->ball.posToField.x,
        brain->data->ball.posToField.y,
        brain->data->tmMyCost);
    const bool requireTeamLead = brain->get_parameter(
        "goalkeeper.claim.require_team_lead").as_bool();
    const bool postBlockClearance =
        brain->data->goalkeeperPostBlockClearance &&
        brain->data->ballDetected &&
        ballRange <= brain->get_parameter(
            "goalkeeper.claim.max_ball_range").as_double();

    string newDecision;
    auto color = 0xFFFFFFFF; // for log
    bool iKnowBallPos = brain->tree->getEntry<bool>("ball_location_known");
    bool tmBallPosReliable = brain->tree->getEntry<bool>("tm_ball_pos_reliable");
    const bool incomingShot =
        brain->get_parameter("goalkeeper.prediction.enabled").as_bool() &&
        brain->data->ballWillBreach;
    if (incomingShot)
    {
        // Incoming shots have priority over possession. Chasing the current
        // ball position can pull a goalkeeper out of the predicted goal line.
        newDecision = "block_shot";
        color = 0xFF8800FF;
    }
    else if (!(iKnowBallPos || tmBallPosReliable))
    {
        newDecision = "find";
        color = 0x0000FFFF;
    }
    else if ((!goalieMayClaimBall && !postBlockClearance) ||
             (requireTeamLead && !brain->data->tmImLead &&
              !postBlockClearance))
    {
        newDecision = "retreat";
        color = 0xFF00FFFF;
    }
    else if (ballRange > chaseRangeThreshold * (lastDecision == "chase" ? 0.9 : 1.0))
    {
        newDecision = "chase";
        color = 0x00FF00FF;
    }
    else if (angleIsGood)
    {
        newDecision = "kick";
        color = 0xFF0000FF;
    }
    else
    {
        newDecision = "adjust";
        color = 0x00FFFFFF;
    }

    setOutput("decision_out", newDecision);
    brain->data->goalkeeperDecision = newDecision;
    const string configuredKickType =
        brain->get_parameter("goalkeeper.kick.type").as_string();
    brain->tree->setEntry<string>(
        "goalie_kick_type",
        configuredKickType == "visual" ? "visual" : "default");
    brain->log->logToScreen("tree/Decide",
                            format("Decision: %s ballrange: %.2f ballyaw: %.2f kickDir: %.2f rbDir: %.2f angleIsGood: %d lead: %d claim: %d postBlock: %d", newDecision.c_str(), ballRange, ballYaw, kickDir, dir_rb_f, angleIsGood, brain->data->tmImLead, goalieMayClaimBall, postBlockClearance),
                            color);
    return NodeStatus::SUCCESS;
}

tuple<double, double, double> Kick::_calcSpeed() {
    double vx, vy, msecKick;

    // Read parameters.
    double vxLimit, vyLimit;
    getInput("vx_limit", vxLimit);
    getInput("vy_limit", vyLimit);
    int minMSecKick;
    getInput("min_msec_kick", minMSecKick);
    double vxFactor = brain->config->vxFactor;   // Scale vx to compensate for directional differences between commands and motion.
    double yawOffset = brain->config->yawOffset; // Compensate for localization-angle bias.

    // Calculate the velocity command.
    double adjustedYaw = brain->data->ball.yawToRobot + yawOffset;
    double tx = cos(adjustedYaw) * brain->data->ball.range; // Movement target
    double ty = sin(adjustedYaw) * brain->data->ball.range;

    if (fabs(ty) < 0.01 && fabs(adjustedYaw) < 0.01)
    { // Within kickable range, walk straight and avoid division by zero below.
        vx = vxLimit;
        vy = 0.0;
    }
    else
    { // Otherwise calculate a feasible velocity toward the required direction.
        vy = ty > 0 ? vyLimit : -vyLimit;
        vx = vy / ty * tx * vxFactor;
        if (fabs(vx) > vxLimit)
        {
            vy *= vxLimit / vx;
            vx = vxLimit;
        }
    }

    // Estimate movement time.
    double speed = norm(vx, vy);
    msecKick = speed > 1e-5 ? minMSecKick + static_cast<int>(brain->data->ball.range / speed * 1000) : minMSecKick;
     
    return make_tuple(vx, vy, msecKick);
}

NodeStatus Kick::onStart()
{
    _minRange = brain->data->ball.range;
    _speed = 0.1;
    _plannedKickStarted = false;
    _ballMotionObserved = false;
    _kickStartBallValid = brain->data->ballDetected &&
        std::isfinite(brain->data->ball.posToField.x) &&
        std::isfinite(brain->data->ball.posToField.y);
    if (_kickStartBallValid) {
        _kickStartBallField = brain->data->ball.posToField;
    }
    bool avoidPushing;
    double kickAoSafeDist;
    brain->get_parameter("obstacle_avoidance.avoid_during_kick", avoidPushing);
    brain->get_parameter("obstacle_avoidance.kick_ao_safe_dist", kickAoSafeDist);
    string role = brain->tree->getEntry<string>("player_role");
    if (
        avoidPushing
        && (role != "goal_keeper")
        && brain->data->robotPoseToField.x < brain->config->fieldDimensions.length / 2 - brain->config->fieldDimensions.goalAreaLength
        && brain->distToObstacle(brain->data->ball.yawToRobot) < kickAoSafeDist
    ) {
        brain->client->setVelocity(-0.1, 0, 0);
        return NodeStatus::SUCCESS;
    }

    // Initialize the node.
    _startTime = brain->get_clock()->now();
    const bool isGoalkeeper =
        brain->tree->getEntry<string>("player_role") == "goal_keeper";
    const bool enableStabilize = isGoalkeeper
        ? brain->get_parameter(
            "goalkeeper.kick.default.enable_stabilize").as_bool()
        : brain->config->enableStableKick;
    if (enableStabilize && brain->threatLevel() < 0.5) _state = "stablize"; // Stabilize before kicking when risk is low.
    else _state = "kick";
     
    // Send the motion command.
    if (_state == "kick") {
        double angle = brain->data->ball.yawToRobot;
        double speed = isGoalkeeper
            ? brain->get_parameter(
                "goalkeeper.kick.default.speed_limit").as_double()
            : getInput<double>("speed_limit").value();
        bool softKickoff;
        double softKickoffSpeed;
        brain->get_parameter("strategy.soft_kickoff", softKickoff);
        brain->get_parameter("strategy.soft_kickoff_speed", softKickoffSpeed);
        if (
            softKickoff
            && (brain->data->isFreekickKickingOff || brain->data->isKickingOff)
            ) speed = softKickoffSpeed;
        brain->client->crabWalk(angle, speed);
        if (brain->isPlannedFreeKickActor()) {
            brain->markFreeKickKickStarted();
            _plannedKickStarted = true;
        }
    } else if (_state == "stablize") {
        brain->client->setVelocity(-0.05, 0, 0, true, false, false);
    }
    // Return RUNNING so the node continues processing.
    return NodeStatus::RUNNING;
}

NodeStatus Kick::onRunning()
{
    const bool logKickDebug = brain->log->shouldLog(
        "kick_debug", brain->config->rerunLogDebugHz);
    auto log = [=](string msg) {
        if (logKickDebug) {
            brain->log->setTimeNow();
            brain->log->log("debug/Kick", rerun::TextLog(msg));
        }
    };
    const bool isGoalkeeper =
        brain->tree->getEntry<string>("player_role") == "goal_keeper";
    bool enableAbort = isGoalkeeper
        ? brain->get_parameter(
            "goalkeeper.kick.default.abort_when_ball_moved").as_bool()
        : brain->get_parameter(
            "strategy.abort_kick_when_ball_moved").as_bool();
    auto ballRange = brain->data->ball.range;
    double MOVE_RANGE_THRESHOLD = 0.3;
    MOVE_RANGE_THRESHOLD = isGoalkeeper
        ? brain->get_parameter(
            "goalkeeper.kick.default.ball_move_threshold").as_double()
        : brain->get_parameter(
            "strategy.abort_kick_ball_move_threshold").as_double();
    double KICK_RANGE = 1.0;
    KICK_RANGE = isGoalkeeper
        ? brain->get_parameter(
            "goalkeeper.kick.default.exit_range").as_double()
        : brain->get_parameter("strategy.kick_range").as_double();

    const double BALL_LOST_THRESHOLD = 1000;  // ms
    const double motionThreshold = std::max(0.15, MOVE_RANGE_THRESHOLD);
    if (brain->data->ballDetected) {
        const double fieldDisplacement = _kickStartBallValid
            ? std::hypot(
                brain->data->ball.posToField.x - _kickStartBallField.x,
                brain->data->ball.posToField.y - _kickStartBallField.y)
            : 0.0;
        const bool motionEvidence = _kickStartBallValid
            ? fieldDisplacement > motionThreshold
            : ballRange - _minRange > MOVE_RANGE_THRESHOLD;
        if (motionEvidence) {
            _ballMotionObserved = true;
        }
    }
    if (ballRange > KICK_RANGE){
        log(_ballMotionObserved
            ? "ball moved after kick"
            : "ball too far without confirmed motion");
        if (_plannedKickStarted) {
            if (_ballMotionObserved) {
                brain->markFreeKickKickCompleted();
            } else {
                brain->cancelFreeKickKickAttempt();
            }
            _plannedKickStarted = false;
        }
        return NodeStatus::SUCCESS;
    }
    if (
        enableAbort &&
        (brain->msecsSince(brain->data->ball.timePoint) >
             BALL_LOST_THRESHOLD ||
         _ballMotionObserved)
    ) {
        log("ball moved, abort kick");
        if (_plannedKickStarted) {
            if (_ballMotionObserved) {
                brain->markFreeKickKickCompleted();
            } else {
                brain->cancelFreeKickKickAttempt();
            }
            _plannedKickStarted = false;
        }
        return NodeStatus::SUCCESS;
    }
    log(format("ballrange: %.1f, minRange: %.1f", ballRange, _minRange));
    if (ballRange < _minRange) _minRange = ballRange;    

    bool avoidPushing;
    brain->get_parameter("obstacle_avoidance.avoid_during_kick", avoidPushing);
    double kickAoSafeDist;
    brain->get_parameter("obstacle_avoidance.kick_ao_safe_dist", kickAoSafeDist);
    if (
        avoidPushing
        && brain->data->robotPoseToField.x < brain->config->fieldDimensions.length / 2 - brain->config->fieldDimensions.goalAreaLength
        && brain->distToObstacle(brain->data->ball.yawToRobot) < kickAoSafeDist
    ) {
        brain->client->setVelocity(-0.1, 0, 0);
        if (_plannedKickStarted) {
            brain->cancelFreeKickKickAttempt();
            _plannedKickStarted = false;
        }
        return NodeStatus::SUCCESS;
    }

    if (_state == "stablize") {
        double msecs = isGoalkeeper
            ? brain->get_parameter(
                "goalkeeper.kick.default.stabilize_msec").as_double()
            : getInput<double>("msecs_stablize").value();
        if (brain->msecsSince(_startTime) > msecs) {
            _state = "kick";
            _startTime = brain->get_clock()->now();
            double angle = brain->data->ball.yawToRobot;
            double speed = isGoalkeeper
                ? brain->get_parameter(
                    "goalkeeper.kick.default.speed_limit").as_double()
                : getInput<double>("speed_limit").value();
            bool softKickoff;
            double softKickoffSpeed;
            brain->get_parameter("strategy.soft_kickoff", softKickoff);
            brain->get_parameter("strategy.soft_kickoff_speed", softKickoffSpeed);
            if (
                softKickoff
                && (brain->data->isFreekickKickingOff || brain->data->isKickingOff)
                ) speed = softKickoffSpeed;
            brain->client->crabWalk(angle, speed);
            if (brain->isPlannedFreeKickActor()) {
                brain->markFreeKickKickStarted();
                _plannedKickStarted = true;
            }
        }
        return NodeStatus::RUNNING;
    } else if (_state == "kick") {
        double msecs = isGoalkeeper
            ? brain->get_parameter(
                "goalkeeper.kick.default.min_msec").as_double()
            : getInput<double>("min_msec_kick").value();
        double speed = isGoalkeeper
            ? brain->get_parameter(
                "goalkeeper.kick.default.speed_limit").as_double()
            : getInput<double>("speed_limit").value();
        msecs = msecs + brain->data->ball.range / speed * 1000;
        if (brain->msecsSince(_startTime) > msecs) { // Complete the kick action.
            brain->client->setVelocity(0, 0, 0);
            if (_plannedKickStarted) {
                // A timer only means that the motion command finished.  Do
                // not advance a synchronized restart unless the ball showed
                // evidence of leaving the original contact area.
                if (_ballMotionObserved) {
                    brain->markFreeKickKickCompleted();
                } else {
                    brain->cancelFreeKickKickAttempt();
                }
                _plannedKickStarted = false;
            }
            return NodeStatus::SUCCESS;
        }
        // else
        if (brain->data->ballDetected) { // Correct direction while the ball remains visible.
            double angle = brain->data->ball.yawToRobot;
            double speed = isGoalkeeper
                ? brain->get_parameter(
                    "goalkeeper.kick.default.speed_limit").as_double()
                : getInput<double>("speed_limit").value();
            _speed += 0.08;
            speed = min(speed, _speed);
            brain->client->crabWalk(angle, speed);
        }

        return NodeStatus::RUNNING;
    }

    // should not reach here
    prtErr("Kick: Reached impossible condition");
    return NodeStatus::SUCCESS;
}

void Kick::onHalted()
{
    brain->client->setVelocity(0.0, 0.0, 0.0, false, false, false);
    if (_plannedKickStarted) {
        brain->cancelFreeKickKickAttempt();
        _plannedKickStarted = false;
    }
    _startTime -= rclcpp::Duration(100, 0);
}


rclcpp::Time RLVisionKick::_lastExitTime = rclcpp::Time(0, 0, RCL_ROS_TIME);

NodeStatus RLVisionKick::onStart()
{
    _startTime = brain->get_clock()->now();
    _isDecelerating = false;
    _visionKickStarted = false;
    _pendingRobocupWalk = false;
    _plannedKickStarted = false;
    _ballMotionObserved = false;
    _kickStartBallValid = brain->data->ballDetected &&
        std::isfinite(brain->data->ball.posToField.x) &&
        std::isfinite(brain->data->ball.posToField.y);
    if (_kickStartBallValid) {
        _kickStartBallField = brain->data->ball.posToField;
    }

    GameObject fallenRobot;
    if (brain->isFallenRobotVisualKickExitEnabled() &&
        brain->hasFallenRobotInVisualKickZone(&fallenRobot)) {
        brain->client->requestFallenRobotAvoidance();
        brain->data->tmImInVisualKick = false;
        brain->client->robocupWalk();
        brain->client->setVelocity(0.0, 0.0, 0.0, false, false, false);
        recordExitTime();
        return NodeStatus::SUCCESS;
    }

    const bool isGoalkeeper =
        brain->tree->getEntry<string>("player_role") == "goal_keeper";
    const double preDelayMsec = isGoalkeeper
        ? std::max(0.0, brain->get_parameter(
            "goalkeeper.kick.visual.pre_delay_msec").as_double())
        : 1000.0;
    startDecelerate(preDelayMsec);
    stepDecelerate();

    return NodeStatus::RUNNING;
}

NodeStatus RLVisionKick::onRunning()
{
    auto logExit = [=](const string &msg) {
        brain->log->setTimeNow();
        brain->log->log("debug/RLVisionKick", rerun::TextLog(msg));
        std::cout << "[RLVisionKick] " << msg << std::endl;
    };

    updatePlannedFreeKickBallMotion();

    GameObject fallenRobot;
    if (brain->isFallenRobotVisualKickExitEnabled() &&
        brain->hasFallenRobotInVisualKickZone(&fallenRobot)) {
        logExit(format(
            "exit visual kick: fallen robot at range=%.2f yaw=%.2f",
            fallenRobot.range,
            fallenRobot.yawToRobot));
        brain->client->requestFallenRobotAvoidance();
        finishPlannedFreeKickAttempt(_ballMotionObserved);
        brain->data->tmImInVisualKick = false;
        brain->client->robocupWalk();
        brain->client->setVelocity(0.0, 0.0, 0.0, false, false, false);
        recordExitTime();
        _isDecelerating = false;
        _visionKickStarted = false;
        _pendingRobocupWalk = false;
        return NodeStatus::SUCCESS;
    }

    if (brain->data->shouldExitRLVisionKick) {
        logExit("exit visual kick: shouldExitRLVisionKick=true");
        finishPlannedFreeKickAttempt(_ballMotionObserved);
        brain->data->shouldExitRLVisionKick = false;
        brain->data->tmImInVisualKick = false;
        recordExitTime();
        return NodeStatus::SUCCESS;
    }

    if (_isDecelerating) {
        stepDecelerate();

        if (!_isDecelerating) {
            if (_pendingRobocupWalk) {
                brain->client->robocupWalk();
                _pendingRobocupWalk = false;
                brain->data->tmImInVisualKick = false;
                return NodeStatus::SUCCESS;
            } else if (!_visionKickStarted) {
                const int result = brain->client->RLVisionKick(true);
                if (result != 0) {
                    logExit(format(
                        "failed to start visual kick: result=%d", result));
                    brain->data->tmImInVisualKick = false;
                    recordExitTime();
                    return NodeStatus::SUCCESS;
                }
                _headScanStartTime = brain->get_clock()->now();
                _visionKickStarted = true;
                if (brain->isPlannedFreeKickActor()) {
                    brain->markFreeKickKickStarted();
                    _plannedKickStarted = true;
                }
            }
        }
        return NodeStatus::RUNNING;
    }

    if (_visionKickStarted) {
        double headMsec = brain->msecsSince(_headScanStartTime);
        if (headMsec < 300.0) {
            brain->client->moveHead(0.4, 0.0);
        } else if (headMsec < 550.0) {
            brain->client->moveHead(0.7, 0.0);
        }
    }

    double elapsed = brain->msecsSince(_startTime);
    const bool isGoalkeeper =
        brain->tree->getEntry<string>("player_role") == "goal_keeper";
    double minMsecKick = isGoalkeeper
        ? brain->get_parameter("goalkeeper.kick.visual.min_msec").as_double()
        : getInput<double>("min_msec_kick").value();
    double maxMsecKick = isGoalkeeper
        ? brain->get_parameter("goalkeeper.kick.visual.max_msec").as_double()
        : getInput<double>("max_msec_kick").value();
    double rangeThreshold = isGoalkeeper
        ? brain->get_parameter("goalkeeper.kick.visual.range").as_double()
        : getInput<double>("range").value();

    bool ballTooFar = brain->data->ballDetected && brain->data->ball.range > rangeThreshold;
    bool costTooHigh = brain->data->tmMyCost > 8.0;
    bool elapsedEnough = elapsed > minMsecKick;
    bool elapsedTimeout = elapsed > maxMsecKick;
    bool loseBall = brain->data->lose_ball;
    bool loseBallExit = loseBall && elapsedEnough;
    bool ballOut = brain->tree->getEntry<bool>("ball_out");
    bool plannedKickCompleted =
        _plannedKickStarted && _ballMotionObserved;
    bool shouldExit =
        ((ballTooFar || costTooHigh) && elapsedEnough) ||
        loseBallExit || ballOut || elapsedTimeout || plannedKickCompleted;

    if (shouldExit) {
        string reasons = "";
        auto appendReason = [&](const string &reason) {
            if (!reasons.empty()) reasons += ", ";
            reasons += reason;
        };
        if (ballTooFar && elapsedEnough) appendReason(format("ball_too_far(range=%.2f,threshold=%.2f)", brain->data->ball.range, rangeThreshold));
        if (costTooHigh && elapsedEnough) appendReason(format("tm_cost_high(cost=%.2f,min=%.0fms,elapsed=%.0fms)", brain->data->tmMyCost, minMsecKick, elapsed));
        if (loseBallExit) appendReason("lose_ball=true");
        if (ballOut) appendReason("ball_out=true");
        if (elapsedTimeout) appendReason(format("timeout(max=%.0fms,elapsed=%.0fms)", maxMsecKick, elapsed));
        if (plannedKickCompleted) appendReason("free_kick_ball_motion_confirmed");
        if (reasons.empty()) reasons = "unknown";
        logExit(format("exit visual kick: %s", reasons.c_str()));

        finishPlannedFreeKickAttempt(plannedKickCompleted);
        recordExitTime();
        const double postDelayMsec = isGoalkeeper
            ? std::max(0.0, brain->get_parameter(
                "goalkeeper.kick.visual.post_delay_msec").as_double())
            : 1000.0;
        startDecelerate(postDelayMsec);
        _pendingRobocupWalk = true;
        stepDecelerate();
        return NodeStatus::RUNNING;
    }

    return NodeStatus::RUNNING;
}

void RLVisionKick::onHalted()
{
    const string haltMsg = "halted by behavior tree, force exit visual kick";
    brain->log->setTimeNow();
    brain->log->log("debug/RLVisionKick", rerun::TextLog(haltMsg));
    std::cout << "[RLVisionKick] " << haltMsg << std::endl;
    updatePlannedFreeKickBallMotion();
    finishPlannedFreeKickAttempt(_ballMotionObserved);
    brain->data->tmImInVisualKick = false;
    brain->client->setVelocity(0.0, 0.0, 0.0, false, false, false);
    brain->client->robocupWalk();
    recordExitTime();

    _isDecelerating = false;
    _visionKickStarted = false;
    _pendingRobocupWalk = false;
}

bool RLVisionKick::isMinIntervalSatisfied(double minIntervalMsec)
{
    (void)minIntervalMsec;
    return true;
}

void RLVisionKick::recordExitTime()
{
    _lastExitTime = brain->get_clock()->now();
}

void RLVisionKick::startDecelerate(double durationMs)
{
    if (_isDecelerating) {
        return;
    }

    _isDecelerating = true;
    _decelStartTime = brain->get_clock()->now();
    _decelDurationMs = durationMs;
}

bool RLVisionKick::stepDecelerate()
{
    if (!_isDecelerating) {
        return true;
    }

    double elapsed = brain->msecsSince(_decelStartTime);
    brain->client->setVelocity(0.0, 0.0, 0.0, false, false, false);

    if (elapsed >= _decelDurationMs) {
        _isDecelerating = false;
        return true;
    }

    return false;
}

void RLVisionKick::updatePlannedFreeKickBallMotion()
{
    if (!_plannedKickStarted || _ballMotionObserved ||
        !_kickStartBallValid || !brain->data->ballDetected) {
        return;
    }

    const Point &ball = brain->data->ball.posToField;
    if (!std::isfinite(ball.x) || !std::isfinite(ball.y)) {
        return;
    }

    double configuredThreshold = 0.3;
    brain->get_parameter(
        "strategy.abort_kick_ball_move_threshold", configuredThreshold);
    const double motionThreshold = std::max(0.15, configuredThreshold);
    const double displacement = std::hypot(
        ball.x - _kickStartBallField.x,
        ball.y - _kickStartBallField.y);
    _ballMotionObserved = displacement > motionThreshold;
}

void RLVisionKick::finishPlannedFreeKickAttempt(bool completed)
{
    if (!_plannedKickStarted) {
        return;
    }

    if (completed) {
        brain->markFreeKickKickCompleted();
    } else {
        brain->cancelFreeKickKickAttempt();
    }
    _plannedKickStarted = false;
}


NodeStatus Intercept::onStart()
{
    auto log = [=](string msg) {
        // brain->log->setTimeNow();
        // brain->log->log("debug/intercept", rerun::TextLog(msg));
    };
    log("onStart");

    brain->log->setTimeNow();
    brain->log->log("debug/intercept", rerun::TextLog("start"));

    _interceptX = brain->data->ballInterceptPoint.x;
    _interceptY = brain->data->ballInterceptPoint.y;
    _startTime = brain->get_clock()->now();
    return NodeStatus::RUNNING;
}

NodeStatus Intercept::onRunning()
{
    auto log = [=](string msg) {
        // brain->log->setTimeNow();
        // brain->log->log("debug/intercept", rerun::TextLog(msg));
    };
    log("onRunning");

    bool useMove;
    brain->get_parameter("strategy.use_move_block", useMove);
    double moveMsecs;
    brain->get_parameter("strategy.move_block_msecs", moveMsecs);

    if (brain->msecsSince(_startTime) > moveMsecs) {
        log("Time reached");
        brain->client->setVelocity(0.0, 0.0, 0.0, false, false, false);
        brain->tree->setEntry<string>("goalie_mode", "attack");
        return NodeStatus::SUCCESS;
    }

    if (!useMove) {
        return NodeStatus::SUCCESS;
    }

    const auto target_r = brain->data->field2robot(
        Pose2D({_interceptX, _interceptY}));
    const double theta = atan2(target_r.y, target_r.x);
    const double dist = norm(target_r.x, target_r.y);
    const double speed = dist > 0.5 ? 1.0 : dist;
    const double vx = speed * cos(theta);
    const double vy = speed * sin(theta);
    brain->client->setVelocity(vx, vy, 0);
    log(format(
        "Move to intercept point. dist: %.1f, theta: %.1f, vx: %.1f, vy: %.1f",
        dist, theta, vx, vy));
    return NodeStatus::RUNNING;
}

void Intercept::onHalted()
{
    auto log = [=](string msg) {
        // brain->log->setTimeNow();
        // brain->log->log("debug/intercept", rerun::TextLog(msg));
    };
    log("halted");
    brain->client->setVelocity(0.0, 0.0, 0.0, false, false, false);
    brain->speak("halt");
    return;
}

NodeStatus StandStill::onStart()
{
    // Initialize the node.
    _startTime = brain->get_clock()->now();

    // Send the motion command.
    brain->client->setVelocity(0, 0, 0);
    return NodeStatus::RUNNING;
}

NodeStatus StandStill::onRunning()
{
    double msecs;
    getInput("msecs", msecs);
    if (brain->msecsSince(_startTime) < msecs) {
        brain->client->setVelocity(0, 0, 0);
        return NodeStatus::RUNNING;
    }

    // else
    return NodeStatus::SUCCESS;
}

void StandStill::onHalted()
{
    double msecs;
    getInput("msecs", msecs);
    _startTime -= rclcpp::Duration(- 2 * msecs, 0);
}


NodeStatus RobotFindBall::onStart()
{
    if (brain->data->ballDetected ||
        brain->tree->getEntry<bool>("ball_location_known") ||
        brain->tree->getEntry<bool>("tm_ball_pos_reliable")) {
        brain->client->setVelocity(0, 0, 0);
        return NodeStatus::SUCCESS;
    }

    _turnDir = brain->data->ball.yawToRobot >= 0.0 ? 1.0 : -1.0;
    _phase = Phase::InitialSweep;
    _phaseStartTime = brain->get_clock()->now();
    _waypointIndex = 0;

    const BallSearchRuntime runtime = getBallSearchRuntime(brain);
    _setPlayKind = static_cast<int>(runtime.setPlay);
    _ourSetPlay = runtime.ourSetPlay;
    _searcherIndex = runtime.searcherIndex;
    _searcherCount = runtime.searcherCount;
    _waypoints = makeRobotBallSearchPlan(brain, runtime);

    return NodeStatus::RUNNING;
}

NodeStatus RobotFindBall::onRunning()
{
    if (brain->data->ballDetected ||
        brain->tree->getEntry<bool>("ball_location_known") ||
        brain->tree->getEntry<bool>("tm_ball_pos_reliable")) {
        brain->client->setVelocity(0, 0, 0);
        return NodeStatus::SUCCESS;
    }

    const BallSearchRuntime runtime = getBallSearchRuntime(brain);
    const int setPlayKind = static_cast<int>(runtime.setPlay);
    if (setPlayKind != _setPlayKind ||
        runtime.ourSetPlay != _ourSetPlay ||
        runtime.searcherIndex != _searcherIndex ||
        runtime.searcherCount != _searcherCount) {
        _setPlayKind = setPlayKind;
        _ourSetPlay = runtime.ourSetPlay;
        _searcherIndex = runtime.searcherIndex;
        _searcherCount = runtime.searcherCount;
        _waypoints = makeRobotBallSearchPlan(brain, runtime);
        _waypointIndex = 0;
        _phase = Phase::InitialSweep;
        _phaseStartTime = brain->get_clock()->now();
    }

    if (!ballSearchMotionAllowed(brain)) {
        brain->client->setVelocity(0, 0, 0);
        return NodeStatus::RUNNING;
    }

    double vyawLimit = 0.8;
    getInput("vyaw_limit", vyawLimit);
    const double configuredVthetaLimit = std::max(
        0.0,
        brain->get_parameter("strategy.search.vtheta_limit").as_double());
    vyawLimit = std::min(std::fabs(vyawLimit), configuredVthetaLimit);

    // A fresh teammate sighting is useful even when its field position failed
    // the existing reliability filter. Stay local and scan until that resolves.
    if (runtime.anyFreshTeammateSeesBall) {
        _phase = Phase::InitialSweep;
        _phaseStartTime = brain->get_clock()->now();
        brain->client->setVelocity(0, 0, vyawLimit * _turnDir);
        return NodeStatus::RUNNING;
    }

    const double initialSweepMs = 1000.0 * std::max(
        0.0,
        brain->get_parameter("strategy.search.initial_spin_secs").as_double());
    const double waypointSweepMs = 1000.0 * std::max(
        0.0,
        brain->get_parameter("strategy.search.waypoint_scan_secs").as_double());
    const double waypointTimeoutMs = 1000.0 * std::max(
        1.0,
        brain->get_parameter("strategy.search.waypoint_timeout_secs").as_double());
    const double arrivalTolerance = std::max(
        0.2,
        brain->get_parameter("strategy.search.arrival_tolerance").as_double());

    if (_phase == Phase::InitialSweep) {
        if (brain->msecsSince(_phaseStartTime) < initialSweepMs ||
            _waypoints.empty()) {
            brain->client->setVelocity(0, 0, vyawLimit * _turnDir);
            return NodeStatus::RUNNING;
        }
        _phase = Phase::Transit;
        _phaseStartTime = brain->get_clock()->now();
    }

    if (_waypoints.empty()) {
        brain->client->setVelocity(0, 0, vyawLimit * _turnDir);
        return NodeStatus::RUNNING;
    }
    _waypointIndex %= _waypoints.size();

    if (_phase == Phase::WaypointSweep) {
        if (brain->msecsSince(_phaseStartTime) < waypointSweepMs) {
            brain->client->setVelocity(0, 0, vyawLimit * _turnDir);
            return NodeStatus::RUNNING;
        }
        _waypointIndex = (_waypointIndex + 1) % _waypoints.size();
        _turnDir *= -1.0;
        _phase = Phase::Transit;
        _phaseStartTime = brain->get_clock()->now();
    }

    if (_phase == Phase::Transit &&
        brain->msecsSince(_phaseStartTime) >= waypointTimeoutMs) {
        _waypointIndex = (_waypointIndex + 1) % _waypoints.size();
        _turnDir *= -1.0;
        _phaseStartTime = brain->get_clock()->now();
    }

    const Pose2D &target = _waypoints[_waypointIndex];
    const double targetDistance = std::hypot(
        target.x - brain->data->robotPoseToField.x,
        target.y - brain->data->robotPoseToField.y);
    if (targetDistance <= arrivalTolerance) {
        _phase = Phase::WaypointSweep;
        _phaseStartTime = brain->get_clock()->now();
        brain->client->setVelocity(0, 0, vyawLimit * _turnDir);
        return NodeStatus::RUNNING;
    }

    const double vxLimit = std::max(
        0.0,
        brain->get_parameter("strategy.search.vx_limit").as_double());
    const double vyLimit = std::max(
        0.0,
        brain->get_parameter("strategy.search.vy_limit").as_double());
    brain->client->moveToPoseOnField2(
        target.x,
        target.y,
        target.theta,
        1.2,
        0.45,
        vxLimit,
        vyLimit,
        configuredVthetaLimit,
        arrivalTolerance * 0.6,
        arrivalTolerance * 0.6,
        0.6,
        true);
    return NodeStatus::RUNNING;
}

void RobotFindBall::onHalted()
{
    brain->client->setVelocity(0, 0, 0);
    _turnDir = 1.0;
    _phase = Phase::InitialSweep;
    _waypoints.clear();
    _waypointIndex = 0;
}

NodeStatus CamFastScan::onStart()
{
    _cmdIndex = 0;
    _timeLastCmd = brain->get_clock()->now();
    brain->client->moveHead(_cmdSequence[_cmdIndex][0], _cmdSequence[_cmdIndex][1]);
    return NodeStatus::RUNNING;
}

NodeStatus CamFastScan::onRunning()
{
    double interval = getInput<double>("msecs_interval").value();
    if (brain->msecsSince(_timeLastCmd) < interval) return NodeStatus::RUNNING;

    // else 
    if (_cmdIndex >= 6) return NodeStatus::SUCCESS;

    // else
    _cmdIndex++;
    _timeLastCmd = brain->get_clock()->now();
    brain->client->moveHead(_cmdSequence[_cmdIndex][0], _cmdSequence[_cmdIndex][1]);
    return NodeStatus::RUNNING;
}

NodeStatus TurnOnSpot::onStart()
{
    _timeStart = brain->get_clock()->now();
    _lastAngle = brain->data->robotPoseToOdom.theta;
    _cumAngle = 0.0;

    bool towardsBall = false;
    _angle = getInput<double>("rad").value();
    getInput("towards_ball", towardsBall);
    if (towardsBall) {
        double ballPixX = (brain->data->ball.boundingBox.xmin + brain->data->ball.boundingBox.xmax) / 2;
        _angle = fabs(_angle) * (ballPixX < brain->config->camPixX / 2 ? 1 : -1);
    }

    brain->client->setVelocity(0, 0, _angle, false, false, true);
    return NodeStatus::RUNNING;
}

NodeStatus TurnOnSpot::onRunning()
{
    double curAngle = brain->data->robotPoseToOdom.theta;
    double deltaAngle = toPInPI(curAngle - _lastAngle);
    _lastAngle = curAngle;
    _cumAngle += deltaAngle;
    double turnTime = brain->msecsSince(_timeStart);
    // brain->log->log("debug/turn_on_spot", rerun::TextLog(format(
    //      "angle: %.2f, cumAngle: %.2f, deltaAngle: %.2f, time: %.2f",
    //      _angle, _cumAngle, deltaAngle, turnTime
    // )));
    if (
        fabs(_cumAngle) - fabs(_angle) > -0.1
        || turnTime > _msecLimit
    ) {
        brain->client->setVelocity(0, 0, 0);
        return NodeStatus::SUCCESS;
    }

    // else 
    brain->client->setVelocity(0, 0, (_angle - _cumAngle)*2);
    return NodeStatus::RUNNING;
}


NodeStatus SelfLocate::tick()
{
    if (brain->isRecoveryLocalizationBlocked()) return NodeStatus::SUCCESS;

    const bool logSelfLocateDebug = brain->log->shouldLog(
        "self_locate_debug", brain->config->rerunLogDebugHz);
    auto log = [=](string msg) {
        if (logSelfLocateDebug) {
            brain->log->setTimeNow();
            brain->log->log("debug/SelfLocate", rerun::TextLog(msg));
        }
    };
    string mode = getInput<string>("mode").value();
    double xMin, xMax, yMin, yMax, thetaMin, thetaMax; // Search bounds
    auto markers = brain->data->getMarkersForLocator();
    const bool headingRealignActive =
        mode == "trust_direction" &&
        brain->isPostRecoveryHeadingRealignActive();
    bool headingRealignAttempt = false;
    if (headingRealignActive) {
        if (markers.size() < static_cast<size_t>(brain->locator->minMarkerCnt)) {
            return NodeStatus::SUCCESS;
        }
        headingRealignAttempt =
            brain->beginPostRecoveryHeadingRealignAttempt();
        if (!headingRealignAttempt) return NodeStatus::SUCCESS;
    } else {
        double interval = getInput<double>("msecs_interval").value();
        if (brain->msecsSince(brain->data->lastSuccessfulLocalizeTime) < interval) {
            return NodeStatus::SUCCESS;
        }
    }

    // Calculate localization constraints.
    if (mode == "enter_field")
    {
        // x range: our half, stopping before the center-circle boundary.
        xMin = -brain->config->fieldDimensions.length / 2;
        xMax = -brain->config->fieldDimensions.circleRadius;

        // y range: outside the touchline and within 1 m of it.
        // yMin = - brain->config->fieldDimensions.width / 2 - 1.0;
        // yMax = brain->config->fieldDimensions.width / 2 + 1.0;
        if (brain->config->playerStartPos == "left")
        {
            yMin = brain->config->fieldDimensions.width / 2;
            yMax = brain->config->fieldDimensions.width / 2 + 1.0;
        }
        else if (brain->config->playerStartPos == "right")
        {
            yMin = -brain->config->fieldDimensions.width / 2 - 1.0;
            yMax = -brain->config->fieldDimensions.width / 2;
        }

        // Theta range: face the field within 30 degrees.
        if (brain->config->playerStartPos == "left")
        {
            thetaMin = -M_PI / 2 - M_PI / 6;
            thetaMax = -M_PI / 2 + M_PI / 6;
        }
        else if (brain->config->playerStartPos == "right")
        {
            thetaMin = M_PI / 2 - M_PI / 6;
            thetaMax = M_PI / 2 + M_PI / 6;
        }
    }
    else if (mode == "face_forward")
    {
        xMin = -brain->config->fieldDimensions.length / 2;
        xMax = brain->config->fieldDimensions.length / 2;
        yMin = -brain->config->fieldDimensions.width / 2;
        yMax = brain->config->fieldDimensions.width / 2;
        thetaMin = -M_PI / 4;
        thetaMax = M_PI / 4;
    }
    else if (mode == "trust_direction")
    {
        if (headingRealignAttempt) {
            const double maxPositionDelta = std::max(
                0.1,
                brain->get_parameter(
                    "recovery.heading_realign_max_candidate_pos_m").as_double());
            const double maxHeadingDelta = std::max(
                0.1,
                brain->get_parameter(
                    "recovery.heading_realign_max_delta_rad").as_double());
            const auto &fd = brain->config->fieldDimensions;
            const Pose2D currentPose = brain->data->robotPoseToField;
            xMin = max(-fd.length / 2 - 2, currentPose.x - maxPositionDelta);
            xMax = min(fd.length / 2 + 2, currentPose.x + maxPositionDelta);
            yMin = max(-fd.width / 2 - 2, currentPose.y - maxPositionDelta);
            yMax = min(fd.width / 2 + 2, currentPose.y + maxPositionDelta);
            thetaMin = currentPose.theta - maxHeadingDelta;
            thetaMax = currentPose.theta + maxHeadingDelta;
        } else {
            int msec = static_cast<int>(brain->msecsSince(brain->data->lastSuccessfulLocalizeTime));
            double maxDriftSpeed = 0.1;
            double maxDrift = msec / 1000.0 * maxDriftSpeed;

            xMin = max(-brain->config->fieldDimensions.length / 2 - 2, brain->data->robotPoseToField.x - maxDrift);
            xMax = min(brain->config->fieldDimensions.length / 2 + 2, brain->data->robotPoseToField.x + maxDrift);
            yMin = max(-brain->config->fieldDimensions.width / 2 - 2, brain->data->robotPoseToField.y - maxDrift);
            yMax = min(brain->config->fieldDimensions.width / 2 + 2, brain->data->robotPoseToField.y + maxDrift);
            thetaMin = brain->data->robotPoseToField.theta - M_PI / 180;
            thetaMax = brain->data->robotPoseToField.theta + M_PI / 180;
        }
    }
    else if (mode == "fall_recovery")
    {
        int msec = static_cast<int>(brain->msecsSince(brain->data->lastSuccessfulLocalizeTime));
        double maxDriftSpeed = 0.1;                       // m/s
        double maxDrift = msec / 1000.0 * maxDriftSpeed; // Maximum odometry drift over this interval

        xMin = -brain->config->fieldDimensions.length / 2 - 2;
        xMax = brain->config->fieldDimensions.length / 2 + 2;
        yMin = -brain->config->fieldDimensions.width / 2 - 2;
        yMax = brain->config->fieldDimensions.width / 2 + 2;
        thetaMin = brain->data->robotPoseToField.theta - M_PI / 180;
        thetaMax = brain->data->robotPoseToField.theta + M_PI / 180;
    }

    // TODO other modes

    // Locate
    PoseBox2D constraints{xMin, xMax, yMin, yMax, thetaMin, thetaMax};
    double residual;
    // Run particle-filter localization.
    auto res = headingRealignAttempt
        ? brain->locator->locateRobot(
              markers, constraints, 600, 1.0, 1.0,
              brain->get_parameter(
                  "recovery.heading_realign_max_delta_rad").as_double())
        : brain->locator->locateRobot(markers, constraints);

    brain->log->setTimeNow();
    string mstring = "";
    for (int i = 0; i < markers.size(); i++) {
        auto m = markers[i];
        mstring += format("type: %c  x: %.1f y: %.1f", m.type, m.x, m.y);
    }
    if (res.success) {
        
        brain->log->log(
            "field/recal",
            rerun::Arrows2D::from_vectors({{res.pose.x - brain->data->robotPoseToField.x, -res.pose.y + brain->data->robotPoseToField.y}})
            .with_origins({{brain->data->robotPoseToField.x, - brain->data->robotPoseToField.y}})
            .with_colors(res.success ? 0x00FF00FF : 0xFF0000FF)
            .with_radii(0.01)
            .with_draw_order(10)
            .with_labels({"pf"})
        );
    }
    log(
        format(
            "success: %d  residual: %.2f  marker.size: %d  minMarkerCnt: %d  resTolerance: %.2f marker: %s",
            res.success,
            res.residual,
            markers.size(),
            brain->locator->minMarkerCnt,
            brain->locator->residualTolerance,
            mstring.c_str()
        )
    );
    
    // Localization failed.
    if (!res.success)
        return NodeStatus::SUCCESS; // Do not block following nodes.

    if (headingRealignAttempt) {
        brain->applyPostRecoveryHeadingRealignCandidate(
            res.pose, res.residual);
        return NodeStatus::SUCCESS;
    }

    // Otherwise localization succeeded.
    brain->calibrateOdom(
        res.pose.x, res.pose.y, res.pose.theta, "SelfLocate");
    brain->tree->setEntry<bool>("odom_calibrated", true);
    brain->data->lastSuccessfulLocalizeTime = brain->get_clock()->now();
    prtDebug("Localization succeeded: " + to_string(res.pose.x) + " " + to_string(res.pose.y) + " " + to_string(rad2deg(res.pose.theta)) + " Dur: " + to_string(res.msecs));

    return NodeStatus::SUCCESS;
}

// SelfLocateEnterField automatically detects the left or right entry position.
NodeStatus SelfLocateEnterField::tick()
{
    if (brain->isRecoveryLocalizationBlocked() ||
        brain->isPostRecoveryHeadingRealignActive()) return NodeStatus::SUCCESS;

    const bool logEnterFieldDebug = brain->log->shouldLog(
        "self_locate_enter_field_debug", brain->config->rerunLogDebugHz);
    auto log = [=](string msg, bool success) {
        if (logEnterFieldDebug) {
            brain->log->setTimeNow();
            brain->log->log("debug/SelfLocateEnterField", rerun::TextLog(msg).with_level(success? rerun::TextLogLevel::Info : rerun::TextLogLevel::Error));
        }
    };
    double interval = getInput<double>("msecs_interval").value();
    if (brain->msecsSince(brain->data->lastSuccessfulLocalizeTime) < interval) return NodeStatus::SUCCESS;

    auto markers = brain->data->getMarkersForLocator();
    auto fd = brain->config->fieldDimensions;
    
    // Define constraints for both sides.
    PoseBox2D cEnterLeft = {-fd.length / 2, -fd.circleRadius, fd.width / 2, fd.width / 2 + 1, -M_PI / 2 - M_PI / 6, -M_PI / 2 + M_PI / 6};
    PoseBox2D cEnterRight = {-fd.length / 2, -fd.circleRadius, -fd.width / 2 - 1, -fd.width / 2, M_PI / 2 - M_PI / 6, M_PI / 2 + M_PI / 6};

    // Attempt localization on each side.
    auto resLeft = brain->locator->locateRobot(markers, cEnterLeft);
    auto resRight = brain->locator->locateRobot(markers, cEnterRight);
    LocateResult res;

    static string lastReport = "";
    string report = lastReport;
    
    // Select the best localization result.
    if (resLeft.success && !resRight.success) {
        res = resLeft;
        report = "Entering Left";
    }
    else if (!resLeft.success && resRight.success) {
        res = resRight;
        report = "Entering Right";
    }
    else if (resLeft.success && resRight.success) {
        // Both succeeded; select the lower residual.
        if (resLeft.residual < resRight.residual) {
            res = resLeft;
            report = "Entering Left";
        }
        else {
            res = resRight;
            report = "Entering Right";
        }
    } else {
        res = resLeft; // Both failed; retain the left-side result.
    }

    if (report != lastReport) {
        brain->speak(report);
        lastReport = report;
    }

    brain->log->setTimeNow();
    string logPath = res.success ? "debug/locator_enter_field/success" : "debug/locator_enter_field/fail";
    log(
            format(
                "%s left success: %d  left residual: %.2f  right success %d  right residual %.2f resTolerance: %.2f markers: %d minMarkerCnt: %d ",
                report.c_str(),
                resLeft.success, 
                resLeft.residual,
                resRight.success,
                resRight.residual,
                brain->locator->residualTolerance,
                markers.size(),
                brain->locator->minMarkerCnt
            ),
            res.success
        );

    if (res.success || brain->log->shouldLog(
            "self_locate_enter_field_visual", brain->config->rerunLogVisualHz)) {
        brain->log->log(
            "field/recal_enter_field",
            rerun::Arrows2D::from_vectors({{res.pose.x - brain->data->robotPoseToField.x, -res.pose.y + brain->data->robotPoseToField.y}})
                .with_origins({{brain->data->robotPoseToField.x, - brain->data->robotPoseToField.y}})
                .with_colors(res.success ? 0x00FF00FF: 0xFF0000FF)
                .with_radii(0.01)
                .with_draw_order(10)
                .with_labels({"pfe"})
        );
    }

    if (!res.success) return NodeStatus::SUCCESS; // Do not block subsequent nodes.

    // Localization succeeded.
    brain->calibrateOdom(
        res.pose.x, res.pose.y, res.pose.theta, "SelfLocateEnterField");
    brain->tree->setEntry<bool>("odom_calibrated", true);
    brain->data->lastSuccessfulLocalizeTime = brain->get_clock()->now();
    prtDebug("Localization succeeded: " + to_string(res.pose.x) + " " + to_string(res.pose.y) + " " +  to_string(rad2deg(res.pose.theta)) + " Dur: " + to_string(res.msecs));


    return NodeStatus::SUCCESS;
}

bool SelfLocateLocal::_singlePenalty() {
    auto penaltyPoints = brain->data->getMarkingsByType({"PenaltyPoint"});
    if (penaltyPoints.size() != 1) {
        brain->log->logThrottled("SelfLocateLocal/SinglePenalty/fail", brain->config->rerunLogDebugHz, "SelfLocateLocal/SinglePenalty",
            rerun::TextLog(format("Failed, penaltyPoints.size() = %d", penaltyPoints.size()))
        );
        return false;
    }

    // int msec = static_cast<int>(brain->msecsSince(brain->data->lastSuccessfulLocalizeTime));
    // double maxDriftSpeed = 0.1;                      // m/s

    double maxDrift = 2.0;

    brain->log->setTimeNow();

    auto pp = penaltyPoints[0]; // Observed penalty mark
    if (pp.range > 5.0) {
        brain->log->logThrottled("SelfLocateLocal/SinglePenalty/fail", brain->config->rerunLogDebugHz, "SelfLocateLocal/SinglePenalty", rerun::TextLog(format("Failed, Penalty point is too far (%.2f)", pp.range)));
        return false;
    }

    // Identify the penalty mark.
    auto fd = brain->config->fieldDimensions;
    Point2D ppo({fd.length / 2 - fd.penaltyDist, 0}); // Opponent penalty-mark map position
    Point2D pps({-fd.length / 2 + fd.penaltyDist, 0}); // Our penalty-mark map position
    Point2D ppt; // Target map position of the observed penalty mark

    double disto = norm(pp.posToField.x - ppo.x, pp.posToField.y - ppo.y); // Observation error for opponent mark
    double dists = norm(pp.posToField.x - pps.x, pp.posToField.y - pps.y); // Observation error for our mark

    if (disto < dists && disto < maxDrift)  ppt = ppo;
    else if (dists < disto && dists < maxDrift) ppt = pps;
    else {
        brain->log->logThrottled("SelfLocateLocal/SinglePenalty/fail", brain->config->rerunLogDebugHz, "SelfLocateLocal/SinglePenalty",
            rerun::TextLog(format("Failed: disto= %.2f  dists=%.2f maxDrift=%.2f", disto, dists, maxDrift))
        );
        return false;
    }

    // Calculate a pose hypothesis from a valid target penalty mark.
    Pose2D hypoPose = brain->data->robotPoseToField;
    hypoPose.x += ppt.x - pp.posToField.x;
    hypoPose.y += ppt.y - pp.posToField.y;

    // validate the hypo with other 
    auto allMarkers = brain->data->getMarkersForLocator();
    if (allMarkers.size() < 3) {
        brain->log->logThrottled("SelfLocateLocal/SinglePenalty/fail", brain->config->rerunLogDebugHz, "SelfLocateLocal/SinglePenalty",
            rerun::TextLog("Failed. Not enough markers for validation.")
        );
        return false;        
    }
    double residual = brain->locator->residual(allMarkers, hypoPose) / allMarkers.size();
    if (residual > brain->locator->residualTolerance) { // Validation failed; the penalty mark may be a false positive.
        brain->log->logThrottled("SelfLocateLocal/SinglePenalty/fail", brain->config->rerunLogDebugHz, "SelfLocateLocal/SinglePenalty",
            rerun::TextLog("Failed, validation failed. Possible misdetection.")
        );
        return false;
    }

    // else everything is ok, recalibrate with this hypo pose
    brain->calibrateOdom(
        hypoPose.x, hypoPose.y, hypoPose.theta,
        "SelfLocateLocal/SinglePenalty");
    brain->data->lastSuccessfulLocalizeTime = brain->get_clock()->now();
    brain->log->log("SelfLocateLocal/SinglePenalty", rerun::TextLog(format("Success. Residual = %.2f", residual)));
    brain->log->log(
        "field/recal",
        rerun::Arrows2D::from_vectors({{hypoPose.x - brain->data->robotPoseToField.x, -hypoPose.y + brain->data->robotPoseToField.y}})
            .with_origins({{brain->data->robotPoseToField.x, - brain->data->robotPoseToField.y}})
            .with_colors(0x00FF00FF)
            .with_radii(0.01)
            .with_draw_order(10)
            .with_labels({"1p"})
    );
    return true;
}

bool SelfLocateLocal::_doubleX() {
    auto points = brain->data->getMarkingsByType({"XCross"});
    if (points.size() != 2) {
        brain->log->logThrottled("SelfLocateLocal/DoubleX/fail", brain->config->rerunLogDebugHz, "SelfLocateLocal/DoubleX",
            rerun::TextLog(format("Failed, points.size() = %d", points.size()))
        );
        return false;
    }

    auto p0 = points[0]; auto p1 = points[1];

    if (
        fabs(p0.posToField.y - p1.posToField.y) > 0.3 // Incorrect orientation
        || fabs(fabs(p0.posToField.y - p1.posToField.y) - brain->config->fieldDimensions.circleRadius * 2.0) > 0.5 // Incorrect separation
        || p0.range > 5.0 || p1.range > 5.0 // Too far away
    ) {
        brain->log->logThrottled("SelfLocateLocal/DoubleX/fail", brain->config->rerunLogDebugHz, "SelfLocateLocal/DoubleX",
            rerun::TextLog(format("Failed, did not pass feature validation. dy = %.2f, dist = %.2f, range = [%.2f, %.2f]",
                p0.posToField.y - p1.posToField.y,
                fabs(p0.posToField.y - p1.posToField.y) - brain->config->fieldDimensions.circleRadius * 2.0,
                p0.range, p1.range
        ))
        );
        return false;
    }

    // Observed field-center position.
    double xc = (p0.posToField.x + p1.posToField.x) / 2.0;
    double yc = (p1.posToField.y + p1.posToField.y) / 2.0;

    double maxDrift = 2.0;
    if (norm(xc, yc) > maxDrift) {
        brain->log->logThrottled("SelfLocateLocal/DoubleX/fail", brain->config->rerunLogDebugHz, "SelfLocateLocal/DoubleX",
            rerun::TextLog(format("Failed, dist = %.2f > maxDrift(%.2f)", norm(xc, yc), maxDrift))
        );
        return false; 
    }
    // Validation passed; calculate a pose hypothesis from this point.
    brain->log->setTimeNow();
    Pose2D hypoPose = brain->data->robotPoseToField;
    hypoPose.x -= xc;
    hypoPose.y -= yc;

    // validate the hypo with other markings
    auto allMarkers = brain->data->getMarkersForLocator();
    if (allMarkers.size() < 3) {
        brain->log->logThrottled("SelfLocateLocal/DoubleX/fail", brain->config->rerunLogDebugHz, "SelfLocateLocal/DoubleX",
            rerun::TextLog("Failed. Not enough markers for validation.")
        );
        return false;        
    }
    double residual = brain->locator->residual(allMarkers, hypoPose) / allMarkers.size();
    if (residual > brain->locator->residualTolerance) { // Validation failed; the observed marking may be a false positive.
        brain->log->logThrottled("SelfLocateLocal/DoubleX/fail", brain->config->rerunLogDebugHz, "SelfLocateLocal/DoubleX",
            rerun::TextLog("Failed, validation failed. Possible misdetection.")
        );
        return false;
    }

    // else everything is ok, recalibrate with this hypo pose
    brain->calibrateOdom(
        hypoPose.x, hypoPose.y, hypoPose.theta,
        "SelfLocateLocal/DoubleX");
    brain->data->lastSuccessfulLocalizeTime = brain->get_clock()->now();
    brain->log->log("SelfLocateLocal/DoubleX", rerun::TextLog(format("Success. Residual = %.2f", residual)));
    brain->log->log(
        "field/recal",
        rerun::Arrows2D::from_vectors({{hypoPose.x - brain->data->robotPoseToField.x, -hypoPose.y + brain->data->robotPoseToField.y}})
            .with_origins({{brain->data->robotPoseToField.x, - brain->data->robotPoseToField.y}})
            .with_colors(0x00FF00FF)
            .with_radii(0.01)
            .with_draw_order(10)
            .with_labels({"2x"})
    );
    return true;
}

NodeStatus SelfLocateLocal::tick()
{
    if (brain->isRecoveryLocalizationBlocked() ||
        brain->isPostRecoveryHeadingRealignActive()) return NodeStatus::SUCCESS;

    double interval = getInput<double>("msecs_interval").value();
    if (brain->msecsSince(brain->data->lastSuccessfulLocalizeTime) < interval) return NodeStatus::SUCCESS;

    if (_singlePenalty()) return NodeStatus::SUCCESS;
    if (_doubleX()) return NodeStatus::SUCCESS;
    // TODO other features

    // All Features failed
    return NodeStatus::SUCCESS;
}

NodeStatus SelfLocate1P::tick()
{
    if (brain->isRecoveryLocalizationBlocked() ||
        brain->isPostRecoveryHeadingRealignActive()) return NodeStatus::SUCCESS;

    double interval = getInput<double>("msecs_interval").value();
    double maxDist = getInput<double>("max_dist").value();
    if (brain->client->isStandingStill(2000)) maxDist *= 1.5; // Allow a larger distance while stationary.
    double maxDrift = getInput<double>("max_drift").value();
    bool validate = getInput<bool>("validate").value();
    
    auto log = brain->log;
    log->setTimeNow();
    string logPathS = "/locate/1p/success";
    string logPathF = "/locate/1p/fail";

    auto msecs = brain->msecsSince(brain->data->lastSuccessfulLocalizeTime);
    if (msecs < interval){
        log->logThrottled(logPathF, brain->config->rerunLogDebugHz, logPathF, rerun::TextLog(format("Failed, msecs(%.1f) < interval(%.1f)", msecs, interval)));
        return NodeStatus::SUCCESS;
    }

    auto penaltyPoints = brain->data->getMarkingsByType({"PenaltyPoint"});
    if (penaltyPoints.size() != 1) {
        log->logThrottled(logPathF, brain->config->rerunLogDebugHz, logPathF,
            rerun::TextLog(format("Failed, penaltyCnt(%d) != 1", penaltyPoints.size()))
        );
        return NodeStatus::SUCCESS;
    }

    auto pp = penaltyPoints[0];
    if (pp.range > maxDist) {
        log->logThrottled(logPathF, brain->config->rerunLogDebugHz, logPathF,
            rerun::TextLog(format("Failed, penalty Dist(%.2f) > maxDist(%.2f)", pp.range, maxDist))
        );
        return NodeStatus::SUCCESS;
    }

    if (!brain->isBoundingBoxInCenter(pp.boundingBox)) {
        log->logThrottled(logPathF, brain->config->rerunLogDebugHz, logPathF,
            rerun::TextLog(format("Failed, boundingbox is not in the center area"))
        );
        return NodeStatus::SUCCESS;
    }

    // Identify the penalty mark.
    auto fd = brain->config->fieldDimensions;
    Point2D ppo({fd.length / 2 - fd.penaltyDist, 0}); // Opponent penalty-mark map position
    Point2D pps({-fd.length / 2 + fd.penaltyDist, 0}); // Our penalty-mark map position
    double dx, dy; // Offset: expected minus observed

    double disto = norm(pp.posToField.x - ppo.x, pp.posToField.y - ppo.y); // Observation error for opponent mark
    double dists = norm(pp.posToField.x - pps.x, pp.posToField.y - pps.y); // Observation error for our mark

    if (disto < dists && disto < maxDrift) { // Opponent penalty mark
        dx = ppo.x - pp.posToField.x;
        dy = ppo.y - pp.posToField.y;
    }
    else if (dists < disto && dists < maxDrift) {
        dx = pps.x - pp.posToField.x;
        dy = pps.y - pp.posToField.y;
    }
    else {
        log->logThrottled(logPathF, brain->config->rerunLogDebugHz, logPathF,
            rerun::TextLog(format("Failed: disto= %.2f dists=%.2f maxDrift=%.2f", disto, dists, maxDrift))
        );
        return NodeStatus::SUCCESS;
    }

    // Calculate a pose hypothesis from a valid target penalty mark.
    Pose2D hypoPose = brain->data->robotPoseToField;
    hypoPose.x += dx;
    hypoPose.y += dy;

    // validate the hypo with other markers
    auto allMarkers = brain->data->getMarkersForLocator();
    if (allMarkers.size() > 0) {
        double residual = brain->locator->residual(allMarkers, hypoPose) / allMarkers.size();
        if (residual > brain->locator->residualTolerance) { // Validation failed; the penalty mark may be a false positive.
            log->logThrottled(logPathF, brain->config->rerunLogDebugHz, logPathF,
                rerun::TextLog(format("Failed, validation residual(%.2f) > tolerance(%.2f)", residual, brain->locator->residualTolerance))
            );
            return NodeStatus::SUCCESS;
        }
    }

    // else everything is ok, recalibrate with this hypo pose
    double drift = norm(dx, dy);
    brain->log->log(logPathS, rerun::TextLog(format("Success. Dist = %.2f", drift)));
    brain->log->log(
        "field/recal/1p/success",
        rerun::Arrows2D::from_vectors({{hypoPose.x - brain->data->robotPoseToField.x, -hypoPose.y + brain->data->robotPoseToField.y}})
            .with_origins({{brain->data->robotPoseToField.x, - brain->data->robotPoseToField.y}})
            .with_colors(0x00FF00FF)
            .with_radii(0.01)
            .with_draw_order(10)
            .with_labels({"1p"})
    );
    brain->calibrateOdom(
        hypoPose.x, hypoPose.y, hypoPose.theta, "SelfLocate1P");
    brain->data->lastSuccessfulLocalizeTime = brain->get_clock()->now();
    return NodeStatus::SUCCESS;
}

NodeStatus SelfLocate1M::tick()
{
    if (brain->isRecoveryLocalizationBlocked() ||
        brain->isPostRecoveryHeadingRealignActive()) return NodeStatus::SUCCESS;

    double interval = getInput<double>("msecs_interval").value();
    double maxDist = getInput<double>("max_dist").value();
    if (brain->client->isStandingStill(2000)) maxDist *= 1.5; // Allow a larger distance while stationary.
    double maxDrift = getInput<double>("max_drift").value();
    bool validate = getInput<bool>("validate").value();
    
    auto log = brain->log;
    log->setTimeNow();
    string logPathS = "/locate/1m/success";
    string logPathF = "/locate/1m/fail";

    // Rate-limit localization.
    auto msecs = brain->msecsSince(brain->data->lastSuccessfulLocalizeTime);
    if (msecs < interval){
        log->logThrottled(logPathF, brain->config->rerunLogDebugHz, logPathF, rerun::TextLog(format("Failed, msecs(%.1f) < interval(%.1f)", msecs, interval)));
        return NodeStatus::SUCCESS;
    }

    // find nearest marker
    int markerIndex = -1;
    GameObject marker;
    MapMarking mapMarker; 
    double minDist = 100;
    auto markings = brain->data->getMarkings();
    for (int i = 0; i < markings.size(); i++) {
        auto m = markings[i];

        // Reject markings prone to false-positive localization.
        if (m.name == "LOLG" || m.name == "LORG" || m.name == "LSLG" || m.name == "LSRG") continue; 

        if (m.range < minDist) {
            minDist = m.range;
            markerIndex = i;
            marker = m;
        }
    }
    
    if (
        markerIndex < 0 || markerIndex >= markings.size()
        || marker.id < 0 || marker.id >= brain->config->mapMarkings.size()
    ) {
        log->logThrottled(logPathF, brain->config->rerunLogDebugHz, logPathF, rerun::TextLog("Failed, No markings Found. Or marker id invalid."));
        return NodeStatus::SUCCESS;
    }
    mapMarker = brain->config->mapMarkings[marker.id];

    // Reject distant observations because ranging becomes inaccurate.
    if (marker.range > maxDist) {
        log->logThrottled(logPathF, brain->config->rerunLogDebugHz, logPathF,
            rerun::TextLog(format("Failed, min marker Dist(%.2f) > maxDist(%.2f)", marker.range, maxDist))
        );
        return NodeStatus::SUCCESS;
    }

    // Reject observations near image edges where distortion reduces ranging accuracy.
    if (!brain->isBoundingBoxInCenter(marker.boundingBox)) {
        log->logThrottled(logPathF, brain->config->rerunLogDebugHz, logPathF,
            rerun::TextLog(format("Failed, boundingbox is not in the center area"))
        );
        return NodeStatus::SUCCESS;
    }

    double dx, dy; // Offset: expected minus observed
    dx = mapMarker.x - marker.posToField.x;
    dy = mapMarker.y - marker.posToField.y;

    // Reject excessive offsets, which may indicate a false positive.
    double drift = norm(dx, dy);
    if (drift > maxDrift) {
        log->logThrottled(logPathF, brain->config->rerunLogDebugHz, logPathF,
            rerun::TextLog(format("Failed, drift(%.2f) > maxDrift(%.2f)", drift, maxDrift))
        );
        return NodeStatus::SUCCESS;
    }
    
    // Calculate a pose hypothesis from this point.
    Pose2D hypoPose = brain->data->robotPoseToField;
    hypoPose.x += dx;
    hypoPose.y += dy;

    // validate the hypo with other markers
    auto allMarkers = brain->data->getMarkersForLocator();
    if (allMarkers.size() > 0) {
        double residual = brain->locator->residual(allMarkers, hypoPose) / allMarkers.size();
        if (residual > brain->locator->residualTolerance) { // Validation failed; the marking may be a false positive.
            log->logThrottled(logPathF, brain->config->rerunLogDebugHz, logPathF,
                rerun::TextLog(format("Failed, validation residual(%.2f) > tolerance(%.2f)", residual, brain->locator->residualTolerance))
            );
            return NodeStatus::SUCCESS;
        }
    }

    // else everything is ok, recalibrate with this hypo pose
    brain->log->log(logPathS, rerun::TextLog(format("Success. Drift = %.2f", drift)));
    brain->log->log(
        "field/recal/1m/success",
        rerun::Arrows2D::from_vectors({{hypoPose.x - brain->data->robotPoseToField.x, -hypoPose.y + brain->data->robotPoseToField.y}})
            .with_origins({{brain->data->robotPoseToField.x, - brain->data->robotPoseToField.y}})
            .with_colors(0x00FF00FF)
            .with_radii(0.01)
            .with_draw_order(10)
            .with_labels({marker.name})
    );
    brain->calibrateOdom(
        hypoPose.x, hypoPose.y, hypoPose.theta, "SelfLocate1M");
    brain->data->lastSuccessfulLocalizeTime = brain->get_clock()->now();
    return NodeStatus::SUCCESS;
}

NodeStatus SelfLocate2X::tick()
{
    if (brain->isRecoveryLocalizationBlocked() ||
        brain->isPostRecoveryHeadingRealignActive()) return NodeStatus::SUCCESS;

    double interval = getInput<double>("msecs_interval").value();
    double maxDist = getInput<double>("max_dist").value();
    if (brain->client->isStandingStill(2000)) maxDist *= 1.5; // Allow a larger distance while stationary.
    double maxDrift = getInput<double>("max_drift").value();
    bool validate = getInput<bool>("validate").value();
    
    auto log = brain->log;
    log->setTimeNow();
    string logPathS = "/locate/2x/success";
    string logPathF = "/locate/2x/fail";

    auto msecs = brain->msecsSince(brain->data->lastSuccessfulLocalizeTime);
    if (msecs < interval){
        log->logThrottled(logPathF, brain->config->rerunLogDebugHz, logPathF, rerun::TextLog(format("Failed, msecs(%.1f) < interval(%.1f)", msecs, interval)));
        return NodeStatus::SUCCESS;
    }

    auto points = brain->data->getMarkingsByType({"XCross"});
    if (points.size() != 2) {
        log->logThrottled(logPathF, brain->config->rerunLogDebugHz, logPathF,
            rerun::TextLog(format("Failed, point cnt(%d) != 2", points.size()))
        );
        return NodeStatus::SUCCESS;
    }

    auto p0 = points[0]; auto p1 = points[1];
    
    if (p0.range > maxDist || p1.range > maxDist) { // Too far away.
        log->logThrottled(logPathF, brain->config->rerunLogDebugHz, logPathF,
            rerun::TextLog(format("Failed, p0 range (%.2f) or p1 range (%.2f) > maxDist(%.2f)", p0.range, p1.range, maxDist))
        );
        return NodeStatus::SUCCESS;
    }

    double xDist = fabs(p0.posToField.x - p1.posToField.x);
    if (xDist > 0.5) { // Incorrect orientation.
        log->logThrottled(logPathF, brain->config->rerunLogDebugHz, logPathF,
            rerun::TextLog(format("Failed, xDist(%.2f) > maxDist(%.2f)", xDist, 0.5))
        );
        return NodeStatus::SUCCESS;
    }

    double yDist = fabs(p0.posToField.y - p1.posToField.y);
    double mapYDist = brain->config->fieldDimensions.circleRadius * 2.0;
    if (fabs(yDist - mapYDist) > 0.5) { // Incorrect separation.
        log->logThrottled(logPathF, brain->config->rerunLogDebugHz, logPathF,
            rerun::TextLog(format("Failed, yDist(%.2f) too far (%.2f) from mapYDist(%.2f)", yDist, 0.5, mapYDist))
        );
        return NodeStatus::SUCCESS;
    }

    // Expected-minus-observed offset.
    double dx = - (p0.posToField.x + p1.posToField.x) / 2.0;
    double dy = - (p1.posToField.y + p1.posToField.y) / 2.0;
    double drift = norm(dx, dy);

    if (drift > maxDrift) { // Correction is too large.
        log->logThrottled(logPathF, brain->config->rerunLogDebugHz, logPathF,
            rerun::TextLog(format("Failed, dirft(%.2f) > maxDrift(%.2f)", drift, maxDrift))
        );
        return NodeStatus::SUCCESS;
    }

    // Validation passed; calculate a pose hypothesis from this point.
    Pose2D hypoPose = brain->data->robotPoseToField;
    hypoPose.x += dx;
    hypoPose.y += dy;

    // validate the hypo with other markers
    auto allMarkers = brain->data->getMarkersForLocator();
    if (allMarkers.size() > 0) {
        double residual = brain->locator->residual(allMarkers, hypoPose) / allMarkers.size();
        if (residual > brain->locator->residualTolerance) { // Validation failed; the marking may be a false positive.
            log->logThrottled(logPathF, brain->config->rerunLogDebugHz, logPathF,
                rerun::TextLog(format("Failed, validation residual(%.2f) > tolerance(%.2f)", residual, brain->locator->residualTolerance))
            );
            return NodeStatus::SUCCESS;
        }
    }

    // else everything is ok, recalibrate with this hypo pose
    brain->log->log(logPathS, rerun::TextLog(format("Success. Dist = %.2f", drift)));
    brain->log->log(
        "field/recal/2x/success",
        rerun::Arrows2D::from_vectors({{hypoPose.x - brain->data->robotPoseToField.x, -hypoPose.y + brain->data->robotPoseToField.y}})
            .with_origins({{brain->data->robotPoseToField.x, - brain->data->robotPoseToField.y}})
            .with_colors(0x00FF00FF)
            .with_radii(0.01)
            .with_draw_order(10)
            .with_labels({"1p"})
    );
    brain->calibrateOdom(
        hypoPose.x, hypoPose.y, hypoPose.theta, "SelfLocate2X");
    brain->data->lastSuccessfulLocalizeTime = brain->get_clock()->now();
    return NodeStatus::SUCCESS;
}

NodeStatus SelfLocate2T::tick()
{
    if (brain->isRecoveryLocalizationBlocked() ||
        brain->isPostRecoveryHeadingRealignActive()) return NodeStatus::SUCCESS;

    double interval = getInput<double>("msecs_interval").value();
    double maxDist = getInput<double>("max_dist").value();
    if (brain->client->isStandingStill(2000)) maxDist *= 1.5; // Allow a larger distance while stationary.
    double maxDrift = getInput<double>("max_drift").value();
    bool validate = getInput<bool>("validate").value();
    
    auto log = brain->log;
    log->setTimeNow();
    string logPathS = "/locate/2t/success";
    string logPathF = "/locate/2t/fail";

    auto msecs = brain->msecsSince(brain->data->lastSuccessfulLocalizeTime);
    if (msecs < interval){
        log->logThrottled(logPathF, brain->config->rerunLogDebugHz, logPathF, rerun::TextLog(format("Failed, msecs(%.1f) < interval(%.1f)", msecs, interval)));
        return NodeStatus::SUCCESS;
    }

    auto markers = brain->data->getMarkingsByType({"TCross"});
    GameObject m1, m2;
    bool found = false;
    auto fd = brain->config->fieldDimensions;
    for (int i = 0; i < markers.size(); i++) {
        m1 = markers[i];
        
        if (m1.range > maxDist) continue; // Too far away.

        for (int j = i + 1; j < markers.size(); j++) {
            m2 = markers[j];

            if (m2.range > maxDist) continue; // Too far away.

            if (
                fabs(m1.posToField.x - m2.posToField.x) < 0.3
                && fabs(fabs(m1.posToField.y - m2.posToField.y) - fabs(fd.goalAreaWidth - fd.penaltyAreaWidth)/2.0)< 0.3
            ) {
                found = true;
                break;
            }
        }
        if (found) break;
    }


    if (!found) {
        log->logThrottled(logPathF, brain->config->rerunLogDebugHz, logPathF, rerun::TextLog(format("Failed, No pattern within maxDist(%.2f) Found", maxDist)));
        return NodeStatus::SUCCESS;
    }

    Point2D pos_o = { // _o for observed
        (m1.posToField.x + m2.posToField.x)/2,
        (m1.posToField.y + m2.posToField.y)/2
    };
    Point2D pos_m; // _m for map

    vector<double> halfs = {-1, 1};
    vector<double> sides = {-1, 1};
    bool matched = false;
    for (auto half: halfs) {
        for (auto side: sides) {
            pos_m = {
                half * (fd.length / 2.0), 
                side * (fd.penaltyAreaWidth + fd.goalAreaWidth) / 4.0
            };
            double dist = norm(pos_o.x - pos_m.x, pos_o.y - pos_m.y);
            if (dist < maxDrift) {
                matched = true;
                break;
            }
        }
        if (matched) break;
    }

    if (!matched) {
        log->logThrottled(logPathF, brain->config->rerunLogDebugHz, logPathF, rerun::TextLog(format("Failed, can not match to any map positions within maxDrift(%.2f)", maxDrift)));
        return NodeStatus::SUCCESS;
    }

    // Expected-minus-observed offset.
    double dx = pos_m.x - pos_o.x;
    double dy = pos_m.y - pos_o.y;
    double drift = norm(dx, dy);

    // Validation passed; calculate a pose hypothesis from this point.
    Pose2D hypoPose = brain->data->robotPoseToField;
    hypoPose.x += dx;
    hypoPose.y += dy;

    // validate the hypo with other markers
    auto allMarkers = brain->data->getMarkersForLocator();
    if (allMarkers.size() > 0) {
        double residual = brain->locator->residual(allMarkers, hypoPose) / allMarkers.size();
        if (residual > brain->locator->residualTolerance) { // Validation failed; the marking may be a false positive.
            log->logThrottled(logPathF, brain->config->rerunLogDebugHz, logPathF,
                rerun::TextLog(format("Failed, validation residual(%.2f) > tolerance(%.2f)", residual, brain->locator->residualTolerance))
            );
            return NodeStatus::SUCCESS;
        }
    }

    // else everything is ok, recalibrate with this hypo pose
    brain->log->log(logPathS, rerun::TextLog(format("Success. Dist = %.2f", drift)));
    brain->log->log(
        "field/recal/2t/success",
        rerun::Arrows2D::from_vectors({{hypoPose.x - brain->data->robotPoseToField.x, -hypoPose.y + brain->data->robotPoseToField.y}})
            .with_origins({{brain->data->robotPoseToField.x, - brain->data->robotPoseToField.y}})
            .with_colors(0x00FF00FF)
            .with_radii(0.01)
            .with_draw_order(10)
            .with_labels({"2t"})
    );
    brain->calibrateOdom(
        hypoPose.x, hypoPose.y, hypoPose.theta, "SelfLocate2T");
    brain->data->lastSuccessfulLocalizeTime = brain->get_clock()->now();
    return NodeStatus::SUCCESS;
}

NodeStatus SelfLocateLT::tick()
{
    if (brain->isRecoveryLocalizationBlocked() ||
        brain->isPostRecoveryHeadingRealignActive()) return NodeStatus::SUCCESS;

    double interval = getInput<double>("msecs_interval").value();
    double maxDist = getInput<double>("max_dist").value();
    if (brain->client->isStandingStill(2000)) maxDist *= 1.5; // Allow a larger distance while stationary.
    double maxDrift = getInput<double>("max_drift").value();
    bool validate = getInput<bool>("validate").value();
    
    auto log = brain->log;
    log->setTimeNow();
    string logPathS = "/locate/lt/success";
    string logPathF = "/locate/lt/fail";

    auto msecs = brain->msecsSince(brain->data->lastSuccessfulLocalizeTime);
    if (msecs < interval){
        log->logThrottled(logPathF, brain->config->rerunLogDebugHz, logPathF, rerun::TextLog(format("Failed, msecs(%.1f) < interval(%.1f)", msecs, interval)));
        return NodeStatus::SUCCESS;
    }

    auto tMarkers = brain->data->getMarkingsByType({"TCross"});
    auto lMarkers = brain->data->getMarkingsByType({"LCross"});

    GameObject t, l;
    bool found = false;
    auto fd = brain->config->fieldDimensions;
    for (int i = 0; i < tMarkers.size(); i++) {
        t = tMarkers[i];
        
        if (t.range > maxDist) continue; // Too far away.

        for (int j = i + 1; j < lMarkers.size(); j++) {
            l = lMarkers[j];

            if (l.range > maxDist) continue;

            if (
                fabs(t.posToField.y - l.posToField.y) < 0.3
                && fabs(fabs(t.posToField.x - l.posToField.x) - fd.goalAreaLength)< 0.3
            ) {
                found = true;
                break;
            }
        }

        if (found) break;
    }


    if (!found) {
        log->logThrottled(logPathF, brain->config->rerunLogDebugHz, logPathF, rerun::TextLog(format("Failed, No pattern within MaxDist(%.2f) Found", maxDist)));
        return NodeStatus::SUCCESS;
    }

    Point2D pos_o = { // _o for observed
        (t.posToField.x + l.posToField.x)/2,
        (t.posToField.y + l.posToField.y)/2
    };
    Point2D pos_m; // _m for map

    vector<double> halfs = {-1, 1};
    vector<double> sides = {-1, 1};
    bool matched = false;
    for (auto half: halfs) {
        for (auto side: sides) {
            pos_m = {
                half * (fd.length / 2.0 - fd.goalAreaLength / 2.0), 
                side * (fd.goalAreaWidth / 2.0)
            };
            double dist = norm(pos_o.x - pos_m.x, pos_o.y - pos_m.y);
            if (dist < maxDrift) {
                matched = true;
                break;
            }
        }
        if (matched) break;
    }
    if (!matched) {
        log->logThrottled(logPathF, brain->config->rerunLogDebugHz, logPathF, rerun::TextLog(format("Failed, can not match to any map positions within maxDrift(%.2f)", maxDrift)));
        return NodeStatus::SUCCESS;
    }

    // Expected-minus-observed offset.
    double dx = pos_m.x - pos_o.x;
    double dy = pos_m.y - pos_o.y;
    double drift = norm(dx, dy);

    // Validation passed; calculate a pose hypothesis from this point.
    Pose2D hypoPose = brain->data->robotPoseToField;
    hypoPose.x += dx;
    hypoPose.y += dy;

    // validate the hypo with other markers
    auto allMarkers = brain->data->getMarkersForLocator();
    if (allMarkers.size() > 0) {
        double residual = brain->locator->residual(allMarkers, hypoPose) / allMarkers.size();
        if (residual > brain->locator->residualTolerance) { // Validation failed; the marking may be a false positive.
            log->logThrottled(logPathF, brain->config->rerunLogDebugHz, logPathF,
                rerun::TextLog(format("Failed, validation residual(%.2f) > tolerance(%.2f)", residual, brain->locator->residualTolerance))
            );
            return NodeStatus::SUCCESS;
        }
    }

    // else everything is ok, recalibrate with this hypo pose
    brain->log->log(logPathS, rerun::TextLog(format("Success. Dist = %.2f", drift)));
    brain->log->log(
        "field/recal/lt/success",
        rerun::Arrows2D::from_vectors({{hypoPose.x - brain->data->robotPoseToField.x, -hypoPose.y + brain->data->robotPoseToField.y}})
            .with_origins({{brain->data->robotPoseToField.x, - brain->data->robotPoseToField.y}})
            .with_colors(0x00FF00FF)
            .with_radii(0.01)
            .with_draw_order(10)
            .with_labels({"2t"})
    );
    brain->calibrateOdom(
        hypoPose.x, hypoPose.y, hypoPose.theta, "SelfLocateLT");
    brain->data->lastSuccessfulLocalizeTime = brain->get_clock()->now();
    return NodeStatus::SUCCESS;
}

NodeStatus SelfLocatePT::tick()
{
    if (brain->isRecoveryLocalizationBlocked() ||
        brain->isPostRecoveryHeadingRealignActive()) return NodeStatus::SUCCESS;

    double interval = getInput<double>("msecs_interval").value();
    double maxDist = getInput<double>("max_dist").value();
    if (brain->client->isStandingStill(2000)) maxDist *= 1.5; // Allow a larger distance while stationary.
    double maxDrift = getInput<double>("max_drift").value();
    bool validate = getInput<bool>("validate").value();
    
    auto log = brain->log;
    log->setTimeNow();
    string logPathS = "/locate/pt/success";
    string logPathF = "/locate/pt/fail";

    auto msecs = brain->msecsSince(brain->data->lastSuccessfulLocalizeTime);
    if (msecs < interval){
        log->logThrottled(logPathF, brain->config->rerunLogDebugHz, logPathF, rerun::TextLog(format("Failed, msecs(%.1f) < interval(%.1f)", msecs, interval)));
        return NodeStatus::SUCCESS;
    }

    auto posts = brain->data->getGoalposts();
    auto tMarkers = brain->data->getMarkingsByType({"TCross"});
    
    GameObject p, t;
    bool found = false;
    auto fd = brain->config->fieldDimensions;
    for (int i = 0; i < posts.size(); i++) {
        p = posts[i];
        if (p.range > maxDist) continue;

        for (int j = i + 1; j < tMarkers.size(); j++) {
            t = tMarkers[j];
            if (t.range > maxDist) continue;
            if (
                fabs(t.posToField.x - p.posToField.x) < 0.5
                && fabs(fabs(t.posToField.x - p.posToField.x) - fabs(fd.goalAreaWidth - fd.goalWidth) / 2.0) < 0.3
            ) {
                found = true;
                break;
            }
        }
        
        if (found) break;
    }


    if (!found) {
        log->logThrottled(logPathF, brain->config->rerunLogDebugHz, logPathF, rerun::TextLog(format("Failed, No pattern within maxDist(%.2f) Found", maxDist)));
        return NodeStatus::SUCCESS;
    }

    Point2D pos_o = { // _o for observed
        t.posToField.x,
        t.posToField.y
    };
    Point2D pos_m; // _m for map

    vector<double> halfs = {-1, 1};
    vector<double> sides = {-1, 1};
    bool matched = false;
    for (auto half: halfs) {
        for (auto side: sides) {
            pos_m = {
                half * (fd.length), 
                side * (fd.goalAreaWidth / 2.0)
            };
            double dist = norm(pos_o.x - pos_m.x, pos_o.y - pos_m.y);
            if (dist < maxDrift) {
                matched = true;
                break;
            }
        }
        if (matched) break;
    }
    if (!matched) {
        log->logThrottled(logPathF, brain->config->rerunLogDebugHz, logPathF, rerun::TextLog(format("Failed, can not match to any map positions within maxDrift(%.2f)", maxDrift)));
        return NodeStatus::SUCCESS;
    }

    // Expected-minus-observed offset.
    double dx = pos_m.x - pos_o.x;
    double dy = pos_m.y - pos_o.y;
    double drift = norm(dx, dy);

    // Validation passed; calculate a pose hypothesis from this point.
    Pose2D hypoPose = brain->data->robotPoseToField;
    hypoPose.x += dx;
    hypoPose.y += dy;

    // validate the hypo with other markers
    auto allMarkers = brain->data->getMarkersForLocator();
    if (allMarkers.size() > 0) {
        double residual = brain->locator->residual(allMarkers, hypoPose) / allMarkers.size();
        if (residual > brain->locator->residualTolerance) { // Validation failed; the marking may be a false positive.
            log->logThrottled(logPathF, brain->config->rerunLogDebugHz, logPathF,
                rerun::TextLog(format("Failed, validation residual(%.2f) > tolerance(%.2f)", residual, brain->locator->residualTolerance))
            );
            return NodeStatus::SUCCESS;
        }
    }

    // else everything is ok, recalibrate with this hypo pose
    brain->log->log(logPathS, rerun::TextLog(format("Success. Dist = %.2f", drift)));
    brain->log->log(
        "field/recal/pt/success",
        rerun::Arrows2D::from_vectors({{hypoPose.x - brain->data->robotPoseToField.x, -hypoPose.y + brain->data->robotPoseToField.y}})
            .with_origins({{brain->data->robotPoseToField.x, - brain->data->robotPoseToField.y}})
            .with_colors(0x00FF00FF)
            .with_radii(0.01)
            .with_draw_order(10)
            .with_labels({"2t"})
    );
    brain->calibrateOdom(
        hypoPose.x, hypoPose.y, hypoPose.theta, "SelfLocatePT");
    brain->data->lastSuccessfulLocalizeTime = brain->get_clock()->now();
    return NodeStatus::SUCCESS;
}

NodeStatus SelfLocateBorder::tick()
{
    if (brain->isRecoveryLocalizationBlocked() ||
        brain->isPostRecoveryHeadingRealignActive()) return NodeStatus::SUCCESS;

    double interval = getInput<double>("msecs_interval").value();
    double maxDist = getInput<double>("max_dist").value();
    if (brain->client->isStandingStill(2000)) maxDist *= 1.5; // Allow a larger distance while stationary.
    double maxDrift = getInput<double>("max_drift").value();
    bool validate = getInput<bool>("validate").value();
    
    auto log = brain->log;
    log->setTimeNow();
    string logPathS = "/locate/border/success";
    string logPathF = "/locate/border/fail";

    // Rate-limit localization.
    auto msecs = brain->msecsSince(brain->data->lastSuccessfulLocalizeTime);
    if (msecs < interval){
        log->logThrottled(logPathF, brain->config->rerunLogDebugHz, logPathF, rerun::TextLog(format("Failed, msecs(%.1f) < interval(%.1f)", msecs, interval)));
        return NodeStatus::SUCCESS;
    }

    // Line-based localization is unstable while moving.
    // if (!brain->client->isStandingStill(1000)) {
    //     log->log(logPathF, rerun::TextLog(format("Failed, Not Standing Still")));
    //     return NodeStatus::SUCCESS;
    // }
    
    // find best touchline and best goalline
    bool touchLineFound = false;
    FieldLine touchLine;
    double bestConfidenceTouchline = 0.0;
    bool goalLineFound = false;
    FieldLine goalLine;
    double bestConfidenceGoalline = 0.0;
    bool middleLineFound = false;
    FieldLine middleLine;
    double bestConfidenceMiddleLine = 0.0;

    auto fieldLines = brain->data->getFieldLines();
    for (int i = 0; i < fieldLines.size(); i++) {
        auto line = fieldLines[i];
        if (line.type != LineType::TouchLine && line.type != LineType::GoalLine && line.type != LineType::MiddleLine) continue;
        if (line.confidence < 0.8) continue;
        
        double dist = pointMinDistToLine(
            Point2D({brain->data->robotPoseToField.x, brain->data->robotPoseToField.y}), 
            line.posToField
        );
        if (dist > maxDist) continue;

        if (line.type == LineType::TouchLine) {
           if (line.confidence > bestConfidenceTouchline) {
               bestConfidenceTouchline = line.confidence;
               touchLine = line;
               touchLineFound = true;
           }
        } else if (line.type == LineType::GoalLine) {
            if (line.confidence > bestConfidenceGoalline) {
                bestConfidenceGoalline = line.confidence;
                goalLine = line;
                goalLineFound = true;
            }
        } else if (line.type == LineType::MiddleLine) {
            if (line.confidence > bestConfidenceMiddleLine) {
                bestConfidenceMiddleLine = line.confidence;
                middleLine = line;
                middleLineFound = true;
            }
        }
    }

    // Calculate the correction.
    double dx = 0; 
    double dy = 0; 
    auto fd = brain->config->fieldDimensions;
    if (touchLineFound) {
       double y_m = touchLine.side == LineSide::Left ? fd.width / 2.0 : - fd.width / 2.0;
       double perpDist = pointPerpDistToLine(
           Point2D({brain->data->robotPoseToField.x, brain->data->robotPoseToField.y}),
           touchLine.posToField
       );
       double y_o = touchLine.side == LineSide::Left ? 
           brain->data->robotPoseToField.y - perpDist :
           brain->data->robotPoseToField.y + perpDist;
       dy = y_m - y_o;
    }
    if (goalLineFound) {
        double x_m = goalLine.half == LineHalf::Opponent ? fd.length / 2.0: - fd.length / 2.0;
        double perpDist = pointPerpDistToLine(
            Point2D({brain->data->robotPoseToField.x, brain->data->robotPoseToField.y}),
            goalLine.posToField
        );
        double x_o = goalLine.half == LineHalf::Opponent?
            brain->data->robotPoseToField.x - perpDist :
            brain->data->robotPoseToField.x + perpDist;
        dx = x_m - x_o;
    } else if (middleLineFound) {
        double x_m = 0;
        auto linePos = middleLine.posToField;
        auto robotPose = brain->data->robotPoseToField;
        vector<double> pointA(2);
        vector<double> pointB(2);
        vector<double> pointR = {robotPose.x, robotPose.y};

        if (linePos.y0 > linePos.y1) {
            pointA = {linePos.x0, linePos.y0};
            pointB = {linePos.x1, linePos.y1};
        } else {
            pointA = {linePos.x1, linePos.y1};
            pointB = {linePos.x0, linePos.y0};
        }

        vector<double> vl = {pointB[0] - pointA[0], pointB[1] - pointA[1]};
        vector<double> vr = {pointR[0] - pointA[0], pointR[1] - pointA[1]};

        double normvl = norm(vl);
        double normvr = norm(vr);
        if (normvl < 1e-3 || normvr < 1e-3) {
            dx = 10000; // a large enough number that will certainly be bigger than max drift
        } else {
            double dist = crossProduct(vr, vl) / normvl;
            double x_o = robotPose.x + dist;
            dx = x_m - x_o;
        }
    }

    // No match found.
    if ((!touchLineFound && !goalLineFound && !middleLineFound)) {
        log->logThrottled(logPathF, brain->config->rerunLogDebugHz, logPathF,
            rerun::TextLog("No touchline or goalline or middleLine found.")
        );
        return NodeStatus::SUCCESS;
    }

    // Reject excessive offsets, which may indicate a false positive.
    double drift = norm(dx, dy);
    if (drift > maxDrift) {
        log->logThrottled(logPathF, brain->config->rerunLogDebugHz, logPathF,
            rerun::TextLog(format("Failed, drift(%.2f) > maxDrift(%.2f)", drift, maxDrift))
        );
        return NodeStatus::SUCCESS;
    }
    
    // Calculate a pose hypothesis from this point.
    Pose2D hypoPose = brain->data->robotPoseToField;
    hypoPose.x += dx;
    hypoPose.y += dy;

    // validate the hypo with other markers
    auto allMarkers = brain->data->getMarkersForLocator();
    if (allMarkers.size() > 0) {
        double residual = brain->locator->residual(allMarkers, hypoPose) / allMarkers.size();
        if (residual > brain->locator->residualTolerance) { // Validation failed; the marking may be a false positive.
            log->logThrottled(logPathF, brain->config->rerunLogDebugHz, logPathF,
                rerun::TextLog(format("Failed, validation residual(%.2f) > tolerance(%.2f)", residual, brain->locator->residualTolerance))
            );
            return NodeStatus::SUCCESS;
        }
    }

    // else everything is ok, recalibrate with this hypo pose
    brain->log->log(logPathS, rerun::TextLog(format("Success. Drift = %.2f", drift)));
    string label = "";
    if (touchLineFound) label += "TouchLine";
    if (touchLineFound && (goalLineFound || middleLineFound)) label += " ";
    if (goalLineFound) label += "GoalLine";
    if (middleLineFound) label += "MiddleLine";
    brain->log->log(
        "field/recal/border/success",
        rerun::Arrows2D::from_vectors({{hypoPose.x - brain->data->robotPoseToField.x, -hypoPose.y + brain->data->robotPoseToField.y}})
            .with_origins({{brain->data->robotPoseToField.x, - brain->data->robotPoseToField.y}})
            .with_colors(0x00FF00FF)
            .with_radii(0.01)
            .with_draw_order(10)
            .with_labels({label})
    );
    brain->calibrateOdom(
        hypoPose.x, hypoPose.y, hypoPose.theta, "SelfLocateBorder");
    brain->data->lastSuccessfulLocalizeTime = brain->get_clock()->now();
    return NodeStatus::SUCCESS;
}
double SelfLocateLine::lineToLineAvgDist(const FieldLine& a, const FieldLineRef& b, int samples) {
    double sum = 0.0;
    for (int i = 0; i <= samples; ++i) {
        double t = double(i) / samples;
        double x = a.posToField.x0 + t * (a.posToField.x1 - a.posToField.x0);
        double y = a.posToField.y0 + t * (a.posToField.y1 - a.posToField.y0);
        // Shortest distance from point (x, y) to segment B.
        double dx = b.x1 - b.x0;
        double dy = b.y1 - b.y0;
        double len2 = dx*dx + dy*dy;
        double t_proj = ((x - b.x0) * dx + (y - b.y0) * dy) / (len2 + 1e-8);
        t_proj = std::max(0.0, std::min(1.0, t_proj));
        double px = b.x0 + t_proj * dx;
        double py = b.y0 + t_proj * dy;
        double dist = sqrt((x - px)*(x - px) + (y - py)*(y - py));
        sum += dist;
    }
    return sum / (samples + 1);
}

NodeStatus SelfLocateLine::tick()
{
    if (brain->isRecoveryLocalizationBlocked() ||
        brain->isPostRecoveryHeadingRealignActive()) return NodeStatus::SUCCESS;

    double interval = getInput<double>("msecs_interval").value();
    double maxDist = getInput<double>("max_dist").value();
    if (brain->client->isStandingStill(2000)) maxDist *= 1.5; // Allow a larger distance while stationary.
    double maxDrift = getInput<double>("max_drift").value();
    bool validate = getInput<bool>("validate").value();
    
    auto log = brain->log;
    log->setTimeNow();
    string logPathS = "/locate/line/success";
    string logPathF = "/locate/line/fail";

    // Rate-limit localization.
    auto msecs = brain->msecsSince(brain->data->lastSuccessfulLocalizeTime);
    if (msecs < interval){
        log->logThrottled(logPathF, brain->config->rerunLogDebugHz, logPathF, rerun::TextLog(format("Failed, msecs(%.1f) < interval(%.1f)", msecs, interval)));
        return NodeStatus::SUCCESS;
    }

    auto fieldLines = brain->data->getFieldLines();
    // Calculate each line's distance from the robot.
    double dist0;
    double dist1;
    std::vector<std::pair<double, FieldLine>> lineWithDist;
    for (const auto& line : fieldLines) {
        dist0 = norm(
            line.posToField.x0 - brain->data->robotPoseToField.x,
            line.posToField.y0 - brain->data->robotPoseToField.y
        );
        dist1 = norm(
            line.posToField.x1 - brain->data->robotPoseToField.x,
            line.posToField.y1 - brain->data->robotPoseToField.y
        );
        if (dist0 > maxDist || dist1 > maxDist) continue; // Both endpoints must be within maxDist.
        double dist = norm(
            (line.posToField.x0+line.posToField.x1)/2 - brain->data->robotPoseToField.x,
            (line.posToField.y0+line.posToField.y1)/2 - brain->data->robotPoseToField.y
        );
        lineWithDist.emplace_back(dist, line);
    }
    // Sort by distance.
    std::sort(lineWithDist.begin(), lineWithDist.end(),
        [](const auto& a, const auto& b) { return a.first < b.first; }
    );
    // Sorted field lines.
    std::vector<FieldLine> sortedFieldLines;
    for (const auto& p : lineWithDist) {
        sortedFieldLines.push_back(p.second);
    }

    // Calculate the correction.
    double dx = 0; 
    double dy = 0; 
    auto fd = brain->config->fieldDimensions;
    // Use sortedFieldLines from this point onward.
    

    std::vector<FieldLineRef> standardLines = {
        {"bottom", -fd.length/2, -fd.width/2, -fd.length/2, fd.width/2, false}, // Bottom goal line
        {"top", fd.length/2, -fd.width/2, fd.length/2, fd.width/2, false},      // Top goal line
        {"left", -fd.length/2, -fd.width/2, fd.length/2, -fd.width/2, true},   // Left touchline
        {"right", -fd.length/2, fd.width/2, fd.length/2, fd.width/2, true},    // Right touchline
        {"middle", 0, -fd.width/2, 0, fd.width/2, false},                       // Halfway line
        {"top_penalty_left", fd.length / 2, -fd.penaltyAreaWidth / 2, fd.length / 2 - fd.penaltyAreaLength, -fd.penaltyAreaWidth / 2, true},
        {"top_penalty_right", fd.length / 2, fd.penaltyAreaWidth / 2, fd.length / 2 - fd.penaltyAreaLength, fd.penaltyAreaWidth / 2, true},
        {"top_penalty_middle", fd.length / 2 - fd.penaltyAreaLength, -fd.penaltyAreaWidth / 2, fd.length / 2 - fd.penaltyAreaLength, fd.penaltyAreaWidth / 2, false},
        {"top_goal_left", fd.length / 2, -fd.goalAreaWidth / 2, (fd.length / 2 - fd.goalAreaLength), -fd.goalAreaWidth / 2, true},
        {"top_goal_right", fd.length / 2, fd.goalAreaWidth / 2, (fd.length / 2 - fd.goalAreaLength), fd.goalAreaWidth / 2, true},
        {"top_goal_middle", (fd.length / 2 - fd.goalAreaLength), -fd.goalAreaWidth / 2, (fd.length / 2 - fd.goalAreaLength), fd.goalAreaWidth / 2, false},
        {"bottom_penalty_left", -fd.length / 2, -fd.penaltyAreaWidth / 2, -(fd.length / 2 - fd.penaltyAreaLength), -fd.penaltyAreaWidth / 2, true},
        {"bottom_penalty_right", -fd.length / 2, fd.penaltyAreaWidth / 2, -(fd.length / 2 - fd.penaltyAreaLength), fd.penaltyAreaWidth / 2, true},
        {"bottom_penalty_middle", -(fd.length / 2 - fd.penaltyAreaLength), -fd.penaltyAreaWidth / 2, -(fd.length / 2 - fd.penaltyAreaLength), fd.penaltyAreaWidth / 2, false},
        {"bottom_goal_left", -fd.length / 2, -fd.goalAreaWidth / 2, -(fd.length / 2 - fd.goalAreaLength), -fd.goalAreaWidth / 2, true},
        {"bottom_goal_right", -fd.length / 2, fd.goalAreaWidth / 2, -(fd.length / 2 - fd.goalAreaLength), fd.goalAreaWidth / 2, true},
        {"bottom_goal_middle",  -(fd.length / 2 - fd.goalAreaLength), -fd.goalAreaWidth / 2, -(fd.length / 2 - fd.goalAreaLength), fd.goalAreaWidth / 2, false},
    };
    
    // Calculate the distance and angle between two lines.
    auto lineDistanceAngle = [this](const FieldLine& obs, const FieldLineRef& def) -> std::pair<double, double> {
        double dist = this->lineToLineAvgDist(obs, def, 20); // Sample 20 points.

        // Calculate the orientation difference.
        double angle = angleBetweenLines(obs.posToField, {def.x0, def.y0, def.x1, def.y1});
        return std::make_pair(dist, angle);
    };

    FieldLineRef bestLine;
    FieldLineRef secondBestLine;
    FieldLine firstLine;
    FieldLine secondLine;
    bool firstLineFound = false;
    bool secondLineFound = false;
    // Match each observed line.
    for (const auto& line : sortedFieldLines) {
        double minScore = 1e6;
        double angleThreshold = 0.2; // Approximately 11 degrees; tune as needed.
        for (const auto& def : standardLines) {
            auto [dist, angle] = lineDistanceAngle(line, def);
            double score;
            if (angle < angleThreshold) {
                score = dist; // Distance and angle weighted score
            } else {
                score = 1e6; // Orientation mismatch
            }
            if (score < minScore) {
                minScore = score;
                if (!firstLineFound) {
                    bestLine = def;
                    firstLine = line;
                } else if (!secondLineFound) {
                    secondBestLine = def;
                    secondLine = line;
                }
            }
        }
        if (minScore < maxDrift) {
            if (!firstLineFound) {
                firstLineFound = true;
            } else if (!secondLineFound && bestLine.name != secondBestLine.name) {
                secondLineFound = true;
            }
        }
    }

    if (!firstLineFound) {
        log->logThrottled(logPathF, brain->config->rerunLogDebugHz, logPathF, rerun::TextLog("Failed, no matching line found"));
        return NodeStatus::SUCCESS;
    }

    // Calculate dx or dy correction from line orientation.
    if (bestLine.isVertical) {
        // Vertical line: adjust y.
        double dy0 = bestLine.y0 - firstLine.posToField.y0;
        double dy1 = bestLine.y0 - firstLine.posToField.y1;
        dy = fabs(dy0) < fabs(dy1) ? dy0 : dy1;
    } else {
        // Horizontal line: adjust x.
        double dx0 = bestLine.x0 - firstLine.posToField.x0;
        double dx1 = bestLine.x0 - firstLine.posToField.x1;
        dx = fabs(dx0) < fabs(dx1) ? dx0 : dx1;
    }

    // A second line with a different orientation can further refine the correction.
    if (secondLineFound && secondBestLine.isVertical != bestLine.isVertical) {
        if (secondBestLine.isVertical) {
            double dy0 = secondBestLine.y0 - secondLine.posToField.y0;
            double dy1 = secondBestLine.y0 - secondLine.posToField.y1;
            dy = fabs(dy0) < fabs(dy1) ? dy0 : dy1;
        } else {
            double dx0 = secondBestLine.x0 - secondLine.posToField.x0;
            double dx1 = secondBestLine.x0 - secondLine.posToField.x1;
            dx = fabs(dx0) < fabs(dx1) ? dx0 : dx1;
        }
    }


    double drift = norm(dx, dy);
    if (drift > maxDrift) {
        log->logThrottled(logPathF, brain->config->rerunLogDebugHz, logPathF, rerun::TextLog(format("Failed, drift(%.2f) > maxDrift(%.2f)", drift, maxDrift)));
        return NodeStatus::SUCCESS;
    }

    // Calculate a pose hypothesis from this point.
    Pose2D hypoPose = brain->data->robotPoseToField;
    hypoPose.x += dx;
    hypoPose.y += dy;

    // validate the hypo with other markers
    if (validate) {
        auto allMarkers = brain->data->getMarkersForLocator();
        if (allMarkers.size() > 0) {
            double residual = brain->locator->residual(allMarkers, hypoPose) / allMarkers.size();
            if (residual > brain->locator->residualTolerance) {
                log->logThrottled(logPathF, brain->config->rerunLogDebugHz, logPathF,
                    rerun::TextLog(format("Failed, validation residual(%.2f) > tolerance(%.2f)", residual, brain->locator->residualTolerance))
                );
                return NodeStatus::SUCCESS;
            }
        }
    }

    // else everything is ok, recalibrate with this hypo pose
    brain->log->log(logPathS, rerun::TextLog(format("Success. Drift = %.2f", drift)));
    brain->log->log(
        "field/recal/line/success",
        rerun::Arrows2D::from_vectors({{hypoPose.x - brain->data->robotPoseToField.x, -hypoPose.y + brain->data->robotPoseToField.y}})
            .with_origins({{brain->data->robotPoseToField.x, - brain->data->robotPoseToField.y}})
            .with_colors(0x00FF00FF)
            .with_radii(0.01)
            .with_draw_order(10)
            .with_labels({bestLine.name})
    );
    brain->calibrateOdom(
        hypoPose.x, hypoPose.y, hypoPose.theta, "SelfLocateLine");
    brain->data->lastSuccessfulLocalizeTime = brain->get_clock()->now();
    return NodeStatus::SUCCESS;
}


NodeStatus MoveToPoseOnField::tick()
{
    auto log = [=](string msg) {
        // brain->log->setTimeNow();
        // brain->log->log("debug/Move", rerun::TextLog(msg));
    };
    log("Move ticked");

    double tx, ty, ttheta, longRangeThreshold, turnThreshold, vxLimit, vyLimit, vthetaLimit, xTolerance, yTolerance, thetaTolerance;
    getInput("x", tx);
    getInput("y", ty);
    getInput("theta", ttheta);
    getInput("long_range_threshold", longRangeThreshold);
    getInput("turn_threshold", turnThreshold);
    getInput("vx_limit", vxLimit);
    getInput("vx_limit", vxLimit);
    getInput("vy_limit", vyLimit);
    getInput("vtheta_limit", vthetaLimit);
    getInput("x_tolerance", xTolerance);
    getInput("y_tolerance", yTolerance);
    getInput("theta_tolerance", thetaTolerance);
    bool avoidObstacle;
    getInput("avoid_obstacle", avoidObstacle);

    brain->client->moveToPoseOnField2(tx, ty, ttheta, longRangeThreshold, turnThreshold, vxLimit, vyLimit, vthetaLimit, xTolerance, yTolerance, thetaTolerance, avoidObstacle);
    return NodeStatus::SUCCESS;
}

NodeStatus GoToReadyPosition::tick()
{
    auto log = [=](string msg) {
        // brain->log->setTimeNow();
        // brain->log->log("debug/GoToReadyPosition", rerun::TextLog(msg));
    };
    log("GoToReadyPosition ticked");

    double distTolerance, thetaTolerance;
    getInput("dist_tolerance", distTolerance);
    getInput("theta_tolerance", thetaTolerance);
    string role = brain->tree->getEntry<string>("player_role");
    bool isKickoff = brain->tree->getEntry<bool>("gc_is_kickoff_side");
    auto fd = brain->config->fieldDimensions;

    // A stale communication snapshot must not make two robots select the
    // default origin. Four striker slots are available in the 5v5 setup.
    const int readyRank = std::clamp(
        brain->data->myStrikerIDRank, 0, 3);
    int selectedRank = readyRank;
    if (role == "striker" && isKickoff &&
        brain->data->kickoffReadyRank >= 0) {
        selectedRank = std::clamp(brain->data->kickoffReadyRank, 0, 3);
    }

    // default values, override with different conditions
    double tx = 0, ty = 0, ttheta = 0; 
    double longRangeThreshold = 1.0;
    double turnThreshold = 0.4;
    double vxLimit, vyLimit;
    getInput("vx_limit", vxLimit);
    getInput("vy_limit", vyLimit);
    double vthetaLimit = 1.5;
    bool avoidObstacle = true;

    if (role == "goal_keeper") {
        distTolerance = std::max(0.02, brain->get_parameter(
            "goalkeeper.ready.dist_tolerance").as_double());
        thetaTolerance = std::max(0.02, brain->get_parameter(
            "goalkeeper.ready.theta_tolerance").as_double());
        longRangeThreshold = std::max(0.1, brain->get_parameter(
            "goalkeeper.ready.long_range_threshold").as_double());
        turnThreshold = std::max(0.02, brain->get_parameter(
            "goalkeeper.ready.turn_threshold").as_double());
        vxLimit = std::max(0.0, brain->get_parameter(
            "goalkeeper.ready.vx_limit").as_double());
        vyLimit = std::max(0.0, brain->get_parameter(
            "goalkeeper.ready.vy_limit").as_double());
        vthetaLimit = std::max(0.0, brain->get_parameter(
            "goalkeeper.ready.vtheta_limit").as_double());
        avoidObstacle = brain->get_parameter(
            "goalkeeper.ready.avoid_obstacles").as_bool();
    }
    if (brain->distToBorder() > -1.0) { // near border
        vxLimit = std::min(vxLimit, 0.6);
        vyLimit = std::min(vyLimit, 0.4);
    }

    if (role == "striker") {
        const double ownGoalX = -fd.length / 2.0;
        const double safePenaltyFront = ownGoalX +
            fd.penaltyAreaLength + 0.4;
        const double safeOppPenaltyFront = fd.length / 2.0 -
            fd.penaltyAreaLength - 0.4;
        const double safeYLimit = std::max(0.5, fd.width / 2.0 - 0.8);
        const auto safeX = [&](double requested) {
            return std::clamp(
                requested, safePenaltyFront, safeOppPenaltyFront);
        };
        const auto safeY = [&](double requested) {
            return std::clamp(requested, -safeYLimit, safeYLimit);
        };

        if (selectedRank == 0) {
            tx = safeX(isKickoff ? -fd.circleRadius : -fd.circleRadius * 2.0);
            ty = 0.0;
        } else if (selectedRank == 1) {
            tx = safeX(isKickoff ? -fd.circleRadius : -fd.circleRadius * 2.0);
            ty = safeY(-1.5);
        } else if (selectedRank == 2) {
            // Leave a robot-radius margin in front of the own penalty line.
            tx = safeX(ownGoalX + fd.penaltyAreaLength + 0.4);
            ty = safeY(fd.circleRadius);
        } else {
            tx = safeX(ownGoalX + fd.penaltyDist);
            ty = safeY(-fd.circleRadius - 1.0);
        }
    } else if (role == "goal_keeper") {
        // Match the normal blocking line (1 m from the goal line) while
        // remaining inside the goal area during READY.
        tx = -fd.length / 2.0 + std::clamp(
            brain->get_parameter(
                "goalkeeper.ready.dist_to_goalline").as_double(),
            0.4,
            std::max(0.4, fd.goalAreaLength - 0.2));
        ty = 0;
        ttheta = 0;
    }

    brain->client->moveToPoseOnField2(tx, ty, ttheta, longRangeThreshold, turnThreshold, vxLimit, vyLimit, vthetaLimit, distTolerance / 1.5, distTolerance / 1.5, thetaTolerance, avoidObstacle);
    return NodeStatus::SUCCESS;
}

NodeStatus GoBackInField::tick()
{
    const bool logGoBackDebug = brain->log->shouldLog(
        "go_back_in_field_debug", brain->config->rerunLogDebugHz);
    auto log = [=](string msg) {
        if (logGoBackDebug) {
            brain->log->setTimeNow();
            brain->log->log("debug/GoBackInField", rerun::TextLog(msg));
        }
    };
    log("GoBackInField ticked");

    double valve;
    getInput("valve", valve);
    double vx = 0; 
    double vy = 0; 
    double dir = 0;
    auto fd = brain->config->fieldDimensions;
    if (brain->data->robotPoseToField.x > fd.length / 2.0 - valve) dir = - M_PI;
    else if (brain->data->robotPoseToField.x < - fd.length / 2.0 + valve) dir = 0;
    else if (brain->data->robotPoseToField.y > fd.width / 2.0 + valve) dir = - M_PI / 2.0;
    else if (brain->data->robotPoseToField.y < - fd.width / 2.0 - valve) dir = M_PI / 2.0;
    else { // Still in bounds.
        brain->client->setVelocity(0, 0, 0);
        return NodeStatus::SUCCESS;
    }

    // Move back onto the field.
    double dir_r = toPInPI(dir - brain->data->robotPoseToField.theta);
    vx = 0.4 * cos(dir_r);
    vy = 0.4 * sin(dir_r);
    brain->client->setVelocity(vx, vy, 0, false, false, false);
    return NodeStatus::SUCCESS;
}


NodeStatus WaveHand::tick()
{
    string action;
    getInput("action", action);
    if (action == "start")
        brain->client->waveHand(true);
    else
        brain->client->waveHand(false);
    return NodeStatus::SUCCESS;
}

NodeStatus MoveHead::tick()
{
    double pitch, yaw;
    getInput("pitch", pitch);
    getInput("yaw", yaw);
    brain->client->moveHead(pitch, yaw);
    return NodeStatus::SUCCESS;
}





NodeStatus CheckAndStandUp::tick()
{
    const bool logRecoverySnapshot = brain->log->shouldLog(
        "recovery_snapshot_debug", brain->config->rerunLogDebugHz);
    if (brain->tree->getEntry<bool>("gc_is_under_penalty")) {
        brain->data->recoveryPerformedRetryCount = 0;
        brain->data->recoveryPerformed = false;
        if (logRecoverySnapshot) {
            brain->log->setTimeNow();
            brain->log->log("recovery", rerun::TextLog("reset recovery"));
        }
        return NodeStatus::SUCCESS;
    }
    if (logRecoverySnapshot) {
        brain->log->setTimeNow();
        brain->log->log("recovery", rerun::TextLog(format("Recovery retry count: %d, recoveryPerformed: %d recoveryState: %d currentRobotModeIndex: %d", brain->data->recoveryPerformedRetryCount, brain->data->recoveryPerformed, brain->data->recoveryState, brain->data->currentRobotModeIndex)));
    }

    if (!brain->data->recoveryPerformed &&
        brain->data->recoveryState == RobotRecoveryState::HAS_FALLEN &&
        brain->isRecoveryDampingMode() &&
        brain->data->recoveryPerformedRetryCount < brain->get_parameter("recovery.retry_max_count").get_value<int>()) {
        brain->data->shouldExitRLVisionKick = true;
        const int ret = brain->client->standUp();
        if (ret != 0) {
            brain->log->log("recovery", rerun::TextLog(format(
                "standUp request failed: %d", ret)));
            return NodeStatus::FAILURE;
        }
        brain->data->recoveryPerformed = true;
        brain->speak("Trying to stand up");
        brain->log->log("recovery", rerun::TextLog(format("Recovery retry count: %d", brain->data->recoveryPerformedRetryCount)));
        return NodeStatus::FAILURE;
    }

    if (brain->data->recoveryState == RobotRecoveryState::IS_FALLING ||
        brain->data->recoveryState == RobotRecoveryState::HAS_FALLEN ||
        brain->data->recoveryState == RobotRecoveryState::IS_GETTING_UP ||
        brain->isRecoveryModeTransitionActive() ||
        brain->isRecoveryLocalizationBlocked()) {
        return NodeStatus::FAILURE;
    }

    if (brain->data->recoveryState == RobotRecoveryState::IS_READY &&
        brain->isRecoveryOperationalMode()) {
        brain->data->recoveryPerformedRetryCount = 0;
        brain->data->recoveryPerformed = false;
        brain->data->shouldExitRLVisionKick = false;
        if (logRecoverySnapshot) {
            brain->log->setTimeNow();
            brain->log->log("recovery", rerun::TextLog("Reset recovery, recoveryState: " + to_string(static_cast<int>(brain->data->recoveryState))));
        }
    }

    return NodeStatus::SUCCESS;
}




NodeStatus RoleSwitchIfNeeded::tick()
{
    std::array<TMStatus, MAX_NUM_PLAYERS> teamStatuses{};
    {
        std::lock_guard<std::mutex> teamStatusLock(
            brain->data->teamStatusMutex);
        std::copy(
            std::begin(brain->data->tmStatus),
            std::end(brain->data->tmStatus),
            teamStatuses.begin());
    }
    brain->updatePlayerRoleForTeamAvailability(teamStatuses);
    return NodeStatus::SUCCESS;
}

/* ------------------------------------ Diagnostic nodes ------------------------------------ */

double AutoCalibrateVision::_calcResidual() {
    double res = 0;
    auto markers = brain->data->getMarkings();
    for (auto marker: markers) {
       double minDist = 100;
       for (auto mapMarker : brain->config->mapMarkings) {
           if (marker.label != mapMarker.type) continue;
               
           double dist = norm(marker.posToField.x - mapMarker.x, marker.posToField.y - mapMarker.y);
           if (dist < minDist) minDist = dist;
       }
       res += minDist;
    }
    if (markers.size() > 0) return res / markers.size();
    // else
    return 100;
}

NodeStatus AutoCalibrateVision::onStart()
{
    auto fd = brain->config->fieldDimensions;
    brain->calibrateOdom(
        -fd.length / 2.0 + fd.penaltyAreaLength,
        -fd.width / 2.0,
        M_PI / 2.0,
        "AutoCalibrateVision");

    _res = {};
    _index = -1;
    int steps = 10;

    string state = brain->tree->getEntry<string>("calibrate_state"); 
    if (state == "pitch") {
        for (int i = 0; i < 2 * steps +  1; i++) {
            double center = brain->tree->getEntry<double>("calibrate_pitch_center");
            double step = brain->tree->getEntry<double>("calibrate_pitch_step");
            double p = center + step * (i - steps);
            double y = brain->tree->getEntry<double>("calibrate_yaw_center");
            double z = brain->tree->getEntry<double>("calibrate_z_center");
            _res.push_back({p, y, z, 0, 0});
        }
    } else if (state == "yaw") {
        for (int i = 0; i < 2 * steps +  1; i++) {
            double center = brain->tree->getEntry<double>("calibrate_yaw_center");
            double step = brain->tree->getEntry<double>("calibrate_yaw_step");
            double p = brain->tree->getEntry<double>("calibrate_pitch_center");
            double y = center + step * (i - steps);
            double z = brain->tree->getEntry<double>("calibrate_z_center");
            _res.push_back({p, y, z, 0, 0});
        }
    } else if (state == "z") {
        for (int i = 0; i < 2 * steps +  1; i++) {
            double center = brain->tree->getEntry<double>("calibrate_z_center");
            double step = brain->tree->getEntry<double>("calibrate_z_step");
            double p = brain->tree->getEntry<double>("calibrate_pitch_center");
            double y = brain->tree->getEntry<double>("calibrate_yaw_center");
            double z = center + step * (i - steps);
            _res.push_back({p, y, z, 0, 0});
        }
    } else {
        prtErr("Invalid calibrate state: " + state);
        return NodeStatus::SUCCESS;
    }
    cout << "Calibration started, pitch: " << brain->tree->getEntry<double>("calibrate_pitch_center") << ", yaw: " << brain->tree->getEntry<double>("calibrate_yaw_center") << ", z: " << brain->tree->getEntry<double>("calibrate_z_center") << endl;
    brain->speak(format("Calibration started"));

    return NodeStatus::RUNNING;
}

NodeStatus AutoCalibrateVision::onRunning()
{
    cout << "pitch_step: " << brain->tree->getEntry<double>("calibrate_pitch_step") << endl;
    //  cout << "Calibration running, pitch: " << brain->tree->getEntry<double>("calibrate_pitch_center") << ", yaw: " << brain->tree->getEntry<double>("calibrate_yaw_center") << ", z: " << brain->tree->getEntry<double>("calibrate_z_center") << endl;
    if (brain->tree->getEntry<double>("calibrate_pitch_step") < 0.1) { 
        // cout << RED_CODE << "should stop" << endl;
        prtDebug(
            format("Calibration finished, pitch: %.2f, yaw: %.2f, z: %.2f, best residual: %.2f",
            brain->tree->getEntry<double>("calibrate_pitch_center"),
            brain->tree->getEntry<double>("calibrate_yaw_center"),
            brain->tree->getEntry<double>("calibrate_z_center"),
            _bestResidual),
            GREEN_CODE
        );
        double pitch = brain->tree->getEntry<double>("calibrate_pitch_center"); 
        double yaw = brain->tree->getEntry<double>("calibrate_yaw_center");
        double z = brain->tree->getEntry<double>("calibrate_z_center");
        brain->pubCalParamMsg(pitch, yaw, z);
        brain->log->setTimeNow();
        brain->log->log(
            "/field/vision_param",
            rerun::Points2D({{0, 5}})
            .with_labels({format("BEST: P: %.2f Y: %.2f Z: %.2f RES: %.2f", 
                pitch, yaw, z, _bestResidual)})
            .with_colors(0xFFFFFFFF)
        );
        return NodeStatus::SUCCESS;
    }
    
    double t = brain->msecsSince(_paramChangeTime);
    if (t > 100) { // Publish the next parameter.
        _index++;
        // cout << "Calibration index: " << _index << " size: " << _res.size() << endl;
        if (_index >= _res.size()) { // All parameter tests completed.
            if (brain->tree->getEntry<string>("calibrate_state") == "pitch") {
                // Find the minimum error among entries with at least one sample.
                double minRes = std::numeric_limits<double>::max();
                int minIndex = -1;
                
                for (int i = 0; i < _res.size(); i++) {
                    if (_res[i][4] != 0 && _res[i][3] < minRes) {
                        minRes = _res[i][3];
                        minIndex = i;
                    }
                }
                
                if (minIndex != -1) {
                    brain->tree->setEntry<double>("calibrate_pitch_center", _res[minIndex][0]);
                    _bestResidual = _res[minIndex][3];
                    prtDebug(format("Best pitch: %.2f, error: %.4f, samples: %.0f",
                              _res[minIndex][0], _res[minIndex][3], _res[minIndex][4]));
                }
                brain->tree->setEntry<double>("calibrate_pitch_step", brain->tree->getEntry<double>("calibrate_pitch_step") / 2.0);
                brain->tree->setEntry<string>("calibrate_state", "yaw");
            } else if (brain->tree->getEntry<string>("calibrate_state") == "yaw") {
                double minRes = std::numeric_limits<double>::max();
                int minIndex = -1;
                
                for (int i = 0; i < _res.size(); i++) {
                    if (_res[i][4] != 0 && _res[i][3] < minRes) {
                        minRes = _res[i][3];    
                        minIndex = i;
                    }
                }
                
                if (minIndex != -1) {
                    brain->tree->setEntry<double>("calibrate_yaw_center", _res[minIndex][1]);
                    _bestResidual = _res[minIndex][3];
                    prtDebug(format("Best yaw: %.2f, error: %.4f, samples: %.0f",
                              _res[minIndex][1], _res[minIndex][3], _res[minIndex][4]));
                }
                brain->tree->setEntry<double>("calibrate_yaw_step", brain->tree->getEntry<double>("calibrate_yaw_step") / 2.0); 
                brain->tree->setEntry<string>("calibrate_state", "z");
            } else if (brain->tree->getEntry<string>("calibrate_state") == "z") {
                double minRes = std::numeric_limits<double>::max();
                int minIndex = -1;
                
                for (int i = 0; i < _res.size(); i++) {
                    if (_res[i][4] != 0 && _res[i][3] < minRes) {
                        minRes = _res[i][3];    
                        minIndex = i;        
                    }
                }
                
                if (minIndex != -1) {
                    brain->tree->setEntry<double>("calibrate_z_center", _res[minIndex][2]);
                    _bestResidual = _res[minIndex][3];
                    prtDebug(format("Best z: %.2f, error: %.4f, samples: %.0f",
                              _res[minIndex][2], _res[minIndex][3], _res[minIndex][4]));
                }
                double z_step = brain->tree->getEntry<double>("calibrate_z_step");
                if (z_step > 0.005) 
                    brain->tree->setEntry<double>("calibrate_z_step", z_step / 2.0); 
                brain->tree->setEntry<string>("calibrate_state", "pitch");
            }
            return NodeStatus::SUCCESS;
        }

        // else
        brain->pubCalParamMsg(_res[_index][0], _res[_index][1], _res[_index][2]);
        _paramChangeTime = brain->get_clock()->now();
        _subTotal = 0;
        _count = 0;
        return NodeStatus::RUNNING;
    } else if (t < 50) {
        return NodeStatus::RUNNING; // wait for param to take effect
    }
    
    // else 
    double res = _calcResidual();
    cout << format(
        "Calibrate [%d/%d] P:\t%.2f\tY:\t%.2f\tZ:\t%.2f\tRES:\t%.2f\tCNT:\t%d", 
        _index + 1, _res.size(), _res[_index][0], _res[_index][1], _res[_index][2], res, brain->data->getMarkings().size()
    ) << endl;

    _subTotal += res;
    _count++;
    if (_count > 0) {
        _res[_index][3] = _subTotal / _count;
        _res[_index][4] = _count;  
    } 
    if (brain->log->shouldLog(
            "vision_calibration_visual", brain->config->rerunLogVisualHz)) {
        brain->log->setTimeNow();
        brain->log->log(
            "/field/vision_param",
            rerun::Points2D({{0, 5}})
            .with_labels({format("P: %.2f Y: %.2f Z: %.2f RES: %.2f CNT: %.f",
                _res[_index][0], _res[_index][1], _res[_index][2], _res[_index][3], _res[_index][4])})
            .with_colors(0xFFFFFFFF)
        );
    }
    return NodeStatus::RUNNING;
}

NodeStatus CrabWalk::tick()
{
    double angle, speed;
    getInput("angle", angle);
    getInput("speed", speed);
    brain->client->crabWalk(angle, speed);
    return NodeStatus::SUCCESS;
}

NodeStatus CalibrateOdom::tick()
{
    double x, y, theta;
    getInput("x", x);
    getInput("y", y);
    getInput("theta", theta);

    brain->calibrateOdom(x, y, theta, "CalibrateOdom");
    return NodeStatus::SUCCESS;
}

NodeStatus PrintMsg::tick()
{
    Expected<std::string> msg = getInput<std::string>("msg");
    if (!msg)
    {
        throw RuntimeError("missing required input [msg]: ", msg.error());
    }
    std::cout << "[MSG] " << msg.value() << std::endl;
    return NodeStatus::SUCCESS;
}

NodeStatus PlaySound::tick()
{
    string sound;
    getInput("sound", sound);
    bool allowRepeat;
    getInput("allow_repeat", allowRepeat);
    brain->playSound(sound, allowRepeat);
    return NodeStatus::SUCCESS;
}

NodeStatus Speak::tick()
{
    const string lastText;
    string text;
    getInput("text", text);
    if (text == lastText) return NodeStatus::SUCCESS;

    brain->speak(text, false);
    return NodeStatus::SUCCESS;
}
