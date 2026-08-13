#include <algorithm>
#include <cmath>
#include <cctype>
#include <limits>
#include <optional>
#include <utility>
#include <booster/robot/b1/b1_loco_api.hpp>
#include "brain.h"
#include "fallen_robot_avoidance_policy.h"
#include "robot_path_planner_policy.h"
#include "robot_client.h"
#include "booster_interface/message_utils.hpp"
#include "utils/math.h"
#include "utils/print.h"
#include "utils/misc.h"

namespace {

static_assert(
    static_cast<int64_t>(booster::robot::b1::LocoApiId::kGetTrainedTrajStatus) == 2047,
    "This Brain version requires the public Booster Robotics release SDK");

void copyPathPlanToLog(
    const robot_path_planner_policy::PathPlan &plan,
    ObstacleAvoidanceLogRecord &record)
{
    record.planDirect = plan.direct;
    record.planHasPath = plan.hasPath;
    record.escapingOverlap = plan.escapingOverlap;
    record.overlappingObstacleCount = plan.overlappingObstacleCount;
    record.waypointX = plan.waypoint.x;
    record.waypointY = plan.waypoint.y;
    record.pathMinimumClearance = plan.minimumClearance;
    record.pathStartClearance = plan.startClearance;
    record.escapeClearanceGain = plan.escapeClearanceGain;
    record.avoidanceSide = plan.avoidanceSide;
}

} // namespace

void RobotClient::init()
{
    publisher = brain->create_publisher<booster_msgs::msg::RpcReqMsg>("LocoApiTopicReq", 10);
}

int RobotClient::call(booster_interface::msg::BoosterApiReqMsg msg)
{
    std::string uuid = gen_uuid();
    auto message = booster_msgs::msg::RpcReqMsg();
    message.uuid = uuid;

    nlohmann::json req_header;
    req_header["api_id"] = msg.api_id;
    message.header = req_header.dump();
    message.body = msg.body;
    publisher->publish(message);
    return 0;
}

int RobotClient::moveHead(double pitch, double yaw)
{
    // Apply soft limits.
    yaw = cap(yaw, brain->config->headYawLimitLeft, brain->config->headYawLimitRight);
    pitch = max(pitch, brain->config->headPitchLimitUp);

    if (brain->log->shouldLog("move_head_debug", brain->config->rerunLogDebugHz)) {
        brain->log->setTimeNow();
        auto level = (fabs(pitch > 2.0) || fabs(yaw > 2.0)) ? rerun::TextLogLevel::Error : rerun::TextLogLevel::Info;
        brain->log->log("debug/move_head", rerun::TextLog(format("pitch: %.1f, yaw: %.1f", pitch, yaw)).with_level(level));
    }

    return call(booster_interface::CreateRotateHeadMsg(pitch, yaw));
}

int RobotClient::standUp()
{
    std::string versionText = brain->get_parameter("recovery.getup_version").as_string();
    std::string versionLower;
    versionLower.reserve(versionText.size());
    for (unsigned char c : versionText) {
        versionLower.push_back(static_cast<char>(std::tolower(c)));
    }

    auto version = booster::robot::b1::GetUpVersion::kV2;
    if (versionLower == "kv1" || versionLower == "v1") {
        version = booster::robot::b1::GetUpVersion::kV1;
    } else if (versionLower != "kv2" && versionLower != "v2") {
        RCLCPP_ERROR(
            brain->get_logger(),
            "Invalid recovery.getup_version='%s'; expected kV1 or kV2. Using kV2.",
            versionText.c_str());
    }

    const int ret = call(booster_interface::CreateChangeModeMsg(
        booster::robot::RobotMode::kPrepare));
    if (ret != 0) {
        return ret;
    }
    brain->beginRecoveryPrepareSequence(version);
    return 0;
}

int RobotClient::requestGetUp(booster::robot::b1::GetUpVersion version)
{
    booster::robot::b1::GetUpParameter parameter(version);
    booster_interface::msg::BoosterApiReqMsg msg;
    msg.api_id = static_cast<int64_t>(booster::robot::b1::LocoApiId::kGetUp);
    msg.body = parameter.ToJson().dump();

    const int ret = call(msg);
    if (ret == 0) {
        brain->data->recoveryState = RobotRecoveryState::IS_GETTING_UP;
        brain->data->lastGetupStartedUs =
            brain->get_clock()->now().nanoseconds() / 1000;
        brain->data->lastGetupMovementDetectedUs = 0;
        brain->data->getupNoMovementGraceSec =
            brain->get_parameter("recovery.getup_no_movement_grace_sec").as_double();
    } else {
        enterDamping();
    }
    return ret;
}

int RobotClient::RLVisionKick(bool start)
{
    std::string verLower;
    const std::string configuredVersion = brain->get_parameter(
        "RLVisionKick.visual_kick_version").as_string();
    verLower.reserve(configuredVersion.size());
    for (unsigned char c : configuredVersion) {
        verLower.push_back(static_cast<char>(std::tolower(c)));
    }
    booster::robot::b1::VisualKickVersion vk = booster::robot::b1::VisualKickVersion::kV2;
    if (verLower == "kv1" || verLower == "v1") {
        vk = booster::robot::b1::VisualKickVersion::kV1;
    } else if (verLower == "kv2" || verLower == "v2") {
        vk = booster::robot::b1::VisualKickVersion::kV2;
    }
    booster::robot::b1::VisualKickParameter parameter(start, vk);
    booster_interface::msg::BoosterApiReqMsg msg;
    msg.api_id = static_cast<int64_t>(booster::robot::b1::LocoApiId::kVisualKick);
    msg.body = parameter.ToJson().dump();
    std::cout << "RobotClient::RLVisionKick called with start=" << (start ? "true" : "false") << std::endl;
    return call(msg);
}

int RobotClient::robocupWalk()
{
    std::cout << "RobotClient::robocupWalk exit VisualKick(false)" << std::endl;

    // Only exit VisualKick mode; gait/mode switch is handled elsewhere.
    return RLVisionKick(false);
}

int RobotClient::changeRobocupMode()
{
    std::cout << "RobotClient::changeRobocupMode switch to kSoccer and exit VisualKick(false)" << std::endl;

    int ret = call(booster_interface::CreateChangeModeMsg(booster::robot::RobotMode::kSoccer));
    if (ret != 0) return ret;

    return RLVisionKick(false);
}

int RobotClient::enterDamping()
{
    return call(booster_interface::CreateChangeModeMsg(booster::robot::RobotMode::kDamping));
}

int RobotClient::waveHand(bool doWaveHand)
{
    return call(booster_interface::CreateWaveHandMsg(booster::robot::b1::HandIndex::kRightHand, doWaveHand ? booster::robot::b1::HandAction::kHandOpen : booster::robot::b1::HandAction::kHandClose));
}

void RobotClient::requestFallenRobotAvoidance()
{
    const auto now = brain->get_clock()->now();
    _fallenAvoidanceRequested = true;
    _fallenAvoidanceRequestUntil = now + rclcpp::Duration(2, 0);
}

int RobotClient::setVelocity(double x, double y, double theta, bool applyMinX, bool applyMinY, bool applyMinTheta)
{
    const double requestedX = x;
    const double requestedY = y;
    const double requestedTheta = theta;
    std::optional<ObstacleAvoidanceLogRecord> fallenAvoidanceLog;
    const bool logVelocity =
        brain->log->shouldLog("velocity_visual", brain->config->rerunLogVisualHz);
    if (logVelocity) {
        brain->log->setTimeNow();
        brain->log->log("RobotClient/setVelocity_in",
                        rerun::TextLog(format("vx: %.2f  vy: %.2f  vtheta: %.2f", x, y, theta)));
    }

    const auto now = brain->get_clock()->now();
    const double desiredSpeed = std::hypot(x, y);
    GameObject requestedFallenRobot;
    bool forcedAvoidance = false;
    if (_fallenAvoidanceRequested) {
        const bool requestActive =
            now.nanoseconds() < _fallenAvoidanceRequestUntil.nanoseconds();
        forcedAvoidance = requestActive &&
            brain->hasFallenRobotInVisualKickZone(&requestedFallenRobot);
        if (!forcedAvoidance) {
            _fallenAvoidanceRequested = false;
        }
    }

    if (!brain->data->tmImInVisualKick &&
        (forcedAvoidance || desiredSpeed > 1e-4)) {
        const double pathAngle = forcedAvoidance ? 0.0 : std::atan2(y, x);
        GameObject fallenRobot;
        double entryDistance = std::numeric_limits<double>::infinity();
        const bool fallenRobotOnPath = !forcedAvoidance &&
            brain->findFallenRobotOnPath(
                pathAngle, fallenRobot, &entryDistance);
        if (forcedAvoidance) {
            fallenRobot = requestedFallenRobot;
            const Point obstacleCenter =
                brain->robotObstacleCenterToRobot(fallenRobot);
            entryDistance = std::max(
                0.0,
                std::hypot(obstacleCenter.x, obstacleCenter.y) -
                    brain->robotObstacleRadius(fallenRobot) -
                    brain->selfRobotCollisionRadius());
        }

        if (forcedAvoidance || fallenRobotOnPath) {
            const Point obstacleCenter =
                brain->robotObstacleCenterToRobot(fallenRobot);
            const double obstacleAngle = std::atan2(
                obstacleCenter.y,
                obstacleCenter.x);
            const double obstacleSide = toPInPI(obstacleAngle - pathAngle);
            const double preferredSide = obstacleSide >= 0.0 ? -1.0 : 1.0;
            const double configuredAvoidanceSpeed = std::max(
                0.0,
                brain->get_parameter(
                    "obstacle_avoidance.robot_avoidance_speed").as_double());
            const double configuredMinimumVy = std::max(
                0.0,
                brain->get_parameter(
                    "obstacle_avoidance.robot_avoidance_min_vy").as_double());
            const double configuredReverseSpeed = std::max(
                0.0,
                brain->get_parameter(
                    "obstacle_avoidance.robot_avoidance_reverse_speed")
                    .as_double());
            double forwardSpeed = forcedAvoidance || entryDistance < 0.35
                ? 0.0
                : std::min(desiredSpeed,
                           std::min(std::max(0.0, brain->config->vxLimit),
                                    configuredAvoidanceSpeed * 0.8));
            const double lateralSpeed = std::min(
                {std::max(0.0, brain->config->vyLimit),
                 configuredAvoidanceSpeed,
                 std::max(configuredMinimumVy,
                          std::max(0.0, brain->config->minVy))});

            struct AvoidanceCandidate
            {
                double x = 0.0;
                double y = 0.0;
                double clearance = 0.0;
            };
            const auto makeCandidate = [&](double side, double candidateForwardSpeed) {
                AvoidanceCandidate candidate;
                const auto velocity =
                    fallen_robot_avoidance_policy::makeAvoidanceVelocity(
                        pathAngle,
                        candidateForwardSpeed,
                        lateralSpeed,
                        side,
                        std::max(0.0, brain->config->vxLimit),
                        std::max(0.0, brain->config->vyLimit));
                auto limitedVelocity =
                    robot_path_planner_policy::limitVelocityMagnitude(
                        {velocity.x, velocity.y},
                        configuredAvoidanceSpeed);
                if (limitedVelocity.x < -1e-4) {
                    limitedVelocity =
                        robot_path_planner_policy::limitVelocityMagnitude(
                            limitedVelocity,
                            std::min(configuredAvoidanceSpeed,
                                     configuredReverseSpeed));
                }
                candidate.x = limitedVelocity.x;
                candidate.y = limitedVelocity.y;
                const double speed = std::hypot(candidate.x, candidate.y);
                const bool movingAwayFromPrimary =
                    candidate.x * obstacleCenter.x +
                    candidate.y * obstacleCenter.y <= 1e-6;
                if (speed > 1e-4) {
                    const double candidateAngle =
                        std::atan2(candidate.y, candidate.x);
                    candidate.clearance = entryDistance <= 0.01
                        ? brain->distToDepthObstacle(candidateAngle)
                        : brain->distToObstacle(
                              candidateAngle,
                              movingAwayFromPrimary ? &fallenRobot : nullptr);
                }
                return candidate;
            };

            const double minimumClearance = std::max(
                std::max(0.5, std::min(1.0, brain->config->safeDistance)),
                brain->config->collisionThreshold + 0.15);
            const bool sameTarget = _fallenAvoidanceTargetValid &&
                std::hypot(
                    fallenRobot.posToField.x - _fallenAvoidanceTargetX,
                    fallenRobot.posToField.y - _fallenAvoidanceTargetY) < 1.0;
            const bool sideLockActive = sameTarget &&
                std::fabs(_fallenAvoidanceSide) > 0.1 &&
                now.nanoseconds() < _fallenAvoidanceSideLockUntil.nanoseconds();

            auto positiveCandidate = makeCandidate(1.0, forwardSpeed);
            auto negativeCandidate = makeCandidate(-1.0, forwardSpeed);
            double avoidSide = fallen_robot_avoidance_policy::selectAvoidanceSide(
                preferredSide,
                _fallenAvoidanceSide,
                sideLockActive,
                positiveCandidate.clearance,
                negativeCandidate.clearance,
                minimumClearance);
            auto selectedCandidate = avoidSide > 0.0
                ? positiveCandidate
                : negativeCandidate;
            std::optional<robot_path_planner_policy::PathPlan> localPlan;

            if (selectedCandidate.clearance < minimumClearance &&
                forwardSpeed > 1e-4) {
                forwardSpeed = 0.0;
                positiveCandidate = makeCandidate(1.0, forwardSpeed);
                negativeCandidate = makeCandidate(-1.0, forwardSpeed);
                avoidSide = fallen_robot_avoidance_policy::selectAvoidanceSide(
                    preferredSide,
                    _fallenAvoidanceSide,
                    sideLockActive,
                    positiveCandidate.clearance,
                    negativeCandidate.clearance,
                    minimumClearance);
                selectedCandidate = avoidSide > 0.0
                    ? positiveCandidate
                    : negativeCandidate;
            }

            bool avoidanceDirectionSafe =
                lateralSpeed > 1e-4 &&
                selectedCandidate.clearance >= minimumClearance;
            if (!avoidanceDirectionSafe) {
                const double localGoalDistance = std::max(
                    1.0, std::min(2.0, brain->config->safeDistance + 0.5));
                const double planningClearance = std::max(
                    0.0,
                    brain->get_parameter(
                        "obstacle_avoidance.robot_path_clearance").as_double());
                localPlan = brain->planRobotPath(
                    localGoalDistance * std::cos(pathAngle),
                    localGoalDistance * std::sin(pathAngle),
                    planningClearance,
                    _fallenAvoidanceSide,
                    sameTarget && std::fabs(_fallenAvoidanceSide) > 0.1,
                    sideLockActive);
                if (sideLockActive && !localPlan->direct &&
                    !localPlan->hasPath) {
                    localPlan = brain->planRobotPath(
                        localGoalDistance * std::cos(pathAngle),
                        localGoalDistance * std::sin(pathAngle),
                        planningClearance,
                        _fallenAvoidanceSide,
                        true,
                        false);
                }

                if (localPlan->hasPath) {
                    double escapeAngle = localPlan->direct
                        ? pathAngle
                        : std::atan2(
                              localPlan->waypoint.y, localPlan->waypoint.x);
                    const double maximumEscapeSpeed = std::cos(escapeAngle) < 0.0
                        ? std::min(configuredAvoidanceSpeed,
                                   configuredReverseSpeed)
                        : configuredAvoidanceSpeed;
                    const double minimumEscapeVy = std::min(
                        {configuredMinimumVy,
                         std::max(0.0, brain->config->vyLimit),
                         maximumEscapeSpeed});
                    const double adjustedEscapeAngle =
                        robot_path_planner_policy::angleWithMinimumLateralSpeed(
                            escapeAngle,
                            maximumEscapeSpeed,
                            minimumEscapeVy,
                            std::fabs(localPlan->avoidanceSide) > 0.1
                                ? localPlan->avoidanceSide
                                : avoidSide);
                    const auto clearanceAt = [&](double angle) {
                        return localPlan->escapingOverlap ||
                                entryDistance <= 0.01
                            ? brain->distToDepthObstacle(angle)
                            : brain->distToObstacle(angle);
                    };
                    double escapeClearance = clearanceAt(escapeAngle);
                    const double adjustedClearance =
                        clearanceAt(adjustedEscapeAngle);
                    if (adjustedClearance >= 0.20) {
                        escapeAngle = adjustedEscapeAngle;
                        escapeClearance = adjustedClearance;
                    }
                    const double requiredClearance = localPlan->escapingOverlap ||
                            entryDistance <= 0.01
                        ? 0.20
                        : minimumClearance;
                    if (std::isfinite(escapeAngle) &&
                        escapeClearance >= requiredClearance) {
                        const double minimumSpeed = std::max(
                            0.0,
                            brain->get_parameter(
                                "obstacle_avoidance.robot_avoidance_min_speed")
                                .as_double());
                        const double escapeSpeed =
                            robot_path_planner_policy::velocityForPath(
                                escapeAngle,
                                std::max(0.0, brain->config->vxLimit),
                                std::max(0.0, brain->config->vyLimit),
                                localGoalDistance,
                                std::max(0.0, localPlan->minimumClearance),
                                maximumEscapeSpeed,
                                minimumSpeed,
                                minimumEscapeVy);
                        const auto escapeVelocity =
                            robot_path_planner_policy::limitVelocityMagnitude(
                                {escapeSpeed * std::cos(escapeAngle),
                                 escapeSpeed * std::sin(escapeAngle)},
                                maximumEscapeSpeed);
                        selectedCandidate = {
                            escapeVelocity.x,
                            escapeVelocity.y,
                            escapeClearance};
                        avoidanceDirectionSafe =
                            std::hypot(selectedCandidate.x,
                                       selectedCandidate.y) > 1e-4;
                        if (std::fabs(selectedCandidate.y) > 0.05) {
                            avoidSide = std::copysign(
                                1.0, selectedCandidate.y);
                        } else if (std::fabs(localPlan->avoidanceSide) > 0.1) {
                            avoidSide = localPlan->avoidanceSide;
                        }
                    }
                }
            }
            if (avoidanceDirectionSafe) {
                x = selectedCandidate.x;
                y = selectedCandidate.y;
                theta = 0.0;
                const bool sideFirstSelected =
                    std::fabs(_fallenAvoidanceSide) <= 0.1;
                const bool sideChanged = _fallenAvoidanceSide * avoidSide < 0.0;
                if (sideFirstSelected || sideChanged) {
                    const double lockMsecs = std::max(
                        0.0,
                        brain->get_parameter(
                            "obstacle_avoidance.avoidance_direction_lock_msecs")
                            .as_double());
                    _fallenAvoidanceSideLockUntil =
                        now + rclcpp::Duration::from_seconds(
                            lockMsecs / 1000.0);
                }
                _fallenAvoidanceSide = avoidSide;
                _fallenAvoidanceTargetX = fallenRobot.posToField.x;
                _fallenAvoidanceTargetY = fallenRobot.posToField.y;
                _fallenAvoidanceTargetValid = true;
            } else {
                x = 0.0;
                y = 0.0;
                const double recoveryRate = std::min(
                    std::max(0.0, brain->config->vthetaLimit),
                    std::max(
                        0.0,
                        brain->get_parameter(
                            "obstacle_avoidance.blocked_recovery_turn_rate")
                            .as_double()));
                theta = recoveryRate > 1e-4
                    ? std::copysign(recoveryRate, avoidSide)
                    : 0.0;
            }

            ObstacleAvoidanceLogRecord record;
            record.source = "fallen";
            record.reason = avoidanceDirectionSafe
                ? (entryDistance <= 0.01 ||
                       (localPlan.has_value() && localPlan->escapingOverlap)
                       ? "escape_overlap"
                       : "detour")
                : (std::fabs(theta) > 1e-4
                       ? "blocked_turn"
                       : "blocked_stop");
            record.targetX = fallenRobot.posToField.x;
            record.targetY = fallenRobot.posToField.y;
            record.targetRange = std::hypot(
                obstacleCenter.x, obstacleCenter.y);
            record.targetAngle = obstacleAngle;
            if (localPlan.has_value()) {
                copyPathPlanToLog(*localPlan, record);
            } else {
                record.planDirect = false;
                record.planHasPath = avoidanceDirectionSafe;
            }
            record.waypointX = selectedCandidate.x;
            record.waypointY = selectedCandidate.y;
            record.safeDistance = minimumClearance;
            record.targetClearance = entryDistance;
            record.selectedAngle = std::atan2(
                selectedCandidate.y, selectedCandidate.x);
            record.selectedClearance = selectedCandidate.clearance;
            record.avoidanceSide = avoidSide;
            record.requestedVx = requestedX;
            record.requestedVy = requestedY;
            record.requestedVtheta = requestedTheta;
            fallenAvoidanceLog = std::move(record);
            applyMinX = false;
            applyMinY = false;
            applyMinTheta = false;

            if (brain->log->shouldLog(
                    "fallen_robot_avoidance_debug",
                    brain->config->rerunLogDebugHz)) {
                brain->log->setTimeNow();
                brain->log->log(
                    "debug/fallen_robot_avoidance",
                    rerun::TextLog(format(
                        "entry=%.2f obstacle=(%.2f, %.2f) forced=%d side=%.0f clearance=%.2f safe=%d cmd=(%.2f, %.2f, %.2f)",
                        entryDistance,
                        fallenRobot.posToRobot.x,
                        fallenRobot.posToRobot.y,
                        forcedAvoidance ? 1 : 0,
                        avoidSide,
                        selectedCandidate.clearance,
                        avoidanceDirectionSafe ? 1 : 0,
                        x,
                        y,
                        theta)));
            }
        }
    }

    // rerun::Collection<rerun::Vec2D> vxLine = {{0, 0}, {x, 0}};
    // rerun::Collection<rerun::Vec2D> vyLine = {{0, 0}, {0, -y}};
    // rerun::Collection<rerun::Vec2D> vthetaLine = {{0, 0}, {2.0 * cos(-theta), 2.0 * sin(-theta)}};

    // brain->log->log("robotframe/velocity",
    //                 rerun::LineStrips2D({vxLine, vyLine, vthetaLine})
    //                     .with_colors({0xFF0000FF, 0x00FF00FF, 0x0000FFFF})
    //                     .with_radii({0.05, 0.05, 0.02})
    //                     .with_draw_order(1.0));

    // Raise small velocity commands above the actuator dead zone.
    const double minx = std::max(
        0.0, std::isfinite(brain->config->minVx) ? brain->config->minVx : 0.0);
    const double miny = std::max(
        0.0, std::isfinite(brain->config->minVy) ? brain->config->minVy : 0.0);
    const double mintheta = std::max(
        0.0, std::isfinite(brain->config->minVtheta)
            ? brain->config->minVtheta : 0.0);
    if (applyMinX && fabs(x) < minx && fabs(x) > 1e-5)
        x = x > 0 ? minx : -minx;
    if (applyMinY && fabs(y) < miny && fabs(y) > 1e-5)
        y = y > 0 ? miny : -miny;
    if (applyMinTheta && fabs(theta) < mintheta && fabs(theta) > 1e-5)
        theta = theta > 0 ? mintheta : -mintheta;
    x = cap(x, brain->config->vxLimit, -brain->config->vxLimit);
    y = cap(y, brain->config->vyLimit, -brain->config->vyLimit);
    theta = cap(theta, brain->config->vthetaLimit, -brain->config->vthetaLimit);


    // brain->log->log("RobotClient/velocity_out/x", rerun::Scalar(x));
    // brain->log->log("RobotClient/velocity_out/y", rerun::Scalar(y));
    // brain->log->log("RobotClient/velocity_out/theta", rerun::Scalar(theta));
    
    if (logVelocity) {
        // log simulated path based on velocity
        vector<Pose2D> path = {{0, 0, 0}}; // Frame origin is the robot and theta=0 follows the linear velocity direction.
        double v = norm(x, y);
        double simStep = 1e-1; double simLength = 5.;
        for (int i = 0; i < simLength/simStep; i++) {
            double dt = simStep * (i + 1);
            if (fabs(theta) < 1e-3) path.push_back({v * dt, 0, 0});
            else path.push_back({v/theta * sin(theta * dt), v/theta * (1 - cos(theta * dt)), theta * dt});
        }
        vector<Pose2D> path_f = {};
        for (int i = 0; i < path.size(); i++) {
            auto p = trans(
                path[i].x, path[i].y, path[i].theta,
                brain->data->robotPoseToField.x,
                brain->data->robotPoseToField.y,
                brain->data->robotPoseToField.theta + atan2(y, x),
                "back"
            );
            path_f.push_back({p[0], -p[1], -p[2]});
        }

        if (path_f.size() > 0) {
            auto p = path_f[0];
            brain->log->log(
                "field/velocity",
                rerun::Arrows2D::from_vectors({{v * cos(p.theta), v * sin(p.theta)}})
                    .with_origins({{p.x, p.y}})
                    .with_colors(0xCCCCCCFF)
                    .with_radii(0.005)
                    .with_draw_order(40)
            );
        }
    }
    // prtWarn(format("path_f size: %d", path_f.size()));
    {
        std::lock_guard<std::mutex> lock(_velocityCommandMutex);
        _requestedVx = requestedX;
        _requestedVy = requestedY;
        _requestedVtheta = requestedTheta;
        _vx = x;
        _vy = y;
        _vtheta = theta;
        _lastCmdTime = brain->get_clock()->now();
        if (fabs(_vx) > 1e-3 || fabs(_vy) > 1e-3 || fabs(_vtheta) > 1e-3) {
            _lastNonZeroCmdTime = _lastCmdTime;
        }
    }
    if (logVelocity) {
        brain->log->log("RobotClient/setVelocity_out",
            rerun::TextLog(format("vx: %.2f  vy: %.2f  vtheta: %.2f", x, y, theta)));
    }
    if (fallenAvoidanceLog.has_value()) {
        fallenAvoidanceLog->sentVx = x;
        fallenAvoidanceLog->sentVy = y;
        fallenAvoidanceLog->sentVtheta = theta;
        brain->logObstacleAvoidance(std::move(*fallenAvoidanceLog));
    }
    return call(booster_interface::CreateMoveMsg(x, y, theta));
}

int RobotClient::crabWalk(double angle, double speed) {
    double vxFactor = brain->config->vxFactor;   // Scale vx to compensate for directional differences between commands and motion.
    double yawOffset = brain->config->yawOffset; // Compensate for localization-angle bias.
    double vxLimit = brain->config->vxLimit;
    double vyLimit = brain->config->vyLimit;

    // Calculate the velocity command.
    double cmdAngle = angle + yawOffset;
    double vx = cos(cmdAngle) * speed * vxFactor;
    double vy = sin(cmdAngle) * speed;

    if (fabs(vy) > vyLimit) {
        vx *= vyLimit / fabs(vy);
        vy = vyLimit * fabs(vy) / vy;
    }

    return setVelocity(vx, vy, 0);
}



bool RobotClient::isStandingStill(double timeBuffer) {
    std::lock_guard<std::mutex> lock(_velocityCommandMutex);
    return fabs(_vx) < 1e-3 && fabs(_vy) < 1e-3 && fabs(_vtheta) < 1e-3 && brain->msecsSince(_lastNonZeroCmdTime) > timeBuffer;
}

VelocityCommandSnapshot RobotClient::lastVelocityCommand() const
{
    std::lock_guard<std::mutex> lock(_velocityCommandMutex);
    return {
        _requestedVx,
        _requestedVy,
        _requestedVtheta,
        _vx,
        _vy,
        _vtheta,
        _lastCmdTime};
}

int RobotClient::moveToPoseOnField(double tx, double ty, double ttheta, double longRangeThreshold, double turnThreshold, double vxLimit, double vyLimit, double vthetaLimit, double xTolerance, double yTolerance, double thetaTolerance, bool avoidObstacle) {
    Pose2D target_f, target_r; // Target pose in field and robot frames
    static Pose2D target_temp_r; // Temporary obstacle-avoidance target
    static rclcpp::Time timeEndTempTarget = brain->get_clock()->now(); // Follow the temporary target until this time.

    if (brain->get_clock()->now() < timeEndTempTarget) { // Follow the temporary obstacle-avoidance target.
        target_r = target_temp_r;
        vxLimit = 0.4;
        vyLimit = 0.4;
    } else { // Obstacle-avoidance interval ended; follow the actual target.
        target_f.x = tx;
        target_f.y = ty;
        target_f.theta = ttheta;
        target_r = brain->data->field2robot(target_f);
    }

    double targetAngle = atan2(target_r.y, target_r.x);
    double targetDist = norm(target_r.x, target_r.y);

    double vx = 0.0;
    double vy = 0.0;
    double vtheta = 0.0;

    // Has the robot reached the target?
    if (
        (fabs(brain->data->robotPoseToField.x - target_f.x) < xTolerance) && (fabs(brain->data->robotPoseToField.y - target_f.y) < yTolerance) && (fabs(toPInPI(brain->data->robotPoseToField.theta - target_f.theta)) < thetaTolerance))
    {
        return setVelocity(0, 0, 0);
    }

    // Long-range approach.
    static double breakOscillate = 0.0;
    if (targetDist > longRangeThreshold - breakOscillate)
    {
        breakOscillate = 0.5;

        // Turn toward the target first when the bearing error is large.
        if (fabs(targetAngle) > turnThreshold)
        {
            vtheta = cap(targetAngle, vthetaLimit, -vthetaLimit);
        } else {
            vx = cap(target_r.x, vxLimit, -vxLimit);
            vtheta = cap(targetAngle, vthetaLimit, -vthetaLimit);

        }
    } else { // Short-range approach.
        breakOscillate = 0.0;
        vx = cap(target_r.x, vxLimit, -vxLimit);
        vy = cap(target_r.y, vyLimit, -vyLimit);
        vtheta = cap(target_r.theta, vthetaLimit, -vthetaLimit);
    }

    // Avoid obstacles.
    if (avoidObstacle) {
        double etc = msecsToCollide(vx, vy, vtheta); // estimated time to collide
        double eta = norm(target_r.x, target_r.y) / max(1e-5, norm(vx, vy)) * 1000;// estimated time to arrive

        auto color = 0x00FF00FF;
        if (etc < 3000) color = 0xFF0000FF;
        else if (etc < 5000) color = 0x00FF00FF;
        else color = 0x00FF00FF;

        if (brain->log->shouldLog(
                "collision_visual", brain->config->rerunLogVisualHz)) {
            brain->log->setTimeNow();
            brain->log->log(
                "tick/time_to_collide",
                rerun::LineStrips2D(rerun::LineStrip2D({{10., -200.}, {10. + min(etc / 10, 1260.), -200.}}))
                    .with_colors(color)
                    .with_radii(2.0)
                    .with_draw_order(10)
                    .with_labels({format("etc:  %.0fms", etc)})
            );
        }
        if (etc < min(eta, 3000.)) {
            std::srand(std::time(0));
            vx = (std::rand() / (RAND_MAX / 0.02)) - 0.01; // step on spot, don't full stop
            vy = 0.0;
            vtheta = 0;

            auto rbtPose = brain->data->robotPoseToField;
            double theta0 = toPInPI(atan2(ty - rbtPose.y, tx - rbtPose.x) - rbtPose.theta);
            double theta1;
            double vx_temp, vy_temp;
            for (int i = 0; i < 9; i++) {
                theta1 = theta0 + M_PI / 9 * i;
                vx_temp = 0.4 * cos(theta1);
                vy_temp = 0.4 * sin(theta1);
                if (msecsToCollide(vx_temp, vy_temp, 0) > 3000) break;

                theta1 = theta0 - M_PI / 9 * i;
                vx_temp = 0.4 * cos(theta1);
                vy_temp = 0.4 * sin(theta1);
                if (msecsToCollide(vx_temp, vy_temp, 0) > 3000) break;
            }
            if (brain->log->shouldLog(
                    "avoid_obstacle_debug", brain->config->rerunLogDebugHz)) {
                brain->log->setTimeNow();
                brain->log->log("debug/avoidObstacle", rerun::TextLog(format(
                    "theta1 = %.2f",
                    theta1
                )));
            }
            target_temp_r.x = 1 * cos(theta1);
            target_temp_r.y = 1 * sin(theta1);
            timeEndTempTarget = brain->get_clock()->now() + rclcpp::Duration(3.5, 0.0);
        }
        else if (etc < min(eta, 6000.)) {
            vtheta += 0.2;
        }
    }

    return setVelocity(vx, vy, vtheta);
}

int RobotClient::moveToPoseOnField2(double tx, double ty, double ttheta, double longRangeThreshold, double turnThreshold, double vxLimit, double vyLimit, double vthetaLimit, double xTolerance, double yTolerance, double thetaTolerance, bool avoidObstacle) {
    const auto now = brain->get_clock()->now();
    const double safeDist = std::max(
        0.0, std::isfinite(brain->config->safeDistance)
            ? brain->config->safeDistance : 0.0);
    const double targetDelta = _move2TargetValid
        ? std::hypot(tx - _move2TargetX, ty - _move2TargetY)
        : std::numeric_limits<double>::infinity();
    double callGapSeconds = std::numeric_limits<double>::infinity();
    if (_move2LastCallAt.nanoseconds() != 0 &&
        _move2LastCallAt.get_clock_type() == now.get_clock_type()) {
        callGapSeconds = (now - _move2LastCallAt).seconds();
    }
    const bool staleState = !_move2TargetValid ||
        !std::isfinite(callGapSeconds) || callGapSeconds < 0.0 ||
        callGapSeconds > 1.0;
    if (staleState || targetDelta > 0.75) {
        _move2AvoidActive = false;
        _move2Backing = false;
        _move2AvoidSide = 0.0;
    }
    _move2TargetX = tx;
    _move2TargetY = ty;
    _move2TargetValid = true;
    _move2LastCallAt = now;
    if (!avoidObstacle) {
        _move2AvoidActive = false;
        _move2Backing = false;
        _move2AvoidSide = 0.0;
    }

    const double range = norm(
        tx - brain->data->robotPoseToField.x,
        ty - brain->data->robotPoseToField.y);
    if (range > longRangeThreshold * (_move2LongRange ? 0.9 : 1.0)) {
        _move2LongRange = true;
    } else {
        _move2LongRange = false;
    }

    double vx = 0.0;
    double vy = 0.0;
    double vtheta = 0.0;
    bool avoidanceCommand = false;
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
    double tarDir = atan2(ty - brain->data->robotPoseToField.y, tx - brain->data->robotPoseToField.x);
    double faceDir = brain->data->robotPoseToField.theta;
    double tarDir_r = toPInPI(tarDir - faceDir); // Target bearing relative to current heading
    ObstacleAvoidanceLogRecord avoidanceLog;
    avoidanceLog.source = "move2";
    avoidanceLog.reason = "direct";
    avoidanceLog.targetX = tx;
    avoidanceLog.targetY = ty;
    avoidanceLog.targetRange = range;
    avoidanceLog.targetAngle = tarDir_r;
    avoidanceLog.safeDistance = safeDist;
    const auto blockedRecoveryTurn = [&](double selectedAngle) {
        const double configuredRate = std::max(
            0.0,
            brain->get_parameter(
                "obstacle_avoidance.blocked_recovery_turn_rate").as_double());
        const double rate = std::min(
            configuredRate, std::max(0.0, vthetaLimit));
        if (!std::isfinite(rate) || rate <= 1e-4) return 0.0;
        const double delta = toPInPI(selectedAngle - tarDir_r);
        return (std::fabs(delta) > 0.1 ? std::copysign(rate, delta) : rate);
    };
    const auto applyAvoidance = [&]() {
        if (!avoidObstacle) {
            _move2AvoidActive = false;
            _move2AvoidSide = 0.0;
            return false;
        }

        const double planningClearance = std::max(
            0.0,
            brain->get_parameter(
                "obstacle_avoidance.robot_path_clearance").as_double());
        const double lockMsecs = std::max(
            0.0,
            brain->get_parameter(
                "obstacle_avoidance.avoidance_direction_lock_msecs")
                .as_double());
        const bool preferPreviousSide = _move2AvoidActive &&
            std::fabs(_move2AvoidSide) > 0.1;
        const bool hardLockActive = preferPreviousSide &&
            now.nanoseconds() < _move2AvoidUntil.nanoseconds();
        auto plan = brain->planRobotPath(
            range * std::cos(tarDir_r),
            range * std::sin(tarDir_r),
            planningClearance,
            _move2AvoidSide,
            preferPreviousSide,
            hardLockActive);
        if (hardLockActive && !plan.direct && !plan.hasPath) {
            plan = brain->planRobotPath(
                range * std::cos(tarDir_r),
                range * std::sin(tarDir_r),
                planningClearance,
                _move2AvoidSide,
                true,
                false);
        }
        const double targetClearance = brain->distToObstacle(tarDir_r);
        copyPathPlanToLog(plan, avoidanceLog);
        avoidanceLog.pathClearance = planningClearance;
        avoidanceLog.targetClearance = targetClearance;
        avoidanceLog.selectedAngle = tarDir_r;
        avoidanceLog.selectedClearance = targetClearance;
        const bool routeBlocked = !plan.direct ||
            targetClearance < std::min(safeDist, range + 0.15);
        if (!routeBlocked) {
            _move2AvoidActive = false;
            _move2AvoidSide = 0.0;
            return false;
        }
        _move2AvoidActive = true;
        avoidanceCommand = true;

        double avoidAngle = tarDir_r;
        if (!plan.direct && plan.hasPath) {
            avoidAngle = std::atan2(plan.waypoint.y, plan.waypoint.x);
        } else {
            avoidAngle = brain->calcAvoidDir(tarDir_r, safeDist);
        }

        // A planner route is checked against visual circles. A very close
        // depth hit still vetoes it and causes a fresh angular scan.
        const bool visualPathSafe = plan.direct || plan.escapingOverlap ||
            (plan.hasPath && plan.minimumClearance >= -0.01);
        double selectedClearance = plan.escapingOverlap
            ? brain->distToDepthObstacle(avoidAngle)
            : brain->distToObstacle(avoidAngle);
        bool usedScanFallback = false;
        if (!visualPathSafe || selectedClearance < 0.20) {
            avoidAngle = brain->calcAvoidDir(tarDir_r, safeDist);
            selectedClearance = brain->distToObstacle(avoidAngle);
            usedScanFallback = true;
        }
        if (!std::isfinite(avoidAngle) ||
            selectedClearance < 0.20) {
            vx = 0.0;
            vy = 0.0;
            vtheta = blockedRecoveryTurn(avoidAngle);
            avoidanceLog.reason = std::fabs(vtheta) > 1e-4
                ? "blocked_turn"
                : "blocked_stop";
            avoidanceLog.selectedAngle = avoidAngle;
            avoidanceLog.selectedClearance = selectedClearance;
            return true;
        }

        const double preferredSpeed = std::max(
            0.0,
            brain->get_parameter(
                "obstacle_avoidance.robot_avoidance_speed").as_double());
        const double reverseSpeed = std::max(
            0.0,
            brain->get_parameter(
                "obstacle_avoidance.robot_avoidance_reverse_speed").as_double());
        const double maximumSpeed = std::cos(avoidAngle) < 0.0
            ? std::min(preferredSpeed, reverseSpeed)
            : preferredSpeed;
        const double minimumVy = std::min(
            {configuredMinimumVy, avoidanceVyLimit, maximumSpeed});
        const double adjustedAngle =
            robot_path_planner_policy::angleWithMinimumLateralSpeed(
                avoidAngle,
                maximumSpeed,
                minimumVy,
                std::fabs(plan.avoidanceSide) > 0.1
                    ? plan.avoidanceSide
                    : _move2AvoidSide);
        if (std::fabs(toPInPI(adjustedAngle - avoidAngle)) > 1e-4) {
            const double adjustedClearance = plan.escapingOverlap
                ? brain->distToDepthObstacle(adjustedAngle)
                : brain->distToObstacle(adjustedAngle);
            if (adjustedClearance >= 0.20) {
                avoidAngle = adjustedAngle;
                selectedClearance = adjustedClearance;
            }
        }
        const double minimumSpeed = std::max(
            0.0,
            brain->get_parameter(
                "obstacle_avoidance.robot_avoidance_min_speed").as_double());
        const double clearance = plan.direct
            ? std::max(0.25, std::min(safeDist, 0.60))
            : std::max(0.0, plan.minimumClearance);
        const double avoidSpeed = robot_path_planner_policy::velocityForPath(
            avoidAngle,
            std::max(0.0, vxLimit),
            avoidanceVyLimit,
            range,
            clearance,
            maximumSpeed,
            minimumSpeed,
            minimumVy);
        const auto limitedVelocity =
            robot_path_planner_policy::limitVelocityMagnitude(
                {avoidSpeed * std::cos(avoidAngle),
                 avoidSpeed * std::sin(avoidAngle)},
                maximumSpeed);
        vx = limitedVelocity.x;
        vy = limitedVelocity.y;
        vtheta = 0.0;
        avoidanceLog.reason = plan.escapingOverlap
            ? "escape_overlap"
            : (usedScanFallback || !plan.hasPath
                   ? "scan_fallback"
                   : "detour");
        avoidanceLog.selectedAngle = avoidAngle;
        avoidanceLog.selectedClearance = selectedClearance;
        const double selectedSide = std::fabs(std::sin(avoidAngle)) > 0.05
            ? std::copysign(1.0, std::sin(avoidAngle))
            : plan.avoidanceSide;
        if (std::fabs(selectedSide) > 0.1) {
            const bool sideFirstSelected = std::fabs(_move2AvoidSide) <= 0.1;
            const bool sideChanged = _move2AvoidSide * selectedSide < 0.0;
            if (sideFirstSelected || sideChanged) {
                _move2AvoidUntil = now + rclcpp::Duration::from_seconds(
                    lockMsecs / 1000.0);
            }
            _move2AvoidSide = selectedSide;
            avoidanceLog.avoidanceSide = selectedSide;
        }
        if (std::hypot(vx, vy) < 1e-4) {
            vtheta = blockedRecoveryTurn(avoidAngle);
            avoidanceLog.reason = std::fabs(vtheta) > 1e-4
                ? "blocked_turn"
                : "blocked_stop";
        }
        _move2Backing = std::cos(avoidAngle) < 0.0;
        return true;
    };

    if (_move2LongRange) { // At long range, turn and walk toward the target before fine position adjustment.
        // Without obstacle avoidance.
        if (fabs(tarDir_r) > turnThreshold) { // Turn toward the target.
            vx = 0.0;
            vy = 0.0;
            vtheta = tarDir_r;
        } else { // Walk forward while making a small heading correction.
            vx = cap(range, vxLimit, -vxLimit);
            vy = 0.0;
            vtheta = tarDir_r;
        }
    } else if (!_move2LongRange) { // Adjust position directly at short range.
        
        // Without obstacle avoidance.
        vx = range * cos(tarDir_r);
        vy = range * sin(tarDir_r);
        vtheta = toPInPI(ttheta - faceDir); // Turn to the requested final heading.
        if (fabs(vx) < xTolerance && fabs(vy) < yTolerance && fabs(vtheta) < thetaTolerance) {
            vx = 0.0;
            vy = 0.0;
            vtheta = 0.0;
        }
    }

    // Apply one planner to both long- and short-range phases. This is also
    // what keeps the last metre of Freekick/Ready movement obstacle-aware.
    applyAvoidance();

    vx = cap(vx, vxLimit, -vxLimit);
    const double commandVyLimit = avoidanceCommand
        ? avoidanceVyLimit
        : vyLimit;
    vy = cap(vy, commandVyLimit, -commandVyLimit);
    vtheta = cap(vtheta, vthetaLimit, -vthetaLimit); 
    const int result = setVelocity(
        vx,
        vy,
        vtheta,
        !avoidanceCommand,
        !avoidanceCommand,
        !avoidanceCommand);
    if (avoidObstacle) {
        const auto sent = lastVelocityCommand();
        avoidanceLog.requestedVx = vx;
        avoidanceLog.requestedVy = vy;
        avoidanceLog.requestedVtheta = vtheta;
        avoidanceLog.sentVx = sent.sentX;
        avoidanceLog.sentVy = sent.sentY;
        avoidanceLog.sentVtheta = sent.sentTheta;
        brain->logObstacleAvoidance(std::move(avoidanceLog));
    }
    return result;
}

int RobotClient::moveToPoseOnField3(double tx, double ty, double ttheta, double longRangeThreshold, double turnThreshold, double vxLimit, double vyLimit, double vthetaLimit, double xTolerance, double yTolerance, double thetaTolerance, bool avoidObstacle) {
    const auto now = brain->get_clock()->now();
    const bool isFreekickStartPlacing = brain->isFreekickStartPlacing();
    const double safeDist = std::max(
        0.0, isFreekickStartPlacing
            ? brain->get_parameter(
                  "obstacle_avoidance.freekick_start_placing_safe_distance")
                  .get_value<double>()
            : brain->config->safeDistance);
    const double targetDelta = _move3TargetValid
        ? std::hypot(tx - _move3TargetX, ty - _move3TargetY)
        : std::numeric_limits<double>::infinity();
    double callGapSeconds = std::numeric_limits<double>::infinity();
    if (_move3LastCallAt.nanoseconds() != 0 &&
        _move3LastCallAt.get_clock_type() == now.get_clock_type()) {
        callGapSeconds = (now - _move3LastCallAt).seconds();
    }
    const bool staleState = !_move3TargetValid ||
        !std::isfinite(callGapSeconds) || callGapSeconds < 0.0 ||
        callGapSeconds > 1.0;
    if (staleState || targetDelta > 0.75) {
        _move3AvoidActive = false;
        _move3AvoidSide = 0.0;
    }
    _move3TargetX = tx;
    _move3TargetY = ty;
    _move3TargetValid = true;
    _move3LastCallAt = now;
    if (!avoidObstacle) {
        _move3AvoidActive = false;
        _move3AvoidSide = 0.0;
    }

    double range = norm(tx - brain->data->robotPoseToField.x, ty - brain->data->robotPoseToField.y);
    if (range > longRangeThreshold * (_move3LongRange ? 0.9 : 1.0)) { // At long range, turn and walk toward the target before fine adjustment.
        _move3LongRange = true;
    } else {
        _move3LongRange = false;
    }

    double vx = 0.0;
    double vy = 0.0;
    double vtheta = 0.0;
    bool avoidanceCommand = false;
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
    double tarDir = atan2(ty - brain->data->robotPoseToField.y, tx - brain->data->robotPoseToField.x);
    double faceDir = brain->data->robotPoseToField.theta;
    double tarDir_r = toPInPI(tarDir - faceDir); // Target bearing relative to current heading
    ObstacleAvoidanceLogRecord avoidanceLog;
    avoidanceLog.source = "move3";
    avoidanceLog.reason = "direct";
    avoidanceLog.targetX = tx;
    avoidanceLog.targetY = ty;
    avoidanceLog.targetRange = range;
    avoidanceLog.targetAngle = tarDir_r;
    avoidanceLog.safeDistance = safeDist;
    const auto blockedRecoveryTurn = [&](double selectedAngle) {
        const double configuredRate = std::max(
            0.0,
            brain->get_parameter(
                "obstacle_avoidance.blocked_recovery_turn_rate").as_double());
        const double rate = std::min(
            configuredRate, std::max(0.0, vthetaLimit));
        if (!std::isfinite(rate) || rate <= 1e-4) return 0.0;
        const double delta = toPInPI(selectedAngle - tarDir_r);
        return (std::fabs(delta) > 0.1 ? std::copysign(rate, delta) : rate);
    };
    const auto applyAvoidance = [&]() {
        if (!avoidObstacle) return false;
        const double planningClearance = std::max(
            0.0,
            brain->get_parameter(
                "obstacle_avoidance.robot_path_clearance").as_double());
        const double lockMsecs = std::max(
            0.0,
            brain->get_parameter(
                "obstacle_avoidance.avoidance_direction_lock_msecs")
                .as_double());
        const bool preferPreviousSide = _move3AvoidActive &&
            std::fabs(_move3AvoidSide) > 0.1;
        const bool hardLockActive = preferPreviousSide &&
            now.nanoseconds() < _move3AvoidUntil.nanoseconds();
        auto plan = brain->planRobotPath(
            range * std::cos(tarDir_r),
            range * std::sin(tarDir_r),
            planningClearance,
            _move3AvoidSide,
            preferPreviousSide,
            hardLockActive);
        if (hardLockActive && !plan.direct && !plan.hasPath) {
            plan = brain->planRobotPath(
                range * std::cos(tarDir_r),
                range * std::sin(tarDir_r),
                planningClearance,
                _move3AvoidSide,
                true,
                false);
        }
        const double targetClearance = brain->distToObstacle(tarDir_r);
        copyPathPlanToLog(plan, avoidanceLog);
        avoidanceLog.pathClearance = planningClearance;
        avoidanceLog.targetClearance = targetClearance;
        avoidanceLog.selectedAngle = tarDir_r;
        avoidanceLog.selectedClearance = targetClearance;
        if (plan.direct && targetClearance >=
                std::min(safeDist, range + 0.15)) {
            _move3AvoidActive = false;
            _move3AvoidSide = 0.0;
            return false;
        }
        _move3AvoidActive = true;
        avoidanceCommand = true;
        double avoidAngle = plan.direct
            ? brain->calcAvoidDir(tarDir_r, safeDist)
            : (plan.hasPath
                   ? std::atan2(plan.waypoint.y, plan.waypoint.x)
                   : tarDir_r);
        const bool visualPathSafe = plan.direct || plan.escapingOverlap ||
            (plan.hasPath && plan.minimumClearance >= -0.01);
        double selectedClearance = plan.escapingOverlap
            ? brain->distToDepthObstacle(avoidAngle)
            : brain->distToObstacle(avoidAngle);
        bool usedScanFallback = false;
        if (!visualPathSafe || selectedClearance < 0.20) {
            avoidAngle = brain->calcAvoidDir(tarDir_r, safeDist);
            selectedClearance = brain->distToObstacle(avoidAngle);
            usedScanFallback = true;
        }
        if (!std::isfinite(avoidAngle) ||
            selectedClearance < 0.20) {
            vx = 0.0;
            vy = 0.0;
            vtheta = blockedRecoveryTurn(avoidAngle);
            avoidanceLog.reason = std::fabs(vtheta) > 1e-4
                ? "blocked_turn"
                : "blocked_stop";
            avoidanceLog.selectedAngle = avoidAngle;
            avoidanceLog.selectedClearance = selectedClearance;
            return true;
        }
        const double preferredSpeed = std::max(
            0.0,
            brain->get_parameter(
                "obstacle_avoidance.robot_avoidance_speed").as_double());
        const double reverseSpeed = std::max(
            0.0,
            brain->get_parameter(
                "obstacle_avoidance.robot_avoidance_reverse_speed").as_double());
        const double maximumSpeed = std::cos(avoidAngle) < 0.0
            ? std::min(preferredSpeed, reverseSpeed)
            : preferredSpeed;
        const double minimumVy = std::min(
            {configuredMinimumVy, avoidanceVyLimit, maximumSpeed});
        const double adjustedAngle =
            robot_path_planner_policy::angleWithMinimumLateralSpeed(
                avoidAngle,
                maximumSpeed,
                minimumVy,
                std::fabs(plan.avoidanceSide) > 0.1
                    ? plan.avoidanceSide
                    : _move3AvoidSide);
        if (std::fabs(toPInPI(adjustedAngle - avoidAngle)) > 1e-4) {
            const double adjustedClearance = plan.escapingOverlap
                ? brain->distToDepthObstacle(adjustedAngle)
                : brain->distToObstacle(adjustedAngle);
            if (adjustedClearance >= 0.20) {
                avoidAngle = adjustedAngle;
                selectedClearance = adjustedClearance;
            }
        }
        const double minimumSpeed = std::max(
            0.0,
            brain->get_parameter(
                "obstacle_avoidance.robot_avoidance_min_speed").as_double());
        const double clearance = plan.direct
            ? std::max(0.25, std::min(safeDist, 0.60))
            : std::max(0.0, plan.minimumClearance);
        const double avoidSpeed = robot_path_planner_policy::velocityForPath(
            avoidAngle,
            std::max(0.0, vxLimit),
            avoidanceVyLimit,
            range,
            clearance,
            maximumSpeed,
            minimumSpeed,
            minimumVy);
        const auto limitedVelocity =
            robot_path_planner_policy::limitVelocityMagnitude(
                {avoidSpeed * std::cos(avoidAngle),
                 avoidSpeed * std::sin(avoidAngle)},
                maximumSpeed);
        vx = limitedVelocity.x;
        vy = limitedVelocity.y;
        vtheta = 0.0;
        avoidanceLog.reason = plan.escapingOverlap
            ? "escape_overlap"
            : (usedScanFallback || !plan.hasPath
                   ? "scan_fallback"
                   : "detour");
        avoidanceLog.selectedAngle = avoidAngle;
        avoidanceLog.selectedClearance = selectedClearance;
        const double selectedSide = std::fabs(std::sin(avoidAngle)) > 0.05
            ? std::copysign(1.0, std::sin(avoidAngle))
            : plan.avoidanceSide;
        if (std::fabs(selectedSide) > 0.1) {
            const bool sideFirstSelected = std::fabs(_move3AvoidSide) <= 0.1;
            const bool sideChanged = _move3AvoidSide * selectedSide < 0.0;
            if (sideFirstSelected || sideChanged) {
                _move3AvoidUntil = now + rclcpp::Duration::from_seconds(
                    lockMsecs / 1000.0);
            }
            _move3AvoidSide = selectedSide;
            avoidanceLog.avoidanceSide = selectedSide;
        }
        if (std::hypot(vx, vy) < 1e-4) {
            vtheta = blockedRecoveryTurn(avoidAngle);
            avoidanceLog.reason = std::fabs(vtheta) > 1e-4
                ? "blocked_turn"
                : "blocked_stop";
        }
        return true;
    };
    if (_move3LongRange) { // At long range, turn and walk toward the target before fine position adjustment.
        // brain->log->log("debug/freekick_position", rerun::TextLog(format(
        //         "long range"
        // )));
        // Without obstacle avoidance.
        if (fabs(tarDir_r) > turnThreshold) { // Turn toward the target.
            vx = 0.0;
            vy = 0.0;
            vtheta = tarDir_r;
        } else { // Walk forward while making a small heading correction.
            vx = cap(range, vxLimit, -vxLimit);
            vy = 0.0;
            vtheta = tarDir_r;
        }
    } else if (!_move3LongRange) { // Adjust position directly at short range.
        // brain->log->log("debug/freekick_position", rerun::TextLog(format(
        //     "shortRange"
        // )));
       // Without obstacle avoidance.
        vx = range * cos(tarDir_r);
        vy = range * sin(tarDir_r);
        vtheta = toPInPI(ttheta - faceDir); // Turn to the requested final heading.
        if (fabs(vx) < xTolerance && fabs(vy) < yTolerance && fabs(vtheta) < thetaTolerance) {
            vx = 0.0;
            vy = 0.0;
            vtheta = 0.0;

            // brain->log->log("debug/freekick_position", rerun::TextLog(format(
            //     "shortRange zero case"
            // )));
        }


    }

    applyAvoidance();
    vx = cap(vx, vxLimit, -vxLimit);
    const double commandVyLimit = avoidanceCommand
        ? avoidanceVyLimit
        : vyLimit;
    vy = cap(vy, commandVyLimit, -commandVyLimit);
    vtheta = cap(vtheta, vthetaLimit, -vthetaLimit); 
    const int result = setVelocity(
        vx,
        vy,
        vtheta,
        !avoidanceCommand,
        !avoidanceCommand,
        !avoidanceCommand);
    if (avoidObstacle) {
        const auto sent = lastVelocityCommand();
        avoidanceLog.requestedVx = vx;
        avoidanceLog.requestedVy = vy;
        avoidanceLog.requestedVtheta = vtheta;
        avoidanceLog.sentVx = sent.sentX;
        avoidanceLog.sentVy = sent.sentY;
        avoidanceLog.sentVtheta = sent.sentTheta;
        brain->logObstacleAvoidance(std::move(avoidanceLog));
    }
    return result;
}

double RobotClient::msecsToCollide(double vx, double vy, double vtheta, double maxTime) {
    (void)vtheta;
    const double boundedMaxTime = std::max(0.0, maxTime);
    const double speed = std::hypot(vx, vy);
    if (!std::isfinite(speed) || speed < 1e-3) return boundedMaxTime;

    const bool logCollisionDebug = brain->log->shouldLog(
        "msecs_to_collide_debug", brain->config->rerunLogDebugHz);
    const double pathAngle = std::atan2(vy, vx);
    const double distance = brain->distToObstacle(pathAngle);
    const double collisionTime = distance / speed * 1000.0;
    if (logCollisionDebug) {
        brain->log->setTimeNow();
        brain->log->log("debug/msecsToCollide", rerun::TextLog(format(
            "angle=%.2f speed=%.2f distance=%.2f time=%.0fms",
            pathAngle, speed, distance, collisionTime)));
    }
    if (!std::isfinite(collisionTime)) return boundedMaxTime;
    return std::clamp(collisionTime, 0.0, boundedMaxTime);
}
