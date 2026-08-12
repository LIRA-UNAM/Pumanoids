#pragma once

#include <iostream>
#include <mutex>
#include <string>
#include <rerun.hpp>

#include "booster_interface/srv/rpc_service.hpp"
#include "booster_interface/msg/booster_api_req_msg.hpp"
#include "booster_msgs/msg/rpc_req_msg.hpp"
#include <booster/robot/b1/b1_loco_api.hpp>

using namespace std;

class Brain; // Forward declaration; RobotClient and Brain depend on each other.

struct VelocityCommandSnapshot
{
    double requestedX = 0.0;
    double requestedY = 0.0;
    double requestedTheta = 0.0;
    double sentX = 0.0;
    double sentY = 0.0;
    double sentTheta = 0.0;
    rclcpp::Time time;
};


/**
 * RobotClient contains all RobotSDK operations that control the robot.
 * The current implementation has a bidirectional dependency with Brain.
 */
class RobotClient
{
public:
    RobotClient(Brain* argBrain) : brain(argBrain) {}

    void init();

    /**
     * @brief Move the robot's head.
     *
     * @param pitch
     * @param yaw
     *
     * @return 0 on success.
     */
    int moveHead(double pitch, double yaw);

    /**
     * @brief Set the robot's walking velocity.
     * 
     * @param x Forward velocity (m/s).
     * @param y Leftward velocity (m/s).
     * @param theta Counterclockwise angular velocity (rad/s).
     * @param applyMinX, applyMinY, applyMinTheta Whether to raise small commands above the actuator dead zone.
     * 
     * @return 0 on success.
     * 
    */
    int setVelocity(double x, double y, double theta, bool applyMinX=true, bool applyMinY=true, bool applyMinTheta=true);

    // Request a persistent lateral avoidance maneuver after VisualKick exits near a fallen robot.
    void requestFallenRobotAvoidance();

    // Move at a specified speed and direction without rotating.
    int crabWalk(double angle, double speed);

    /**
     * @brief Walk in velocity mode to a pose in the field frame, including the final orientation.
     * 
     * @param tx, ty, ttheta Target pose in the field frame.
     * @param longRangeThreshold Prefer turning and walking forward beyond this distance.
     * @param turnThreshold Turn toward the target when its bearing differs by more than this value; this is not the final ttheta.
     * @param vxLimit, vyLimit, vthetaLimit Velocity limits in m/s and rad/s.
     * @param xTolerance, yTolerance, thetaTolerance Arrival tolerances.
     * @param avoidObstacle Whether to avoid obstacles while moving.
     * 
     * @return Motion-control result; 0 indicates success.
     */
    int moveToPoseOnField(double tx, double ty, double ttheta, double longRangeThreshold, double turnThreshold, double vxLimit, double vyLimit, double vthetaLimit, double xTolerance, double yTolerance, double thetaTolerance, bool avoidObstacle = false);

    /**
     * @brief Updated velocity-mode controller for reaching a field-frame pose and final orientation.
     * 
     * @param tx, ty, ttheta Target pose in the field frame.
     * @param longRangeThreshold Prefer turning and walking forward beyond this distance.
     * @param turnThreshold Turn toward the target when its bearing differs by more than this value; this is not the final ttheta.
     * @param vxLimit, vyLimit, vthetaLimit Velocity limits in m/s and rad/s.
     * @param xTolerance, yTolerance, thetaTolerance Arrival tolerances.
     * @param avoidObstacle Whether to avoid obstacles while moving.
     * 
     * @return Motion-control result; 0 indicates success.
     */

    int moveToPoseOnField2(double tx, double ty, double ttheta, double longRangeThreshold, double turnThreshold, double vxLimit, double vyLimit, double vthetaLimit, double xTolerance, double yTolerance, double thetaTolerance, bool avoidObstacle = false);
    /**
     * @brief Short-range velocity-mode controller for reaching a field-frame pose and final orientation.
     * 
     * @param tx, ty, ttheta Target pose in the field frame.
     * @param longRangeThreshold Prefer turning and walking forward beyond this distance.
     * @param turnThreshold Turn toward the target when its bearing differs by more than this value; this is not the final ttheta.
     * @param vxLimit, vyLimit, vthetaLimit Velocity limits in m/s and rad/s.
     * @param xTolerance, yTolerance, thetaTolerance Arrival tolerances.
     * @param avoidObstacle Whether to avoid obstacles while moving.
     * 
     * @return Motion-control result; 0 indicates success.
     */
    int moveToPoseOnField3(double tx, double ty, double ttheta, double longRangeThreshold, double turnThreshold, double vxLimit, double vyLimit, double vthetaLimit, double xTolerance, double yTolerance, double thetaTolerance, bool avoidObstacle = false);

    /**
     * @brief Wave a hand.
     */
    int waveHand(bool doWaveHand);

    /**
     * @brief Get up after a fall.
     */
    int standUp();

    // kPrepare is confirmed by Brain before this public GetUp request is sent.
    int requestGetUp(booster::robot::b1::GetUpVersion version);

    /**
     * @brief Switch to reinforcement-learning visual kicking.
     */
    int RLVisionKick(bool start = true);

    /**
     * @brief Switch to the RoboCup gait.
     */
    int robocupWalk();

    /**
     * @brief Switch robocup mode (kSoccer) and exit VisualKick(false)
     */
     int changeRobocupMode();
     
    /**
     * @brief Enter damping mode.
     */
    int enterDamping();

    // Estimate milliseconds to collision at the specified velocity.
    double msecsToCollide(double vx, double vy, double vtheta, double maxTime=10000);

    // Estimate whether the robot is standing still.
    bool isStandingStill(double timeBuffer = 1000);

    // Return the latest requested velocity and the command sent after limits and dead-zone handling.
    VelocityCommandSnapshot lastVelocityCommand() const;

private:
    int call(booster_interface::msg::BoosterApiReqMsg msg);
    rclcpp::Publisher<booster_msgs::msg::RpcReqMsg>::SharedPtr publisher;
    Brain *brain;
    double _vx = 0.0;
    double _vy = 0.0;
    double _vtheta = 0.0;
    double _requestedVx = 0.0;
    double _requestedVy = 0.0;
    double _requestedVtheta = 0.0;
    rclcpp::Time _lastCmdTime;
    rclcpp::Time _lastNonZeroCmdTime;
    mutable std::mutex _velocityCommandMutex;
    bool _fallenAvoidanceRequested = false;
    rclcpp::Time _fallenAvoidanceRequestUntil;
    double _fallenAvoidanceSide = 0.0;
    rclcpp::Time _fallenAvoidanceSideLockUntil;
    double _fallenAvoidanceTargetX = 0.0;
    double _fallenAvoidanceTargetY = 0.0;
    bool _fallenAvoidanceTargetValid = false;

    // moveToPoseOnField2/3 are used by different behavior-tree branches.
    // Keep their hysteresis per RobotClient and reset it when the target
    // changes, so one behavior cannot leak an avoidance state into another.
    bool _move2LongRange = true;
    bool _move2AvoidActive = false;
    bool _move2Backing = false;
    double _move2AvoidSide = 0.0;
    rclcpp::Time _move2AvoidUntil;
    rclcpp::Time _move2LastCallAt;
    double _move2TargetX = 0.0;
    double _move2TargetY = 0.0;
    bool _move2TargetValid = false;

    bool _move3LongRange = true;
    bool _move3AvoidActive = false;
    double _move3AvoidSide = 0.0;
    rclcpp::Time _move3AvoidUntil;
    rclcpp::Time _move3LastCallAt;
    double _move3TargetX = 0.0;
    double _move3TargetY = 0.0;
    bool _move3TargetValid = false;
};
