#pragma once

#include <atomic>
#include <array>
#include <chrono>
#include <string>
#include <mutex>
#include <tuple>

#include <sensor_msgs/msg/image.hpp>
#include "booster_interface/msg/odometer.hpp"


#include "locator.h"
#include "types.h"
#include "RoboCupGameControlData.h"
#include "buffer.h"
#include "freekick_policy.h"

using namespace std;

struct FreeKickPlanState
{
    uint64_t proposerBootId = 0;
    uint32_t refereePacketNumber = 0;
    uint32_t eventId = 0;
    uint32_t leaderTerm = 0;
    freekick_policy::Type type = freekick_policy::Type::None;
    freekick_policy::Phase phase = freekick_policy::Phase::Inactive;
    bool ourRestart = false;
    int kickerId = 0;
    int receiverId = 0;
    Point ballSnapshot{0.0, 0.0, 0.0};
    Point2D passTarget{0.0, 0.0};
    // Assignment captured with the plan. The normal ball owner/assist
    // election may continue to update after a restart has been frozen.
    std::array<AssistSlot, MAX_NUM_PLAYERS> assistSlots{};
    rclcpp::Time phaseChangedAt;
    rclcpp::Time executionStartTime;
};

struct TeamOutboundSnapshot
{
    int playerRole = 0;
    bool isAlive = false;
    bool isLead = false;
    bool ballDetected = false;
    bool ballLocationKnown = false;
    double ballConfidence = 0.0;
    double ballRange = 0.0;
    double cost = 0.0;
    Point ballPosToField{0.0, 0.0, 0.0};
    Pose2D robotPoseToField;
    double kickDir = 0.0;
    double thetaRb = 0.0;
    int ballOwnerId = 0;
    uint32_t leaderTerm = 0;
    int formationOwnerId = 0;
    uint32_t formationRevision = 0;
    int formationShadowSide = 0;
    std::array<AssistSlot, MAX_NUM_PLAYERS> assistSlots{};
    AssistSlot assistSlot = AssistSlot::NONE;
    AssistPhase assistPhase = AssistPhase::HOLD;
    Point2D assistTarget{0.0, 0.0};
    bool kickoffActive = false;
    int kickoffOwnerId = 0;
    uint32_t kickoffOwnerTerm = 0;
    uint64_t freeKickProposerBootId = 0;
    uint32_t freeKickRefereePacketNumber = 0;
    uint32_t freeKickEventId = 0;
    uint32_t freeKickLeaderTerm = 0;
    freekick_policy::Type freeKickType = freekick_policy::Type::None;
    freekick_policy::Phase freeKickPhase = freekick_policy::Phase::Inactive;
    bool freeKickOurRestart = false;
    int freeKickKickerId = 0;
    int freeKickReceiverId = 0;
    Point freeKickBall{0.0, 0.0, 0.0};
    Point2D freeKickPassTarget{0.0, 0.0};
    std::array<AssistSlot, MAX_NUM_PLAYERS> freeKickAssistSlots{};
    uint32_t freeKickExecutionElapsedMs = 0;
    int cmdId = 0;
    int cmd = 0;
};

/**
 * BrainData stores dynamic runtime state used for decisions, unlike the static BrainConfig.
 * Data-processing utilities may also be placed here.
 */
class BrainData
{
public:
    BrainData();
    /* ------------------------------------ Match state ------------------------------------ */

    int score = 0;
    int oppoScore = 0;
    int penalty[MAX_NUM_PLAYERS]; // Penalty state for our robots
    int oppoPenalty[MAX_NUM_PLAYERS]; // Penalty state for opponent robots
    bool isKickingOff = false; // True during the 10-second kickoff window
    rclcpp::Time kickoffStartTime; // Kickoff start time
    bool kickoffSetWasActive = false;
    bool kickoffOwnerLocked = false;
    int kickoffOwnerId = 0;
    uint32_t kickoffOwnerTerm = 0;
    Point kickoffBallSnapshot{0.0, 0.0, 0.0};
    bool kickoffBallSnapshotValid = false;
    // READY/SET placement rank, with the cost-elected kickoff owner at rank 0.
    int kickoffReadyRank = -1;
    bool isFreekickKickingOff = false; // Our free-kick plan is executing after release
    rclcpp::Time freekickKickoffStartTime; // Time when the authoritative kicker confirmed release
    FreeKickPlanState freeKickPlan;
    bool freeKickStopActive = false;
    bool freeKickPlanBlocked = false;
    freekick_policy::Type pendingFreeKickType = freekick_policy::Type::None;
    rclcpp::Time freeKickStopDetectedAt;
    uint32_t gameControllerPacketNumber = 0;
    uint32_t freeKickStopPacketNumber = 0;
    bool gameControllerPacketNumberValid = false;
    bool freeKickStopPacketNumberValid = false;
    bool gcOwnFreeKickStopWasActive = false;
    // Kept separately from the legacy game-state strings because goalkeeper=0
    // has a valid meaning during a penalty shoot-out.
    uint8_t gcGamePhase = GAME_PHASE_NORMAL;
    uint32_t nextFreeKickEventId = 1;
    int liveCount = 0; // Number of active robots on our team
    int oppoLiveCount = 0; // Number of active opponent robots
    string realGameSubState = "NONE"; // Current set-play type

    /* ------------------------------------ Runtime data ------------------------------------ */
    
    // Body pose and velocity commands
    Pose2D robotPoseToOdom;  // Robot pose in the odometry frame, updated by odomCallback
    Pose2D odomToField;      // Odometry-frame origin in the field frame, calibrated from a known pose such as the entry point
    Pose2D robotPoseToField; // Robot pose in the field frame: origin at center, x toward opponent goal, y left, theta counterclockwise

    // Head pose, updated by lowStateCallback
    double headPitch = 0.0; // Radians; zero is level and forward, down is positive
    double headYaw = 0.0;   // Radians; zero is forward, left is positive
    Eigen::Matrix4d camToRobot = Eigen::Matrix4d::Identity();  // Camera-to-robot transform, initialized to identity

    // vector<TimestampedData> headPosBuffer = {};
    // Ball
    bool ballDetected = false;    // The camera currently detects the ball
    std::atomic_bool ballEverDetected{false}; // At least one ball detection, used for GameController ballAge
    GameObject ball{};              // Ball data including position and bounding box
    GameObject tmBall{};            // Ball data received from a teammate
    double robotBallAngleToField = 0.0; // Field-frame angle from the x-axis to the robot-to-ball vector, in (-PI, PI]
    bool lose_ball = false;       // Visual ball-loss state used to exit visual kick
    vector<array<double, 2>> predictedBallPos; // Predicted field-frame ball positions in meters as {x, y}
    vector<array<double, 2>> ballPredictionObservations; // Recent measured field-frame positions
    rclcpp::Time ballPosPredictTime; // Timestamp of the last ball-position prediction
    bool ballWillBreach = false; // Whether the ball will pass the robot
    Point2D ballBreachPoint; // Ball position when it passes the robot
    rclcpp::Time ballBreachTime; // Estimated pass time
    Point2D ballInterceptPoint; // Best interception point
    rclcpp::Time ballInterceptTime; // Time when the ball reaches the interception point
    double ballVelocityX = 0.0; // Predicted field-frame velocity toward the opponent goal (m/s)
    double ballVelocityY = 0.0; // Predicted field-frame lateral velocity (m/s)
    double ballPredictionSpeed = 0.0; // Magnitude of the fitted velocity (m/s)
    double ballPredictionRSquared = 0.0; // Linear-fit quality used by goalkeeper prediction
    double ballPredictionRSquaredX = 0.0; // Longitudinal fit quality used by the acceptance gate
    double ballPredictionRSquaredY = 0.0; // Lateral fit quality, diagnostic only
    double ballPredictionResidual = 0.0; // RMS fit residual in meters
    double ballPredictionSampleSpanSec = 0.0; // Time covered by retained observations
    double ballPredictionObservationAgeSec = 0.0; // Age of the newest observation
    double ballTimeToIntercept = 0.0; // Seconds until the predicted defensive-line crossing
    int ballPredictionSampleCount = 0;
    bool goalkeeperPredictionEnabled = false;
    bool goalkeeperPredictionLocalizationReady = false;
    bool goalkeeperPredictionRequireLocalization = true;
    bool ballPredictionValid = false;
    bool ballPredictionFitComputed = false;
    bool ballMovingTowardOwnGoal = false;
    bool goalkeeperPostBlockClearance = false;
    bool goalkeeperUrgentBlock = false;
    bool goalkeeperForwardInterceptActive = false;
    bool goalkeeperFrontIntercept = false;
    double goalkeeperMeasuredOdomSpeed = 0.0;
    double goalkeeperAdaptiveReachSpeed = 0.0;
    double goalkeeperBlockTargetFieldX = 0.0;
    double goalkeeperBlockTargetFieldY = 0.0;
    double goalkeeperBlockTargetTime = 0.0;
    double goalkeeperBlockTargetRobotX = 0.0;
    double goalkeeperBlockTargetRobotY = 0.0;
    string ballPredictionReason = "initializing";
    string goalkeeperDecision = "initializing";

    // Robots
    inline vector<GameObject> getRobots() const {
        std::lock_guard<std::mutex> lock(_robotsMutex);
        return _robots;
    }
    inline void setRobots(const vector<GameObject>& newVec) {
        std::lock_guard<std::mutex> lock(_robotsMutex);
        _robots = newVec;
    }

    // Goalposts
    inline vector<GameObject> getGoalposts() const {
        std::lock_guard<std::mutex> lock(_goalpostsMutex);
        return _goalposts;
    }
    inline void setGoalposts(const vector<GameObject>& newVec) {
        std::lock_guard<std::mutex> lock(_goalpostsMutex);
        _goalposts = newVec;
    }

    // Detected field-marking intersections
    inline vector<GameObject> getMarkings() const {
        std::lock_guard<std::mutex> lock(_markingsMutex);
        return _markings;
    }
    inline void setMarkings(const vector<GameObject>& newVec) {
        std::lock_guard<std::mutex> lock(_markingsMutex);
        _markings = newVec;
    }

    // Detected field lines
    inline vector<FieldLine> getFieldLines() const {
        std::lock_guard<std::mutex> lock(_fieldLinesMutex);
        return _fieldLines;
    }
    inline void setFieldLines(const vector<FieldLine>& newVec) {
        std::lock_guard<std::mutex> lock(_fieldLinesMutex);
        _fieldLines = newVec;
    }

    // Detected field obstacles
    inline vector<GameObject> getObstacles() const {
        std::lock_guard<std::mutex> lock(_obstaclesMutex);
        return _obstacles;
    }
    inline void setObstacles(const vector<GameObject>& newVec) {
        std::lock_guard<std::mutex> lock(_obstaclesMutex);
        _obstacles = newVec;
    }

    /* ------------------------------------ Buffered Data ------------------------------------ */
    Buffer<sensor_msgs::msg::Image> imageBuffer{50};        // Image buffer
    Buffer<sensor_msgs::msg::Image> depthImageBuffer{50};   // Depth-image buffer
    Buffer<GameObject> detectionBuffer{50};                  // Detected-object buffer
    Buffer<booster_interface::msg::Odometer> odomBuffer{50}; // Odometry buffer
    

    // Motion planning
    double kickDir = 0.; // Planned kick direction in the field frame
    string kickType = "shoot"; // "shoot" | "cross" | "block"
    bool isDirectShoot = false; // True while executing a direct-free-kick plan


    // Multi-robot coordination; tm means teammate
    TMStatus tmStatus[MAX_NUM_PLAYERS]; // Teammate states indexed by player_id - 1
    int tmCmdId = 0; // Latest team command ID, incremented whenever a player publishes a new command
    rclcpp::Time tmLastCmdChangeTime; // Last time a command was sent or received
    int tmMyCmd = 0; // Last command sent by this robot
    int tmMyCmdId = 0; // ID of the last command sent by this robot
    int tmReceivedCmd = 0; // Updated when a different cmdId arrives, then reset to zero after execution
    bool tmImLead = true; // This robot currently controls the ball
    bool tmImAlive = true; // This robot is active according to the GameController
    double tmMyCost = 0.; // Approximate seconds required for this robot to reach and kick the ball
    bool tmMyCostInitialized = false;
    int tmMyCostRank = 0; // Cost rank among active strikers; -1 when excluded
    int myStrikerIDRank = 0; // This robot's ID rank among strikers
    bool tmImInVisualKick = false; // This robot is in visual-kick mode
    bool shouldExitRLVisionKick = false; // Visual-kick mode should exit proactively

    // Leader and formation state. Cost rank is diagnostic/set-play data and does not select normal-play positions.
    int tmBallOwnerId = 0;
    int tmPreviousBallOwnerId = 0;
    int tmPendingBallOwnerId = 0;
    uint32_t tmLeaderTerm = 0;
    std::chrono::steady_clock::time_point tmPendingOwnerSince;
    std::chrono::steady_clock::time_point tmOwnerAdoptedAt;
    std::chrono::steady_clock::time_point tmOwnerlessSince;

    int tmFormationOwnerId = 0;
    uint32_t tmFormationRevision = 0;
    uint32_t tmFormationRosterMask = 0;
    Point tmFormationBall{0.0, 0.0, 0.0};
    Pose2D tmFormationLeaderPose;
    double tmFormationKickDir = 0.0;
    int tmFormationShadowSide = 0;
    std::chrono::steady_clock::time_point tmFormationShadowSideChangedAt;
    std::array<AssistSlot, MAX_NUM_PLAYERS> tmAssistSlots{};
    AssistSlot tmMyAssistSlot = AssistSlot::NONE;
    Pose2D tmAssistTarget;
    bool tmAssistTargetValid = false;
    std::chrono::steady_clock::time_point tmAssistTargetUpdatedAt;

    AssistPhase tmAssistPhase = AssistPhase::HOLD;
    std::chrono::steady_clock::time_point tmAssistPhaseChangedAt;
    int tmAssistYieldSide = 0;
    double tmAssistYieldTimeoutMs = 0.0;
    bool tmAssistYieldRouteExtended = false;
    Pose2D tmAssistWaypoint;
    bool tmAssistWaypointValid = false;
    std::chrono::steady_clock::time_point tmAssistWaypointLockedAt;
    std::chrono::steady_clock::time_point tmAssistWaypointLastProgressAt;
    double tmAssistWaypointBestDistance = 1e9;
    int tmAssistWaypointFailureCount = 0;

    // Stable recovery state after the original goalkeeper returns.
    int tmInitialGoalkeeperId = 0;
    int tmAssignedGoalkeeperId = 0;
    bool tmInitialGoalkeeperUnavailable = false;
    std::chrono::steady_clock::time_point tmGoalkeeperRestoreReadySince;

    // Separate locks for inbound teammates, local coordination, and complete outbound snapshots.
    mutable std::mutex teamStatusMutex;
    mutable std::mutex cooperationMutex;
    TeamOutboundSnapshot tmOutboundSnapshot;
    mutable std::mutex teamOutboundMutex;

    // Original goalkeeper and active player count reported by GameController; protected by recoveryStateMutex_.
    int gcGoalkeeperId = 0;
    int gcPlayersPerTeam = 0;

    // Communication
    int discoveryMsgId = 0;
    rclcpp::Time discoveryMsgTime;
    int sendId = 0;
    rclcpp::Time sendTime;
    int receiveId[MAX_NUM_PLAYERS];
    rclcpp::Time receiveTime[MAX_NUM_PLAYERS];
    string tmIP;
    
    // Fall recovery
    RobotRecoveryState recoveryState = RobotRecoveryState::IS_READY;
    bool isRecoveryAvailable = false; // Recovery is available
    int currentRobotModeIndex = -1;
    int recoveryPerformedRetryCount = 0; // Number of recovery attempts
    bool recoveryPerformed = false;

    enum class FallDownSide {
        FRONT,
        BACK
    };
    FallDownSide fallDownSide = FallDownSide::FRONT;
    int64_t lastGetupStartedUs = 0;
    int64_t lastGetupMovementDetectedUs = 0;
    double frontGetUpTimeSec = 6.5;
    double backGetUpTimeSec = 6.5;
    double noMovementTimeoutSec = 1.0;
    double getupNoMovementGraceSec = 3.0;
    double movementThresholdRadPerSec = 0.1;
    double fallenThresholdRad = 0.99;

    // Diagnostics
    rclcpp::Time timeLastDet; // Timestamp of the last Detection message
    bool camConnected = false; // Set true when the first image arrives
    rclcpp::Time timeLastLineDet; // Timestamp of the last FieldLine Detection message
    rclcpp::Time lastSuccessfulLocalizeTime;
    rclcpp::Time timeLastGamecontrolMsg; // Timestamp of the last GameController message
    rclcpp::Time timeLastLogSave; // Last log-file rollover time, used to keep files readable
    VisionBox visionBox;  
    rclcpp::Time lastTick; 


    /**
     * @brief Return markings by type.
     * 
     * @param type Empty for all types, or a set containing LCross, TCross, XCross, and/or PenaltyPoint.
     * 
     * @return Markings matching the requested types.
     */
    vector<GameObject> getMarkingsByType(set<string> types={});

    // Return field markings usable by the locator.
    vector<FieldMarker> getMarkersForLocator();

    // Transform a pose from the robot frame to the field frame.
    Pose2D robot2field(const Pose2D &poseToRobot);

    // Transform a pose from the field frame to the robot frame.
    Pose2D field2robot(const Pose2D &poseToField);

private:
    vector<GameObject> _robots = {}; 
    mutable std::mutex _robotsMutex;

    vector<GameObject> _goalposts = {}; 
    mutable std::mutex _goalpostsMutex;

    vector<GameObject> _markings = {};                             
    mutable std::mutex _markingsMutex;

    vector<FieldLine> _fieldLines = {};
    mutable std::mutex _fieldLinesMutex;

    vector<GameObject> _obstacles = {};
    mutable std::mutex _obstaclesMutex;

};
