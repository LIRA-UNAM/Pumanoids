#pragma once

#include <string>
#include <ostream>

#include "types.h"
#include "utils/math.h"
#include "RoboCupGameControlData.h"


using namespace std;

/**
 * Stores Brain configuration values that are fixed at initialization and read-only during decisions.
 * Values that change during decision-making belong in BrainData.
 * Configuration is loaded from config/config.yaml, then overridden by config/config_local.yaml when present.
 * 
 */
class BrainConfig
{
public:
    // ---------- start config from config.yaml ---------------------------------------------
    // Raw values loaded from configuration. Add matching storage here when adding a parameter.
    // BrainNode overwrites these values, so defaults must be set through declare_parameter.
    int teamId;                 // game.team_id
    int playerId;               // game.player_id
    string fieldType;           // game.field_type: "adult_size" (14x9) or "kid_size" (9x6)
    string playerRole;          // game.player_role: "striker" or "goal_keeper"
    string playerStartPos;      // game.player_start_pos: "left" or "right" for entry localization
    
    double robotHeight;         // robot.robot_height in meters, used for distance estimation; tunable with SetParam
    double robotOdomFactor;     // Distance scale for /odometer_state x/y; does not reverse direction
    double robotOdomThetaOffset; // Theta offset used when automatic alignment is disabled
    bool robotOdomThetaAutoAlign; // Align odometry x/y axes with the zero-theta direction automatically
    double robotOdomThetaAlignmentDistance; // Minimum accumulated translation before locking the alignment angle
    double robotOdomThetaAlignmentMinConcentration; // Minimum circular concentration of alignment-angle samples
    double vxFactor;            // robot.vx_factor correction when actual vx exceeds the command
    double yawOffset;           // robot.yaw_offset correction for left-biased distance estimates

    bool enableCom;             // enable_com: enable communication

    bool rerunLogEnableTCP;     // rerunLog.enable_tcp: enable TCP streaming
    string rerunLogServerIP;    // rerunLog.server_ip: Rerun server IP address
    bool rerunLogEnableFile;    // rerunLog.enable_file: enable file logging
    string rerunLogLogDir;      // rerunLog.log_dir: Rerun log directory
    double rerunLogMaxFileMins; // Maximum minutes per log file; large files may become unreadable

    int rerunLogImgInterval;    // rerunLog.img_interval: log one image per N messages
    double rerunLogVisualHz;    // Maximum rate for high-frequency 2D/3D visualization
    double rerunLogTimeseriesHz; // Maximum rate for continuous scalar time series
    double rerunLogDebugHz;     // Maximum rate for repeated per-frame debug text

    bool odomLogEnable;         // Enable a separate odometry diagnostic log
    string odomLogDir;          // Empty means reuse the current Rerun session directory
    double odomLogHz;           // /odometer_state sampling rate
    double odomLogFlushIntervalMs; // Background file-flush interval
    
    string treeFilePath;        // Behavior-tree path supplied by launch.py rather than config.yaml
    // ----------  end config from config.yaml ---------------------------------------------

    // Match parameters
    FieldDimensions fieldDimensions; // Field dimensions
    vector<FieldLine> mapLines;       // Expected map lines used to match observed lines
    vector<MapMarking> mapMarkings;   // Expected map markings used to match observed markings
    
    int numOfPlayers = 2;             // Robot count; two or more enables automatic decision switching

    // Obstacle-avoidance parameters
    double collisionThreshold;        // Avoid obstacles closer than this collision threshold
    double safeDistance;              // Avoid obstacles closer than this safety distance
    double avoidSecs;                 // Duration of each obstacle-bypass maneuver

    // Camera parameters; configuration values override these defaults.
    // Image dimensions
    double camPixX = 1280;
    double camPixY = 720;

    // Camera field of view
    double camAngleX = deg2rad(90);
    double camAngleY = deg2rad(65);

    // Camera intrinsics
    double camfx = 643.898;
    double camfy = 643.216;
    double camcx = 649.038;
    double camcy = 357.21;

    // Depth-camera intrinsics. T2 depth images are aligned with RGB by default,
    // so missing depth calibration falls back to the RGB intrinsics above.
    double depthfx = 643.898;
    double depthfy = 643.216;
    double depthcx = 649.038;
    double depthcy = 357.21;
    double depthScale16U = 0.001;  // T2 16UC1/mono16: millimetres -> metres
    double depthScale32F = 0.001;  // T2 32FC1 numeric values are millimetres
    bool depthAlignedToColor = true;
    bool depthIsZ = true;          // depth image stores optical-axis Z, not range
    // Kept for configuration compatibility. The 2-D occupancy map always
    // projects transformed depth points vertically by retaining x/y.
    bool depthProjectToGround = true;

    // Camera extrinsics
    Eigen::Matrix4d camToHead;


    // Head soft limits
    double headYawLimitLeft = 1.1;
    double headYawLimitRight = -1.1;
    double headPitchLimitUp = 0.2; // Sufficient to see the field while excluding distant visual noise

    // Velocity limits
    double vxLimit = 1.2;
    double vyLimit = 0.4;
    double vthetaLimit = 1.5;
    double minVx = 0.4;
    double minVy = 0.4;
    double minVtheta = 0.4;

    // Strategy parameters
    double safeDist = 2.0;                  // Collision-detection safety distance
    double goalPostMargin = 0.4;
    double goalPostMarginForTouch = 0.1; // Inset applied to goalposts when calculating a safe touch angle
    double ballConfidenceThreshold;        // Values below this are not balls; current detection confidence is above 20
    double ballConfidenceDecayRate;        // Seconds for a confidence-100 observation to decay; weaker observations expire sooner
    bool enableStableKick = false;          // Stabilize before kicking when risk is low
    bool treatPersonAsRobot = false;        // Treat people as robots for debugging
    double ballOutThreshold = 0.25;         // Threshold used to determine whether the ball is out
    double tmBallDistThreshold = 2.0;       // Trust a teammate's field-frame ball position only beyond this local-distance threshold
    bool limitNearBallSpeed = true;         // Limit chase speed near the ball
    double nearBallSpeedLimit = 0.6;        // Speed limit near the ball
    double nearBallRange = 2.0;             // Range within which near-ball speed limiting applies

    // Localization parameters
    int pfMinMarkerCnt = 5; // Minimum observed markings required for particle-filter localization
    double pfMaxResidual = 0.3; // Reject localization when the final weighted residual exceeds this value

    // Sound parameters
    bool soundEnable = false;
    string soundPack = "espeak"; // "espeak" or a pack name whose files are under sound_play/sounds/<name>/

    // RLVisionKick parameters
    string RLVisionKickVisualKickVersion; // RLVisionKick.visual_kick_version: kV1 or kV2

    // Calculate expected field lines and marking positions.
    void calcMapLines();
    void calcMapMarkings();

    // Validate and derive configuration after BrainNode populates the parameters; return true on success.
    void handle();
    
    // Write configuration details to a stream for debugging.
    void print(ostream &os);
};
