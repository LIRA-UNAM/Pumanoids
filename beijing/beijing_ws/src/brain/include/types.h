/**
 * @file type.h
 * @brief Defines structures and enumerations used by the brain package.
 */
#pragma once

#include <array>
#include <chrono>
#include <cstdint>
#include <string>
#include <vector>
#include <numeric>
#include <iterator>
#include <limits>
#include <rclcpp/rclcpp.hpp>

#include "RoboCupGameControlData.h"

using namespace std;

/* ------------------ Struct ------------------------*/

// Field dimensions: distances parallel to the goal line are widths; perpendicular distances are lengths.
struct FieldDimensions
{
    double length;            // Field length
    double width;             // Field width
    double penaltyDist;       // Perpendicular distance from penalty mark to goal line
    double goalWidth;         // Goal width
    double circleRadius;      // Center-circle radius
    double penaltyAreaLength; // Penalty-area length
    double penaltyAreaWidth;  // Penalty-area width
    double goalAreaLength;    // Goal-area length
    double goalAreaWidth;     // Goal-area width
                              // Naming follows the competition rules; the penalty area is larger than the goal area.
};
const FieldDimensions FD_KIDSIZE{9, 6, 1.5, 2.6, 0.75, 2, 5, 1, 3};
// const FieldDimensions FD_ADULTSIZE{14, 9, 2.1, 2.6, 1.5, 3, 6, 1, 4};
const FieldDimensions FD_ADULTSIZE{14.16, 9.22, 2.242, 2.6, 1.54, 3.24, 6.192, 1.345, 4};
// const FieldDimensions FD_ROBOLEAGUE{22, 14, 3.6, 2.6, 2, 2.25, 6.9, 0.75, 3.9};
// const FieldDimensions FD_ROBOLEAGUE{22, 14, 3.5, 2.6, 2, 5, 8, 2, 5};
const FieldDimensions FD_ROBOLEAGUE{22.003, 14.126, 3.635, 2.6, 1.99, 5.221, 8.121, 2.307, 5.083};

// A point and orientation in a plane.
struct Pose2D
{
    double x = 0;
    double y = 0;
    double theta = 0; // Radians from the positive x-axis; counterclockwise is positive
};

// Three-dimensional point.
struct Point
{
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
};

// Two-dimensional point.
struct Point2D
{
    double x = 0.0;
    double y = 0.0;
};

// BoundingBox
struct BoundingBox
{
    double xmin = 0.0;
    double xmax = 0.0;
    double ymin = 0.0;
    double ymax = 0.0;
};

// Rich representation of match entities such as balls and goalposts, derived from detection::DetectedObject.
struct GameObject
{
    // --- Populated from /detect messages ---
    string label;                // Detected object class
    string color;                // Opponent color, used to distinguish teams
    BoundingBox boundingBox;     // Image bounding box; origin at top left, x right, y down
    Point2D precisePixelPoint;   // Precise pixel location, available only for field markings
    double confidence = 0.0;     // Detection confidence; for obstacles, count of grid points above threshold
    bool fallen = false;         // Whether this robot-like object may be lying on the field
    double fallenConfidence = 0.0; // Single-frame fallen confidence in [0, 1]
    double uprightScore = 0.0;   // Multi-frame pose evidence: positive is upright, negative is fallen
    Point posToRobot;            // Two-dimensional position in the robot frame; z is ignored
    Point obstacleLeftToRobot;   // Left footprint boundary of a robot obstacle in the ground frame
    Point obstacleRightToRobot;  // Right footprint boundary of a robot obstacle in the ground frame
    Point obstacleLeftToField;
    Point obstacleRightToField;
    bool obstacleFootprintValid = false;
    uint32_t obstacleSeenCount = 1;
    uint32_t obstacleMissedCount = 0;

    // --- Calculated by processDetectedObject ---
    Point posToField;                                 // Two-dimensional position in the field frame; x forward, y left
    double range = 0.0;                               // Planar distance from the robot center to the object
    double pitchToRobot = 0.0;
    double yawToRobot = 0.0;                          // Object pitch/yaw relative to robot forward, radians; down/left positive
    rclcpp::Time timePoint;                           // Detection timestamp

    // --- Object-specific fields; not populated for every class ---
    int id = 0;    // Detected ID
    string name;  // human readable id
    double idConfidence = 0.0; // ID confidence in [0, 1]
    string info; // Additional information
};

// Line in an arbitrary frame, represented by endpoints x0, y0, x1, y1.
struct Line {
    double x0, y0, x1, y1;
};

struct FieldLineRef {
    std::string name;
    double x0, y0, x1, y1; // Endpoints in meters
    bool isVertical;
};

// enum LineID {
//     LT, // left touch line
//     RT, // right touch line
//     M, // middle line
//     SB, // self bottom (goal line)
//     OB, // opposite bottom (goal line)
//     SGL, // self goal area left
//     SGH, // self goal area horizontal
//     SGR, // self goal area right
//     OGL, // opposite goal area left
//     OGH, // opposite goal area horizontal
//     OGR, // opposite goal area right
//     SPL, // self penalty area left
//     SPH, // self penalty area horizontal
//     SPR, // self penalty area right
//     OPL, // opposite penalty area left
//     OPH, // opposite penalty area horizontal
//     OPR // opoosite penalty area right
// };

enum class LineHalf {
    NA,
    Self,
    Opponent
};

enum class LineDir {
    NA,
    Horizontal,
    Vertical
};

enum class LineSide {
    NA,
    Left,
    Right
};

enum class LineType {
    NA,
    TouchLine,
    MiddleLine,
    GoalLine,
    GoalArea,
    PenaltyArea
};

// Field-line representation.
struct FieldLine {
    Line posToField;
    Line posToRobot;
    Line posOnCam;
    rclcpp::Time timePoint;
    LineDir dir;
    LineHalf half;
    LineSide side;
    LineType type;
    double confidence;
};

// Expected map position of a field marking.
struct MapMarking {
    double x;
    double y;
    string type; // TCross | LCross | XCross | PenaltyPoint
    string name; // Four characters, e.g. LSCL: type; half S/M/O; side L/M/R; region B/P/G/C
};

// Camera field-of-view bounds.
struct VisionBox {
    vector<double> posToField;
    vector<double> posToRobot;
    rclcpp::Time timePoint;                           // Detection timestamp
};

struct TimestampedData {
    vector<double> data;
    rclcpp::Time timePoint;
};

enum class AssistSlot : uint8_t {
    NONE = 0,
    COVER_MID = 1,
    SHADOW_SUPPORT = 2,
    ANCHOR_COVER = 3,
    WIDE_OUTLET = 4,
};

enum class AssistPhase : uint8_t {
    HOLD = 0,
    YIELD_LANE = 1,
    TRANSIT_SLOT = 2,
};

inline const char *assistSlotName(AssistSlot slot)
{
    switch (slot) {
    case AssistSlot::COVER_MID: return "cover_mid";
    case AssistSlot::SHADOW_SUPPORT: return "shadow_support";
    case AssistSlot::ANCHOR_COVER: return "anchor_cover";
    case AssistSlot::WIDE_OUTLET: return "wide_outlet";
    default: return "none";
    }
}

inline const char *assistPhaseName(AssistPhase phase)
{
    switch (phase) {
    case AssistPhase::HOLD: return "hold";
    case AssistPhase::YIELD_LANE: return "yield_lane";
    case AssistPhase::TRANSIT_SLOT: return "transit_slot";
    default: return "unknown";
    }
}

// Fall recovery.
struct RobotRecoveryStateData {
    uint8_t state; // IS_READY = 0, IS_FALLING = 1, HAS_FALLEN = 2, IS_GETTING_UP = 3,
    uint8_t is_recovery_available; // 1 for available, 0 for not available
    uint8_t current_planner_index;
};

// Teammate communication state.
struct TMStatus {
    string role = "not initialized"; // triker, goal_keeper
    string startRole = "not initialized"; // Startup role; unchanged by temporary role substitution
    bool isAlive = false; // On the field, unpenalized, and communication is active
    bool ballDetected = false;
    bool ballLocationKnown = false;
    double ballConfidence = 0.;
    double ballRange = 0.;
    double cost = 0.; // Estimated cost from the current state to reaching a kickable ball position
    bool isLead = true; // Currently controlling the ball
    Point ballPosToField;
    Pose2D robotPoseToField;
    double kickDir = 0.; // Planned kick direction
    double thetaRb = 0.; // Robot-to-ball angle in the field frame
    int cmd = 0; // Last command sent
    int cmdId = 0; // ID of the last command sent
    uint64_t bootId = 0; // Sender process identity, used to reject packets from before a restart
    uint32_t sequence = 0; // Sender's UDP packet sequence number
    vector<uint64_t> retiredBootIds; // Retired process IDs whose delayed packets must be rejected
    uint64_t pendingBootId = 0;
    uint32_t pendingBootSequence = 0;
    uint8_t pendingBootPacketCount = 0;
    std::chrono::steady_clock::time_point pendingBootFirstSeen;
    int ballOwnerId = 0;
    uint32_t leaderTerm = 0;
    int formationOwnerId = 0;
    uint32_t formationRevision = 0;
    int formationShadowSide = 0;
    array<int, MAX_NUM_PLAYERS> assistSlots{}; // Complete player-to-slot mapping broadcast by the leader
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
    int freeKickType = 0;
    int freeKickPhase = 0;
    bool freeKickOurRestart = false;
    int freeKickKickerId = 0;
    int freeKickReceiverId = 0;
    Point freeKickBall{0.0, 0.0, 0.0};
    Point2D freeKickPassTarget{0.0, 0.0};
    array<int, MAX_NUM_PLAYERS> freeKickAssistSlots{};
    uint32_t freeKickExecutionElapsedMs = 0;
    rclcpp::Time timeLastCom; // Last communication time
};


enum class RobotRecoveryState {
    IS_READY = 0,
    IS_FALLING = 1,
    HAS_FALLEN = 2,
    IS_GETTING_UP = 3
};
