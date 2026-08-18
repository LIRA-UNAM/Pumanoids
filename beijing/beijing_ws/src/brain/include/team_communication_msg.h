#pragma once

#include <cstdint>
#include <type_traits>

#include "RoboCupGameControlData.h"
#include "types.h"

#define VALIDATION_COMMUNICATION 31204
#define VALIDATION_DISCOVERY 41203
#define TEAM_COMMUNICATION_PROTOCOL_VERSION 6
struct TeamCommunicationMsg
{
    int validation = VALIDATION_COMMUNICATION; // validate msg, to determine if it's sent by us.
    uint16_t protocolVersion = TEAM_COMMUNICATION_PROTOCOL_VERSION;
    uint16_t payloadBytes = 0;
    uint64_t bootId = 0;
    uint32_t sequence = 0;
    int communicationId;
    int teamId;
    int playerId;
    int playerRole; // 1: striker, 2: goal_keeper, 3: unknown
    int playerStartRole; // Initially configured role, used for temporary goalkeeper replacement and recovery
    bool isAlive; // On the field and not penalized
    bool isLead; // Currently controlling the ball
    bool ballDetected;
    bool ballLocationKnown;
    uint32_t ballObservationAgeMs;
    double ballConfidence;
    double ballRange;
    double cost; // Estimated cost from the current state to reaching a kickable ball position
    Point ballPosToField;
    Pose2D robotPoseToField;
    double kickDir;
    double thetaRb;
    int ballOwnerId;
    uint32_t leaderTerm;
    int formationOwnerId;
    uint32_t formationRevision;
    int formationShadowSide;
    int assistSlots[MAX_NUM_PLAYERS];
    int assistSlot;
    int assistPhase;
    Point2D assistTarget;
    bool kickoffActive;
    int kickoffOwnerId;
    uint32_t kickoffOwnerTerm;
    uint64_t freeKickProposerBootId;
    uint32_t freeKickRefereePacketNumber;
    uint32_t freeKickEventId;
    uint32_t freeKickLeaderTerm;
    int freeKickType;
    int freeKickPhase;
    bool freeKickOurRestart;
    int freeKickKickerId;
    int freeKickReceiverId;
    Point freeKickBall;
    Point2D freeKickPassTarget;
    int freeKickAssistSlots[MAX_NUM_PLAYERS];
    uint32_t freeKickExecutionElapsedMs;
    int cmdId; // Incremented for every player command to preserve publication order
    int cmd; // 100 means this player takes the ball while other strikers switch to support
};

static_assert(std::is_trivially_copyable<TeamCommunicationMsg>::value,
              "TeamCommunicationMsg must remain safe for the existing UDP transport");
static_assert(sizeof(TeamCommunicationMsg) <= UINT16_MAX,
              "TeamCommunicationMsg no longer fits payloadBytes");

struct TeamDiscoveryMsg
{
    int validation = VALIDATION_DISCOVERY; // validate msg, to determine if it's sent by us.
    int communicationId;
    int teamId;
    int playerId;
};
