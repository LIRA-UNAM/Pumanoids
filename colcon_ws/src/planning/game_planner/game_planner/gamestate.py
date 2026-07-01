#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from construct import (
    Struct,
    Byte,
    Enum,
    Const,
    Array,
    Flag,
    Int16ul,
    Int16sl,
    Float32b,
)

# ---------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------

GAMECONTROLLER_DATA_PORT = 3838
GAMECONTROLLER_RETURN_PORT = 3939

GAMECONTROLLER_STRUCT_VERSION = 20
GAMECONTROLLER_RETURN_STRUCT_VERSION = 4

MAX_NUM_PLAYERS = 20


# ---------------------------------------------------------------------
# RobotInfo
# ---------------------------------------------------------------------

RobotInfo = Struct(
    "penalty" / Enum(
        Byte,
        PENALTY_NONE=0,
        PENALTY_ILLEGAL_POSITIONING=1,
        PENALTY_MOTION_IN_SET=2,
        PENALTY_MOTION_IN_STOP=3,
        PENALTY_LOCAL_GAME_STUCK=4,
        PENALTY_INCAPABLE_ROBOT=5,
        PENALTY_PICK_UP=6,
        PENALTY_BALL_HOLDING=7,
        PENALTY_LEAVING_THE_FIELD=8,
        PENALTY_PLAYING_WITH_ARMS_HANDS=9,
        PENALTY_PUSHING=10,
        PENALTY_CAUTIONED=11,
        PENALTY_SENT_OFF=12,
        PENALTY_SUBSTITUTE=13,
    ),
    "secsTillUnpenalised" / Byte,
    "cautions" / Byte,
)

# ---------------------------------------------------------------------
# TeamInfo
# ---------------------------------------------------------------------

ColourEnum = Enum(
    Byte,
    BLUE=0,
    RED=1,
    YELLOW=2,
    BLACK=3,
    WHITE=4,
    GREEN=5,
    ORANGE=6,
    PURPLE=7,
    BROWN=8,
    GRAY=9,
)

TeamInfo = Struct(
    "teamNumber" / Byte,
    "fieldPlayerColour" / ColourEnum,
    "goalkeeperColour" / ColourEnum,
    "goalkeeper" / Byte,
    "score" / Byte,
    "penaltyShot" / Byte,
    "singleShots" / Int16ul,
    "messageBudget" / Int16ul,
    "players" / Array(MAX_NUM_PLAYERS, RobotInfo),
)

# ---------------------------------------------------------------------
# RoboCupGameControlData
# ---------------------------------------------------------------------

GameState = Struct(
    "header" / Const(b"RGme"),
    "version" / Byte,
    "packetNumber" / Byte,
    "playersPerTeam" / Byte,

    "competitionType" / Enum(
        Byte,
        SMALL=0,
        MIDDLE=1,
        LARGE=2,
    ),

    "stopped" / Flag,

    "gamePhase" / Enum(
        Byte,
        GAME_PHASE_NORMAL=0,
        GAME_PHASE_PENALTY_SHOOT_OUT=1,
        GAME_PHASE_EXTRA_TIME=2,
        GAME_PHASE_TIMEOUT=3,
    ),

    "state" / Enum(
        Byte,
        STATE_INITIAL=0,
        STATE_READY=1,
        STATE_SET=2,
        STATE_PLAYING=3,
        STATE_FINISHED=4,
    ),

    "setPlay" / Enum(
        Byte,
        SET_PLAY_NONE=0,
        SET_PLAY_DIRECT_FREE_KICK=1,
        SET_PLAY_INDIRECT_FREE_KICK=2,
        SET_PLAY_PENALTY_KICK=3,
        SET_PLAY_THROW_IN=4,
        SET_PLAY_GOAL_KICK=5,
        SET_PLAY_CORNER_KICK=6,
    ),

    "firstHalf" / Flag,

    "kickingTeam" / Byte,

    "secsRemaining" / Int16sl,

    "secondaryTime" / Int16sl,

    "teams" / Array(2, TeamInfo),
)

# ---------------------------------------------------------------------
# RoboCupGameControlReturnData
# ---------------------------------------------------------------------

ReturnData = Struct(
    "header" / Const(b"RGrt"),
    "version" / Byte,
    "playerNum" / Byte,
    "teamNum" / Byte,
    "fallen" / Byte,

    "pose" / Array(3, Float32b),

    "ballAge" / Float32b,

    "ball" / Array(2, Float32b),
)
