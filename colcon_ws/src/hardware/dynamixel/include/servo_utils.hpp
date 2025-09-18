#ifndef _SERVO_UTILS_H_
#define _SERVO_UTILS_H_
#pragma once

#include "dynamixel_sdk/dynamixel_sdk.h"
#include <string>
#define ID_CM730    200                    //-> ROS THINKS CM730 IS A SERVO

#define SERVO_MX_BITS_PER_RAD       651.898646904   // Servos have 4096 different positions, 
                                                    // since there are 2  PI radians in a circumference, 
                                                    // each radian has 4096/2*PI = 651.898646904 BITS PER RADIAN

#define SERVO_MX_RADS_PER_BIT       0.001533981     // Inverse of SERVO_MX_BITS_PER_RAD

#define SERVO_PROTOCOL_VERSION      1.0             //  SERVOS USED BY THE HUMANOID USE PROTOCOL 1.0

enum MX64
{
    // EEPROM AREA
    MODEL_NUMBER        = 0,
    FIRMWARE_VERSION    = 2,
    ID                  = 3,
    BAUD_RATE           = 4,
    RETURN_DELAY_T      = 5,
    CW_ANGLE_LIMIT      = 6,
    CCW_ANGLE_LIMIT     = 8,
    TEMP_LIMIT          = 11,
    MIN_VOLT_LIMIT      = 12,
    MAX_VOLT_LIMIT      = 13,
    MAX_TORQUE          = 14,
    STATUS_RETURN_LEVEL = 16,
    ALARM_LED           = 17,
    SHUTDOWN_INFO       = 18,
    MULTI_TURN_OFFSET   = 20,
    RESOLUTION_DIVIDER  = 22,

    // RAM AREA
    TORQUE_ENABLE       = 24,
    LED_STATUS          = 25,
    D_GAIN              = 26,
    I_GAIN              = 27,
    P_GAIN              = 28,
    GOAL_POSITION       = 30,
    MOVING_SPEED        = 32,
    TORQUE_LIMT         = 34,
    PRESENT_POSITION    = 36,
    PRESENT_SPEED       = 38,
    PRESENT_LOAD        = 40,
    PRESENT_VOLTAGE     = 42,
    PRESENT_TEMP        = 43,
    REGISTERED          = 44,
    MOVING              = 46,
    LOCK                = 47,
    PUNCH               = 48,
    REALTIME_KICK       = 50,
    CURRENT             = 68,
    TORQUE_CONTROL      = 70,
    GOAL_TORQUE         = 71,
    GOAL_ACCELERATION   = 73
};

namespace Servo
{
    struct servo_t
    {
        int id;
        std::string joint_name;
        int     cw;
        uint16_t zero;
        bool    enabled;
        bool    is4Pin;
    };

    class CommHandler
    {
    private:
        const char * portName;
        dynamixel::PortHandler   *port_h   ; 
        dynamixel::PacketHandler *packet_h ;
        dynamixel::GroupBulkRead  bulk_read_current_position_3pin;
        dynamixel::GroupBulkRead  bulk_read_current_position_4pin;
        dynamixel::GroupSyncWrite sync_write_goal_position ;
        long baudrate {1000000};

    public:
        //Public registered ids
        std::vector<Servo::servo_t> registered_servos;
        
        // Set Constructor
        CommHandler(const std::string& portName);

        // Methods
        bool    startComm();
        bool    wakeupAllServos(bool torque_enable);
        bool    portIsActive();
        uint16_t getPosition(const int& id);
        bool    setPosition(const int& id, const uint16_t& pos);
        bool    shutdownAllServos();
        bool    getAllPositions(std::vector<uint16_t>& present_position);
        bool    setPositions(const std::vector<uint16_t>& position_vector, const std::vector<Servo::servo_t>& all_servos);

        //Create and configure bulkRead and SyncWrite
        bool    registerIDs(std::vector<Servo::servo_t>& servo_list);
        
    };
}

#endif