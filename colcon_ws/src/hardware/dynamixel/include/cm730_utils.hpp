#ifndef _CM730_UTILS_H_
#define _CM730_UTILS_H_
#pragma once

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "servo_utils.hpp"
#include <map>

namespace CM730
{
    class CM730Node: public rclcpp::Node
    {
    private: 
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_legs_goal_pose; 
        rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_leg_left_goal_pose;  
        rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_leg_right_goal_pose;
        rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_arms_goal_pose;
        rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_arm_left_goal_pose;
        rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_arm_right_goal_pose; 
        rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_head_goal_pose;
    
        rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_legs_current_pose;
        rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_leg_left_current_pose;
        rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_leg_right_current_pose;
        rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_arms_current_pose;
        rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_arm_left_current_pose;
        rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_arm_right_current_pose;
        rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_head_current_pose;
        rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_joint_current_angles;
        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_joint_states;
        
        sensor_msgs::msg::JointState     msg_joint_states;
        std_msgs::msg::Float32MultiArray msg_joint_current_angles;

        std::vector<Servo::servo_t> all_servos;

        Servo::CommHandler comm;

        std::vector<uint16_t> present_position; //In Dynamixel bits
        std::vector<uint16_t> goal_position;    //In Dynamixel bits

    public:
        //Constructor
        CM730Node(std::string port_name);

        //Methods
        bool start();
        void stop();
        
        bool fillServoParameters(const std::vector<std::string>& servo_names, std::vector<Servo::servo_t>& servo_list);
        bool readAndPublishAllPositions();
        bool writePresentPositions();
        bool writePositions(const std::vector<Servo::servo_t>& servos);

        //Callbacks
        void callback_legs_goal_pose(const std_msgs::msg::Float32MultiArray::ConstSharedPtr& msg);
        void callback_leg_left_goal_pose(const std_msgs::msg::Float32MultiArray::ConstSharedPtr& msg);
        void callback_leg_right_goal_pose(const std_msgs::msg::Float32MultiArray::ConstSharedPtr& msg); 
        void callback_arms_goal_pose(const std_msgs::msg::Float32MultiArray::ConstSharedPtr& msg);
        void callback_arm_left_goal_pose(const std_msgs::msg::Float32MultiArray::ConstSharedPtr& msg);
        void callback_arm_right_goal_pose(const std_msgs::msg::Float32MultiArray::ConstSharedPtr& msg);
        void callback_head_goal_pose(const std_msgs::msg::Float32MultiArray::ConstSharedPtr& msg);
    };
}

#endif