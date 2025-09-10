#include "servo_utils.hpp"
#include "cm730_utils.hpp"
#include "rclcpp/rclcpp.hpp"

#include <algorithm>

uint16_t servos_goal_position[21];  // PENDING TO BE READ
std::vector<Servo::servo_t> left_arm_servos;
std::vector<Servo::servo_t> right_arm_servos;
std::vector<Servo::servo_t> left_leg_servos;
std::vector<Servo::servo_t> right_leg_servos;
std::vector<Servo::servo_t> head_servos;

std::vector<std::string> left_arm_names
{
    "/left/arm/shoulder/pitch",
    "/left/arm/shoulder/roll",
    "/left/arm/elbow/pitch"
};

std::vector<std::string> right_arm_names
{
    "/right/arm/shoulder/pitch",
    "/right/arm/shoulder/roll",
    "/right/arm/elbow/pitch"
};

std::vector<std::string> left_leg_names
{
    "/left/leg/hip/yaw",
    "/left/leg/hip/roll",
    "/left/leg/hip/pitch",
    "/left/leg/knee/pitch",
    "/left/leg/ankle/pitch",
    "/left/leg/ankle/roll"
};

std::vector<std::string> right_leg_names
{
    "/right/leg/hip/yaw",
    "/right/leg/hip/roll",
    "/right/leg/hip/pitch",
    "/right/leg/knee/pitch",
    "/right/leg/ankle/pitch",
    "/right/leg/ankle/roll"
};

std::vector<std::string> head_names
{
    "/head/yaw",
    "/head/pitch"
};

namespace CM730
{
    CM730Node::CM730Node(std::string port_name): comm{port_name}
	    {
        sub_legs_goal_pose          = this->create_subscription<std_msgs::msg::Float32MultiArray>("legs_goal_pose", 10, std::bind(&callback_legs_goal_pose, this, _1));
        sub_leg_left_goal_pose      = this->subscribe("leg_left_goal_pose",  1, &CM730Node::callback_leg_left_goal_pose,    this);
        sub_leg_right_goal_pose     = this->subscribe("leg_right_goal_pose", 1, &CM730Node::callback_leg_right_goal_pose,   this);
        sub_arms_goal_pose          = this->subscribe("arms_goal_pose",      1, &CM730Node::callback_arms_goal_pose,        this);
        sub_arm_left_goal_pose      = this->subscribe("arm_left_goal_pose",  1, &CM730Node::callback_arm_left_goal_pose,    this);
        sub_arm_right_goal_pose     = this->subscribe("arm_right_goal_pose", 1, &CM730Node::callback_arm_right_goal_pose,   this);
        sub_head_goal_pose          = this->subscribe("head_goal_pose",      1, &CM730Node::callback_head_goal_pose,        this);
        
        pub_legs_current_pose       = this->advertise<std_msgs::msg::float32_multi_array>("legs_current_pose",       1);
        pub_leg_left_current_pose   = this->advertise<std_msgs::msg::float32_multi_array>("leg_left_current_pose",   1);
        pub_leg_right_current_pose  = this->advertise<std_msgs::msg::float32_multi_array>("leg_right_current_pose",  1);
        pub_arms_current_pose       = this->advertise<std_msgs::msg::float32_multi_array>("arms_current_pose",       1);
        pub_arm_left_current_pose   = this->advertise<std_msgs::msg::float32_multi_array>("arm_left_current_pose",   1);
        pub_arm_right_current_pose  = this->advertise<std_msgs::msg::float32_multi_array>("arm_right_current_pose",  1);
        pub_head_current_pose       = this->advertise<std_msgs::msg::float32_multi_array>("head_current_pose",       1);
        pub_joint_current_angles    = this->advertise<std_msgs::msg::float32_multi_array>("joint_current_angles",    1);
        pub_joint_states            = this->advertise<sensor_msgs::JointState>("/joint_states", 1);

        msg_joint_current_angles.data.resize(21);
        msg_joint_states.name.resize(21);
        msg_joint_states.position.resize(21);

    }

    bool CM730Node::start()
    {
        bool torque_enable_param;
        if (!ros::param::get("torque_enable", torque_enable_param))
        {
            ROS_ERROR("Missing param in config file: %s", "torque_enable");
            return false;
        }
        if(!torque_enable_param)
        {
            ROS_INFO("TORQUE ENABLED IS FALSE");
        }
        
        if(!fillServoParameters(left_arm_names,  left_arm_servos))  return false;
        if(!fillServoParameters(right_arm_names, right_arm_servos)) return false;
        if(!fillServoParameters(left_leg_names,  left_leg_servos))  return false;
        if(!fillServoParameters(right_leg_names, right_leg_servos)) return false;
        if(!fillServoParameters(head_names,      head_servos))      return false;

        {int counter{0};
        while( !comm.startComm() )
        {
            counter++;
            ros::Duration(1.0).sleep();
            if(counter > 10)
            {
                ROS_ERROR("Shutting down after attempting to open port. Shutting down");
                return false;
            };
        }}
        

        std::copy(left_arm_servos.begin(), left_arm_servos.end(), std::back_inserter(all_servos));
        std::copy(right_arm_servos.begin(), right_arm_servos.end(), std::back_inserter(all_servos));
        std::copy(left_leg_servos.begin(), left_leg_servos.end(), std::back_inserter(all_servos));
        std::copy(right_leg_servos.begin(), right_leg_servos.end(), std::back_inserter(all_servos));
        std::copy(head_servos.begin(), head_servos.end(), std::back_inserter(all_servos));

        if(!comm.registerIDs(all_servos))  return false;
        if(!comm.wakeupAllServos(torque_enable_param)) return false;
        ros::Duration(1.0).sleep();
        
        present_position.resize(21);
        goal_position.resize(21);

        return true;
    }

    void CM730Node::stop()
    {
        if(!comm.shutdownAllServos())
        {
            std::cout << "ERROR: Manual shutdown required." << std::endl;
        }
    }

    void CM730Node::callback_legs_goal_pose(const std_msgs::Float32MultiArray::ConstPtr& msg)
    {
        if(msg->data.size() != 12)
        {
           std::cout << "CM730.->Error!!: goal position for both legs must be a 12-value array." << std::endl;
           return;
        }
        
        int msg_idx{0};
        for(auto servo: left_leg_servos)
        {
            goal_position[servo.id] = uint16_t(msg->data[msg_idx++] * SERVO_MX_BITS_PER_RAD * servo.cw + servo.zero);
        }
        for(auto servo: right_leg_servos)
        {
            goal_position[servo.id] = uint16_t(msg->data[msg_idx++] * SERVO_MX_BITS_PER_RAD * servo.cw + servo.zero);
        }
        std::vector<Servo::servo_t> legs;
        legs.insert(legs.end(), left_leg_servos.begin(), left_leg_servos.end());
        legs.insert(legs.end(), right_leg_servos.begin(), right_leg_servos.end());
        writePositions(legs);
    }

    void CM730Node::callback_leg_left_goal_pose(const std_msgs::Float32MultiArray::ConstPtr& msg)
    {
        if(msg->data.size() != 6)
        {
           std::cout << "CM730.->Error!!: goal position for left leg must be a 6-value array." << std::endl;
           return;
        }
        int msg_idx{0};
        for(auto servo: left_leg_servos)
        {
            goal_position[servo.id] = uint16_t(msg->data[msg_idx++] * SERVO_MX_BITS_PER_RAD * servo.cw + servo.zero);
        }
        writePositions(left_leg_servos);
    }

    void CM730Node::callback_leg_right_goal_pose(const std_msgs::Float32MultiArray::ConstPtr& msg)
    {
        if(msg->data.size() != 6)
        {
            std::cout << "CM730.->Error!!: goal position for right leg must be a 6-value array." << std::endl;
            return;
        }
        int msg_idx{0};
        for(auto servo: right_leg_servos)
        {
            goal_position[servo.id] = uint16_t(msg->data[msg_idx++] * SERVO_MX_BITS_PER_RAD * servo.cw + servo.zero);
        }
        writePositions(right_leg_servos);
    }

    void CM730Node::callback_arms_goal_pose(const std_msgs::Float32MultiArray::ConstPtr& msg)
    {
        if(msg->data.size() != 6)
        {
            std::cout << "CM730.->Error!!: goal position for both arms must be a 6-value array." << std::endl;
            return;
        }
        int msg_idx{0};
        for(auto servo: left_arm_servos)
        {
            goal_position[servo.id] = uint16_t(msg->data[msg_idx++] * SERVO_MX_BITS_PER_RAD * servo.cw + servo.zero);
        }
        for(auto servo: right_arm_servos)
        {
            goal_position[servo.id] = uint16_t(msg->data[msg_idx++] * SERVO_MX_BITS_PER_RAD * servo.cw + servo.zero);
        }
        std::vector<Servo::servo_t> arms;
        arms.insert(arms.end(), left_arm_servos.begin(), left_arm_servos.end());
        arms.insert(arms.end(), right_arm_servos.begin(), right_arm_servos.end());
        writePositions(arms);
    }

    void CM730Node::callback_arm_left_goal_pose(const std_msgs::Float32MultiArray::ConstPtr& msg)
    {
        if(msg->data.size() != 3)
        {
            std::cout << "CM730.->Error!!: goal position for left arm must be a 3-value array." << std::endl;
            return;
        }
        int msg_idx{0};
        for(auto servo: left_arm_servos)
        {
            goal_position[servo.id] = uint16_t(msg->data[msg_idx++] * SERVO_MX_BITS_PER_RAD * servo.cw + servo.zero);
        }
        writePositions(left_arm_servos);
    }

    void CM730Node::callback_arm_right_goal_pose(const std_msgs::Float32MultiArray::ConstPtr& msg)
    {
        if(msg->data.size() != 3)
        {
            std::cout << "CM730.->Error!!: goal position for right leg must be a 3-value array." << std::endl;
            return;
        }
        int msg_idx{0};
        for(auto servo: right_arm_servos)
        {
            goal_position[servo.id] = uint16_t(msg->data[msg_idx++] * SERVO_MX_BITS_PER_RAD * servo.cw + servo.zero);
        }
        writePositions(right_arm_servos);
    }
    
    void CM730Node::callback_head_goal_pose(const std_msgs::Float32MultiArray::ConstPtr& msg)
    {
        if(msg->data.size() != 2)
        {
            std::cout << "CM730.->Error!! goal position for head must be a 2-value array" << std::endl;
            return;
        }
        int msg_idx{0};
        for(auto servo: head_servos)
        {
            goal_position[servo.id] = uint16_t(msg->data[msg_idx++] * SERVO_MX_BITS_PER_RAD * servo.cw + servo.zero);
        }
        writePositions(head_servos);
    }

    bool CM730Node::fillServoParameters(const std::vector<std::string>& servo_names, std::vector<Servo::servo_t>& servo_list)
    {
        for(auto name: servo_names)
        {
            Servo::servo_t servo;
            std::string param_id_str      {name + "/id"};
            std::string param_cw_str      {name + "/cw"};
            std::string param_zero_str    {name + "/zero"};
            std::string param_enabled_str {name + "/enabled"};
            std::string param_is4pin_str  {name + "/is4pin"};
            std::string param_joint_str   {name + "/joint"};
            int zero;
            if (!ros::param::get(param_id_str, servo.id))
            {
                ROS_ERROR("Missing param in config file: %s", param_id_str.c_str());
                return false;
            }
            if (!ros::param::get(param_cw_str, servo.cw))
            {
                ROS_ERROR("Missing param in config file: %s", param_cw_str.c_str());
                return false;
            }
            if (!ros::param::get(param_zero_str, zero))
            {
                ROS_ERROR("Missing param in config file: %s", param_zero_str.c_str());
                return false;
            }
            servo.zero = zero;
            if (!ros::param::get(param_enabled_str, servo.enabled))
            {
                ROS_ERROR("Missing param in config file: %s", param_enabled_str.c_str());
                return false;
            }
            if (!ros::param::get(param_is4pin_str, servo.is4Pin))
            {
                ROS_ERROR("Missing param in config file: %s", param_is4pin_str.c_str());
                return false;
            }
            if (!ros::param::get(param_joint_str, servo.joint_name))
            {
                ROS_ERROR("Missing param in config file: %s", param_joint_str.c_str());
                return false;
            }
            std::cout << "[CM730_UTILS] Servo added. ID: " << servo.id 
                      << "\t CW: " << servo.cw 
                      << "\t ZERO: " << servo.zero 
                      << "\t NAME: " << servo.joint_name
                      << "\t is4Pin: " << servo.is4Pin
                      << std::endl;

            servo_list.push_back(servo);
        }
        return true;
    }

    bool CM730Node::readAndPublishAllPositions()
    {   
        if(!comm.getAllPositions(present_position))
        {
            return false;
        }
        msg_joint_states.header.stamp = ros::Time::now();
        for(auto servo: all_servos)
        {
            msg_joint_states.name[servo.id] = servo.joint_name;
            if(servo.enabled)
            {
                msg_joint_states.position[servo.id] = (int(present_position[servo.id]) - int(servo.zero))
                                      * SERVO_MX_RADS_PER_BIT
                                      * servo.cw;
                msg_joint_current_angles.data[servo.id] = msg_joint_states.position[servo.id];
            }
            else
            {
                msg_joint_states.position[servo.id] = 0.0;
                msg_joint_current_angles.data[servo.id] = msg_joint_states.position[servo.id];
            }
        }
        pub_joint_states.publish(msg_joint_states);
        pub_joint_current_angles.publish(msg_joint_current_angles);

        rate.sleep();
        return true;
    }

    bool CM730Node::writePresentPositions()
    {
        if(!comm.getAllPositions(present_position))
        {
            return false;
        }

        goal_position = present_position;
        
        if(!comm.setPositions(goal_position, all_servos))
        {
            return false;
        }
        return true;
    }

    bool CM730Node::writePositions(const std::vector<Servo::servo_t>& servos)
    {
        if(!comm.setPositions(goal_position, servos))
        {
            return false;
        }
        return true;
    }

    
}
