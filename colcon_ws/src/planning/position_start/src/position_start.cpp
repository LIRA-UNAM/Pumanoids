#include <chrono>
#include <iostream>
#include <string>


#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include <yaml-cpp/yaml.h>

#define RAD2DEG(x) ((x)*180/M_PI)
#define DEG2RAD(x) ((x)*M_PI/180.)


class PositionStart : public rclcpp::Node
{
public:
    PositionStart(const std::string& target_position) : Node("position_start"), target_position_(target_position)
    {
        loadConfiguration("config/positions_demo.yaml");

        state_mach_sub_ = this->create_subscription<std_msgs::msg::Bool>("/position_start/enable", 10, std::bind(&PositionStart::sm_enable, this, std::placeholders::_1));
        state_mach_pub_ = this->create_publisher<std_msgs::msg::Bool>("/position_start/finish", 10);

        tf_buffer_ =std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ =std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        timer_ = this->create_wall_timer(std::chrono::milliseconds(timer_period),std::bind(&PositionStart::timer_callback, this));

        RCLCPP_INFO(this->get_logger(), "position_start node started");
    }

private:
    // ROS2 objects declarations
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr state_mach_sub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr state_mach_pub_;

    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

    rclcpp::TimerBase::SharedPtr timer_;

    // Variables
    int timer_period = 500;
    float current_timer_pos_x = 0;
    float current_timer_pos_y = 0;

    float initial_theta_ = 0.0f;
    bool has_initial_theta_ = false;
    bool has_started_moving_ = false;
    float target_angle = 0.0f;
    float angular_error = 0.0f;
    float current_angle = 0.0f;

    std::string target_position_;
    double target_x_ = 0.0;
    double target_y_ = 0.0;
    size_t count_;

    bool YAML_success = true;

    // States of the position_start
    enum class State {
        WAITING_FOR_STATE_MACHINE,
        INITIAL_POSE,
        MOVING_X,
        ROTATING,
        MOVING_Y,
        FINISHED
    };

    // When using this node with the robot's state machine, uncomment the following line
    State current_state_ = State::WAITING_FOR_STATE_MACHINE; 

    // When running this node standalone (without the state machine), uncomment the following line
    //State current_state_ = State::INITIAL_POSE;


    void sm_enable(const std_msgs::msg::Bool::SharedPtr msg)
    {
        if (msg->data && current_state_ == State::WAITING_FOR_STATE_MACHINE)
        {
            // The state machine enable is true
            current_state_ = State::INITIAL_POSE;
        }
        if (!msg->data){
            // The state machine enable is false
            current_state_ = State::WAITING_FOR_STATE_MACHINE;
        }
    }

    void loadConfiguration(const std::string& config_file)
    {
        try
        {
            // Loading the file
            std::string package_share_dir = ament_index_cpp::get_package_share_directory("position_start");
            std::string full_path = package_share_dir + "/" + config_file;
            RCLCPP_INFO(this->get_logger(), full_path.c_str());
            YAML::Node config = YAML::LoadFile(full_path);

            // Parsing
            YAML::Node position_params = config["position_start"]["ros__parameters"];

            try
            {
                if (target_position_ == "no_input")
                {
                    // User didnt specified a target
                    YAML_success = false;
                    RCLCPP_WARN(this->get_logger(), "TARGET NOT SPECIFIED");
                    RCLCPP_WARN(this->get_logger(), "Specify a target ('center', 'left' or 'right')");
                }
                else
                {
                    // Store x and y
                    target_x_ = position_params[target_position_]["x"].as<float>();
                    target_y_ = position_params[target_position_]["y"].as<float>();
                }
            }
            catch (const YAML::Exception& ex)
            {
                // Target wasnt found in the yaml
                YAML_success = false;
                RCLCPP_ERROR(this->get_logger(), "Unknown target position: %s", target_position_.c_str());
            }
            

            if (YAML_success)
            {
                // Successsss
                RCLCPP_INFO(this->get_logger(), "Target %s: x=%f, y=%f", target_position_.c_str(), target_x_, target_y_);
            }
        } 
        catch (const YAML::Exception& e)
        {
            // Yaml file not found
            YAML_success = false;
            RCLCPP_ERROR(this->get_logger(), "Error loading YAML: %s", e.what());
        }
    }

    void timer_callback()
    {
        if (!YAML_success || current_state_ == State::FINISHED)
        {
            // No need to run the whole callback when finish or when error
            RCLCPP_DEBUG(this->get_logger(), "Timer returning");
            return;
        }

        geometry_msgs::msg::TransformStamped t;

        // Get the robot rotation angle (yaw)
        try
        {
            t = tf_buffer_->lookupTransform("odom", "base_link",tf2::TimePointZero);
            current_angle = normalizeAngle(atan2(t.transform.rotation.z, t.transform.rotation.w)*2);
            // Has initial angle. Can start moving
            has_initial_theta_ = true;
        }
        catch (const tf2::TransformException & ex)
        {
            // It won't move if there's no angle
            RCLCPP_ERROR(this->get_logger(), "Could not transform odom to base_link: %s", ex.what());
            return;
        }

        RCLCPP_INFO(this->get_logger(),"ANGLE: %f", current_angle);

        // Dont move unitl we have the initial angle
        if (current_state_ == State::INITIAL_POSE) {
            if (has_initial_theta_)
            {
                current_state_ = State::MOVING_X;
                has_started_moving_ = true;
                RCLCPP_INFO(this->get_logger(), "Starting movement along X axis");
            }
            else
            {
                RCLCPP_INFO(this->get_logger(), "Waiting for initial theta from odometer...");
                return;
            }
        }


        auto twist_msg = geometry_msgs::msg::Twist();
        
        switch (current_state_)
        {
            case State::MOVING_X:
                if (current_timer_pos_x < target_x_)
                {
                    // Moving forward along the X axis
                    twist_msg.linear.x = 0.4; // Walk speed
                    publisher_->publish(twist_msg);
                    current_timer_pos_x += timer_period / 1000.0; // Convert ms to seconds
                    
                    RCLCPP_INFO(this->get_logger(), "Moving X: %f/%f s", current_timer_pos_x, target_x_);
                }
                else
                {
                    // Stop the robot
                    twist_msg.linear.x = 0;
                    publisher_->publish(twist_msg);
                    // Finished moving in X, start rotation
                    current_state_ = State::ROTATING;
                    RCLCPP_INFO(this->get_logger(), "X movement complete, starting rotation");
                    target_angle = normalizeAngle(current_angle + DEG2RAD(90.0));
                    RCLCPP_INFO(this->get_logger(), "Target angle: %f", target_angle);
                }
                break;
            case State::ROTATING:
                angular_error = shortestAngularDistance(current_angle, target_angle);
                RCLCPP_DEBUG(this->get_logger(), "Angular error: %f rad", angular_error);
                if (fabs(angular_error) > 0.1) // Rotation tolerance of 0.1 rad
                {
                    float angular_vel = 0.6 * angular_error / fabs(angular_error);
                    twist_msg.angular.z = angular_vel;
                    publisher_->publish(twist_msg);
                    RCLCPP_DEBUG(this->get_logger(), "Rotating, error: %f rad, angular vel: %f", angular_error, angular_vel);
                }
                else
                {
                    // Stop rotation
                    twist_msg.angular.z = 0;
                    publisher_->publish(twist_msg);
                    current_state_ = State::MOVING_Y;
                    RCLCPP_INFO(this->get_logger(), "Rotation complete, starting Y movement");
                }
                break;
            case State::MOVING_Y:
                if (current_timer_pos_y < target_y_)
                {
                    // Move forward along Y axis
                    twist_msg.linear.x = 0.4; // Walk speed
                    publisher_->publish(twist_msg);
                    current_timer_pos_y += timer_period / 1000.0; // Convert ms to seconds

                    RCLCPP_INFO(this->get_logger(), "Moving Y: %f/%f s", current_timer_pos_y, target_y_);
                }
                else
                {
                    // Stop the robot
                    twist_msg.linear.x = 0;
                    publisher_->publish(twist_msg);
                    // Finished moving in Y, start rotation
                    current_state_ = State::FINISHED;
                    RCLCPP_INFO(this->get_logger(), "Y movement complete.");
                    auto bool_msg = std_msgs::msg::Bool();
                    bool_msg.data = true;
                    state_mach_pub_->publish(bool_msg); // Publishes a true in the "finish" topic
                }
                break;
            default:
                    // Stop the robot
                    twist_msg.linear.x = 0;
                    twist_msg.angular.z = 0;
                    publisher_->publish(twist_msg);
                break;
        }

    }

    float normalizeAngle(float angle) 
    {
        angle = fmod(angle + M_PI, 2.0 * M_PI);
        if (angle < 0)
            angle += 2.0 * M_PI;
        return angle - M_PI;
    }

    float shortestAngularDistance(float from, float to)
    {
        float diff = normalizeAngle(to - from);
        if (diff > M_PI)
            diff -= 2.0 * M_PI;
        else if (diff < -M_PI)
            diff += 2.0 * M_PI;
        return diff;
    }

};

int main(int argc, char * argv[])
{
    std::string target_position = "no_input"; //Setting a safe default to avoid unfunny errors
    if (argc > 1) 
    {
        target_position = argv[1];  // First argument after node name
    }

    rclcpp::init(argc, argv);
    auto node = std::make_shared<PositionStart>(target_position);
    rclcpp::spin(node);
    rclcpp::shutdown();    
    return 0;
}
