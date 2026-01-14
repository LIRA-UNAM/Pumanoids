#include <chrono>
#include <iostream>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "booster_interface/msg/odometer.hpp"
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

        state_mach_sub_ = this->create_subscription<std_msgs::msg::Bool>("enable", 10, std::bind(&PositionStart::sm_enable, this, std::placeholders::_1));
        state_mach_pub_ = this->create_publisher<std_msgs::msg::Bool>("finish", 10);

        subscriber_ = this->create_subscription<booster_interface::msg::Odometer>("/odometer_state", 10, std::bind(&PositionStart::odometer_callback, this, std::placeholders::_1));
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        timer_ = this->create_wall_timer(std::chrono::milliseconds(timer_period),std::bind(&PositionStart::timer_callback, this));

        RCLCPP_INFO(this->get_logger(), "position_start node started");
    }

private:
    // ROS2 objects declarations
    rclcpp::Subscription<booster_interface::msg::Odometer>::SharedPtr subscriber_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr state_mach_sub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr state_mach_pub_;

    rclcpp::TimerBase::SharedPtr timer_;

    

    // Variables
    int timer_period = 500;
    float current_timer_pos_x = 0;
    float current_timer_pos_y = 0;

    float initial_theta_ = 0.0f;
    bool has_initial_theta_ = false;
    bool has_started_moving_ = false;
    float target_angle = 0.0f;
    float target_range[2] = {0, 0};
    float current_angle = 0.0f;

    std::string target_position_;
    double target_x_ = 0.0;
    double target_y_ = 0.0;
    size_t count_;

    // States of the position_start
    enum class State {
        WAITING_FOR_STATE_MACHINE,
        INITIAL_POSE,
        MOVING_X,
        ROTATING,
        MOVING_Y,
        FINISHED
    };


    State current_state_ = State::WAITING_FOR_STATE_MACHINE; //UNCOMMENT WHEN USING WITH THE STATE MACHINE
    //State current_state_ = State::INITIAL_POSE; //LEAVE THIS COMMENTED

    bool YAML_success = true;

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
            std::string package_share_dir = ament_index_cpp::get_package_share_directory("position_start");
            std::string full_path = package_share_dir + "/" + config_file;
            
            RCLCPP_INFO(this->get_logger(), full_path.c_str());

            YAML::Node config = YAML::LoadFile(full_path);
            YAML::Node position_params = config["position_start"]["ros__parameters"];

            if (target_position_ == "center")
            {
                target_x_ = position_params["center"]["x"].as<float>();
                target_y_ = position_params["center"]["y"].as<float>();
            }
            else if (target_position_ == "left")
            {
                target_x_ = position_params["left"]["x"].as<float>();
                target_y_ = position_params["left"]["y"].as<float>();
            }
            else if (target_position_ == "right")
            {
                target_x_ = position_params["right"]["x"].as<float>();
                target_y_ = position_params["right"]["y"].as<float>();
            }
            else
            {
                YAML_success = false;
                if (target_position_ == "no_input")
                {
                    RCLCPP_WARN(this->get_logger(), "TARGET NOT SPECIFIED");
                    RCLCPP_WARN(this->get_logger(), "Specify a target ('center', 'left' or 'right')");
                }
                RCLCPP_ERROR(this->get_logger(), "Unknown target position: %s", target_position_.c_str());
            }
            

            if (YAML_success)
            {
                RCLCPP_INFO(this->get_logger(), "Target %s: x=%f, y=%f", target_position_.c_str(), target_x_, target_y_);
            }
        } 
        catch (const YAML::Exception& e)
        {
            YAML_success = false;
            RCLCPP_ERROR(this->get_logger(), "Error loading YAML: %s", e.what());
        }
    }

    void timer_callback()
    {
        if (!YAML_success || current_state_ == State::FINISHED)
        {
            RCLCPP_INFO(this->get_logger(), "Timer returning");
            return;
        }
        
        
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
                    twist_msg.linear.x = 0.4; // Your desired forward speed
                    current_timer_pos_x += timer_period / 1000.0; // Convert ms to seconds
                    publisher_->publish(twist_msg);
                    
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
                }
                break;
            case State::ROTATING:
                if (current_angle < target_range[0] && current_angle < target_range[1])
                {
                    // Rotate to the left
                    twist_msg.angular.z = 0.6;
                    publisher_->publish(twist_msg);
                }
                else if (current_angle > target_range[0] && current_angle > target_range[1])
                {
                    // Rotate to the right
                    twist_msg.angular.z = -0.6;
                    publisher_->publish(twist_msg);
                }
                else if (current_angle < target_range[0] && current_angle > target_range[1])
                {
                    // Stop rotation
                    twist_msg.angular.z = 0;
                    publisher_->publish(twist_msg);
                    current_state_ = State::MOVING_Y;
                }
                break;
            case State::MOVING_Y:
                if (current_timer_pos_y < target_y_)
                {
                    // Move forward along -Y axis

                    twist_msg.linear.x = 0.4; // Your desired forward speed
                    current_timer_pos_y += timer_period / 1000.0; // Convert ms to seconds
                    publisher_->publish(twist_msg);

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

    void odometer_callback(const booster_interface::msg::Odometer::SharedPtr msg)
    {
        if (YAML_success)
        {
            RCLCPP_DEBUG(this->get_logger(), "x: %f\ty: %f\ttheta: %f", msg->x, msg->y, msg->theta);

            if (!has_initial_theta_ && !has_started_moving_)
            {
                initial_theta_ = msg->theta;
                target_angle = initial_theta_ + DEG2RAD(90.0);
                target_range[0] = target_angle + 0.2;
                target_range[1] = target_angle - 0.2;
                has_initial_theta_ = true;
                RCLCPP_INFO(this->get_logger(), "Initial theta captured: %f radians (%f degrees)", initial_theta_, RAD2DEG(initial_theta_));
            }

            current_angle = msg->theta;

        }
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
