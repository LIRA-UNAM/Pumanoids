/* 
* position_start.cpp
* Package: position_start
* Node: position_start
* This node walks the robots from the side of the field to their
* starting positions for the initial kick-off.
* Written by Sebastian Garcia
* Developed at LIRA UNAM
* https://lira.unam.mx/
*/

#include <chrono>
#include <iostream>
#include <string>
#include <getopt.h>
#include <yaml-cpp/yaml.h>

#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

#define RAD2DEG(x) ((x)*180/M_PI)
#define DEG2RAD(x) ((x)*M_PI/180.)

// Do NOT change this variables manually
// Used for command options
// To-do: Change to ros2 parameters
bool inverse_rotation_mode = false; // To rotate to the left (clockwise) instead of the right (counterclockwise)
bool standalone_mode = false; // To run the node without the robot state machine
bool debug_mode = false; // To lower the verbose level to DEBUG
std::string target_position; // To store the user-specified target position

class PositionStart : public rclcpp::Node
{
public:
    PositionStart(const std::string& target_position) : Node("position_start"), target_position_(target_position)
    {
        /*
        // DOESN'T WORK ON ROS2 FOXY (Unitree G1)
        // To be adapted to ROS2 older versions
        if (debug_mode)
        {
            // Set logger to debug level if -d option is used
            this->get_logger().set_level(rclcpp::Logger::Level::Debug);
        }
        else
        {
            // Default logger level
            this->get_logger().set_level(rclcpp::Logger::Level::Info);
        }
        */

        loadConfiguration("config/positions_demo.yaml"); // YAML file parsing function

        // Publisher and subscriber for state machine topics
        state_mach_sub_ = this->create_subscription<std_msgs::msg::Bool>("/position_start/enable", 10, std::bind(&PositionStart::sm_enable, this, std::placeholders::_1));
        state_mach_pub_ = this->create_publisher<std_msgs::msg::Bool>("/position_start/finish", 10);

        // Coordinate transformation
        tf_buffer_ =std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ =std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // Publisher for robot movement
        movement_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        // Timer for the node operation
        timer_ = this->create_wall_timer(std::chrono::milliseconds(timer_period),std::bind(&PositionStart::timer_callback, this));

        // Node initialization success feedback
        RCLCPP_INFO(this->get_logger(), "position_start node started");
        RCLCPP_DEBUG(this->get_logger(), "Logger level set to DEBUG");

        // Set standalone_mode if specified
        if (!standalone_mode)
            {
                // Using the state machine
                RCLCPP_INFO(this->get_logger(), "Waiting for state machine...");
                current_state_ = State::WAITING_FOR_STATE_MACHINE;
            }
            else
            {
                // Not using the state machine
                RCLCPP_INFO(this->get_logger(), "Using node in standalone mode (without state machine)");
                current_state_ = State::INITIAL_POSE;
            }
        }

private:
    // -- ROS2 OBJECTS DECLARATIONS --
    // Movement publisher
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr movement_publisher_;
    // State machine subscriber and publisher
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr state_mach_sub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr state_mach_pub_;

    // TF2 objects for coordinate transformation
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

    // Timer for the callback to the primary function
    rclcpp::TimerBase::SharedPtr timer_;

    // -- VARIABLES --
    // Timer callback logic
    int timer_period = 500; // Timer callback period
    float current_timer_pos_x = 0; // Timer for X movement
    float current_timer_pos_y = 0; // Timer for Y movement

    // Robot position and orientation
    float initial_theta_ = 0.0f; // Initial orientation of the robot
    bool has_initial_theta_ = false; // Flag for the first angle reading
    bool has_started_moving_ = false; // Flag for the first walking movement
    float target_angle = 0.0f; // Target angle of rotation
    float angular_error = 0.0f; // Error in angular position
    float current_angle = 0.0f; // Current rotation angle of the robot
    geometry_msgs::msg::TransformStamped t; // TransformStamped object for TF2 transformations

    // Target position parameters
    std::string target_position_; // Target position specified by the user
    double target_x_ = 0.0; // Target in the X axis
    double target_y_ = 0.0; // Target in the Y axis
    bool YAML_success = true; // Flag for YAML loading success


    // -- STATES OF THE NODE --
    enum class State {
        WAITING_FOR_STATE_MACHINE, // Waiting for true from the state machine
        INITIAL_POSE, // Initial pose, waiting for the first angle reading
        MOVING_X, // Moving forward along the X axis
        ROTATING, // Rotating to the target angle
        MOVING_Y, // Moving forward along the Y axis
        FINISHED // Finished, published true to the state machine
    };

    // Setting initial state.
    State current_state_ = State::WAITING_FOR_STATE_MACHINE;

    // Callback function for the state machine enable topic.
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

    // Function to load and parse the YAML configuration file.
    void loadConfiguration(const std::string& config_file)
    {
        // The node tries to load the YAML file specified by the config_file parameter.
        try
        {
            // -- LOADING THE FILE --
            // The path is constructed using ament_index_cpp to find the package share directory.
            std::string package_share_dir = ament_index_cpp::get_package_share_directory("position_start");
            std::string full_path = package_share_dir + "/" + config_file;
            RCLCPP_DEBUG(this->get_logger(), full_path.c_str());

            // Once we have the full path, we load the YAML file.
            YAML::Node config = YAML::LoadFile(full_path);

            // -- PARSING THE FILE --
            // We look for the parameters under the specified structure.
            YAML::Node position_params = config["position_start"]["ros__parameters"];

            // The node tries to find the specified target position in the YAML file.
            try
            {
                // If the target position is not specified in the command line,
                // the target_position_ variable will be "no_input"
                // as set in the parse_arguments function.
                if (target_position_ == "no_input")
                {
                    // This is considered an error, since the user
                    // must specify a target position for the node to work.
                    YAML_success = false;
                    RCLCPP_WARN(this->get_logger(), "TARGET NOT SPECIFIED");
                    RCLCPP_WARN(this->get_logger(), "Specify a target ('center', 'left' or 'right')");
                }
                else
                {
                    // If the target position is specified, we look for it in the YAML file.
                    // The target components represent the time to move along the X and Y axes,
                    // which are used in the timer callback to calculate the movement duration.
                    target_x_ = position_params[target_position_]["x"].as<float>();
                    target_y_ = position_params[target_position_]["y"].as<float>();
                }
            }
            // The target position wasn't found in the YAML.
            catch (const YAML::Exception& ex)
            {
                YAML_success = false;
                RCLCPP_ERROR(this->get_logger(), "Unknown target position: %s", target_position_.c_str());
            }
            
            // If we got this far without exceptions, the YAML loading and parsing was successful.
            if (YAML_success)
            {
                // Success
                RCLCPP_INFO(this->get_logger(), "Target %s: x=%0.2f, y=%0.2f", target_position_.c_str(), target_x_, target_y_);
            }
        } 
        // If there was an error loading the YAML file (e.g., file not found,
        // syntax error), we catch the exception.
        catch (const YAML::Exception& e)
        {
            YAML_success = false;
            RCLCPP_ERROR(this->get_logger(), "Error loading YAML: %s", e.what());
        }
    }

    // Primary function of the node.
    // The timer period can be adjusted with the timer_period variable,
    // but it is recommended to keep it around 500 ms.
    void timer_callback()
    {
        // No need to run the whole callback when it's finish or when there was an error.
        if (!YAML_success || current_state_ == State::FINISHED)
        {
            RCLCPP_DEBUG(this->get_logger(), "Timer returning");
            return;
        }

        // Get the robot rotation angle (yaw) from the odometry transform.
        // This is used for the rotation step and to ensure the robot starts moving in the correct direction.
        try
        {
            // It looks for the transform between "odom" and
            // "base_link" frames, published by the odom_to_tf node.
            t = tf_buffer_->lookupTransform("odom", "base_link",tf2::TimePointZero);
            // The angle is calculated from the quaternion rotation in the transform.
            current_angle = normalizeAngle(atan2(t.transform.rotation.z, t.transform.rotation.w)*2);
            // Once we have the initial angle, we set the flag to true so we can start moving in the next steps.
            has_initial_theta_ = true;
        }
        
        // The transform takes some seconds to be available after the node starts,
        // making it necessary to catch the exception that is thrown when the transform is not found.
        catch (const tf2::TransformException & ex)
        {
            RCLCPP_ERROR(this->get_logger(), "Could not transform odom to base_link: %s", ex.what());
            return;
        }

        // Debug output for the current angle of the robot.
        RCLCPP_DEBUG(this->get_logger(),"ANGLE: %0.3f", current_angle);

        // Dont move unitl we have the initial angle.
        if (current_state_ == State::INITIAL_POSE) {
            if (has_initial_theta_)
            {
                // If we have the initial angle, we can start moving along the X axis.
                current_state_ = State::MOVING_X;
                has_started_moving_ = true;
                RCLCPP_INFO(this->get_logger(), "Starting movement along X axis");
            }
            else
            {
                RCLCPP_DEBUG(this->get_logger(), "Waiting for initial theta from odometer...");
                return;
            }
        }

        auto twist_msg = geometry_msgs::msg::Twist(); // Twist message for robot movement commands.

        // Switch for the different moving states of the node.
        // The node goes from moving along the X axis, to rotating,
        // to moving along the Y axis, and finally to finished.
        switch (current_state_)
        {
            case State::MOVING_X:
                // For each timer callback, we compare how much time
                // the robot has been moving forward with the target time.
                if (current_timer_pos_x < target_x_)
                {
                    // Moving forward along the X axis.
                    twist_msg.linear.x = 0.4; // Set the forward walk speed.
                    movement_publisher_->publish(twist_msg); // Publish the movement command.
                    // The walk timer progress is calculated according to the timer period
                    current_timer_pos_x += timer_period / 1000.0;
                    
                    RCLCPP_DEBUG(this->get_logger(), "Moving X: %f/%f s", current_timer_pos_x, target_x_);
                }
                else
                {
                    // The robot is stopped before starting the rotation.
                    twist_msg.linear.x = 0;
                    twist_msg.angular.z = 0;
                    movement_publisher_->publish(twist_msg);

                    // Once it finishes moving along the X axis, the state changes to ROTATING.
                    current_state_ = State::ROTATING;
                    RCLCPP_INFO(this->get_logger(), "X movement complete, starting rotation");

                    // Check if the rotation was specified to be clockwise (inverse) or counterclockwise (default).
                    // We calculate the target angle accordingly, adding or substracting 90 degrees to the current angle.
                    if (inverse_rotation_mode)
                    {
                        target_angle = normalizeAngle(current_angle - DEG2RAD(90.0));
                    }
                    else
                    {
                        target_angle = normalizeAngle(current_angle + DEG2RAD(90.0));
                    }
                    RCLCPP_DEBUG(this->get_logger(), "Target angle: %f", target_angle);
                }
                break;
            case State::ROTATING:
                angular_error = shortestAngularDistance(current_angle, target_angle);
                RCLCPP_DEBUG(this->get_logger(), "Angular error: %f rad", angular_error);
                if (fabs(angular_error) > 0.1) // Rotation tolerance of 0.1 rad
                {
                    float angular_vel = 0.6 * angular_error / fabs(angular_error);
                    // Rotate
                    twist_msg.angular.z = angular_vel;
                    movement_publisher_->publish(twist_msg);
                    RCLCPP_DEBUG(this->get_logger(), "Rotating, error: %f rad, angular vel: %f", angular_error, angular_vel);
                }
                else
                {
                    // Stop rotation
                    twist_msg.angular.z = 0;
                    movement_publisher_->publish(twist_msg);
                    current_state_ = State::MOVING_Y;
                    RCLCPP_INFO(this->get_logger(), "Rotation complete, starting Y movement");
                }
                break;
            case State::MOVING_Y:
                if (current_timer_pos_y < target_y_)
                {
                    // Move forward along Y axis
                    twist_msg.linear.x = 0.4; // Walk speed
                    movement_publisher_->publish(twist_msg);
                    current_timer_pos_y += timer_period / 1000.0; // Convert ms to seconds

                    RCLCPP_DEBUG(this->get_logger(), "Moving Y: %f/%f s", current_timer_pos_y, target_y_);
                }
                else
                {
                    // Stop the robot
                    twist_msg.linear.x = 0;
                    movement_publisher_->publish(twist_msg);
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
                    movement_publisher_->publish(twist_msg);
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


void parse_arguments(int argc, char* argv[]) {
    int opt;
    while ((opt = getopt(argc, argv, "isdh")) != -1) {
        switch (opt) {
            case 'i':
                inverse_rotation_mode = true;
                break;
            case 's':
                standalone_mode = true;
                break;
            case 'd':
                debug_mode = true;
                break;
            case 'h':
                std::cout << "Usage: position_start [-i] [-s] [-d] <target_position>\n"
                          << "Options:\n"
                          << "  -i    Inverse rotation mode (rotate clockwise)\n"
                          << "  -s    Standalone mode (without state machine)\n"
                          << "  -d    Enable debug mode (verbose output)\n"
                          << "  -h    Show this help message\n";
                exit(0);
                break;
            case '?':
                std::cerr << "Unknown option: -" << char(optopt) << std::endl;
                break;
            default:
                break;
        }
    }

    // The target position should be after options
    if (optind < argc)
    {
        target_position = argv[optind];
    } else
    {
        target_position = "no_input";
    }
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    std::vector<std::string> args = rclcpp::remove_ros_arguments(argc, argv);
    int custom_argc = static_cast<int>(args.size());
    std::vector<char*> custom_argv;
    for (auto& arg : args) {
        custom_argv.push_back(&arg[0]);
    }
    custom_argv.push_back(nullptr);
    parse_arguments(custom_argc, custom_argv.data());

    auto node = std::make_shared<PositionStart>(target_position);
    rclcpp::spin(node);
    rclcpp::shutdown();    
    return 0;
}
