#include <iostream>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "booster_interface/msg/odometer.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include <yaml-cpp/yaml.h>
// Commented out due to compilation errors

class PositionStart : public rclcpp::Node
{
public:
    PositionStart(const std::string& target_position) : Node("position_start"), target_position_(target_position)
    {
        loadConfiguration("config/positions.yaml");

        subscriber_ = this->create_subscription<booster_interface::msg::Odometer>("/odometer_state", 10, std::bind(&PositionStart::topic_callback, this, std::placeholders::_1));
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        RCLCPP_INFO(this->get_logger(), "position_start node started");
    }

private:
    rclcpp::Subscription<booster_interface::msg::Odometer>::SharedPtr subscriber_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    
    std::string target_position_;
    double target_x_ = 0.0;
    double target_y_ = 0.0;

    bool YAML_success = true;

    void loadConfiguration(const std::string& config_file) {
        try {
            std::string package_share_dir = ament_index_cpp::get_package_share_directory("position_start");
            std::string full_path = package_share_dir + "/" + config_file;
            
            RCLCPP_INFO(this->get_logger(), full_path.c_str());

            YAML::Node config = YAML::LoadFile(full_path);
            YAML::Node position_params = config["position_start"]["ros__parameters"];

            if (target_position_ == "center") {
                target_x_ = position_params["center"]["x"].as<float>();
                target_y_ = position_params["center"]["y"].as<float>();
            } else if (target_position_ == "left") {
                target_x_ = position_params["left"]["x"].as<float>();
                target_y_ = position_params["left"]["y"].as<float>();
            } else if (target_position_ == "right") {
                target_x_ = position_params["right"]["x"].as<float>();
                target_y_ = position_params["right"]["y"].as<float>();
            } else {
                YAML_success = false;
                RCLCPP_ERROR(this->get_logger(), "Unknown target position: %s", target_position_.c_str());
            }

            RCLCPP_INFO(this->get_logger(), "Target %s: x=%f, y=%f", target_position_.c_str(), target_x_, target_y_);
        } 
        catch (const YAML::Exception& e) {
            YAML_success = false;
            RCLCPP_ERROR(this->get_logger(), "Error loading YAML: %s", e.what());
        }
    }


    void topic_callback(const booster_interface::msg::Odometer::SharedPtr msg)
    {
        if (YAML_success)
        {
            RCLCPP_INFO(this->get_logger(), "x: %f\ty: %f\ttheta: %f", msg->x, msg->y, msg->theta);
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
    else
    {
        std::cout << "Specify a target ('center', 'left' or 'right')" << std::endl;
    }


    rclcpp::init(argc, argv);
    auto node = std::make_shared<PositionStart>(target_position);
    rclcpp::spin(node);
    rclcpp::shutdown();    
    return 0;
}
