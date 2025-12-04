#include <iostream>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "booster_interface/msg/odometer.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <yaml-cpp/yaml.h>


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

    

    void loadConfiguration(const std::string& config_file) {
        YAML::Node config = YAML::LoadFile(config_file);
    }


    void topic_callback(const booster_interface::msg::Odometer::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "x: %f\ty: %f\ttheta: %f", msg->x, msg->y, msg->theta);
    }
};

int main(int argc, char * argv[])
{
    std::string target_position = "no_input"; //Setting a safe default to avoid unfunny errors
    if (argc > 1) {
        target_position = argv[1];  // First argument after node name
    }


    rclcpp::init(argc, argv);
    auto node = std::make_shared<PositionStart>(target_position);
    rclcpp::spin(node);
    rclcpp::shutdown();    
    return 0;
}