#pragma once

#include <atomic>
#include <iostream>
#include <string>
#include <thread>
#include <vector>
#include <rclcpp/rclcpp.hpp>

#include "RoboCupGameControlData.h"

#include "game_controller_interface/msg/game_control_data.hpp"

using namespace std;

class GameControllerNode : public rclcpp::Node
{
public:
    GameControllerNode(string name);
    ~GameControllerNode();

    // Initialize the UDP socket.
    void init();

    // Receive UDP broadcasts, process them, and publish them to a ROS 2 topic.
    void spin();

private:
    // Check whether a packet came from an allowlisted host.
    bool check_ip_white_list(const string &ip) const;

    // Process a packet by copying each field explicitly.
    void handle_packet(
        const RoboCupGameControlData &data,
        game_controller_interface::msg::GameControlData &msg) const;

    // Listening port loaded from configuration.
    int _port;
    // Whether the IP allowlist is enabled.
    bool _enable_ip_white_list;
    // Allowed IP addresses.
    vector<string> _ip_white_list;

    // UDP Socket
    int _socket;
    std::atomic_bool _running{false};
    // thread
    
    thread _thread;

    // Ros2 publisher
    rclcpp::Publisher<game_controller_interface::msg::GameControlData>::SharedPtr _publisher;
};
