/* 
* ---------------------------------------------------
* joints_service.cpp
* Package: joint_states_package
* Node: joints_service
* ---------------------------------------------------
* This nodes acts as an intermediate service between the /joint_states topic and this project nodes.
* Subscribing to it directly can cause CPU overhead, as messages get published at 500 hz. Instead,
* this server node is the only one subscribed to the /joint_states topic and shares the data to
* whatever client node wants it at a slower rate.
*
* Currently, the nodes in this project only use the head pan and tilt, so this service only return these
* values. If your node needs another joint reading, feel free to change this file and the
* service interface (HeadJoints.srv).
*
* Also, if you are going to modify it for the Pumanoids 
* repo, notify me o_o
*               |
*               |
*               V
* ---------------------------------------------------
* Written by Sebastian Garcia
* Developed at LIRA UNAM
* https://lira.unam.mx/
*/

#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/joint_state.hpp>

#include "joint_states_package/srv/head_joints.hpp"

using joint_states_package::srv::HeadJoints;

class JointsService : public rclcpp::Node
{
public:
    JointsService() : Node("joints_service"), pan_pos(0.0), tilt_pos(0.0), data_received(false)
    {
        
        // Subscriber to /joint_states
        joint_sub = this->create_subscription<sensor_msgs::msg::JointState>(
                "/joint_states", 
                rclcpp::SensorDataQoS(), 
                [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
                    this->jointStateCallback(msg);
                });

        // Service
        joint_service = this->create_service<HeadJoints>(
                "get_head_joints",
                [this](const std::shared_ptr<HeadJoints::Request> req, std::shared_ptr<HeadJoints::Response> res) {
                    this->handleService(req, res);
                });
        
        // Log the node start
        RCLCPP_INFO(this->get_logger(), "Joint service node started");
    }

private:
    // Decalre the subscriber variable
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub;
    // Declare the service variable
    rclcpp::Service<HeadJoints>::SharedPtr joint_service;

    // -- VARIABLES --
    double pan_pos;
    double tilt_pos;
    bool data_received;

    // -- CALLBACKS --
    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        pan_pos = msg->position[0];
        tilt_pos = msg->position[1];
        data_received = true;
        return;
    }
    void handleService(const std::shared_ptr<HeadJoints::Request> request, std::shared_ptr<HeadJoints::Response> response)
    {
        response->pan  = pan_pos;
        response->tilt = tilt_pos;
        response->success = data_received;
    }

};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<JointsService>();
    rclcpp::spin(node);
    rclcpp::shutdown();    
    return 0;
}
