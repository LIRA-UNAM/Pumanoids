#include "rclcomm.h"
#include <string>
#include <chrono>

RclComm::RclComm(): Node("justina_gui_node")
{
    std::cout << "Initializing RclComm node ..." << std::endl;
    this->_node = rclcpp::Node::make_shared("ros2_qt_demo"); 
    this->_pub_cmd_vel =  this->_node->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    // auto timer_callback =
    // 	[this]() -> void {
    // 	    std::cout << "Testing callback" << std::endl;
    // 	};
    //_timer = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&RclComm::timer_callback, this));
    //this->_subp = this->_node->create_subscription<std_msgs::msg::String>("chat_qt", 10, std::bind(&RclComm::recv_callback, this, std::placeholders::_1));
    this->_sub_test = this->_node->create_subscription<std_msgs::msg::Float32>("test", 10, std::bind(&RclComm::callback_current_arm_pose, this, std::placeholders::_1));
    this->_clt_plan_path = this->_node->create_client<nav_msgs::srv::GetPlan>("/path_planning/plan_path");
}

void RclComm::timer_callback()
{
    std::cout << "Testing callback" << std::endl;
}

void RclComm::callback_current_arm_pose(const std_msgs::msg::Float32 &msg)
{
    std::cout << "Received float " << msg.data << std::endl;
}

void RclComm::publish_cmd_vel(double linear_x, double linear_y, double angular)
{
    geometry_msgs::msg::Twist msg;
    msg.linear.x = linear_x;
    msg.linear.y = linear_y;
    msg.angular.z =  angular;
    this->_pub_cmd_vel->publish(msg);
}

void RclComm::publish_cmd_vel(double linear_x, double angular)
{
    this->publish_cmd_vel(linear_x, 0, angular);
}

void RclComm::start_publishing_cmd_vel(double linear_x, double linear_y, double angular)
{
    _cmd_vel.linear.x = linear_x;
    _cmd_vel.linear.y = linear_y;
    _cmd_vel.angular.z = angular;
    _publishing_cmd_vel = true;
}

void RclComm::start_publishing_cmd_vel(double linear_x, double angular)
{
    this->start_publishing_cmd_vel(linear_x, 0, angular);
}

void RclComm::stop_publishing_cmd_vel()
{
    _cmd_vel.linear.x = 0;
    _cmd_vel.linear.y = 0;
    _cmd_vel.angular.z = 0;
    _publishing_cmd_vel = false;
}

bool RclComm::call_plan_path(double start_x, double start_y, double goal_x, double goal_y, nav_msgs::msg::Path& path)
{
    auto request = std::make_shared<nav_msgs::srv::GetPlan::Request>();
    request->start.pose.position.x = start_x;
    request->start.pose.position.y = start_y;
    request->goal.pose.position.x = goal_x;
    request->goal.pose.position.y = goal_y;
    std::cout << "JustinaGUI->Waiting for plan path service to be available..." << std::endl;
    while(!this->_clt_plan_path->wait_for_service(std::chrono::milliseconds(500)))
    {
	if (!rclcpp::ok()) {
	    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the plan path service. Exiting.");
	    return 0;
	}
	RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service plan path not available, waiting again...");
    }
    std::cout << "JustinaGUI->Plan path service available. Trying to plan path..." << std::endl;
    auto result = this->_clt_plan_path->async_send_request(request);
    // Wait for the result.
    bool success = rclcpp::spin_until_future_complete(this->_node, result) == rclcpp::FutureReturnCode::SUCCESS;
    if (success)
    {
	path = result.get()->plan;
	std::cout << "JustinaGUI.->Path planned successfully. Path with " << path.poses.size() << " points." << std::endl;
	return true;
    } else {
	std::cout << "JustinaGUI.->Cannot plan path :'(" << std::endl;
	return false;
    }
    return true;
}

// void RclComm::recv_callback(const std_msgs::msg::String &msg)
// {
//     emit emitTopicData("pub send a msgs:" + QString::fromStdString(msg.data));
// }

// // spin
// void RclComm::sping()
// {
//     rclcpp::spin_some(_node);
// }
