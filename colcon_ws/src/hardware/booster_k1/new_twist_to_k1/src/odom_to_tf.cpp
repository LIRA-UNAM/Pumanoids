#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <booster_interface/msg/odometer.hpp>

#include <cmath>
#include <string>

class OdomToTFNode : public rclcpp::Node
{
public:
    OdomToTFNode()
    : Node("odom_to_tf_node")
    {
        broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        subscription_ = this->create_subscription<booster_interface::msg::Odometer>(
            "/odometer_state", 10,
            std::bind(&OdomToTFNode::callback_odometer, this, std::placeholders::_1));

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&OdomToTFNode::timer_callback, this));

        RCLCPP_INFO(this->get_logger(), "Booster Odom to TF Node has been started.");
    }

private:
    void callback_odometer(const booster_interface::msg::Odometer::SharedPtr msg)
    {
        double current_x = static_cast<double>(msg->x);
        double current_y = static_cast<double>(msg->y);
        double current_a = static_cast<double>(msg->theta);

        if (!initial_pose_set_) {
            init_x_ = current_x;
            init_y_ = current_y;
            init_a_ = current_a;
            initial_pose_set_ = true;
            RCLCPP_INFO(this->get_logger(), "Initial odometry captured. Setting starting frame to (0,0,0).");
        }

        double dx = current_x - init_x_;
        double dy = current_y - init_y_;

        robot_x_ = dx * std::cos(init_a_) + dy * std::sin(init_a_);
        robot_y_ = -dx * std::sin(init_a_) + dy * std::cos(init_a_);

        double relative_yaw = current_a - init_a_;
        robot_a_ = std::atan2(std::sin(relative_yaw), std::cos(relative_yaw));

    }

    void timer_callback()
    {
        if (!initial_pose_set_) return;

        auto t = this->get_clock()->now();

        geometry_msgs::msg::TransformStamped transform;
        transform.header.stamp = t;
        transform.header.frame_id = "pumas_odom";
        transform.child_frame_id = "pumas_base_link";

        transform.transform.translation.x = robot_x_;
        transform.transform.translation.y = robot_y_;
        transform.transform.translation.z = 0.0;

        auto q = euler_to_quaternion(0.0, 0.0, robot_a_);
        transform.transform.rotation.x = std::get<0>(q);
        transform.transform.rotation.y = std::get<1>(q);
        transform.transform.rotation.z = std::get<2>(q);
        transform.transform.rotation.w = std::get<3>(q);

        broadcaster_->sendTransform(transform);
    }

    std::tuple<double, double, double, double> euler_to_quaternion(double roll, double pitch, double yaw)
    {
        double qx = std::sin(roll/2) * std::cos(pitch/2) * std::cos(yaw/2) -
                    std::cos(roll/2) * std::sin(pitch/2) * std::sin(yaw/2);
        double qy = std::cos(roll/2) * std::sin(pitch/2) * std::cos(yaw/2) +
                    std::sin(roll/2) * std::cos(pitch/2) * std::sin(yaw/2);
        double qz = std::cos(roll/2) * std::cos(pitch/2) * std::sin(yaw/2) -
                    std::sin(roll/2) * std::sin(pitch/2) * std::cos(yaw/2);
        double qw = std::cos(roll/2) * std::cos(pitch/2) * std::cos(yaw/2) +
                    std::sin(roll/2) * std::sin(pitch/2) * std::sin(yaw/2);
        return {qx, qy, qz, qw};
    }

    rclcpp::Subscription<booster_interface::msg::Odometer>::SharedPtr subscription_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster_;

    bool initial_pose_set_ = false;
    double init_x_ = 0.0;
    double init_y_ = 0.0;
    double init_a_ = 0.0;
    

    double robot_x_ = 0.0;
    double robot_y_ = 0.0;
    double robot_a_ = 0.0;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<OdomToTFNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
