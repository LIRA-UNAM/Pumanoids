#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <booster_msgs/msg/rpc_req_msg.hpp>
#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <booster_interface/msg/odometer.hpp>
#include <nlohmann/json.hpp>

#include <cmath>
#include <string>

using json = nlohmann::json;
class OdomToTFNode : public rclcpp::Node
{
public:
    OdomToTFNode()
    : Node("odom_to_tf")
    {
        broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        
        vel_sub_ = this->create_subscription<booster_msgs::msg::RpcReqMsg>(
            "/LocoApiTopicReq", 10,
            std::bind(&OdomToTFNode::callback_vel_, this, std::placeholders::_1));

        subscription_ = this->create_subscription<booster_interface::msg::Odometer>(
            "/odometer_state", 10,
            std::bind(&OdomToTFNode::callback_odometer, this, std::placeholders::_1));

        reset_odom_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/odometry_restart", 3,
            std::bind(&OdomToTFNode::reset_odometry, this, std::placeholders::_1));

        tf_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&OdomToTFNode::tf_timer_callback, this));

        odom_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&OdomToTFNode::odom_timer_callback,this));

        RCLCPP_INFO(this->get_logger(), "Booster Odom to TF Node has been started.");
    }

private:

    void callback_vel_(const booster_msgs::msg::RpcReqMsg::SharedPtr msg) const
    {
        //robot_vel_x_ = msg->linear.x;
        //robot_vel_y_ = msg->linear.y;
        //vyaw_ = msg->angular.z;
        try
        {
            json header_json = json::parse(msg->header);
            if (header_json.contains("api_id") && header_json["api_id"] == 2001)
            {
                json body_json = json::parse(msg->body);
                if (body_json.contains("vx") && body_json.contains("vy")) 
                {
                    robot_vel_x_ = body_json["vx"];
                    robot_vel_y_ = body_json["vy"];

                    RCLCPP_DEBUG(this->get_logger(),
                            "Received Velocities -> vx: %.3f, vy: %.3f",
                            robot_vel_x_, robot_vel_y_);
                }
            }
        }
        catch (const json::parse_error& e)
        {
            RCLCPP_ERROR(this->get_logger(), "JSON Parse Error: %s. Message body was: %s", 
                         e.what(), msg->body.c_str());
        }
        catch (const json::type_error& e)
        {
            RCLCPP_ERROR(this->get_logger(), "JSON Type Error (missing field?): %s", e.what());
        }
    }

    void callback_odometer(const booster_interface::msg::Odometer::SharedPtr msg)
    {
        // Due to the inaccuracy of the Booster T1 linear odometer, we calculate
        // an estimation with the velocities sent through /cmd_vel. We only retreive
        // the angular reading of the odometer.
        

        double current_yaw = static_cast<double>(msg->theta);

        if (!first_odom_) {
            init_yaw_ = current_yaw;
            first_odom_ = true;
            RCLCPP_INFO(this->get_logger(),
                    "Initial yaw captured: %.3f rad. Setting baseline to 0.", init_yaw_);
        }

        double relative_yaw = current_yaw - init_yaw_;

        //robot_x_ = static_cast<double>(msg->x);
        //robot_y_ = static_cast<double>(msg->y);
        // Normalize to [-pi, pi)
        robot_a_ = std::atan2(std::sin(relative_yaw), std::cos(relative_yaw));
        latest_a_ = robot_a_;
    }


    void odom_timer_callback()
    {
        temp_robot_x_ = getTempX(robot_vel_x_);
        temp_robot_y_ = getTempY(robot_vel_y_);
        
        if (first_odom_)
        {
            robot_x_ += (temp_robot_x_ * std::cos(robot_a_)) - (temp_robot_y_ * std::sin(robot_a_));
            robot_y_ += (temp_robot_x_ * std::sin(robot_a_)) + (temp_robot_y_ * std::cos(robot_a_));
            latest_x_ = robot_x_;
            latest_y_ = robot_y_;
        }

        RCLCPP_DEBUG(this->get_logger(),"X: %0.3f , Y: %0.3f", robot_x_, robot_y_);
    }
        
    double getTempX(double temp_vel)
    {
        return (( 50 * temp_vel * error_x_ ) / 1000);
    }

    double getTempY(double temp_vel)
    {
        if (temp_vel != 0)
        {
            if (temp_vel > 0)
            {
                 return (( 50 * ( -0.33 * log(temp_vel) + 0.31 ) * temp_vel + ( 0.17 * temp_vel + 0.06 ) ) / 1000);
            }
            else
            {
                return (( 50 * ( 0.33 * log(-temp_vel) + 0.31 ) * temp_vel + ( 0.17 * temp_vel + 0.06 ) ) / 1000);
            }
        }
        else return 0;
    }

    void reset_odometry(const std_msgs::msg::Bool::SharedPtr msg)
    {
        if (!msg->data) return;

        init_yaw_ = latest_a_;

        robot_x_ = 0.0;
        robot_y_ = 0.0;
        robot_a_ = 0.0;
    }

    void tf_timer_callback()
    {
        auto t = this->get_clock()->now();

        geometry_msgs::msg::TransformStamped transform;
        transform.header.stamp = t;
        transform.header.frame_id = "pumas_odom";
        transform.child_frame_id = "pumas_base_link";

        //RCLCPP_DEBUG(this->get_logger(),"X: %0.3f , Y: %0.3f", robot_x_, robot_y_);

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
        double qx = std::sin(roll/2) * std::cos(pitch/2) * std::cos(yaw/2)
                    - std::cos(roll/2) * std::sin(pitch/2) * std::sin(yaw/2);
        double qy = std::cos(roll/2) * std::sin(pitch/2) * std::cos(yaw/2)
                    + std::sin(roll/2) * std::cos(pitch/2) * std::sin(yaw/2);
        double qz = std::cos(roll/2) * std::cos(pitch/2) * std::sin(yaw/2)
                    - std::sin(roll/2) * std::sin(pitch/2) * std::cos(yaw/2);
        double qw = std::cos(roll/2) * std::cos(pitch/2) * std::cos(yaw/2)
                    + std::sin(roll/2) * std::sin(pitch/2) * std::sin(yaw/2);
        return {qx, qy, qz, qw};
    }

    rclcpp::Time now = this->get_clock()->now();
    rclcpp::Subscription<booster_interface::msg::Odometer>::SharedPtr subscription_;
    rclcpp::Subscription<booster_msgs::msg::RpcReqMsg>::SharedPtr vel_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr reset_odom_sub_;    
    rclcpp::TimerBase::SharedPtr tf_timer_;
    rclcpp::TimerBase::SharedPtr odom_timer_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster_;

    // Position variables
    double temp_robot_x_ {};
    double temp_robot_y_ {};

    double latest_x_ = 0.0;
    double latest_y_ = 0.0;
    double latest_a_ = 0.0;   

    double robot_x_ {};
    double robot_y_ {};
    double robot_a_ {};
    double vyaw_ {};
    double init_yaw_ { 0.0 };

    // -- ERROR CONSTANTS --
    double error_x_ { 0.59 };
    double error_y_ { 1 };
    
    // Speed variables
    mutable double robot_vel_x_ {};
    mutable double robot_vel_y_ {};

    int count_ {};
    mutable bool first_odom_ = false;

    int iteration {};

};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<OdomToTFNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
