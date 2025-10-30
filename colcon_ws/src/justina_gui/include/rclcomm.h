#ifndef RCLCOMM_H
#define RCLCOMM_H
#include <QObject>
#include <QThread>
#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/float32.hpp"
#include "nav_msgs/srv/get_plan.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/twist.hpp"
class RclComm : public QThread, rclcpp::Node
{
    Q_OBJECT
public:
    RclComm(); 

    void publish_cmd_vel(double linear_x, double linear_y, double angular);
    void publish_cmd_vel(double linear_x, double angular);
    void start_publishing_cmd_vel(double linear_x, double linear_y, double angular);
    void start_publishing_cmd_vel(double linear_x, double angular);
    void stop_publishing_cmd_vel();
    bool call_plan_path(double start_x, double start_y, double goal_x, double goal_y, nav_msgs::msg::Path& path);

private:
    bool _publishing_cmd_vel;
    geometry_msgs::msg::Twist _cmd_vel;
    std::shared_ptr<rclcpp::Node> _node;
    rclcpp::TimerBase::SharedPtr _timer;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr _pub_cmd_vel;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr _sub_test;

    rclcpp::Client<nav_msgs::srv::GetPlan>::SharedPtr _clt_plan_path;

    void timer_callback();
    void callback_current_arm_pose(const std_msgs::msg::Float32 &msg);
    
signals:
    //void emitTopicData(QString);
};
#endif // RCLCOMM_H
