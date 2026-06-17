/* 
 *  ----------------------------------------
 *  go_to_target.cpp
 *  Package: go_to_target
 *  Node: go_to_target
 *  ----------------------------------------
 *  This node receives a Pose2D point in the map and walks the robot to it.
 *  ----------------------------------------
 *  Written by Camile Frias, Ruth Moreno and Sebastian Garcia.
 *  Developed at LIRA UNAM.
 *  https://lira.unam.mx/
 *  ----------------------------------------
 */

#include <cstdio>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2/LinearMath/Quaternion.h"
#include <cmath>

enum class State {iddle, ball_align, forward, goal_align}; 
class GoToTarget : public rclcpp::Node
{
    public:
        GoToTarget(): Node("go_to_target")
    {
        // -- VARIABLES  --
        state = State::iddle;

        // -- PARAMETERS --
        this->declare_parameter<double>("v_max", 0.3);        // Maximum velocity
        this->declare_parameter<double>("w_max", 1);        // Maximum rotation
        this->declare_parameter<double>("alpha", 0.3);        // alpha constant
        this->declare_parameter<double>("beta", 0.3);         // beta constant
        this->declare_parameter<double>("dist_min", 0.5);   // Minimum distance to target for the robot to stop

        this->get_parameter("v_max", v_max);
        this->get_parameter("w_max", w_max);
        this->get_parameter("alpha", alpha);
        this->get_parameter("beta", beta);
        this->get_parameter("dist_min", dist_min);

        // -- TOPICS --
        // Publishers
        movement_publisher = this->create_publisher<geometry_msgs::msg::Twist>(
                "/cmd_vel", 10);
        success_publisher = this->create_publisher<std_msgs::msg::Bool>(
                "/go_to_target/success", 10);

        // Subscribers
        pose_subscriber = this->create_subscription<geometry_msgs::msg::Pose2D>(
                "/go_to_target/target",
                10,
                std::bind(&GoToTarget::target_callback, this, std::placeholders::_1));

        enable_subscriber = this->create_subscription<std_msgs::msg::Bool>(
                "/go_to_target/enable", 1,
                std::bind(&GoToTarget::enable_callback, this, std::placeholders::_1));

        // -- TF2 --
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // -- TIMERS --
        timer_ = this->create_wall_timer(
                std::chrono::milliseconds(100),
                std::bind(&GoToTarget::timer_callback, this));
    }
    private:

        // -- ROS2 OBJECTS DECLARATIONS --
        // Topics
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr enable_subscriber;
        rclcpp::Subscription<geometry_msgs::msg::Pose2D>::SharedPtr pose_subscriber;
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr success_publisher;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr movement_publisher;

        // Timers
        rclcpp::TimerBase::SharedPtr timer_;

        // TF2
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
        std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
        geometry_msgs::msg::TransformStamped t;

        // Pose2D
        geometry_msgs::msg::Pose2D target {};

        // -- VARIABLES --
        bool enable { false };  // To enable or disable the robot movement.
        bool new_target {false};        // Variable para saber si es la primera vez del callback en la máquina de estados 
        double robot_yaw {};    // yaw orientation of the robot.
        double robot_x {};      // Orientation of the robot in the X axis.
        double robot_y {};      // Orientation of the robot in the Y axis.
        double err_a {};        // Angular error.
        double err_dist {};     // Distance error.
        double vel {};          // Robot linear velocity to be sent through /cmd_vel.
        double w {};            // Robot angular velocity to be sent through /cmd_vel.
        State state;            // Estado de la máquina de estados
        geometry_msgs::msg::Twist twist_msg{};// Mensaje de Twist ROS2
        std_msgs::msg::Bool success{};  // Mensaje de Success
              

        // Constants
        double v_max; 
        double w_max;
        double alpha; 
        double beta;
        double dist_min;

        // -- METHODS --
        void target_callback(const geometry_msgs::msg::Pose2D::SharedPtr msg)
        {
            //RCLCPP_INFO(this->get_logger(), "Target point x= '%.3f' y= '%.3f'", msg->x, msg->y);
            success.data = false;
            success_publisher->publish(success);
            new_target = true;
            enable = true;
            target = *msg;
        }

        void enable_callback(const std_msgs::msg::Bool::SharedPtr msg)
        {
            //RCLCPP_INFO(this->get_logger(), "Recived: '%s'", msg->data ? "true" : "false");
            enable = msg->data;
        }

        // Main logic of the program
        void timer_callback()
        {
          try 
          {
              // Please follow this naming convention for tf2 frames.
              t = tf_buffer_->lookupTransform("pumas_map", "pumas_base_link", tf2::TimePointZero);

              // Get the current robot poistion in the map.
              robot_yaw = normalizeAngle(atan2(t.transform.rotation.z, t.transform.rotation.w)*2);
              robot_x = t.transform.translation.x;
              robot_y = t.transform.translation.y; 
              err_dist = sqrt(std::pow(target.y - robot_y, 2) + std::pow(target.x - robot_x, 2));
              err_a    = normalizeAngle(atan2(target.y - robot_y, target.x - robot_x) - robot_yaw);
              vel      = v_max * std::exp((std::pow(err_a, 2)) / (-alpha));
              w        = w_max * (2 / (1 + std::exp(-err_a / beta)) - 1);
              //RCLCPP_INFO(this->get_logger(), "\n err_dist: %.2f, err_a: %.2f, vel: %.2f, w: %.2f", err_dist, err_a, vel, w );
              if(!enable) state = State::iddle;
              switch(state)
              {
                case State::iddle:
                 // RCLCPP_INFO(this->get_logger(), "Not Moving to the ball");
                  if ( new_target )
                  {
                      state = State::ball_align;
                      new_target = false;
                  }
                  else
                  {
                    return;
                  }
                  break;

                case State::ball_align:
                  //RCLCPP_INFO(this->get_logger(), "Facing to the ball");
                  twist_msg.angular.z = w;
                  state = std::abs(err_a) < 0.2 ? State::forward : State::ball_align;
                  break;

                case State::forward:
                  //RCLCPP_INFO(this->get_logger(), "Walking forward");
                  twist_msg.linear.x = vel;
                  twist_msg.angular.z = w;//std::abs(err_a) > 0.2 ? w : 0.0;
                  state = err_dist < dist_min ? State::goal_align : State::forward;
                  break;

                case State::goal_align:
                  //RCLCPP_INFO(this->get_logger(), "Facing the goal");
                  twist_msg.linear.x = 0.0;
                  err_a = normalizeAngle(target.theta - robot_yaw);
                  w = w_max * (2/(1 + std::exp(-err_a / beta)) - 1);
                  if (std::abs(err_a) < 0.2)
                  {
                      twist_msg.angular.z = 0.0;
                      success.data = true;
                      success_publisher->publish(success);
                      state = State::iddle;
                      enable = false;
                  }
                  twist_msg.angular.z = w;
                  break;

              }
              if( enable ) movement_publisher->publish(twist_msg);
          }            
          // This one happens a lot. Takes a few secons for the transformation to be published.
          catch (const tf2::LookupException& ex)
          {
              //RCLCPP_INFO(this->get_logger(), "Waiting for pumas_map -> pumas_base_link");
          }
          // Any other exception should not happen.
          catch (const tf2::TransformException& ex) 
          {
              //RCLCPP_ERROR(this->get_logger(), "TF2 exception: %s", ex.what());
          }
        }

        // Returns the given angle normalized
        float normalizeAngle(float angle) 
        {
            //return fmod(angle + M_PI, 2.0 * M_PI) - M_PI;
            if (angle > M_PI) angle -= 2* M_PI;
            if ( angle <= -M_PI ) angle += 2 * M_PI;
            return angle;
        }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<GoToTarget>();
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
