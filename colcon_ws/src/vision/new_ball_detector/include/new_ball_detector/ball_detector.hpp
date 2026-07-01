#pragma once

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <pumas_vision_msgs/msg/vision_object.hpp>
#include <localization_msg/msg/vision_landmark.hpp>
#include <localization_msg/msg/vision_landmark_array.hpp>

#include <joint_states_package/srv/head_joints.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <opencv2/opencv.hpp>
#include "tensorrt_detector.hpp"

class BallDetectorNode : public rclcpp::Node {
public:
  BallDetectorNode();

private:
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_img_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr sub_cam_info_;

  rclcpp::Client<joint_states_package::srv::HeadJoints>::SharedPtr joint_client_;
  rclcpp::TimerBase::SharedPtr joint_timer_;

  rclcpp::Publisher<pumas_vision_msgs::msg::VisionObject>::SharedPtr pub_ball_;
  rclcpp::Publisher<geometry_msgs::msg::Pose2D>::SharedPtr pub_ball_map_;
  rclcpp::Publisher<localization_msg::msg::VisionLandmarkArray>::SharedPtr pub_landmarks_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_debug_img_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);
  void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);
  void jointStateRequest();
  void publishBallInMap(double bx, double by, const rclcpp::Time & stamp);

  // Undistort one pixel back to an ideal pinhole image (returns pixel coords).
  // No-op until a valid CameraInfo has been received.
  cv::Point2f undistortPixel(float u, float v) const;

  double computeAngle(double img_x, int img_width);
  std::vector<Detection> angularPrune(std::vector<Detection> & dets,
                                      int img_width, double min_sep_rad);
  std::pair<double, double> getBallPosition(double img_x, double img_y,
                                            double bbox_w, double bbox_h,
                                            int img_width, int img_height);

  double hfov_deg;
  double vfov_deg;
  double hfov_rad;
  double vfov_rad;
  double head_z_;
  double ball_radius_;
  double proc_interval_;
  std::string model_path_;
  std::string camera_topic_;
  std::string camera_info_topic_;
  bool show_debug_;
  std::string base_frame_;
  std::string map_frame_;
  std::string ball_map_topic_;

  // --- Camera intrinsics / distortion (populated by cameraInfoCallback) ---
  // NOTE: cameraInfoCallback and imageCallback are assumed to run on the same
  //       (default, single-threaded) executor, so these are unguarded. If you
  //       switch to a MultiThreadedExecutor, protect them with a mutex.
  bool have_camera_info_ = false;
  double fx_ = 0.0;
  double fy_ = 0.0;
  double cx_ = 0.0;
  double cy_ = 0.0;
  cv::Mat K_;   // 3x3 intrinsic matrix
  cv::Mat D_;   // distortion coefficients

  double current_head_pan_ = 0.0;
  double current_head_tilt_ = 0.0;
  rclcpp::Time last_proc_time_;

  std::unique_ptr<YoloDetector> detector_;
};
