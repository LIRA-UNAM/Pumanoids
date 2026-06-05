#include "new_ball_detector/ball_detector.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<BallDetectorNode>();
  // Multi-threaded executor for parallelism
  rclcpp::executors::MultiThreadedExecutor exec(rclcpp::ExecutorOptions(), 3);
  exec.add_node(node);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}
