cmake_minimum_required(VERSION 3.8)
project(carry_ball_to_goal_interfaces)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/GetGoalRobotPose.srv"
  DEPENDENCIES geometry_msgs
)

ament_package()
