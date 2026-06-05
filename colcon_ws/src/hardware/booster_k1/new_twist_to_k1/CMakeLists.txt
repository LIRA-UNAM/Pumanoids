cmake_minimum_required(VERSION 3.8)
project(new_twist_to_k1)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(booster_msgs REQUIRED)
find_package(booster_interface REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(twist_to_k1 src/twist_to_k1.cpp)
ament_target_dependencies(twist_to_k1
  rclcpp geometry_msgs booster_msgs
)
add_executable(pantilt_to_k1 src/pantilt_to_k1.cpp)
ament_target_dependencies(pantilt_to_k1
  rclcpp std_msgs booster_msgs
)
add_executable(odom_to_tf src/odom_to_tf.cpp)
ament_target_dependencies(odom_to_tf
  rclcpp booster_interface tf2_ros geometry_msgs
)

install(TARGETS
  twist_to_k1
  pantilt_to_k1
  odom_to_tf
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
