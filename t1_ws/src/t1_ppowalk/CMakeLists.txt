cmake_minimum_required(VERSION 3.16)
project(t1_ppowalk)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(booster_interface REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

include(cmake/FetchOnnxRuntime.cmake)

# Executables live in lib/t1_ppowalk, the bundled onnxruntime in lib.
set(CMAKE_INSTALL_RPATH "$ORIGIN/..;$ORIGIN")

# ONNX policy + observation helpers (vendored from G1-Software's getup, without
# its Unitree dependencies).
add_library(t1_ppowalk_core STATIC
  src/onnx_policy.cpp
  src/observation.cpp
)
target_include_directories(t1_ppowalk_core PUBLIC include)
target_link_libraries(t1_ppowalk_core PUBLIC onnxruntime::onnxruntime)
set_target_properties(t1_ppowalk_core PROPERTIES POSITION_INDEPENDENT_CODE ON)

add_executable(t1_ppowalk_policy_node src/t1_ppowalk_policy_node.cpp)
target_link_libraries(t1_ppowalk_policy_node t1_ppowalk_core)
ament_target_dependencies(t1_ppowalk_policy_node
  rclcpp booster_interface geometry_msgs std_msgs std_srvs)

install(TARGETS t1_ppowalk_policy_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch models DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_motion test/test_motion.cpp)
  target_link_libraries(test_motion t1_ppowalk_core)
endif()

ament_package()
