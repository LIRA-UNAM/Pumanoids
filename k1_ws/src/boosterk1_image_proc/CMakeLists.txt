cmake_minimum_required(VERSION 3.8)
project(boosterk1_image_proc)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)

find_package(CUDA REQUIRED)
include_directories(${CUDA_INCLUDE_DIRS})

# jetson-utils for efficient conversion
# get and install the utils from https://github.com/dusty-nv/jetson-utils
find_package(jetson-utils REQUIRED)

add_executable(nv12_converter_node src/nv12_converter_node.cpp)

target_include_directories(nv12_converter_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${CUDA_INCLUDE_DIRS}
)

ament_target_dependencies(nv12_converter_node
  rclcpp
  sensor_msgs
  cv_bridge
)

target_link_libraries(nv12_converter_node
  jetson-utils
  ${CUDA_LIBRARIES}
)

install(TARGETS
  nv12_converter_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
