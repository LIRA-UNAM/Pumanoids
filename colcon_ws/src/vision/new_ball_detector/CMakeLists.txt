cmake_minimum_required(VERSION 3.8)
project(new_ball_detector)
cmake_policy(SET CMP0074 NEW)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(localization_msg REQUIRED)
find_package(joint_states_package REQUIRED)
find_package(pumas_vision_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(CUDA REQUIRED)

include_directories(${CUDA_INCLUDE_DIRS})
message(STATUS "CUDA include dirs: ${CUDA_INCLUDE_DIRS}")
message(STATUS "CUDA libraries: ${CUDA_LIBRARIES}")

find_library(TensorRT_LIB nvinfer PATHS /usr/lib/aarch64-linux-gnu REQUIRED)

include_directories(/usr/include/aarch64-linux-gnu)

set(SOURCES
  src/ball_detector.cpp
  src/tensorrt_detector.cpp
  src/main.cpp
)

add_executable(ball_detector ${SOURCES})

target_include_directories(ball_detector PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(ball_detector
  ${TensorRT_LIB}
  ${OpenCV_LIBS}
  ${CUDA_CUDART_LIBRARY}
  ${CUDA_CUDA_LIBRARY}
)

ament_target_dependencies(
  ball_detector
  rclcpp
  sensor_msgs
  cv_bridge
  joint_states_package
  pumas_vision_msgs
  localization_msg
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
)

install(TARGETS
  ball_detector
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(DIRECTORY models
    DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_library(TensorRT_LIB nvinfer PATHS /usr/lib/aarch64-linux-gnu REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
