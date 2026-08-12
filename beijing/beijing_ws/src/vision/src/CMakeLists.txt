add_subdirectory(base)
add_subdirectory(model)
add_subdirectory(pose_estimator)

if(CMAKE_SYSTEM_PROCESSOR STREQUAL "aarch64" AND BUILD_CALIBRATION)
  # Do not fall back to thirdparty/lib here. That Ceres binary was built for
  # CUDA 12 and is incompatible with the JetPack 7.2 target.
  find_package(Ceres 2.1 QUIET)
  if(Ceres_FOUND)
    add_subdirectory(calibration)
  else()
    message(FATAL_ERROR
      "BUILD_CALIBRATION is ON but a target-compatible Ceres was not found. "
      "Install libceres-dev on the robot, then rebuild with a clean CMake "
      "cache; do not use src/vision/thirdparty/lib on JetPack 7.2")
  endif()
endif()

add_executable(vision_node main.cpp 
                           img_bridge.cpp
                           vision_node.cpp)


target_link_libraries(vision_node booster_vision_base
                                     booster_vision_pose_estimator
                                     yolov8_model
                                     ${typesupport_target}
                                     )

ament_target_dependencies(vision_node
  rclcpp
  ament_index_cpp
  std_msgs
  sensor_msgs
  geometry_msgs
  image_transport
  tf2
  builtin_interfaces
  vision_interface
)


install(TARGETS vision_node 
  DESTINATION lib/${PROJECT_NAME})
