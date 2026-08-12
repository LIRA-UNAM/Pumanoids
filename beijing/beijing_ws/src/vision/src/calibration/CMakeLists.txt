message(STATUS "Builing Calibration")
find_package(Ceres 2.1 REQUIRED)

add_library(booster_vision_calibration SHARED calibration.cpp
                                              board_detector.cpp
                                              )
target_link_libraries(booster_vision_calibration PUBLIC ${OpenCV_LIBRARIES}
                                                        ${YAML_CPP_LIBRARIES}
                                                        booster_vision_base
                                                        Ceres::ceres
                                                        )

if(TARGET glog::glog)
  target_link_libraries(booster_vision_calibration PUBLIC glog::glog)
endif()

add_executable(calibration_node calibration_node.cpp
                                ../img_bridge.cpp)
target_include_directories(calibration_node PRIVATE
  ${PROJECT_SOURCE_DIR}/thirdparty/include)

target_link_libraries(calibration_node booster_vision_base
                                       booster_vision_pose_estimator
                                       yolov8_model
                                       booster_vision_calibration
                                       ${typesupport_target}
                                     )

ament_target_dependencies(calibration_node
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  image_transport
  builtin_interfaces
)

install(TARGETS calibration_node
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS booster_vision_calibration
        DESTINATION lib)
