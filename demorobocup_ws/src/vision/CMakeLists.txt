cmake_minimum_required(VERSION 3.10)
project(vision)

set(CMAKE_CXX_STANDARD 17)

option(BUILD_TEST "Build test" OFF)
option(ENABLE_ONNX "use onnx format model" OFF)
if(ENABLE_ONNX AND NOT DEFINED ENABLE_TENSORRT)
  set(_default_enable_tensorrt OFF)
else()
  set(_default_enable_tensorrt ON)
endif()
option(ENABLE_TENSORRT "use CUDA/TensorRT inference backend" ${_default_enable_tensorrt})

if(NOT ENABLE_ONNX AND NOT ENABLE_TENSORRT)
  message(FATAL_ERROR "At least one vision inference backend must be enabled (ENABLE_ONNX=ON or ENABLE_TENSORRT=ON).")
endif()

if(ENABLE_ONNX)
  add_definitions(-DENABLE_ONNX)
  find_path(ONNXRUNTIME_INCLUDE_DIRS onnxruntime_cxx_api.h
    HINTS
      ENV ONNXRUNTIME_ROOT
      ${ONNXRUNTIME_ROOT}
      /opt/onnxruntime
      /usr/local
      /usr
    PATH_SUFFIXES include include/onnxruntime onnxruntime core/session)
  find_library(ONNXRUNTIME_LIBRARIES
    NAMES onnxruntime
    HINTS
      ENV ONNXRUNTIME_ROOT
      ${ONNXRUNTIME_ROOT}
      /opt/onnxruntime
      /usr/local
      /usr
    PATH_SUFFIXES lib lib64)
  if(NOT ONNXRUNTIME_INCLUDE_DIRS OR NOT ONNXRUNTIME_LIBRARIES)
    message(FATAL_ERROR
      "ENABLE_ONNX=ON requires ONNX Runtime (onnxruntime_cxx_api.h and libonnxruntime). "
      "Install it in /opt/onnxruntime or pass -DONNXRUNTIME_INCLUDE_DIRS=... -DONNXRUNTIME_LIBRARIES=...")
  endif()
  message(STATUS "Vision ONNX backend enabled: ${ONNXRUNTIME_LIBRARIES}")
  set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
endif()

if(ENABLE_TENSORRT)
  add_definitions(-DENABLE_TENSORRT)
  message(STATUS "Vision TensorRT backend enabled")
endif()

# find ros2 package
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(tf2 REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(vision_interface REQUIRED)

find_package(OpenCV 4.5 REQUIRED)
find_package(YAML-CPP REQUIRED)
find_package(PCL REQUIRED)

# rosidl_generate_interfaces(${PROJECT_NAME}
#   "msg/DetectedObject.msg"
#   "msg/Detections.msg"
#   "msg/LineSegments.msg"
#   "msg/CalParam.msg"
#   DEPENDENCIES std_msgs builtin_interfaces
# )
# ament_export_dependencies(rosidl_default_runtime)
# rosidl_get_typesupport_target(typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

include_directories(
    ${PROJECT_SOURCE_DIR}/include
    ${PROJECT_SOURCE_DIR}/thirdparty/include
    ${OpenCV_INCLUDE_DIRS}
    ${YAML_CPP_INCLUDE_DIR}
    ${PCL_INCLUDE_DIRS}
)

add_subdirectory(src)
if(BUILD_TEST)
  add_subdirectory(test)
endif()

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY model
  DESTINATION share/${PROJECT_NAME})

ament_export_include_directories(include)
ament_package()
