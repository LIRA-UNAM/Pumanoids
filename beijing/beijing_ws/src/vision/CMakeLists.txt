cmake_minimum_required(VERSION 3.10)
project(vision)

set(CMAKE_CXX_STANDARD 17)

option(BUILD_TEST "Build test" OFF)
option(ENABLE_ONNX "use onnx format model" OFF)
option(BUILD_CALIBRATION "Build calibration module" ON)

# find ros2 package
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ament_index_cpp REQUIRED)
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
find_package(CUDAToolkit REQUIRED)

find_path(TENSORRT_INCLUDE_DIR NvInfer.h
  HINTS /usr/include /usr/local/include /usr/include/aarch64-linux-gnu)
find_library(TENSORRT_LIB nvinfer
  HINTS /usr/lib /usr/local/lib /usr/lib/aarch64-linux-gnu)
if(NOT TENSORRT_INCLUDE_DIR OR NOT TENSORRT_LIB)
  message(FATAL_ERROR "TensorRT headers/libs not found")
endif()

# Keep the TensorRT major version available to subdirectories.  TensorRT 10
# removed the legacy IPluginV2 APIs used by the engine builder in this package;
# those sources must only be compiled for the older TensorRT 8.6 path.
if(NOT DEFINED TENSORRT_MAJOR_VERSION OR
   "${TENSORRT_MAJOR_VERSION}" STREQUAL "")
  set(TENSORRT_MAJOR_VERSION "")
endif()
if(NOT DEFINED TENSORRT_MINOR_VERSION OR
   "${TENSORRT_MINOR_VERSION}" STREQUAL "")
  set(TENSORRT_MINOR_VERSION "")
endif()
find_path(TENSORRT_VERSION_INCLUDE_DIR NvInferVersion.h
  HINTS "${TENSORRT_INCLUDE_DIR}" /usr/include /usr/local/include
        /usr/include/aarch64-linux-gnu)
set(_trt_version_headers
  "${TENSORRT_VERSION_INCLUDE_DIR}/NvInferVersion.h"
  "${TENSORRT_INCLUDE_DIR}/NvInfer.h"
  "${TENSORRT_INCLUDE_DIR}/NvInferRuntime.h"
  "${TENSORRT_INCLUDE_DIR}/NvInferRuntimeBase.h")
foreach(_trt_version_header IN LISTS _trt_version_headers)
  if(EXISTS "${_trt_version_header}")
    file(STRINGS "${_trt_version_header}" _trt_major_define
         REGEX "^[ \t]*#define[ \t]+NV_TENSORRT_MAJOR[ \t]+[0-9]+")
    if(TENSORRT_MAJOR_VERSION STREQUAL "")
      string(REGEX MATCH "[0-9]+" TENSORRT_MAJOR_VERSION "${_trt_major_define}")
    endif()
    file(STRINGS "${_trt_version_header}" _trt_minor_define
         REGEX "^[ \t]*#define[ \t]+NV_TENSORRT_MINOR[ \t]+[0-9]+")
    if(TENSORRT_MINOR_VERSION STREQUAL "")
      string(REGEX MATCH "[0-9]+" TENSORRT_MINOR_VERSION "${_trt_minor_define}")
    endif()
  endif()
endforeach()
if(TENSORRT_MAJOR_VERSION STREQUAL "")
  # Some JetPack packages omit the version macro from their installed public
  # headers. JetPack 7.2 is TensorRT 10; allow that target to configure while
  # still permitting an explicit value for other installations.
  set(TENSORRT_MAJOR_VERSION 10)
  message(WARNING
    "TensorRT version header was not found; assuming TensorRT 10. "
    "Set -DTENSORRT_MAJOR_VERSION=8 for a TensorRT 8.6 installation.")
endif()
if(TENSORRT_MINOR_VERSION STREQUAL "")
  # TensorRT 8 support in this package is specifically for 8.6. TensorRT 10
  # does not use the minor value in any compile-time branch below.
  if(TENSORRT_MAJOR_VERSION VERSION_EQUAL 8)
    set(TENSORRT_MINOR_VERSION 6)
  else()
    set(TENSORRT_MINOR_VERSION 0)
  endif()
endif()
message(STATUS "Using TensorRT ${TENSORRT_MAJOR_VERSION}: ${TENSORRT_LIB}")

# NvInferVersion.h is not installed by some JetPack 7.2 packages. The
# sources still use NV_TENSORRT_* to select the API implementation, so pass
# the detected/configured version explicitly.
add_compile_definitions(
  NV_TENSORRT_MAJOR=${TENSORRT_MAJOR_VERSION}
  NV_TENSORRT_MINOR=${TENSORRT_MINOR_VERSION})

if(TENSORRT_MAJOR_VERSION VERSION_GREATER_EQUAL 10 AND BUILD_CALIBRATION)
  message(STATUS
    "Building calibration for TensorRT ${TENSORRT_MAJOR_VERSION}; "
    "a target-compatible system Ceres installation is required")
endif()

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
    ${OpenCV_INCLUDE_DIRS}
    ${YAML_CPP_INCLUDE_DIR}
    ${PCL_INCLUDE_DIRS}
    ${TENSORRT_INCLUDE_DIR}
)
if(TENSORRT_VERSION_INCLUDE_DIR)
  include_directories(${TENSORRT_VERSION_INCLUDE_DIR})
endif()

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
