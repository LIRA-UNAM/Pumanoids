cmake_minimum_required(VERSION 3.8)
project(dynamixel)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(dynamixel_sdk REQUIRED)

set(SOURCES
    src/cm730_utils.cpp
    src/servo_utils.cpp
)

add_executable(cm730_v3_node src/cm730_v3.cpp ${SOURCES})

target_include_directories(cm730_v3_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)

target_compile_features(cm730_v3_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

ament_target_dependencies( cm730_v3_node
  "rclcpp"
  "std_msgs"
  "geometry_msgs"
  "sensor_msgs"
  "tf2_msgs"
  "dynamixel_sdk"
)

add_executable(allpositions_node src/allpositions.cpp ${SOURCES})

target_include_directories(allpositions_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
  ${dynamixel_sdk_INCLUDE_DIRS}
)

target_compile_features(allpositions_node PUBLIC c_std_99 cxx_std_17)

ament_target_dependencies(allpositions_node
  "rclcpp"
  "std_msgs"
  "geometry_msgs"
  "sensor_msgs"
  "tf2_msgs"
  "dynamixel_sdk"
)

add_executable(position_test_node src/position_test.cpp ${SOURCES})

target_include_directories(position_test_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
  ${dynamixel_sdk_INCLUDE_DIRS}
)

target_compile_features(position_test_node PUBLIC c_std_99 cxx_std_17)

ament_target_dependencies(position_test_node
  "rclcpp"
  "std_msgs"
  "geometry_msgs"
  "sensor_msgs"
  "tf2_msgs"
  "dynamixel_sdk"
)

add_executable(read1byte_node src/read1byte.cpp ${SOURCES})

target_include_directories(read1byte_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
  ${dynamixel_sdk_INCLUDE_DIRS}
)

target_compile_features(read1byte_node PUBLIC c_std_99 cxx_std_17)

ament_target_dependencies(read1byte_node
  "rclcpp"
  "std_msgs"
  "geometry_msgs"
  "sensor_msgs"
  "tf2_msgs"
  "dynamixel_sdk"
)

add_executable(read2byte_node src/read2byte.cpp ${SOURCES})

target_include_directories(read2byte_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
  ${dynamixel_sdk_INCLUDE_DIRS}
)

target_compile_features(read2byte_node PUBLIC c_std_99 cxx_std_17)

ament_target_dependencies(read2byte_node
  "rclcpp"
  "std_msgs"
  "geometry_msgs"
  "sensor_msgs"
  "tf2_msgs"
  "dynamixel_sdk"
)

add_executable(write1byte_node src/write1byte.cpp ${SOURCES})

target_include_directories(write1byte_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
  ${dynamixel_sdk_INCLUDE_DIRS}
)

target_compile_features(write1byte_node PUBLIC c_std_99 cxx_std_17)

ament_target_dependencies(write1byte_node
  "rclcpp"
  "std_msgs"
  "geometry_msgs"
  "sensor_msgs"
  "tf2_msgs"
  "dynamixel_sdk"
)

install(TARGETS cm730_v3_node 
                allpositions_node 
                position_test_node
                read1byte_node
                read2byte_node
                write1byte_node
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
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
