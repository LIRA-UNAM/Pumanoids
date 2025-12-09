cmake_minimum_required(VERSION 3.8)
project(mapless_goal)

# Enable compiler warnings for GCC and Clang
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)  # If using geometry_msgs

# Find PCL (Point Cloud Library)
find_package(PCL REQUIRED)

# Find pcl_conversions (needed for pcl_conversions.h)
find_package(pcl_conversions REQUIRED)

# Explicitly find Zlib (if you're still using it for compression)
find_package(ZLIB REQUIRED)

# Add executable for compressed point cloud publisher
add_executable(compressed_point_cloud_publisher src/compressed_point_cloud_publisher.cpp)
add_executable(point_cloud_downsampling src/point_cloud_downsampling.cpp)

# Include directories for both executables
target_include_directories(compressed_point_cloud_publisher PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_include_directories(point_cloud_downsampling PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS}  # Ensure that PCL include directories are included
  ${pcl_conversions_INCLUDE_DIRS})  # Add pcl_conversions include directories

# Link PCL and pcl_conversions to the point_cloud_downsampling executable
target_link_libraries(point_cloud_downsampling
  ${PCL_LIBRARIES}  # Link PCL libraries
  ${ZLIB_LIBRARIES} # Link Zlib
)

# Link the necessary dependencies to the executables
ament_target_dependencies(
  compressed_point_cloud_publisher
  rclcpp
  sensor_msgs
  nav_msgs
  std_msgs
  tf2_ros
  tf2_geometry_msgs
  ZLIB  # Automatically links Zlib to your target
)

ament_target_dependencies(
  point_cloud_downsampling
  rclcpp
  sensor_msgs
  nav_msgs
  std_msgs
  tf2_ros
  tf2_geometry_msgs
  ZLIB  # Links Zlib to your target
  pcl_conversions  # Links pcl_conversions to your target
)

# Set compile features for C and C++
target_compile_features(compressed_point_cloud_publisher PUBLIC c_std_99 cxx_std_17)
target_compile_features(point_cloud_downsampling PUBLIC c_std_99 cxx_std_17)

# Install the executables
install(TARGETS
  compressed_point_cloud_publisher
  point_cloud_downsampling
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python scripts (if any)
install(PROGRAMS
  scripts/local_occ_grid.py
  DESTINATION lib/${PROJECT_NAME}/scripts
)

# If you want to include testing and linting, you can enable the following:
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # Skip copyright linter (comment out the line once the license is added)
  set(ament_cmake_copyright_FOUND TRUE)
  # Skip cpplint if not in a git repo
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Final package setup
ament_package()

