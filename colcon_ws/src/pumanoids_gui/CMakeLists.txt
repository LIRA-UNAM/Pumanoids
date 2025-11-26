cmake_minimum_required(VERSION 3.8)
project(pumanoids_gui LANGUAGES CXX)

set(ROS_DEPENDS
  ament_cmake
  rclcpp
  std_msgs
  sensor_msgs
  cv_bridge
  geometry_msgs
)

set(EXTERNAL_DEPENDS
  Qt5Widgets
  OpenCV
)

set(EXTERNAL_DEPENDS_LIB
  Qt5::Widgets
  Qt5::Network
  Qt5::Core
)

## Compile as C++14
add_compile_options(-std=c++17)

# Find external dependencies
set(CMAKE_INCLUDE_CURRENT_DIR ON)
foreach(DEPEND ${EXTERNAL_DEPENDS})
  find_package(${DEPEND} REQUIRED)
endforeach(DEPEND)

#Find ROS dependencies
foreach(DEPEND ${ROS_DEPENDS})
  find_package(${DEPEND})
endforeach(DEPEND)

#Find Qt
find_package(Qt5 COMPONENTS Core Network Widgets)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

file(GLOB SRCS
  src/*.cpp
)

file(GLOB QRC
  resources/*.qrc
)

file(GLOB UI 
  ui/*.ui
)

file(GLOB HDRS
  include/*.h
  include/*.hpp
)

message("Adding ${PROJECT_NAME} UI files: " )
foreach(source ${UI})
    message("  - " ${source})
endforeach(source)

# Generate headers
qt5_wrap_ui(UI_HDRS ${UI})
qt5_wrap_cpp(QT_MOC_HPP ${HDRS})
qt5_add_resources(QRC_RSC ${QRC})


message("Generating ${PROJECT_NAME} UI Headers: " )
foreach(source ${UI_HDRS})
    message("  - " ${source})
endforeach(source)

include_directories(
  include
  ${CMAKE_CURRENT_BINARY_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}
)

add_executable(pumanoids_gui_node ${SRCS} ${UI_HDRS} ${QT_MOC_HPP} ${QRC_RSC})
ament_target_dependencies(pumanoids_gui_node ${ROS_DEPENDS})
target_link_libraries(pumanoids_gui_node ${EXTERNAL_DEPENDS_LIB})

install(TARGETS pumanoids_gui_node
  DESTINATION lib/${PROJECT_NAME}
)

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
