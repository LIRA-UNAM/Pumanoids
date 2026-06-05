cmake_minimum_required(VERSION 3.8)
project(config_files)

find_package(ament_cmake REQUIRED)

install(DIRECTORY maps/
  DESTINATION share/${PROJECT_NAME}/maps
)

ament_package()
