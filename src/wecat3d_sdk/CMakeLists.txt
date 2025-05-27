cmake_minimum_required(VERSION 3.8)
project(wecat3d_sdk)

find_package(ament_cmake REQUIRED)

# Export headers and library for other ROS 2 packages
include_directories(
  include
)

link_directories(
  lib
)

# Export to other packages
ament_export_include_directories(include)
ament_export_libraries(EthernetScanner)
ament_export_dependencies(ament_cmake)

ament_package()

