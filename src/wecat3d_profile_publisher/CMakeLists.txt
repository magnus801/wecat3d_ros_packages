cmake_minimum_required(VERSION 3.8)
project(wecat3d_profile_publisher)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find standard ROS 2 dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# Manually add SDK include and library paths
include_directories(
  /home/atharva/ros_ws/src/wecat3d_sdk/include
)

link_directories(
  /home/atharva/ros_ws/src/wecat3d_sdk/lib
)

add_executable(profile_publisher_node src/profile_publisher_node.cpp)
ament_target_dependencies(profile_publisher_node rclcpp sensor_msgs std_msgs)
target_link_libraries(profile_publisher_node EthernetScanner)

install(TARGETS
  profile_publisher_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
