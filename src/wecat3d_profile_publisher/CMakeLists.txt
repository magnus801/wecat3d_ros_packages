cmake_minimum_required(VERSION 3.8)
project(wecat3d_profile_publisher)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(PCL REQUIRED COMPONENTS io common)
find_package(Boost REQUIRED COMPONENTS system filesystem iostreams thread date_time)
find_package(pcl_conversions REQUIRED)

# Include and library paths
include_directories(
  ${PCL_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  /usr/include/eigen3
  ${CMAKE_CURRENT_SOURCE_DIR}/../wecat3d_sdk/include
)

link_directories(
  ${PCL_LIBRARY_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/../wecat3d_sdk/lib
)

add_definitions(${PCL_DEFINITIONS})

add_executable(encoder_pointcloud src/encoder_pointcloud.cpp)
ament_target_dependencies(encoder_pointcloud
  rclcpp
  sensor_msgs
  std_msgs
)
target_link_libraries(encoder_pointcloud
  EthernetScanner
  ${PCL_IO_LIBRARIES}
  ${PCL_COMMON_LIBRARIES}
  ${Boost_LIBRARIES}
)
add_executable(encoder_test src/encoder_test.cpp)
ament_target_dependencies(encoder_test
  rclcpp
  sensor_msgs
  std_msgs
)

target_link_libraries(encoder_test
  EthernetScanner
  ${PCL_IO_LIBRARIES}
  ${PCL_COMMON_LIBRARIES}
  ${Boost_LIBRARIES}
)
#
# Executable 1: profile_publisher_node
#
add_executable(profile_publisher_node src/profile_publisher_node.cpp)

ament_target_dependencies(profile_publisher_node
  rclcpp
  sensor_msgs
  std_msgs
)

target_link_libraries(profile_publisher_node
  EthernetScanner
  ${PCL_IO_LIBRARIES}
  ${PCL_COMMON_LIBRARIES}
  ${Boost_LIBRARIES}
)

#
# Executable 2: save_profile_to_pcd
#
add_executable(save_profile_to_pcd src/save_profile_to_pcd.cpp)

ament_target_dependencies(save_profile_to_pcd
  rclcpp
  sensor_msgs
  std_msgs
)

target_link_libraries(save_profile_to_pcd
  EthernetScanner
  ${PCL_IO_LIBRARIES}
  ${PCL_COMMON_LIBRARIES}
  ${Boost_LIBRARIES}
)

add_executable(merge_and_save_node src/merge_and_save_node.cpp)
ament_target_dependencies(merge_and_save_node rclcpp sensor_msgs pcl_conversions)
target_link_libraries(merge_and_save_node ${PCL_LIBRARIES})
# Install both targets
install(TARGETS
  profile_publisher_node
  save_profile_to_pcd
  merge_and_save_node
  encoder_test
  encoder_pointcloud
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

