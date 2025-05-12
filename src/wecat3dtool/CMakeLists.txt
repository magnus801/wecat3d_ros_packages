cmake_minimum_required(VERSION 3.5)
project(wecat3dtool)

find_package(ament_cmake REQUIRED)

# Include SDK headers
include_directories(
  include
  ../wecat3d_sdk/include
)

link_directories(
  ../wecat3d_sdk/lib
)

# List of your tools
add_executable(check_connection src/check_connection.cpp)
target_link_libraries(check_connection EthernetScanner)

add_executable(get_x_y_pts src/get_x_y_pts.cpp)
target_link_libraries(get_x_y_pts EthernetScanner)

add_executable(get_x_y_z_pts src/get_x_y_z_pts.cpp)
target_link_libraries(get_x_y_z_pts EthernetScanner)

# Install them so you can run with ros2 run
install(TARGETS
  check_connection
  get_x_y_pts
  get_x_y_z_pts
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

