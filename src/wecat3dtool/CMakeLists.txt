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

add_executable(x_z_cont src/x_z_cont.cpp)
target_link_libraries(x_z_cont EthernetScanner)

add_executable(get_x_y_pts src/get_x_y_pts.cpp)
target_link_libraries(get_x_y_pts EthernetScanner)

add_executable(get_x_y_z_pts src/get_x_y_z_pts.cpp)
target_link_libraries(get_x_y_z_pts EthernetScanner)

add_executable(get_readings src/get_readings.cpp)
target_link_libraries(get_readings EthernetScanner)

add_executable(test_x_y_z_print src/test_x_y_z_print.cpp)
target_link_libraries(test_x_y_z_print EthernetScanner)

add_executable(test2 src/test2.cpp)
target_link_libraries(test2 EthernetScanner)

add_executable(test_profile src/test_profile.cpp)
target_link_libraries(test_profile EthernetScanner)
# Install them so you can run with ros2 run
install(TARGETS
  check_connection
  get_x_y_pts
  get_x_y_z_pts
  x_z_cont
  get_readings
  test_x_y_z_print
  test2
  test_profile
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

