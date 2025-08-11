cmake_minimum_required(VERSION 3.8)
project(wecat3d_profile_publisher)

# Modern CMake: Use target-based dependencies instead of global variables
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ======== Dependencies ========
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)  # Modern PCL uses targets
find_package(Boost REQUIRED COMPONENTS system filesystem iostreams thread date_time)
find_package(pcl_conversions REQUIRED)
find_package(wecat3d_sdk REQUIRED)  # Critical missing dependency!

ament_environment_hooks(env/set_pcd_path.sh)

# ======== Target Setup ========
add_library(profile_generator_lib
  src/profile_generator.cpp
)

# Modern target-based approach
target_include_directories(profile_generator_lib
  PUBLIC 
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_compile_definitions(profile_generator_lib
  PRIVATE ${PCL_DEFINITIONS}
)

# Use modern ament_target_dependencies
ament_target_dependencies(profile_generator_lib
  rclcpp
  sensor_msgs
  std_msgs
  pcl_conversions
  PCL
  Boost
  wecat3d_sdk  # Link SDK via target
)

# # Link dependencies using CMake targets
# target_link_libraries(profile_generator_lib
#   PRIVATE
#     wecat3d_sdk::EthernetScanner  # Use exported target from SDK
#     pcl_conversions::pcl_conversions
#     Boost::boost
#     ${PCL_LIBRARIES}
# )

# ======== Executables ========
add_executable(profile_publisher_node src/profile_publisher_node.cpp)
target_link_libraries(profile_publisher_node profile_generator_lib)
ament_target_dependencies(profile_publisher_node rclcpp)

add_executable(profile_generator src/profile_generator.cpp)
target_link_libraries(profile_generator profile_generator_lib)
ament_target_dependencies(profile_generator rclcpp)

add_executable(profile_make src/profile_make.cpp)
target_link_libraries(profile_make profile_generator_lib)
ament_target_dependencies(profile_make rclcpp)
# ======== Installation ========
install(TARGETS 
  profile_publisher_node
  profile_generator
  profile_make
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS profile_generator_lib
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(DIRECTORY include/ DESTINATION include)

# Install the environment hook with proper permissions
install(
  FILES env/set_pcd_path.sh
  DESTINATION share/${PROJECT_NAME}/env
  PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE
)

# Install the pcd_output directory
install(
  DIRECTORY pcd_output/
  DESTINATION share/${PROJECT_NAME}/pcd_output
)

install(
  PROGRAMS   scripts/gauge_measurment.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  PROGRAMS   scripts/combined_wenglor.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  PROGRAMS   scripts/testing_combined.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  PROGRAMS   scripts/publish_pcd.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  PROGRAMS scripts/drishti_odometry.py
  DESTINATION lib/${PROJECT_NAME}
)

# install(
#   PROGRAMS   scripts/video_output_basler.py
#   DESTINATION lib/${PROJECT_NAME}
# )

# install(
#   PROGRAMS scripts/camera_inference.py
#   DESTINATION share/${PROJECT_NAME}/camera_inference
# )

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# install(PROGRAMS
#   scripts/detection_data_logging.py
#   DESTINATION lib/${PROJECT_NAME}
# )
##############################################################################
# Install Python nodes (drishti_odometry & combined_wenglor)
##############################################################################
install(
  PROGRAMS
    scripts/drishti_odometry.py
    scripts/combined_wenglor.py
  DESTINATION lib/${PROJECT_NAME}
)

# ======== Exports ========
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  ament_cmake
  rclcpp
  sensor_msgs
  std_msgs
  PCL
  Boost
  pcl_conversions
  wecat3d_sdk  # Export SDK as dependency
)
ament_export_include_directories(include)
ament_package()
