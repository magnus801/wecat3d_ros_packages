cmake_minimum_required(VERSION 3.8)
project(wecat3d_sdk)

find_package(ament_cmake REQUIRED)

# Optionally, create an IMPORTED target for use in this package or for export
# (but we won't install it, since IMPORTED targets can't be installed)
add_library(EthernetScanner SHARED IMPORTED)
set_target_properties(EthernetScanner PROPERTIES
  IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/lib/libEthernetScanner.so
  INTERFACE_INCLUDE_DIRECTORIES ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# Only export the target for consumers if you want to use modern CMake target-based exports
# (but since we can't install the IMPORTED target, it's better to use legacy exports here)
# So, we'll use the legacy export method:

# Install the .so file directly
install(FILES lib/libEthernetScanner.so DESTINATION lib)
# Install the headers
install(DIRECTORY include/ DESTINATION include)

# Export the include directories and library for other packages
ament_export_include_directories(include)
ament_export_libraries(EthernetScanner)

ament_package()
