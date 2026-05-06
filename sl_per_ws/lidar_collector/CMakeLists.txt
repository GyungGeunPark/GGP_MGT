cmake_minimum_required(VERSION 3.8)
project(lidar_collector)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)

# Include PCL directories
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Build dual_lidar_node executable
add_executable(dual_lidar_node src/dual_lidar_node.cpp)

ament_target_dependencies(dual_lidar_node
  rclcpp
  sensor_msgs
  std_srvs
  pcl_conversions
)

target_link_libraries(dual_lidar_node
  ${PCL_LIBRARIES}
)

# Install executables
install(TARGETS
  dual_lidar_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Install config files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

# Install Python scripts
install(PROGRAMS
  scripts/mainwindow_simple.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
