cmake_minimum_required(VERSION 3.8)
project(sensor_cam_mini)

# C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ROS2 / OpenCV 등
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc highgui cudaimgproc cudaarithm)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(livox_ros2_driver REQUIRED)

# GStreamer 찾기
find_package(PkgConfig REQUIRED)
pkg_check_modules(GST REQUIRED gstreamer-1.0>=1.14)
pkg_check_modules(GST_BASE REQUIRED gstreamer-plugins-base-1.0)

# Spinnaker SDK
include_directories(/opt/spinnaker/include)
link_directories(/opt/spinnaker/lib)

# include 디렉터리 설정
include_directories(
  ${GST_INCLUDE_DIRS}
  ${GST_BASE_INCLUDE_DIRS}
  # 혹은 target_include_directories를 통해 타겟에 연결
)

# 라이브러리 링크 디렉터리 설정
link_directories(
  ${GST_LIBRARY_DIRS}
  ${GST_BASE_LIBRARY_DIRS}
)

############################################
# camera_node
############################################
add_executable(camera_node src/camera_node.cpp)

ament_target_dependencies(camera_node
  rclcpp
  sensor_msgs
  cv_bridge
  image_transport
  OpenCV
  livox_ros2_driver
)

target_link_libraries(camera_node
  ${OpenCV_LIBS}
  ${GST_LIBRARIES}
  ${GST_BASE_LIBRARIES}
  # Spinnaker
  Spinnaker
  GenApi_gcc9_v3_0
  GCBase_gcc9_v3_0
)

############################################
# lidar_node
############################################
add_executable(lidar_node src/lidar_node.cpp)

ament_target_dependencies(lidar_node
  rclcpp
  sensor_msgs
  std_srvs
)

target_link_libraries(lidar_node
  stdc++fs
)

############################################
# 설치
############################################
install(TARGETS
  camera_node
  lidar_node
  DESTINATION lib/${PROJECT_NAME}
)

# Python 설치
find_package(ament_cmake_python REQUIRED)
ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  sensor_cam_mini/flaskapp.py
  sensor_cam_mini/MultiActuator.py
  sensor_cam_mini/PythonLibMightyZap_PC.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()

