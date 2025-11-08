cmake_minimum_required(VERSION 3.8)
project(sensor_cam_main)

# C++ 표준 설정
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

# ROS2 패키지
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core highgui imgproc videoio)
find_package(pcl_conversions REQUIRED)

# Python 노드용 빌드 설정
find_package(ament_cmake_python REQUIRED)

# Python 패키지 설치
ament_python_install_package(${PROJECT_NAME})

# Python 실행 파일(노드) 설정 (mainwindow.py)
install(PROGRAMS
  sensor_cam_main/mainwindow.py
  sensor_cam_main/detection.py
  sensor_cam_main/d2p.py
  sensor_cam_main/cam_cal.py
  sensor_cam_main/depth_utils.py
  DESTINATION lib/${PROJECT_NAME}
)

# Launch 파일 설치
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# 모델 파일 설치
install(DIRECTORY models/
  DESTINATION share/${PROJECT_NAME}/models
)

# 캘리브레이션 파일 설치
install(DIRECTORY calibration/
  DESTINATION share/${PROJECT_NAME}/calibration
)

ament_package()
