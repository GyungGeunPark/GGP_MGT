# sensor_cam_main/__init__.py

import rclpy
import logging

# 패키지 전역 로거 설정
logger = logging.getLogger('sensor_cam_main')
logger.setLevel(logging.INFO)

# 필요 시 다른 모듈에서 사용될 상수나 함수 정의
# 예: YOLO 모델 경로, 기본 토픽 이름 등
# LIDAR_TOPIC = '/livox/lidar'


# 패키지 초기화 시 필요한 로직
# 여기에 복잡한 초기화 과정을 넣을 수도 있으나, 
# 일반적으로 ROS 노드 초기화는 개별 노드에서 수행한다.
