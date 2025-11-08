#!/usr/bin/env python3
import os
import sys
sys.path.append(os.path.dirname(os.path.realpath(__file__)))

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import PythonLibMightyZap_PC as MightyZap
import asyncio

# 시리얼 및 기본 세팅
SERIAL_PORT = '/dev/ttyUSB0'
BAUD_RATE = 57600

# 예: 카메라 이름별로 제어해야 할 액추에이터 ID 목록
# Robot_Local 카메라는 액추에이터 ID=1, Gantry_Global1 카메라는 액추에이터 ID=2 라고 가정
CAMERA_ACTUATOR_MAP = {
    'Robot_Local': [1],
    'Gantry_Global1': [2],
    'Gantry_Global2': [3]
}

# 열림/닫힘 위치(각 액추에이터에 동일하다고 가정)
OPEN_POSITIONS =  [3670]  # "open" 시 목표
CLOSED_POSITIONS = [0]     # "close" 시 목표

class CommandSubscriber(Node):
    def __init__(self):
        super().__init__('actuator_controller')

        # 카메라별 현재 열림/닫힘 상태를 기록 (None은 아직 상태 모름)
        # 예: {'Robot_Local': True, 'Gantry_Global1': False, ...}
        self.camera_states = {}
        # 카메라별 현재 이동 중인지 여부
        # 예: {'Robot_Local': False, 'Gantry_Global1': True, ...}
        self.camera_moving = {}

        # perc/cover 토픽(String) 구독
        # 예: "Robot_Local:open" 또는 "Gantry_Global1:close"
        self.cam_cover_sub = self.create_subscription(
            String,
            'perc/cover',
            self.cam_cover_cb,
            10
        )

        # MightyZap 초기화
        self.initialize_mighty_zap()

    def initialize_mighty_zap(self):
        """MightyZap 초기화 및 액츄에이터 비활성화"""
        MightyZap.OpenMightyZap(SERIAL_PORT, BAUD_RATE)
        # 혹은 카메라에 관계없이 쓰일 수 있는 모든 액추에이터를 미리 disable
        # 여기서는 1~10 범위를 사용하거나, 필요한 범위만 disable해도 됨
        for cam_name, actuator_ids in CAMERA_ACTUATOR_MAP.items():
            for aid in actuator_ids:
                MightyZap.ForceEnable(aid, 0)

        print("==========================================================", flush=True)
        print("SYSTEM START", flush=True)

    def cam_cover_cb(self, msg: String):
        """
        카메라 커버 명령 콜백 함수 (String)
        예: "Robot_Local:open" or "Gantry_Global1:close"
        """
        raw_str = msg.data.strip()
        print("==========================================================")
        print(f"[cam_cover_cb] Received : {raw_str}")
        print("==========================================================")

        splitted = raw_str.split(":")
        if len(splitted) != 2:
            print("Invalid message format. Expected 'CameraName:open' or 'CameraName:close'")
            return

        camera_name, command = splitted[0], splitted[1].lower()

        # 매핑된 액추에이터가 있는지 확인
        if camera_name not in CAMERA_ACTUATOR_MAP:
            print(f"No actuator mapping for camera: {camera_name}")
            return

        # 열기/닫기 구분
        if command == "open":
            desired_flag = True
        elif command == "close":
            desired_flag = False
        else:
            print(f"Unknown command: {command}")
            return

        # 현재 카메라의 상태 가져오기 (없으면 None)
        current_state = self.camera_states.get(camera_name, None)

        # 만약 이동 중이면 override
        if self.camera_moving.get(camera_name, False):
            print(f"Camera [{camera_name}] is MOVING. Forcing override!")
            # 강제 override를 위해, 일단 ForceEnable=0으로 중단
            for aid in CAMERA_ACTUATOR_MAP[camera_name]:
                MightyZap.ForceEnable(aid, 0)
            # (실제로는 이전 이동을 asyncio.Task 취소하는 로직도 필요하지만, 간단 예시)

        # 상태가 이미 같다면 움직일 필요 없음
        if current_state == desired_flag:
            print(f"Camera [{camera_name}] is already in the desired state ({command}). No action taken.")
            return

        # 이동 시작
        self.camera_moving[camera_name] = True
        self.camera_states[camera_name] = desired_flag

        # 목표 위치 계산
        target_positions = self.calculate_target_positions(desired_flag, camera_name)
        
        # 비동기로 이동
        asyncio.run(self.move_actuators(camera_name, target_positions))

        # 이동 끝
        self.camera_moving[camera_name] = False

    def calculate_target_positions(self, is_open: bool, camera_name: str):
        """
        열림/닫힘 상태에 따라 목표 위치 계산
        카메라별로 다른 위치가 필요하다면 여기서 분기해도 됨.
        """
        actuator_ids = CAMERA_ACTUATOR_MAP[camera_name]
        if is_open:
            # 만약 여러 액추에이터가 있으면, OPEN_POSITIONS의 길이와 match 시키거나
            # 모든 액추에이터에 동일한 위치를 할당하는 로직을 작성
            if len(actuator_ids) == len(OPEN_POSITIONS):
                return OPEN_POSITIONS
            else:
                # 예: 액추에이터가 2개인데 OPEN_POSITIONS가 1개인 경우 → 동일 위치 할당
                return [OPEN_POSITIONS[0]] * len(actuator_ids)
        else:
            if len(actuator_ids) == len(CLOSED_POSITIONS):
                return CLOSED_POSITIONS
            else:
                return [CLOSED_POSITIONS[0]] * len(actuator_ids)

    async def move_actuators(self, camera_name: str, target_positions):
        """
        액추에이터를 목표 위치로 비동기 이동
        카메라에 매핑된 모든 액추에이터를 순회
        """
        tasks = []
        actuator_ids = CAMERA_ACTUATOR_MAP[camera_name]
        for aid, tpos in zip(actuator_ids, target_positions):
            self.get_logger().info(f"[{camera_name}] actuator={aid}, position command={tpos}")
            MightyZap.GoalPosition(aid, tpos)
            tasks.append(self.wait_for_position(camera_name, aid, tpos))

        # 모든 액추에이터 이동이 끝날 때까지 비동기로 기다림
        await asyncio.gather(*tasks)

    async def wait_for_position(self, camera_name: str, actuator_id: int, target_position: int):
        """
        지정한 위치에 도달할 때까지 비동기 대기
        간단히 4.5초 기다린 뒤 ForceEnable=0 처리
        """
        await asyncio.sleep(4.5)
        # 강제 비활성화
        MightyZap.ForceEnable(actuator_id, 0)
        self.get_logger().info(
            f"[{camera_name}] Actuator {actuator_id} reached position: {MightyZap.PresentPosition(actuator_id)}"
        )

def main(args=None):
    rclpy.init(args=args)
    commandSubscriber = CommandSubscriber()
    
    try:
        rclpy.spin(commandSubscriber)
    except Exception as e:
        print(f"Exception occurred: {e}")
    finally:
        # 종료 시 모든 액추에이터 비활성화
        for cam_name, actuator_ids in CAMERA_ACTUATOR_MAP.items():
            for aid in actuator_ids:
                MightyZap.ForceEnable(aid, 0)
        MightyZap.CloseMightyZap()
        commandSubscriber.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
