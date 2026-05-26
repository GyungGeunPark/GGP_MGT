#!/usr/bin/env python3
"""
JRT U81 LDS ROS2 제어 노드
──────────────────────────────────────────────────
perc/lds 토픽 하나로 명령 수신 및 응답 발행.

명령 (서버 → 로봇):  {"cmd": "power_on"}  등
응답 (로봇 → 서버):  {"type": "ack", ...}  등

실행:
  python3 main.py
  또는 ROS2 런치에서 실행
"""

import json
import threading
import logging
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from lds_controller import LDSController

logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s [%(levelname)s] %(name)s: %(message)s',
    datefmt='%H:%M:%S',
)
logger = logging.getLogger(__name__)


class LDSNode(Node):
    """LDS 센서 ROS2 브릿지 노드 — 시리얼 ↔ perc/lds 토픽"""

    def __init__(self):
        super().__init__('lds_controller_node')

        # LDS 센서 초기화
        self.lds = LDSController(port='/dev/ttyLDS', baudrate=19200)
        if self.lds.connect():
            self.get_logger().info('LDS 센서 연결 완료')
        else:
            self.get_logger().error(
                'LDS 센서 연결 실패! /dev/ttyLDS 확인 필요. '
                '노드는 계속 실행됩니다.'
            )

        # perc/lds 토픽 — 단일 토픽으로 명령/응답 처리
        self.publisher = self.create_publisher(String, 'perc/lds', 10)
        self.subscription = self.create_subscription(
            String, 'perc/lds', self.topic_callback, 10)

        # 주기적 상태 발행 (2초 간격)
        self.status_timer = self.create_timer(2.0, self.publish_status)

        self.get_logger().info('LDS 제어 노드 시작 — perc/lds 토픽 사용')

    # ── 토픽 수신 ─────────────────────────────────────────────────────

    def topic_callback(self, msg):
        """토픽 메시지 수신 — cmd 필드가 있는 명령만 처리"""
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return

        # 응답 메시지(type 필드가 있는)는 무시 — 자기가 보낸 것
        if 'type' in data:
            return

        cmd = data.get('cmd', '').strip()
        if not cmd:
            return

        self.get_logger().info(f'명령 수신: {cmd}')

        # 시리얼 통신은 블로킹이므로 별도 스레드에서 처리
        threading.Thread(
            target=self._dispatch, args=(data,), daemon=True
        ).start()

    # ── 명령 라우팅 ───────────────────────────────────────────────────

    def _dispatch(self, data):
        """수신된 명령을 LDSController 메서드로 라우팅"""
        cmd = data.get('cmd', '').strip()

        if not self.lds.is_connected():
            self._publish({
                'type': 'error',
                'message': '센서가 연결되어 있지 않습니다.',
            })
            return

        # ── 레이저 ON ──
        if cmd == 'power_on':
            result = self.lds.power_on()
            self._publish({
                'type': 'ack', 'cmd': cmd,
                'success': result['success'],
                'message': '레이저 ON 완료' if result['success']
                           else result.get('error', ''),
            })

        # ── 레이저 OFF ──
        elif cmd == 'power_off':
            result = self.lds.power_off()
            self._publish({
                'type': 'ack', 'cmd': cmd,
                'success': result['success'],
                'message': '레이저 OFF 완료' if result['success']
                           else result.get('error', ''),
            })

        # ── 순간 측정 ──
        elif cmd == 'measure_once':
            result = self.lds.measure_once()
            if result.get('success'):
                self._publish({'type': 'distance', **result})
            else:
                self._publish({
                    'type': 'error',
                    'message': result.get('error', '측정 실패'),
                })

        # ── 연속 측정 시작 ──
        elif cmd == 'continuous_on':
            result = self.lds.start_continuous(self._continuous_callback)
            self._publish({
                'type': 'ack', 'cmd': cmd,
                'success': result['success'],
                'message': '연속 측정 시작' if result['success']
                           else result.get('error', ''),
            })

        # ── 연속 측정 중지 ──
        elif cmd == 'continuous_off':
            result = self.lds.stop_continuous()
            self._publish({
                'type': 'ack', 'cmd': cmd,
                'success': result['success'],
                'message': '연속 측정 중지' if result['success']
                           else result.get('error', ''),
            })

        # ── 측정 모드 변경 ──
        elif cmd == 'set_mode':
            mode = data.get('mode', 'slow')
            result = self.lds.set_measure_mode(mode)
            self._publish({
                'type': 'ack', 'cmd': cmd,
                'success': result['success'],
                'message': f'측정 모드: {mode}' if result['success']
                           else result.get('error', ''),
            })

        # ── 공급 전압 읽기 ──
        elif cmd == 'read_voltage':
            result = self.lds.read_voltage()
            self._publish({
                'type': 'voltage',
                'success': result['success'],
                'message': f"{result.get('voltage_v', 0):.3f} V"
                           if result['success']
                           else result.get('error', ''),
                'voltage_v': result.get('voltage_v'),
                'voltage_mv': result.get('voltage_mv'),
            })

        else:
            self._publish({
                'type': 'error',
                'message': f'알 수 없는 명령: {cmd}',
            })

        # 상태 변경 명령 후 즉시 상태 브로드캐스트
        if cmd in ('power_on', 'power_off', 'continuous_on',
                    'continuous_off', 'set_mode'):
            time.sleep(0.1)  # 상태 반영 대기
            self.publish_status()

    # ── 연속 측정 콜백 ────────────────────────────────────────────────

    def _continuous_callback(self, result):
        """연속 측정 결과를 토픽으로 발행"""
        self._publish({'type': 'distance', **result})

    # ── 상태 발행 ────────────────────────────────────────────────────

    def publish_status(self):
        """현재 센서 상태를 토픽으로 주기적 발행"""
        status = self.lds.get_status()
        self._publish({'type': 'status', **status})

    # ── 발행 헬퍼 ────────────────────────────────────────────────────

    def _publish(self, payload):
        """JSON dict를 std_msgs/String으로 발행"""
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False, default=str)
        self.publisher.publish(msg)

    # ── 종료 ─────────────────────────────────────────────────────────

    def destroy_node(self):
        self.get_logger().info('LDS 노드 종료 중...')
        self.lds.disconnect()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LDSNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
