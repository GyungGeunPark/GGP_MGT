# #!/usr/bin/env python3
# import socket
# import struct
# import threading
# import sys  # sys 모듈 추가

# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import String

# import cv2
# import numpy as np
# from flask import Flask, Response

# app = Flask(__name__)

# # ─────────────────────────────────────────────────────────
# # 전역 변수들 (영상 & 명령 상태)
# # ─────────────────────────────────────────────────────────
# latest_frame = None
# lock = threading.Lock()

# # 예: rec_on/off, save_on/off 등 플래그
# rec_flag = False
# save_flag = False
# detect_flag = False
# max_frame_count = 0

# # 실제로 프레임 저장 시 사용할 폴더 경로 예시 (원하면 변경)
# SAVE_DIR = "/tmp/flaskapp_capture/"

# # ─────────────────────────────────────────────────────────
# # Flask 라우트: / & /video_feed
# # ─────────────────────────────────────────────────────────
# @app.route('/')
# def index():
#     return "TCP-based streaming + ROS2 commands. Go to /video_feed"

# @app.route('/video_feed')
# def video_feed():
#     """
#     웹 브라우저에서 /video_feed 경로로 접속 시,
#     latest_frame을 MJPEG 형태로 실시간 스트리밍.
#     """
#     def gen():
#         while True:
#             lock.acquire()
#             frame_copy = None if (latest_frame is None) else latest_frame.copy()
#             lock.release()

#             if frame_copy is not None:
#                 ret, buffer = cv2.imencode('.jpg', frame_copy)
#                 if ret:
#                     yield (b'--frame\r\n'
#                            b'Content-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')
#             else:
#                 # 프레임이 없으면 잠시 대기
#                 cv2.waitKey(30)
#     return Response(gen(), mimetype='multipart/x-mixed-replace; boundary=frame')

# # ─────────────────────────────────────────────────────────
# # TCP 서버 스레드 (C++ 프로그램이 connect해서 JPEG 바이트 전송)
# # ─────────────────────────────────────────────────────────
# def tcp_server_thread(port=9100):
#     """
#     C++ 카메라 노드(camera_tcp_streamer.cpp)가 이 서버(예: <JetsonIP>:9100)
#     로 연결하여, JPEG 바이트를 전송해 주면, Flask에서 MJPEG로 스트리밍.
#     """
#     import os
#     if not os.path.exists(SAVE_DIR):
#         os.makedirs(SAVE_DIR, exist_ok=True)

#     host = '0.0.0.0'
    
#     print(f"[TCP SERVER] Listening on {host}:{port}")
    
#     server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
#     server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    
#     # 수신 버퍼 사이즈 증가 (최적화)
#     server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 262144)  # 256KB
    
#     server_sock.bind((host, port))
#     server_sock.listen(1)

#     frame_counter = 0  # 저장용

#     while True:
#         conn, addr = server_sock.accept()
#         print(f"[TCP SERVER] Client connected from: {addr}")

#         try:
#             while True:
#                 # 1) 이미지 크기(4바이트 int) 수신
#                 header = recv_all(conn, 4)
#                 if not header:
#                     print("[TCP SERVER] Connection closed (no header).")
#                     break

#                 frame_size = struct.unpack('i', header)[0]
#                 if frame_size <= 0 or frame_size > 100000000:
#                     print("[TCP SERVER] Invalid frame size:", frame_size)
#                     break

#                 # 2) JPEG 데이터 수신
#                 jpg_data = recv_all(conn, frame_size)
#                 if not jpg_data:
#                     print("[TCP SERVER] Connection closed (no frame data).")
#                     break

#                 # 3) JPEG → OpenCV Mat 복원
#                 arr = np.frombuffer(jpg_data, dtype=np.uint8)
#                 frame = cv2.imdecode(arr, cv2.IMREAD_COLOR)

#                 # 4) 최신 프레임 업데이트
#                 lock.acquire()
#                 global latest_frame
#                 latest_frame = frame
#                 lock.release()

#                 # 5) rec_flag / save_flag 등이 True 면 디스크 저장
#                 if rec_flag or save_flag:
#                     frame_counter += 1
#                     filename = f"{SAVE_DIR}/cap_{frame_counter:06d}.jpg"
#                     cv2.imwrite(filename, frame)
#                     # 혹은 detect_flag True 일 때, detect 로직 등 구현 가능
#                     # ...
#         except Exception as e:
#             print("[TCP SERVER] Exception:", e)
#         finally:
#             conn.close()
#             print("[TCP SERVER] Client disconnected.")

# def recv_all(sock, length):
#     """
#     length만큼 정확히 recv() 해서 바이트 배열을 리턴.
#     연결 끊김 등으로 length만큼 못 받으면 None
#     """
#     data = b''
#     while len(data) < length:
#         chunk = sock.recv(length - len(data))
#         if not chunk:
#             return None
#         data += chunk
#     return data

# # ─────────────────────────────────────────────────────────
# # ROS2 노드 정의: /cam_command 구독, 필요 시 /report(예: topicName) Publish
# # ─────────────────────────────────────────────────────────
# class FlaskRosNode(Node):
#     def __init__(self):
#         super().__init__("flask_ros_node")

#         # 인터페이스 코드에서 publish하는 /cam_command(string) 구독
#         self.cmd_sub = self.create_subscription(
#             String,
#             "/cam_command",
#             self.command_callback,
#             10
#         )

#         # 인터페이스 코드가 구독할 "topicName" (예: "/imgpro_report")에 Publish
#         # 실제 topicName은 상황에 맞게 수정
#         self.report_pub = self.create_publisher(
#             String,
#             "/imgpro_report",
#             10
#         )

#     def command_callback(self, msg):
#         global rec_flag, save_flag, detect_flag, max_frame_count
#         cmdstr = msg.data  # 예: "KA1_CAM:rec_on", "KA1_CAM:rec_off", ...
#         self.get_logger().info(f"Received command: {cmdstr}")

#         # 단순 파싱 예: "채널이름:명령[:추가파라미터]"
#         parts = cmdstr.split(":")
#         if len(parts) < 2:
#             return

#         ch_name = parts[0]      # 예: KA1_CAM
#         command = parts[1]      # 예: rec_on, save_on, etc
#         param = ""
#         if len(parts) > 2:
#             param = parts[2]    # MAX_FRAME, READY 등이면 추가 값이 있을 수도

#         # 명령에 따라 전역 플래그 On/Off
#         # (필요하다면 여기서 실제 로직 구현)
#         if command == "rec_on":
#             rec_flag = True
#             self.publish_report(f"EXEC: rec_on (channel={ch_name})")

#         elif command == "rec_off":
#             rec_flag = False
#             self.publish_report(f"EXEC: rec_off (channel={ch_name})")

#         elif command == "save_on" or command == "img_save_on" or command == "pc_save_on":
#             save_flag = True
#             self.publish_report(f"EXEC: save_on (channel={ch_name})")

#         elif command == "save_off":
#             save_flag = False
#             self.publish_report(f"EXEC: save_off (channel={ch_name})")

#         elif command == "detect_on":
#             detect_flag = True
#             self.publish_report(f"EXEC: detect_on (channel={ch_name})")

#         elif command == "detect_off":
#             detect_flag = False
#             self.publish_report(f"EXEC: detect_off (channel={ch_name})")

#         elif command == "MAX_FRAME":
#             # param 이 frame 개수라고 가정
#             if param.isdigit():
#                 max_frame_count = int(param)
#             self.publish_report(f"EXEC: MAX_FRAME={max_frame_count} (channel={ch_name})")

#         elif command == "READY":
#             # param 사용 예시
#             self.publish_report(f"EXEC: READY param={param} (channel={ch_name})")

#         elif command == "long_ready":
#             self.publish_report(f"EXEC: long_ready (channel={ch_name})")

#         elif command == "start_long":
#             self.publish_report(f"EXEC: start_long (channel={ch_name})")

#         elif command == "stop_long":
#             self.publish_report(f"EXEC: stop_long (channel={ch_name})")

#         # ... 필요하면 mv_save, mv_stop 등 추가

#         else:
#             self.publish_report(f"EXEC: unknown_cmd={command}")

#     def publish_report(self, msg_text: str):
#         """
#         인터페이스 코드의 `report_callback`에서 수신하도록
#         report_pub로 메시지 전송. (topicName="/imgpro_report" 가정)
#         """
#         msg = String()
#         msg.data = msg_text
#         self.report_pub.publish(msg)
#         self.get_logger().info(f"Publish report: {msg_text}")

# # ─────────────────────────────────────────────────────────
# # 메인 함수
# # ─────────────────────────────────────────────────────────
# def main():
#     # 커맨드 라인 인자로 포트 받기
#     port = 5000  # 웹 서버 기본 포트
#     tcp_port = 9100  # TCP 서버 기본 포트
  
#     if len(sys.argv) > 1:
#         try:
#             tcp_port = int(sys.argv[1])
#         except ValueError:
#             print(f"Invalid port number: {sys.argv[1]}, using default: {tcp_port}")
    
#     print(f"Web server will run on port {port}, TCP server on port {tcp_port}")
    
#     # (1) 먼저 ROS2 Node + 스레드를 띄운다.
#     rclpy.init()
#     node = FlaskRosNode()

#     # spin 스레드
#     ros_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
#     ros_thread.start()

#     # (2) TCP 서버 스레드로 영상 수신 - 포트 전달
#     t = threading.Thread(target=lambda: tcp_server_thread(tcp_port), daemon=True)
#     t.start()

#     # (3) Flask 웹 서버 실행
#     #     * 브라우저에서 http://<JetsonIP>:5000/video_feed
#     app.run(host='0.0.0.0', port=port, debug=False, threaded=True)

#     # 종료 처리
#     node.get_logger().info("Flask app stopped. Shutting down rclpy.")
#     rclpy.shutdown()
#     ros_thread.join()

# if __name__ == '__main__':
#     main()