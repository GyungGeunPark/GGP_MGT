#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import sys
import math
import cv2
import time
import datetime
import threading
import subprocess
import struct
from threading import Semaphore

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2

from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

# Flask
from flask import Flask, request, render_template_string, jsonify, send_from_directory

app = Flask(__name__)

def save_pcd(points, filename):
    """
    points: Nx3 형태의 float 좌표 (list of [x, y, z])
    filename: 저장할 PCD 파일 경로
    """
    num_points = len(points)
    with open(filename, 'w') as f:
        # 헤더 작성
        f.write("# .PCD v0.7 - Point Cloud Data file format\n")
        f.write("FIELDS x y z\n")
        f.write("SIZE 4 4 4\n")
        f.write("TYPE F F F\n")
        f.write("COUNT 1 1 1\n")
        f.write(f"WIDTH {num_points}\n")
        f.write("HEIGHT 1\n")
        f.write("VIEWPOINT 0 0 0 1 0 0 0\n")
        f.write(f"POINTS {num_points}\n")
        f.write("DATA ascii\n")
        
        # 배치 쓰기로 성능 개선 - 한 번에 여러 줄을 버퍼에 모아서 쓰기
        batch_size = 10000
        buffer = []
        for i, p in enumerate(points):
            buffer.append(f"{p[0]} {p[1]} {p[2]}\n")
            if (i + 1) % batch_size == 0:
                f.writelines(buffer)
                buffer = []
        # 남은 버퍼 쓰기
        if buffer:
            f.writelines(buffer)

    if num_points > 0:
        print(f"[save_pcd] Saved {num_points} points to {filename}")
    else:
        print(f"[save_pcd] Created empty PCD (0 points) at {filename}")

class CameraManager:
    def __init__(self):
        # ROS 초기화
        rclpy.init()
        self.node = rclpy.create_node('imgpro_node')

        # QoS 설정
        qos_profile_sensor_data = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        # 퍼블리셔
        self.command_publisher = self.node.create_publisher(String, "/cam_command", 10)
        self.cover_publisher = self.node.create_publisher(String, "perc/cover", 10)
        self.camera_control_publisher = self.node.create_publisher(String, "/camera_control", 10)

        # 상태 구독자들 - 각 카메라별 상태 구독
        self.camera_status_subscribers = {
            'Robot_Local': self.node.create_subscription(
                String, "/camera_status/robot_local", 
                lambda msg: self.camera_status_callback(msg, 'Robot_Local'), 10
            ),
            'Gantry_Global1': self.node.create_subscription(
                String, "/camera_status/gantry_global1", 
                lambda msg: self.camera_status_callback(msg, 'Gantry_Global1'), 10
            ),
            'Gantry_Global2': self.node.create_subscription(
                String, "/camera_status/gantry_global2", 
                lambda msg: self.camera_status_callback(msg, 'Gantry_Global2'), 10
            ),
            'Gantry_Global3': self.node.create_subscription(
                String, "/camera_status/gantry_global3", 
                lambda msg: self.camera_status_callback(msg, 'Gantry_Global3'), 10
            ),
            'Gantry_Global4': self.node.create_subscription(
                String, "/camera_status/gantry_global4", 
                lambda msg: self.camera_status_callback(msg, 'Gantry_Global4'), 10
            )
        }

        # 구독자
        self.pcl_collection_threads = {}
        self.pcl_collection_locks = {}  # 각 카메라별 PCL 수집 락 추가
        self.pcd_parse_semaphore = Semaphore(2)  # 동시 PCD 파싱을 2개로 제한
        self.report_subscriber = self.node.create_subscription(
            String, "/camera_topic", self.report_callback, 10
        )

        # 스레드 종료 플래그
        self.shutdown_requested = False

        # 카메라별 캡처 상태
        self.camera_caps = {}
        self.camera_threads = {}
        self.cap_locks = {}
        
        # 각 카메라별 상태 관리
        self.camera_states = {
            'Robot_Local': {
                'cap': None,
                'is_recording_images': False,
                'is_recording_video': False,
                'is_collecting_pcl': False,
                'pcl_points': [],
                'pcl_files_collected': 0,
                'cover_state': 'closed',
                'auto_exposure': 'Off',
                'exposure_value': 10000.0,
                'gain_value': 6.0,
                'last_saved_img_path': '',
                'last_saved_pcd_path': '',
                'last_detection_path': ''
            },
            'Gantry_Global1': {
                'cap': None,
                'is_recording_images': False,
                'is_recording_video': False,
                'is_collecting_pcl': False,
                'pcl_points': [],
                'pcl_files_collected': 0,
                'cover_state': 'closed',
                'auto_exposure': 'Off',
                'exposure_value': 10000.0,
                'gain_value': 6.0,
                'last_saved_img_path': '',
                'last_saved_pcd_path': '',
                'last_detection_path': ''
            },
            'Gantry_Global2': {
                'cap': None,
                'is_recording_images': False,
                'is_recording_video': False,
                'is_collecting_pcl': False,
                'pcl_points': [],
                'pcl_files_collected': 0,
                'cover_state': 'closed',
                'auto_exposure': 'Off',
                'exposure_value': 10000.0,
                'gain_value': 6.0,
                'last_saved_img_path': '',
                'last_saved_pcd_path': '',
                'last_detection_path': ''
            },
            'Gantry_Global3': {
                'cap': None,
                'is_recording_images': False,
                'is_recording_video': False,
                'is_collecting_pcl': False,
                'pcl_points': [],
                'pcl_files_collected': 0,
                'cover_state': 'closed',
                'auto_exposure': 'Off',
                'exposure_value': 10000.0,
                'gain_value': 6.0,
                'last_saved_img_path': '',
                'last_saved_pcd_path': '',
                'last_detection_path': ''
            },
            'Gantry_Global4': {
                'cap': None,
                'is_recording_images': False,
                'is_recording_video': False,
                'is_collecting_pcl': False,
                'pcl_points': [],
                'pcl_files_collected': 0,
                'cover_state': 'closed',
                'auto_exposure': 'Off',
                'exposure_value': 10000.0,
                'gain_value': 6.0,
                'last_saved_img_path': '',
                'last_saved_pcd_path': '',
                'last_detection_path': ''
            }
        }
        
        # 시스템 상태 관리 - 각 카메라별 상태
        self.system_status = {
            'Robot_Local': {'status': 'off', 'last_update': 0, 'error_msg': ''},
            'Gantry_Global1': {'status': 'off', 'last_update': 0, 'error_msg': ''},
            'Gantry_Global2': {'status': 'off', 'last_update': 0, 'error_msg': ''},
            'Gantry_Global3': {'status': 'off', 'last_update': 0, 'error_msg': ''},
            'Gantry_Global4': {'status': 'off', 'last_update': 0, 'error_msg': ''}
        }
        self.status_lock = threading.Lock()
        self.STATUS_TIMEOUT = 5

        # 로그 및 기타
        self.weights = "sample.weights"
        self.log_messages_perc = []

        # 경로 설정
        to_date = datetime.datetime.now().strftime("%Y%m%d")
        self.saveLogPath = '/root/sam_ws/worklog/' + to_date + '/'
        self.saveImgPath = '/root/sam_ws/imgData/' + to_date + '/'
        self.logFileName = 'imgpro.log'
        self.createFolder()

        # 카메라 & 라이다 채널 매핑
        self.ch_dict = {
            'Robot_Local': {
                'camera_url': "http://192.168.0.150:9100",
                'lidar_folder': os.path.expanduser("~/sam_ws/storage/lidar_img1"),
                'width': 4000,
                'height': 4000,
                'fps': 1
            },
            'Gantry_Global1': {
                'camera_url': "http://192.168.0.151:9101",
                # [FIXED] 경로의 이중 슬래시(//)를 수정
                'lidar_folder': os.path.expanduser("~/sam_ws/storage/lidar_img2"),
                'width': 4000,
                'height': 4000,
                'fps': 1
            },
            'Gantry_Global2': {
                'camera_url': "http://192.168.0.152:9102",
                'lidar_folder': os.path.expanduser("~/sam_ws/storage/lidar_img3"),
                'width': 1200,
                'height': 1200,
                'fps': 1
            },
            'Gantry_Global3': {
                'camera_url': "http://192.168.0.153:9103",
                # [FIXED] 경로의 이중 슬래시(//)를 수정
                'lidar_folder': os.path.expanduser("~/sam_ws/storage/lidar_img4"),
                'width': 4000,
                'height': 4000,
                'fps': 1
            },
            'Gantry_Global4': {
                'camera_url': "http://192.168.0.154:9104",
                # [FIXED] 경로의 이중 슬래시(//)를 수정
                'lidar_folder': os.path.expanduser("~/sam_ws/storage/lidar_img5"),
                'width': 4000,
                'height': 4000,
                'fps': 1
            }
        }

        # 모든 카메라 초기화
        self.initialize_all_cameras()

        # ROS spin 스레드
        self.ros_thread = threading.Thread(target=self.spin, daemon=True)
        self.ros_thread.start()

        # 상태 체크 스레드
        self.status_check_thread = threading.Thread(target=self.status_check_loop, daemon=True)
        self.status_check_thread.start()

        self.writeLog("[CameraManager] Initialized.")

    def initialize_all_cameras(self):
        """모든 카메라 초기화"""
        for cam_name in self.ch_dict.keys():
            # 각 카메라별 락 생성
            self.cap_locks[cam_name] = threading.Lock()
            self.pcl_collection_locks[cam_name] = threading.Lock()  # PCL 수집 락 추가
            
            # 카메라 연결
            cam_url = self.ch_dict[cam_name]['camera_url']
            cap = cv2.VideoCapture(cam_url)
            if cap.isOpened():
                self.camera_states[cam_name]['cap'] = cap
                self.writeLog(f"[initialize_all_cameras] Camera {cam_name} opened successfully")
            else:
                cap.release()
                self.writeLog(f"[initialize_all_cameras] Failed to open camera {cam_name}")
            
            # 각 카메라별 캡처 스레드 시작
            thread = threading.Thread(target=self.capture_loop, args=(cam_name,), daemon=True)
            thread.start()
            self.camera_threads[cam_name] = thread

    def capture_loop(self, cam_name):
        """각 카메라별 캡처 루프"""
        last_frame_time = 0
        while not self.shutdown_requested:
            with self.cap_locks[cam_name]:
                cap = self.camera_states[cam_name]['cap']
                if cap is not None:
                    ret, frame = cap.read()
                else:
                    ret, frame = False, None

            if ret and frame is not None:
                # 프레임을 성공적으로 읽었을 때 상태 업데이트
                current_time = time.time()
                if current_time - last_frame_time > 0.5:
                    with self.status_lock:
                        self.system_status[cam_name]['status'] = 'normal'
                        self.system_status[cam_name]['last_update'] = current_time
                        self.system_status[cam_name]['error_msg'] = ''
                    last_frame_time = current_time
                
                # 캡처 모드 처리
                if self.camera_states[cam_name]['is_recording_images']:
                    timestr = datetime.datetime.now().strftime("%Y%m%d_%H%M%S_%f")
                    filename = os.path.join(self.saveImgPath, f"{cam_name}_capture_{timestr}.jpg")
                    cv2.imwrite(filename, frame)
                    
                    self.camera_states[cam_name]['last_saved_img_path'] = filename
                    self.camera_states[cam_name]['is_recording_images'] = False
                    self.writeLog(f"Image captured for {cam_name}: {filename}")
            else:
                time.sleep(0.05)
                continue

            time.sleep(0.05)

        # 종료 시 카메라 정리
        with self.cap_locks[cam_name]:
            if self.camera_states[cam_name]['cap'] is not None:
                self.camera_states[cam_name]['cap'].release()
                self.camera_states[cam_name]['cap'] = None
        self.writeLog(f"[CameraManager] Capture thread for {cam_name} terminated.")

    def collect_latest_pcl_files(self, cam_name):
        """최근 PCD 파일들을 수집하는 대체 방법"""
        source_folder = self.ch_dict[cam_name].get('lidar_folder')
        if not os.path.isdir(source_folder):
            self.writeLog(f"[{cam_name}] PCL Error: Source folder not found: {source_folder}")
            return []
        
        try:
            # PCD 파일들만 필터링
            pcd_files = [f for f in os.listdir(source_folder) if f.endswith('.pcd')]
            if not pcd_files:
                self.writeLog(f"[{cam_name}] No PCD files found in {source_folder}")
                return []
            
            # 파일을 수정 시간 기준으로 정렬 (최신 파일이 먼저)
            pcd_files_with_time = []
            for file in pcd_files:
                filepath = os.path.join(source_folder, file)
                try:
                    mtime = os.path.getmtime(filepath)
                    pcd_files_with_time.append((file, filepath, mtime))
                except:
                    continue
            
            pcd_files_with_time.sort(key=lambda x: x[2], reverse=True)
            
            # 필요한 개수만큼 최신 파일 선택
            target_count = 1 if cam_name == 'Robot_Local' else 5
            selected_files = pcd_files_with_time[:target_count]
            
            self.writeLog(f"[{cam_name}] Selected {len(selected_files)} most recent PCD files")
            
            # 선택된 파일들의 포인트 수집
            all_points = []
            for file_name, filepath, mtime in selected_files:
                self.writeLog(f"[{cam_name}] Processing: {file_name}")
                # fallback 모드에서도 세마포어 사용
                with self.pcd_parse_semaphore:
                    points = self.parse_point_cloud_file(filepath, cam_name)
                if points:
                    all_points.extend(points)
                    self.writeLog(f"[{cam_name}] Added {len(points)} points from {file_name}")
            
            return all_points
            
        except Exception as e:
            self.writeLog(f"[{cam_name}] Error in collect_latest_pcl_files: {e}")
            return []
    
    def parse_point_cloud_file(self, filepath, cam_name=None):
        """
        포인트 클라우드 파일(txt 또는 ascii/binary pcd)을 파싱합니다.
        cam_name: 카메라 이름 (로깅용, 선택적)
        """
        points = []
        try:
            with open(filepath, 'rb') as f:
                header = {}
                while True:
                    line_bytes = f.readline()
                    line = line_bytes.decode('ascii', errors='ignore').strip()
                    
                    if line.startswith('DATA'):
                        header['DATA'] = line.split(' ')[1]
                        break
                    
                    parts = line.split(' ')
                    if len(parts) > 1:
                        header[parts[0]] = parts[1:]
                
                if not all(k in header for k in ['WIDTH', 'HEIGHT', 'DATA']):
                    self.writeLog(f"[parse_point_cloud_file] Invalid PCD header in {filepath}")
                    return []

                num_points = int(header['WIDTH'][0]) * int(header['HEIGHT'][0])

                if header['DATA'] == 'ascii':
                    self.writeLog(f"[{filepath}] Parsing as ASCII PCD.")
                    # ASCII PCD 파일 파싱 최적화 - 큰 청크로 읽기
                    points = []
                    chunk_size = 1024 * 1024  # 1MB 단위로 읽기
                    remaining_points = num_points
                    points_read = 0
                    
                    while points_read < num_points:
                        # 큰 청크로 읽기
                        chunk = f.read(chunk_size)
                        if not chunk:
                            break
                        
                        # 마지막 불완전한 줄 처리
                        last_newline = chunk.rfind(b'\n')
                        if last_newline != -1 and last_newline != len(chunk) - 1:
                            # 다음 청크에 연결하기 위해 불완전한 줄 보관
                            f.seek(-(len(chunk) - last_newline - 1), 1)
                            chunk = chunk[:last_newline + 1]
                        
                        # 줄 단위로 분리하여 처리
                        lines = chunk.decode('ascii', errors='ignore').strip().split('\n')
                        
                        for line in lines:
                            if points_read >= num_points:
                                break
                            parts = line.strip().split(' ')
                            if len(parts) >= 3:
                                try:
                                    points.append([float(parts[0]), float(parts[1]), float(parts[2])])
                                    points_read += 1
                                except ValueError:
                                    continue
                        
                        # CPU 양보 (다른 스레드에게 기회 제공)
                        if points_read % 100000 == 0 and points_read > 0:
                            time.sleep(0.001)  # 더 짧은 휴식

                elif header['DATA'] == 'binary':
                    self.writeLog(f"[{filepath}] Parsing as BINARY PCD.")
                    binary_data = f.read(num_points * 12)
                    point_iterator = struct.iter_unpack('<fff', binary_data)
                    points = [list(p) for p in point_iterator]
                
                else:
                    self.writeLog(f"[parse_point_cloud_file] Unsupported DATA type: {header['DATA']} in {filepath}")

        except Exception as e:
            self.writeLog(f"[parse_point_cloud_file] Exception while processing {filepath}: {e}")

        return points

    # [MODIFIED] 포인트 클라우드 수집 로직: 파일 크기 안정화 확인 후 병합
    def collect_and_save_single_pcl(self, cam_name):
        """폴더에서 수집 완료된 PCL 파일을 감지하여 병합 후 저장합니다.
        Robot_Local은 1개, 다른 카메라는 5개 파일을 수집합니다."""
        source_folder = self.ch_dict[cam_name].get('lidar_folder')
        if not os.path.isdir(source_folder):
            self.writeLog(f"[{cam_name}] PCL Error: Source folder not found: {source_folder}")
            self.camera_states[cam_name]['is_collecting_pcl'] = False
            return

        # Robot_Local은 1개 파일, 나머지는 5개 파일 수집
        target_count = 1 if cam_name == 'Robot_Local' else 5
        self.writeLog(f"[{cam_name}] Waiting for {target_count} completed PCL file(s) in {source_folder}...")

        # 즉시 사용 가능한 파일이 있는지 확인 (fallback mode)
        use_fallback = False
        collection_start_time = time.time()
        
        try:
            initial_files = set(os.listdir(source_folder))
            self.writeLog(f"[{cam_name}] Initial files in folder: {len(initial_files)} files")
            
            # 파일 수집 시작 시점 기록 (새 파일 판단용)
            initial_pcd_files = {f for f in initial_files if f.endswith('.pcd')}
            files_at_start = {}
            for file in initial_pcd_files:
                filepath = os.path.join(source_folder, file)
                try:
                    files_at_start[file] = os.path.getmtime(filepath)
                except:
                    pass
                    
        except Exception as e:
            self.writeLog(f"[{cam_name}] PCL Error: Cannot list files in {source_folder}: {e}")
            self.camera_states[cam_name]['is_collecting_pcl'] = False
            return

        collected_files = []
        collected_count = 0
        file_size_tracker = {}  # 파일 크기 추적을 위한 딕셔너리
        no_file_timeout = 60  # 60초 동안 새 파일이 없으면 타임아웃 (리소스 경쟁 고려하여 증가)
        last_new_file_time = time.time()
        parsing_in_progress = False  # 파싱 진행 중 플래그
        
        # 초기 대기: 첫 새 파일이 나타날 때까지 최대 25초 대기
        initial_wait_done = False
        
        # is_collecting_pcl 플래그가 True인 동안 target_count 개의 완료된 파일을 찾기 위해 루프
        while self.camera_states[cam_name]['is_collecting_pcl'] and collected_count < target_count:
            try:
                current_files = set(os.listdir(source_folder))
                # PCD 파일만 필터링하고 수집된 파일 제외
                pcd_files = {f for f in current_files if f.endswith('.pcd')}
                
                # 새 파일 감지: 시작 시점 이후 생성되거나 수정된 파일
                new_files = set()
                for file in pcd_files:
                    if file not in collected_files:  # 이미 수집된 파일 제외
                        filepath = os.path.join(source_folder, file)
                        try:
                            file_mtime = os.path.getmtime(filepath)
                            # 수집 시작 이후 생성/수정된 파일이거나, 시작 시점에 없었던 파일
                            if file not in files_at_start or file_mtime > collection_start_time:
                                new_files.add(file)
                                if not initial_wait_done:
                                    initial_wait_done = True
                                    self.writeLog(f"[{cam_name}] First new file detected, starting collection...")
                        except:
                            pass
                
                if len(new_files) > 0:
                    last_new_file_time = time.time()
                    self.writeLog(f"[{cam_name}] Found {len(new_files)} new file(s) to process")

                # 새로운 파일들의 크기를 추적
                for new_file in new_files:
                    filepath = os.path.join(source_folder, new_file)
                    try:
                        current_size = os.path.getsize(filepath)
                        
                        if new_file not in file_size_tracker:
                            # 새 파일 발견, 크기 추적 시작
                            file_size_tracker[new_file] = {
                                'size': current_size,
                                'last_update': time.time(),
                                'stable_count': 0
                            }
                            self.writeLog(f"[{cam_name}] New file detected: {new_file}, monitoring size...")
                        else:
                            # 기존 파일의 크기 변화 확인
                            prev_size = file_size_tracker[new_file]['size']
                            if current_size == prev_size and current_size > 0:  # 크기가 0보다 크고 안정적
                                # 크기가 변하지 않음
                                file_size_tracker[new_file]['stable_count'] += 1
                                
                                # 2초 동안 크기가 안정적이면 수집 완료로 판단
                                if file_size_tracker[new_file]['stable_count'] >= 4:  # 0.5초 간격 * 4 = 2초
                                    if collected_count < target_count:
                                        self.writeLog(f"[{cam_name}] File {collected_count + 1}/{target_count} collection completed: {new_file} (size: {current_size} bytes)")
                                        
                                        # 파일 파싱 및 포인트 추가 (세마포어 사용하여 동시 파싱 제한)
                                        parsing_in_progress = True
                                        with self.pcd_parse_semaphore:
                                            new_points = self.parse_point_cloud_file(filepath, cam_name)
                                        parsing_in_progress = False
                                        last_new_file_time = time.time()  # 파싱 완료 후 타이머 리셋
                                        
                                        if new_points:
                                            self.camera_states[cam_name]['pcl_points'].extend(new_points)
                                            collected_count += 1
                                            self.camera_states[cam_name]['pcl_files_collected'] = collected_count
                                            collected_files.append(new_file)
                                            self.writeLog(f"[{cam_name}] Added {len(new_points)} points from {new_file}. Total: {len(self.camera_states[cam_name]['pcl_points'])} points")
                                        else:
                                            self.writeLog(f"[{cam_name}] No points parsed from {new_file}.")
                                        
                                        # 처리된 파일은 추적에서 제거
                                        del file_size_tracker[new_file]
                            else:
                                # 크기가 변함 - 아직 수집 중
                                file_size_tracker[new_file]['size'] = current_size
                                file_size_tracker[new_file]['stable_count'] = 0
                                file_size_tracker[new_file]['last_update'] = time.time()
                    
                    except OSError as e:
                        self.writeLog(f"[{cam_name}] Error checking file {new_file}: {e}")
                        continue
                
                # 오래된 추적 항목 정리 (30초 이상 업데이트 없음)
                current_time = time.time()
                for file_name in list(file_size_tracker.keys()):
                    if current_time - file_size_tracker[file_name]['last_update'] > 30:
                        self.writeLog(f"[{cam_name}] Removing stale file from tracking: {file_name}")
                        del file_size_tracker[file_name]
                
                # 타임아웃 체크 - 파싱 중이 아닐 때만 체크
                if not parsing_in_progress and time.time() - last_new_file_time > no_file_timeout:
                    self.writeLog(f"[{cam_name}] Timeout: No new files for {no_file_timeout} seconds.")
                    
                    # Fallback: 기존 파일들 중 최신 파일을 사용
                    if collected_count == 0:
                        self.writeLog(f"[{cam_name}] Using fallback mode: collecting latest existing files")
                        fallback_points = self.collect_latest_pcl_files(cam_name)
                        if fallback_points:
                            self.camera_states[cam_name]['pcl_points'] = fallback_points
                            collected_count = target_count  # 수집 완료로 표시
                            self.writeLog(f"[{cam_name}] Fallback mode collected {len(fallback_points)} points")
                    elif collected_count < target_count:
                        # 일부만 수집된 경우, 부족한 만큼 기존 파일에서 보충
                        self.writeLog(f"[{cam_name}] Partial collection: {collected_count}/{target_count} files. Using fallback for remaining files.")
                        remaining = target_count - collected_count
                        fallback_points = self.collect_latest_pcl_files(cam_name)
                        if fallback_points:
                            # 이미 수집된 포인트에 추가
                            self.camera_states[cam_name]['pcl_points'].extend(fallback_points[:remaining * 10000])  # 대략적인 포인트 수 제한
                            collected_count = target_count
                            self.writeLog(f"[{cam_name}] Added {len(fallback_points)} fallback points")
                    break
                    
            except Exception as e:
                self.writeLog(f"[{cam_name}] PCL Error scanning directory: {e}")
                break

            time.sleep(0.5) # 0.5초마다 폴더 확인

        # --- 저장 로직 ---
        if collected_count == target_count:
            timestr = datetime.datetime.now().strftime("%Y%m%d_%H%M%S_%f")
            pcd_filename = os.path.join(self.saveImgPath, f"{cam_name}_pcl_merged_{timestr}.pcd")
            
            save_pcd(self.camera_states[cam_name]['pcl_points'], pcd_filename)
            
            point_count = len(self.camera_states[cam_name]['pcl_points'])
            msg = f"PCD merged and saved for [{cam_name}]: {collected_count} files, {point_count} total points -> {pcd_filename}"
            self.writeLog(msg)
            
            self.camera_states[cam_name]['last_saved_pcd_path'] = pcd_filename
        else:
            self.writeLog(f"[{cam_name}] PCL collection stopped with {collected_count}/{target_count} files collected (cancelled).")

        # --- 정리 ---
        self.camera_states[cam_name]['is_collecting_pcl'] = False
        self.camera_states[cam_name]['pcl_points'] = []
        self.camera_states[cam_name]['pcl_files_collected'] = 0
        if cam_name in self.pcl_collection_threads:
            del self.pcl_collection_threads[cam_name]

        self.writeLog(f"[{cam_name}] PCL collection process finished.")

    def spin(self):
        while rclpy.ok() and not self.shutdown_requested:
            rclpy.spin_once(self.node, timeout_sec=0.1)
        self.node.destroy_node()
        rclpy.shutdown()
        self.writeLog("[CameraManager] ROS spin thread terminated.")

    def camera_status_callback(self, msg, cam_name):
        with self.status_lock:
            if msg.data.startswith("error:"):
                self.system_status[cam_name]['status'] = 'error'
                self.system_status[cam_name]['error_msg'] = msg.data[6:]
            else:
                self.system_status[cam_name]['status'] = msg.data
                self.system_status[cam_name]['error_msg'] = ''
            self.system_status[cam_name]['last_update'] = time.time()

    def status_check_loop(self):
        while not self.shutdown_requested:
            current_time = time.time()
            with self.status_lock:
                for cam_name, info in self.system_status.items():
                    if current_time - info['last_update'] > self.STATUS_TIMEOUT:
                        if info['status'] != 'off':
                            info['status'] = 'off'
                            self.writeLog(f"[StatusCheck] {cam_name} status changed to OFF (timeout)")
            time.sleep(1)

    def report_callback(self, msg):
        self.writeLog("[camera_topic] " + msg.data)

    def createFolder(self):
        if not os.path.exists(self.saveLogPath):
            os.makedirs(self.saveLogPath, exist_ok=True)
        if not os.path.exists(self.saveImgPath):
            os.makedirs(self.saveImgPath, exist_ok=True)
        now = time.localtime()
        str_time = '%02d%02d%02d' % (now.tm_hour, now.tm_min, now.tm_sec)
        self.logFileName = os.path.join(self.saveLogPath, f'imgpro_{str_time}.log')
        with open(self.logFileName, 'w') as file:
            file.write('LOAD start log \n')

    def writeLog(self, txt):
        with open(self.logFileName, 'a') as file:
            now = datetime.datetime.now()
            timestamp = now.strftime("%H:%M:%S.%f")
            file.write(timestamp + ' ' + txt + '\n')
        now_str = datetime.datetime.now().strftime("%H:%M:%S")
        self.log_messages_perc.append(f"[{now_str}] {txt}")
        if len(self.log_messages_perc) > 200:
            self.log_messages_perc = self.log_messages_perc[-200:]

    def get_system_status(self):
        with self.status_lock:
            status = {}
            for cam_name in self.system_status.keys():
                status[cam_name] = {
                    'status': self.system_status[cam_name]['status'],
                    'error_msg': self.system_status[cam_name].get('error_msg', '')
                }
            return status

    def img_save(self, cam_name):
        if cam_name not in self.camera_states:
            msg = f"Invalid camera: {cam_name}"
            self.writeLog(msg)
            return {"message": msg, "file_path": ""}
        
        self.camera_states[cam_name]['is_recording_images'] = True
        
        wait_count = 0
        max_wait = 20
        while self.camera_states[cam_name]['is_recording_images'] and wait_count < max_wait:
            time.sleep(0.1)
            wait_count += 1
        
        if self.camera_states[cam_name]['last_saved_img_path']:
            msg = f"이미지 저장됨 [{cam_name}]: {self.camera_states[cam_name]['last_saved_img_path']}"
            self.writeLog(msg)
            relative_path = os.path.relpath(self.camera_states[cam_name]['last_saved_img_path'], self.saveImgPath)
            return {"message": msg, "file_path": relative_path, "camera": cam_name}
        else:
            msg = f"이미지 저장 실패 [{cam_name}]"
            self.writeLog(msg)
            return {"message": msg, "file_path": "", "camera": cam_name}

    def img_save_all(self):
        results = {}
        for cam_name in self.camera_states.keys():
            results[cam_name] = self.img_save(cam_name)
        return results

    # [MODIFIED] PCL 수집 시작 함수 수정 - 스레드 안전성 강화
    def pc_save_start(self, cam_name):
        """특정 카메라 포인트클라우드 수집 시작
        Robot_Local: 1개 파일 즉시 저장
        기타 카메라: 5개 파일 병합 후 저장"""
        if cam_name not in self.camera_states:
            msg = f"Invalid camera: {cam_name}"
            self.writeLog(msg)
            return {"message": msg, "is_collecting": False}
        
        # 스레드 안전성을 위한 락 사용
        with self.pcl_collection_locks[cam_name]:
            if self.camera_states[cam_name]['is_collecting_pcl']:
                msg = f"이미 수집 중 [{cam_name}]"
                self.writeLog(msg)
                return {"message": msg, "is_collecting": True}
        
            source_folder = self.ch_dict[cam_name].get('lidar_folder')
            if not os.path.isdir(source_folder):
                msg = f"LiDAR source folder not found for {cam_name}: {source_folder}"
                self.writeLog(msg)
                return {"message": msg, "is_collecting": False}

            # 인덴테이션 수정 - 락 내부로 이동
            self.camera_states[cam_name]['is_collecting_pcl'] = True
            self.camera_states[cam_name]['pcl_points'] = []
            self.camera_states[cam_name]['pcl_files_collected'] = 0  # 수집된 파일 개수 추적
            self.camera_states[cam_name]['last_saved_pcd_path'] = '' # 이전 경로 초기화
            
            # 새 로직을 수행하는 스레드 시작
            thread = threading.Thread(target=self.collect_and_save_single_pcl, args=(cam_name,), daemon=True)
            thread.start()
            self.pcl_collection_threads[cam_name] = thread

        # Robot_Local은 1개, 나머지는 5개 파일 대기
        target_count = 1 if cam_name == 'Robot_Local' else 5
        msg = f"포인트클라우드 수집 시작 [{cam_name}] - {target_count}개 파일 대기 중..."
        self.writeLog(msg)
        return {"message": msg, "is_collecting": True, "camera": cam_name}

    def pc_save_start_all(self):
        """모든 카메라의 포인트클라우드 수집을 순차적으로 시작하여 리소스 경쟁 방지"""
        results = {}
        threads = []
        
        # Robot_Local이 없으면 제외하고 순서 조정
        camera_order = []
        if 'Robot_Local' in self.camera_states:
            camera_order.append('Robot_Local')
        for cam_name in ['Gantry_Global2', 'Gantry_Global1', 'Gantry_Global3', 'Gantry_Global4']:
            if cam_name in self.camera_states:
                camera_order.append(cam_name)
        
        # 각 카메라에 대해 순차적으로 수집 시작 (지연 추가)
        for idx, cam_name in enumerate(camera_order):
            # 각 카메라 수집을 별도 스레드에서 시작하되, 스태거링 적용
            thread = threading.Thread(
                target=lambda cn=cam_name, delay=idx*2.0: (
                    time.sleep(delay),  # 각 카메라 시작 시 2초씩 지연
                    results.update({cn: self.pc_save_start(cn)})
                ),
                daemon=True
            )
            threads.append(thread)
            thread.start()
        
        # 모든 스레드가 시작될 때까지 대기
        for thread in threads:
            thread.join(timeout=20.0)  # 충분한 대기 시간 제공
        
        self.writeLog("모든 카메라 PCL 수집 시작 요청 완료")
        return results

    # [MODIFIED] PCL 수집 중지 함수는 이제 '취소' 역할 - 스레드 안전성 강화
    def pc_save_stop(self, cam_name):
        """특정 카메라 포인트클라우드 수집을 취소합니다."""
        if cam_name not in self.camera_states:
            msg = f"Invalid camera: {cam_name}"
            self.writeLog(msg)
            return {"message": msg, "is_collecting": False, "file_path": ""}
        
        with self.pcl_collection_locks[cam_name]:
            if self.camera_states[cam_name]['is_collecting_pcl']:
                self.writeLog(f"Cancelling PCL collection for [{cam_name}]...")
                self.camera_states[cam_name]['is_collecting_pcl'] = False
                
                if cam_name in self.pcl_collection_threads:
                    self.pcl_collection_threads[cam_name].join(timeout=3.0)
                    if cam_name in self.pcl_collection_threads:
                        del self.pcl_collection_threads[cam_name]

                msg = f"PCL collection cancelled by user [{cam_name}]"
                self.writeLog(msg)
                
                return {"message": msg, "is_collecting": False, "file_path": "", "camera": cam_name}
            else:
                msg = f"수집 중이 아님 [{cam_name}]"
                self.writeLog(msg)
                return {"message": msg, "is_collecting": False, "file_path": "", "camera": cam_name}

    def pc_save_stop_all(self):
        """모든 카메라의 포인트클라우드 수집을 동시에 중지"""
        results = {}
        threads = []
        
        # 모든 카메라에 대해 동시에 수집 중지
        for cam_name in self.camera_states.keys():
            thread = threading.Thread(
                target=lambda cn=cam_name: results.update({cn: self.pc_save_stop(cn)}),
                daemon=True
            )
            threads.append(thread)
            thread.start()
        
        # 모든 스레드가 완료될 때까지 대기
        for thread in threads:
            thread.join(timeout=1.0)
        
        self.writeLog("모든 카메라 PCL 수집 중지 요청 완료")
        return results

    # [MODIFIED] PCL 상태 반환 함수에 마지막 저장 경로 및 수집 진행상황 추가
    def get_pcl_status(self, cam_name):
        """특정 카메라 포인트클라우드 수집 상태와 마지막 저장 경로 반환"""
        if cam_name not in self.camera_states:
            return {"is_collecting": False, "point_count": 0, "last_saved_pcd_path": "", "files_collected": 0}
        
        state = self.camera_states[cam_name]
        relative_path = ""
        if state['last_saved_pcd_path']:
             relative_path = os.path.relpath(state['last_saved_pcd_path'], self.saveImgPath)

        return {
            "is_collecting": state['is_collecting_pcl'],
            "point_count": len(state['pcl_points']) if state['is_collecting_pcl'] else 0,
            "camera": cam_name,
            "last_saved_pcd_path": relative_path,
            "files_collected": state.get('pcl_files_collected', 0)  # 수집된 파일 개수 반환
        }

    def get_pcl_status_all(self):
        results = {}
        for cam_name in self.camera_states.keys():
            results[cam_name] = self.get_pcl_status(cam_name)
        return results

    def detection(self, cam_name):
        if cam_name not in self.camera_states:
            msg = f"Invalid camera: {cam_name}"
            self.writeLog(msg)
            return {"message": msg, "camera": cam_name}
        
        def capture_with_gain_zero():
            control_msg = String()
            control_msg.data = f"{cam_name}:set_gain:0.0"
            self.camera_control_publisher.publish(control_msg)
            self.writeLog(f"Gain 변경 요청 [{cam_name}]: set_gain:0.0")
            
            time.sleep(2.0)
            
            with self.cap_locks[cam_name]:
                cap = self.camera_states[cam_name]['cap']
                if cap is None:
                    self.writeLog(f"카메라 초기화 안됨 [{cam_name}]")
                    return
                ret, frame = cap.read()
                if not ret or frame is None:
                    self.writeLog(f"이미지 캡처 실패 [{cam_name}]")
                    return
                
                timestr = datetime.datetime.now().strftime("%Y%m%d_%H%M%S_%f")
                filename = os.path.join(self.saveImgPath, f"{cam_name}_detection_{timestr}.jpg")
                cv2.imwrite(filename, frame)
                self.camera_states[cam_name]['last_detection_path'] = filename
                self.writeLog(f"Detection 이미지 캡처됨 (gain=0) [{cam_name}]: {filename}")
            
            time.sleep(0.5)
            reset_msg = String()
            reset_msg.data = f"{cam_name}:set_gain:{self.camera_states[cam_name]['gain_value']}"
            self.camera_control_publisher.publish(reset_msg)
            self.writeLog(f"Gain 복구 [{cam_name}]: {self.camera_states[cam_name]['gain_value']}")
        
        capture_thread = threading.Thread(target=capture_with_gain_zero)
        capture_thread.daemon = True
        capture_thread.start()
        
        msg = f"Detection 트리거됨 [{cam_name}]"
        self.writeLog(msg)
        return {"message": msg, "camera": cam_name}

    def detection_all(self):
        results = {}
        for cam_name in self.camera_states.keys():
            results[cam_name] = self.detection(cam_name)
        return results

    def open_cover(self, cam_name):
        if cam_name not in self.camera_states:
            msg = f"Invalid camera: {cam_name}"
            self.writeLog(msg)
            return {"message": msg, "state": "unknown", "camera": cam_name}
        
        cmd = f"{cam_name}:open"
        self.cover_publisher.publish(String(data=cmd))
        self.camera_states[cam_name]['cover_state'] = "open"
        msg = f"커버 열림 [{cam_name}]"
        
        self.writeLog(msg)
        return {"message": msg, "state": "open", "camera": cam_name}

    def close_cover_all(self):
        results = {}
        for cam_name in self.camera_states.keys():
            cmd = f"{cam_name}:close"
            self.cover_publisher.publish(String(data=cmd))
            self.camera_states[cam_name]['cover_state'] = "closed"
            results[cam_name] = {"state": "closed", "camera": cam_name}
            self.writeLog(f"커버 닫힘 [{cam_name}]")
        
        return {"message": "모든 카메라 커버 닫힘", "results": results}
    
    def get_cover_status(self):
        status = {}
        for cam_name in self.camera_states.keys():
            status[cam_name] = self.camera_states[cam_name]['cover_state']
        return status

    def start_recording(self, cam_name):
        if cam_name not in self.camera_states:
            msg = f"Invalid camera: {cam_name}"
            self.writeLog(msg)
            return {"message": msg, "camera": cam_name}
        
        if self.camera_states[cam_name]['is_recording_video']:
            msg = f"이미 녹화 중 [{cam_name}]"
            self.writeLog(msg)
            return {"message": msg, "camera": cam_name}
        
        timestr = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        video_dir = os.path.join(self.saveImgPath, f"recording_{timestr}")
        os.makedirs(video_dir, exist_ok=True)
        
        width = self.ch_dict[cam_name].get('width', 1200)
        height = self.ch_dict[cam_name].get('height', 1200)
        fps = self.ch_dict[cam_name].get('fps', 5)
        
        video_path = os.path.join(video_dir, f"{cam_name}.avi")
        fourcc = cv2.VideoWriter_fourcc(*'XVID')
        
        self.camera_states[cam_name]['is_recording_video'] = True
        
        def record_camera():
            cap = self.camera_states[cam_name]['cap']
            if cap is None:
                self.writeLog(f"카메라 없음 [{cam_name}]")
                self.camera_states[cam_name]['is_recording_video'] = False
                return
            
            out = cv2.VideoWriter(video_path, fourcc, fps, (width, height))
            if not out.isOpened():
                self.writeLog(f"VideoWriter 생성 실패 [{cam_name}]")
                self.camera_states[cam_name]['is_recording_video'] = False
                return
            
            frame_count = 0
            start_time = time.time()
            
            while self.camera_states[cam_name]['is_recording_video']:
                with self.cap_locks[cam_name]:
                    ret, frame = cap.read()
                    if ret and frame is not None:
                        if frame.shape[1] != width or frame.shape[0] != height:
                            frame = cv2.resize(frame, (width, height))
                        out.write(frame)
                        frame_count += 1
                
                time.sleep(1.0 / fps)
            
            out.release()
            elapsed = time.time() - start_time
            self.writeLog(f"녹화 완료 [{cam_name}]: {frame_count} frames in {elapsed:.1f} seconds")
        
        thread = threading.Thread(target=record_camera, daemon=True)
        thread.start()
        
        msg = f"녹화 시작 [{cam_name}]: {video_path}"
        self.writeLog(msg)
        return {"message": msg, "camera": cam_name}

    def start_recording_all(self):
        results = {}
        for cam_name in self.camera_states.keys():
            results[cam_name] = self.start_recording(cam_name)
        return results

    def stop_recording(self, cam_name):
        if cam_name not in self.camera_states:
            msg = f"Invalid camera: {cam_name}"
            self.writeLog(msg)
            return {"message": msg, "camera": cam_name}
        
        if not self.camera_states[cam_name]['is_recording_video']:
            msg = f"녹화 중이 아님 [{cam_name}]"
            self.writeLog(msg)
            return {"message": msg, "camera": cam_name}
        
        self.camera_states[cam_name]['is_recording_video'] = False
        msg = f"녹화 중지 요청 [{cam_name}]"
        self.writeLog(msg)
        return {"message": msg, "camera": cam_name}

    def stop_recording_all(self):
        results = {}
        for cam_name in self.camera_states.keys():
            results[cam_name] = self.stop_recording(cam_name)
        return results

iface = CameraManager()
# [MODIFIED] HTML/Javascript 코드 수정
HTML_TEMPLATE = """
<!DOCTYPE html>
<html>
<head>
    <meta charset="utf-8"/>
    <title>인식 프로그램 매니저</title>
    <style>
      body { 
        font-family: 'Malgun Gothic', sans-serif; 
        margin: 20px;
        background-color: #f5f5f5;
      }
      h1 {
        color: #333;
        border-bottom: 3px solid #2196F3;
        padding-bottom: 10px;
      }
      .section {
        background-color: white;
        border: 1px solid #ddd;
        border-radius: 8px;
        padding: 20px;
        margin-bottom: 20px;
        box-shadow: 0 2px 4px rgba(0,0,0,0.1);
      }
      .section-title {
        font-size: 20px;
        font-weight: bold;
        margin-bottom: 15px;
        color: #333;
      }
      .camera-grid {
        display: grid;
        grid-template-columns: repeat(5, 1fr);
        gap: 10px;
        margin-bottom: 15px;
      }
      .camera-column {
        text-align: center;
        padding: 15px;
        background-color: #f8f9fa;
        border: 1px solid #dee2e6;
        border-radius: 5px;
      }
      .camera-name {
        font-weight: bold;
        margin-bottom: 10px;
        color: #495057;
      }
      .all-cameras-row {
        text-align: center;
        padding: 15px;
        background-color: #fffde7;
        border: 2px solid #fbc02d;
        border-radius: 5px;
        margin-bottom: 15px;
      }
      button {
        padding: 8px 16px;
        margin: 3px;
        border: none;
        border-radius: 4px;
        cursor: pointer;
        font-family: 'Malgun Gothic', sans-serif;
        font-size: 14px;
        transition: all 0.3s;
      }
      .btn-primary {
        background-color: #2196F3;
        color: white;
      }
      .btn-primary:hover {
        background-color: #1976D2;
      }
      .btn-primary:disabled {
        background-color: #cccccc;
        cursor: not-allowed;
      }
      .btn-success {
        background-color: #4CAF50;
        color: white;
      }
      .btn-success:hover {
        background-color: #45a049;
      }
      .btn-danger {
        background-color: #f44336;
        color: white;
      }
      .btn-danger:hover {
        background-color: #da190b;
      }
      .btn-warning {
        background-color: #ff9800;
        color: white;
      }
      .btn-warning:hover {
        background-color: #e68900;
      }
      .btn-all {
        background-color: #fbc02d;
        color: #333;
        font-weight: bold;
      }
      .btn-all:hover {
        background-color: #f9a825;
      }
      .status-indicator {
        display: inline-block;
        width: 12px;
        height: 12px;
        border-radius: 50%;
        margin-right: 5px;
      }
      .status-on { background-color: #4CAF50; }
      .status-off { background-color: #f44336; }
      .status-recording { background-color: #ff9800; }
      .file-info {
        font-size: 12px;
        color: #666;
        margin-top: 5px;
        word-break: break-all;
        min-height: 1.2em;
      }
      .detection-results {
        margin-top: 20px;
      }
      .detection-grid {
        display: grid;
        grid-template-columns: repeat(5, 1fr);
        gap: 10px;
      }
      .detection-item {
        text-align: center;
      }
      .detection-item img {
        max-width: 100%;
        height: 200px;
        object-fit: contain;
        border: 1px solid #ddd;
        border-radius: 4px;
      }
      .log-area {
        background-color: #263238;
        color: #aed581;
        padding: 10px;
        height: 200px;
        overflow-y: auto;
        font-family: 'Consolas', monospace;
        font-size: 12px;
        border-radius: 4px;
      }
      .system-status {
        display: grid;
        grid-template-columns: repeat(5, 1fr);
        gap: 10px;
        margin-bottom: 20px;
      }
      .status-item {
        text-align: center;
        padding: 15px;
        border-radius: 5px;
        color: white;
        font-weight: bold;
        font-size: 14px;
      }
      .status-normal { background-color: #4CAF50; }
      .status-error { background-color: #f44336; }
      .status-off { background-color: #9e9e9e; }
    </style>
</head>
<body>
    <h1>인식 프로그램 매니저</h1>

    <div class="system-status">
      <div id="cameraStatus_Robot_Local" class="status-item status-off">
        Robot_Local: <span id="cameraStatusText_Robot_Local">OFF</span>
      </div>
      <div id="cameraStatus_Gantry_Global1" class="status-item status-off">
        Gantry_Global 1: <span id="cameraStatusText_Gantry_Global1">OFF</span>
      </div>
      <div id="cameraStatus_Gantry_Global2" class="status-item status-off">
        Gantry_Global 2: <span id="cameraStatusText_Gantry_Global2">OFF</span>
      </div>
      <div id="cameraStatus_Gantry_Global3" class="status-item status-off">
        Gantry_Global 3: <span id="cameraStatusText_Gantry_Global3">OFF</span>
      </div>
      <div id="cameraStatus_Gantry_Global4" class="status-item status-off">
        Gantry_Global 4: <span id="cameraStatusText_Gantry_Global4">OFF</span>
      </div>
    </div>

    <div class="section">
      <div class="section-title">카메라 커버</div>
      <div class="camera-grid">
        <div class="camera-column">
          <div class="camera-name">Robot_Local</div>
          <button id="coverBtn_Robot_Local" class="btn-primary" onclick="openCover('Robot_Local')">
            <span id="coverStatus_Robot_Local" class="status-indicator status-off"></span>
            열기
          </button>
        </div>
        <div class="camera-column">
          <div class="camera-name">Gantry_Global 1</div>
          <button id="coverBtn_Gantry_Global1" class="btn-primary" onclick="openCover('Gantry_Global1')">
            <span id="coverStatus_Gantry_Global1" class="status-indicator status-off"></span>
            열기
          </button>
        </div>
        <div class="camera-column">
          <div class="camera-name">Gantry_Global 2</div>
          <button id="coverBtn_Gantry_Global2" class="btn-primary" onclick="openCover('Gantry_Global2')">
            <span id="coverStatus_Gantry_Global2" class="status-indicator status-off"></span>
            열기
          </button>
        </div>
        <div class="camera-column">
          <div class="camera-name">Gantry_Global 3</div>
          <button id="coverBtn_Gantry_Global3" class="btn-primary" onclick="openCover('Gantry_Global3')">
            <span id="coverStatus_Gantry_Global3" class="status-indicator status-off"></span>
            열기
          </button>
        </div>
        <div class="camera-column">
          <div class="camera-name">Gantry_Global 4</div>
          <button id="coverBtn_Gantry_Global4" class="btn-primary" onclick="openCover('Gantry_Global4')">
            <span id="coverStatus_Gantry_Global4" class="status-indicator status-off"></span>
            열기
          </button>
        </div>
      </div>
      <div class="all-cameras-row">
        <button class="btn-all" onclick="closeCoverAll()">모든 카메라 커버 닫기</button>
      </div>
    </div>

    <div class="section">
      <div class="section-title">영상 캡처</div>
      <div class="camera-grid">
        <div class="camera-column">
          <div class="camera-name">Robot_Local</div>
          <button class="btn-success" onclick="captureImage('Robot_Local')">저장</button>
          <div id="imgPath_Robot_Local" class="file-info"></div>
        </div>
        <div class="camera-column">
          <div class="camera-name">Gantry_Global 1</div>
          <button class="btn-success" onclick="captureImage('Gantry_Global1')">저장</button>
          <div id="imgPath_Gantry_Global1" class="file-info"></div>
        </div>
        <div class="camera-column">
          <div class="camera-name">Gantry_Global 2</div>
          <button class="btn-success" onclick="captureImage('Gantry_Global2')">저장</button>
          <div id="imgPath_Gantry_Global2" class="file-info"></div>
        </div>
        <div class="camera-column">
          <div class="camera-name">Gantry_Global 3</div>
          <button class="btn-success" onclick="captureImage('Gantry_Global3')">저장</button>
          <div id="imgPath_Gantry_Global3" class="file-info"></div>
        </div>
        <div class="camera-column">
          <div class="camera-name">Gantry_Global 4</div>
          <button class="btn-success" onclick="captureImage('Gantry_Global4')">저장</button>
          <div id="imgPath_Gantry_Global4" class="file-info"></div>
        </div>
      </div>
    </div>

    <div class="section">
      <div class="section-title">포인트 클라우드 수집</div>
      <div class="camera-grid">
        <div class="camera-column">
          <div class="camera-name">Robot_Local</div>
          <button id="pclStart_Robot_Local" class="btn-success" onclick="startPCL('Robot_Local')">시작</button>
          <button id="pclStop_Robot_Local" class="btn-danger" onclick="stopPCL('Robot_Local')" disabled>정지</button>
          <div id="pclStatus_Robot_Local" class="file-info"></div>
          <div id="pclPath_Robot_Local" class="file-info"></div>
        </div>
        <div class="camera-column">
          <div class="camera-name">Gantry_Global 1</div>
          <button id="pclStart_Gantry_Global1" class="btn-success" onclick="startPCL('Gantry_Global1')">시작</button>
          <button id="pclStop_Gantry_Global1" class="btn-danger" onclick="stopPCL('Gantry_Global1')" disabled>정지</button>
          <div id="pclStatus_Gantry_Global1" class="file-info"></div>
          <div id="pclPath_Gantry_Global1" class="file-info"></div>
        </div>
        <div class="camera-column">
          <div class="camera-name">Gantry_Global 2</div>
          <button id="pclStart_Gantry_Global2" class="btn-success" onclick="startPCL('Gantry_Global2')">시작</button>
          <button id="pclStop_Gantry_Global2" class="btn-danger" onclick="stopPCL('Gantry_Global2')" disabled>정지</button>
          <div id="pclStatus_Gantry_Global2" class="file-info"></div>
          <div id="pclPath_Gantry_Global2" class="file-info"></div>
        </div>
        <div class="camera-column">
          <div class="camera-name">Gantry_Global 3</div>
          <button id="pclStart_Gantry_Global3" class="btn-success" onclick="startPCL('Gantry_Global3')">시작</button>
          <button id="pclStop_Gantry_Global3" class="btn-danger" onclick="stopPCL('Gantry_Global3')" disabled>정지</button>
          <div id="pclStatus_Gantry_Global3" class="file-info"></div>
          <div id="pclPath_Gantry_Global3" class="file-info"></div>
        </div>
        <div class="camera-column">
          <div class="camera-name">Gantry_Global 4</div>
          <button id="pclStart_Gantry_Global4" class="btn-success" onclick="startPCL('Gantry_Global4')">시작</button>
          <button id="pclStop_Gantry_Global4" class="btn-danger" onclick="stopPCL('Gantry_Global4')" disabled>정지</button>
          <div id="pclStatus_Gantry_Global4" class="file-info"></div>
          <div id="pclPath_Gantry_Global4" class="file-info"></div>
        </div>
      </div>
      <div class="all-cameras-row">
        <button id="pclStartAll" class="btn-all" onclick="startPCLAll()">모든 카메라 수집 시작</button>
        <button id="pclStopAll" class="btn-danger" onclick="stopPCLAll()" style="display: none;">모든 카메라 수집 정지</button>
      </div>
    </div>

    <div class="section">
      <div class="section-title">녹화</div>
      <div class="all-cameras-row">
        <span id="recordingStatus" style="margin-right: 20px;">
          <span class="status-indicator status-off"></span>NOT RECORDING
        </span>
        <button id="recordStart" class="btn-success" onclick="startRecordingAll()">모든 카메라 녹화 시작</button>
        <button id="recordStop" class="btn-danger" onclick="stopRecordingAll()" disabled>모든 카메라 녹화 정지</button>
      </div>
    </div>

    <div class="section">
      <div class="section-title">카메라 측정</div>
      <div class="all-cameras-row">
        <button class="btn-warning" onclick="detectionAll()">측정</button>
      </div>
      <div class="detection-results">
        <div class="detection-grid">
          <div class="detection-item">
            <div class="camera-name">Local 측정 파일 :</div>
            <img id="detectionImg_Robot_Local" src="" alt="측정 결과 없음">
          </div>
          <div class="detection-item">
            <div class="camera-name">Gantry1 측정 파일:</div>
            <img id="detectionImg_Gantry_Global1" src="" alt="측정 결과 없음">
          </div>
          <div class="detection-item">
            <div class="camera-name">Gantry2 측정 파일 :</div>
            <img id="detectionImg_Gantry_Global2" src="" alt="측정 결과 없음">
          </div>
          <div class="detection-item">
            <div class="camera-name">Gantry3 측정 파일 :</div>
            <img id="detectionImg_Gantry_Global3" src="" alt="측정 결과 없음">
          </div>
          <div class="detection-item">
            <div class="camera-name">Gantry4 측정 파일 :</div>
            <img id="detectionImg_Gantry_Global4" src="" alt="측정 결과 없음">
          </div>
        </div>
      </div>
    </div>

    <div class="section">
      <div class="section-title">로그 메시지</div>
      <div class="log-area" id="log-area">
        {% for line in logs_perc %}
          {{ line }}<br>
        {% endfor %}
      </div>
    </div>

    <script>
      let pclTimers = {};
      let detectionImages = {
        'Robot_Local': '',
        'Gantry_Global1': '',
        'Gantry_Global2': '',
        'Gantry_Global3': '',
        'Gantry_Global4': ''
      };
      
      function updateSystemStatus(status) {
        const cameras = ['Robot_Local', 'Gantry_Global1', 'Gantry_Global2', 'Gantry_Global3', 'Gantry_Global4'];
        for (const cam of cameras) {
          const element = document.getElementById(`cameraStatus_${cam}`);
          const textElement = document.getElementById(`cameraStatusText_${cam}`);
          if (status[cam]) {
            const currentStatus = status[cam].status || 'off';
            element.classList.remove('status-normal', 'status-error', 'status-off');
            if (currentStatus === 'normal') {
              element.classList.add('status-normal');
              textElement.textContent = 'NORMAL';
            } else if (currentStatus === 'error') {
              element.classList.add('status-error');
              textElement.textContent = 'ERROR';
              if (status[cam].error_msg) {
                addLog(`[${cam}] Error: ${status[cam].error_msg}`);
              }
            } else {
              element.classList.add('status-off');
              textElement.textContent = 'OFF';
            }
          }
        }
      }
      
      async function openCover(camName) {
        try {
          const formData = new FormData();
          formData.append('camera', camName);
          const response = await fetch('/open_cover', { method: 'POST', body: formData });
          const data = await response.json();
          updateCoverStatus(camName, data.state);
          addLog(data.message);
        } catch (error) {
          console.error('Error opening cover:', error);
        }
      }
      
      async function closeCoverAll() {
        try {
          const response = await fetch('/close_cover_all', { method: 'POST' });
          const data = await response.json();
          const cameras = ['Robot_Local', 'Gantry_Global1', 'Gantry_Global2', 'Gantry_Global3', 'Gantry_Global4'];
          for (const cam of cameras) {
            updateCoverStatus(cam, 'closed');
          }
          addLog(data.message);
        } catch (error) {
          console.error('Error closing all covers:', error);
        }
      }
      
      function updateCoverStatus(camName, state) {
        const statusElem = document.getElementById(`coverStatus_${camName}`);
        const btnElem = document.getElementById(`coverBtn_${camName}`);
        if (state === 'open') {
          statusElem.classList.remove('status-off');
          statusElem.classList.add('status-on');
          btnElem.disabled = true;
          btnElem.textContent = '열림';
        } else {
          statusElem.classList.remove('status-on');
          statusElem.classList.add('status-off');
          btnElem.disabled = false;
          btnElem.innerHTML = '<span id="coverStatus_' + camName + '" class="status-indicator status-off"></span>열기';
        }
      }
      
      async function captureImage(camName) {
        try {
          const formData = new FormData();
          formData.append('camera', camName);
          const response = await fetch('/img_save', { method: 'POST', body: formData });
          const data = await response.json();
          if (data.file_path) {
            document.getElementById(`imgPath_${camName}`).textContent = `저장됨: ${data.file_path}`;
          }
          addLog(data.message);
        } catch (error) {
          console.error('Error capturing image:', error);
        }
      }
      
      async function startPCL(camName) {
        try {
          document.getElementById(`pclPath_${camName}`).textContent = '';
          document.getElementById(`pclStatus_${camName}`).textContent = '수집 시작중...';
          
          const formData = new FormData();
          formData.append('camera', camName);
          const response = await fetch('/pc_save_start', { method: 'POST', body: formData });
          const data = await response.json();
          
          if (data.is_collecting) {
              document.getElementById(`pclStart_${camName}`).disabled = true;
              document.getElementById(`pclStop_${camName}`).disabled = false;
              startPCLTimer(camName);
          }
          addLog(data.message);
        } catch (error) {
          console.error('Error starting PCL:', error);
          document.getElementById(`pclStatus_${camName}`).textContent = '시작 오류';
        }
      }
      
      async function stopPCL(camName) {
        try {
          const formData = new FormData();
          formData.append('camera', camName);
          const response = await fetch('/pc_save_stop', { method: 'POST', body: formData });
          const data = await response.json();
          
          stopPCLTimer(camName);
          document.getElementById(`pclStart_${camName}`).disabled = false;
          document.getElementById(`pclStop_${camName}`).disabled = true;
          document.getElementById(`pclStatus_${camName}`).textContent = '수집이 취소되었습니다.';
          
          addLog(data.message);
        } catch (error) {
          console.error('Error stopping PCL:', error);
        }
      }
      
      async function startPCLAll() {
        try {
          const response = await fetch('/pc_save_start_all', { method: 'POST' });
          const data = await response.json();
          
          // 각 카메라의 상태 업데이트
          for (const [cam, result] of Object.entries(data)) {
            if (result.is_collecting) {
              document.getElementById(`pclStart_${cam}`).disabled = true;
              document.getElementById(`pclStop_${cam}`).disabled = false;
              document.getElementById(`pclPath_${cam}`).textContent = '';
              document.getElementById(`pclStatus_${cam}`).textContent = '수집 시작중...';
              startPCLTimer(cam);
            }
          }
          
          // 전체 버튼 상태 변경
          document.getElementById('pclStartAll').style.display = 'none';
          document.getElementById('pclStopAll').style.display = 'inline-block';
          
          addLog('모든 카메라 PCL 수집 시작');
        } catch (error) {
          console.error('Error starting all PCL:', error);
        }
      }
      
      async function stopPCLAll() {
        try {
          const response = await fetch('/pc_save_stop_all', { method: 'POST' });
          const data = await response.json();
          
          // 각 카메라의 상태 업데이트
          for (const [cam, result] of Object.entries(data)) {
            stopPCLTimer(cam);
            document.getElementById(`pclStart_${cam}`).disabled = false;
            document.getElementById(`pclStop_${cam}`).disabled = true;
            document.getElementById(`pclStatus_${cam}`).textContent = '수집이 취소되었습니다.';
          }
          
          // 전체 버튼 상태 변경
          document.getElementById('pclStartAll').style.display = 'inline-block';
          document.getElementById('pclStopAll').style.display = 'none';
          
          addLog('모든 카메라 PCL 수집 정지');
        } catch (error) {
          console.error('Error stopping all PCL:', error);
        }
      }
      
      function startPCLTimer(camName) {
        pclTimers[camName] = setInterval(async () => {
          try {
            const response = await fetch(`/pcl_status/${camName}`);
            const data = await response.json();
            
            if (data.is_collecting) {
              const filesCollected = data.files_collected || 0;
              // Robot_Local은 1개, 나머지는 5개 파일
              const targetCount = camName === 'Robot_Local' ? 1 : 5;
              document.getElementById(`pclStatus_${camName}`).textContent = `파일 수집 중... (${filesCollected}/${targetCount})`;
            } else {
              // 수집이 자동으로 완료됨
              stopPCLTimer(camName);
              document.getElementById(`pclStart_${camName}`).disabled = false;
              document.getElementById(`pclStop_${camName}`).disabled = true;
              document.getElementById(`pclStatus_${camName}`).textContent = '';
              
              if (data.last_saved_pcd_path) {
                  document.getElementById(`pclPath_${camName}`).textContent = `저장됨: ${data.last_saved_pcd_path}`;
                  addLog(`[${camName}] PCL 자동 저장 완료: ${data.last_saved_pcd_path}`);
              } else {
                  document.getElementById(`pclPath_${camName}`).textContent = '';
              }
              
              // 모든 타이머가 종료되었는지 확인하여 전체 버튼 상태 업데이트
              checkAllPCLCompleted();
            }
          } catch (error) {
            console.error('Error updating PCL status:', error);
            stopPCLTimer(camName);
            checkAllPCLCompleted();
          }
        }, 1000);
      }
      
      function checkAllPCLCompleted() {
        // 모든 PCL 타이머가 종료되었는지 확인
        if (Object.keys(pclTimers).length === 0) {
          // 모든 수집이 완료됨 - 전체 버튼 상태 복원
          document.getElementById('pclStartAll').style.display = 'inline-block';
          document.getElementById('pclStopAll').style.display = 'none';
        }
      }
      
      function stopPCLTimer(camName) {
        if (pclTimers[camName]) {
          clearInterval(pclTimers[camName]);
          delete pclTimers[camName];
        }
      }
      
      async function startRecordingAll() {
        try {
          const response = await fetch('/start_recording_all', { method: 'POST' });
          const data = await response.json();
          document.getElementById('recordStart').disabled = true;
          document.getElementById('recordStop').disabled = false;
          document.getElementById('recordingStatus').innerHTML = '<span class="status-indicator status-recording"></span>RECORDING';
          addLog('모든 카메라 녹화 시작');
        } catch (error) {
          console.error('Error starting recording:', error);
        }
      }
      
      async function stopRecordingAll() {
        try {
          const response = await fetch('/stop_recording_all', { method: 'POST' });
          const data = await response.json();
          document.getElementById('recordStart').disabled = false;
          document.getElementById('recordStop').disabled = true;
          document.getElementById('recordingStatus').innerHTML = '<span class="status-indicator status-off"></span>NOT RECORDING';
          addLog('모든 카메라 녹화 정지');
        } catch (error) {
          console.error('Error stopping recording:', error);
        }
      }
      
      async function detectionAll() {
        try {
          const response = await fetch('/detection_all', { method: 'POST' });
          const data = await response.json();
          for (const [cam, result] of Object.entries(data)) {
            addLog(result.message);
          }
          setTimeout(updateDetectionImages, 3000);
        } catch (error) {
          console.error('Error running detection:', error);
        }
      }
      
      async function updateDetectionImages() {
        try {
          const response = await fetch('/detection_images');
          const data = await response.json();
          for (const [cam, imagePath] of Object.entries(data)) {
            if (imagePath) {
              const imgElem = document.getElementById(`detectionImg_${cam}`);
              imgElem.src = `/images/${imagePath}?t=${Date.now()}`;
              imgElem.alt = '측정 결과';
            }
          }
        } catch (error) {
          console.error('Error updating detection images:', error);
        }
      }
      
      function addLog(message) {
        const logArea = document.getElementById('log-area');
        const timestamp = new Date().toLocaleTimeString();
        logArea.innerHTML += `[${timestamp}] ${message}<br>`;
        logArea.scrollTop = logArea.scrollHeight;
      }
      
      async function updateLogs() {
        try {
          const response = await fetch('/logs_perc');
          const data = await response.json();
          const logDiv = document.getElementById('log-area');
          const isNearBottom = (logDiv.scrollTop + logDiv.clientHeight >= logDiv.scrollHeight - 10);
          
          logDiv.innerHTML = data.logs_perc.join('<br>');
          
          if (isNearBottom) {
            logDiv.scrollTop = logDiv.scrollHeight;
          }
        } catch (error) {
          // console.error('Error updating logs:', error);
        }
      }
      
      async function fetchSystemStatus() {
        try {
          const response = await fetch('/system_status');
          const data = await response.json();
          updateSystemStatus(data);
        } catch (error) {
          console.error('Error fetching system status:', error);
        }
      }
      
      async function fetchCoverStatus() {
        try {
          const response = await fetch('/cover_status');
          const data = await response.json();
          for (const [cam, state] of Object.entries(data)) {
            updateCoverStatus(cam, state);
          }
        } catch (error) {
          console.error('Error fetching cover status:', error);
        }
      }

      document.addEventListener('DOMContentLoaded', function() {
        setInterval(updateLogs, 1000);
        setInterval(fetchSystemStatus, 2000);
        fetchSystemStatus();
        fetchCoverStatus();
      });
    </script>
</body>
</html>
"""

@app.route('/')
def index():
    return render_template_string(
        HTML_TEMPLATE,
        logs_perc=iface.log_messages_perc
    )

@app.route('/images/<path:filename>')
def serve_image(filename):
    return send_from_directory(iface.saveImgPath, filename)

@app.route('/system_status')
def system_status():
    return jsonify(iface.get_system_status())

@app.route('/logs_perc')
def get_logs():
    return jsonify({'logs_perc': iface.log_messages_perc})

@app.route('/img_save', methods=['POST'])
def route_img_save():
    cam_name = request.form.get('camera', '')
    if cam_name == 'all':
        result = iface.img_save_all()
    else:
        result = iface.img_save(cam_name)
    return jsonify(result)

@app.route('/pc_save_start', methods=['POST'])
def route_pc_save_start():
    cam_name = request.form.get('camera', '')
    if cam_name == 'all':
        result = iface.pc_save_start_all()
    else:
        result = iface.pc_save_start(cam_name)
    return jsonify(result)

@app.route('/pc_save_start_all', methods=['POST'])
def route_pc_save_start_all():
    result = iface.pc_save_start_all()
    return jsonify(result)

@app.route('/pc_save_stop', methods=['POST'])
def route_pc_save_stop():
    cam_name = request.form.get('camera', '')
    if cam_name == 'all':
        result = iface.pc_save_stop_all()
    else:
        result = iface.pc_save_stop(cam_name)
    return jsonify(result)

@app.route('/pc_save_stop_all', methods=['POST'])
def route_pc_save_stop_all():
    result = iface.pc_save_stop_all()
    return jsonify(result)

@app.route('/pcl_status/<cam_name>')
def pcl_status(cam_name):
    return jsonify(iface.get_pcl_status(cam_name))

@app.route('/open_cover', methods=['POST'])
def route_open_cover():
    cam_name = request.form.get('camera', '')
    result = iface.open_cover(cam_name)
    return jsonify(result)

@app.route('/close_cover_all', methods=['POST'])
def route_close_cover_all():
    result = iface.close_cover_all()
    return jsonify(result)

@app.route('/cover_status')
def cover_status():
    return jsonify(iface.get_cover_status())

@app.route('/start_recording_all', methods=['POST'])
def route_start_recording_all():
    result = iface.start_recording_all()
    return jsonify(result)

@app.route('/stop_recording_all', methods=['POST'])
def route_stop_recording_all():
    result = iface.stop_recording_all()
    return jsonify(result)

@app.route('/detection_all', methods=['POST'])
def route_detection_all():
    result = iface.detection_all()
    return jsonify(result)

@app.route('/detection_images')
def detection_images():
    images = {}
    for cam_name in iface.camera_states.keys():
        if iface.camera_states[cam_name]['last_detection_path']:
            relative_path = os.path.relpath(
                iface.camera_states[cam_name]['last_detection_path'], 
                iface.saveImgPath
            )
            images[cam_name] = relative_path
        else:
            images[cam_name] = ''
    return jsonify(images)

def main():
    app.run(host='0.0.0.0', port=5020, debug=False)

if __name__ == '__main__':
    main()