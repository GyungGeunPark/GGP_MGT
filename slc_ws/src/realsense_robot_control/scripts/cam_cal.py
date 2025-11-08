#!/usr/bin/env python3

import numpy as np
import cv2
import glob
import os
import yaml

class CameraCalibrator:
    def __init__(self):
        self.camera_matrix = None
        self.dist_coeffs = None
        self.calibration_error = None
        
    def calibrate_with_chessboard(self, image_paths, board_size=(4, 6), square_size=32.0):
        """체스보드를 사용한 카메라 캘리브레이션
        
        Args:
            image_paths (list): 캘리브레이션 이미지 경로 리스트
            board_size (tuple): 체스보드 내부 코너 수 (가로, 세로)
            square_size (float): 체스보드 한 칸의 크기 (mm)
        """
        # 3D 점 좌표 생성
        objp = np.zeros((board_size[0] * board_size[1], 3), np.float32)
        objp[:, :2] = np.mgrid[0:board_size[0], 0:board_size[1]].T.reshape(-1, 2)
        objp = objp * square_size

        # 캘리브레이션 데이터 저장용 리스트
        objpoints = []  # 3D 점 좌표
        imgpoints = []  # 2D 점 좌표

        for fname in image_paths:
            # 이미지 읽기
            img = cv2.imread(fname)
            if img is None:
                print(f"이미지를 읽을 수 없습니다: {fname}")
                continue

            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            
            # 체스보드 코너 검출
            ret, corners = cv2.findChessboardCorners(
                gray, 
                board_size,
                flags=cv2.CALIB_CB_ADAPTIVE_THRESH + 
                      cv2.CALIB_CB_NORMALIZE_IMAGE + 
                      cv2.CALIB_CB_FAST_CHECK
            )

            if ret:
                objpoints.append(objp)
                
                # 코너 위치 정밀화
                criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
                corners2 = cv2.cornerSubPix(
                    gray,
                    corners,
                    (11, 11),
                    (-1, -1),
                    criteria
                )
                imgpoints.append(corners2)
                print(f"{fname} 코너 검출 완료: {len(imgpoints)}")
                # 검출된 코너 시각화
                #cv2.drawChessboardCorners(img, board_size, corners2, ret)
                #cv2.imshow('Corners', img)
                #cv2.waitKey(500)
            else:
                print(f"{fname} 코너 검출 실패") 

        #cv2.destroyAllWindows()

        if len(objpoints) > 0:
            # 카메라 캘리브레이션 수행
            ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
                objpoints, imgpoints, gray.shape[::-1], None, None
            )

            # 재투영 오차 계산
            total_error = 0
            for i in range(len(objpoints)):
                imgpoints2, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], mtx, dist)
                error = cv2.norm(imgpoints[i], imgpoints2, cv2.NORM_L2)/len(imgpoints2)
                total_error += error
            
            self.calibration_error = total_error/len(objpoints)
            self.camera_matrix = mtx
            self.dist_coeffs = dist
            
            print("\n캘리브레이션 완료")
            print("카메라 행렬:\n", mtx)
            print("왜곡 계수:\n", dist)
            print(f"평균 재투영 오차: {self.calibration_error}")
            
            return True
        
        print("코너를 충분히 검출하지 못했습니다.")
        return False

    def calibrate_with_charuco(self, 
                             image_paths,
                             squares_x=5, 
                             squares_y=7,
                             square_length=32.0,
                             marker_length=24.0):
        """ChArUco 보드를 사용한 카메라 캘리브레이션
        
        Args:
            image_paths (list): 캘리브레이션 이미지 경로 리스트
            squares_x (int): ChArUco 보드의 가로 방향 사각형 개수
            squares_y (int): ChArUco 보드의 세로 방향 사각형 개수
            square_length (float): 사각형 한 변의 길이 (mm)
            marker_length (float): ArUco 마커 한 변의 길이 (mm)
        """
        # ArUco 사전 및 보드 설정
        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        board = cv2.aruco.CharucoBoard(
            (squares_x, squares_y),
            float(square_length),
            float(marker_length),
            aruco_dict
        )

        all_corners = []
        all_ids = []
        image_size = None

        for fname in image_paths:
            img = cv2.imread(fname)
            if img is None:
                print(f"이미지를 읽을 수 없습니다: {fname}")
                continue

            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            image_size = gray.shape

            # ArUco 마커 검출
            detector = cv2.aruco.ArucoDetector(aruco_dict)
            marker_corners, marker_ids, _ = detector.detectMarkers(gray)

            if len(marker_corners) > 0:
                # ChArUco 코너 검출
                ret, charuco_corners, charuco_ids = cv2.aruco.interpolateCornersCharuco(
                    marker_corners, marker_ids, gray, board
                )

                if ret and charuco_corners is not None and charuco_ids is not None:
                    all_corners.append(charuco_corners)
                    all_ids.append(charuco_ids)

                    # 검출 결과 시각화
                    cv2.aruco.drawDetectedMarkers(img, marker_corners)
                    cv2.aruco.drawDetectedCornersCharuco(img, charuco_corners, charuco_ids)
                    cv2.imshow('Detected Markers', img)
                    cv2.waitKey(500)

        cv2.destroyAllWindows()

        if len(all_corners) > 0:
            # 카메라 캘리브레이션 수행
            ret, mtx, dist, rvecs, tvecs = cv2.aruco.calibrateCameraCharuco(
                all_corners, all_ids, board, image_size, None, None
            )

            self.camera_matrix = mtx
            self.dist_coeffs = dist
            
            print("\n캘리브레이션 완료")
            print("카메라 행렬:\n", mtx)
            print("왜곡 계수:\n", dist)
            
            return True

        print("충분한 코너를 검출하지 못했습니다.")
        return False

    def save_calibration(self, filepath='calibration_data.npz'):
        """캘리브레이션 결과 저장"""
        if self.camera_matrix is None or self.dist_coeffs is None:
            print("저장할 캘리브레이션 데이터가 없습니다.")
            return False
            
        np.savez(filepath, 
                 camera_matrix=self.camera_matrix, 
                 dist_coeffs=self.dist_coeffs,
                 calibration_error=self.calibration_error)
        print(f"캘리브레이션 데이터 저장됨: {filepath}")
        return True

    def load_calibration(self, filepath='calibration_data.npz'):
        """저장된 캘리브레이션 결과 로드"""
        if not os.path.exists(filepath):
            print(f"캘리브레이션 파일을 찾을 수 없습니다: {filepath}")
            return False
            
        data = np.load(filepath)
        self.camera_matrix = data['camera_matrix']
        self.dist_coeffs = data['dist_coeffs']
        self.calibration_error = data.get('calibration_error', None)
        
        print("캘리브레이션 데이터 로드됨")
        print("카메라 행렬:\n", self.camera_matrix)
        print("왜곡 계수:\n", self.dist_coeffs)
        if self.calibration_error is not None:
            print(f"캘리브레이션 오차: {self.calibration_error}")
        return True

    def measure_board_distance(self, image, board_size=(4, 6), square_size=32.0):
        """체스보드까지의 거리를 측정하는 함수
        
        Args:
            image: 입력 이미지 (numpy array 또는 이미지 경로)
            board_size (tuple): 체스보드 내부 코너 수 (가로, 세로)
            square_size (float): 체스보드 한 칸의 크기 (mm)
            
        Returns:
            tuple: (거리(mm), 회전 벡터, 이동 벡터) 또는 검출 실패시 None
        """
        if self.camera_matrix is None or self.dist_coeffs is None:
            print("카메라 캘리브레이션이 필요합니다.")
            return None

        # 이미지가 경로로 입력된 경우 로드
        if isinstance(image, str):
            image = cv2.imread(image)
            if image is None:
                print("이미지를 읽을 수 없습니다.")
                return None

        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        
        # 3D 점 좌표 생성
        objp = np.zeros((board_size[0] * board_size[1], 3), np.float32)
        objp[:, :2] = np.mgrid[0:board_size[0], 0:board_size[1]].T.reshape(-1, 2)
        objp = objp * square_size

        # 체스보드 코너 검출
        ret, corners = cv2.findChessboardCorners(
            gray, 
            board_size,
            flags=cv2.CALIB_CB_ADAPTIVE_THRESH + 
                  cv2.CALIB_CB_NORMALIZE_IMAGE + 
                  cv2.CALIB_CB_FAST_CHECK
        )

        if ret:
            # 코너 위치 정밀화
            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
            corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)

            # PnP 문제 해결로 회전과 이동 벡터 추정
            ret, rvec, tvec = cv2.solvePnP(objp, corners2, self.camera_matrix, self.dist_coeffs)

            # 거리 계산 (tvec의 norm)
            distance = np.linalg.norm(tvec)

            # 결과 시각화
            # 좌표축 그리기
            axis_length = square_size * 2
            axis_points = np.float32([[0,0,0], 
                                    [axis_length,0,0], 
                                    [0,axis_length,0], 
                                    [0,0,axis_length]])
            imgpts, _ = cv2.projectPoints(axis_points, rvec, tvec, 
                                        self.camera_matrix, self.dist_coeffs)
            
            # 축 그리기
            origin = tuple(map(int, imgpts[0].ravel()))
            image = cv2.line(image, origin, tuple(map(int, imgpts[1].ravel())), (0,0,255), 3) # X축 (빨강)
            image = cv2.line(image, origin, tuple(map(int, imgpts[2].ravel())), (0,255,0), 3) # Y축 (초록)
            image = cv2.line(image, origin, tuple(map(int, imgpts[3].ravel())), (255,0,0), 3) # Z축 (파랑)

            # 거리 표시
            cv2.putText(image, f'Distance: {distance:.1f}mm', (10, 430), 
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            
            cv2.imshow('Distance Measurement', image)
            cv2.waitKey(0)
            cv2.destroyAllWindows()

            return distance, rvec, tvec
        
        print("체스보드를 검출할 수 없습니다.")
        return None
    def measure_board_pose(self, image, board_size=(4, 6), square_size=32.0):
        """체스보드의 위치와 회전을 측정하는 함수"""
        if self.camera_matrix is None or self.dist_coeffs is None:
            print("카메라 캘리브레이션이 필요합니다.")
            return False, None, None

        # 이미지가 경로로 입력된 경우 로드
        if isinstance(image, str):
            image = cv2.imread(image)
            if image is None:
                print("이미지를 읽을 수 없습니다.")
                return False, None, None

        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        
        # 3D 점 좌표 생성
        objp = np.zeros((board_size[0] * board_size[1], 3), np.float32)
        objp[:, :2] = np.mgrid[0:board_size[0], 0:board_size[1]].T.reshape(-1, 2)
        objp = objp * square_size

        # 체스보드 코너 검출
        ret, corners = cv2.findChessboardCorners(gray, board_size)
        if ret:
            # 코너 위치 정밀화
            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
            corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            
            # PnP로 회전과 이동 벡터 추정
            ret, rvec, tvec = cv2.solvePnP(objp, corners2, self.camera_matrix, self.dist_coeffs)
            return True, rvec, tvec
        
        return False, None, None
    
    def load_camera_params(self, yaml_path='Calibration_result_d455.yaml'):
        """
        카메라 캘리브레이션 YAML 파일에서 파라미터 읽기
        """
        with open(yaml_path, 'r') as f:
            calib_data = yaml.safe_load(f)
        
        # 카메라 매트릭스 문자열을 숫자 리스트로 변환
        camera_matrix = [float(x) for x in calib_data['CameraMatrix'].split(',')]
        distortion_coeffs = [float(x) for x in calib_data['DistortionCoefficient'].split(',')]
        
        camera_params = {
            'focal_length_x': camera_matrix[0],  # fx: 901.7842337379202
            'focal_length_y': camera_matrix[4],  # fy: 902.3262059595946
            'center_x': camera_matrix[2],        # cx: 657.3882862862949
            'center_y': camera_matrix[5],        # cy: 354.373848309395
            'distortion_coeffs': distortion_coeffs  # [k1, k2, p1, p2, k3]
        }
        return camera_params

if __name__ == '__main__':
    # 사용 예시
    calibrator = CameraCalibrator()
    
    # 체스보드 캘리브레이션
    chess_images = glob.glob('cal_data/*.jpg')
    if calibrator.calibrate_with_chessboard(chess_images):
        calibrator.save_calibration('chessboard_calibration.npz')
    
    # ChArUco 캘리브레이션
    #charuco_images = glob.glob('cal_data_ww1/*.jpg')
    #if calibrator.calibrate_with_charuco(charuco_images):
    #    calibrator.save_calibration('charuco_calibration.npz')
    
