#!/usr/bin/env python3

import numpy as np
import open3d as o3d
import cv2
import argparse
import os
import torch
import gzip
import matplotlib.pyplot as plt

# ROS2에서 패키지 경로를 가져오기 위해 필요
from ament_index_python.packages import get_package_share_directory

# 기존 cam_cal.py 내 CameraCalibrator 클래스를 import 한다고 가정
# 파일명이 cam_cal.py가 맞다면 아래처럼 import 가능
# (단, sensor_cam_main 내에 cam_cal.py가 존재해야 하며 패키지로 인식되어야 함)
from cam_cal import CameraCalibrator
import yaml

class PointCloudGenerator:
    
    def __init__(self, save_path=None, camera_matrix=None, dist_coeffs=None):
        """
        모든 결과물을 저장할 기본 경로를 설정한다.
        만약 save_path가 None이면, './detection/'를 사용한다.
        """
        if save_path is None:   
            self.save_path = './detection/'
        else:
            self.save_path = save_path
            
        # Ensure base save path exists
        os.makedirs(self.save_path, exist_ok=True)
        
        # sensor_cam_main/calibration 폴더 경로를 얻는다
        try:
            share_dir = get_package_share_directory('sensor_cam_main')
            calibration_dir = os.path.join(share_dir, 'calibration')
        except Exception as e:
            print(f"Warning: Could not find sensor_cam_main share directory: {e}")
            calibration_dir = '.'  # fallback to current dir

        # calibration 폴더 내 npz, yaml 파일 경로 지정
        npz_path = os.path.join(calibration_dir, 'chessboard_calibration.npz')
        yaml_path = os.path.join(calibration_dir, 'Calibration_result_d455.yaml')
        
        # Camera calibration
        self.calibrator = CameraCalibrator()
        try:
            self.calibrator.load_calibration(npz_path)
            self.camera_params = self.calibrator.load_camera_params(yaml_path)
            self.calibration_loaded = True
        except Exception as e:
            print(f"Warning: Could not load calibration: {e}")
            self.calibration_loaded = False
            self.camera_params = None
        
        # Default parameters
        self.scale_factor = 2.0
        self.min_depth = 0.1
        self.max_depth = 5.0
        self.z_offset = 2.0
        self.y_offset = 0.7
        
    def calculate_view_angle(self, image_path):
        """Calculate camera view angle using chessboard detection."""
        if not self.calibration_loaded:
            print("Warning: Calibration not loaded, using default angle (3 degrees)")
            return 3.0
            
        try:
            success, rvec, tvec = self.calibrator.measure_board_pose(image_path)
            if not success:
                print("Warning: Could not detect chessboard, using default angle (3 degrees)")
                return 3.0
            
            rmat, _ = cv2.Rodrigues(rvec)
            
            pitch = np.arctan2(-rmat[2,0], np.sqrt(rmat[2,1]**2 + rmat[2,2]**2))
            pitch_deg = np.degrees(pitch)
            
            print(f"Measured pitch angle: {pitch_deg:.2f} degrees")
            return abs(pitch_deg)
            
        except Exception as e:
            print(f"Error in view angle calculation: {e}")
            print("Using default angle (3 degrees)")
            return 3.0

    def create_point_cloud(self, 
                           depth_map: np.ndarray, 
                           image_path: str,
                           real_distance: float = 1000.0,
                           min_depth: float = 0.01,
                           max_depth: float = 10.0,
                           save_name: str = None) -> o3d.geometry.PointCloud:
        """Create a point cloud from depth map and RGB image."""
        print("\nInput Depth Map Statistics:")
        print(f"Shape: {depth_map.shape}")
        print(f"Min value: {np.min(depth_map)}")
        print(f"Max value: {np.max(depth_map)}")
        print(f"Mean value: {np.mean(depth_map)}")
        print(f"Non-zero points: {np.count_nonzero(depth_map)}")
        
        bgr_image = cv2.imread(image_path, cv2.IMREAD_COLOR)
        if bgr_image is None:
            raise FileNotFoundError(f"Could not find image: {image_path}")
        
        if bgr_image.shape[:2] != depth_map.shape:
            print(f"Warning: Image shape {bgr_image.shape[:2]} != depth map shape {depth_map.shape}")
            print("Resizing depth map to match image...")
            depth_map = cv2.resize(depth_map, (bgr_image.shape[1], bgr_image.shape[0]), interpolation=cv2.INTER_LINEAR)
        
        print(f"\nImage shape: {bgr_image.shape}")
        
        height, width = depth_map.shape
        points = []
        colors = []
        
        # Calculate view angle for perspective correction
        view_angle = self.calculate_view_angle(image_path)
        print(f"Calculated view angle: {view_angle:.2f} degrees")
        
        scale_factor_z = 100  # Base or fallback
        if real_distance > 0:
            center_depth = np.median(depth_map[height//3:2*height//3, width//3:2*width//3])
            if center_depth > 0:
                computed_scale = real_distance / center_depth
                if 10 < computed_scale < 1000:
                    scale_factor_z = computed_scale
                    print(f"Computed scale factor from real distance: {scale_factor_z:.2f}")
        
        # 임의 고정값 (사용 환경에 맞춰 조정)
        scale_factor_x = 555
        scale_factor_y = 400
        
        z_values = []
        z_scaled_values = []
        z_adjusted_values = []
        
        # 첫 번째 패스: 깊이값 통계
        for v in range(height):
            for u in range(width):
                if depth_map[v, u] > 0:
                    z = depth_map[v, u]
                    z_values.append(z)
                    
                    z_scaled = z / scale_factor_z
                    z_scaled_values.append(z_scaled)
                    
                    z_adjusted = z_scaled + self.z_offset
                    z_adjusted_values.append(z_adjusted)
        
        if z_values:
            print("\nDepth Processing Statistics:")
            print(f"Original Z - Min: {min(z_values):.3f}, Max: {max(z_values):.3f}, Mean: {np.mean(z_values):.3f}")
            print(f"Scaled Z - Min: {min(z_scaled_values):.3f}, Max: {max(z_scaled_values):.3f}, Mean: {np.mean(z_scaled_values):.3f}")
            print(f"Adjusted Z - Min: {min(z_adjusted_values):.3f}, Max: {max(z_adjusted_values):.3f}, Mean: {np.mean(z_adjusted_values):.3f}")

        if z_adjusted_values:
            adaptive_min = max(0.001, np.percentile(z_adjusted_values, 1))
            adaptive_max = min(20.0, np.percentile(z_adjusted_values, 99))
            print(f"Adaptive depth range: {adaptive_min:.3f}m to {adaptive_max:.3f}m")
        else:
            adaptive_min, adaptive_max = min_depth, max_depth
        
        # 두 번째 패스: 실제 포인트 생성
        tilt_angle = np.radians(view_angle)
        for v in range(height):
            for u in range(width):
                if depth_map[v, u] > 0:
                    z = depth_map[v, u]
                    z_scaled = z / scale_factor_z
                    z_adjusted = z_scaled + self.z_offset
                    if adaptive_min <= z_adjusted <= adaptive_max:
                        x_adjusted = (u - width/2) / scale_factor_x
                        y_adjusted = (v/scale_factor_y - self.y_offset) * np.cos(tilt_angle)
                        
                        points.append([x_adjusted, y_adjusted, z_adjusted])
                        colors.append(bgr_image[v, u] / 255.0)
        
        if len(points) == 0:
            print("\nNo points found!")
            print(f"Z offset: {self.z_offset}")
            print(f"Scale factors - X: {scale_factor_x}, Y: {scale_factor_y}, Z: {scale_factor_z}")
            raise ValueError(f"No points in specified depth range ({adaptive_min}m ~ {adaptive_max}m)")
        
        points = np.array(points, dtype=np.float32)
        colors = np.array(colors, dtype=np.float32)
        
        print(f"\nFinal Results:")
        print(f"Total points: {len(points)}")
        print(f"Depth range: {np.min(points[:, 2]):.3f}m ~ {np.max(points[:, 2]):.3f}m")
        
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)
        pcd.colors = o3d.utility.Vector3dVector(colors)
        
        # -------- 결과물 저장 부분: ./detection 내부에 파일명만으로 저장 --------
        if self.save_path and save_name:
            # 파일명 접두사(prefix) 구성: "./detection/my_image"
            file_prefix = os.path.join(self.save_path, save_name)
            
            # 시각화 이미지 저장
            preview_path = f"{file_prefix}_pointcloud_preview.png"
            self.visualize_point_cloud(pcd, preview_path)
            
            # NPY 저장
            structured_data = np.zeros(len(points), dtype=[
                ('x', '<f4'), ('y', '<f4'), ('z', '<f4'),
                ('red', 'u1'), ('green', 'u1'), ('blue', 'u1')
            ])
            structured_data['x'] = points[:, 0]
            structured_data['y'] = points[:, 1]
            structured_data['z'] = points[:, 2]
            structured_data['red'] = colors[:, 0] * 255
            structured_data['green'] = colors[:, 1] * 255
            structured_data['blue'] = colors[:, 2] * 255
            
            npy_gz_path = f"{file_prefix}.npy.gz"
            with gzip.GzipFile(npy_gz_path, 'w') as f:
                np.save(f, structured_data)
            print(f"Point cloud saved: {npy_gz_path}")
            
            # PLY 저장
            ply_path = f"{file_prefix}.ply"
            o3d.io.write_point_cloud(ply_path, pcd)
            print(f"PLY file saved: {ply_path}")
        
        return pcd

    def calculate_camera_angle_from_intrinsics(self, image_path=None):
        """Estimate camera angle using intrinsic parameters from calibration file."""
        if not hasattr(self, 'camera_params') or not self.calibration_loaded:
            print("Warning: No calibration loaded, using default angle (3 degrees)")
            return 3.0
            
        try:
            if image_path:
                img = cv2.imread(image_path)
                if img is not None:
                    height, width = img.shape[:2]
                else:
                    width, height = 1280, 720
            else:
                width, height = 1280, 720
                
            fx = self.camera_params.get('fx', 1933.51)
            fy = self.camera_params.get('fy', 1950.31)
            cx = self.camera_params.get('cx', 661.44)
            cy = self.camera_params.get('cy', 391.99)
            
            vfov = 2 * np.arctan(height / (2 * fy))
            vfov_degrees = np.degrees(vfov)
            
            hfov = 2 * np.arctan(width / (2 * fx))
            hfov_degrees = np.degrees(hfov)
            
            cy_ratio = (cy / height) - 0.5
            estimated_tilt = 5.0 + (cy_ratio * 20.0)
            
            print(f"Camera Parameters: fx={fx:.1f}, fy={fy:.1f}, cx={cx:.1f}, cy={cy:.1f}")
            print(f"Field of View: vertical={vfov_degrees:.1f}°, horizontal={hfov_degrees:.1f}°")
            print(f"Estimated tilt angle: {estimated_tilt:.2f} degrees")
            
            return abs(estimated_tilt)
        except Exception as e:
            print(f"Error estimating camera angle from intrinsics: {e}")
            return 3.0

    def load_camera_params(self, yaml_file):
        """Load camera parameters from YAML file or from NPY fallback."""
        try:
            with open(yaml_file, 'r') as f:
                params = yaml.safe_load(f)
            return params
        except Exception as e:
            print(f"Could not load YAML file: {e}")
            try:
                camera_matrix = np.load('camera_matrix.npy')
                dist_coeffs = np.load('dist_coeffs.npy')
                
                params = {
                    'fx': float(camera_matrix[0, 0]),
                    'fy': float(camera_matrix[1, 1]),
                    'cx': float(camera_matrix[0, 2]),
                    'cy': float(camera_matrix[1, 2]),
                    'distortion_coefficients': dist_coeffs.tolist()
                }
                return params
            except Exception as e2:
                print(f"Failed to load from NPY files: {e2}")
                return {
                    'fx': 1933.51,
                    'fy': 1950.31,
                    'cx': 661.44,
                    'cy': 391.99,
                    'distortion_coefficients': [[-9.93, 597.35, -0.082, 0.032, -12562.32]]
                }

    def create_wall_point_cloud(self, 
                                depth_map, 
                                image_path, 
                                wall_mask, 
                                save_name=None):
        """Create a point cloud for a wall region using a wall mask."""
        print("\nWall Point Cloud Input Statistics:")
        print(f"Depth map shape: {depth_map.shape}")
        print(f"Wall mask shape: {wall_mask.shape}")
        print(f"Wall mask non-zero points: {np.count_nonzero(wall_mask)}")
        
        if np.count_nonzero(wall_mask) == 0:
            raise ValueError("Wall mask is empty - no wall area detected")
        
        bgr_image = cv2.imread(image_path, cv2.IMREAD_COLOR)
        if bgr_image is None:
            raise FileNotFoundError(f"Could not find image: {image_path}")
        
        height, width = bgr_image.shape[:2]
        
        # Resize mask, depth_map if needed
        if wall_mask.shape != (height, width):
            wall_mask = cv2.resize(wall_mask.astype(np.uint8), (width, height), interpolation=cv2.INTER_NEAREST).astype(bool)
        
        if depth_map.shape != (height, width):
            depth_map = cv2.resize(depth_map, (width, height), interpolation=cv2.INTER_LINEAR)
        
        # included_pixels.png도 ./detection 내에 저장
        if save_name:
            included_pixels_path = os.path.join(self.save_path, f"{save_name}_included_pixels.png")
        else:
            included_pixels_path = os.path.join(self.save_path, 'included_pixels.png')
        
        mask_vis = np.zeros_like(bgr_image)
        for v in range(height):
            for u in range(width):
                if wall_mask[v, u]:
                    mask_vis[v, u] = bgr_image[v, u]
        cv2.imwrite(included_pixels_path, mask_vis)
        
        points = []
        colors = []
        
        # Better camera-based angle
        view_angle = self.calculate_camera_angle_from_intrinsics(image_path)
        tilt_angle = np.radians(view_angle)
        
        if hasattr(self, 'camera_params') and self.camera_params:
            fx = self.camera_params.get('fx', 1933.51)
            fy = self.camera_params.get('fy', 1950.31)
            scale_factor_x = fx / 3.5
            scale_factor_y = fy / 5.0
        else:
            scale_factor_x = 555
            scale_factor_y = 400
        
        scale_factor_z = 100
        z_values = []
        z_scaled_values = []
        z_adjusted_values = []
        
        for v in range(height):
            for u in range(width):
                if depth_map[v, u] > 0 and wall_mask[v, u]:
                    z = depth_map[v, u]
                    z_values.append(z)
                    
                    z_scaled = z / scale_factor_z
                    z_scaled_values.append(z_scaled)
                    
                    z_corrected = z_scaled * np.cos(tilt_angle)
                    z_adjusted = z_corrected + self.z_offset
                    z_adjusted_values.append(z_adjusted)
        
        if z_values:
            print("\nWall Depth Processing Statistics:")
            print(f"Original Z - Min: {min(z_values):.3f}, Max: {max(z_values):.3f}")
            print(f"Scaled Z - Min: {min(z_scaled_values):.3f}, Max: {max(z_scaled_values):.3f}")
            print(f"Adjusted Z - Min: {min(z_adjusted_values):.3f}, Max: {max(z_adjusted_values):.3f}")
            
            wall_z_min = max(0.001, np.percentile(z_adjusted_values, 1))
            wall_z_max = min(20.0, np.percentile(z_adjusted_values, 99))
            print(f"Wall depth range: {wall_z_min:.3f}m to {wall_z_max:.3f}m")
        else:
            print("Warning: No valid wall depth points found!")
            wall_z_min, wall_z_max = 0.001, 20.0
        
        for v in range(height):
            for u in range(width):
                if depth_map[v, u] > 0 and wall_mask[v, u]:
                    z = depth_map[v, u]
                    z_scaled = z / scale_factor_z
                    z_corrected = z_scaled * np.cos(tilt_angle)
                    
                    cx = width/2
                    cy = height/2
                    if hasattr(self, 'camera_params') and self.camera_params:
                        cx = self.camera_params.get('cx', width/2)
                        cy = self.camera_params.get('cy', height/2)
                    
                    x_adjusted = (u - cx) / scale_factor_x
                    y_adjusted = (v - cy) / scale_factor_y
                    y_adjusted = y_adjusted * np.cos(tilt_angle) - self.y_offset
                    z_adjusted = z_corrected + self.z_offset
                    
                    if wall_z_min <= z_adjusted <= wall_z_max:
                        points.append([x_adjusted, y_adjusted, z_adjusted])
                        colors.append(bgr_image[v, u] / 255.0)
        
        if len(points) == 0:
            raise ValueError("No points found in wall region after depth filtering")

        points = np.array(points, dtype=np.float32)
        colors = np.array(colors, dtype=np.float32)
        
        print(f"\nWall Point Cloud Results:")
        print(f"Total wall points: {len(points)}")
        print(f"Wall depth range: {np.min(points[:, 2]):.3f}m ~ {np.max(points[:, 2]):.3f}m")
        
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)
        pcd.colors = o3d.utility.Vector3dVector(colors)
        
        # 간단한 후처리
        pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
        clean_pcd, _ = pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
        
        plane_model, inliers = clean_pcd.segment_plane(distance_threshold=0.02, ransac_n=3, num_iterations=1000)
        a, b, c, d = plane_model
        plane_normal = np.array([a, b, c])
        wall_angle = np.arccos(np.dot(plane_normal, np.array([0, 0, 1])))
        wall_angle_deg = np.degrees(wall_angle)
        print(f"Detected wall orientation: {wall_angle_deg:.2f}° from vertical")
        
        plane_pcd = clean_pcd.select_by_index(inliers)
        
        # -------- 결과물 저장 부분: ./detection 내부에 파일명만으로 저장 --------
        if save_name:
            file_prefix = os.path.join(self.save_path, save_name)
            
            final_points = np.asarray(plane_pcd.points)
            final_colors = np.asarray(plane_pcd.colors)
            final_normals = np.asarray(plane_pcd.normals)
            
            structured_data = np.zeros(len(final_points), dtype=[
                ('x', '<f4'), ('y', '<f4'), ('z', '<f4'),
                ('red', 'u1'), ('green', 'u1'), ('blue', 'u1'),
                ('normal_x', '<f4'), ('normal_y', '<f4'), ('normal_z', '<f4')
            ])
            
            structured_data['x'] = final_points[:, 0]
            structured_data['y'] = final_points[:, 1]
            structured_data['z'] = final_points[:, 2]
            structured_data['red'] = final_colors[:, 0] * 255
            structured_data['green'] = final_colors[:, 1] * 255
            structured_data['blue'] = final_colors[:, 2] * 255
            structured_data['normal_x'] = final_normals[:, 0]
            structured_data['normal_y'] = final_normals[:, 1]
            structured_data['normal_z'] = final_normals[:, 2]
            
            npy_gz_path = f"{file_prefix}_wall.npy.gz"
            with gzip.GzipFile(npy_gz_path, 'w') as f:
                np.save(f, structured_data)
            print(f"Wall point cloud saved: {npy_gz_path}")
            
            ply_path = f"{file_prefix}_wall.ply"
            o3d.io.write_point_cloud(ply_path, plane_pcd)
            print(f"Wall PLY file saved: {ply_path}")
        
        return plane_pcd
    
    def visualize_point_cloud(self, pcd, output_path):
        """Create a basic visualization of the point cloud."""
        try:
            vis = o3d.visualization.Visualizer()
            vis.create_window(visible=False)
            vis.add_geometry(pcd)
            
            view_control = vis.get_view_control()
            view_control.set_zoom(0.8)
            view_control.set_front([0, 0, -1])
            view_control.set_up([0, -1, 0])
            
            vis.poll_events()
            vis.update_renderer()
            vis.capture_screen_image(output_path)
            vis.destroy_window()
            
            print(f"Point cloud visualization saved to {output_path}")
        except Exception as e:
            print(f"Error creating point cloud visualization: {e}")

    def calculate_scale_factor(self, depth_map, real_distance):
        """Calculate a scale factor based on real distance and center depth."""
        center_region = depth_map[
            depth_map.shape[0]//3:2*depth_map.shape[0]//3,
            depth_map.shape[1]//3:2*depth_map.shape[1]//3
        ]
        depth_value = np.median(center_region)
        
        if depth_value > 0:
            scale_factor = real_distance / depth_value
            print(f"Depth value at center: {depth_value}")
            print(f"Real distance: {real_distance}mm")
            print(f"Calculated scale factor: {scale_factor}")
            return scale_factor
        else:
            print("Warning: Could not calculate scale factor, depth value is zero")
            return 100

    def depth_correction(self, depth_map, focal_length):
        """Correct depth map using a simple radial factor based on focal length."""
        height, width = depth_map.shape
        cx, cy = width/2, height/2
        
        x, y = np.meshgrid(np.arange(width), np.arange(height))
        x = x - cx
        y = y - cy
        
        r = np.sqrt(x**2 + y**2)
        angle_factor = focal_length / np.sqrt(focal_length**2 + x**2 + y**2)
        
        return depth_map * angle_factor


if __name__ == '__main__':
    # 사용 예시
    pcg = PointCloudGenerator(save_path='./detection')
    # depth_map, image_path, mask 등을 준비한 뒤 사용:
    # pcg.create_point_cloud(depth_map, image_path, save_name='내_이미지')
    pass
