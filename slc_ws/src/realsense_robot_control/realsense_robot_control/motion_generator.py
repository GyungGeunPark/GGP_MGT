#!/usr/bin/env python3

import numpy as np
import json
import yaml
import os
from datetime import datetime
from typing import List, Tuple, Dict, Optional
import math

class MotionGenerator:
    """Generate robot motion path based on detected curved surfaces"""
    
    def __init__(self):
        # Motion parameters
        self.approach_distance = 0.05  # 5cm approach distance
        self.motion_height = 0.04      # 4cm height above surface (평면 이동을 위해 높이 확보)
        
        # Robot motion speed parameters
        self.linear_speed = 100.0      # mm/s
        self.angular_speed = 30.0      # deg/s
        self.acceleration = 200.0      # mm/s^2
        
    def load_surfaces_from_json(self, json_files: List[str]) -> List[Dict]:
        """Load surface data from multiple JSON files"""
        surfaces = []
        for json_path in json_files:
            with open(json_path, 'r') as f:
                surface_data = json.load(f)
                surfaces.append(surface_data)
        return surfaces
    
    def convert_pixel_to_world(self, pixel_coords: List[float], depth_value: float) -> np.ndarray:
        """Convert pixel coordinates to world coordinates"""
        fx, fy, cx, cy = 640.0, 640.0, 640.0, 360.0
        depth_m = depth_value / 1000.0
        x = (pixel_coords[0] - cx) * depth_m / fx
        y = (pixel_coords[1] - cy) * depth_m / fy
        z = depth_m
        return np.array([x, y, z])
    
    def process_curved_surface(self, surface_data: Dict) -> Dict:
        """Process curved surface data and extract relevant information"""
        center_pixel = surface_data['center_point']
        depth = surface_data['cv_value']
        center_world = self.convert_pixel_to_world(center_pixel[:2], depth)
        normal = np.array(surface_data['normal_vector'])
        # JSON의 경계점 순서: top-left, top-right, bottom-right, bottom-left
        boundary_points = [self.convert_pixel_to_world(pt[:2], depth) for pt in surface_data['boundary_points']]
        return {
            'id': surface_data['id'],
            'center': center_world,
            'normal': normal / np.linalg.norm(normal),
            'corners': {
                'tl': boundary_points[0],
                'tr': boundary_points[1],
                'br': boundary_points[2],
                'bl': boundary_points[3]
            }
        }

    def _generate_planar_edge_path(self, surface1: Dict, surface2: Dict) -> List[Dict]:
        """
        Generates a planar path that moves between the midpoints of the edges of the two parts.
        Path: Red(L->T->R) -> Green(B->L->T->R) -> Return to start
        """
        # Y 좌표를 기준으로 위/아래 부품 식별 (Y값이 작을수록 이미지상 위쪽)
        if surface1['center'][1] < surface2['center'][1]:
            red_part = surface1
            green_part = surface2
        else:
            red_part = surface2
            green_part = surface1
            
        g = green_part['corners']
        r = red_part['corners']
        
        # 1. 두 부품의 평균 Normal을 사용하여 작업 평면 설정
        avg_normal = (green_part['normal'] + red_part['normal']) / 2
        avg_normal /= np.linalg.norm(avg_normal)
        
        # 2. 사용자가 요청한 경로에 따라 각 모서리의 중심점을 Vertex로 정의
        path_vertices = [
            {'pos': (r['tl'] + r['bl']) / 2, 'name': 'Red_Left'},        # 1. 빨간색 좌
            {'pos': (r['tl'] + r['tr']) / 2, 'name': 'Red_Top'},         # 2. 빨간색 위
            {'pos': (r['tr'] + r['br']) / 2, 'name': 'Red_Right'},       # 3. 빨간색 우
            {'pos': (g['bl'] + g['br']) / 2, 'name': 'Green_Bottom'},    # 4. 초록색 아래
            {'pos': (g['tl'] + g['bl']) / 2, 'name': 'Green_Left'},      # 5. 초록색 좌
            {'pos': (g['tl'] + g['tr']) / 2, 'name': 'Green_Top'},       # 6. 초록색 위
            {'pos': (g['tr'] + g['br']) / 2, 'name': 'Green_Right'},     # 7. 초록색 우
        ]

        waypoints = []
        
        # 3. 시작점으로 접근 (Approach)
        # 모든 웨이포인트는 이제 avg_normal을 기준으로 한 평면 위에 생성됨
        motion_plane_offset = avg_normal * self.motion_height
        
        approach_pos = path_vertices[0]['pos'] + motion_plane_offset + avg_normal * self.approach_distance
        tangent_to_first = path_vertices[1]['pos'] - path_vertices[0]['pos']
        waypoints.append({
            'position': approach_pos.tolist(),
            'orientation': self.calculate_tool_orientation(-avg_normal, tangent_to_first),
            'speed': self.linear_speed,
            'name': 'Approach'
        })
        
        # 4. 정의된 Vertex들을 순서대로 이동
        for i, vertex_info in enumerate(path_vertices):
            next_vertex_info = path_vertices[(i + 1) % len(path_vertices)]
            
            wp_pos = vertex_info['pos'] + motion_plane_offset
            tangent = next_vertex_info['pos'] - vertex_info['pos']
            orientation = self.calculate_tool_orientation(-avg_normal, tangent)
            
            waypoints.append({
                'position': wp_pos.tolist(),
                'orientation': orientation,
                'speed': self.linear_speed,
                'name': vertex_info['name']
            })

        # 5. 마지막 지점에서 시작 지점으로 복귀 (Return to Start)
        return_pos = path_vertices[0]['pos'] + motion_plane_offset
        tangent_to_return = path_vertices[0]['pos'] - path_vertices[-1]['pos']
        waypoints.append({
            'position': return_pos.tolist(),
            'orientation': self.calculate_tool_orientation(-avg_normal, tangent_to_return),
            'speed': self.linear_speed,
            'name': 'Return_to_Start'
        })

        # 6. 시작 지점에서 후퇴 (Retreat)
        retreat_pos = return_pos + avg_normal * self.approach_distance
        waypoints.append({
            'position': retreat_pos.tolist(),
            'orientation': waypoints[-1]['orientation'],
            'speed': self.linear_speed,
            'name': 'Retreat'
        })

        return waypoints

    def calculate_tool_orientation(self, z_axis_dir: np.ndarray, x_axis_hint: np.ndarray) -> Dict:
        """Calculate tool orientation as Euler angles (rx, ry, rz in degrees)."""
        z_axis = z_axis_dir / np.linalg.norm(z_axis_dir)
        y_axis = np.cross(z_axis, x_axis_hint)
        if np.linalg.norm(y_axis) < 1e-6:
             x_axis_hint_alt = np.array([1.0, 0.0, 0.0])
             if np.linalg.norm(np.cross(z_axis, x_axis_hint_alt)) < 1e-6:
                  x_axis_hint_alt = np.array([0.0, 1.0, 0.0])
             y_axis = np.cross(z_axis, x_axis_hint_alt)
        y_axis = y_axis / np.linalg.norm(y_axis)
        x_axis = np.cross(y_axis, z_axis)
        R = np.column_stack((x_axis, y_axis, z_axis))
        sy = np.sqrt(R[0, 0]**2 + R[1, 0]**2)
        singular = sy < 1e-6
        if not singular:
            rx = np.arctan2(R[2, 1], R[2, 2])
            ry = np.arctan2(-R[2, 0], sy)
            rz = np.arctan2(R[1, 0], R[0, 0])
        else:
            rx, ry, rz = np.arctan2(-R[1, 2], R[1, 1]), np.arctan2(-R[2, 0], sy), 0
        return {'rx': np.degrees(rx), 'ry': np.degrees(ry), 'rz': np.degrees(rz)}
    
    def save_motion_yaml(self, waypoints: List[Dict], output_path: str):
        """Save motion data to YAML file for robot execution"""
        motion_data = {'motion_info': {'name': 'planar_edge_path', 'timestamp': datetime.now().isoformat(), 'total_waypoints': len(waypoints), 'motion_type': 'cartesian_path', 'coordinate_frame': 'camera_frame'}, 'motion_parameters': {'approach_distance_mm': self.approach_distance * 1000, 'motion_height_mm': self.motion_height * 1000, 'linear_speed_mm_s': self.linear_speed, 'acceleration_mm_s2': self.acceleration}, 'waypoints': []}
        for i, wp in enumerate(waypoints):
            robot_wp = {'id': i, 'name': wp['name'], 'pose': {'position': {'x': wp['position'][0] * 1000, 'y': wp['position'][1] * 1000, 'z': wp['position'][2] * 1000}, 'orientation': wp['orientation']}, 'speed': wp['speed'], 'blend_radius': 15.0} # 블렌딩 반경을 늘려 더 부드럽게
            motion_data['waypoints'].append(robot_wp)
        with open(output_path, 'w') as f:
            yaml.dump(motion_data, f, default_flow_style=False, sort_keys=False)
        return motion_data
    
    def save_motion_json(self, waypoints: List[Dict], output_path: str):
        """Save motion data to JSON file for visualization"""
        motion_data = {'timestamp': datetime.now().isoformat(), 'motion_type': 'planar_edge_path', 'coordinate_frame': 'camera_frame', 'motion_parameters': {'approach_distance': self.approach_distance, 'motion_height': self.motion_height, 'linear_speed': self.linear_speed, 'angular_speed': self.angular_speed, 'acceleration': self.acceleration}, 'waypoints': waypoints, 'total_waypoints': len(waypoints)}
        with open(output_path, 'w') as f:
            json.dump(motion_data, f, indent=2)
        return motion_data

    def generate_motion_from_surfaces(self, surface_files: List[str], 
                                    output_dir: str = ".") -> Tuple[str, str]:
        """
        Main function to generate motion from surface data files.
        It requires exactly two surface files to generate a planar edge-following path.
        """
        if len(surface_files) != 2:
            raise ValueError(f"Planar edge path generation requires exactly 2 surface files, but {len(surface_files)} were provided.")

        all_surfaces_data = self.load_surfaces_from_json(surface_files)
        
        processed_surface1 = self.process_curved_surface(all_surfaces_data[0])
        processed_surface2 = self.process_curved_surface(all_surfaces_data[1])
        
        # Generate the custom planar path using the new function
        all_waypoints = self._generate_planar_edge_path(processed_surface1, processed_surface2)

        if not all_waypoints:
            raise ValueError("Could not generate any waypoints from the provided surface files.")

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        yaml_file = os.path.join(output_dir, f"robot_motion_{timestamp}.yaml")
        json_file = os.path.join(output_dir, f"motion_visualization_{timestamp}.json")
        
        self.save_motion_yaml(all_waypoints, yaml_file)
        self.save_motion_json(all_waypoints, json_file)
        
        return yaml_file, json_file


# Test the motion generator
if __name__ == "__main__":
    generator = MotionGenerator()
    surface_files = [
        "selected_part1_bt_face_data.json",
        "selected_part2_bt_face_data.json"
    ]
    output_directory = "test_motion_output"
    os.makedirs(output_directory, exist_ok=True)
    
    try:
        print(f"Generating planar edge-following motion for {len(surface_files)} surfaces...")
        yaml_file, json_file = generator.generate_motion_from_surfaces(
            surface_files,
            output_dir=output_directory
        )
        print(f"Planar motion generated successfully!")
        print(f"Robot motion file: {yaml_file}")
        print(f"Visualization file: {json_file}")
    except Exception as e:
        print(f"Error generating motion: {e}")