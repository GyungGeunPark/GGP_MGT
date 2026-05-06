"""
STL 도장 경로 생성기 (반구 레이캐스팅)
- STL 로드 → 반구 레이캐스팅 → 표면 투영 → 법선 추출 → 6DOF 경로
"""

import numpy as np
import pyvista as pv
from pathlib import Path
import json


class PaintPathGenerator:
    """STL 파일에서 도장 경로를 생성하는 클래스"""
    
    def __init__(self, stl_path: str):
        self.stl_path = Path(stl_path)
        self.mesh = pv.read(stl_path)
        self.bounds = self.mesh.bounds
        
        self.surface_points = None
        self.surface_normals = None
        self.path_6dof = None
        self.path_by_lines = None
        self.line_indices = []
        self.outlier_indices = []
        
        self.hemisphere_center = None
        self.hemisphere_radius = None
        self.ray_start_points = None
        
        self.size_x = self.bounds[1] - self.bounds[0]
        self.size_y = self.bounds[3] - self.bounds[2]
        self.size_z = self.bounds[5] - self.bounds[4]
        
        print(f"[INFO] STL 로드 완료: {stl_path}")
        print(f"[INFO] Bounds: X({self.size_x:.4f}), Y({self.size_y:.4f}), Z({self.size_z:.4f})")
    
    def generate_multi_direction_path(self, x_margin_min=0.0, x_margin_max=0.0,
                                       line_spacing=0.05, points_per_line=30,
                                       normal_offset=0.0, zigzag=False,
                                       y_rotation_limit=None):
        """반구 형태로 레이캐스팅하여 표면 경로 생성"""
        print(f"\n{'='*50}")
        print(f"[INFO] 반구 레이캐스팅 경로 생성")
        print(f"[INFO] 라인 간격: {line_spacing}, 라인당 점: {points_per_line}")
        print('='*50)
        
        radius = self.size_y / 2
        center = np.array([
            (self.bounds[0] + self.bounds[1]) / 2,
            (self.bounds[2] + self.bounds[3]) / 2,
            self.bounds[4]
        ])
        
        self.hemisphere_radius = radius
        self.hemisphere_center = center
        self.normal_offset = normal_offset
        
        x_min = self.bounds[0] + x_margin_min
        x_max = self.bounds[1] - x_margin_max
        if x_min >= x_max:
            print("[WARN] X 마진이 너무 큼!")
            return {'points': np.array([]), 'normals': np.array([]), 'line_indices': []}
        
        x_positions = np.arange(x_min, x_max + line_spacing / 2, line_spacing)
        num_lines = len(x_positions)
        theta_angles = np.linspace(0, np.pi, points_per_line)
        
        all_points, all_normals, all_ray_starts = [], [], []
        self.line_indices = []
        all_lines_points, all_lines_normals, all_lines_ray_dirs = [], [], []
        
        for line_idx, x in enumerate(x_positions):
            line_points, line_normals, line_ray_dirs = [], [], []
            angles = theta_angles if not zigzag or line_idx % 2 == 0 else theta_angles[::-1]
            
            for theta in angles:
                ray_start = np.array([x, center[1] + radius * np.cos(theta), center[2] + radius * np.sin(theta)])
                ray_end = center.copy()
                ray_end[0] = x
                ray_dir = ray_end - ray_start
                ray_dir = ray_dir / np.linalg.norm(ray_dir)
                all_ray_starts.append(ray_start.copy())
                
                intersection_points, intersection_cells = self.mesh.ray_trace(ray_start, ray_end)
                hit_point, normal = None, None
                
                if len(intersection_points) > 0:
                    valid_points, valid_cells, valid_distances = [], [], []
                    for pt, cell_id in zip(intersection_points, intersection_cells):
                        cell = self.mesh.get_cell(cell_id)
                        pts = cell.points
                        face_normal = np.cross(pts[1] - pts[0], pts[2] - pts[0])
                        norm_len = np.linalg.norm(face_normal)
                        if norm_len > 1e-10:
                            face_normal = face_normal / norm_len
                            if np.dot(face_normal, ray_dir) < 0:
                                valid_points.append(pt)
                                valid_cells.append(cell_id)
                                valid_distances.append(np.linalg.norm(pt - ray_start))
                    
                    if valid_points:
                        closest_idx = np.argmin(valid_distances)
                        hit_point = valid_points[closest_idx]
                        cell_id = valid_cells[closest_idx]
                        cell = self.mesh.get_cell(cell_id)
                        pts = cell.points
                        normal = np.cross(pts[1] - pts[0], pts[2] - pts[0])
                        norm_len = np.linalg.norm(normal)
                        normal = normal / norm_len if norm_len > 1e-10 else np.array([0, 0, 1])
                        if np.dot(normal, ray_dir) > 0:
                            normal = -normal
                        if y_rotation_limit is not None:
                            nx, ny, nz = normal
                            y_rot_angle = np.degrees(np.arctan2(abs(nx), np.sqrt(ny**2 + nz**2)))
                            if y_rot_angle > y_rotation_limit:
                                normal_yz = np.array([0, ny, nz])
                                norm_len = np.linalg.norm(normal_yz)
                                if norm_len > 1e-10:
                                    normal = normal_yz / norm_len
                
                line_points.append(hit_point)
                line_normals.append(normal)
                line_ray_dirs.append(-ray_dir)
            
            all_lines_points.append(line_points)
            all_lines_normals.append(line_normals)
            all_lines_ray_dirs.append(line_ray_dirs)
        
        # miss된 점 채우기
        num_lines = len(all_lines_points)
        pts_per_line = len(all_lines_points[0]) if num_lines > 0 else 0
        
        for line_idx in range(num_lines):
            for pt_idx in range(pts_per_line):
                if all_lines_points[line_idx][pt_idx] is None:
                    for offset in range(1, num_lines):
                        if line_idx - offset >= 0 and all_lines_points[line_idx - offset][pt_idx] is not None:
                            ref = line_idx - offset
                            break
                        if line_idx + offset < num_lines and all_lines_points[line_idx + offset][pt_idx] is not None:
                            ref = line_idx + offset
                            break
                    else:
                        continue
                    new_point = all_lines_points[ref][pt_idx].copy()
                    new_point[0] = x_positions[line_idx]
                    all_lines_points[line_idx][pt_idx] = new_point
                    all_lines_normals[line_idx][pt_idx] = all_lines_normals[ref][pt_idx]
                    all_lines_ray_dirs[line_idx][pt_idx] = all_lines_ray_dirs[ref][pt_idx]
        
        # 최종 결과 조합
        for line_idx in range(num_lines):
            line_start = len(all_points)
            for pt_idx in range(pts_per_line):
                hit_point = all_lines_points[line_idx][pt_idx]
                normal = all_lines_normals[line_idx][pt_idx]
                outward_dir = all_lines_ray_dirs[line_idx][pt_idx]
                if hit_point is not None and normal is not None:
                    offset_point = hit_point + outward_dir * normal_offset
                    all_points.append(offset_point)
                    all_normals.append(normal)
            line_end = len(all_points)
            if line_end > line_start:
                self.line_indices.append((line_start, line_end))
        
        self.ray_start_points = np.array(all_ray_starts)
        self.surface_points = np.array(all_points) if all_points else np.array([]).reshape(0, 3)
        self.surface_normals = np.array(all_normals) if all_normals else np.array([]).reshape(0, 3)
        
        print(f"[INFO] 전체 경로: {len(self.surface_points)}개 점, {len(self.line_indices)}개 라인")
        return {'points': self.surface_points, 'normals': self.surface_normals, 'line_indices': self.line_indices}
    
    def find_sharp_turns(self, angle_threshold=30.0):
        """같은 행(라인) 내에서 갑자기 꺾이는 점 찾기 (단순 버전)"""
        if self.surface_points is None or len(self.surface_points) == 0:
            return
        
        self.outlier_indices = []
        
        for line_idx, (start, end) in enumerate(self.line_indices):
            line_points = self.surface_points[start:end]
            n_pts = len(line_points)
            if n_pts < 3:
                continue
            
            for i in range(1, n_pts - 1):
                v1 = line_points[i] - line_points[i - 1]
                v2 = line_points[i + 1] - line_points[i]
                len_v1, len_v2 = np.linalg.norm(v1), np.linalg.norm(v2)
                if len_v1 < 1e-10 or len_v2 < 1e-10:
                    continue
                cos_angle = np.clip(np.dot(v1, v2) / (len_v1 * len_v2), -1.0, 1.0)
                angle_deg = np.degrees(np.arccos(cos_angle))
                if angle_deg > angle_threshold:
                    self.outlier_indices.append(start + i)
                    print(f"  [라인{line_idx}] 점{i}: 각도 {angle_deg:.1f}°")
        
        print(f"[INFO] 꺾임 이상점: {len(self.outlier_indices)}개 (threshold={angle_threshold}°)")
    
    def find_sharp_turns_with_chain(self, angle_threshold=30.0, deviation_threshold=0.02, trend_window=3):
        """같은 행(라인) 내에서 갑자기 꺾이는 점 + 연쇄 이상점 찾기"""
        if self.surface_points is None or len(self.surface_points) == 0:
            return
        
        self.outlier_indices = []
        
        for line_idx, (start, end) in enumerate(self.line_indices):
            line_points = self.surface_points[start:end]
            n_pts = len(line_points)
            if n_pts < 3:
                continue
            
            line_outliers = set()
            i = 1
            
            while i < n_pts - 1:
                if i in line_outliers:
                    i += 1
                    continue
                
                v1 = line_points[i] - line_points[i - 1]
                v2 = line_points[i + 1] - line_points[i]
                len_v1, len_v2 = np.linalg.norm(v1), np.linalg.norm(v2)
                
                if len_v1 < 1e-10 or len_v2 < 1e-10:
                    i += 1
                    continue
                
                cos_angle = np.clip(np.dot(v1, v2) / (len_v1 * len_v2), -1.0, 1.0)
                angle_deg = np.degrees(np.arccos(cos_angle))
                
                if angle_deg > angle_threshold:
                    line_outliers.add(i)
                    print(f"  [라인{line_idx}] 점{i}: 각도 {angle_deg:.1f}° (1차)")
                    
                    # 연쇄 감지
                    normal_pts = [line_points[j] for j in range(i - 1, -1, -1) if j not in line_outliers][:trend_window]
                    
                    if len(normal_pts) >= 2:
                        trend_dir = normal_pts[0] - normal_pts[-1]
                        trend_len = np.linalg.norm(trend_dir)
                        
                        if trend_len > 1e-10:
                            trend_dir /= trend_len
                            last_pt = normal_pts[0]
                            avg_step = trend_len / (len(normal_pts) - 1)
                            
                            for j in range(i + 1, n_pts):
                                if j in line_outliers:
                                    continue
                                steps = j - (i - 1)
                                expected = last_pt + trend_dir * avg_step * steps
                                deviation = np.linalg.norm(line_points[j] - expected)
                                
                                if deviation > deviation_threshold:
                                    line_outliers.add(j)
                                    print(f"  [라인{line_idx}] 점{j}: 이탈 {deviation:.4f} (연쇄)")
                                else:
                                    break
                i += 1
            
            for local_idx in sorted(line_outliers):
                self.outlier_indices.append(start + local_idx)
        
        print(f"[INFO] 꺾임+연쇄 이상점: {len(self.outlier_indices)}개")
    
    def correct_outliers_by_column(self):
        """이상점을 같은 열의 가장 가까운 정상 점으로 보정"""
        if not self.outlier_indices:
            print("[INFO] 보정할 이상점 없음")
            return
        
        num_lines = len(self.line_indices)
        outlier_set = set(self.outlier_indices)
        corrected = 0
        
        for global_idx in self.outlier_indices:
            line_idx, pt_idx = None, None
            for li, (start, end) in enumerate(self.line_indices):
                if start <= global_idx < end:
                    line_idx, pt_idx = li, global_idx - start
                    break
            if line_idx is None:
                continue
            
            ref_line = None
            for offset in range(1, num_lines):
                for check in [line_idx - offset, line_idx + offset]:
                    if 0 <= check < num_lines:
                        check_idx = self.line_indices[check][0] + pt_idx
                        if check_idx not in outlier_set:
                            ref_line = check
                            break
                if ref_line is not None:
                    break
            
            if ref_line is None:
                continue
            
            ref_idx = self.line_indices[ref_line][0] + pt_idx
            ref_point = self.surface_points[ref_idx].copy()
            ref_point[0] = self.surface_points[global_idx][0]
            
            self.surface_points[global_idx] = ref_point
            self.surface_normals[global_idx] = self.surface_normals[ref_idx].copy()
            corrected += 1
        
        print(f"[INFO] 이상점 보정: {corrected}개 수정")
    
    def smooth_x_rotation_outliers(self, threshold=10.0, window=3):
        """X축 회전 이상치 보정"""
        if self.surface_normals is None:
            return
        
        total = 0
        for start, end in self.line_indices:
            normals = self.surface_normals[start:end]
            n_pts = len(normals)
            if n_pts < 3:
                continue
            
            x_rots = np.array([np.degrees(np.arctan2(n[1], n[2])) for n in normals])
            
            for i in range(n_pts):
                neighbors = [x_rots[i + w] for w in range(-window, window + 1) if w != 0 and 0 <= i + w < n_pts]
                if len(neighbors) < 2:
                    continue
                
                if abs(x_rots[i] - np.mean(neighbors)) > threshold:
                    before = [x_rots[i - w] for w in range(1, window + 1) if i - w >= 0]
                    after = [x_rots[i + w] for w in range(1, window + 1) if i + w < n_pts]
                    expected = np.mean(before + after) if before and after else np.mean(before or after)
                    
                    nx, ny, nz = self.surface_normals[start + i]
                    yz_mag = np.sqrt(ny**2 + nz**2)
                    if yz_mag > 1e-10:
                        rad = np.radians(expected)
                        new_n = np.array([nx, yz_mag * np.sin(rad), yz_mag * np.cos(rad)])
                        self.surface_normals[start + i] = new_n / np.linalg.norm(new_n)
                        total += 1
        
        print(f"[INFO] X축 회전 보정: {total}개")
    
    def get_6dof_path(self, invert_normal=True):
        """6DOF 경로 반환"""
        if self.surface_points is None or len(self.surface_points) == 0:
            self.path_6dof = np.array([]).reshape(0, 6)
            return self.path_6dof
        
        normals = -self.surface_normals if invert_normal else self.surface_normals
        self.path_6dof = np.hstack([self.surface_points, normals])
        print(f"[INFO] 6DOF 경로: {len(self.path_6dof)}개 점")
        return self.path_6dof
    
    def get_path_by_lines(self, invert_normal=True):
        """라인별 6DOF 경로 리스트"""
        if self.path_6dof is None:
            self.get_6dof_path(invert_normal=invert_normal)
        
        self.path_by_lines = [self.path_6dof[start:end].tolist() for start, end in self.line_indices]
        print(f"[INFO] 라인별 경로: {len(self.path_by_lines)}개 라인")
        return self.path_by_lines
    
    def save_path_by_lines(self, output_path, format='json', invert_normal=True):
        """라인별 경로 저장"""
        if self.path_by_lines is None:
            self.get_path_by_lines(invert_normal=invert_normal)
        
        if format == 'json':
            data = {
                "num_lines": len(self.path_by_lines),
                "total_points": sum(len(l) for l in self.path_by_lines),
                "lines": self.path_by_lines
            }
            with open(output_path, 'w') as f:
                json.dump(data, f, indent=2)
        elif format == 'csv':
            rows = [pt + [i] for i, line in enumerate(self.path_by_lines) for pt in line]
            np.savetxt(output_path, rows, delimiter=',', header="x,y,z,nx,ny,nz,line_idx", comments='')
        
        print(f"[INFO] 저장: {output_path}")
    
    def visualize(self, show_mesh=True, show_normals=True, show_outliers=True, 
                  show_hemisphere=True, show_ray_starts=True, normal_scale=None, 
                  show_inverted_normals=True):
        """시각화"""
        if normal_scale is None:
            normal_scale = min(self.size_x, self.size_y, self.size_z) * 0.1
        
        p = pv.Plotter()
        p.set_background('white')
        
        if show_mesh:
            p.add_mesh(self.mesh, color='lightblue', opacity=0.5)
        
        if show_hemisphere and self.hemisphere_center is not None:
            sphere = pv.Sphere(radius=self.hemisphere_radius, center=self.hemisphere_center)
            hemi = sphere.clip(normal=[0, 0, -1], origin=self.hemisphere_center)
            p.add_mesh(hemi, color='yellow', opacity=0.3, style='wireframe')
        
        if self.surface_points is not None and len(self.surface_points) > 0:
            if show_outliers and self.outlier_indices:
                mask = np.ones(len(self.surface_points), dtype=bool)
                mask[self.outlier_indices] = False
                p.add_mesh(pv.PolyData(self.surface_points[mask]), color='red', point_size=8, render_points_as_spheres=True)
                p.add_mesh(pv.PolyData(self.surface_points[self.outlier_indices]), color='blue', point_size=12, render_points_as_spheres=True)
            else:
                p.add_mesh(pv.PolyData(self.surface_points), color='red', point_size=8, render_points_as_spheres=True)
            
            if show_normals:
                arrows = pv.PolyData(self.surface_points)
                arrows['vectors'] = (-self.surface_normals if show_inverted_normals else self.surface_normals) * normal_scale
                p.add_mesh(arrows.glyph(orient='vectors', scale=False, factor=normal_scale), color='darkred' if show_inverted_normals else 'green')
            
            for start, end in self.line_indices:
                pts = self.surface_points[start:end]
                if len(pts) > 1:
                    n = len(pts)
                    lines = np.column_stack([np.full(n-1, 2), np.arange(n-1), np.arange(1, n)]).flatten()
                    p.add_mesh(pv.PolyData(pts, lines=lines), color='blue', line_width=2)
        
        p.add_axes()
        p.camera_position = 'iso'
        p.show()


def main():
    # ========== 파라미터 ==========
    STL_FILE = "gene1.stl"
    X_MARGIN_MIN = 0.05 # 0.05
    X_MARGIN_MAX = 0.02 # 0.02
    LINE_SPACING = 0.05
    POINTS_PER_LINE = 50
    NORMAL_OFFSET = 0.20
    Y_ROTATION_LIMIT = 5
    
    SMOOTH_X_ROTATION = True
    X_ROTATION_THRESHOLD = 5
    EXTEND_COLUMNS = True
    EXTEND_COUNT = 4  # 앞뒤로 각각 추가할 열 개수
    
    # 이상점 감지/보정
    FIND_SHARP_TURNS = True
    ANGLE_THRESHOLD = 20
    USE_CHAIN_DETECTION = True  # True: 연쇄 감지, False: 단순 감지
    DEVIATION_THRESHOLD = 0.1  # 연쇄 감지용
    CORRECT_OUTLIERS = True
    
    # 저장
    INVERT_NORMAL = True
    OUTPUT_FILE = "robot_path.json"
    # ==============================
    
    gen = PaintPathGenerator(STL_FILE)
    
    gen.generate_multi_direction_path(
        x_margin_min=X_MARGIN_MIN, x_margin_max=X_MARGIN_MAX,
        line_spacing=LINE_SPACING, points_per_line=POINTS_PER_LINE,
        normal_offset=NORMAL_OFFSET, y_rotation_limit=Y_ROTATION_LIMIT
    )
    
    if SMOOTH_X_ROTATION:
        gen.smooth_x_rotation_outliers(threshold=X_ROTATION_THRESHOLD)
    
    if FIND_SHARP_TURNS:
        if USE_CHAIN_DETECTION:
            gen.find_sharp_turns_with_chain(
                angle_threshold=ANGLE_THRESHOLD,
                deviation_threshold=DEVIATION_THRESHOLD
            )
        else:
            gen.find_sharp_turns(angle_threshold=ANGLE_THRESHOLD)
        
        if CORRECT_OUTLIERS:
            gen.correct_outliers_by_column()
    
    gen.get_6dof_path(invert_normal=INVERT_NORMAL)
    gen.get_path_by_lines(invert_normal=INVERT_NORMAL)
    
    if OUTPUT_FILE:
        gen.save_path_by_lines(OUTPUT_FILE, invert_normal=INVERT_NORMAL)
    
    gen.visualize(show_inverted_normals=INVERT_NORMAL)


if __name__ == '__main__':
    main()