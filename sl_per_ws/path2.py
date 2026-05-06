"""
경로 생성기 v4.1 - 자동 시작점 버전 (Y+ 필터링 + 균일 샘플링)
- 변경: JSON 출력을 [로봇위치 + 표면접촉점] 형태로 수정
- 변경: 시각화에서 로봇위치 → 표면점 연결선 표시
"""

import numpy as np
import open3d as o3d
import pyvista as pv
import json


# ============================================================
# [공통] 유틸리티 함수
# ============================================================

def get_perpendicular_xplus_direction(normal):
    """법선과 수직이면서 X+ 방향 성분이 양수인 단위 벡터 계산"""
    normal = normal / np.linalg.norm(normal)
    x_axis = np.array([1.0, 0.0, 0.0])
    projection = x_axis - np.dot(x_axis, normal) * normal
    proj_len = np.linalg.norm(projection)
    if proj_len < 1e-10:
        y_axis = np.array([0.0, 1.0, 0.0])
        projection = y_axis - np.dot(y_axis, normal) * normal
        proj_len = np.linalg.norm(projection)
    ray_dir = projection / proj_len
    if ray_dir[0] < 0:
        ray_dir = -ray_dir
    return ray_dir


def check_normal_angle_change(new_normal, prev_normal, angle_threshold=30.0):
    """법선 벡터 간 직접 각도 비교"""
    new_norm = new_normal / np.linalg.norm(new_normal)
    prev_norm = prev_normal / np.linalg.norm(prev_normal)
    dot = np.clip(np.dot(new_norm, prev_norm), -1.0, 1.0)
    angle_diff = np.degrees(np.arccos(dot))
    return angle_diff > angle_threshold, angle_diff


def check_path_direction_change(prev_point, current_point, new_point, angle_threshold=45.0):
    """경로의 방향 변화 체크"""
    prev_direction = current_point - prev_point
    prev_length = np.linalg.norm(prev_direction)
    if prev_length < 1e-10:
        return False, 0.0
    prev_direction = prev_direction / prev_length
    new_direction = new_point - current_point
    new_length = np.linalg.norm(new_direction)
    if new_length < 1e-10:
        return False, 0.0
    new_direction = new_direction / new_length
    dot_product = np.clip(np.dot(prev_direction, new_direction), -1.0, 1.0)
    angle_diff = np.degrees(np.arccos(dot_product))
    return angle_diff > angle_threshold, angle_diff


def normalize_robot_positions_to_line(robot_positions, surface_points):
    """
    로봇 타겟 포지션들을 PCA 기반 중심 라인으로 정규화
    (표면 접촉점은 변경하지 않음 - 이미 거의 일직선이므로)

    방법:
    1. 로봇 포지션들의 중심점(centroid) 계산
    2. PCA로 주성분 방향(라인 방향) 추출
    3. 각 로봇 포지션을 라인에 투영하여 정규화

    Args:
        robot_positions: 로봇 타겟 포지션 배열 (N, 3)
        surface_points: 표면 접촉점 배열 (N, 3) - 변경되지 않음

    Returns:
        normalized_robot_positions: 정규화된 로봇 포지션 배열
        surface_points: 원본 표면 접촉점 (그대로 반환)
        line_info: 라인 정보 딕셔너리 (center, direction, deviations)
    """
    if len(robot_positions) < 2:
        return robot_positions.copy(), surface_points.copy(), None

    positions = np.array(robot_positions)

    # 1. 중심점 계산
    centroid = np.mean(positions, axis=0)
    print(f"  [로봇 포지션 정규화] 중심점: ({centroid[0]:.4f}, {centroid[1]:.4f}, {centroid[2]:.4f})")

    # 2. PCA로 주 방향축 계산
    centered = positions - centroid
    cov_matrix = np.cov(centered.T)
    eigenvalues, eigenvectors = np.linalg.eigh(cov_matrix)

    # 가장 큰 고유값에 해당하는 고유벡터 = 주 방향 (라인 방향)
    sort_idx = np.argsort(eigenvalues)[::-1]
    principal_direction = eigenvectors[:, sort_idx[0]]

    # 방향 일관성: Y+ 방향으로 정렬
    if principal_direction[1] < 0:
        principal_direction = -principal_direction

    print(f"  [로봇 포지션 정규화] 주 방향: ({principal_direction[0]:.4f}, {principal_direction[1]:.4f}, {principal_direction[2]:.4f})")

    # 3. 각 로봇 포지션을 라인에 투영
    normalized_positions = []
    deviations = []

    for i, pos in enumerate(positions):
        # 중심점에서 현재 포지션까지의 벡터
        vec_to_point = pos - centroid

        # 라인 위 투영점 계산: centroid + t * direction
        t = np.dot(vec_to_point, principal_direction)
        projected_point = centroid + t * principal_direction

        # 편차 계산 (원래 점과 투영점 사이 거리)
        deviation = np.linalg.norm(pos - projected_point)
        deviations.append(deviation)

        normalized_positions.append(projected_point)

    normalized_positions = np.array(normalized_positions)
    deviations = np.array(deviations)

    print(f"  [로봇 포지션 정규화] 평균 편차: {np.mean(deviations)*1000:.2f}mm, 최대 편차: {np.max(deviations)*1000:.2f}mm")

    line_info = {
        'center': centroid,
        'direction': principal_direction,
        'deviations': deviations,
        'mean_deviation': np.mean(deviations),
        'max_deviation': np.max(deviations)
    }

    # 표면 접촉점은 그대로 반환 (이미 일직선이므로)
    return normalized_positions, np.array(surface_points), line_info


# ============================================================
# [1단계] 반구 레이캐스팅으로 X 최소 모서리 점 + 법선 자동 추출
# ============================================================

def extract_edge_points_with_normals(stl_path, x_extend=0.2, line_spacing=0.0005,
                                      points_per_line=180, radius_scale=1.2,
                                      x_offset=0.03, lift_distance=0.1,
                                      trim_start=0, trim_end=0):
    """반구 레이캐스팅으로 X 최소 모서리 점들과 해당 삼각형 법선을 추출"""
    print("\n" + "="*60)
    print("[1단계] 반구 레이캐스팅 - X 최소 모서리 점 추출")
    print("="*60)
    
    mesh_o3d = o3d.io.read_triangle_mesh(stl_path)
    mesh_o3d.compute_vertex_normals()
    mesh_o3d.compute_triangle_normals()
    
    bbox = mesh_o3d.get_axis_aligned_bounding_box()
    min_bound, max_bound = np.array(bbox.min_bound), np.array(bbox.max_bound)
    size_x, size_y, size_z = max_bound - min_bound
    print(f"\n[INFO] 메시 크기: X={size_x:.4f}, Y={size_y:.4f}, Z={size_z:.4f}")
    
    radius = max(size_y, size_z) / 2 * radius_scale
    center_x = (min_bound[0] + max_bound[0]) / 2
    center_y = (min_bound[1] + max_bound[1]) / 2
    center_z = (min_bound[2] + max_bound[2]) / 2
    print(f"[INFO] 중심: ({center_x:.4f}, {center_y:.4f}, {center_z:.4f})")
    print(f"[INFO] 반지름: {radius:.4f}")
    
    x_positions = np.arange(min_bound[0] - x_extend, max_bound[0] + x_extend + line_spacing/2, line_spacing)
    theta_angles = np.linspace(0, 2*np.pi, points_per_line, endpoint=False)
    num_x, num_theta = len(x_positions), len(theta_angles)
    print(f"[INFO] X 라인 수: {num_x}, theta 점 수: {num_theta}, 총 레이 수: {num_x * num_theta}")
    
    mesh_t = o3d.t.geometry.TriangleMesh.from_legacy(mesh_o3d)
    scene = o3d.t.geometry.RaycastingScene()
    scene.add_triangles(mesh_t)
    triangle_normals = np.asarray(mesh_o3d.triangle_normals)
    
    all_ray_starts, all_ray_dirs, ray_theta_indices, all_max_dists = [], [], [], []
    for theta_idx, theta in enumerate(theta_angles):
        for x in x_positions:
            ray_start = np.array([x, center_y + radius*np.cos(theta), center_z + radius*np.sin(theta)])
            ray_end = np.array([x, center_y, center_z])
            ray_vec = ray_end - ray_start
            max_dist = np.linalg.norm(ray_vec)
            ray_dir = ray_vec / max_dist if max_dist > 1e-10 else np.array([0,0,0])
            all_ray_starts.append(ray_start)
            all_ray_dirs.append(ray_dir)
            ray_theta_indices.append(theta_idx)
            all_max_dists.append(max_dist)
    
    all_ray_starts = np.array(all_ray_starts, dtype=np.float32)
    all_ray_dirs = np.array(all_ray_dirs, dtype=np.float32)
    ray_theta_indices = np.array(ray_theta_indices)
    all_max_dists = np.array(all_max_dists, dtype=np.float32)
    
    print(f"\n[INFO] 레이캐스팅 시작...")
    rays = np.hstack([all_ray_starts, all_ray_dirs]).astype(np.float32)
    rays_tensor = o3d.core.Tensor(rays, dtype=o3d.core.Dtype.Float32)
    result = scene.cast_rays(rays_tensor)
    t_hit = result['t_hit'].numpy()
    primitive_ids = result['primitive_ids'].numpy()
    print(f"[INFO] 레이캐스팅 완료")
    
    hit_mask = np.isfinite(t_hit) & (t_hit <= all_max_dists)
    hit_points = all_ray_starts + all_ray_dirs * t_hit[:, np.newaxis]
    
    min_x_points, min_x_normals = [], []
    for theta_idx in range(num_theta):
        theta_mask = (ray_theta_indices == theta_idx) & hit_mask
        if np.any(theta_mask):
            theta_hits = hit_points[theta_mask]
            theta_prim_ids = primitive_ids[theta_mask]
            min_idx = np.argmin(theta_hits[:, 0])
            min_point = theta_hits[min_idx]
            tri_id = theta_prim_ids[min_idx]
            tri_normal = triangle_normals[tri_id].copy() if tri_id < len(triangle_normals) else np.array([0.,0.,1.])
            if tri_normal[2] < 0:
                tri_normal = -tri_normal
            min_x_points.append(min_point)
            min_x_normals.append(tri_normal)
    
    print(f"\n[결과] 히트 레이: {np.sum(hit_mask)}개, X 최소 모서리 점: {len(min_x_points)}개")
    if len(min_x_points) == 0:
        return np.array([]).reshape(0,3), np.array([]).reshape(0,3)
    
    min_x_points, min_x_normals = np.array(min_x_points), np.array(min_x_normals)
    center = np.mean(min_x_points, axis=0)
    
    flipped = 0
    for i in range(len(min_x_normals)):
        to_center = center - min_x_points[i]
        to_center_norm = to_center / (np.linalg.norm(to_center) + 1e-10)
        if np.dot(min_x_normals[i], to_center_norm) > 0:
            min_x_normals[i] = -min_x_normals[i]
            flipped += 1
    print(f"[INFO] 법선 방향 보정: {flipped}개 뒤집음")
    
    print(f"[INFO] 시작점 오프셋: 띄우기 {lift_distance*100:.1f}cm, X+ 이동 {x_offset*100:.1f}cm")
    final_points, final_normals = [], []
    
    for i in range(len(min_x_points)):
        pt, norm = min_x_points[i], min_x_normals[i]
        lifted = pt + norm * lift_distance
        move_dir = get_perpendicular_xplus_direction(norm)
        new_air = lifted + move_dir * x_offset
        
        rays = o3d.core.Tensor([[*new_air, *(-norm)]], dtype=o3d.core.Dtype.Float32)
        res = scene.cast_rays(rays)
        t_val, prim_id = res['t_hit'].numpy()[0], res['primitive_ids'].numpy()[0]
        
        if np.isfinite(t_val) and t_val <= lift_distance * 3:
            hit_pt = new_air + (-norm) * t_val
            new_norm = triangle_normals[prim_id].copy() if prim_id < len(triangle_normals) else norm.copy()
            to_c = center - hit_pt
            if np.dot(new_norm, to_c/(np.linalg.norm(to_c)+1e-10)) > 0:
                new_norm = -new_norm
            final_points.append(hit_pt)
            final_normals.append(new_norm)
        else:
            final_points.append(pt)
            final_normals.append(norm)
    
    final_points, final_normals = np.array(final_points), np.array(final_normals)
    print(f"[INFO] 시작점 오프셋 완료: {len(final_points)}개")
    
    if trim_start + trim_end > 0:
        orig = len(final_points)
        z_sorted = np.argsort(final_points[:, 2])
        remove_set = set(z_sorted[:trim_start + trim_end])
        keep = np.array([i not in remove_set for i in range(len(final_points))])
        final_points, final_normals = final_points[keep], final_normals[keep]
        print(f"[INFO] Z축 최하단 {trim_start+trim_end}개 제거 → {orig} → {len(final_points)}개")
    
    return final_points, final_normals


# ============================================================
# [2단계] 경로 생성 함수
# ============================================================

def generate_path_along_surface(mesh, start_point, start_normal, lift_distance=0.1,
                                 step_distance=0.05, num_points=5,
                                 normal_angle_threshold=30.0, path_angle_threshold=45.0, verbose=False):
    """시작점에서 법선 수직 + X+ 방향으로 곡면 따라 경로 생성"""
    hit_points, hit_normals = [start_point.copy()], [start_normal.copy()]
    current_point, current_normal = start_point.copy(), start_normal.copy()
    
    for step in range(num_points - 1):
        lifted = current_point + current_normal * lift_distance
        move_dir = get_perpendicular_xplus_direction(current_normal)
        new_air = lifted + move_dir * step_distance
        ray_end = new_air - current_normal * (lift_distance * 3)
        
        intersections, cells = mesh.ray_trace(new_air, ray_end)
        
        if len(intersections) == 0:
            new_pt = current_point + move_dir * step_distance
            hit_points.append(new_pt)
            hit_normals.append(current_normal.copy())
            current_point = new_pt
            continue
        
        hit_pt, cell_id = intersections[0], cells[0]
        
        if len(hit_points) >= 3:
            is_outlier, angle = check_path_direction_change(hit_points[-2], hit_points[-1], hit_pt, path_angle_threshold)
            if is_outlier:
                prev_dir = hit_points[-1] - hit_points[-2]
                prev_len = np.linalg.norm(prev_dir)
                predicted = hit_points[-1] + (prev_dir/prev_len if prev_len > 1e-10 else move_dir) * step_distance
                hit_points.append(predicted)
                hit_normals.append(current_normal.copy())
                current_point = predicted
                continue
        
        cell = mesh.get_cell(cell_id)
        tri_pts = cell.points
        new_normal = np.cross(tri_pts[1]-tri_pts[0], tri_pts[2]-tri_pts[0])
        norm_len = np.linalg.norm(new_normal)
        new_normal = new_normal/norm_len if norm_len > 1e-10 else current_normal.copy()
        if np.dot(new_normal, current_normal) < 0:
            new_normal = -new_normal
        
        if step >= 1:
            is_outlier, _ = check_normal_angle_change(new_normal, current_normal, normal_angle_threshold)
            if is_outlier:
                new_normal = current_normal.copy()
        
        hit_points.append(hit_pt)
        hit_normals.append(new_normal)
        current_point, current_normal = hit_pt, new_normal
    
    return np.array(hit_points), np.array(hit_normals)


# ============================================================
# [3단계] 로봇 좌표 변환 - 표면점 포함
# ============================================================

def convert_to_robot_pose(points, normals, offset_distance=0.15):
    """표면 점들을 로봇 엔드이펙터 좌표로 변환
    반환: (로봇 위치, 표면 접촉점)
    """
    robot_positions = []
    surface_points = []
    
    for pt, norm in zip(points, normals):
        # 로봇 위치 = 표면점 + 법선방향 오프셋
        robot_pos = pt + norm * offset_distance
        robot_positions.append(robot_pos)
        # 표면 접촉점 = 원래 점
        surface_points.append(pt.copy())
    
    return np.array(robot_positions), np.array(surface_points)


def transpose_and_save_json(all_robot_positions, all_surface_points,
                            all_points, output_path="robot_path.json"):
    """행 기준 데이터를 열 기준으로 전치하여 JSON 저장
    출력 형식: [robot_x, robot_y, robot_z, surface_x, surface_y, surface_z]
    """
    num_lines = len(all_points)
    if num_lines == 0:
        print("[WARN] 저장할 데이터가 없습니다.")
        return
    
    num_points_per_line = len(all_points[0])
    
    # 라인별로 데이터 분리
    robot_pos_per_line, surface_pts_per_line = [], []
    idx = 0
    for pts in all_points:
        n = len(pts)
        robot_pos_per_line.append(all_robot_positions[idx:idx+n])
        surface_pts_per_line.append(all_surface_points[idx:idx+n])
        idx += n
    
    # 열 기준으로 전치
    columns = []
    for col_idx in range(num_points_per_line):
        column_points = []
        for line_idx in range(num_lines):
            if col_idx < len(robot_pos_per_line[line_idx]):
                pos = robot_pos_per_line[line_idx][col_idx]
                surf = surface_pts_per_line[line_idx][col_idx]
                # [로봇위치(3) + 표면점(3)]
                point_data = [
                    float(pos[0]), float(pos[1]), float(pos[2]),
                    float(surf[0]), float(surf[1]), float(surf[2])
                ]
                column_points.append(point_data)
        columns.append(column_points)
    
    total_points = sum(len(col) for col in columns)
    
    output_data = {
        "description": "각 점: [robot_x, robot_y, robot_z, surface_x, surface_y, surface_z]",
        "num_lines": len(columns),
        "total_points": total_points,
        "lines": columns
    }
    
    with open(output_path, 'w', encoding='utf-8') as f:
        json.dump(output_data, f, indent=2, ensure_ascii=False)
    
    print(f"\n[JSON 저장] {output_path}")
    print(f"  - 형식: [로봇위치(3) + 표면접촉점(3)]")
    print(f"  - 열 개수: {len(columns)}")
    print(f"  - 총 점 개수: {total_points}")


# ============================================================
# [4단계] 시각화 - 로봇위치↔표면점 연결선 표시
# ============================================================

def visualize_result(stl_path, start_points, all_points, all_normals,
                     robot_positions=None, surface_points=None,
                     normal_scale=0.03, debug_mode=False,
                     original_robot_positions=None, robot_line_info=None):
    """결과 시각화 - 로봇 위치와 표면 점을 연결선으로 표시

    Args:
        original_robot_positions: 정규화 전 원본 로봇 포지션 (비교 시각화용)
        robot_line_info: 로봇 포지션 정규화 라인 정보 (center, direction 등)
    """
    mesh = pv.read(stl_path)
    p = pv.Plotter()
    p.set_background('white')
    p.add_mesh(mesh, color='lightblue', opacity=0.5)
    
    # 시작점 (녹색)
    if len(start_points) > 0:
        p.add_mesh(pv.PolyData(start_points), color='green', point_size=15,
                   render_points_as_spheres=True, label='시작점')

        if debug_mode and len(start_points) >= 2:
            p.add_mesh(pv.PolyData(start_points[0:1]), color='red', point_size=25, render_points_as_spheres=True)
            p.add_point_labels(pv.PolyData(start_points[0:1]), ["[0] 첫점"], font_size=20, text_color='red')
            p.add_mesh(pv.PolyData(start_points[-1:]), color='blue', point_size=25, render_points_as_spheres=True)
            p.add_point_labels(pv.PolyData(start_points[-1:]), [f"[{len(start_points)-1}] 끝점"], font_size=20, text_color='blue')
    
    # 경로 점들 (빨간색) + 법선 화살표 (파란색)
    all_pts_combined, all_norms_combined = [], []
    for pts, norms in zip(all_points, all_normals):
        if len(pts) > 0:
            all_pts_combined.extend(pts)
            all_norms_combined.extend(norms)
    
    if len(all_pts_combined) > 0:
        all_pts_arr = np.array(all_pts_combined)
        all_norms_arr = np.array(all_norms_combined)
        p.add_mesh(pv.PolyData(all_pts_arr), color='red', point_size=10, render_points_as_spheres=True)
        arrows = pv.PolyData(all_pts_arr)
        arrows['vectors'] = all_norms_arr * normal_scale
        p.add_mesh(arrows.glyph(orient='vectors', scale=False, factor=normal_scale), color='blue')
    
    # 정규화 전 원본 로봇 포지션 (회색, 작은 점) - 비교용
    if original_robot_positions is not None and len(original_robot_positions) > 0:
        p.add_mesh(pv.PolyData(original_robot_positions), color='gray', point_size=8,
                   render_points_as_spheres=True, opacity=0.5, label='원본 로봇 포지션')

        # 원본 → 정규화 이동 연결선
        if robot_positions is not None and len(robot_positions) == len(original_robot_positions):
            move_lines_pts = []
            move_lines_cells = []
            for i in range(len(original_robot_positions)):
                idx = len(move_lines_pts)
                move_lines_pts.append(original_robot_positions[i])
                move_lines_pts.append(robot_positions[i])
                move_lines_cells.extend([2, idx, idx + 1])
            move_lines_pts = np.array(move_lines_pts)
            move_lines_cells = np.array(move_lines_cells)
            move_mesh = pv.PolyData(move_lines_pts, lines=move_lines_cells)
            p.add_mesh(move_mesh, color='lightgray', line_width=1, opacity=0.4)

    # 로봇 포지션 정규화 라인 시각화 (중심점 + 방향 표시)
    if robot_line_info is not None:
        center = robot_line_info['center']
        direction = robot_line_info['direction']

        # 중심점 (큰 금색 점)
        p.add_mesh(pv.PolyData([center]), color='gold', point_size=20,
                   render_points_as_spheres=True, label='로봇 포지션 중심')

        # 주 방향 라인 (양쪽으로 확장)
        line_length = 0.15
        line_start = center - direction * line_length
        line_end = center + direction * line_length
        line_pts = np.array([line_start, line_end])
        line_cells = np.array([2, 0, 1])
        center_line = pv.PolyData(line_pts, lines=line_cells)
        p.add_mesh(center_line, color='gold', line_width=4, label='정규화 축')

    # 로봇 위치 (노란색) + 표면점 연결선 (초록색)
    if robot_positions is not None and len(robot_positions) > 0:
        p.add_mesh(pv.PolyData(robot_positions), color='yellow', point_size=12,
                   render_points_as_spheres=True, label='정규화된 로봇 포지션')

        if surface_points is not None and len(surface_points) == len(robot_positions):
            # 로봇위치 → 표면점 연결선 생성
            line_points = []
            line_cells = []
            for i in range(len(robot_positions)):
                idx = len(line_points)
                line_points.append(robot_positions[i])
                line_points.append(surface_points[i])
                line_cells.extend([2, idx, idx+1])

            line_points = np.array(line_points)
            line_cells = np.array(line_cells)
            lines_mesh = pv.PolyData(line_points, lines=line_cells)
            p.add_mesh(lines_mesh, color='darkgreen', line_width=2, label='Robot→Surface')

            # 표면점도 별도 표시 (주황색, 작은 점)
            p.add_mesh(pv.PolyData(surface_points), color='orange', point_size=8, render_points_as_spheres=True)
    
    # 경로 연결선
    colors = ['red', 'orange', 'purple', 'cyan', 'magenta']
    for i, pts in enumerate(all_points):
        if len(pts) > 1:
            n = len(pts)
            lines = np.column_stack([np.full(n-1, 2), np.arange(n-1), np.arange(1, n)]).flatten()
            p.add_mesh(pv.PolyData(pts, lines=lines), color=colors[i % len(colors)], line_width=2)
    
    p.add_axes()
    p.camera_position = 'iso'
    p.show()


# ============================================================
# 메인 함수
# ============================================================

def main():
    # ==================== 파라미터 ====================
    STL_FILE = "gene1.stl"
    
    # [1단계] 반구 레이캐스팅 파라미터
    X_EXTEND = 0.15
    LINE_SPACING = 0.00005
    POINTS_PER_LINE = 80
    RADIUS_SCALE = 1.2
    START_OFFSET = 0.02
    START_LIFT = 0.1
    TRIM_START = 0
    TRIM_END = 0
    
    # [1.5단계] 시작점 균일 샘플링 파라미터
    MIN_DISTANCE = 0.05

    # [2단계] 경로 생성 파라미터
    PATH_LIFT_DISTANCE = 0.1
    STEP_DISTANCE = 0.05
    NUM_POINTS = 4
    NORMAL_ANGLE_THRESHOLD = 30.0
    PATH_ANGLE_THRESHOLD = 20.0
    
    # [3단계] 로봇 좌표 변환 파라미터
    ROBOT_OFFSET = 0.20

    # [3.5단계] 로봇 타겟 포지션 정규화 파라미터
    NORMALIZE_ROBOT_POSITIONS = True  # 로봇 타겟 포지션 라인 정규화 활성화

    # [4단계] 시각화 파라미터
    NORMAL_SCALE = 0.03
    
    VERBOSE = False
    DEBUG_MODE = False
    # =================================================
    
    print("\n" + "="*60)
    print("경로 생성기 v4.1 - 자동 시작점 버전 (표면점 출력)")
    print("="*60)
    print("\n[파라미터 설정]")
    print(f"  [1단계] X 확장: {X_EXTEND}m, 라인 간격: {LINE_SPACING}m")
    print(f"  [1.5단계] 시작점 최소 거리: {MIN_DISTANCE*1000:.1f}mm")
    print(f"  [2단계] 레이캐스팅 높이: {PATH_LIFT_DISTANCE}m, 스텝: {STEP_DISTANCE}m, 점 개수: {NUM_POINTS}개")
    print(f"  [3단계] 로봇 오프셋: {ROBOT_OFFSET}m")
    
    # 1단계: 시작점 추출
    start_points, start_normals = extract_edge_points_with_normals(
        STL_FILE, X_EXTEND, LINE_SPACING, POINTS_PER_LINE, RADIUS_SCALE,
        START_OFFSET, START_LIFT, TRIM_START, TRIM_END
    )
    
    if len(start_points) == 0:
        print("[ERROR] 시작점이 추출되지 않았습니다.")
        return
    
    print(f"\n[1단계 완료] 자동 추출된 시작점: {len(start_points)}개")
    
    # 1.5단계: 균일 샘플링
    print("\n" + "="*60)
    print("[1.5단계] 시작점 균일 샘플링")
    print("="*60)
    
    orig_count = len(start_points)
    sampled = [0]
    last_pt = start_points[0]
    for i in range(1, len(start_points)):
        if np.linalg.norm(start_points[i] - last_pt) >= MIN_DISTANCE:
            sampled.append(i)
            last_pt = start_points[i]
    
    start_points, start_normals = start_points[sampled], start_normals[sampled]
    print(f"[샘플링] {orig_count} → {len(start_points)}개")

    # 2단계: 경로 생성
    print("\n" + "="*60)
    print("[2단계] 경로 생성")
    print("="*60)
    
    mesh = pv.read(STL_FILE)
    all_points, all_normals = [], []
    
    for pt, norm in zip(start_points, start_normals):
        pts, norms = generate_path_along_surface(
            mesh, pt, norm, PATH_LIFT_DISTANCE, STEP_DISTANCE, NUM_POINTS,
            NORMAL_ANGLE_THRESHOLD, PATH_ANGLE_THRESHOLD, VERBOSE
        )
        all_points.append(pts)
        all_normals.append(norms)
    
    print(f"\n[2단계 완료] {len(all_points)}개 경로 생성")
    
    # 3단계: 로봇 좌표 변환
    print("\n" + "="*60)
    print("[3단계] 로봇 좌표 변환")
    print("="*60)
    
    all_robot_positions, all_surface_points = [], []
    for pts, norms in zip(all_points, all_normals):
        robot_pos, surf_pts = convert_to_robot_pose(pts, norms, ROBOT_OFFSET)
        all_robot_positions.extend(robot_pos)
        all_surface_points.extend(surf_pts)
    
    all_robot_positions = np.array(all_robot_positions)
    all_surface_points = np.array(all_surface_points)
    print(f"[3단계 완료] 로봇 위치 {len(all_robot_positions)}개 변환")

    # 3.5단계: 로봇 타겟 포지션 정규화
    original_robot_positions = None
    robot_line_info = None

    if NORMALIZE_ROBOT_POSITIONS and len(all_robot_positions) >= 2:
        print("\n" + "="*60)
        print("[3.5단계] 로봇 타겟 포지션 정규화 (PCA 기반)")
        print("="*60)
        print("  - 표면 접촉점은 이미 일직선이므로 변경하지 않음")
        print("  - 로봇 포지션만 중심 라인으로 정규화")

        # 정규화 전 원본 저장 (시각화 비교용)
        original_robot_positions = all_robot_positions.copy()

        # 로봇 포지션 정규화 수행
        all_robot_positions, all_surface_points, robot_line_info = normalize_robot_positions_to_line(
            all_robot_positions, all_surface_points
        )

        if robot_line_info:
            print(f"[3.5단계 완료] 정규화된 로봇 포지션: {len(all_robot_positions)}개")
            print(f"  - 평균 편차: {robot_line_info['mean_deviation']*1000:.2f}mm")
            print(f"  - 최대 편차: {robot_line_info['max_deviation']*1000:.2f}mm")

    # Y값 기준 재정렬
    print("\n" + "="*60)
    print("[인덱스 재정렬] Y=0 → Y+ 방향")
    print("="*60)
    
    if len(all_points) > 0:
        start_y = np.array([pts[0][1] for pts in all_points])
        sorted_idx = np.argsort(start_y)
        all_points = [all_points[i] for i in sorted_idx]
        all_normals = [all_normals[i] for i in sorted_idx]
        start_points = start_points[sorted_idx]
        start_normals = start_normals[sorted_idx]
        print(f"[정렬 완료] 첫점 Y={all_points[0][0][1]:.4f} → 끝점 Y={all_points[-1][0][1]:.4f}")
    
    # 로봇 좌표 재생성
    all_robot_positions, all_surface_points = [], []
    for pts, norms in zip(all_points, all_normals):
        robot_pos, surf_pts = convert_to_robot_pose(pts, norms, ROBOT_OFFSET)
        all_robot_positions.extend(robot_pos)
        all_surface_points.extend(surf_pts)

    all_robot_positions = np.array(all_robot_positions) if all_robot_positions else np.array([]).reshape(0,3)
    all_surface_points = np.array(all_surface_points) if all_surface_points else np.array([]).reshape(0,3)
    print(f"[재정렬 후] 총 점 개수: {len(all_robot_positions)}개")

    # 재정렬 후 로봇 포지션 정규화 다시 적용
    if NORMALIZE_ROBOT_POSITIONS and len(all_robot_positions) >= 2:
        print("\n[재정렬 후 로봇 포지션 정규화 재적용]")
        original_robot_positions = all_robot_positions.copy()
        all_robot_positions, all_surface_points, robot_line_info = normalize_robot_positions_to_line(
            all_robot_positions, all_surface_points
        )
    
    # JSON 저장
    transpose_and_save_json(all_robot_positions, all_surface_points, all_points, "robot_path.json")
    
    # 4단계: 시각화
    print("\n" + "="*60)
    print("[4단계] 시각화")
    print("="*60)
    
    visualize_result(
        STL_FILE, start_points, all_points, all_normals,
        robot_positions=all_robot_positions,
        surface_points=all_surface_points,
        normal_scale=NORMAL_SCALE,
        debug_mode=DEBUG_MODE,
        original_robot_positions=original_robot_positions if NORMALIZE_ROBOT_POSITIONS else None,
        robot_line_info=robot_line_info if NORMALIZE_ROBOT_POSITIONS else None
    )


if __name__ == '__main__':
    main()