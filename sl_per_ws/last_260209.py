"""
경로 생성기 v4.2 - Vertex Normal 보간 + 오프셋 스무딩 통합
- v4.1: Face Normal → Vertex Normal (Barycentric 보간)
- v4.2: 법선 스무딩 → 오프셋 → B-Spline 피팅 파이프라인 추가
"""

import numpy as np
import open3d as o3d
import pyvista as pv
import json
from scipy.ndimage import uniform_filter1d, gaussian_filter1d
from scipy.interpolate import splprep, splev


# ============================================================
# [신규] Barycentric 좌표 및 Vertex Normal 보간 함수
# ============================================================

def compute_barycentric_coordinates(point, tri_vertices):
    """삼각형 내 점의 Barycentric 좌표 계산"""
    v0 = tri_vertices[0]
    v1 = tri_vertices[1]
    v2 = tri_vertices[2]
    
    edge1 = v1 - v0
    edge2 = v2 - v0
    
    normal = np.cross(edge1, edge2)
    normal_len_sq = np.dot(normal, normal)
    
    if normal_len_sq < 1e-20:
        return (1/3, 1/3, 1/3)
    
    p = point - v0
    
    d00 = np.dot(edge1, edge1)
    d01 = np.dot(edge1, edge2)
    d11 = np.dot(edge2, edge2)
    d20 = np.dot(p, edge1)
    d21 = np.dot(p, edge2)
    
    denom = d00 * d11 - d01 * d01
    
    if abs(denom) < 1e-20:
        return (1/3, 1/3, 1/3)
    
    v = (d11 * d20 - d01 * d21) / denom
    w = (d00 * d21 - d01 * d20) / denom
    u = 1.0 - v - w
    
    u = np.clip(u, 0.0, 1.0)
    v = np.clip(v, 0.0, 1.0)
    w = np.clip(w, 0.0, 1.0)
    
    total = u + v + w
    if total > 0:
        u, v, w = u/total, v/total, w/total
    
    return (u, v, w)


def interpolate_vertex_normal(hit_point, tri_vertices, vertex_normals):
    """Barycentric 보간으로 Vertex Normal 계산"""
    u, v, w = compute_barycentric_coordinates(hit_point, tri_vertices)
    
    interpolated = (u * vertex_normals[0] + 
                    v * vertex_normals[1] + 
                    w * vertex_normals[2])
    
    norm_len = np.linalg.norm(interpolated)
    if norm_len > 1e-10:
        interpolated = interpolated / norm_len
    else:
        edge1 = tri_vertices[1] - tri_vertices[0]
        edge2 = tri_vertices[2] - tri_vertices[0]
        interpolated = np.cross(edge1, edge2)
        interpolated = interpolated / (np.linalg.norm(interpolated) + 1e-10)
    
    return interpolated


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


# ============================================================
# [1단계] 반구 레이캐스팅 - Vertex Normal 보간 버전
# ============================================================

def extract_edge_points_with_normals(stl_path: str,
                                      x_extend: float = 0.2,
                                      line_spacing: float = 0.0005,
                                      points_per_line: int = 180,
                                      radius_scale: float = 1.2,
                                      x_offset: float = 0.03,
                                      lift_distance: float = 0.1,
                                      trim_start: int = 0,
                                      trim_end: int = 0):
    """반구 레이캐스팅으로 X 최소 모서리 점들과 Vertex Normal 보간 법선 추출"""
    print("\n" + "="*60)
    print("[1단계] 반구 레이캐스팅 - Vertex Normal 보간")
    print("="*60)
    
    mesh_o3d = o3d.io.read_triangle_mesh(stl_path)
    mesh_o3d.compute_vertex_normals()
    mesh_o3d.compute_triangle_normals()
    
    bbox = mesh_o3d.get_axis_aligned_bounding_box()
    min_bound = np.array(bbox.min_bound)
    max_bound = np.array(bbox.max_bound)
    
    size_x = max_bound[0] - min_bound[0]
    size_y = max_bound[1] - min_bound[1]
    size_z = max_bound[2] - min_bound[2]
    
    print(f"\n[INFO] 메시 크기: X={size_x:.4f}, Y={size_y:.4f}, Z={size_z:.4f}")
    
    radius = max(size_y, size_z) / 2 * radius_scale
    center_x = (min_bound[0] + max_bound[0]) / 2
    center_y = (min_bound[1] + max_bound[1]) / 2
    center_z = (min_bound[2] + max_bound[2]) / 2
    
    print(f"[INFO] 중심: ({center_x:.4f}, {center_y:.4f}, {center_z:.4f})")
    print(f"[INFO] 반지름: {radius:.4f}")
    
    x_min = min_bound[0] - x_extend
    x_max = max_bound[0] + x_extend
    x_positions = np.arange(x_min, x_max + line_spacing / 2, line_spacing)
    
    theta_angles = np.linspace(0, 2 * np.pi, points_per_line, endpoint=False)
    
    num_x = len(x_positions)
    num_theta = len(theta_angles)
    
    print(f"[INFO] X 라인 수: {num_x}, theta 점 수: {num_theta}")
    print(f"[INFO] 총 레이 수: {num_x * num_theta}")
    
    mesh_t = o3d.t.geometry.TriangleMesh.from_legacy(mesh_o3d)
    scene = o3d.t.geometry.RaycastingScene()
    scene.add_triangles(mesh_t)
    
    vertex_normals = np.asarray(mesh_o3d.vertex_normals)
    vertices = np.asarray(mesh_o3d.vertices)
    triangles = np.asarray(mesh_o3d.triangles)
    
    print(f"[INFO] Vertex Normal 사용: {len(vertex_normals)}개 정점")
    
    all_ray_starts = []
    all_ray_dirs = []
    ray_theta_indices = []
    all_max_dists = []
    
    for theta_idx, theta in enumerate(theta_angles):
        for x in x_positions:
            ray_start = np.array([
                x,
                center_y + radius * np.cos(theta),
                center_z + radius * np.sin(theta)
            ])
            ray_end = np.array([x, center_y, center_z])
            
            ray_vec = ray_end - ray_start
            max_dist = np.linalg.norm(ray_vec)
            ray_dir = ray_vec / max_dist if max_dist > 1e-10 else np.array([0, 0, 0])
            
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
    
    min_x_points = []
    min_x_normals = []
    
    for theta_idx in range(num_theta):
        theta_mask = (ray_theta_indices == theta_idx) & hit_mask
        
        if np.any(theta_mask):
            theta_hits = hit_points[theta_mask]
            theta_primitive_ids = primitive_ids[theta_mask]
            
            min_idx = np.argmin(theta_hits[:, 0])
            min_point = theta_hits[min_idx]
            
            tri_id = theta_primitive_ids[min_idx]
            
            if tri_id < len(triangles):
                tri_vertex_ids = triangles[tri_id]
                tri_vertices = vertices[tri_vertex_ids]
                tri_vertex_normals = vertex_normals[tri_vertex_ids]
                
                interpolated_normal = interpolate_vertex_normal(
                    min_point, tri_vertices, tri_vertex_normals
                )
                
                if interpolated_normal[2] < 0:
                    interpolated_normal = -interpolated_normal
            else:
                interpolated_normal = np.array([0.0, 0.0, 1.0])
            
            min_x_points.append(min_point)
            min_x_normals.append(interpolated_normal)
    
    print(f"\n[결과] 히트 레이: {np.sum(hit_mask)}개")
    print(f"[결과] X 최소 모서리 점: {len(min_x_points)}개")
    
    if len(min_x_points) == 0:
        return np.array([]).reshape(0, 3), np.array([]).reshape(0, 3)
    
    min_x_points = np.array(min_x_points)
    min_x_normals = np.array(min_x_normals)
    
    center = np.mean(min_x_points, axis=0)
    
    flipped_count = 0
    for i in range(len(min_x_normals)):
        to_center = center - min_x_points[i]
        to_center_norm = to_center / (np.linalg.norm(to_center) + 1e-10)
        
        if np.dot(min_x_normals[i], to_center_norm) > 0:
            min_x_normals[i] = -min_x_normals[i]
            flipped_count += 1
    
    print(f"[INFO] 법선 방향 보정: {flipped_count}개 뒤집음 (바깥쪽 향하도록)")
    
    print(f"[INFO] 시작점 오프셋 처리 중...")
    print(f"       - 띄우기: {lift_distance*100:.1f}cm")
    print(f"       - 법선⊥+X+ 이동: {x_offset*100:.1f}cm")
    
    final_points = []
    final_normals = []
    
    for i in range(len(min_x_points)):
        pt = min_x_points[i]
        norm = min_x_normals[i]
        
        lifted_point = pt + norm * lift_distance
        move_dir = get_perpendicular_xplus_direction(norm)
        new_air_point = lifted_point + move_dir * x_offset
        
        ray_origin = new_air_point
        ray_direction = -norm
        ray_length = lift_distance * 3
        
        rays = o3d.core.Tensor(
            [[ray_origin[0], ray_origin[1], ray_origin[2],
              ray_direction[0], ray_direction[1], ray_direction[2]]],
            dtype=o3d.core.Dtype.Float32
        )
        
        result = scene.cast_rays(rays)
        t_hit_val = result['t_hit'].numpy()[0]
        prim_id = result['primitive_ids'].numpy()[0]
        
        if np.isfinite(t_hit_val) and t_hit_val <= ray_length:
            hit_point = ray_origin + ray_direction * t_hit_val
            
            if prim_id < len(triangles):
                tri_vertex_ids = triangles[prim_id]
                tri_vertices = vertices[tri_vertex_ids]
                tri_vertex_normals = vertex_normals[tri_vertex_ids]
                
                new_normal = interpolate_vertex_normal(
                    hit_point, tri_vertices, tri_vertex_normals
                )
                
                to_center = center - hit_point
                to_center_norm = to_center / (np.linalg.norm(to_center) + 1e-10)
                if np.dot(new_normal, to_center_norm) > 0:
                    new_normal = -new_normal
            else:
                new_normal = norm.copy()
            
            final_points.append(hit_point)
            final_normals.append(new_normal)
        else:
            print(f"  [WARN] 점 {i}: 레이캐스팅 실패 - 원래 점 사용")
            final_points.append(pt)
            final_normals.append(norm)
    
    final_points = np.array(final_points)
    final_normals = np.array(final_normals)
    
    print(f"[INFO] 시작점 오프셋 완료: {len(final_points)}개")
    
    if trim_start > 0 or trim_end > 0:
        original_count = len(final_points)
        z_sorted_indices = np.argsort(final_points[:, 2])
        num_to_remove = trim_start + trim_end
        indices_to_remove = set(z_sorted_indices[:num_to_remove])
        keep_mask = np.array([i not in indices_to_remove for i in range(len(final_points))])
        final_points = final_points[keep_mask]
        final_normals = final_normals[keep_mask]
        print(f"[INFO] Z축 최하단 점 {num_to_remove}개 제거 → {original_count} → {len(final_points)}개")
    
    return final_points, final_normals


# ============================================================
# [2단계] 경로 생성 함수 - Vertex Normal 보간 버전
# ============================================================

def generate_path_along_surface(mesh_o3d, mesh_pv, start_point, start_normal,
                                 lift_distance=0.1, step_distance=0.05, num_points=5,
                                 normal_angle_threshold=30.0,
                                 path_angle_threshold=45.0,
                                 verbose=False):
    """시작점에서 법선 수직 + X+ 방향으로 곡면 따라 경로 생성 (Vertex Normal 보간)"""
    
    vertex_normals = np.asarray(mesh_o3d.vertex_normals)
    vertices = np.asarray(mesh_o3d.vertices)
    triangles = np.asarray(mesh_o3d.triangles)
    
    hit_points = [start_point.copy()]
    hit_normals = [start_normal.copy()]
    
    current_point = start_point.copy()
    current_normal = start_normal.copy()
    
    if verbose:
        print(f"    [Step 0] 시작점")
    
    for step in range(num_points - 1):
        lifted_point = current_point + current_normal * lift_distance
        move_dir = get_perpendicular_xplus_direction(current_normal)
        new_air_point = lifted_point + move_dir * step_distance
        
        ray_start = new_air_point
        ray_end = new_air_point - current_normal * (lift_distance * 3)
        
        intersection_points, intersection_cells = mesh_pv.ray_trace(ray_start, ray_end)
        
        if len(intersection_points) == 0:
            if verbose:
                print(f"    [Step {step+1}] 히트 없음 - 방향 이동 + 법선 복사")
            new_point = current_point + move_dir * step_distance
            hit_points.append(new_point)
            hit_normals.append(current_normal.copy())
            current_point = new_point
            continue
        
        hit_pt = intersection_points[0]
        cell_id = intersection_cells[0]
        
        if len(hit_points) >= 3:
            prev_point = hit_points[-2]
            curr_point = hit_points[-1]
            
            is_path_outlier, path_angle = check_path_direction_change(
                prev_point, curr_point, hit_pt, angle_threshold=path_angle_threshold
            )
            
            if is_path_outlier:
                if verbose:
                    print(f"    [Step {step+1}] 경로 각도 급변 ({path_angle:.1f}°)")
                
                prev_direction = curr_point - prev_point
                prev_length = np.linalg.norm(prev_direction)
                
                if prev_length > 1e-10:
                    prev_direction = prev_direction / prev_length
                    predicted_point = curr_point + prev_direction * step_distance
                else:
                    predicted_point = curr_point + move_dir * step_distance
                
                hit_points.append(predicted_point)
                hit_normals.append(current_normal.copy())
                current_point = predicted_point
                continue
        
        if cell_id < len(triangles):
            tri_vertex_ids = triangles[cell_id]
            tri_vertices = vertices[tri_vertex_ids]
            tri_vertex_normals = vertex_normals[tri_vertex_ids]
            
            new_normal = interpolate_vertex_normal(
                hit_pt, tri_vertices, tri_vertex_normals
            )
            
            if np.dot(new_normal, current_normal) < 0:
                new_normal = -new_normal
        else:
            new_normal = current_normal.copy()
        
        if step >= 1:
            is_normal_outlier, normal_angle = check_normal_angle_change(
                new_normal, current_normal, angle_threshold=normal_angle_threshold
            )
            
            if is_normal_outlier:
                if verbose:
                    print(f"    [Step {step+1}] 법선 급변 ({normal_angle:.1f}°)")
                new_normal = current_normal.copy()
        
        hit_points.append(hit_pt)
        hit_normals.append(new_normal)
        
        current_point = hit_pt
        current_normal = new_normal
    
    return np.array(hit_points), np.array(hit_normals)


# ============================================================
# [3단계] 로봇 좌표 변환 및 JSON 저장
# ============================================================

def convert_to_robot_pose(points, normals, offset_distance=0.15):
    """표면 점들을 로봇 엔드이펙터 좌표로 변환"""
    positions = []
    directions = []
    
    for pt, norm in zip(points, normals):
        offset_pt = pt + norm * offset_distance
        positions.append(offset_pt)
        look_dir = -norm
        directions.append(look_dir)
    
    return np.array(positions), np.array(directions)


def transpose_and_save_json(all_offset_points_per_col, all_surface_points_per_col,
                            output_path="robot_path.json"):
    """
    열별 스무딩 오프셋점 + 표면점을 JSON 저장
    
    각 점 데이터: [offset_x, offset_y, offset_z, surface_x, surface_y, surface_z]
    
    Parameters:
        all_offset_points_per_col: list of (N, 3) - 열별 오프셋점 (B-Spline 스무딩 결과)
        all_surface_points_per_col: list of (N, 3) - 열별 표면점
        output_path: 저장 경로
    """
    num_cols = len(all_offset_points_per_col)
    if num_cols == 0:
        print("[WARN] 저장할 데이터가 없습니다.")
        return
    
    columns = []
    for col_idx in range(num_cols):
        offset_pts = all_offset_points_per_col[col_idx]
        surface_pts = all_surface_points_per_col[col_idx]
        
        column_points = []
        for i in range(len(offset_pts)):
            op = offset_pts[i]
            sp = surface_pts[i]
            point_data = [
                float(op[0]), float(op[1]), float(op[2]),
                float(sp[0]), float(sp[1]), float(sp[2])
            ]
            column_points.append(point_data)
        columns.append(column_points)
    
    total_points = sum(len(col) for col in columns)
    
    output_data = {
        "num_lines": len(columns),
        "total_points": total_points,
        "format": "offset_x, offset_y, offset_z, surface_x, surface_y, surface_z",
        "lines": columns
    }
    
    with open(output_path, 'w', encoding='utf-8') as f:
        json.dump(output_data, f, indent=2, ensure_ascii=False)
    
    print(f"\n[JSON 저장] {output_path}")
    print(f"  - 열 개수: {len(columns)}")
    print(f"  - 총 점 개수: {total_points}")
    print(f"  - 형식: [띄운점 x,y,z, 닿는점 x,y,z]")


# ============================================================
# [스무딩] 법선 스무딩 + 오프셋 + B-Spline 피팅
# ============================================================

def smooth_normals(normals, method='gaussian', window_size=5, sigma=2.0):
    """
    법선 벡터 배열을 스무딩
    
    Parameters:
        normals: (N, 3) 법선 벡터 배열
        method: 'gaussian' 또는 'moving_avg'
        window_size: 이동평균 윈도우 크기 (홀수 권장)
        sigma: 가우시안 표준편차
    
    Returns:
        (N, 3) 스무딩된 법선 벡터 (정규화됨)
    """
    smoothed = np.zeros_like(normals)
    
    for axis in range(3):
        if method == 'gaussian':
            smoothed[:, axis] = gaussian_filter1d(normals[:, axis], sigma=sigma)
        elif method == 'moving_avg':
            smoothed[:, axis] = uniform_filter1d(normals[:, axis], size=window_size)
        else:
            raise ValueError(f"Unknown method: {method}")
    
    norms = np.linalg.norm(smoothed, axis=1, keepdims=True)
    norms = np.where(norms < 1e-10, 1.0, norms)
    smoothed = smoothed / norms
    
    return smoothed


def offset_points_along_normals(points, normals, offset_distance=0.20):
    """표면 점들을 법선 방향으로 오프셋"""
    return points + normals * offset_distance


def fit_bspline(points, num_output_points=None, smoothing_factor=None, spline_degree=3):
    """
    3D 점들에 B-Spline 피팅
    
    Parameters:
        points: (N, 3) 입력 점
        num_output_points: 출력 점 개수 (None이면 입력과 동일)
        smoothing_factor: 스플라인 평활도 (None=자동, 0=보간, 클수록 부드러움)
        spline_degree: 스플라인 차수 (기본 3 = cubic)
    
    Returns:
        fitted_points: (M, 3) 피팅된 점
        spline_params: (tck, u) splprep 결과
    """
    if num_output_points is None:
        num_output_points = len(points)
    
    n = len(points)
    if n < spline_degree + 1:
        print(f"[WARN] 점 개수({n})가 spline_degree({spline_degree})+1 보다 적음 → 원본 반환")
        return points.copy(), None
    
    try:
        tck, u = splprep(
            [points[:, 0], points[:, 1], points[:, 2]],
            s=smoothing_factor,
            k=spline_degree
        )
    except Exception as e:
        print(f"[WARN] B-Spline 피팅 실패: {e} → 원본 반환")
        return points.copy(), None
    
    u_new = np.linspace(0, 1, num_output_points)
    fitted = splev(u_new, tck)
    fitted_points = np.column_stack(fitted)
    
    return fitted_points, (tck, u)


def smooth_offset_pipeline(surface_points, raw_normals,
                           offset_distance=0.20,
                           normal_smooth_method='gaussian',
                           normal_smooth_sigma=2.0,
                           normal_smooth_window=5,
                           spline_smoothing=None,
                           spline_num_points=None,
                           spline_degree=3):
    """
    전체 스무딩 파이프라인
    
    흐름: 원본 법선 → 가우시안 스무딩 → 오프셋 → B-Spline 피팅
    
    Returns:
        dict:
            'raw_offset': 원본 법선 오프셋 점 (비교용)
            'smoothed_normals': 스무딩된 법선
            'smoothed_offset': 스무딩 법선 오프셋 점
            'spline_points': B-Spline 피팅 결과 (최종)
            'spline_params': (tck, u)
    """
    n = len(surface_points)
    print(f"\n[스무딩 파이프라인] 입력 점: {n}개, 오프셋: {offset_distance*100:.0f}cm")
    
    # (1) 원본 오프셋 (비교용)
    raw_offset = offset_points_along_normals(surface_points, raw_normals, offset_distance)
    print(f"  [Step 1] 원본 오프셋 완료")
    
    # (2) 법선 스무딩
    smoothed_normals = smooth_normals(
        raw_normals,
        method=normal_smooth_method,
        window_size=normal_smooth_window,
        sigma=normal_smooth_sigma
    )
    
    dot_products = np.sum(raw_normals * smoothed_normals, axis=1)
    angle_diffs = np.degrees(np.arccos(np.clip(dot_products, -1, 1)))
    print(f"  [Step 2] 법선 스무딩 완료 ({normal_smooth_method}, "
          f"{'sigma=' + str(normal_smooth_sigma) if normal_smooth_method == 'gaussian' else 'window=' + str(normal_smooth_window)})")
    print(f"           법선 변화: 평균 {np.mean(angle_diffs):.2f}°, "
          f"최대 {np.max(angle_diffs):.2f}°")
    
    # (3) 스무딩 법선으로 오프셋
    smoothed_offset = offset_points_along_normals(surface_points, smoothed_normals, offset_distance)
    print(f"  [Step 3] 스무딩 법선 오프셋 완료")
    
    # (4) B-Spline 피팅
    spline_points, spline_params = fit_bspline(
        smoothed_offset,
        num_output_points=spline_num_points,
        smoothing_factor=spline_smoothing,
        spline_degree=spline_degree
    )
    
    if len(spline_points) == len(smoothed_offset):
        deviations = np.linalg.norm(spline_points - smoothed_offset, axis=1)
        print(f"  [Step 4] B-Spline 피팅 완료 (s={spline_smoothing}, k={spline_degree})")
        print(f"           편차: 평균 {np.mean(deviations)*1000:.2f}mm, "
              f"최대 {np.max(deviations)*1000:.2f}mm")
    else:
        print(f"  [Step 4] B-Spline 피팅 완료 ({len(spline_points)}개 리샘플링)")
    
    return {
        'raw_offset': raw_offset,
        'smoothed_normals': smoothed_normals,
        'smoothed_offset': smoothed_offset,
        'spline_points': spline_points,
        'spline_params': spline_params
    }


# ============================================================
# [4단계] 시각화
# ============================================================

def visualize_result(stl_path, start_points, all_points, all_normals,
                     robot_positions=None, robot_directions=None,
                     normal_scale=0.03, robot_arrow_scale=0.05,
                     debug_mode=False):
    """전체 결과 시각화"""
    mesh = pv.read(stl_path)
    
    p = pv.Plotter()
    p.set_background('white')
    p.add_mesh(mesh, color='lightblue', opacity=0.5)
    
    if len(start_points) > 0:
        p.add_mesh(pv.PolyData(start_points), color='green',
                   point_size=15, render_points_as_spheres=True)
        
        if debug_mode and len(start_points) >= 2:
            p.add_mesh(pv.PolyData(start_points[0:1]), color='red',
                       point_size=25, render_points_as_spheres=True)
            p.add_point_labels(pv.PolyData(start_points[0:1]), 
                              ["[0] 첫점"], font_size=20, text_color='red',
                              point_size=1, shape_opacity=0.5)
            
            p.add_mesh(pv.PolyData(start_points[-1:]), color='blue',
                       point_size=25, render_points_as_spheres=True)
            p.add_point_labels(pv.PolyData(start_points[-1:]), 
                              [f"[{len(start_points)-1}] 끝점"], font_size=20, text_color='blue',
                              point_size=1, shape_opacity=0.5)
            
            step = max(1, len(start_points) // 10)
            for i in range(0, len(start_points), step):
                if i != 0 and i != len(start_points) - 1:
                    p.add_point_labels(pv.PolyData(start_points[i:i+1]),
                                      [f"[{i}]"], font_size=12, text_color='black',
                                      point_size=1, shape_opacity=0.3)
    
    all_pts_combined = []
    all_norms_combined = []
    for pts, norms in zip(all_points, all_normals):
        if len(pts) > 0:
            all_pts_combined.extend(pts)
            all_norms_combined.extend(norms)
    
    if len(all_pts_combined) > 0:
        all_pts_arr = np.array(all_pts_combined)
        all_norms_arr = np.array(all_norms_combined)
        
        p.add_mesh(pv.PolyData(all_pts_arr), color='red',
                   point_size=10, render_points_as_spheres=True)
        
        arrows = pv.PolyData(all_pts_arr)
        arrows['vectors'] = all_norms_arr * normal_scale
        glyphs = arrows.glyph(orient='vectors', scale=False, factor=normal_scale)
        p.add_mesh(glyphs, color='blue')
    
    if robot_positions is not None and len(robot_positions) > 0:
        p.add_mesh(pv.PolyData(robot_positions), color='yellow',
                   point_size=12, render_points_as_spheres=True)
        
        if robot_directions is not None:
            robot_arrows = pv.PolyData(robot_positions)
            robot_arrows['vectors'] = robot_directions * robot_arrow_scale
            robot_glyphs = robot_arrows.glyph(orient='vectors', scale=False, factor=robot_arrow_scale)
            p.add_mesh(robot_glyphs, color='darkgreen')
    
    colors = ['red', 'orange', 'purple', 'cyan', 'magenta']
    for i, pts in enumerate(all_points):
        if len(pts) > 1:
            color = colors[i % len(colors)]
            n = len(pts)
            lines = np.column_stack([np.full(n-1, 2), np.arange(n-1), np.arange(1, n)]).flatten()
            p.add_mesh(pv.PolyData(pts, lines=lines), color=color, line_width=2)
    
    p.add_axes()
    p.camera_position = 'iso'
    p.show()


def visualize_first_column_only(stl_path, all_points, all_normals,
                                 robot_offset=0.20, normal_scale=0.03):
    """첫 열(각 경로의 index 0 점)만 시각화"""
    mesh = pv.read(stl_path)

    first_col_points = []
    first_col_normals = []

    for pts, norms in zip(all_points, all_normals):
        if len(pts) > 0:
            first_col_points.append(pts[0])
            first_col_normals.append(norms[0])

    if len(first_col_points) == 0:
        print("[WARN] 첫 열 데이터가 없습니다.")
        return

    first_col_points = np.array(first_col_points)
    first_col_normals = np.array(first_col_normals)

    robot_points = first_col_points + first_col_normals * robot_offset

    print(f"[시각화] 첫 열 점 개수: {len(first_col_points)}")
    print(f"[시각화] 로봇 오프셋: {robot_offset*100:.0f}cm")

    p = pv.Plotter()
    p.set_background('white')
    p.add_mesh(mesh, color='lightblue', opacity=0.3)

    p.add_mesh(pv.PolyData(first_col_points), color='red',
               point_size=12, render_points_as_spheres=True, label='표면 점')

    arrows = pv.PolyData(first_col_points)
    arrows['vectors'] = first_col_normals * normal_scale
    glyphs = arrows.glyph(orient='vectors', scale=False, factor=normal_scale)
    p.add_mesh(glyphs, color='blue', label='법선 벡터')

    p.add_mesh(pv.PolyData(robot_points), color='yellow',
               point_size=12, render_points_as_spheres=True,
               label=f'오프셋 점 ({robot_offset*100:.0f}cm)')

    for sp, rp in zip(first_col_points, robot_points):
        line = pv.Line(sp, rp)
        p.add_mesh(line, color='gray', line_width=1, opacity=0.5)

    p.add_legend(face='circle')
    p.add_axes()
    p.camera_position = 'iso'
    p.show()
    
    # ========== JSON 저장 (스무딩 결과) ==========
    print("\n" + "="*60)
    print("[JSON 저장] 스무딩 오프셋점 + 표면점")
    print("="*60)
    
    json_offset_per_col = []
    json_surface_per_col = []
    
    for col_idx in range(num_cols):
        result = all_col_smoothing_results[col_idx]
        col_pts = all_col_points[col_idx]
        
        if result is not None and len(col_pts) > 0:
            json_offset_per_col.append(result['spline_points'])
            json_surface_per_col.append(col_pts)
        elif len(col_pts) > 0:
            # 스무딩 실패한 열은 원본 오프셋 사용
            col_norms = all_col_normals[col_idx]
            raw_offset = col_pts + col_norms * ROBOT_OFFSET
            json_offset_per_col.append(raw_offset)
            json_surface_per_col.append(col_pts)
    
    transpose_and_save_json(
        json_offset_per_col,
        json_surface_per_col,
        output_path="robot_path.json"
    )


def visualize_smoothing_comparison(stl_path, surface_points, pipeline_result,
                                    normal_scale=0.03):
    """
    원본 vs 스무딩 비교 시각화
    - 빨강: 표면 점
    - 주황 + 주황 선: 원본 오프셋 (울퉁불퉁)
    - 초록 + 초록 선: B-Spline 최종 (부드러움)
    - 파랑 화살표: 스무딩된 법선
    """
    mesh = pv.read(stl_path)
    
    raw_offset = pipeline_result['raw_offset']
    smoothed_offset = pipeline_result['smoothed_offset']
    spline_points = pipeline_result['spline_points']
    smoothed_normals = pipeline_result['smoothed_normals']
    
    p = pv.Plotter()
    p.set_background('white')
    
    p.add_mesh(mesh, color='lightblue', opacity=0.3)
    
    p.add_mesh(pv.PolyData(surface_points), color='red',
               point_size=10, render_points_as_spheres=True, label='표면 점')
    
    arrows = pv.PolyData(surface_points)
    arrows['vectors'] = smoothed_normals * normal_scale
    glyphs = arrows.glyph(orient='vectors', scale=False, factor=normal_scale)
    p.add_mesh(glyphs, color='blue', label='스무딩 법선')
    
    p.add_mesh(pv.PolyData(raw_offset), color='orange',
               point_size=8, render_points_as_spheres=True, label='원본 오프셋 (raw)')
    
    if len(raw_offset) > 1:
        n = len(raw_offset)
        lines = np.column_stack([np.full(n-1, 2), np.arange(n-1), np.arange(1, n)]).flatten()
        p.add_mesh(pv.PolyData(raw_offset, lines=lines), color='orange',
                   line_width=2, opacity=0.6)
    
    p.add_mesh(pv.PolyData(spline_points), color='green',
               point_size=10, render_points_as_spheres=True, label='B-Spline 최종')
    
    if len(spline_points) > 1:
        n = len(spline_points)
        lines = np.column_stack([np.full(n-1, 2), np.arange(n-1), np.arange(1, n)]).flatten()
        p.add_mesh(pv.PolyData(spline_points, lines=lines), color='green',
                   line_width=3)
    
    if len(spline_points) == len(surface_points):
        for sp, rp in zip(surface_points, spline_points):
            line = pv.Line(sp, rp)
            p.add_mesh(line, color='gray', line_width=1, opacity=0.3)
    
    p.add_legend(face='circle')
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
    STEP_DISTANCE = 0.07
    # 레퍼런스 : STEP_DISTANCE = 0.05
    NUM_POINTS = 5
    
    # [2단계] 이상치 필터링 파라미터
    NORMAL_ANGLE_THRESHOLD = 30.0
    PATH_ANGLE_THRESHOLD = 20.0
    
    # [3단계] 로봇 좌표 변환 파라미터
    ROBOT_OFFSET = 0.15
    
    # [스무딩] 후처리 파라미터
    NORMAL_SMOOTH_METHOD = 'gaussian'  # 'gaussian' 또는 'moving_avg'
    NORMAL_SMOOTH_SIGMA = 3.0          # 가우시안 sigma (클수록 부드러움)
    NORMAL_SMOOTH_WINDOW = 7           # 이동평균 윈도우 (moving_avg일 때)
    SPLINE_SMOOTHING = None            # B-Spline 평활도 (None=자동, 0=보간)
    SPLINE_NUM_POINTS = None           # 출력 점 개수 (None=입력과 동일)
    SPLINE_DEGREE = 3                  # 스플라인 차수 (3=cubic)
    
    # [4단계] 시각화 파라미터
    NORMAL_SCALE = 0.03
    ROBOT_ARROW_SCALE = 0.05
    
    # 출력 옵션
    VERBOSE = False
    DEBUG_MODE = False
    # =================================================
    
    print("\n" + "="*60)
    print("경로 생성기 v4.2 - Vertex Normal 보간 + 스무딩")
    print("="*60)
    print("\n[파라미터 설정]")
    print(f"  [1단계] X 확장: {X_EXTEND}m, 라인 간격: {LINE_SPACING}m")
    print(f"  [1단계] theta 점 수: {POINTS_PER_LINE}, 반지름 배율: {RADIUS_SCALE}")
    print(f"  [1단계] 시작점: 띄우기 {START_LIFT*100:.1f}cm → 법선⊥+X+ {START_OFFSET*100:.1f}cm → 레이캐스팅")
    print(f"  [1단계] 점 제거: Z축 최하단 {TRIM_START + TRIM_END}개")
    print(f"  [1.5단계] 시작점 최소 거리: {MIN_DISTANCE*1000:.1f}mm")
    print(f"  [2단계] 레이캐스팅 높이: {PATH_LIFT_DISTANCE}m, 스텝: {STEP_DISTANCE}m")
    print(f"  [2단계] 점 개수: {NUM_POINTS}개")
    print(f"  [2단계] 법선 임계값: {NORMAL_ANGLE_THRESHOLD}°, 경로 임계값: {PATH_ANGLE_THRESHOLD}°")
    print(f"  [3단계] 로봇 오프셋: {ROBOT_OFFSET}m")
    print(f"  [스무딩] 법선: {NORMAL_SMOOTH_METHOD} (sigma={NORMAL_SMOOTH_SIGMA})")
    print(f"  [스무딩] B-Spline: s={SPLINE_SMOOTHING}, k={SPLINE_DEGREE}")
    
    # ========== 1단계: 반구 레이캐스팅 ==========
    start_points, start_normals = extract_edge_points_with_normals(
        stl_path=STL_FILE,
        x_extend=X_EXTEND,
        line_spacing=LINE_SPACING,
        points_per_line=POINTS_PER_LINE,
        radius_scale=RADIUS_SCALE,
        x_offset=START_OFFSET,
        lift_distance=START_LIFT,
        trim_start=TRIM_START,
        trim_end=TRIM_END
    )
    
    if len(start_points) == 0:
        print("[ERROR] 시작점이 추출되지 않았습니다.")
        return
    
    print(f"\n[1단계 완료] 자동 추출된 시작점: {len(start_points)}개")
    
    # ========== 1.5단계: 시작점 균일 샘플링 ==========
    print("\n" + "="*60)
    print("[1.5단계] 시작점 균일 샘플링")
    print("="*60)
    print(f"[INFO] 최소 거리: {MIN_DISTANCE*1000:.1f}mm")
    
    original_start_count = len(start_points)
    
    sampled_indices = [0]
    last_kept_point = start_points[0]
    
    for i in range(1, len(start_points)):
        current_point = start_points[i]
        distance = np.linalg.norm(current_point - last_kept_point)
        
        if distance >= MIN_DISTANCE:
            sampled_indices.append(i)
            last_kept_point = current_point
    
    start_points = start_points[sampled_indices]
    start_normals = start_normals[sampled_indices]
    
    print(f"[샘플링 전] {original_start_count}개")
    print(f"[샘플링 후] {len(start_points)}개")
    print(f"[제거됨] {original_start_count - len(start_points)}개")
    
    # ========== 2단계: 경로 생성 ==========
    print("\n" + "="*60)
    print("[2단계] 경로 생성 (Vertex Normal 보간)")
    print("="*60)
    
    mesh_o3d = o3d.io.read_triangle_mesh(STL_FILE)
    mesh_o3d.compute_vertex_normals()
    mesh_pv = pv.read(STL_FILE)
    
    all_points = []
    all_normals = []
    
    for i, (pt, norm) in enumerate(zip(start_points, start_normals)):
        if VERBOSE:
            print(f"\n[경로 {i+1}/{len(start_points)}] 시작점: ({pt[0]:.4f}, {pt[1]:.4f}, {pt[2]:.4f})")
        
        pts, norms = generate_path_along_surface(
            mesh_o3d=mesh_o3d,
            mesh_pv=mesh_pv,
            start_point=pt,
            start_normal=norm,
            lift_distance=PATH_LIFT_DISTANCE,
            step_distance=STEP_DISTANCE,
            num_points=NUM_POINTS,
            normal_angle_threshold=NORMAL_ANGLE_THRESHOLD,
            path_angle_threshold=PATH_ANGLE_THRESHOLD,
            verbose=VERBOSE
        )
        
        all_points.append(pts)
        all_normals.append(norms)
    
    print(f"\n[2단계 완료] {len(all_points)}개 경로 생성")
    
    # ========== 3단계: 로봇 좌표 변환 ==========
    print("\n" + "="*60)
    print("[3단계] 로봇 좌표 변환")
    print("="*60)
    
    all_robot_positions = []
    all_robot_directions = []
    
    for pts, norms in zip(all_points, all_normals):
        robot_pos, robot_dir = convert_to_robot_pose(pts, norms, offset_distance=ROBOT_OFFSET)
        all_robot_positions.extend(robot_pos)
        all_robot_directions.extend(robot_dir)
    
    all_robot_positions = np.array(all_robot_positions)
    all_robot_directions = np.array(all_robot_directions)
    
    print(f"[3단계 완료] 로봇 위치 {len(all_robot_positions)}개 변환")
    
    # ========== Y+ 필터링 ==========
    print("\n" + "="*60)
    print("[Y+ 필터링] 시작점 Y좌표가 양수인 경로만 선택")
    print("="*60)
    
    filtered_all_points = []
    filtered_all_normals = []
    filtered_start_points_list = []
    filtered_start_normals_list = []
    
    original_path_count = len(all_points)
    
    for i, (pts, norms) in enumerate(zip(all_points, all_normals)):
        if len(pts) > 0:
            first_point_y = pts[0][1]
            
            if first_point_y > 0:
                filtered_all_points.append(pts)
                filtered_all_normals.append(norms)
                filtered_start_points_list.append(start_points[i])
                filtered_start_normals_list.append(start_normals[i])
    
    print(f"[필터링 전] 경로 개수: {original_path_count}개")
    print(f"[필터링 후] Y+ 경로 개수: {len(filtered_all_points)}개")
    print(f"[제거됨] Y <= 0 경로 개수: {original_path_count - len(filtered_all_points)}개")
    
    # ========== Y값 기준 재정렬 ==========
    print("\n" + "="*60)
    print("[인덱스 재정렬] Y=0 기준 → Y+ 방향으로 정렬")
    print("="*60)
    
    if len(filtered_all_points) > 0:
        start_y_values = np.array([pts[0][1] for pts in filtered_all_points])
        sorted_indices = np.argsort(start_y_values)
        
        filtered_all_points = [filtered_all_points[i] for i in sorted_indices]
        filtered_all_normals = [filtered_all_normals[i] for i in sorted_indices]
        filtered_start_points_list = [filtered_start_points_list[i] for i in sorted_indices]
        filtered_start_normals_list = [filtered_start_normals_list[i] for i in sorted_indices]
        
        new_start_y = filtered_all_points[0][0][1]
        new_end_y = filtered_all_points[-1][0][1]
        print(f"[정렬 완료] 첫점 Y={new_start_y:.4f} → 끝점 Y={new_end_y:.4f}")
    
    filtered_start_points = np.array(filtered_start_points_list) if filtered_start_points_list else np.array([]).reshape(0, 3)
    
    # 로봇 좌표 재생성
    filtered_robot_positions = []
    filtered_robot_directions = []
    
    for pts, norms in zip(filtered_all_points, filtered_all_normals):
        robot_pos, robot_dir = convert_to_robot_pose(pts, norms, offset_distance=ROBOT_OFFSET)
        filtered_robot_positions.extend(robot_pos)
        filtered_robot_directions.extend(robot_dir)
    
    filtered_robot_positions = np.array(filtered_robot_positions) if filtered_robot_positions else np.array([]).reshape(0, 3)
    filtered_robot_directions = np.array(filtered_robot_directions) if filtered_robot_directions else np.array([]).reshape(0, 3)
    
    print(f"[필터링 후] 총 점 개수: {sum(len(pts) for pts in filtered_all_points)}개")
    
    # ========== 4단계: 기존 전체 시각화 ==========
    print("\n" + "="*60)
    print("[4단계] 시각화 (전체 경로)")
    print("="*60)
    
    visualize_result(
        STL_FILE, filtered_start_points, filtered_all_points, filtered_all_normals,
        robot_positions=filtered_robot_positions,
        robot_directions=filtered_robot_directions,
        normal_scale=NORMAL_SCALE,
        robot_arrow_scale=ROBOT_ARROW_SCALE,
        debug_mode=DEBUG_MODE
    )
    
    # ========== 5단계: 전체 열 스무딩 비교 시각화 ==========
    print("\n" + "="*60)
    print("[5단계] 전체 열 오프셋 경로 스무딩")
    print("="*60)
    
    # 경로당 점 개수 파악
    num_cols = max(len(pts) for pts in filtered_all_points) if filtered_all_points else 0
    print(f"[INFO] 열 개수: {num_cols}")
    
    all_col_smoothing_results = []  # 열별 스무딩 결과
    all_col_points = []             # 열별 표면 점
    all_col_normals = []            # 열별 법선
    
    for col_idx in range(num_cols):
        # 해당 열의 점/법선 추출
        col_points = []
        col_normals = []
        for pts, norms in zip(filtered_all_points, filtered_all_normals):
            if col_idx < len(pts):
                col_points.append(pts[col_idx])
                col_normals.append(norms[col_idx])
        
        if len(col_points) < 4:  # B-Spline 최소 점 개수 (degree=3 → 4개 필요)
            print(f"  [열 {col_idx}] 점 {len(col_points)}개 → 스무딩 스킵")
            all_col_smoothing_results.append(None)
            all_col_points.append(np.array(col_points) if col_points else np.array([]).reshape(0, 3))
            all_col_normals.append(np.array(col_normals) if col_normals else np.array([]).reshape(0, 3))
            continue
        
        col_points = np.array(col_points)
        col_normals = np.array(col_normals)
        
        print(f"  [열 {col_idx}] 점 {len(col_points)}개 → 스무딩 실행")
        
        result = smooth_offset_pipeline(
            surface_points=col_points,
            raw_normals=col_normals,
            offset_distance=ROBOT_OFFSET,
            normal_smooth_method=NORMAL_SMOOTH_METHOD,
            normal_smooth_sigma=NORMAL_SMOOTH_SIGMA,
            normal_smooth_window=NORMAL_SMOOTH_WINDOW,
            spline_smoothing=SPLINE_SMOOTHING,
            spline_num_points=SPLINE_NUM_POINTS,
            spline_degree=SPLINE_DEGREE
        )
        
        all_col_smoothing_results.append(result)
        all_col_points.append(col_points)
        all_col_normals.append(col_normals)
    
    # ========== 전체 열 비교 시각화 ==========
    print("\n" + "="*60)
    print("[5단계] 시각화 - 전체 열 원본 vs 스무딩 비교")
    print("="*60)
    
    mesh = pv.read(STL_FILE)
    p = pv.Plotter()
    p.set_background('white')
    p.add_mesh(mesh, color='lightblue', opacity=0.3)
    
    raw_added = False
    spline_added = False
    surface_added = False
    
    for col_idx in range(num_cols):
        col_pts = all_col_points[col_idx]
        result = all_col_smoothing_results[col_idx]
        
        if len(col_pts) == 0:
            continue
        
        # 표면 점 (빨강)
        p.add_mesh(pv.PolyData(col_pts), color='red',
                   point_size=8, render_points_as_spheres=True,
                   label='표면 점' if not surface_added else None)
        surface_added = True
        
        if result is None:
            continue
        
        raw_offset = result['raw_offset']
        spline_points = result['spline_points']
        
        # 원본 오프셋 (주황)
        p.add_mesh(pv.PolyData(raw_offset), color='orange',
                   point_size=6, render_points_as_spheres=True,
                   label='원본 오프셋' if not raw_added else None)
        raw_added = True
        
        if len(raw_offset) > 1:
            n = len(raw_offset)
            lines = np.column_stack([np.full(n-1, 2), np.arange(n-1), np.arange(1, n)]).flatten()
            p.add_mesh(pv.PolyData(raw_offset, lines=lines), color='orange',
                       line_width=1, opacity=0.5)
        
        # B-Spline 최종 (초록)
        p.add_mesh(pv.PolyData(spline_points), color='green',
                   point_size=8, render_points_as_spheres=True,
                   label='B-Spline 스무딩' if not spline_added else None)
        spline_added = True
        
        if len(spline_points) > 1:
            n = len(spline_points)
            lines = np.column_stack([np.full(n-1, 2), np.arange(n-1), np.arange(1, n)]).flatten()
            p.add_mesh(pv.PolyData(spline_points, lines=lines), color='green',
                       line_width=3)
    
    p.add_legend(face='circle')
    p.add_axes()
    p.camera_position = 'iso'
    p.show()
    
    # ========== JSON 저장 (스무딩 결과) ==========
    print("\n" + "="*60)
    print("[JSON 저장] 스무딩 오프셋점 + 표면점")
    print("="*60)
    
    json_offset_per_col = []
    json_surface_per_col = []
    
    for col_idx in range(num_cols):
        result = all_col_smoothing_results[col_idx]
        col_pts = all_col_points[col_idx]
        
        if result is not None and len(col_pts) > 0:
            # 스무딩 성공 → B-Spline 결과 사용
            json_offset_per_col.append(result['spline_points'])
            json_surface_per_col.append(col_pts)
        elif len(col_pts) > 0:
            # 스무딩 실패 → 원본 법선 오프셋 사용
            col_norms = all_col_normals[col_idx]
            raw_offset = col_pts + col_norms * ROBOT_OFFSET
            json_offset_per_col.append(raw_offset)
            json_surface_per_col.append(col_pts)
    
    transpose_and_save_json(
        json_offset_per_col,
        json_surface_per_col,
        output_path="robot_path.json"
    )


if __name__ == '__main__':
    main()