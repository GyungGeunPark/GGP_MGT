"""
경로 생성기 v3.1
- Open3D: 사용자가 점 클릭
- 클릭점 → 추정 법선으로 띄운 후 역방향 레이캐스팅 → 메시 삼각형 법선 획득
- 법선과 수직 + X+ 방향으로 레이캐스팅
- 히트 실패 시 방향 이동 + 법선 복사
- 경로 각도 급변 시 이전 방향으로 예측 이동
- [수정] 법선 각도 체크: X/Y 분리 → 벡터 직접 비교로 변경
"""

import numpy as np
import pyvista as pv
import open3d as o3d


def pick_points_open3d(stl_path: str, point_density=0.001, raycast_lift_distance=0.1):
    """
    Open3D로 STL에서 점 클릭 + 메시 삼각형 법선 반환
    
    과정:
    1. 포인트 클라우드에서 점 클릭
    2. PCA로 법선 추정 → Z+ 방향 보정
    3. 추정 법선 방향으로 띄운 후 역방향 레이캐스팅
    4. 히트한 메시 삼각형의 정확한 법선 반환
    
    Args:
        stl_path: STL 파일 경로
        point_density: 포인트 클라우드 밀도 (작을수록 촘촘)
        raycast_lift_distance: 레이캐스팅용 띄우기 거리 (m)
    
    Returns:
        (points, normals): 히트점 좌표, 메시 삼각형 법선
    """
    print("\n" + "="*50)
    print("[Open3D] STL 파일 로드 중...")
    print("="*50)
    
    mesh = o3d.io.read_triangle_mesh(stl_path)
    mesh.compute_triangle_normals()
    mesh.compute_vertex_normals()
    
    bbox = mesh.get_axis_aligned_bounding_box()
    bbox_size = np.linalg.norm(bbox.get_extent())
    num_points = int(bbox_size / point_density * 100)
    num_points = max(10000, min(num_points, 200000))
    
    pcd = mesh.sample_points_uniformly(number_of_points=num_points)
    pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.05, max_nn=30))
    pcd.paint_uniform_color([0.5, 0.7, 1.0])
    
    print(f"\n[INFO] 메시를 {num_points}개 포인트로 변환")
    print("\n[사용법]")
    print("  1. Shift + 좌클릭: 점 선택")
    print("  2. Q 키: 선택 완료")
    print("="*50 + "\n")
    
    vis = o3d.visualization.VisualizerWithEditing()
    vis.create_window(window_name="점 선택 (Shift+클릭 → Q)", width=1280, height=720)
    vis.add_geometry(pcd)
    vis.get_render_option().background_color = np.array([1, 1, 1])
    vis.get_render_option().point_size = 3.0
    vis.run()
    vis.destroy_window()
    
    picked_indices = vis.get_picked_points()
    
    if not picked_indices:
        print("[WARN] 선택된 점이 없습니다!")
        return np.array([]), np.array([])
    
    points = np.asarray(pcd.points)
    normals = np.asarray(pcd.normals)
    
    picked_points = points[picked_indices]
    estimated_normals = normals[picked_indices].copy()
    
    # 추정 법선을 Z+ 방향으로 보정 (레이캐스팅 방향 결정용)
    for i in range(len(estimated_normals)):
        if estimated_normals[i][2] < 0:
            estimated_normals[i] = -estimated_normals[i]
    
    # ========== 레이캐스팅으로 정확한 메시 법선 획득 ==========
    print("\n[레이캐스팅] 메시 삼각형 법선 획득 중...")
    
    mesh_t = o3d.t.geometry.TriangleMesh.from_legacy(mesh)
    scene = o3d.t.geometry.RaycastingScene()
    scene.add_triangles(mesh_t)
    
    triangle_normals = np.asarray(mesh.triangle_normals)
    
    final_points = []
    final_normals = []
    
    for i, (pt, est_normal) in enumerate(zip(picked_points, estimated_normals)):
        # 1. 추정 법선 방향으로 띄움
        lifted_point = pt + est_normal * raycast_lift_distance
        
        # 2. 역방향(-법선)으로 레이캐스팅
        ray_direction = -est_normal
        rays = o3d.core.Tensor(
            [[lifted_point[0], lifted_point[1], lifted_point[2],
              ray_direction[0], ray_direction[1], ray_direction[2]]],
            dtype=o3d.core.Dtype.Float32
        )
        
        result = scene.cast_rays(rays)
        
        t_hit = result['t_hit'].numpy()[0]
        primitive_ids = result['primitive_ids'].numpy()[0]
        
        if t_hit == np.inf or primitive_ids == scene.INVALID_ID:
            # 히트 실패 → 추정 법선 사용 (fallback)
            print(f"  [점 {i+1}] 레이캐스팅 실패 - 추정 법선 사용")
            final_points.append(pt)
            final_normals.append(est_normal)
            continue
        
        # 3. 히트 성공 → 히트점과 삼각형 법선 획득
        hit_point = lifted_point + ray_direction * t_hit
        tri_normal = triangle_normals[primitive_ids].copy()
        
        # 4. 법선이 Z+ 방향 향하도록 보정
        if tri_normal[2] < 0:
            tri_normal = -tri_normal
        
        final_points.append(hit_point)
        final_normals.append(tri_normal)
        
        print(f"  [점 {i+1}] 히트 성공")
        print(f"    - 원래 클릭점: ({pt[0]:.4f}, {pt[1]:.4f}, {pt[2]:.4f})")
        print(f"    - 히트점:      ({hit_point[0]:.4f}, {hit_point[1]:.4f}, {hit_point[2]:.4f})")
        print(f"    - 추정 법선:   ({est_normal[0]:.4f}, {est_normal[1]:.4f}, {est_normal[2]:.4f})")
        print(f"    - 메시 법선:   ({tri_normal[0]:.4f}, {tri_normal[1]:.4f}, {tri_normal[2]:.4f})")
    
    final_points = np.array(final_points)
    final_normals = np.array(final_normals)
    
    print(f"\n[INFO] 최종 선택된 점 {len(final_points)}개")
    
    return final_points, final_normals


def get_perpendicular_xplus_direction(normal):
    """
    법선과 수직이면서 X+ 방향 성분이 양수인 단위 벡터 계산
    """
    normal = normal / np.linalg.norm(normal)
    x_axis = np.array([1.0, 0.0, 0.0])
    
    # X축을 법선 평면에 투영
    projection = x_axis - np.dot(x_axis, normal) * normal
    proj_len = np.linalg.norm(projection)
    
    if proj_len < 1e-10:
        # 법선이 X축과 평행한 경우 → Y축 사용
        y_axis = np.array([0.0, 1.0, 0.0])
        projection = y_axis - np.dot(y_axis, normal) * normal
        proj_len = np.linalg.norm(projection)
    
    ray_dir = projection / proj_len
    
    # X+ 방향 보장
    if ray_dir[0] < 0:
        ray_dir = -ray_dir
    
    return ray_dir


def check_normal_angle_change(new_normal, prev_normal, angle_threshold=30.0):
    """
    법선 벡터 간 직접 각도 비교 (벡터 내적 방식)
    
    Args:
        new_normal: 새 법선 벡터
        prev_normal: 이전 법선 벡터
        angle_threshold: 허용 각도 (도)
    
    Returns:
        (is_outlier, angle_diff): 이상치 여부, 각도 변화량
    """
    # 단위 벡터로 정규화
    new_norm = new_normal / np.linalg.norm(new_normal)
    prev_norm = prev_normal / np.linalg.norm(prev_normal)
    
    # 내적으로 각도 계산
    dot = np.clip(np.dot(new_norm, prev_norm), -1.0, 1.0)
    angle_diff = np.degrees(np.arccos(dot))
    
    is_outlier = angle_diff > angle_threshold
    
    return is_outlier, angle_diff


def check_path_direction_change(prev_point, current_point, new_point, angle_threshold=45.0):
    """
    경로의 방향 변화 체크 (점들을 이은 선의 각도 변화)
    
    prev_point → current_point → new_point 에서
    이전 이동 방향과 새 이동 방향의 각도를 비교
    
    Args:
        prev_point: 이전 점 (N-2)
        current_point: 현재 점 (N-1)
        new_point: 새로 생성된 점 (N)
        angle_threshold: 허용 각도 변화 (도)
    
    Returns:
        (is_outlier, angle_diff): 이상치 여부, 각도 변화량
    """
    # 이전 이동 방향: prev → current
    prev_direction = current_point - prev_point
    prev_length = np.linalg.norm(prev_direction)
    
    if prev_length < 1e-10:
        return False, 0.0
    
    prev_direction = prev_direction / prev_length
    
    # 새 이동 방향: current → new
    new_direction = new_point - current_point
    new_length = np.linalg.norm(new_direction)
    
    if new_length < 1e-10:
        return False, 0.0
    
    new_direction = new_direction / new_length
    
    # 두 방향 벡터 사이의 각도 계산
    dot_product = np.clip(np.dot(prev_direction, new_direction), -1.0, 1.0)
    angle_diff = np.degrees(np.arccos(dot_product))
    
    is_outlier = angle_diff > angle_threshold
    
    return is_outlier, angle_diff


def generate_path_along_surface(mesh, start_point, start_normal, 
                                 lift_distance=0.1, step_distance=0.05, num_points=5,
                                 normal_angle_threshold=30.0,
                                 path_angle_threshold=45.0):
    """
    시작점에서 법선 수직 + X+ 방향으로 곡면 따라 경로 생성
    
    히트 성공 → 히트점 + 새 법선 저장 
        - 법선 각도 급변 시 이전 법선 유지
        - 경로 각도 급변 시 이전 방향으로 예측 이동
    히트 실패 → (법선⊥ + X+) 방향으로 이동 + 이전 법선 복사
    
    Args:
        mesh: PyVista 메시 객체
        start_point: 시작점 좌표
        start_normal: 시작점 법선
        lift_distance: 레이캐스팅용 띄우기 거리 (m)
        step_distance: X+ 방향 스텝 거리 (m)
        num_points: 생성할 총 점 개수
        normal_angle_threshold: 법선 벡터 각도 변화 허용치 (도)
        path_angle_threshold: 경로 방향 변화 허용 각도 (도)
    
    Returns:
        (points, normals): 생성된 점들과 법선들
    """
    hit_points = [start_point.copy()]
    hit_normals = [start_normal.copy()]
    
    current_point = start_point.copy()
    current_normal = start_normal.copy()
    
    print(f"    [Step 0] 시작점 - 법선: ({current_normal[0]:.4f}, {current_normal[1]:.4f}, {current_normal[2]:.4f})")
    
    for step in range(num_points - 1):
        # 1. 현재 표면점에서 법선 방향으로 띄움
        lifted_point = current_point + current_normal * lift_distance
        
        # 2. (법선⊥ + X+) 방향으로 이동 → 새 점 생성
        move_dir = get_perpendicular_xplus_direction(current_normal)
        new_air_point = lifted_point + move_dir * step_distance
        
        # 3. 새 점에서 법선 반대방향(-N)으로 레이캐스팅
        ray_start = new_air_point
        ray_end = new_air_point - current_normal * (lift_distance * 3)
        
        intersection_points, intersection_cells = mesh.ray_trace(ray_start, ray_end)
        
        if len(intersection_points) == 0:
            # 히트 실패 → (법선⊥ + X+) 방향으로 이동 + 이전 법선 복사
            print(f"    [Step {step+1}] 히트 없음 - 방향 이동 + 법선 복사")
            new_point = current_point + move_dir * step_distance
            hit_points.append(new_point)
            hit_normals.append(current_normal.copy())
            current_point = new_point
            continue
        
        # 4. 히트 성공 → 히트점
        hit_pt = intersection_points[0]
        cell_id = intersection_cells[0]
        
        # 5. 경로 각도 급변 체크 (점이 3개 이상 있을 때부터)
        if len(hit_points) >= 3:
            prev_point = hit_points[-2]
            curr_point = hit_points[-1]
            
            is_path_outlier, path_angle = check_path_direction_change(
                prev_point, curr_point, hit_pt,
                angle_threshold=path_angle_threshold
            )
            
            if is_path_outlier:
                print(f"    [Step {step+1}] 경로 각도 급변 ({path_angle:.1f}° > {path_angle_threshold}°) - 이전 방향으로 예측 이동")
                
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
        
        # 6. 히트점의 법선 계산
        cell = mesh.get_cell(cell_id)
        tri_pts = cell.points
        v1 = tri_pts[1] - tri_pts[0]
        v2 = tri_pts[2] - tri_pts[0]
        new_normal = np.cross(v1, v2)
        norm_len = np.linalg.norm(new_normal)
        
        if norm_len > 1e-10:
            new_normal = new_normal / norm_len
            if np.dot(new_normal, current_normal) < 0:
                new_normal = -new_normal
        else:
            new_normal = current_normal.copy()
        
        # 7. 법선 각도 급변 체크 (벡터 직접 비교)
        #    - step 0: 클릭점 → 첫 번째 생성 점 (수동 선택이라 급변 가능 → 제외)
        #    - step 1+: 생성점 → 생성점 (연속 생성이라 급변 시 이상치 → 체크)
        if step >= 1:
            is_normal_outlier, normal_angle = check_normal_angle_change(
                new_normal, current_normal,
                angle_threshold=normal_angle_threshold
            )
            
            if is_normal_outlier:
                print(f"    [Step {step+1}] 법선 급변 ({normal_angle:.1f}° > {normal_angle_threshold}°) - 이전 법선 유지")
                new_normal = current_normal.copy()
            else:
                print(f"    [Step {step+1}] 정상 - 법선 변화: {normal_angle:.1f}°")
        else:
            # step 0: 첫 번째 생성점은 체크 없이 통과
            print(f"    [Step {step+1}] 첫 생성점 - 법선 체크 제외")
        
        hit_points.append(hit_pt)
        hit_normals.append(new_normal)
        
        current_point = hit_pt
        current_normal = new_normal
    
    return np.array(hit_points), np.array(hit_normals)


def convert_to_robot_pose(points, normals, offset_distance=0.15):
    """
    표면 점들을 로봇 엔드이펙터 좌표로 변환
    
    Args:
        points: 표면 점 좌표들
        normals: 표면 법선들
        offset_distance: 표면에서 떨어질 거리 (m)
    
    Returns:
        (positions, directions): 로봇 위치, 로봇 방향(표면을 바라보는)
    """
    positions = []
    directions = []
    
    for pt, norm in zip(points, normals):
        offset_pt = pt + norm * offset_distance
        positions.append(offset_pt)
        look_dir = -norm
        directions.append(look_dir)
    
    return np.array(positions), np.array(directions)


def transpose_and_save_json(all_robot_positions, all_robot_directions, 
                            all_points, output_path="robot_path.json"):
    """
    행 기준 데이터를 열 기준으로 전치(transpose)하여 JSON 저장
    """
    import json
    
    num_lines = len(all_points)
    if num_lines == 0:
        print("[WARN] 저장할 데이터가 없습니다.")
        return
    
    num_points_per_line = len(all_points[0])
    
    robot_positions_per_line = []
    robot_directions_per_line = []
    
    idx = 0
    for pts in all_points:
        n = len(pts)
        robot_positions_per_line.append(all_robot_positions[idx:idx+n])
        robot_directions_per_line.append(all_robot_directions[idx:idx+n])
        idx += n
    
    columns = []
    for col_idx in range(num_points_per_line):
        column_points = []
        for line_idx in range(num_lines):
            if col_idx < len(robot_positions_per_line[line_idx]):
                pos = robot_positions_per_line[line_idx][col_idx]
                dir = robot_directions_per_line[line_idx][col_idx]
                point_data = [
                    float(pos[0]), float(pos[1]), float(pos[2]),
                    float(dir[0]), float(dir[1]), float(dir[2])
                ]
                column_points.append(point_data)
        columns.append(column_points)
    
    total_points = sum(len(col) for col in columns)
    
    output_data = {
        "num_lines": len(columns),
        "total_points": total_points,
        "lines": columns
    }
    
    with open(output_path, 'w', encoding='utf-8') as f:
        json.dump(output_data, f, indent=2, ensure_ascii=False)
    
    print(f"\n[JSON 저장] {output_path}")
    print(f"  - 열 개수: {len(columns)}")
    print(f"  - 총 점 개수: {total_points}")
    print(f"  - 구조: 각 열에 {num_lines}개 라인의 같은 위치 점들")
    
    return output_data


def visualize_result(stl_path, all_points, all_normals, clicked_points, 
                     robot_positions=None, robot_directions=None,
                     normal_scale=0.03, robot_arrow_scale=0.05):
    """
    결과 시각화
    """
    mesh = pv.read(stl_path)
    
    p = pv.Plotter()
    p.set_background('white')
    p.add_mesh(mesh, color='lightblue', opacity=0.5)
    
    if len(clicked_points) > 0:
        p.add_mesh(pv.PolyData(clicked_points), color='green',
                   point_size=15, render_points_as_spheres=True)
    
    all_pts_combined = []
    all_norms_combined = []
    for pts, norms in zip(all_points, all_normals):
        if len(pts) > 0:
            all_pts_combined.extend(pts)
            all_norms_combined.extend(norms)
    
    if len(all_pts_combined) > 0:
        all_pts_arr = np.array(all_pts_combined)
        all_norms_arr = np.array(all_norms_combined)
        
        print(f"\n[시각화] 표면 점: {len(all_pts_arr)}개")
        
        p.add_mesh(pv.PolyData(all_pts_arr), color='red',
                   point_size=10, render_points_as_spheres=True)
        
        arrows = pv.PolyData(all_pts_arr)
        arrows['vectors'] = all_norms_arr * normal_scale
        glyphs = arrows.glyph(orient='vectors', scale=False, factor=normal_scale)
        p.add_mesh(glyphs, color='blue')
    
    if robot_positions is not None and len(robot_positions) > 0:
        print(f"[시각화] 로봇 위치: {len(robot_positions)}개")
        
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


def main():
    # ==================== 파라미터 ====================
    STL_FILE = "gene1.stl"
    
    # [1단계] 점 선택 파라미터
    POINT_DENSITY = 0.001           # 포인트 클라우드 밀도 (작을수록 촘촘)
    CLICK_RAYCAST_LIFT = 0.1        # 클릭점 법선 획득용 레이캐스팅 띄우기 거리 (m)
    
    # [2단계] 경로 생성 파라미터
    PATH_LIFT_DISTANCE = 0.1        # 경로 생성 시 레이캐스팅용 띄우기 (m)
    STEP_DISTANCE = 0.05            # X+ 방향 스텝 거리 (m)
    NUM_POINTS = 4                  # 각 클릭점당 생성할 점 개수
    
    # [2단계] 이상치 필터링 파라미터 (수정됨)
    NORMAL_ANGLE_THRESHOLD = 30.0   # 법선 벡터 각도 변화 허용치 (도) - 벡터 직접 비교
    PATH_ANGLE_THRESHOLD = 20.0     # 경로 방향 변화 허용 각도 (도)
    
    # [3단계] 로봇 좌표 변환 파라미터
    ROBOT_OFFSET = 0.20             # 로봇 엔드이펙터 오프셋 (m)
    
    # [5단계] 시각화 파라미터
    NORMAL_SCALE = 0.03             # 표면 법선 화살표 크기
    ROBOT_ARROW_SCALE = 0.05        # 로봇 방향 화살표 크기
    # =================================================
    
    print("\n" + "="*60)
    print("경로 생성기 v3.1")
    print("="*60)
    print("\n[파라미터 설정]")
    print(f"  - 포인트 밀도: {POINT_DENSITY}")
    print(f"  - 클릭점 레이캐스팅 높이: {CLICK_RAYCAST_LIFT}m")
    print(f"  - 경로 레이캐스팅 높이: {PATH_LIFT_DISTANCE}m")
    print(f"  - 스텝 거리: {STEP_DISTANCE}m")
    print(f"  - 점 개수: {NUM_POINTS}개")
    print(f"  - 법선 각도 임계값: {NORMAL_ANGLE_THRESHOLD}° (벡터 직접 비교)")
    print(f"  - 경로 각도 임계값: {PATH_ANGLE_THRESHOLD}°")
    print(f"  - 로봇 오프셋: {ROBOT_OFFSET}m")
    
    # 1단계: Open3D에서 점 + 메시 삼각형 법선 획득
    clicked_points, clicked_normals = pick_points_open3d(
        STL_FILE,
        point_density=POINT_DENSITY,
        raycast_lift_distance=CLICK_RAYCAST_LIFT
    )
    
    if len(clicked_points) == 0:
        print("[ERROR] 점이 선택되지 않았습니다.")
        return
    
    # 2단계: 각 클릭점에서 경로 생성
    mesh = pv.read(STL_FILE)
    all_points = []
    all_normals = []
    
    for i, (pt, norm) in enumerate(zip(clicked_points, clicked_normals)):
        print(f"\n[경로 {i+1}] 시작점: ({pt[0]:.4f}, {pt[1]:.4f}, {pt[2]:.4f})")
        print(f"         법선: ({norm[0]:.4f}, {norm[1]:.4f}, {norm[2]:.4f})")
        
        pts, norms = generate_path_along_surface(
            mesh=mesh,
            start_point=pt,
            start_normal=norm,
            lift_distance=PATH_LIFT_DISTANCE,
            step_distance=STEP_DISTANCE,
            num_points=NUM_POINTS,
            normal_angle_threshold=NORMAL_ANGLE_THRESHOLD,
            path_angle_threshold=PATH_ANGLE_THRESHOLD
        )
        
        all_points.append(pts)
        all_normals.append(norms)
        print(f"    → {len(pts)}개 점 생성 완료")
    
    # 3단계: 로봇 엔드이펙터 좌표로 변환
    all_robot_positions = []
    all_robot_directions = []
    
    print(f"\n[로봇 좌표 변환] 오프셋: {ROBOT_OFFSET}m")
    for i, (pts, norms) in enumerate(zip(all_points, all_normals)):
        robot_pos, robot_dir = convert_to_robot_pose(pts, norms, offset_distance=ROBOT_OFFSET)
        all_robot_positions.extend(robot_pos)
        all_robot_directions.extend(robot_dir)
        
        print(f"\n  [경로 {i+1}] 로봇 위치:")
        for j, (rp, rd) in enumerate(zip(robot_pos, robot_dir)):
            print(f"    pt{j+1}: pos({rp[0]:.4f}, {rp[1]:.4f}, {rp[2]:.4f}), dir({rd[0]:.4f}, {rd[1]:.4f}, {rd[2]:.4f})")
    
    all_robot_positions = np.array(all_robot_positions)
    all_robot_directions = np.array(all_robot_directions)
    
    # 4단계: JSON 저장 (열 기준으로 전치)
    transpose_and_save_json(
        all_robot_positions, 
        all_robot_directions, 
        all_points,
        output_path="robot_path.json"
    )
    
    # 5단계: 시각화
    visualize_result(
        STL_FILE, all_points, all_normals, clicked_points,
        robot_positions=all_robot_positions,
        robot_directions=all_robot_directions,
        normal_scale=NORMAL_SCALE,
        robot_arrow_scale=ROBOT_ARROW_SCALE
    )


if __name__ == '__main__':
    main()