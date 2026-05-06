"""
반구 레이캐스팅 - X 최소 모서리 추출
Open3D로 레이캐스팅 (빠름) + PyVista로 시각화
기존 PyVista 코드와 동일한 로직
"""

import numpy as np
import open3d as o3d
import pyvista as pv


def main():
    # ========== 파라미터 ==========
    STL_FILE = "gene1.stl"
    
    X_EXTEND = 0.2
    LINE_SPACING = 0.0001
    POINTS_PER_LINE = 180
    RADIUS_SCALE = 1.2
    # ==============================
    
    # Open3D로 메시 로드
    mesh_o3d = o3d.io.read_triangle_mesh(STL_FILE)
    mesh_o3d.compute_vertex_normals()
    
    # 바운딩박스
    bbox = mesh_o3d.get_axis_aligned_bounding_box()
    min_bound = np.array(bbox.min_bound)
    max_bound = np.array(bbox.max_bound)
    
    size_x = max_bound[0] - min_bound[0]
    size_y = max_bound[1] - min_bound[1]
    size_z = max_bound[2] - min_bound[2]
    
    print(f"[INFO] 메시 크기: X={size_x:.4f}, Y={size_y:.4f}, Z={size_z:.4f}")
    
    # 원 설정 (기존과 동일)
    radius = max(size_y, size_z) / 2 * RADIUS_SCALE
    center_x = (min_bound[0] + max_bound[0]) / 2
    center_y = (min_bound[1] + max_bound[1]) / 2
    center_z = (min_bound[2] + max_bound[2]) / 2
    
    print(f"[INFO] 중심: ({center_x:.4f}, {center_y:.4f}, {center_z:.4f})")
    print(f"[INFO] 반지름: {radius:.4f}")
    
    # X 범위
    x_min = min_bound[0] - X_EXTEND
    x_max = max_bound[0] + X_EXTEND
    x_positions = np.arange(x_min, x_max + LINE_SPACING / 2, LINE_SPACING)
    
    # theta 각도 (0 ~ 2π)
    theta_angles = np.linspace(0, 2 * np.pi, POINTS_PER_LINE, endpoint=False)
    
    num_x = len(x_positions)
    num_theta = len(theta_angles)
    
    print(f"[INFO] X 라인 수: {num_x}, theta 점 수: {num_theta}")
    print(f"[INFO] 총 레이 수: {num_x * num_theta}")
    
    # Open3D 레이캐스팅 씬 생성
    mesh_t = o3d.t.geometry.TriangleMesh.from_legacy(mesh_o3d)
    scene = o3d.t.geometry.RaycastingScene()
    scene.add_triangles(mesh_t)
    
    # 레이 생성 (기존 코드와 동일한 순서: theta → x)
    all_ray_starts = []
    all_ray_dirs = []
    ray_theta_indices = []
    all_max_dists = []
    
    for theta_idx, theta in enumerate(theta_angles):
        for x in x_positions:
            # 레이 시작점 (원 위)
            ray_start = np.array([
                x,
                center_y + radius * np.cos(theta),
                center_z + radius * np.sin(theta)
            ])
            
            # 레이 끝점 (중심, X만 현재값) - 기존과 동일
            ray_end = np.array([x, center_y, center_z])
            
            # 레이 방향과 최대 거리
            ray_vec = ray_end - ray_start
            max_dist = np.linalg.norm(ray_vec)
            if max_dist > 1e-10:
                ray_dir = ray_vec / max_dist
            else:
                ray_dir = np.array([0, 0, 0])
            
            all_ray_starts.append(ray_start)
            all_ray_dirs.append(ray_dir)
            ray_theta_indices.append(theta_idx)
            all_max_dists.append(max_dist)
    
    all_ray_starts = np.array(all_ray_starts, dtype=np.float32)
    all_ray_dirs = np.array(all_ray_dirs, dtype=np.float32)
    ray_theta_indices = np.array(ray_theta_indices)
    all_max_dists = np.array(all_max_dists, dtype=np.float32)
    
    print(f"[INFO] 레이캐스팅 시작...")
    
    # Open3D 레이캐스팅
    rays = np.hstack([all_ray_starts, all_ray_dirs]).astype(np.float32)
    rays_tensor = o3d.core.Tensor(rays, dtype=o3d.core.Dtype.Float32)
    
    result = scene.cast_rays(rays_tensor)
    t_hit = result['t_hit'].numpy()
    
    print(f"[INFO] 레이캐스팅 완료")
    
    # 히트점 계산 (레이 길이 이내만 유효)
    hit_mask = np.isfinite(t_hit) & (t_hit <= all_max_dists)
    hit_points = all_ray_starts + all_ray_dirs * t_hit[:, np.newaxis]
    
    # theta별로 X 최소 히트점 찾기 (기존과 동일한 로직)
    min_x_points = []
    
    for theta_idx in range(num_theta):
        theta_mask = (ray_theta_indices == theta_idx) & hit_mask
        
        if np.any(theta_mask):
            theta_hits = hit_points[theta_mask]
            min_idx = np.argmin(theta_hits[:, 0])
            min_x_points.append(theta_hits[min_idx])
    
    min_x_points = np.array(min_x_points) if min_x_points else np.array([]).reshape(0, 3)
    
    print(f"\n[결과] 히트 레이: {np.sum(hit_mask)}개")
    print(f"[결과] X 최소 모서리 점: {len(min_x_points)}개")
    
    # PyVista로 시각화
    mesh_pv = pv.read(STL_FILE)
    
    p = pv.Plotter()
    p.set_background('white')
    
    # 메시
    p.add_mesh(mesh_pv, color='lightblue', opacity=0.3)
    
    # 레이 시작점 (회색)
    p.add_mesh(pv.PolyData(all_ray_starts), color='gray',
               point_size=2, render_points_as_spheres=True, opacity=0.3)
    
    # X 최소 모서리 점 (빨강)
    if len(min_x_points) > 0:
        p.add_mesh(pv.PolyData(min_x_points), color='red',
                   point_size=10, render_points_as_spheres=True)
    
    p.add_axes()
    p.camera_position = 'iso'
    p.show()


if __name__ == '__main__':
    main()