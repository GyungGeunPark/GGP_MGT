#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
듀얼 라이다 4점 기반 캘리브레이션 (Lidar1 -> World, Lidar2 -> World)
- 포인트클라우드 2개 로드 (mm로 저장된 경우 -> m로 변환)
- 각 라이다에서 4점 선택 (Shift+좌클릭, Q 종료)
- target_points(월드 좌표, m 단위)와 SVD로 정합 T 계산
- 두 라이다의 캘리브레이션 결과를 하나의 파일에 저장
"""

import open3d as o3d
import numpy as np
from datetime import datetime


# ============================================================
# 🎯 설정값 (여기만 수정하세요)
# ============================================================
LIDAR1_FILE = "1_denoised.ply"   # 라이다1 입력 파일
LIDAR2_FILE = "2_denoised.ply"   # 라이다2 입력 파일
POINT_SIZE  = 5.0                # 표시 포인트 크기
SAVE_TXT  = "calib_dual.txt"   # 결과 저장 파일

# ✅ target은 "월드 좌표(m)"로 유지
TARGET_POINTS = np.array([
    [0.578, -0.100, 0.284],  # P1
    [0.843, -0.087, 0.284],  # P2
    [0.856, -0.581, 0.284],  # P3
    [0.590, -0.581, 0.284],  # P4
], dtype=np.float64)

# ✅ 라이다 파일이 mm 단위로 저장되어 있으면 0.001 (mm->m)
LIDAR_INPUT_UNIT_SCALE = 0.001  # m 단위인 경우 1.0

# --- 점 시각화(구) 설정 (m 단위에서 보기 좋은 값) ---
SPHERE_RADIUS = 0.03   # 3cm
AXIS_SIZE     = 0.30   # 30cm
# ============================================================


def load_pointcloud(path: str, unit_scale: float) -> o3d.geometry.PointCloud:
    """포인트클라우드 로드 및 단위 변환"""
    pcd = o3d.io.read_point_cloud(path)
    if pcd.is_empty():
        raise RuntimeError(f"포인트클라우드가 비어있습니다: {path}")

    print(f"✅ 로드 완료: {path}  (points: {len(pcd.points):,})")

    # 단위 통일: mm -> m
    if unit_scale != 1.0:
        pcd.scale(unit_scale, center=(0, 0, 0))
        print(f"   단위 변환 적용: scale={unit_scale}")

    # 범위 로그(디버그)
    pts = np.asarray(pcd.points)
    mn, mx = pts.min(axis=0), pts.max(axis=0)
    print(f"   좌표 범위(m): min={mn}, max={mx}")

    return pcd


def pick_4_points(pcd: o3d.geometry.PointCloud, point_size: float, 
                  lidar_name: str) -> np.ndarray:
    """4개 대응점 선택"""
    print("\n" + "=" * 70)
    print(f"📍 [{lidar_name}] 4개 대응점을 순서대로 선택하세요 (TARGET 순서와 동일!)")
    print("=" * 70)
    print("Target(월드, m) 좌표:")
    for i, pt in enumerate(TARGET_POINTS):
        print(f"  P{i+1}: ({pt[0]:.4f}, {pt[1]:.4f}, {pt[2]:.4f})")
    print("\n조작법:")
    print("  Shift + 좌클릭 : 포인트 선택")
    print("  Shift + 우클릭 : 마지막 선택 취소")
    print("  Q              : 선택 완료")
    print("=" * 70)

    vis = o3d.visualization.VisualizerWithEditing()
    vis.create_window(f"[{lidar_name}] Pick 4 points (Shift+Click) / Q to finish", 
                      1280, 800)
    vis.add_geometry(pcd)

    opt = vis.get_render_option()
    opt.point_size = float(point_size)
    opt.background_color = np.asarray([0.1, 0.1, 0.1])
    opt.show_coordinate_frame = True

    vis.run()
    vis.destroy_window()

    picked = vis.get_picked_points()
    if len(picked) != 4:
        raise RuntimeError(f"❌ [{lidar_name}] 4개를 선택해야 합니다. 현재: {len(picked)}개")

    pts = np.asarray(pcd.points)
    src = np.array([pts[idx] for idx in picked], dtype=np.float64)

    print(f"\n✅ [{lidar_name}] 선택된 Source 4점 (m):")
    for i, p in enumerate(src):
        print(f"  P{i+1}: ({p[0]:.6f}, {p[1]:.6f}, {p[2]:.6f})")

    return src


def compute_T_svd(source_pts: np.ndarray, target_pts: np.ndarray) -> np.ndarray:
    """Rigid transform (SVD): source -> target"""
    sc = source_pts.mean(axis=0)
    tc = target_pts.mean(axis=0)

    S = source_pts - sc
    T = target_pts - tc

    H = S.T @ T
    U, _, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T

    # reflection 방지
    if np.linalg.det(R) < 0:
        Vt[-1, :] *= -1
        R = Vt.T @ U.T

    t = tc - R @ sc

    M = np.eye(4, dtype=np.float64)
    M[:3, :3] = R
    M[:3, 3] = t
    return M


def calc_errors(source_pts: np.ndarray, target_pts: np.ndarray, 
                T: np.ndarray) -> tuple:
    """변환 후 오차 계산"""
    src_h = np.hstack([source_pts, np.ones((len(source_pts), 1))])
    src_w = (T @ src_h.T).T[:, :3]
    errs = np.linalg.norm(src_w - target_pts, axis=1)
    return errs, src_w


def print_matrix(T: np.ndarray, lidar_name: str):
    """변환 행렬 출력"""
    print(f"\n[{lidar_name} Transformation (Lidar -> World)]")
    print("┌" + "─"*50 + "┐")
    for row in T:
        print(f"│ {row[0]:12.6f} {row[1]:12.6f} {row[2]:12.6f} {row[3]:12.6f} │")
    print("└" + "─"*50 + "┘")


def save_dual_txt(path: str, lidar1_file: str, lidar2_file: str,
                  T1: np.ndarray, rms1: float,
                  T2: np.ndarray, rms2: float):
    """두 라이다 캘리브레이션 결과를 하나의 파일에 저장"""
    with open(path, "w") as f:
        # 헤더
        f.write("# Dual Lidar Calibration Result\n")
        f.write(f"# Generated: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
        f.write(f"# Lidar1: {lidar1_file}\n")
        f.write(f"# Lidar2: {lidar2_file}\n\n")

        # Target Points
        f.write("# Target Points (Robot World Coordinates)\n")
        for i, p in enumerate(TARGET_POINTS):
            f.write(f"# Point {i+1}: ({p[0]:.4f}, {p[1]:.4f}, {p[2]:.4f})\n")
        f.write("\n")

        # Lidar1 결과
        f.write("# " + "=" * 60 + "\n")
        f.write("# Lidar1 Transformation Matrix (Lidar1 -> World)\n")
        f.write("# " + "=" * 60 + "\n")
        f.write("transformation_lidar1 = np.array([\n")
        for r in T1:
            f.write(f"    [{r[0]:.6f}, {r[1]:.6f}, {r[2]:.6f}, {r[3]:.6f}],\n")
        f.write("])\n")
        f.write(f"# Lidar1 RMS Error: {rms1:.6f} m\n\n")

        # Lidar2 결과
        f.write("# " + "=" * 60 + "\n")
        f.write("# Lidar2 Transformation Matrix (Lidar2 -> World)\n")
        f.write("# " + "=" * 60 + "\n")
        f.write("transformation_lidar2 = np.array([\n")
        for r in T2:
            f.write(f"    [{r[0]:.6f}, {r[1]:.6f}, {r[2]:.6f}, {r[3]:.6f}],\n")
        f.write("])\n")
        f.write(f"# Lidar2 RMS Error: {rms2:.6f} m\n")

    print(f"\n✅ 저장 완료: {path}")


def make_sphere(center, radius, color):
    """구 생성"""
    s = o3d.geometry.TriangleMesh.create_sphere(radius=float(radius))
    s.compute_vertex_normals()
    s.paint_uniform_color(color)
    s.translate(center)
    return s


def make_spheres(points, radius, color):
    """여러 구 생성"""
    return [make_sphere(p, radius, color) for p in points]


def calibrate_single_lidar(lidar_file: str, lidar_name: str, 
                           color_raw: list, color_aligned: list):
    """단일 라이다 캘리브레이션 수행"""
    print("\n" + "=" * 70)
    print(f"🔧 {lidar_name} 캘리브레이션 시작")
    print("=" * 70)

    # 1) 로드
    pcd_raw = load_pointcloud(lidar_file, LIDAR_INPUT_UNIT_SCALE)

    # 2) 원본 복사 (시각화용)
    pcd_src = o3d.geometry.PointCloud(pcd_raw)
    pcd_src.paint_uniform_color(color_raw)

    # 3) 4점 선택
    source_pts = pick_4_points(pcd_src, POINT_SIZE, lidar_name)

    # 4) T 계산
    T = compute_T_svd(source_pts, TARGET_POINTS)
    errs, source_pts_world = calc_errors(source_pts, TARGET_POINTS, T)
    rms = float(np.sqrt(np.mean(errs ** 2)))

    print_matrix(T, lidar_name)
    print(f"\n오차(각 점, m): {errs}")
    print(f"RMS 오차: {rms:.6f} m")

    # 5) 정합 결과 생성
    pcd_aligned = o3d.geometry.PointCloud(pcd_src)
    pcd_aligned.transform(T)
    pcd_aligned.paint_uniform_color(color_aligned)

    return {
        'T': T,
        'rms': rms,
        'source_pts': source_pts,
        'source_pts_world': source_pts_world,
        'pcd_src': pcd_src,
        'pcd_aligned': pcd_aligned,
    }


def visualize_dual_result(result1: dict, result2: dict):
    """두 라이다 캘리브레이션 결과 동시 시각화"""
    print("\n" + "=" * 70)
    print("🧭 듀얼 시각화")
    print("=" * 70)
    print(" - 빨강: Lidar1 원본")
    print(" - 초록: Lidar1 정합 후")
    print(" - 파랑: Lidar2 원본")
    print(" - 시안: Lidar2 정합 후")
    print(" - 노랑 구: Target 4점 (월드)")
    print(" - 좌표축: 월드")

    # 월드 좌표축
    axes = o3d.geometry.TriangleMesh.create_coordinate_frame(
        size=float(AXIS_SIZE), origin=[0, 0, 0])

    # Target 구 (노랑)
    spheres_target = make_spheres(TARGET_POINTS, SPHERE_RADIUS, [1.0, 1.0, 0.0])

    # 시각화 객체 리스트
    geometries = [
        result1['pcd_src'],      # 빨강
        result1['pcd_aligned'],  # 초록
        result2['pcd_src'],      # 파랑
        result2['pcd_aligned'],  # 시안
        axes,
        *spheres_target,
    ]

    o3d.visualization.draw_geometries(
        geometries,
        window_name="Dual Lidar Calibration Result",
        width=1280,
        height=800,
        point_show_normal=False,
    )


def main():
    print("=" * 70)
    print("🚀 듀얼 라이다 캘리브레이션 시작")
    print("=" * 70)
    print(f"Lidar1: {LIDAR1_FILE}")
    print(f"Lidar2: {LIDAR2_FILE}")

    # ========================================
    # Lidar1 캘리브레이션
    # ========================================
    result1 = calibrate_single_lidar(
        LIDAR1_FILE, 
        "Lidar1",
        color_raw=[1.0, 0.2, 0.2],      # 원본: 빨강
        color_aligned=[0.2, 1.0, 0.2],  # 정합: 초록
    )

    # ========================================
    # Lidar2 캘리브레이션
    # ========================================
    result2 = calibrate_single_lidar(
        LIDAR2_FILE,
        "Lidar2",
        color_raw=[0.2, 0.4, 1.0],      # 원본: 파랑
        color_aligned=[0.2, 1.0, 1.0],  # 정합: 시안
    )

    # ========================================
    # 결과 저장 (하나의 파일에 두 라이다 결과)
    # ========================================
    save_dual_txt(
        SAVE_TXT,
        LIDAR1_FILE, LIDAR2_FILE,
        result1['T'], result1['rms'],
        result2['T'], result2['rms'],
    )

    # ========================================
    # 시각화 (두 라이다 동시)
    # ========================================
    visualize_dual_result(result1, result2)

    print("\n" + "=" * 70)
    print("✨ 듀얼 라이다 캘리브레이션 완료!")
    print("=" * 70)


if __name__ == "__main__":
    main()