#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
듀얼 라이다 정합 및 크롭 (PLY 파일 지원)
- calib_dual.txt 파일에서 변환 행렬 로드
- 두 라이다 PLY를 로봇 월드 좌표계로 변환
- mm -> m 단위 변환 지원
- XYZ 범위로 크롭
- 결과 시각화 및 저장
"""
import open3d as o3d
import numpy as np
import copy
import re
import time
import os
import glob


# ============================================================
# 🎯 설정값 (여기만 수정하세요)
# ============================================================
LIDAR1_FILE = "~/sl_per_ws/pcd_file/left"      # 라이다1 PLY 파일 left
LIDAR2_FILE = "~/sl_per_ws/pcd_file/right"      # 라이다2 PLY 파일 right
CALIB_FILE  = "calib_dual.txt"      # 캘리브레이션 결과 파일
POINT_SIZE  = 3.0

# ✅ 시각화 비활성화 (서버 환경 - 웹에서 별도 표시)
ENABLE_VISUALIZATION = False

# ✅ 라이다 파일이 mm 단위로 저장되어 있으면 0.001 (mm->m)
LIDAR_INPUT_UNIT_SCALE = 0.001

# 크롭 범위 (X_MIN, X_MAX, Y_MIN, Y_MAX, Z_MIN, Z_MAX) - 미터 단위
# None이면 크롭 안함
CROP_BOUNDS = (0.3,1.0,-1.15,0.45,0.35,1)
# 기존 : CROP_BOUNDS = (0.3,0.8,-1.3,0.6,0.35,1)
# CROP_BOUNDS = None  # 크롭 비활성화

# 저장 옵션
SAVE_MERGED = True
SAVE_CROPPED = True
SAVE_CONFIG = True
# ============================================================


def get_latest_pcd_file(path):
    """
    경로가 디렉토리면 최신 .pcd 파일을 찾아 반환
    경로가 파일이면 그대로 반환
    """
    # ~ 확장
    expanded_path = os.path.expanduser(path)

    # 디렉토리인 경우
    if os.path.isdir(expanded_path):
        pcd_files = glob.glob(os.path.join(expanded_path, "*.pcd"))
        if not pcd_files:
            raise FileNotFoundError(f"디렉토리에 .pcd 파일이 없습니다: {expanded_path}")
        # 가장 최근 수정된 파일 반환
        latest_file = max(pcd_files, key=os.path.getmtime)
        print(f"  📁 디렉토리에서 최신 파일 선택: {os.path.basename(latest_file)}")
        return latest_file

    # 파일인 경우 그대로 반환
    return expanded_path


def create_axes_lines(size=1.0, origin=[0, 0, 0]):
    """좌표축을 LineSet으로 생성"""
    origin = np.array(origin)
    
    points = [
        origin,
        origin + [size, 0, 0],
        origin + [0, size, 0],
        origin + [0, 0, size],
    ]
    
    lines = [[0, 1], [0, 2], [0, 3]]
    colors = [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
    
    axes = o3d.geometry.LineSet()
    axes.points = o3d.utility.Vector3dVector(points)
    axes.lines = o3d.utility.Vector2iVector(lines)
    axes.colors = o3d.utility.Vector3dVector(colors)
    
    return axes


def create_axes_with_labels(size=1.0, origin=[0, 0, 0]):
    """좌표축 + 라벨용 구체 생성"""
    geometries = []
    origin = np.array(origin)
    
    axes = create_axes_lines(size, origin)
    geometries.append(axes)
    
    sphere_radius = size * 0.05
    
    # X축 끝 (빨강)
    sphere_x = o3d.geometry.TriangleMesh.create_sphere(radius=sphere_radius)
    sphere_x.translate(origin + [size, 0, 0])
    sphere_x.paint_uniform_color([1, 0, 0])
    geometries.append(sphere_x)
    
    # Y축 끝 (초록)
    sphere_y = o3d.geometry.TriangleMesh.create_sphere(radius=sphere_radius)
    sphere_y.translate(origin + [0, size, 0])
    sphere_y.paint_uniform_color([0, 1, 0])
    geometries.append(sphere_y)
    
    # Z축 끝 (파랑)
    sphere_z = o3d.geometry.TriangleMesh.create_sphere(radius=sphere_radius)
    sphere_z.translate(origin + [0, 0, size])
    sphere_z.paint_uniform_color([0, 0, 1])
    geometries.append(sphere_z)
    
    # 원점 (흰색)
    sphere_origin = o3d.geometry.TriangleMesh.create_sphere(radius=sphere_radius * 1.5)
    sphere_origin.translate(origin)
    sphere_origin.paint_uniform_color([1, 1, 1])
    geometries.append(sphere_origin)
    
    return geometries


def create_grid(size=1.0, divisions=10, plane='xy', height=0):
    """그리드 생성"""
    lines = []
    points = []
    step = size / divisions
    half = size / 2
    
    idx = 0
    for i in range(divisions + 1):
        pos = -half + i * step
        
        if plane == 'xy':
            points.extend([[pos, -half, height], [pos, half, height]])
            lines.append([idx, idx + 1])
            idx += 2
            points.extend([[-half, pos, height], [half, pos, height]])
            lines.append([idx, idx + 1])
            idx += 2
    
    grid = o3d.geometry.LineSet()
    grid.points = o3d.utility.Vector3dVector(points)
    grid.lines = o3d.utility.Vector2iVector(lines)
    grid.paint_uniform_color([0.5, 0.5, 0.5])
    
    return grid


class DualLidarMerge:
    def __init__(self, lidar1_path, lidar2_path, calib_path, 
                 point_size=3.0, unit_scale=0.001):
        self.lidar1_path = lidar1_path
        self.lidar2_path = lidar2_path
        self.calib_path = calib_path
        self.point_size = point_size
        self.unit_scale = unit_scale  # mm -> m 변환용
        
        self.transformation_lidar1 = np.eye(4)
        self.transformation_lidar2 = np.eye(4)
        
        self.lidar1_original = None
        self.lidar2_original = None
        self.lidar1_world = None
        self.lidar2_world = None
        self.merged_pcd = None
        self.cropped_pcd = None
        
        self.crop_min = None
        self.crop_max = None
    
    def load_calib_file(self):
        """calib_dual.txt 파일에서 변환 행렬 로드"""
        print(f"\n📂 캘리브레이션 파일 로드: {self.calib_path}")
        
        try:
            with open(self.calib_path, 'r') as f:
                content = f.read()
        except Exception as e:
            print(f"❌ 파일 로드 실패: {e}")
            return False
        
        # 정규식 패턴 (calib_dual.txt 포맷에 맞게)
        pattern = r'transformation_lidar(\d)\s*=\s*np\.array\(\[\s*' \
                  r'\[([-\d., ]+)\],\s*' \
                  r'\[([-\d., ]+)\],\s*' \
                  r'\[([-\d., ]+)\],\s*' \
                  r'\[([-\d., ]+)\],?\s*\]\)'
        
        matches = re.findall(pattern, content)
        
        if len(matches) < 2:
            print("❌ 캘리브레이션 파일에서 행렬을 찾을 수 없습니다.")
            print("   파일 형식을 확인해주세요 (transformation_lidar1, transformation_lidar2)")
            return False
        
        for match in matches:
            lidar_num = int(match[0])
            rows = []
            for i in range(1, 5):
                row = [float(x.strip()) for x in match[i].split(',')]
                rows.append(row)
            
            matrix = np.array(rows)
            
            if lidar_num == 1:
                self.transformation_lidar1 = matrix
                print("\n✅ Lidar1 변환 행렬 로드 완료:")
            else:
                self.transformation_lidar2 = matrix
                print("\n✅ Lidar2 변환 행렬 로드 완료:")
            
            for row in matrix:
                print(f"    [{row[0]:12.6f}, {row[1]:12.6f}, {row[2]:12.6f}, {row[3]:12.6f}]")
        
        return True
    
    def load_pointclouds(self):
        """포인트클라우드 로드 및 단위 변환"""
        print(f"\n📂 포인트클라우드 로드")

        # 파일 저장 완료를 위한 대기 시간 (3초)
        print("  ⏳ 파일 저장 완료 대기 중... (3초)")
        time.sleep(3)

        # 디렉토리인 경우 최신 .pcd 파일 자동 선택
        try:
            lidar1_file = get_latest_pcd_file(self.lidar1_path)
            lidar2_file = get_latest_pcd_file(self.lidar2_path)
        except FileNotFoundError as e:
            print(f"  ❌ 파일 찾기 실패: {e}")
            return False

        try:
            self.lidar1_original = o3d.io.read_point_cloud(lidar1_file)
            if self.lidar1_original.is_empty():
                raise RuntimeError("빈 포인트클라우드")
            
            # mm -> m 변환
            if self.unit_scale != 1.0:
                self.lidar1_original.scale(self.unit_scale, center=(0, 0, 0))
            
            pts = np.asarray(self.lidar1_original.points)
            print(f"  ✅ Lidar1: {len(pts):,}개 포인트")
            print(f"     범위(m): X[{pts[:,0].min():.3f}~{pts[:,0].max():.3f}] "
                  f"Y[{pts[:,1].min():.3f}~{pts[:,1].max():.3f}] "
                  f"Z[{pts[:,2].min():.3f}~{pts[:,2].max():.3f}]")
        except Exception as e:
            print(f"  ❌ Lidar1 로드 실패: {e}")
            return False
        
        try:
            self.lidar2_original = o3d.io.read_point_cloud(lidar2_file)
            if self.lidar2_original.is_empty():
                raise RuntimeError("빈 포인트클라우드")
            
            # mm -> m 변환
            if self.unit_scale != 1.0:
                self.lidar2_original.scale(self.unit_scale, center=(0, 0, 0))
            
            pts = np.asarray(self.lidar2_original.points)
            print(f"  ✅ Lidar2: {len(pts):,}개 포인트")
            print(f"     범위(m): X[{pts[:,0].min():.3f}~{pts[:,0].max():.3f}] "
                  f"Y[{pts[:,1].min():.3f}~{pts[:,1].max():.3f}] "
                  f"Z[{pts[:,2].min():.3f}~{pts[:,2].max():.3f}]")
        except Exception as e:
            print(f"  ❌ Lidar2 로드 실패: {e}")
            return False
        
        if self.unit_scale != 1.0:
            print(f"\n  ℹ️  단위 변환 적용됨: scale={self.unit_scale} (mm->m)")
        
        return True
    
    def transform_to_world(self):
        """두 라이다를 로봇 월드 좌표계로 변환"""
        print(f"\n🔄 로봇 월드 좌표계로 변환 중...")
        
        # Lidar1 변환 (빨강)
        self.lidar1_world = copy.deepcopy(self.lidar1_original)
        self.lidar1_world.transform(self.transformation_lidar1)
        self.lidar1_world.paint_uniform_color([1.0, 0.3, 0.3])  # 빨강
        
        # Lidar2 변환 (파랑)
        self.lidar2_world = copy.deepcopy(self.lidar2_original)
        self.lidar2_world.transform(self.transformation_lidar2)
        self.lidar2_world.paint_uniform_color([0.3, 0.5, 1.0])  # 파랑
        
        # 병합
        self.merged_pcd = self.lidar1_world + self.lidar2_world
        
        # 결과 출력
        pts1 = np.asarray(self.lidar1_world.points)
        pts2 = np.asarray(self.lidar2_world.points)
        
        print(f"\n  ✅ Lidar1 (월드): {len(pts1):,}개")
        print(f"     범위(m): X[{pts1[:,0].min():.3f}~{pts1[:,0].max():.3f}] "
              f"Y[{pts1[:,1].min():.3f}~{pts1[:,1].max():.3f}] "
              f"Z[{pts1[:,2].min():.3f}~{pts1[:,2].max():.3f}]")
        
        print(f"\n  ✅ Lidar2 (월드): {len(pts2):,}개")
        print(f"     범위(m): X[{pts2[:,0].min():.3f}~{pts2[:,0].max():.3f}] "
              f"Y[{pts2[:,1].min():.3f}~{pts2[:,1].max():.3f}] "
              f"Z[{pts2[:,2].min():.3f}~{pts2[:,2].max():.3f}]")
        
        print(f"\n  ✅ 병합 완료: {len(self.merged_pcd.points):,}개")
        
        bbox = self.merged_pcd.get_axis_aligned_bounding_box()
        min_b, max_b = bbox.get_min_bound(), bbox.get_max_bound()
        print(f"\n📊 병합된 포인트클라우드 범위 (로봇 월드 좌표, m):")
        print(f"  X: {min_b[0]:.4f} ~ {max_b[0]:.4f}")
        print(f"  Y: {min_b[1]:.4f} ~ {max_b[1]:.4f}")
        print(f"  Z: {min_b[2]:.4f} ~ {max_b[2]:.4f}")
    
    def set_crop_bounds(self, x_min, x_max, y_min, y_max, z_min, z_max):
        """크롭 범위 설정"""
        self.crop_min = np.array([x_min, y_min, z_min])
        self.crop_max = np.array([x_max, y_max, z_max])
        
        print(f"\n✂️ 크롭 범위 설정:")
        print(f"  X: {self.crop_min[0]:.3f} ~ {self.crop_max[0]:.3f}")
        print(f"  Y: {self.crop_min[1]:.3f} ~ {self.crop_max[1]:.3f}")
        print(f"  Z: {self.crop_min[2]:.3f} ~ {self.crop_max[2]:.3f}")
    
    def crop_pointcloud(self):
        """포인트클라우드 크롭"""
        print(f"\n✂️ 크롭 적용 중...")
        
        crop_box = o3d.geometry.AxisAlignedBoundingBox(
            min_bound=self.crop_min,
            max_bound=self.crop_max
        )
        
        self.cropped_pcd = self.merged_pcd.crop(crop_box)
        
        print(f"  원본: {len(self.merged_pcd.points):,}개")
        print(f"  크롭 후: {len(self.cropped_pcd.points):,}개")
        print(f"  제거됨: {len(self.merged_pcd.points) - len(self.cropped_pcd.points):,}개")
    
    def visualize_world(self, pcd, title="로봇 월드 좌표계", axes_size=0.3):
        """로봇 월드 좌표계 기준 시각화"""
        print(f"\n🖥️ 시각화: {title}")
        print("  좌표축 (로봇 월드): 🔴X  🟢Y  🔵Z  ⚪원점")
        print("  포인트: 🔴Lidar1  🔵Lidar2")
        print("  Q: 종료")
        
        geometries = [pcd]
        
        # 로봇 월드 좌표축 (원점에서 시작)
        axes_geoms = create_axes_with_labels(size=axes_size, origin=[0, 0, 0])
        geometries.extend(axes_geoms)
        
        # XY 그리드 (Z=0 평면)
        grid = create_grid(size=axes_size * 3, divisions=15, plane='xy', height=0)
        geometries.append(grid)
        
        # 시각화
        vis = o3d.visualization.Visualizer()
        vis.create_window(window_name=title, width=1600, height=900)
        
        for geom in geometries:
            vis.add_geometry(geom)
        
        opt = vis.get_render_option()
        opt.point_size = self.point_size
        opt.background_color = np.array([0.1, 0.1, 0.1])
        opt.show_coordinate_frame = False
        
        # 초기 뷰포인트 설정 (위에서 보기)
        ctr = vis.get_view_control()
        ctr.set_zoom(0.5)
        
        vis.run()
        vis.destroy_window()
    
    def visualize_cropped(self, show_original=True):
        """크롭 결과 시각화"""
        print("\n🖥️ 크롭 결과 시각화")
        print("  좌표축 (로봇 월드): 🔴X  🟢Y  🔵Z  ⚪원점")
        print("  포인트: 🔴Lidar1  🔵Lidar2")
        print("  초록 박스: 크롭 영역")
        print("  Q: 종료")
        
        geometries = []
        
        # 크롭된 포인트클라우드
        geometries.append(self.cropped_pcd)
        
        # 로봇 월드 좌표축
        axes_geoms = create_axes_with_labels(size=0.3, origin=[0, 0, 0])
        geometries.extend(axes_geoms)
        
        # XY 그리드
        grid = create_grid(size=1.0, divisions=10, plane='xy', height=0)
        geometries.append(grid)
        
        # 크롭 박스 표시
        if self.crop_min is not None and self.crop_max is not None:
            crop_box = o3d.geometry.AxisAlignedBoundingBox(
                min_bound=self.crop_min,
                max_bound=self.crop_max
            )
            crop_box.color = (0, 1, 0)
            geometries.append(crop_box)
        
        # 원본 (반투명 회색)
        if show_original:
            original_faded = copy.deepcopy(self.merged_pcd)
            original_faded.paint_uniform_color([0.3, 0.3, 0.3])
            geometries.insert(0, original_faded)
        
        vis = o3d.visualization.Visualizer()
        vis.create_window(
            window_name="크롭 결과 (로봇 월드 좌표계)",
            width=1600, height=900
        )
        
        for geom in geometries:
            vis.add_geometry(geom)
        
        opt = vis.get_render_option()
        opt.point_size = self.point_size
        opt.background_color = np.array([0.1, 0.1, 0.1])
        
        vis.run()
        vis.destroy_window()
    
    def save_results(self, save_merged=True, save_cropped=True, save_config=True):
        """결과 저장"""
        print("\n" + "="*70)
        print("💾 결과 저장")
        print("="*70)
        
        if save_merged and self.merged_pcd is not None:
            o3d.io.write_point_cloud("merged_world.pcd", self.merged_pcd)
            print("  ✅ merged_world.pcd 저장 완료")
        
        if save_cropped and self.cropped_pcd is not None:
            o3d.io.write_point_cloud("cropped_world.pcd", self.cropped_pcd)
            print("  ✅ cropped_world.pcd 저장 완료")
        
        if save_config and self.crop_min is not None:
            with open("crop_config.txt", 'w') as f:
                f.write("# Crop Configuration (Robot World Coordinates, meter)\n")
                f.write(f"x_min = {self.crop_min[0]:.6f}\n")
                f.write(f"x_max = {self.crop_max[0]:.6f}\n")
                f.write(f"y_min = {self.crop_min[1]:.6f}\n")
                f.write(f"y_max = {self.crop_max[1]:.6f}\n")
                f.write(f"z_min = {self.crop_min[2]:.6f}\n")
                f.write(f"z_max = {self.crop_max[2]:.6f}\n")
            print("  ✅ crop_config.txt 저장 완료")
    
    def run(self, crop_bounds=None, save_merged=True, save_cropped=True, save_config=True):
        """메인 실행"""
        print("\n" + "="*70)
        print("🎯 듀얼 라이다 정합 (로봇 월드 좌표계)")
        print("="*70)
        
        # 1. 캘리브레이션 로드
        if not self.load_calib_file():
            return
        
        # 2. 포인트클라우드 로드
        if not self.load_pointclouds():
            return
        
        # 3. 월드 좌표계로 변환
        self.transform_to_world()
        
        # 4. 병합 결과 시각화 (서버 환경에서는 건너뜀)
        if ENABLE_VISUALIZATION:
            print("\n" + "="*70)
            print("[Step 4] 병합 결과 확인 (로봇 월드 좌표계)")
            print("="*70)
            self.visualize_world(
                self.merged_pcd,
                title="듀얼 라이다 정합 결과 (로봇 월드 좌표계)",
                axes_size=0.3
            )
        else:
            print("\n[INFO] 시각화 비활성화됨 (서버 환경)")

        # 5. 크롭 (옵션)
        if crop_bounds is not None:
            x_min, x_max, y_min, y_max, z_min, z_max = crop_bounds
            self.set_crop_bounds(x_min, x_max, y_min, y_max, z_min, z_max)
            self.crop_pointcloud()
            if ENABLE_VISUALIZATION:
                self.visualize_cropped(show_original=True)
        else:
            self.cropped_pcd = self.merged_pcd
            print("\n  ℹ️  크롭 비활성화됨")
        
        # 6. 저장
        self.save_results(save_merged, save_cropped, save_config if crop_bounds else False)
        
        print("\n" + "="*70)
        print("✨ 완료!")
        print("="*70)


def main():
    print("\n" + "="*70)
    print("🚀 듀얼 라이다 정합 도구 (PLY -> 로봇 월드 좌표계)")
    print("="*70)
    print(f"  Lidar1: {LIDAR1_FILE}")
    print(f"  Lidar2: {LIDAR2_FILE}")
    print(f"  Calib:  {CALIB_FILE}")
    print(f"  단위 변환: {'mm->m' if LIDAR_INPUT_UNIT_SCALE == 0.001 else f'scale={LIDAR_INPUT_UNIT_SCALE}'}")
    print("="*70)
    
    merger = DualLidarMerge(
        LIDAR1_FILE, 
        LIDAR2_FILE, 
        CALIB_FILE, 
        POINT_SIZE,
        LIDAR_INPUT_UNIT_SCALE
    )
    
    merger.run(
        crop_bounds=CROP_BOUNDS,
        save_merged=SAVE_MERGED, 
        save_cropped=SAVE_CROPPED, 
        save_config=SAVE_CONFIG
    )


if __name__ == "__main__":
    main()