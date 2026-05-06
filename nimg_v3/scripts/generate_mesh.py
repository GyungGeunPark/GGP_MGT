#!/usr/bin/env python3
"""
generate_mesh.py - RGBD 참조 이미지로부터 메시 생성

housing_M 등 참조 이미지(RGB + Depth + YOLO labels)로부터
Open3D TSDF Fusion을 사용해 3D 메시를 생성합니다.

BundleSDF/NeRF 없이 TSDF 볼륨 통합 방식으로 동작하므로
CUDA 확장 없이 빠르게 메시를 생성할 수 있습니다.

사용법:
    # housing_M 메시 생성
    python generate_mesh.py --data_dir ../models/neural_fields/housing_M/reference_images

    # 커스텀 설정
    python generate_mesh.py --data_dir <path> --voxel_size 0.002 --output mesh.obj

    # BundleSDF 모드 (FoundationPose 설치 필요)
    python generate_mesh.py --data_dir <path> --method bundlesdf

Author: FurSys AI Team
"""

import argparse
import logging
import os
import sys
import re
import glob
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import cv2
import numpy as np

logging.basicConfig(level=logging.INFO, format='%(asctime)s [%(levelname)s] %(message)s')
logger = logging.getLogger(__name__)


# ============================================================
# D455 기본 내부 파라미터 (640x480)
# ============================================================
DEFAULT_INTRINSICS = {
    'fx': 383.883, 'fy': 383.883,
    'cx': 320.499, 'cy': 237.913,
    'width': 640, 'height': 480
}


def parse_view_angle(filename: str) -> Tuple[float, float, str]:
    """
    파일명에서 뷰 각도 파싱

    Returns:
        (azimuth_deg, elevation_deg, view_type)
        view_type: 'horizontal', 'top', 'bottom'
    """
    name = Path(filename).stem.lower()

    # depth 파일 제외
    if '_depth_' in name:
        return None

    # top_XXdeg
    m = re.match(r'top_(\d+)deg', name)
    if m:
        az = float(m.group(1))
        return (az, 55.0, 'top')  # 위에서 55° 각도로 내려다봄

    # bottom_XXdeg
    m = re.match(r'bottom_(\d+)deg', name)
    if m:
        az = float(m.group(1))
        return (az, -55.0, 'bottom')  # 아래에서 55° 각도로 올려다봄

    # XXdeg (수평)
    m = re.match(r'(\d+)deg', name)
    if m:
        az = float(m.group(1))
        return (az, 0.0, 'horizontal')

    return None


def yolo_polygon_to_mask(label_path: str, img_width: int, img_height: int) -> np.ndarray:
    """
    YOLO 세그멘테이션 라벨(폴리곤 좌표)을 바이너리 마스크로 변환

    YOLO 형식: class_id x1 y1 x2 y2 ... (정규화 좌표 0~1)
    """
    mask = np.zeros((img_height, img_width), dtype=np.uint8)

    if not os.path.exists(label_path):
        logger.warning(f"라벨 파일 없음: {label_path}")
        return mask

    with open(label_path, 'r') as f:
        for line in f:
            parts = line.strip().split()
            if len(parts) < 7:  # class_id + 최소 3개 점 (6 좌표)
                continue

            # class_id 제외하고 좌표 파싱
            coords = [float(x) for x in parts[1:]]
            points = []
            for i in range(0, len(coords), 2):
                if i + 1 < len(coords):
                    px = int(coords[i] * img_width)
                    py = int(coords[i + 1] * img_height)
                    points.append([px, py])

            if len(points) >= 3:
                pts = np.array(points, dtype=np.int32).reshape((-1, 1, 2))
                cv2.fillPoly(mask, [pts], 255)

    return mask


def create_lookat_pose(eye: np.ndarray, target: np.ndarray, up: np.ndarray = None) -> np.ndarray:
    """
    Look-at 카메라 자세 행렬 생성 (OpenCV 컨벤션: Z forward, Y down, X right)

    Returns:
        4x4 extrinsic matrix (world-to-camera)
    """
    if up is None:
        up = np.array([0.0, -1.0, 0.0])  # OpenCV: Y down

    # 카메라 Z축 = 시선 방향 (eye → target)
    forward = target - eye
    forward = forward / np.linalg.norm(forward)

    # 카메라 X축 = right = forward × up
    right = np.cross(forward, up)
    norm = np.linalg.norm(right)
    if norm < 1e-6:
        # forward와 up이 거의 평행 → 대안 up 벡터 사용
        up = np.array([0.0, 0.0, 1.0])
        right = np.cross(forward, up)
        norm = np.linalg.norm(right)
    right = right / norm

    # 카메라 Y축 = forward × right (정확한 직교)
    down = np.cross(forward, right)

    # Rotation: world → camera
    R = np.stack([right, down, forward], axis=0)  # 3x3

    # Translation: world → camera
    t = -R @ eye

    pose = np.eye(4)
    pose[:3, :3] = R
    pose[:3, 3] = t
    return pose


def estimate_camera_distance(depth_img: np.ndarray, mask: np.ndarray) -> float:
    """
    Depth 이미지에서 카메라-객체 거리 추정 (마스크 영역의 중앙값)

    Args:
        depth_img: uint16 depth (mm)
        mask: 바이너리 마스크

    Returns:
        거리 (m)
    """
    valid = (mask > 0) & (depth_img > 0)
    if valid.sum() == 0:
        return 0.5  # 기본값

    depths_mm = depth_img[valid].astype(np.float64)
    median_mm = np.median(depths_mm)
    return median_mm / 1000.0


def create_camera_poses_from_angles(
    views: List[Dict],
    camera_distance: float,
    per_view_distances: List[float] = None
) -> List[np.ndarray]:
    """
    터틀테이블 각도로부터 카메라 외부 파라미터(extrinsic) 생성

    카메라가 고정되어 있고 객체가 터틀테이블 위에서 회전하는 설정.
    → 등가적으로 카메라가 객체 주위를 공전하는 것으로 모델링.

    Args:
        views: [{'azimuth': float, 'elevation': float, 'type': str}, ...]
        camera_distance: 기본 카메라-객체 중심 거리 (m)
        per_view_distances: 뷰별 카메라 거리 (있으면 camera_distance 대신 사용)

    Returns:
        카메라 외부 파라미터 행렬 리스트 (world-to-camera, 4x4)
    """
    poses = []
    target = np.array([0.0, 0.0, 0.0])  # 객체 중심

    for i, view in enumerate(views):
        az_rad = np.deg2rad(view['azimuth'])
        el_rad = np.deg2rad(view['elevation'])
        d = per_view_distances[i] if per_view_distances else camera_distance

        # 구면 좌표 → 직교 좌표 (OpenCV: Y down)
        # azimuth: 0°=+Z, 90°=+X, 180°=-Z, 270°=-X
        # elevation: +는 위 (카메라가 위에 있음), -는 아래
        x = d * np.cos(el_rad) * np.sin(az_rad)
        y = -d * np.sin(el_rad)  # OpenCV Y down: 위로 가면 y 음수
        z = d * np.cos(el_rad) * np.cos(az_rad)

        eye = np.array([x, y, z])

        # Look-at 자세 생성
        # Top/bottom 뷰에서는 up 벡터 조정 필요
        if abs(view['elevation']) > 80:
            # 거의 수직 → up 벡터를 Z 방향으로
            up = np.array([0.0, 0.0, -np.sign(view['elevation'])])
        else:
            up = np.array([0.0, -1.0, 0.0])  # 기본 up (OpenCV Y down)

        pose = create_lookat_pose(eye, target, up)
        poses.append(pose)

    return poses


def load_reference_data(data_dir: str, intrinsics: Dict = None,
                        horizontal_only: bool = False) -> Dict:
    """
    참조 이미지 데이터 로드 (RGB + Depth + YOLO labels)

    Args:
        data_dir: 참조 이미지 디렉토리
        intrinsics: 카메라 내부 파라미터
        horizontal_only: True면 수평 뷰만 사용 (top/bottom 제외)

    Returns:
        {
            'rgbs': list of (H,W,3) uint8,
            'depths': list of (H,W) float32 (meters),
            'masks': list of (H,W) uint8,
            'views': list of {'azimuth', 'elevation', 'type', 'filename'},
            'camera_distance': float
        }
    """
    if intrinsics is None:
        intrinsics = DEFAULT_INTRINSICS

    img_dir = os.path.join(data_dir, 'images')
    label_dir = os.path.join(data_dir, 'labels')

    if not os.path.isdir(img_dir):
        # images/ 서브폴더가 없으면 data_dir 자체를 사용
        img_dir = data_dir
        label_dir = data_dir

    # RGB 이미지 목록 (depth 제외)
    all_images = sorted(glob.glob(os.path.join(img_dir, '*.png')) +
                        glob.glob(os.path.join(img_dir, '*.jpg')))

    rgbs = []
    depths = []
    masks = []
    views = []
    distances = []

    for img_path in all_images:
        filename = os.path.basename(img_path)

        # depth 이미지는 건너뛰기
        if '_depth_' in filename:
            continue

        # 뷰 각도 파싱
        angle_info = parse_view_angle(filename)
        if angle_info is None:
            logger.warning(f"각도 파싱 실패, 건너뛰기: {filename}")
            continue

        azimuth, elevation, view_type = angle_info

        # 수평 뷰만 필터링
        if horizontal_only and view_type != 'horizontal':
            logger.info(f"  건너뛰기 (horizontal_only): {filename}")
            continue

        # RGB 로드
        rgb = cv2.imread(img_path, cv2.IMREAD_COLOR)
        if rgb is None:
            logger.warning(f"RGB 로드 실패: {img_path}")
            continue
        rgb = cv2.cvtColor(rgb, cv2.COLOR_BGR2RGB)
        h, w = rgb.shape[:2]

        # Depth 로드 (같은 해시의 _depth_ 파일)
        # 예: 0deg_png.rf.HASH.png → 0deg_depth_png.rf.HASH.png
        stem = Path(filename).stem  # '0deg_png.rf.HASH'
        ext = Path(filename).suffix  # '.png'

        # '_png.rf.' 기준으로 분리해서 depth 파일명 생성
        parts = stem.split('_png.rf.')
        if len(parts) == 2:
            depth_filename = f"{parts[0]}_depth_png.rf.{parts[1]}{ext}"
        else:
            depth_filename = filename.replace('.', '_depth.', 1)

        depth_path = os.path.join(img_dir, depth_filename)
        if not os.path.exists(depth_path):
            logger.warning(f"Depth 파일 없음: {depth_path}")
            continue

        depth_raw = cv2.imread(depth_path, cv2.IMREAD_UNCHANGED)
        if depth_raw is None:
            logger.warning(f"Depth 로드 실패: {depth_path}")
            continue

        # uint16 mm → float32 m
        depth_m = depth_raw.astype(np.float32) / 1000.0

        # YOLO 라벨 → 마스크
        label_filename = stem + '.txt'
        label_path = os.path.join(label_dir, label_filename)
        mask = yolo_polygon_to_mask(label_path, w, h)

        if mask.sum() == 0:
            logger.warning(f"빈 마스크: {label_path}")
            continue

        # 카메라 거리 추정
        dist = estimate_camera_distance(depth_raw, mask)
        distances.append(dist)

        rgbs.append(rgb)
        depths.append(depth_m)
        masks.append(mask)
        views.append({
            'azimuth': azimuth,
            'elevation': elevation,
            'type': view_type,
            'filename': filename
        })

        logger.info(f"  로드됨: {filename} (az={azimuth:.0f}°, el={elevation:.0f}°, "
                     f"dist={dist:.3f}m, mask_pixels={mask.sum()//255})")

    if not rgbs:
        raise RuntimeError(f"유효한 참조 이미지를 찾을 수 없습니다: {data_dir}")

    camera_distance = float(np.median(distances))
    logger.info(f"총 {len(rgbs)}개 뷰 로드, 추정 카메라 거리: {camera_distance:.3f}m")
    logger.info(f"뷰별 거리 범위: {min(distances):.3f}m ~ {max(distances):.3f}m")

    return {
        'rgbs': rgbs,
        'depths': depths,
        'masks': masks,
        'views': views,
        'camera_distance': camera_distance,
        'per_view_distances': distances,
    }


def generate_mesh_tsdf(
    data: Dict,
    intrinsics: Dict = None,
    voxel_size: float = 0.002,
    sdf_trunc: float = 0.01,
    depth_trunc: float = 1.5,
) -> 'open3d.geometry.TriangleMesh':
    """
    Open3D TSDF Fusion으로 메시 생성

    1. 카메라 자세 생성
    2. Depth + Mask → TSDF 볼륨 통합
    3. Marching Cubes 메시 추출
    4. 노이즈 제거 & 정리

    Args:
        data: load_reference_data() 결과
        intrinsics: 카메라 내부 파라미터
        voxel_size: TSDF 복셀 크기 (m)
        sdf_trunc: SDF 절단 거리 (m)
        depth_trunc: 최대 깊이 절단 (m)

    Returns:
        Open3D TriangleMesh
    """
    import open3d as o3d

    if intrinsics is None:
        intrinsics = DEFAULT_INTRINSICS

    # 카메라 자세 생성 (뷰별 거리 사용)
    camera_distance = data['camera_distance']
    per_view_distances = data.get('per_view_distances')
    logger.info(f"카메라 자세 생성 (중앙값 거리={camera_distance:.3f}m, 뷰별 거리 사용)...")
    extrinsics = create_camera_poses_from_angles(
        data['views'], camera_distance, per_view_distances=per_view_distances
    )

    # Open3D 카메라 intrinsic 생성
    o3d_intrinsic = o3d.camera.PinholeCameraIntrinsic(
        width=intrinsics['width'],
        height=intrinsics['height'],
        fx=intrinsics['fx'],
        fy=intrinsics['fy'],
        cx=intrinsics['cx'],
        cy=intrinsics['cy']
    )

    # TSDF 볼륨 생성
    logger.info(f"TSDF 볼륨 생성 (voxel={voxel_size}m, trunc={sdf_trunc}m)...")
    volume = o3d.pipelines.integration.ScalableTSDFVolume(
        voxel_length=voxel_size,
        sdf_trunc=sdf_trunc,
        color_type=o3d.pipelines.integration.TSDFVolumeColorType.RGB8,
    )

    # 각 뷰에서 TSDF 볼륨에 depth 통합
    for i, (rgb, depth, mask, extr) in enumerate(
        zip(data['rgbs'], data['depths'], data['masks'], extrinsics)
    ):
        view = data['views'][i]

        # 마스크 외부 depth를 0으로 설정 (객체 영역만 사용)
        depth_masked = depth.copy()
        depth_masked[mask == 0] = 0.0
        depth_masked[depth_masked > depth_trunc] = 0.0

        # 유효 depth 픽셀 수 확인
        valid_count = (depth_masked > 0).sum()
        if valid_count < 100:
            logger.warning(f"  뷰 {i} ({view['filename']}): 유효 depth 부족 ({valid_count}px), 건너뛰기")
            continue

        # Open3D 이미지 변환
        rgb_o3d = o3d.geometry.Image(rgb.astype(np.uint8))
        depth_o3d = o3d.geometry.Image(depth_masked.astype(np.float32))
        rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
            rgb_o3d, depth_o3d,
            depth_scale=1.0,  # 이미 미터 단위
            depth_trunc=depth_trunc,
            convert_rgb_to_intensity=False
        )

        # extrinsic: world-to-camera (4x4)
        volume.integrate(rgbd, o3d_intrinsic, np.linalg.inv(extr))

        logger.info(f"  뷰 {i}: {view['filename']} (az={view['azimuth']:.0f}°, "
                     f"el={view['elevation']:.0f}°) → {valid_count}px 통합됨")

    # 메시 추출
    logger.info("TSDF 볼륨에서 메시 추출 중...")
    mesh = volume.extract_triangle_mesh()
    mesh.compute_vertex_normals()

    vert_count = len(mesh.vertices)
    face_count = len(mesh.triangles)
    logger.info(f"원본 메시: {vert_count} vertices, {face_count} faces")

    if vert_count == 0:
        logger.error("메시 추출 실패: 정점이 없습니다. 카메라 자세나 depth 데이터를 확인하세요.")
        return mesh

    # === 메시 정리 ===

    # 1. 연결된 컴포넌트 중 가장 큰 것만 유지
    triangle_clusters, cluster_n_triangles, cluster_area = (
        mesh.cluster_connected_triangles()
    )
    triangle_clusters = np.asarray(triangle_clusters)
    cluster_n_triangles = np.asarray(cluster_n_triangles)

    if len(cluster_n_triangles) > 1:
        largest_cluster = cluster_n_triangles.argmax()
        triangles_to_remove = triangle_clusters != largest_cluster
        mesh.remove_triangles_by_mask(triangles_to_remove)
        mesh.remove_unreferenced_vertices()
        removed = triangles_to_remove.sum()
        logger.info(f"  노이즈 제거: {removed} 삼각형 제거 "
                     f"({len(cluster_n_triangles)}개 클러스터 중 최대 유지)")

    # 2. 퇴화 삼각형 제거
    mesh.remove_degenerate_triangles()
    mesh.remove_duplicated_triangles()
    mesh.remove_duplicated_vertices()
    mesh.remove_non_manifold_edges()

    # 3. 스무딩
    mesh = mesh.filter_smooth_simple(number_of_iterations=3)
    mesh.compute_vertex_normals()

    final_verts = len(mesh.vertices)
    final_faces = len(mesh.triangles)

    # 바운딩 박스 정보
    bbox = mesh.get_axis_aligned_bounding_box()
    extent = bbox.get_extent()
    logger.info(f"최종 메시: {final_verts} vertices, {final_faces} faces")
    logger.info(f"바운딩 박스: {extent[0]:.4f} x {extent[1]:.4f} x {extent[2]:.4f} m")

    return mesh


def generate_mesh_pointcloud(
    data: Dict,
    intrinsics: Dict = None,
    voxel_size: float = 0.002,
) -> 'open3d.geometry.TriangleMesh':
    """
    포인트 클라우드 융합 + Poisson Surface Reconstruction으로 메시 생성

    TSDF가 실패할 경우의 대안 방법.
    """
    import open3d as o3d

    if intrinsics is None:
        intrinsics = DEFAULT_INTRINSICS

    camera_distance = data['camera_distance']
    per_view_distances = data.get('per_view_distances')
    extrinsics = create_camera_poses_from_angles(
        data['views'], camera_distance, per_view_distances=per_view_distances
    )

    fx, fy = intrinsics['fx'], intrinsics['fy']
    cx, cy = intrinsics['cx'], intrinsics['cy']

    # 모든 뷰의 포인트 클라우드 합치기
    all_points = []
    all_colors = []

    for i, (rgb, depth, mask, extr) in enumerate(
        zip(data['rgbs'], data['depths'], data['masks'], extrinsics)
    ):
        h, w = depth.shape

        # 마스크 영역만
        valid = (mask > 0) & (depth > 0.01) & (depth < 1.5)
        if valid.sum() < 50:
            continue

        # 2D → 3D 역투영 (카메라 좌표)
        v_coords, u_coords = np.where(valid)
        z = depth[valid]
        x = (u_coords - cx) * z / fx
        y = (v_coords - cy) * z / fy

        pts_cam = np.stack([x, y, z], axis=-1)  # (N, 3) 카메라 좌표

        # 카메라 → 월드 좌표 변환
        cam_to_world = np.linalg.inv(extr)
        R = cam_to_world[:3, :3]
        t = cam_to_world[:3, 3]
        pts_world = (R @ pts_cam.T).T + t

        # 색상
        colors = rgb[valid].astype(np.float64) / 255.0

        all_points.append(pts_world)
        all_colors.append(colors)

    if not all_points:
        raise RuntimeError("포인트 클라우드 생성 실패: 유효한 뷰가 없습니다.")

    all_points = np.vstack(all_points)
    all_colors = np.vstack(all_colors)

    # Open3D 포인트 클라우드 생성
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(all_points)
    pcd.colors = o3d.utility.Vector3dVector(all_colors)

    logger.info(f"융합 포인트 클라우드: {len(all_points)}개 점")

    # Voxel downsampling
    pcd = pcd.voxel_down_sample(voxel_size)
    logger.info(f"다운샘플링 후: {len(pcd.points)}개 점")

    # 이상치 제거
    pcd, ind = pcd.remove_statistical_outlier(nb_neighbors=30, std_ratio=2.0)
    logger.info(f"이상치 제거 후: {len(pcd.points)}개 점")

    # DBSCAN 클러스터링으로 가장 큰 클러스터 유지
    labels = np.array(pcd.cluster_dbscan(eps=voxel_size * 5, min_points=10))
    if len(labels) > 0 and labels.max() >= 0:
        unique_labels, counts = np.unique(labels[labels >= 0], return_counts=True)
        if len(unique_labels) > 0:
            largest = unique_labels[counts.argmax()]
            keep = labels == largest
            pcd = pcd.select_by_index(np.where(keep)[0])
            logger.info(f"클러스터링 후: {len(pcd.points)}개 점")

    # 법선 추정
    pcd.estimate_normals(
        search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_size * 10, max_nn=30)
    )
    pcd.orient_normals_consistent_tangent_plane(30)

    # Poisson Surface Reconstruction
    logger.info("Poisson 표면 재구성 중...")
    mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(
        pcd, depth=9, linear_fit=True
    )

    # 밀도가 낮은 면 제거 (외곽 노이즈)
    densities = np.asarray(densities)
    density_threshold = np.quantile(densities, 0.05)
    vertices_to_remove = densities < density_threshold
    mesh.remove_vertices_by_mask(vertices_to_remove)
    mesh.compute_vertex_normals()

    logger.info(f"Poisson 메시: {len(mesh.vertices)} vertices, {len(mesh.triangles)} faces")

    return mesh


def generate_mesh_bundlesdf(
    data: Dict,
    intrinsics: Dict = None,
    config_path: str = None,
) -> 'trimesh.Trimesh':
    """
    FoundationPose BundleSDF로 메시 생성 (NeRF 기반)

    BundleSDF CUDA 확장이 필요합니다.
    """
    fp_dir = os.path.join(os.path.dirname(__file__), '../../FoundationPose')
    sys.path.insert(0, os.path.join(fp_dir, 'bundlesdf'))
    sys.path.insert(0, fp_dir)

    try:
        from run_nerf import run_neural_object_field
        import yaml
    except ImportError as e:
        raise RuntimeError(
            f"BundleSDF를 불러올 수 없습니다: {e}\n"
            f"TSDF 방식을 사용하세요: --method tsdf"
        )

    if intrinsics is None:
        intrinsics = DEFAULT_INTRINSICS

    # BundleSDF config 로드
    if config_path is None:
        config_path = os.path.join(fp_dir, 'bundlesdf', 'config_linemod.yml')

    with open(config_path, 'r') as f:
        cfg = yaml.safe_load(f)

    # 카메라 내부 파라미터 (3x3)
    K = np.array([
        [intrinsics['fx'], 0, intrinsics['cx']],
        [0, intrinsics['fy'], intrinsics['cy']],
        [0, 0, 1]
    ])

    # 데이터 준비
    rgbs = np.stack(data['rgbs'])
    depths = np.stack(data['depths'])
    masks = np.stack(data['masks'])

    # 카메라 자세 (cam_in_ob, OpenCV 컨벤션)
    camera_distance = data['camera_distance']
    extrinsics = create_camera_poses_from_angles(data['views'], camera_distance)
    # extrinsic은 world-to-camera, BundleSDF는 camera-in-object(=camera-to-world)를 요구
    cam_in_obs = np.stack([np.linalg.inv(e) for e in extrinsics])

    save_dir = os.path.join(os.path.dirname(data.get('data_dir', '/tmp')), 'bundlesdf_output')

    logger.info("BundleSDF NeRF 학습 시작...")
    mesh = run_neural_object_field(cfg, K, rgbs, depths, masks, cam_in_obs, save_dir=save_dir)

    logger.info(f"BundleSDF 메시: {len(mesh.vertices)} vertices, {len(mesh.faces)} faces")
    return mesh


def export_mesh(mesh, output_path: str, as_trimesh: bool = False):
    """메시를 파일로 내보내기"""
    import open3d as o3d

    output_path = str(output_path)
    os.makedirs(os.path.dirname(output_path) or '.', exist_ok=True)

    if isinstance(mesh, o3d.geometry.TriangleMesh):
        if output_path.endswith('.obj'):
            # Open3D → Trimesh 변환 후 OBJ 저장 (OBJ 호환성)
            try:
                import trimesh
                vertices = np.asarray(mesh.vertices)
                faces = np.asarray(mesh.triangles)
                colors = np.asarray(mesh.vertex_colors) if mesh.has_vertex_colors() else None

                tm = trimesh.Trimesh(
                    vertices=vertices,
                    faces=faces,
                    vertex_colors=(colors * 255).astype(np.uint8) if colors is not None else None,
                )
                tm.export(output_path)
                logger.info(f"메시 저장됨 (trimesh OBJ): {output_path}")
                return
            except ImportError:
                pass

        # Open3D 직접 저장
        o3d.io.write_triangle_mesh(output_path, mesh)
        logger.info(f"메시 저장됨 (Open3D): {output_path}")
    else:
        # trimesh 객체
        mesh.export(output_path)
        logger.info(f"메시 저장됨 (trimesh): {output_path}")


def visualize_result(mesh, data: Dict, intrinsics: Dict):
    """결과 시각화 (선택적)"""
    import open3d as o3d

    # 좌표축
    coord = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.05)

    geometries = [mesh, coord]

    logger.info("시각화 창을 열고 있습니다... (닫으려면 Q 키)")
    o3d.visualization.draw_geometries(
        geometries,
        window_name="Generated Mesh",
        width=1280, height=720,
    )


def main():
    parser = argparse.ArgumentParser(
        description='RGBD 참조 이미지로부터 메시 생성',
        formatter_class=argparse.ArgumentDefaultsHelpFormatter
    )
    parser.add_argument(
        '--data_dir', type=str, required=True,
        help='참조 이미지 디렉토리 (images/ + labels/ 서브폴더 포함)'
    )
    parser.add_argument(
        '--output', type=str, default=None,
        help='출력 메시 경로 (기본: data_dir/../mesh.obj)'
    )
    parser.add_argument(
        '--method', type=str, default='tsdf', choices=['tsdf', 'pointcloud', 'bundlesdf'],
        help='메시 생성 방법'
    )
    parser.add_argument(
        '--voxel_size', type=float, default=0.002,
        help='TSDF 복셀 크기 (m)'
    )
    parser.add_argument(
        '--sdf_trunc', type=float, default=0.01,
        help='SDF 절단 거리 (m)'
    )
    parser.add_argument(
        '--depth_trunc', type=float, default=1.5,
        help='최대 깊이 절단 (m)'
    )
    parser.add_argument(
        '--visualize', action='store_true',
        help='결과 시각화 (Open3D 뷰어)'
    )
    parser.add_argument(
        '--save_pointcloud', action='store_true',
        help='중간 포인트 클라우드도 저장'
    )
    parser.add_argument(
        '--horizontal_only', action='store_true',
        help='수평 뷰(8개)만 사용 (top/bottom 뷰 제외, 더 안정적)'
    )
    parser.add_argument(
        '--fx', type=float, default=DEFAULT_INTRINSICS['fx'])
    parser.add_argument(
        '--fy', type=float, default=DEFAULT_INTRINSICS['fy'])
    parser.add_argument(
        '--cx', type=float, default=DEFAULT_INTRINSICS['cx'])
    parser.add_argument(
        '--cy', type=float, default=DEFAULT_INTRINSICS['cy'])

    args = parser.parse_args()

    intrinsics = {
        'fx': args.fx, 'fy': args.fy,
        'cx': args.cx, 'cy': args.cy,
        'width': 640, 'height': 480,
    }

    # 출력 경로 결정
    if args.output is None:
        parent = str(Path(args.data_dir).parent)
        args.output = os.path.join(parent, 'mesh.obj')

    logger.info(f"=== 메시 생성 시작 ===")
    logger.info(f"데이터: {args.data_dir}")
    logger.info(f"방법: {args.method}")
    logger.info(f"출력: {args.output}")

    # 1. 데이터 로드
    logger.info("\n[1/3] 참조 데이터 로드 중...")
    data = load_reference_data(args.data_dir, intrinsics, horizontal_only=args.horizontal_only)
    data['data_dir'] = args.data_dir

    # 2. 메시 생성
    logger.info(f"\n[2/3] 메시 생성 중 ({args.method})...")
    if args.method == 'tsdf':
        mesh = generate_mesh_tsdf(
            data, intrinsics,
            voxel_size=args.voxel_size,
            sdf_trunc=args.sdf_trunc,
            depth_trunc=args.depth_trunc,
        )
    elif args.method == 'pointcloud':
        mesh = generate_mesh_pointcloud(data, intrinsics, voxel_size=args.voxel_size)
    elif args.method == 'bundlesdf':
        mesh = generate_mesh_bundlesdf(data, intrinsics)

    # 3. 저장
    logger.info(f"\n[3/3] 메시 저장 중...")
    export_mesh(mesh, args.output)

    # 포인트 클라우드 저장 (선택)
    if args.save_pointcloud:
        import open3d as o3d
        pcd_path = args.output.replace('.obj', '_pointcloud.ply')

        camera_distance = data['camera_distance']
        per_view_distances = data.get('per_view_distances')
        extrinsics = create_camera_poses_from_angles(
            data['views'], camera_distance, per_view_distances=per_view_distances
        )
        fx, fy = intrinsics['fx'], intrinsics['fy']
        cx, cy = intrinsics['cx'], intrinsics['cy']

        all_pts, all_cols = [], []
        for rgb, depth, mask, extr in zip(data['rgbs'], data['depths'], data['masks'], extrinsics):
            h, w = depth.shape
            valid = (mask > 0) & (depth > 0.01) & (depth < 1.5)
            if valid.sum() < 50:
                continue
            v_c, u_c = np.where(valid)
            z = depth[valid]
            x = (u_c - cx) * z / fx
            y = (v_c - cy) * z / fy
            pts_cam = np.stack([x, y, z], axis=-1)
            cam2w = np.linalg.inv(extr)
            pts_w = (cam2w[:3, :3] @ pts_cam.T).T + cam2w[:3, 3]
            all_pts.append(pts_w)
            all_cols.append(rgb[valid].astype(np.float64) / 255.0)

        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(np.vstack(all_pts))
        pcd.colors = o3d.utility.Vector3dVector(np.vstack(all_cols))
        pcd = pcd.voxel_down_sample(args.voxel_size)
        o3d.io.write_point_cloud(pcd_path, pcd)
        logger.info(f"포인트 클라우드 저장됨: {pcd_path}")

    # 시각화 (선택)
    if args.visualize:
        visualize_result(mesh, data, intrinsics)

    logger.info("\n=== 완료 ===")


if __name__ == '__main__':
    main()
