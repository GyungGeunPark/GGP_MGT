import numpy as np
import open3d as o3d
from pathlib import Path


# ============================================
# 설정 파라미터 (여기서 수정) - mm 단위 기준
# ============================================
CONFIG = {
    # 입출력 1.ply = left, 2.ply = right
    "input_path": "2.ply",
    "output_path": None,  # None이면 자동생성 (input_denoised.ply)
    
    # 다운샘플링 (None이면 스킵)
    "voxel_size": 10,          # 복셀 크기 (mm) 10
    
    # 노이즈 제거 방법: "sor", "ror", "both", None이면 스킵
    "method": "sor",
    
    # Statistical Outlier Removal (SOR) 파라미터
    "sor_nb_neighbors": 20,   # 이웃 포인트 수
    "sor_std_ratio": 2.0,     # 표준편차 배수 (낮을수록 aggressive)
    
    # Radius Outlier Removal (ROR) 파라미터
    "ror_nb_points": 16,      # 반경 내 최소 포인트 수
    "ror_radius": 50,         # 검색 반경 (mm) 50
    
    # 평면 추출 (None이면 스킵)
    "plane_distance_threshold": 10,    # 평면에서 이 거리 이내면 inlier (mm) 10
    "plane_ransac_n": 3,               # RANSAC 샘플 수
    "plane_num_iterations": 1000,      # RANSAC 반복 횟수
    
    # 시각화
    "visualize": True,
}
# ============================================


def load_ply(filepath: str) -> o3d.geometry.PointCloud:
    pcd = o3d.io.read_point_cloud(filepath)
    print(f"[로드] 포인트 수: {len(pcd.points):,}")
    return pcd


def downsample(pcd, voxel_size):
    pcd_down = pcd.voxel_down_sample(voxel_size)
    print(f"[다운샘플] {len(pcd.points):,} → {len(pcd_down.points):,} "
          f"(voxel: {voxel_size})")
    return pcd_down


def remove_statistical_outliers(pcd, nb_neighbors, std_ratio):
    pcd_clean, _ = pcd.remove_statistical_outlier(
        nb_neighbors=nb_neighbors,
        std_ratio=std_ratio
    )
    removed = len(pcd.points) - len(pcd_clean.points)
    print(f"[SOR] 제거: {removed:,} ({removed/len(pcd.points)*100:.1f}%)")
    return pcd_clean


def remove_radius_outliers(pcd, nb_points, radius):
    pcd_clean, _ = pcd.remove_radius_outlier(
        nb_points=nb_points,
        radius=radius
    )
    removed = len(pcd.points) - len(pcd_clean.points)
    print(f"[ROR] 제거: {removed:,} ({removed/len(pcd.points)*100:.1f}%)")
    return pcd_clean


def extract_plane(pcd, distance_threshold, ransac_n, num_iterations):
    plane_model, inliers = pcd.segment_plane(
        distance_threshold=distance_threshold,
        ransac_n=ransac_n,
        num_iterations=num_iterations
    )
    a, b, c, d = plane_model
    print(f"[평면] 방정식: {a:.4f}x + {b:.4f}y + {c:.4f}z + {d:.4f} = 0")
    
    pcd_plane = pcd.select_by_index(inliers)
    print(f"[평면] inlier: {len(inliers):,} / {len(pcd.points):,} "
          f"({len(inliers)/len(pcd.points)*100:.1f}%)")
    return pcd_plane, plane_model


def main():
    cfg = CONFIG
    
    # 설정값 출력
    print(f"=== CONFIG ===")
    print(f"  voxel_size: {cfg.get('voxel_size')}")
    print(f"  method: {cfg.get('method')}")
    print(f"  plane_threshold: {cfg.get('plane_distance_threshold')}")
    print(f"===============")
    
    # 로드
    pcd = load_ply(cfg["input_path"])
    original_count = len(pcd.points)
    
    # 다운샘플링
    if cfg.get("voxel_size"):
        pcd = downsample(pcd, cfg["voxel_size"])
    
    # 노이즈 제거
    if cfg["method"] in ["sor", "both"]:
        pcd = remove_statistical_outliers(
            pcd,
            cfg["sor_nb_neighbors"],
            cfg["sor_std_ratio"]
        )
    
    if cfg["method"] in ["ror", "both"]:
        pcd = remove_radius_outliers(
            pcd,
            cfg["ror_nb_points"],
            cfg["ror_radius"]
        )
    
    # 평면 추출
    if cfg.get("plane_distance_threshold"):
        pcd, plane_model = extract_plane(
            pcd,
            cfg["plane_distance_threshold"],
            cfg["plane_ransac_n"],
            cfg["plane_num_iterations"]
        )
    
    # 결과
    final_count = len(pcd.points)
    print(f"[결과] {original_count:,} → {final_count:,} "
          f"({(1 - final_count/original_count)*100:.1f}% 제거)")
    
    # 저장
    output_path = cfg["output_path"]
    if output_path is None:
        p = Path(cfg["input_path"])
        output_path = str(p.parent / f"{p.stem}_denoised.ply")
    
    o3d.io.write_point_cloud(output_path, pcd)
    print(f"[저장] {output_path}")
    
    # 시각화
    if cfg["visualize"]:
        o3d.visualization.draw_geometries(
            [pcd],
            window_name="Denoised Point Cloud",
            width=1280,
            height=720
        )


if __name__ == "__main__":
    main()