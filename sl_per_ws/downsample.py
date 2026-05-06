import open3d as o3d
from pathlib import Path

# ============================================
# 설정
# ============================================
INPUT_PATH = "cropped_world.pcd"
OUTPUT_PATH = None      # None이면 자동생성 (input_downsampled.pcd)
VOXEL_SIZE = 0.001     # 다운샘플링 복셀 크기 (m), None이면 스킵
# ============================================

pcd = o3d.io.read_point_cloud(INPUT_PATH)
print(f"[로드] 포인트 수: {len(pcd.points):,}")

if VOXEL_SIZE:
    pcd = pcd.voxel_down_sample(VOXEL_SIZE)
    print(f"[다운샘플] 포인트 수: {len(pcd.points):,} (voxel: {VOXEL_SIZE}m)")

if OUTPUT_PATH is None:
    p = Path(INPUT_PATH)
    OUTPUT_PATH = str(p.parent / f"{p.stem}_downsampled.pcd")

o3d.io.write_point_cloud(OUTPUT_PATH, pcd)
print(f"[저장] {OUTPUT_PATH}")