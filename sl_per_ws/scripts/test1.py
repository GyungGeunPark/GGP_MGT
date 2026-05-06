#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
(모델 선행 변환 적용) → ICP(C2M) → 행렬 저장  [직접 STL 경로 지정 버전]

저장 파일:
  - T_icp.txt               : ICP 변환 (Scan→Model)
  - T_total.txt             : Scan→Model  (OBB 정렬 제외이므로 T_total == T_icp)
  - (옵션) T_ms_cpp.txt     : C++ 선행 변환 (Model→Scan)  [PRE_XFORM_PATH가 있을 때]
  - (옵션) T_ms_icp.txt     : inv(T_total) = ICP 결과를 Model→Scan으로 환산
  - (옵션) T_ms_overall.txt : 최종 Model→Scan = inv(T_total) @ T_ms_cpp
  - global_icp_only.bin     : CloudCompare에서 열어보기용 (.bin)

주의:
  - 행렬 방향은 열벡터/우측곱 관례를 가정합니다. 다른 툴과 혼용 시 샘플 포인트로 검증 권장
  - PCD/STL 단위(m/mm) 불일치 시 오차가 커질 수 있습니다.

실행 예:
  # 직접 STL 경로 지정 (새로운 방식)
  python test1.py --model-stl "/mnt/Share/perception/data/stl/cad.stl"
  
  # 기존 블록명 방식 (호환성 유지)
  python test1.py --block "SN2695_T140S_processed"
"""

import os
import re
import argparse
import numpy as np
import cloudComPy as cc

# ─────────────────────────────────────────────────────────────
# 인자 파싱 (수정됨: --model-stl 추가)
# ─────────────────────────────────────────────────────────────
def parse_args():
    p = argparse.ArgumentParser(description="Global ICP (CloudComPy) runner")
    
    # 모델 지정 방식 (둘 중 하나 필수)
    model_group = p.add_mutually_exclusive_group(required=True)
    model_group.add_argument("--model-stl",
                            help="모델 STL 파일의 직접 경로 (예: /mnt/Share/perception/data/stl/cad.stl)")
    model_group.add_argument("--block",
                            help="모델 블럭명 (예: SN2695_T140S_processed). "
                                 "모델 STL은 /mnt/Share/cad_files/modi_stl/<block>/<block>.stl 로 구성됩니다.")
    
    p.add_argument("--scan-pcd", default="/mnt/Share/perception/data/global/global_cropped_point_cloud.pcd",
                   help="스캔 PCD 경로 (기본: /mnt/Share/perception/data/global/global_cropped_point_cloud.pcd)")
    p.add_argument("--pre-xform", default="/home/mgt/cloudcompy/transformation_matrix_inverse.txt",
                   help="(선택) C++ 선행 변환 (Model→Scan) 4x4 행렬 파일 경로. 주석 허용. 빈 문자열로 끄기")
    p.add_argument("--out-dir", default="/mnt/Share/perception/output/global/out_icp_only",
                   help="출력 폴더")
    p.add_argument("--visualize", action="store_true",
                   help="Open3D 시각화 On")
    return p.parse_args()

args = parse_args()

# ===== 경로 설정 (수정됨: 두 가지 방식 지원) =====
if args.model_stl:
    # 직접 STL 경로가 지정된 경우
    MODEL_STL = args.model_stl
    print(f"[INFO] Using direct STL path: {MODEL_STL}")
else:
    # 기존 블록명 방식
    MODEL_STL = os.path.join("/mnt/Share/cad_files/modi_stl", args.block, f"{args.block}.stl")
    print(f"[INFO] Using block-based STL path: {MODEL_STL}")

SCAN_PCD  = args.scan_pcd
OUT_DIR   = args.out_dir
os.makedirs(OUT_DIR, exist_ok=True)

# (선택) C++ 선행 변환( Model->Scan ) 파일 경로 (주석 포함 텍스트 가능)
PRE_XFORM_PATH = args.pre_xform if args.pre_xform.strip() else None

# ===== 시각화 옵션 =====
VISUALIZE  = bool(args.visualize)
VOXEL_SIZE = 0.02  # Open3D 보기 전용 다운샘플

# ─────────────────────────────────────────────────────────────
# 유틸 함수
# ─────────────────────────────────────────────────────────────
def load_matrix_txt_with_comments(path: str) -> np.ndarray:
    """
    주석(# …)과 쉼표/세미콜론/괄호 등이 섞인 4x4 행렬 텍스트를 안전하게 로드하고
    마지막 행을 [0 0 0 1]로 보정, 회전부를 정규화(SO(3) 투영)합니다.
    """
    if not os.path.isfile(path):
        raise FileNotFoundError(f"Matrix file not found: {path}")

    import re
    with open(path, "r", encoding="utf-8", errors="ignore") as f:
        txt = f.read()

    txt = re.sub(r"#[^\n]*", " ", txt)                  # 주석 제거
    txt = re.sub(r"[;,()\[\]]", " ", txt)               # 구분기호 제거
    nums = [float(x) for x in txt.split() if x.strip()] # 숫자만 추출

    if len(nums) != 16:
        raise ValueError(f"Expected 16 numbers, got {len(nums)} in {path}")

    M = np.array(nums, dtype=float).reshape(4, 4)
    M[3, :] = np.array([0, 0, 0, 1], dtype=float)       # 마지막 행 보정

    if not np.isfinite(M).all():
        raise ValueError(f"Matrix has NaN/Inf: {path}")

    # 회전부 정규화(SO(3) 투영) + 스케일 감지
    R = M[:3, :3].copy()
    s = np.mean([np.linalg.norm(R[:, i]) for i in range(3)])
    if s <= 0 or not np.isfinite(s):
        raise ValueError("Invalid scale detected in rotation part")

    Rn = R / s
    U, _, VT = np.linalg.svd(Rn)
    R_ortho = U @ VT
    if np.linalg.det(R_ortho) < 0:
        U[:, -1] *= -1
        R_ortho = U @ VT

    M[:3, :3] = R_ortho

    detR = np.linalg.det(M[:3, :3])
    if abs(s - 1.0) > 1e-6:
        print(f"[WARN] Detected uniform scale ≈ {s:.6f} in input matrix; rotation re-orthogonalized.")
    if abs(detR - 1.0) > 1e-3:
        print(f"[WARN] det(R) = {detR:.6f} (should be ~1.0)")

    print("[DEBUG] Loaded pre-transform matrix (Model->Scan):")
    np.set_printoptions(suppress=True, linewidth=120, precision=9)
    print(M)
    return M

def ccgl_to_np4x4(glmat) -> np.ndarray:
    """ccGLMatrix → numpy 4x4"""
    if hasattr(glmat, "toString"):
        vals = [float(x) for x in glmat.toString().split()]
        if len(vals) == 16:
            return np.array(vals, dtype=float).reshape(4, 4)
    raise RuntimeError("ccGLMatrix → numpy 변환 실패")

def save_mesh_with_fallback(m_o3d, out_basename: str) -> str:
    """
    Open3D TriangleMesh를 STL로 저장 시도하고, 실패하면 OBJ → PLY 순으로 폴백.
    반환: 저장된 파일 경로 (STL/OBJ/PLY)
    """
    import open3d as o3d
    m_o3d.remove_degenerate_triangles()
    m_o3d.remove_duplicated_triangles()
    m_o3d.remove_duplicated_vertices()
    m_o3d.remove_unreferenced_vertices()
    m_o3d.remove_non_manifold_edges()

    V = np.asarray(m_o3d.vertices)
    T = np.asarray(m_o3d.triangles)
    if V.size == 0 or T.size == 0 or (not np.isfinite(V).all()):
        raise RuntimeError("Open3D: 변환/클린업 후 메시가 비정상(빈/NaN)입니다.")

    m_o3d.compute_triangle_normals()
    m_o3d.compute_vertex_normals()

    stl_path = os.path.join(OUT_DIR, out_basename + ".stl")
    obj_path = os.path.join(OUT_DIR, out_basename + ".obj")
    ply_path = os.path.join(OUT_DIR, out_basename + ".ply")

    ok_stl = o3d.io.write_triangle_mesh(stl_path, m_o3d, write_ascii=False)
    if ok_stl:
        return stl_path

    ok_obj = o3d.io.write_triangle_mesh(obj_path, m_o3d)
    if ok_obj:
        print(f"[WARN] STL 저장 실패 → OBJ로 폴백: {obj_path}")
        return obj_path

    ok_ply = o3d.io.write_triangle_mesh(ply_path, m_o3d)
    if ok_ply:
        print(f"[WARN] STL/OBJ 실패 → PLY로 폴백: {ply_path}")
        return ply_path

    raise RuntimeError(f"Open3D: 변환 메시 저장 실패 → {stl_path}/{obj_path}/{ply_path}")

# ─────────────────────────────────────────────────────────────
# 데이터 로드
# ─────────────────────────────────────────────────────────────
if not os.path.isfile(SCAN_PCD):
    raise RuntimeError(f"[로드 실패] {SCAN_PCD}")

scan_cloud = cc.loadPointCloud(SCAN_PCD)
if scan_cloud is None:
    raise RuntimeError(f"[로드 실패] {SCAN_PCD}")
scan_cloud.setName("scan_cloud")

if not os.path.isfile(MODEL_STL):
    raise RuntimeError(f"[로드 실패] 모델 STL을 찾지 못했습니다: {MODEL_STL}")

# (중요) C++ 선행 변환을 모델 STL에 먼저 적용 → 변환 메시 저장(폴백 포함) → CloudComPy로 로드
MODEL_MESH_PATH_FOR_CC = MODEL_STL
T_ms_cpp = None

if PRE_XFORM_PATH and os.path.isfile(PRE_XFORM_PATH):
    # 1) C++ 행렬 로드
    T_ms_cpp = load_matrix_txt_with_comments(PRE_XFORM_PATH)  # Model->Scan
    np.savetxt(os.path.join(OUT_DIR, "T_ms_cpp.txt"), T_ms_cpp, fmt="%.12f")

    # 2) Open3D로 모델 로드 및 선행 변환 적용
    import open3d as o3d
    m_o3d = o3d.io.read_triangle_mesh(MODEL_STL)
    if m_o3d.is_empty():
        raise RuntimeError("Open3D: MODEL_STL이 비어 있습니다.")

    m_o3d.transform(T_ms_cpp)  # 선행 변환 적용

    # 3) 저장(폴백) & 경로 선택
    MODEL_MESH_PATH_FOR_CC = save_mesh_with_fallback(m_o3d, "model_after_cpp_xform")
    print(f"[INFO] Applied C++ pre-transform → Saved mesh: {MODEL_MESH_PATH_FOR_CC}")
else:
    print("[INFO] No PRE_XFORM_PATH or file not found. Skipping model pre-transform.")

# 4) CloudComPy로 (변환된) 모델 로드
model_mesh = cc.loadMesh(MODEL_MESH_PATH_FOR_CC)
if model_mesh is None:
    raise RuntimeError(f"[로드 실패] {MODEL_MESH_PATH_FOR_CC}")
model_mesh.setName("model_mesh_after_cppXform" if MODEL_MESH_PATH_FOR_CC != MODEL_STL else "model_mesh")

# ─────────────────────────────────────────────────────────────
# ICP(C2M) — OBB 정렬 제외: 바로 실행
# ─────────────────────────────────────────────────────────────
res = cc.ICP(
    data=scan_cloud,         # 원본 스캔
    model=model_mesh,        # (선행 변환 적용된) 모델
    finalOverlapRatio=0.9,
    method=cc.CONVERGENCE_TYPE.MAX_ERROR_CONVERGENCE,
    randomSamplingLimit=100_000,
    minRMSDecrease=1e-7,
    removeFarthestPoints=False,             # C2M 세그폴트 회피
    adjustScale=False,
    useC2MSignedDistances=True,
    robustC2MSignedDistances=True,
    transformationFilters=cc.TRANSFORMATION_FILTERS.SKIP_NONE,
    normalMatching=cc.normalMatching.NO_NORMAL,
    maxThreadCount=0
)

# 변환 행렬 저장
T_icp_np = ccgl_to_np4x4(res.transMat)
np.savetxt(os.path.join(OUT_DIR, "T_icp.txt"), T_icp_np, fmt="%.12f")

# 총 변환(원본 스캔 → 현재 모델 좌표계): OBB 정렬 제외 ⇒ T_total = T_icp
T_total = T_icp_np.copy()
np.savetxt(os.path.join(OUT_DIR, "T_total.txt"), T_total, fmt="%.12f")

# 최종 포인트클라우드(원본 → ICP)
scan_final = scan_cloud.cloneThis()
scan_final.applyRigidTransformation(res.transMat)
scan_final.setName("scan_cloud_after_icp")

# CC .bin 저장(CloudCompare에서 확인용)
cc.SaveEntities([model_mesh, scan_cloud, scan_final],
                os.path.join(OUT_DIR, "global_icp_only.bin"))

print("[INFO] 저장 완료:")
print("  ", os.path.join(OUT_DIR, "T_icp.txt"))
print("  ", os.path.join(OUT_DIR, "T_total.txt"))
print("  ", os.path.join(OUT_DIR, "global_icp_only.bin"))

# (옵션) 최종 Model->Scan 합성행렬 저장
if PRE_XFORM_PATH and os.path.isfile(PRE_XFORM_PATH):
    T_ms_icp     = np.linalg.inv(T_total)       # ICP 결과를 Model→Scan으로 환산
    T_ms_overall = T_ms_icp @ T_ms_cpp          # 최종 Model→Scan
    np.savetxt(os.path.join(OUT_DIR, "T_ms_icp.txt"),     T_ms_icp,     fmt="%.12f")
    np.savetxt(os.path.join(OUT_DIR, "T_ms_overall.txt"), T_ms_overall, fmt="%.12f")
    print("[INFO] Saved overall Model->Scan matrix: T_ms_overall.txt")

# ─────────────────────────────────────────────────────────────
# Open3D 시각화 (선택)
# ─────────────────────────────────────────────────────────────
if VISUALIZE:
    import open3d as o3d

    def cc_to_o3d(ccloud):
        pts = ccloud.toNpArrayCopy().astype(np.float64)
        return o3d.geometry.PointCloud(o3d.utility.Vector3dVector(pts))

    def maybe_vdown(pcd, voxel):
        return pcd.voxel_down_sample(voxel) if voxel and voxel > 0 else pcd

    p_before = maybe_vdown(cc_to_o3d(scan_cloud), 0.02); p_before.paint_uniform_color([0.9, 0.2, 0.2])
    p_final  = maybe_vdown(cc_to_o3d(scan_final), 0.02); p_final.paint_uniform_color([0.2, 0.8, 0.2])

    mesh_vis_path = MODEL_MESH_PATH_FOR_CC
    m = o3d.io.read_triangle_mesh(mesh_vis_path)
    if not m.is_empty():
        if not m.has_vertex_normals():
            m.compute_vertex_normals()
        m.paint_uniform_color([0.7, 0.7, 0.7])

    o3d.visualization.draw_geometries([p_before] + ([m] if not m.is_empty() else []),
                                      window_name="Before (scan:red + mesh:gray)", width=1280, height=800)
    o3d.visualization.draw_geometries([p_final]  + ([m] if not m.is_empty() else []),
                                      window_name="After ICP (scan:green + mesh:gray)", width=1280, height=800)

print("[DONE] (모델 선행 변환) → ICP → 합성행렬 저장 완료  [OBB 정렬 없음]")