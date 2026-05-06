"""
JSON 경로 파일을 PyVista로 시각화하는 뷰어
- 변환 행렬(T_ms_overall.txt) 적용 기능 포함
- 원본 + 변환된 경로 동시 시각화
- 변환된 JSON을 mm 단위로 저장
"""

import numpy as np
import pyvista as pv
import json
import os
from datetime import datetime


# ============================================================
# 설정
# ============================================================
OUTPUT_UNIT_SCALE = 1000.0  # m -> mm 변환 (mm -> m이면 0.001)
DEFAULT_OUTPUT_DIR = "./iktarget"  # 기본 출력 디렉토리
ENABLE_VISUALIZATION = False  # 시각화 비활성화 (서버 환경 - 웹에서 별도 표시)
# ============================================================


def load_transform_matrix(txt_path: str) -> np.ndarray:
    """4x4 변환 행렬 파일 로드
    
    Args:
        txt_path: 변환 행렬 텍스트 파일 경로
                  (공백으로 구분된 4x4 행렬)
    
    Returns:
        4x4 numpy 배열
    """
    with open(txt_path, 'r') as f:
        lines = f.readlines()
    
    matrix = []
    for line in lines:
        line = line.strip()
        if line:  # 빈 줄 무시
            values = [float(v) for v in line.split()]
            if len(values) == 4:
                matrix.append(values)
    
    matrix = np.array(matrix)
    
    if matrix.shape != (4, 4):
        raise ValueError(f"행렬 크기가 4x4가 아닙니다: {matrix.shape}")
    
    print(f"[INFO] 변환 행렬 로드 완료:")
    print(matrix)
    
    return matrix


def apply_transform_to_path(data: dict, T: np.ndarray, output_scale: float = 1.0) -> dict:
    """경로 데이터에 변환 행렬 적용
    
    Args:
        data: 원본 JSON 데이터 (lines, num_lines, total_points 포함)
        T: 4x4 변환 행렬
        output_scale: 출력 단위 변환 (m -> mm: 1000.0, mm -> m: 0.001)
    
    Returns:
        변환된 JSON 데이터
    
    변환 과정:
        1. 위치 벡터 (x, y, z) -> homogeneous (x, y, z, 1) -> T @ p -> (x', y', z')
        2. 방향 벡터 (nx, ny, nz) -> 회전 행렬(R = T[:3,:3])만 적용 -> R @ n
        3. 위치 좌표에 output_scale 적용 (단위 변환)
    """
    R = T[:3, :3]  # 회전 행렬 (3x3)
    t = T[:3, 3]   # 이동 벡터 (3,)
    
    transformed_lines = []
    
    for line in data['lines']:
        transformed_line = []
        for pt in line:
            # 원본 데이터 분리
            pos = np.array(pt[:3])      # 위치: x, y, z
            dir_vec = np.array(pt[3:6]) # 방향: nx, ny, nz
            
            # 위치 변환: p' = R @ p + t
            new_pos = R @ pos + t
            
            # 단위 변환 적용 (m -> mm)
            new_pos = new_pos * output_scale
            
            # 방향 벡터 변환: n' = R @ n (회전만 적용, 정규화 유지)
            # 방향 벡터는 단위 벡터이므로 스케일 적용 안함
            new_dir = R @ dir_vec
            # 정규화 (회전 행렬이 직교하면 길이 유지되지만 안전하게)
            norm = np.linalg.norm(new_dir)
            if norm > 1e-10:
                new_dir = new_dir / norm
            
            # 나머지 데이터 유지 (있다면)
            extra = pt[6:] if len(pt) > 6 else []
            
            transformed_pt = list(new_pos) + list(new_dir) + list(extra)
            transformed_line.append(transformed_pt)
        
        transformed_lines.append(transformed_line)
    
    # 새 데이터 구성
    transformed_data = {
        'num_lines': data['num_lines'],
        'total_points': data['total_points'],
        'lines': transformed_lines,
        'transform_applied': T.tolist(),  # 적용된 변환 기록
        'output_unit_scale': output_scale  # 단위 변환 배율 기록
    }
    
    return transformed_data


def generate_timestamped_filename(base_name: str = "robot_path_mm") -> str:
    """타임스탬프가 포함된 파일명 생성

    형식: robot_path_mm_YYMMDD_HHMMSS_ffffff.json (마이크로초 포함)
    예: robot_path_mm_260130_123437_123456.json

    Args:
        base_name: 기본 파일명 (확장자 제외)

    Returns:
        타임스탬프가 포함된 파일명 (마이크로초 포함으로 고유성 보장)
    """
    now = datetime.now()
    timestamp = now.strftime("%y%m%d_%H%M%S_%f")
    filename = f"{base_name}_{timestamp}.json"
    print(f"[INFO] 타임스탬프 생성: {now.isoformat()}")
    print(f"[INFO] 출력 파일명: {filename}")
    return filename


def save_transformed_json(data: dict, output_path: str):
    """변환된 데이터를 JSON으로 저장

    Args:
        data: 변환된 경로 데이터
        output_path: 출력 파일 경로
    """
    # 출력 디렉토리 생성
    output_dir = os.path.dirname(output_path)
    if output_dir and not os.path.exists(output_dir):
        os.makedirs(output_dir, exist_ok=True)
        print(f"[INFO] 출력 디렉토리 생성: {output_dir}")

    with open(output_path, 'w') as f:
        json.dump(data, f, indent=2)
    
    unit_info = ""
    if 'output_unit_scale' in data:
        scale = data['output_unit_scale']
        if scale == 1000.0:
            unit_info = " (mm 단위)"
        elif scale == 0.001:
            unit_info = " (m 단위)"
        else:
            unit_info = f" (scale: {scale})"
    
    print(f"[INFO] 변환된 JSON 저장 완료{unit_info}: {output_path}")


def extract_path_data(data: dict):
    """JSON 데이터에서 점과 방향 벡터 추출
    
    Returns:
        points: (N, 3) 위치 배열
        directions: (N, 3) 방향 벡터 배열
        line_indices: [(start, end), ...] 각 라인의 인덱스 범위
    """
    all_points = []
    all_directions = []
    line_indices = []
    
    for line in data['lines']:
        start = len(all_points)
        for pt in line:
            all_points.append(pt[:3])       # x, y, z
            all_directions.append(pt[3:6])  # nx, ny, nz
        end = len(all_points)
        line_indices.append((start, end))
    
    return np.array(all_points), np.array(all_directions), line_indices


def add_path_to_plotter(plotter, points, directions, line_indices,
                        point_color='red', arrow_color='blue',
                        line_colors=None, normal_scale=0.05,
                        prefix='', opacity=1.0):
    """경로를 plotter에 추가
    
    Args:
        plotter: PyVista Plotter 객체
        points: (N, 3) 위치 배열
        directions: (N, 3) 방향 벡터 배열
        line_indices: 각 라인의 시작/끝 인덱스
        point_color: 점 색상
        arrow_color: 화살표 색상
        line_colors: 라인별 색상 리스트
        normal_scale: 화살표 크기
        prefix: 라벨 접두사 (예: 'Original ', 'Transformed ')
        opacity: 투명도 (0~1)
    """
    if line_colors is None:
        line_colors = ['blue', 'green', 'orange', 'purple', 'cyan', 'magenta']
    
    # 점 표시
    plotter.add_mesh(
        pv.PolyData(points),
        color=point_color,
        point_size=8,
        render_points_as_spheres=True,
        opacity=opacity,
        label=f'{prefix}Points'
    )
    
    # 방향 벡터 (화살표)
    arrows = pv.PolyData(points)
    arrows['vectors'] = directions * normal_scale
    plotter.add_mesh(
        arrows.glyph(orient='vectors', scale=False, factor=normal_scale),
        color=arrow_color,
        opacity=opacity,
        label=f'{prefix}Direction'
    )
    
    # 라인별로 경로 연결
    for idx, (start, end) in enumerate(line_indices):
        pts = points[start:end]
        if len(pts) > 1:
            n = len(pts)
            lines = np.column_stack([
                np.full(n-1, 2),
                np.arange(n-1),
                np.arange(1, n)
            ]).flatten()
            
            color = line_colors[idx % len(line_colors)]
            label = f'{prefix}Line {idx}' if idx < 3 else None
            
            plotter.add_mesh(
                pv.PolyData(pts, lines=lines),
                color=color,
                line_width=2,
                opacity=opacity,
                label=label
            )


def view_path_with_transform(json_path: str,
                             transform_path: str = None,
                             output_json_path: str = None,
                             normal_scale: float = 0.05,
                             output_unit_scale: float = 1.0):
    """JSON 경로와 변환된 경로를 함께 시각화
    
    Args:
        json_path: 원본 JSON 파일 경로
        transform_path: 변환 행렬 파일 경로 (None이면 원본만 표시)
        output_json_path: 변환된 JSON 저장 경로 (None이면 자동 생성)
        normal_scale: 법선 화살표 크기
        output_unit_scale: 출력 단위 변환 (m->mm: 1000.0, mm->m: 0.001)
    
    처리 순서:
        1. 원본 JSON 로드
        2. 변환 행렬 로드 (있으면)
        3. 변환 적용 및 새 JSON 저장 (단위 변환 포함)
        4. PyVista로 원본 + 변환 경로 시각화
    """
    # 1. 원본 JSON 로드
    with open(json_path, 'r') as f:
        original_data = json.load(f)
    
    print(f"[INFO] 원본 JSON 로드 완료: {json_path}")
    print(f"[INFO] 라인 수: {original_data['num_lines']}")
    print(f"[INFO] 총 점 수: {original_data['total_points']}")
    
    # 원본 데이터 추출
    orig_points, orig_dirs, orig_indices = extract_path_data(original_data)

    # PyVista 시각화 설정 (비활성화 시 건너뜀)
    plotter = None
    if ENABLE_VISUALIZATION:
        plotter = pv.Plotter()
        plotter.set_background('white')

        # 원본 경로 추가 (반투명)
        add_path_to_plotter(
            plotter, orig_points, orig_dirs, orig_indices,
            point_color='red',
            arrow_color='darkred',
            line_colors=['darkblue', 'darkgreen', 'darkorange', 'purple', 'teal', 'brown'],
            normal_scale=normal_scale,
            prefix='Original ',
            opacity=0.5
        )
    else:
        print("[INFO] 시각화 비활성화됨 (서버 환경)")

    # 2. 변환 행렬이 있으면 처리
    if transform_path and os.path.exists(transform_path):
        print(f"\n[INFO] 변환 행렬 파일 로드: {transform_path}")
        T = load_transform_matrix(transform_path)
        
        # 단위 변환 정보 출력
        if output_unit_scale == 1000.0:
            print(f"[INFO] 출력 단위: mm (m × 1000)")
        elif output_unit_scale == 0.001:
            print(f"[INFO] 출력 단위: m (mm × 0.001)")
        elif output_unit_scale != 1.0:
            print(f"[INFO] 출력 스케일: {output_unit_scale}")
        
        # 3. 변환 적용 (단위 변환 포함)
        transformed_data = apply_transform_to_path(original_data, T, output_unit_scale)
        
        # 변환된 JSON 저장 (타임스탬프 파일명 + iktarget 폴더)
        if output_json_path is None:
            output_filename = generate_timestamped_filename("robot_path_mm")
            output_json_path = os.path.join(DEFAULT_OUTPUT_DIR, output_filename)
        
        save_transformed_json(transformed_data, output_json_path)
        
        # 변환된 데이터 추출 (시각화용 - 원래 스케일로 역변환)
        # 시각화는 원본과 같은 단위로 표시하기 위해 스케일 보정
        trans_points, trans_dirs, trans_indices = extract_path_data(transformed_data)

        # 시각화용으로 스케일 보정 (저장은 mm, 표시는 m)
        if output_unit_scale != 1.0:
            trans_points = trans_points / output_unit_scale

        # 변환된 경로 추가 (불투명) - 시각화 활성화 시에만
        if ENABLE_VISUALIZATION and plotter:
            add_path_to_plotter(
                plotter, trans_points, trans_dirs, trans_indices,
                point_color='lime',
                arrow_color='green',
                line_colors=['blue', 'green', 'orange', 'purple', 'cyan', 'magenta'],
                normal_scale=normal_scale,
                prefix='Transformed ',
                opacity=1.0
            )
            print(f"\n[INFO] 시각화: 원본(반투명) + 변환(불투명)")
    else:
        if transform_path:
            print(f"[WARN] 변환 행렬 파일을 찾을 수 없습니다: {transform_path}")
        if ENABLE_VISUALIZATION:
            print(f"[INFO] 시각화: 원본만 표시")

    # 4. 시각화 실행 (비활성화 시 건너뜀)
    if ENABLE_VISUALIZATION and plotter:
        plotter.add_axes()
        plotter.add_legend()
        plotter.camera_position = 'iso'
        plotter.show()
    else:
        print("[INFO] 변환 데이터 저장 완료 (시각화 건너뜀)")


# ============================================================
# 메인 실행
# ============================================================
if __name__ == '__main__':
    # === 설정 ===
    JSON_FILE = "robot_path.json"           # 원본 JSON 경로
    TRANSFORM_FILE = "./global/out_icp_only/T_ms_overall.txt"     # 변환 행렬 파일 (없으면 None)
    OUTPUT_JSON = None                       # 출력 경로 (None이면 자동: *_mm.json)
    NORMAL_SCALE = 0.05                      # 화살표 크기
    
    # === 실행 ===
    view_path_with_transform(
        json_path=JSON_FILE,
        transform_path=TRANSFORM_FILE,
        output_json_path=OUTPUT_JSON,
        normal_scale=NORMAL_SCALE,
        output_unit_scale=OUTPUT_UNIT_SCALE  # 상단 설정값 사용
    )