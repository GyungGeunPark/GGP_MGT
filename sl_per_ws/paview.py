"""
JSON 경로 파일을 PyVista로 시각화하는 뷰어 v2.0
- 형식: [robot_position(3) + surface_point(3)]
- 로봇 위치 ↔ 표면 접촉점 연결선 시각화
- 변환 행렬(T_ms_overall.txt) 적용 기능 포함
- 변환된 JSON을 mm 단위로 저장
"""

import numpy as np
import pyvista as pv
import json
import os
import argparse
from datetime import datetime


# ============================================================
# 설정
# ============================================================
OUTPUT_UNIT_SCALE = 1000.0  # m -> mm 변환 (mm -> m이면 0.001)
DEFAULT_OUTPUT_DIR = "./iktarget"  # 기본 출력 디렉토리
ENABLE_VISUALIZATION = False  # 시각화 활성화
# ============================================================


def load_transform_matrix(txt_path: str) -> np.ndarray:
    """4x4 변환 행렬 파일 로드"""
    with open(txt_path, 'r') as f:
        lines = f.readlines()
    
    matrix = []
    for line in lines:
        line = line.strip()
        if line:
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
    
    입력 형식: [robot_x, robot_y, robot_z, surface_x, surface_y, surface_z]
    
    변환 과정:
        1. 로봇 위치: p' = R @ p + t, 그 후 scale 적용
        2. 표면 접촉점: p' = R @ p + t, 그 후 scale 적용
        (둘 다 위치 벡터이므로 동일한 변환 적용)
    """
    R = T[:3, :3]  # 회전 행렬 (3x3)
    t = T[:3, 3]   # 이동 벡터 (3,)
    
    transformed_lines = []
    
    for line in data['lines']:
        transformed_line = []
        for pt in line:
            # 원본 데이터 분리
            robot_pos = np.array(pt[:3])     # 로봇 위치
            surface_pt = np.array(pt[3:6])   # 표면 접촉점
            
            # 로봇 위치 변환: p' = R @ p + t
            new_robot_pos = (R @ robot_pos + t) * output_scale
            
            # 표면 접촉점 변환: p' = R @ p + t
            new_surface_pt = (R @ surface_pt + t) * output_scale
            
            # 나머지 데이터 유지 (있다면)
            extra = pt[6:] if len(pt) > 6 else []
            
            transformed_pt = list(new_robot_pos) + list(new_surface_pt) + list(extra)
            transformed_line.append(transformed_pt)
        
        transformed_lines.append(transformed_line)
    
    transformed_data = {
        'description': data.get('description', '각 점: [robot_x, robot_y, robot_z, surface_x, surface_y, surface_z]'),
        'num_lines': data['num_lines'],
        'total_points': data['total_points'],
        'lines': transformed_lines,
        'transform_applied': T.tolist(),
        'output_unit_scale': output_scale
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
    """JSON 데이터에서 로봇 위치와 표면 접촉점 추출
    
    Returns:
        robot_positions: (N, 3) 로봇 위치 배열
        surface_points: (N, 3) 표면 접촉점 배열
        line_indices: [(start, end), ...] 각 라인의 인덱스 범위
    """
    all_robot_positions = []
    all_surface_points = []
    line_indices = []
    
    for line in data['lines']:
        start = len(all_robot_positions)
        for pt in line:
            all_robot_positions.append(pt[:3])   # robot_x, robot_y, robot_z
            all_surface_points.append(pt[3:6])   # surface_x, surface_y, surface_z
        end = len(all_robot_positions)
        line_indices.append((start, end))
    
    return np.array(all_robot_positions), np.array(all_surface_points), line_indices


def add_path_to_plotter(plotter, robot_positions, surface_points, line_indices,
                        robot_color='yellow', surface_color='orange',
                        connection_color='darkgreen', line_colors=None,
                        prefix='', opacity=1.0):
    """경로를 plotter에 추가
    
    시각화 요소:
        - 로봇 위치: 큰 점 (robot_color)
        - 표면 접촉점: 작은 점 (surface_color)
        - 로봇↔표면 연결선 (connection_color)
        - 경로 연결선 (line_colors)
    """
    if line_colors is None:
        line_colors = ['blue', 'green', 'orange', 'purple', 'cyan', 'magenta']
    
    # 로봇 위치 표시 (큰 점)
    plotter.add_mesh(
        pv.PolyData(robot_positions),
        color=robot_color,
        point_size=12,
        render_points_as_spheres=True,
        opacity=opacity,
        label=f'{prefix}Robot Position'
    )
    
    # 표면 접촉점 표시 (작은 점)
    plotter.add_mesh(
        pv.PolyData(surface_points),
        color=surface_color,
        point_size=8,
        render_points_as_spheres=True,
        opacity=opacity,
        label=f'{prefix}Surface Point'
    )
    
    # 로봇 위치 ↔ 표면 접촉점 연결선
    conn_points = []
    conn_cells = []
    for i in range(len(robot_positions)):
        idx = len(conn_points)
        conn_points.append(robot_positions[i])
        conn_points.append(surface_points[i])
        conn_cells.extend([2, idx, idx + 1])
    
    conn_points = np.array(conn_points)
    conn_cells = np.array(conn_cells)
    conn_mesh = pv.PolyData(conn_points, lines=conn_cells)
    plotter.add_mesh(
        conn_mesh,
        color=connection_color,
        line_width=2,
        opacity=opacity,
        label=f'{prefix}Robot→Surface'
    )
    
    # 라인별로 경로 연결 (표면점 기준)
    for idx, (start, end) in enumerate(line_indices):
        pts = surface_points[start:end]
        if len(pts) > 1:
            n = len(pts)
            lines = np.column_stack([
                np.full(n - 1, 2),
                np.arange(n - 1),
                np.arange(1, n)
            ]).flatten()
            
            color = line_colors[idx % len(line_colors)]
            label = f'{prefix}Path {idx}' if idx < 3 else None
            
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
                             output_unit_scale: float = 1.0):
    """JSON 경로와 변환된 경로를 함께 시각화
    
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
    print(f"[INFO] 형식: {original_data.get('description', 'N/A')}")
    print(f"[INFO] 라인 수: {original_data['num_lines']}")
    print(f"[INFO] 총 점 수: {original_data['total_points']}")
    
    # 원본 데이터 추출
    orig_robot_pos, orig_surface_pts, orig_indices = extract_path_data(original_data)

    # PyVista 시각화 설정 (비활성화 시 건너뜀)
    plotter = None
    if ENABLE_VISUALIZATION:
        plotter = pv.Plotter()
        plotter.set_background('white')

        # 원본 경로 추가 (반투명)
        add_path_to_plotter(
            plotter, orig_robot_pos, orig_surface_pts, orig_indices,
            robot_color='darkred',
            surface_color='red',
            connection_color='brown',
            line_colors=['darkblue', 'darkgreen', 'darkorange', 'purple', 'teal', 'brown'],
            prefix='Original ',
            opacity=0.5
        )
    else:
        print("[INFO] 시각화 비활성화됨 (서버 환경)")
    
    # 2. 변환 행렬이 있으면 처리
    if transform_path and os.path.exists(transform_path):
        print(f"\n[INFO] 변환 행렬 파일 로드: {transform_path}")
        T = load_transform_matrix(transform_path)
        
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
        
        # 변환된 데이터 추출 (시각화용)
        trans_robot_pos, trans_surface_pts, trans_indices = extract_path_data(transformed_data)
        
        # 변환된 데이터 추출 및 시각화 (시각화 활성화 시에만)
        if ENABLE_VISUALIZATION and plotter:
            # 시각화용으로 스케일 보정 (저장은 mm, 표시는 m 단위로 맞춤)
            if output_unit_scale != 1.0:
                trans_robot_pos = trans_robot_pos / output_unit_scale
                trans_surface_pts = trans_surface_pts / output_unit_scale

            # 변환된 경로 추가 (불투명)
            add_path_to_plotter(
                plotter, trans_robot_pos, trans_surface_pts, trans_indices,
                robot_color='yellow',
                surface_color='lime',
                connection_color='green',
                line_colors=['blue', 'green', 'orange', 'purple', 'cyan', 'magenta'],
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
    # 명령줄 인자 파싱
    parser = argparse.ArgumentParser(description='JSON 경로 파일을 변환하고 시각화')
    parser.add_argument('--json', '-j', default='robot_path.json',
                        help='원본 JSON 파일 경로 (기본값: robot_path.json)')
    parser.add_argument('--transform', '-t', default=None,
                        help='변환 행렬 파일 경로 (T_ms_overall.txt)')
    parser.add_argument('--output', '-o', default=None,
                        help='출력 JSON 파일 경로 (기본값: 자동 생성)')
    parser.add_argument('--scale', '-s', type=float, default=OUTPUT_UNIT_SCALE,
                        help=f'출력 단위 스케일 (기본값: {OUTPUT_UNIT_SCALE})')

    args = parser.parse_args()

    # 기본값 설정 (transform 인자가 없으면 기본 경로 검색)
    transform_file = args.transform
    if transform_file is None:
        # 기본 경로 검색
        default_paths = [
            "./global/out_icp_only/T_ms_overall.txt",
            "./out_icp_only/T_ms_overall.txt",
            "./T_ms_overall.txt",
        ]
        for path in default_paths:
            if os.path.exists(path):
                transform_file = path
                print(f"[INFO] 변환 행렬 파일 자동 검색: {path}")
                break

    # === 실행 ===
    view_path_with_transform(
        json_path=args.json,
        transform_path=transform_file,
        output_json_path=args.output,
        output_unit_scale=args.scale
    )