#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
JSON 경로 파일 변환 (Headless 버전)
- PyVista 시각화 없이 실행
- 변환 행렬(T_ms_overall.txt) 적용
- 변환된 JSON을 mm 단위로 저장
- 파일명 형식: robot_path_mm_YYMMDD_HHMMSS.json
- 출력 폴더: ./iktarget/
"""

import numpy as np
import json
import os
import sys
import argparse
from datetime import datetime


# ============================================================
# 설정
# ============================================================
OUTPUT_UNIT_SCALE = 1000.0  # m -> mm 변환 (mm -> m이면 0.001)
# DEFAULT_OUTPUT_DIR = "./iktarget"
# DEFAULT_JSON_FILE = "robot_path.json"
# DEFAULT_TRANSFORM_FILE = "/root/sl_per_ws/out_icp_only/T_ms_overall.txt"
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


def generate_timestamped_filename(base_name: str = "robot_path_mm") -> str:
    """타임스탬프가 포함된 파일명 생성

    형식: robot_path_mm_YYMMDD_HHMMSS_ffffff.json (마이크로초 포함)
    예: robot_path_mm_260130_123437_123456.json

    Args:
        base_name: 기본 파일명 (확장자 제외)

    Returns:
        타임스탬프가 포함된 파일명 (마이크로초 포함으로 고유성 보장)
    """
    # 항상 현재 시스템 시간을 사용 (함수 호출 시점)
    now = datetime.now()
    # 마이크로초까지 포함하여 연속 실행 시에도 고유한 파일명 생성
    timestamp = now.strftime("%y%m%d_%H%M%S_%f")  # %f = 마이크로초 6자리
    filename = f"{base_name}_{timestamp}.json"
    print(f"[INFO] 타임스탬프 생성: {now.isoformat()}")
    print(f"[INFO] 출력 파일명: {filename}")
    return filename


def generate_target_path(json_path: str,
                         transform_path: str,
                         output_dir: str = None,
                         output_unit_scale: float = 1000.0) -> dict:
    """타겟 경로 생성 (시각화 없이)

    Args:
        json_path: 원본 JSON 파일 경로
        transform_path: 변환 행렬 파일 경로
        output_dir: 출력 디렉토리 (None이면 ./iktarget)
        output_unit_scale: 출력 단위 변환 (m->mm: 1000.0)

    Returns:
        결과 정보 딕셔너리
    """
    # 기본 출력 디렉토리 설정
    if output_dir is None:
        output_dir = DEFAULT_OUTPUT_DIR

    # 출력 디렉토리 생성
    os.makedirs(output_dir, exist_ok=True)

    # 1. 원본 JSON 로드
    print(f"\n{'='*60}")
    print(f"[INFO] 원본 JSON 로드: {json_path}")

    with open(json_path, 'r') as f:
        original_data = json.load(f)

    print(f"[INFO] 라인 수: {original_data['num_lines']}")
    print(f"[INFO] 총 점 수: {original_data['total_points']}")

    # 2. 변환 행렬 로드
    if not os.path.exists(transform_path):
        raise FileNotFoundError(f"변환 행렬 파일을 찾을 수 없습니다: {transform_path}")

    print(f"\n[INFO] 변환 행렬 파일 로드: {transform_path}")
    T = load_transform_matrix(transform_path)

    # 단위 변환 정보 출력
    if output_unit_scale == 1000.0:
        print(f"[INFO] 출력 단위: mm (m × 1000)")
    elif output_unit_scale == 0.001:
        print(f"[INFO] 출력 단위: m (mm × 0.001)")
    elif output_unit_scale != 1.0:
        print(f"[INFO] 출력 스케일: {output_unit_scale}")

    # 3. 변환 적용
    transformed_data = apply_transform_to_path(original_data, T, output_unit_scale)

    # 4. 타임스탬프 파일명 생성
    output_filename = generate_timestamped_filename("robot_path_mm")
    output_json_path = os.path.join(output_dir, output_filename)

    # 5. 저장
    save_transformed_json(transformed_data, output_json_path)

    print(f"\n{'='*60}")
    print(f"[SUCCESS] 타겟 경로 생성 완료!")
    print(f"[OUTPUT]  {output_json_path}")
    print(f"{'='*60}")

    return {
        "success": True,
        "output_path": output_json_path,
        "output_filename": output_filename,
        "num_lines": transformed_data['num_lines'],
        "total_points": transformed_data['total_points']
    }


def main():
    """메인 함수 - CLI 인터페이스"""
    parser = argparse.ArgumentParser(
        description='JSON 경로 파일 변환 (Headless 버전)',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
예제:
  # 기본 설정으로 실행
  python paview1_headless.py

  # 파일 지정하여 실행
  python paview1_headless.py --json robot_path.json --transform /path/to/T_ms_overall.txt

  # 출력 디렉토리 지정
  python paview1_headless.py --output-dir ./iktarget

출력 파일 형식:
  robot_path_mm_YYMMDD_HHMMSS.json
  예: robot_path_mm_260130_123437.json
        """
    )

    parser.add_argument(
        '--json', '-j',
        default=DEFAULT_JSON_FILE,
        help=f'원본 JSON 파일 경로 (기본값: {DEFAULT_JSON_FILE})'
    )

    parser.add_argument(
        '--transform', '-t',
        default=DEFAULT_TRANSFORM_FILE,
        help=f'변환 행렬 파일 경로 (기본값: {DEFAULT_TRANSFORM_FILE})'
    )

    parser.add_argument(
        '--output-dir', '-o',
        default=DEFAULT_OUTPUT_DIR,
        help=f'출력 디렉토리 (기본값: {DEFAULT_OUTPUT_DIR})'
    )

    parser.add_argument(
        '--scale', '-s',
        type=float,
        default=OUTPUT_UNIT_SCALE,
        help=f'출력 단위 스케일 (기본값: {OUTPUT_UNIT_SCALE}, m->mm)'
    )

    args = parser.parse_args()

    # 파일 존재 확인
    if not os.path.exists(args.json):
        print(f"[ERROR] 원본 JSON 파일을 찾을 수 없습니다: {args.json}")
        sys.exit(1)

    if not os.path.exists(args.transform):
        print(f"[ERROR] 변환 행렬 파일을 찾을 수 없습니다: {args.transform}")
        sys.exit(1)

    try:
        result = generate_target_path(
            json_path=args.json,
            transform_path=args.transform,
            output_dir=args.output_dir,
            output_unit_scale=args.scale
        )

        print(f"\n[RESULT] 생성된 파일: {result['output_path']}")
        sys.exit(0)

    except Exception as e:
        print(f"\n[ERROR] 타겟 생성 실패: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)


if __name__ == '__main__':
    main()
