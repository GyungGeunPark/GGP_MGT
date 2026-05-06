#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os, time, signal, subprocess, threading, re
from dataclasses import dataclass
from typing import Optional
import open3d as o3d

LOG_DIR = os.path.expanduser("~/logs")
os.makedirs(LOG_DIR, exist_ok=True)

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# 고정 경로 설정 섹션
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

# C++ ICP용 고정 경로 (모든 경로에 expanduser 적용!)
CPP_INPUT_PCD = os.path.expanduser("~/sl_per_ws/gene1.pcd")
CPP_GLOBAL_PCD = os.path.expanduser("~/sl_per_ws/cropped_world_downsampled.pcd")

# CloudComPy용 고정 경로 및 설정
CLOUDCOMPY_WORKDIR = os.path.expanduser("~/sl_per_ws/cloudcompy")
CLOUDCOMPY_SCRIPT = os.path.expanduser("~/sl_per_ws/scripts/test1.py")
CLOUDCOMPY_MODEL_STL = os.path.expanduser("~/sl_per_ws/gene1.stl")  # 직접 STL 경로 지정

# 선택적 설정 (필요시 수정)
PRE_XFORM_PATH = os.path.join(CLOUDCOMPY_WORKDIR, "transformation_matrix_inverse.txt")  # C++ 선행 변환 행렬
OUTPUT_DIR = os.path.expanduser("~/sl_per_ws/global/out_icp_only")
ENABLE_VISUALIZATION = False  # 시각화 비활성화 (서버 환경 - 웹에서 별도 표시)

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

def convert_pcd_to_ply(pcd_path: str) -> str:
    """PCD를 PLY로 변환 (CloudComPy는 PCD 로드 실패, PLY는 성공)"""
    ply_path = pcd_path.rsplit('.', 1)[0] + '.ply'
    if os.path.exists(ply_path):
        pcd_mtime = os.path.getmtime(pcd_path)
        ply_mtime = os.path.getmtime(ply_path)
        if ply_mtime >= pcd_mtime:
            print(f"[변환] 기존 PLY 사용: {ply_path}")
            return ply_path
    print(f"[변환] PCD → PLY: {pcd_path}")
    pcd = o3d.io.read_point_cloud(pcd_path)
    o3d.io.write_point_cloud(ply_path, pcd)
    print(f"[변환] 완료: {ply_path} ({len(pcd.points)} points)")
    return ply_path

# ── 디스플레이(X/GL) 오류 패턴 ──────────────────────────────────────────
DISPLAY_ERR_PATTERNS = [
    r"vtkXOpenGLRenderWindow.*bad X server connection",
    r"QXcbConnection:\s*could not connect to display",
    r"DISPLAY\s*=\s*\.?",            # DISPLAY=. 또는 빈 DISPLAY
    r"GLX.*failed",
    r"Xlib:.*extension.*missing",
    r"xcb:.*ERROR",
    r"Could not initialize offscreen",
    r"qt\.qpa\.xcb.*could not connect to display",  # Qt XCB 연결 실패
    r"QSocketNotifier.*can only be used with threads",  # Qt 스레드 에러
    r"Could not load the Qt platform plugin",  # Qt 플랫폼 플러그인 로드 실패
    r"no Qt platform plugin could be initialized",  # Qt 플랫폼 초기화 실패
]

def is_display_error(log_path: Optional[str]) -> bool:
    """로그 파일에 디스플레이 관련 오류가 있으면 True"""
    if not log_path or not os.path.isfile(log_path):
        return False
    try:
        with open(log_path, "r", encoding="utf-8", errors="ignore") as f:
            text = f.read()
    except Exception:
        return False
    for pat in DISPLAY_ERR_PATTERNS:
        if re.search(pat, text, flags=re.IGNORECASE):
            return True
    return False

# ─────────────────────────────────────────────────────────────

@dataclass
class ProcResult:
    returncode: int
    elapsed: float
    log_path: Optional[str] = None
    killed_by_timeout: bool = False

def _run_stream(cmd, cwd=None, timeout=None, log_path=None) -> ProcResult:
    start = time.time()
    lf = open(log_path, "w", buffering=1) if log_path else None
    try:
        proc = subprocess.Popen(
            cmd,
            cwd=cwd,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1,
            universal_newlines=True,
            start_new_session=True,
        )
        def pump():
            assert proc.stdout is not None
            for line in proc.stdout:
                if lf: lf.write(line)
                else:  print(line, end="")
        t = threading.Thread(target=pump, daemon=True)
        t.start()
        try:
            rc = proc.wait(timeout=timeout)
            return ProcResult(rc, time.time()-start, log_path)
        except subprocess.TimeoutExpired:
            os.killpg(proc.pid, signal.SIGTERM)
            try:
                proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(proc.pid, signal.SIGKILL)
                proc.wait()
            return ProcResult(-9, time.time()-start, log_path, killed_by_timeout=True)
    finally:
        if lf: lf.close()

def run_cpp_icp(timeout_sec: int = 60*30) -> ProcResult:
    """
    고정 경로로 C++ ICP 실행
    """
    ts = int(time.time())
    log = os.path.join(LOG_DIR, f"cpp_icp_{ts}.log")
    
    print(f"[C++ ICP] 고정 경로 사용:")
    print(f"  - Input PCD: {CPP_INPUT_PCD}")
    print(f"  - Global PCD: {CPP_GLOBAL_PCD}")
    print(f"  - Work Dir: {CLOUDCOMPY_WORKDIR}")
    
    # CloudComPy 작업 디렉토리에서 실행 (C++ 출력이 여기에 생성됨)
    cmd = ["bash", os.path.expanduser("~/sl_per_ws/scripts/run_cpp_icp.sh"), CPP_INPUT_PCD, CPP_GLOBAL_PCD]
    result = _run_stream(cmd, cwd=CLOUDCOMPY_WORKDIR, timeout=timeout_sec, log_path=log)
    
    # 생성된 파일 확인
    if result.returncode == 0 or is_display_error(result.log_path):
        xform_file = os.path.join(CLOUDCOMPY_WORKDIR, "transformation_matrix_inverse.txt")
        if os.path.exists(xform_file):
            print(f"\n[C++ ICP] 변환 행렬 파일 생성 확인: {xform_file}")
        else:
            print(f"\n[경고] 변환 행렬 파일이 생성되지 않았습니다: {xform_file}")
    
    return result

def run_cloudcompy_icp(timeout_sec: int = 60*60) -> ProcResult:
    """
    고정 경로와 설정으로 CloudComPy ICP 실행
    """
    ts = int(time.time())
    log = os.path.join(LOG_DIR, f"cloudcompy_icp_{ts}.log")
    
    # CloudComPy는 PCD 로드 실패 → PLY로 변환하여 전달
    scan_ply = convert_pcd_to_ply(CPP_GLOBAL_PCD)

    print(f"[CloudComPy] 고정 설정 사용:")
    print(f"  - Work Dir: {CLOUDCOMPY_WORKDIR}")
    print(f"  - Script: {CLOUDCOMPY_SCRIPT}")
    print(f"  - Model STL: {CLOUDCOMPY_MODEL_STL}")
    print(f"  - Scan PLY: {scan_ply}")
    print(f"  - Output Dir: {OUTPUT_DIR}")

    # CloudComPy 스크립트에 전달할 인자들 구성
    extra_args = [
        "--model-stl", CLOUDCOMPY_MODEL_STL,  # 직접 STL 경로 전달
        "--scan-pcd", scan_ply,  # PLY 파일 경로 (CloudComPy 호환)
        "--out-dir", OUTPUT_DIR,
    ]
    
    # 선택적 인자들
    if PRE_XFORM_PATH and os.path.exists(PRE_XFORM_PATH):
        extra_args.extend(["--pre-xform", PRE_XFORM_PATH])
    
    if ENABLE_VISUALIZATION:
        extra_args.append("--visualize")
    
    cmd = ["bash", os.path.expanduser("~/sl_per_ws/scripts/run_cloudcompy_icp.sh"),
           "--workdir", CLOUDCOMPY_WORKDIR, 
           "--python", CLOUDCOMPY_SCRIPT,
           "--"] + extra_args
    
    return _run_stream(cmd, timeout=timeout_sec, log_path=log)

def run_full_registration() -> None:
    """
    고정 경로로 전체 등록 파이프라인 실행
    
    Note: 더 이상 ship_block_name 인자가 필요없음
    """
    print("="*70)
    print("전체 ICP 등록 파이프라인 시작 (고정 경로 모드)")
    print("="*70)
    
    # 경로 유효성 검증
    if not os.path.exists(CPP_INPUT_PCD):
        raise FileNotFoundError(f"Input PCD를 찾을 수 없습니다: {CPP_INPUT_PCD}")
    if not os.path.exists(CPP_GLOBAL_PCD):
        raise FileNotFoundError(f"Global PCD를 찾을 수 없습니다: {CPP_GLOBAL_PCD}")
    
    print("\n[1/2] C++ ICP 실행 중...")
    print("-"*50)
    r1 = run_cpp_icp()
    print(f"\n[CPP 결과] 종료코드={r1.returncode}, 실행시간={r1.elapsed:.1f}초")
    print(f"로그 파일: {r1.log_path}")
    
    if r1.returncode != 0:
        if is_display_error(r1.log_path):
            print("\n⚠️  [경고] C++ 단계가 디스플레이 관련 오류로 실패했습니다.")
            print("   서버 환경에서는 정상적인 현상입니다. 계속 진행합니다.")
        else:
            print("\n❌ [오류] C++ ICP 단계가 실패했습니다.")
            raise SystemExit(f"종료코드: {r1.returncode}, 로그 확인: {r1.log_path}")
    
    print("\n[2/2] CloudComPy ICP 실행 중...")
    print("-"*50)
    r2 = run_cloudcompy_icp()
    print(f"\n[CloudComPy 결과] 종료코드={r2.returncode}, 실행시간={r2.elapsed:.1f}초")
    print(f"로그 파일: {r2.log_path}")
    
    if r2.returncode != 0:
        print("\n❌ [오류] CloudComPy ICP 단계가 실패했습니다.")
        raise SystemExit(f"종료코드: {r2.returncode}, 로그 확인: {r2.log_path}")
    
    print("\n" + "="*70)
    print("✅ 전체 파이프라인이 성공적으로 완료되었습니다!")
    print(f"출력 디렉토리: {OUTPUT_DIR}")
    print("="*70)

def print_configuration():
    """현재 고정 경로 설정 출력"""
    print("\n" + "="*70)
    print("현재 고정 경로 설정")
    print("="*70)
    print("\n[C++ ICP 경로]")
    print(f"  Input PCD:  {CPP_INPUT_PCD}")
    print(f"  Global PCD: {CPP_GLOBAL_PCD}")
    print(f"  Work Dir:   {CLOUDCOMPY_WORKDIR} (C++ 출력 파일이 여기에 생성됨)")
    
    print("\n[CloudComPy 설정]")
    print(f"  Work Dir:    {CLOUDCOMPY_WORKDIR}")
    print(f"  Script:      {CLOUDCOMPY_SCRIPT}")
    print(f"  Model STL:   {CLOUDCOMPY_MODEL_STL}")
    print(f"  Output Dir:  {OUTPUT_DIR}")
    
    print("\n[선택적 설정]")
    print(f"  Pre-transform: {PRE_XFORM_PATH if PRE_XFORM_PATH else '(없음)'}")
    print(f"  Visualization: {'활성화' if ENABLE_VISUALIZATION else '비활성화'}")
    
    print("\n[로그 디렉토리]")
    print(f"  {LOG_DIR}")
    print("="*70 + "\n")

if __name__ == "__main__":
    import sys
    
    # 명령행 인자 처리
    if len(sys.argv) > 1:
        if sys.argv[1] in ["--help", "-h"]:
            print("\n사용법: python server_runner.py [옵션]")
            print("\n옵션:")
            print("  --config    현재 고정 경로 설정 표시")
            print("  --help, -h  이 도움말 표시")
            print("\n참고: 모든 경로는 스크립트 내부에 고정되어 있습니다.")
            print("      경로 변경이 필요한 경우 스크립트를 직접 수정하세요.")
            sys.exit(0)
        elif sys.argv[1] == "--config":
            print_configuration()
            sys.exit(0)
        else:
            print(f"⚠️  알 수 없는 옵션: {sys.argv[1]}")
            print("   --help 옵션으로 사용법을 확인하세요.")
            sys.exit(1)
    
    # 설정 표시 후 실행
    print_configuration()
    
    # # 사용자 확인 (선택사항)
    # try:
    #     response = input("\n위 설정으로 실행하시겠습니까? (y/n): ").lower()
    #     if response != 'y':
    #         print("실행이 취소되었습니다.")
    #         sys.exit(0)
    # except KeyboardInterrupt:
    #     print("\n실행이 취소되었습니다.")
    #     sys.exit(0)
    
    # 메인 실행
    try:
        run_full_registration()
    except KeyboardInterrupt:
        print("\n\n⚠️  사용자에 의해 중단되었습니다.")
        sys.exit(130)
    except Exception as e:
        print(f"\n❌ 예상치 못한 오류 발생: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)