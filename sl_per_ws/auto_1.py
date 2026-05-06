#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
듀얼 라이다 처리 파이프라인 자동 실행기
실행 순서: view.py → downsample.py → final.py → paview.py
"""

import subprocess
import sys
import os
import time
from dataclasses import dataclass
from typing import Optional, List
from pathlib import Path


# ============================================================
# 🎯 설정
# ============================================================
SCRIPTS = [
    "view.py",       # 1. 듀얼 라이다 정합 및 크롭
    "downsample.py", # 2. 다운샘플링
    "final.py",      # 3. ICP 등록 파이프라인
    "paview.py",     # 4. 경로 시각화
]

# 스크립트가 있는 디렉토리 (None이면 현재 디렉토리)
SCRIPT_DIR = None  # 예: "/root/sl"

# 실패 시 중단 여부
STOP_ON_FAILURE = True

# 각 스크립트별 타임아웃 (초), None이면 무제한
TIMEOUTS = {
    "view.py": None,       # GUI 있음 - 사용자 종료 대기
    "downsample.py": 300,  # 5분
    "final.py": 3600,      # 1시간 (ICP는 오래 걸릴 수 있음)
    "paview.py": None,     # GUI 있음 - 사용자 종료 대기
}

# Python 인터프리터 경로 (None이면 현재 실행 중인 Python 사용)
PYTHON_EXECUTABLE = None  # 예: "/usr/bin/python3"
# ============================================================


@dataclass
class StepResult:
    """각 단계 실행 결과"""
    script: str
    success: bool
    returncode: int
    elapsed: float
    error_msg: Optional[str] = None


def get_python_executable() -> str:
    """사용할 Python 인터프리터 경로 반환"""
    if PYTHON_EXECUTABLE:
        return PYTHON_EXECUTABLE
    return sys.executable


def get_script_path(script_name: str) -> Path:
    """스크립트 전체 경로 반환"""
    if SCRIPT_DIR:
        return Path(SCRIPT_DIR) / script_name
    return Path(script_name)


def run_script(script_name: str, timeout: Optional[float] = None) -> StepResult:
    """
    단일 스크립트 실행
    
    Args:
        script_name: 실행할 스크립트 파일명
        timeout: 타임아웃 (초), None이면 무제한
    
    Returns:
        StepResult: 실행 결과
    """
    script_path = get_script_path(script_name)
    python_exe = get_python_executable()
    
    # 스크립트 존재 여부 확인
    if not script_path.exists():
        return StepResult(
            script=script_name,
            success=False,
            returncode=-1,
            elapsed=0,
            error_msg=f"스크립트를 찾을 수 없음: {script_path}"
        )
    
    print(f"\n{'='*60}")
    print(f"▶ 실행 중: {script_name}")
    print(f"  경로: {script_path.absolute()}")
    if timeout:
        print(f"  타임아웃: {timeout}초")
    print('='*60)
    
    start_time = time.time()
    
    try:
        # 스크립트 실행
        result = subprocess.run(
            [python_exe, str(script_path)],
            cwd=script_path.parent if script_path.parent.exists() else None,
            timeout=timeout,
            # stdout, stderr는 실시간 출력을 위해 상속
        )
        
        elapsed = time.time() - start_time
        success = (result.returncode == 0)
        
        return StepResult(
            script=script_name,
            success=success,
            returncode=result.returncode,
            elapsed=elapsed,
            error_msg=None if success else f"종료 코드: {result.returncode}"
        )
        
    except subprocess.TimeoutExpired:
        elapsed = time.time() - start_time
        return StepResult(
            script=script_name,
            success=False,
            returncode=-9,
            elapsed=elapsed,
            error_msg=f"타임아웃 ({timeout}초 초과)"
        )
        
    except Exception as e:
        elapsed = time.time() - start_time
        return StepResult(
            script=script_name,
            success=False,
            returncode=-1,
            elapsed=elapsed,
            error_msg=str(e)
        )


def print_summary(results: List[StepResult]):
    """실행 결과 요약 출력"""
    print("\n" + "="*60)
    print("📊 파이프라인 실행 결과 요약")
    print("="*60)
    
    total_time = sum(r.elapsed for r in results)
    success_count = sum(1 for r in results if r.success)
    
    for i, r in enumerate(results, 1):
        status = "✅ 성공" if r.success else "❌ 실패"
        print(f"\n  [{i}] {r.script}")
        print(f"      상태: {status}")
        print(f"      실행 시간: {r.elapsed:.1f}초")
        if r.error_msg:
            print(f"      오류: {r.error_msg}")
    
    print("\n" + "-"*60)
    print(f"  전체 실행 시간: {total_time:.1f}초 ({total_time/60:.1f}분)")
    print(f"  성공: {success_count}/{len(results)}")
    print("="*60)


def run_pipeline():
    """
    전체 파이프라인 실행
    
    실행 순서:
        1. view.py      - 듀얼 라이다 정합 및 크롭
        2. downsample.py - 포인트클라우드 다운샘플링
        3. final.py     - C++ ICP + CloudComPy ICP 실행
        4. paview.py    - 결과 경로 시각화
    """
    print("\n" + "="*60)
    print("🚀 듀얼 라이다 처리 파이프라인 시작")
    print("="*60)
    print(f"\n실행할 스크립트 ({len(SCRIPTS)}개):")
    for i, script in enumerate(SCRIPTS, 1):
        print(f"  {i}. {script}")
    
    if SCRIPT_DIR:
        print(f"\n스크립트 디렉토리: {SCRIPT_DIR}")
    
    print(f"실패 시 중단: {'예' if STOP_ON_FAILURE else '아니오'}")
    
    results: List[StepResult] = []
    pipeline_success = True
    
    for script in SCRIPTS:
        timeout = TIMEOUTS.get(script)
        result = run_script(script, timeout)
        results.append(result)
        
        if result.success:
            print(f"\n✅ {script} 완료 ({result.elapsed:.1f}초)")
        else:
            print(f"\n❌ {script} 실패: {result.error_msg}")
            pipeline_success = False
            
            if STOP_ON_FAILURE:
                print("\n⚠️  STOP_ON_FAILURE=True 설정으로 파이프라인 중단")
                break
    
    # 결과 요약 출력
    print_summary(results)
    
    if pipeline_success:
        print("\n✨ 파이프라인이 성공적으로 완료되었습니다!")
        return 0
    else:
        print("\n⚠️  파이프라인이 오류와 함께 종료되었습니다.")
        return 1


def main():
    """메인 진입점"""
    try:
        exit_code = run_pipeline()
        sys.exit(exit_code)
    except KeyboardInterrupt:
        print("\n\n⚠️  사용자에 의해 중단되었습니다.")
        sys.exit(130)
    except Exception as e:
        print(f"\n❌ 예상치 못한 오류: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)


if __name__ == "__main__":
    main()