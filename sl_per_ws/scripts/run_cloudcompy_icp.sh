#!/usr/bin/env bash
set -eo pipefail

# 스크립트 위치 기준으로 워크스페이스 루트 경로 결정
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(dirname "$SCRIPT_DIR")"

# 사용법:
#   run_cloudcompy_icp.sh --workdir <dir> --python <file> -- [이후 인자들은 그대로 python에 전달]
# 기본값:
#   --workdir : <workspace_root>/scripts
#   --python  : test1.py
#
# 예:
#   run_cloudcompy_icp.sh --workdir "$WS_ROOT/scripts" --python test1.py -- --block "SN2695_T140S_processed" --pre-xform "/path/to/inverse.txt"

WORKDIR="$WS_ROOT/scripts"
PYFILE="test1.py"
PASS_ARGS=()

# 인자 파싱
while [[ $# -gt 0 ]]; do
  case "$1" in
    --workdir) WORKDIR="$2"; shift 2 ;;
    --python)  PYFILE="$2";  shift 2 ;;
    --) shift; PASS_ARGS=("$@"); break ;;   # 여기서부터는 파이썬에 그대로 전달
    *) echo "알 수 없는 인자: $1"; exit 2 ;;
  esac
done

# conda 초기화 스크립트
if [[ -f "$HOME/miniconda3/etc/profile.d/conda.sh" ]]; then
  source "$HOME/miniconda3/etc/profile.d/conda.sh"
else
  echo "conda.sh 를 찾지 못했습니다: $HOME/miniconda3/etc/profile.d/conda.sh"
  exit 1
fi

# CloudComPy 전용 활성화 스크립트
# Python 3.10용 CloudComPy 사용 (/root/cloudcompy)
CCY="$HOME/cloudcompy/bin/condaCloud.sh"
if [[ ! -f "$CCY" ]]; then
  echo "condaCloud.sh 를 찾지 못했습니다: $CCY"
  exit 1
fi

# CLOUDCOMPY_ROOT 설정 (Python 3.10용 CloudComPy)
export CLOUDCOMPY_ROOT="$HOME/cloudcompy"

# 비대화식 활성화 (CloudComPy310 환경 사용)
. "$CCY" activate CloudComPy310

# Qt를 offscreen 모드로 설정 (서버 환경용) - 이 3줄만 추가!
export QT_QPA_PLATFORM=offscreen
export QT_DEBUG_PLUGINS=0
echo "[CloudComPy] Qt platform: $QT_QPA_PLATFORM (서버 모드)"

cd "$WORKDIR"
if [[ ! -f "$PYFILE" ]]; then
  echo "파이썬 스크립트를 찾지 못했습니다: $WORKDIR/$PYFILE"
  exit 1
fi

echo "[CloudComPy] python \"$PYFILE\" ${PASS_ARGS[*]}"
exec python "$PYFILE" "${PASS_ARGS[@]}"