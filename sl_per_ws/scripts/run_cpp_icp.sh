#!/usr/bin/env bash
if [ -n "$BASH_VERSION" ]; then
    set -euo pipefail  # bash에서만 작동
else
    set -eu  # 다른 셸에서는 기본 옵션만
fi

# 스크립트 위치 기준으로 워크스페이스 루트 경로 결정
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(dirname "$SCRIPT_DIR")"

# 사용법:
#   run_cpp_icp.sh "<INPUT_PCD>" "<GLOBAL_PCD>"
# 예:
#   run_cpp_icp.sh "/mnt/Share/cad_files/modi_stl/SN2695_T140S_processed/SN2695_T140S_processed.pcd" \
#                  "/mnt/Share/perception/data/global/global_cropped_point_cloud.pcd"

INPUT_PCD="${1:-}"
GLOBAL_PCD="${2:-}"

if [[ -z "${INPUT_PCD}" || -z "${GLOBAL_PCD}" ]]; then
  echo "Usage: run_cpp_icp.sh <INPUT_PCD> <GLOBAL_PCD>"
  exit 2
fi

# 워크스페이스에서 폴더명 자동 탐지 (portable | potable 혼재 대응)
CAND1="$WS_ROOT/icp_registration_portable"
CAND2="$WS_ROOT/icp_registration_potable"
if [[ -d "$CAND1" ]]; then
  ICP_DIR="$CAND1"
elif [[ -d "$CAND2" ]]; then
  ICP_DIR="$CAND2"
else
  echo "icp_registration_(portable|potable) 디렉터리를 워크스페이스에서 찾지 못했습니다: $WS_ROOT"
  exit 1
fi

RUN_SH="$ICP_DIR/run_noviz.sh"
if [[ ! -x "$RUN_SH" ]]; then
  echo "실행 스크립트를 찾지 못했거나 실행 권한이 없습니다: $RUN_SH"
  exit 1
fi

# run_noviz.sh는 내부에서 시각화 없이 동작해야 하며,
# 산출물(예: transformation_matrix_inverse.txt)을 ICP_DIR에 남긴다고 가정합니다.
echo "[CPP-ICP] $RUN_SH \"$INPUT_PCD\" \"$GLOBAL_PCD\""
exec "$RUN_SH" "$INPUT_PCD" "$GLOBAL_PCD"
