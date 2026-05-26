#!/usr/bin/env bash
# ============================================================================
# JRT U81 v4 — 소프트웨어 테스트 러너
#   - 파이썬 구문 컴파일 체크
#   - 패킷 포맷 회귀(15B + SQ)
#   - 브리지 E2E(UDP 주입 → distance/SQ/battery)
#   - 브리지 동시성 견고성(다중 클라 + 플러드)
# 사용: bash run_tests.sh
# 의존: python3, fastapi, uvicorn[standard], websockets
#       (CI/로컬에서 `pip install fastapi "uvicorn[standard]" websockets`)
# ============================================================================
set -uo pipefail
cd "$(dirname "$0")"
fail=0

step() {  # step "이름" 명령...
  local name="$1"; shift
  echo "──────────────────────────────────────────────"
  echo "▶ $name"
  if "$@"; then
    echo "  ✅ $name"
  else
    echo "  ❌ $name (exit $?)"
    fail=1
  fi
}

# 1) 구문 컴파일 (실행 아님 — matplotlib 등 무거운 import 불요)
step "py-compile" python3 -m py_compile \
  circuit_diagram_v3/pc_receiver_v4.py \
  circuit_diagram_v3/generate_v4_images.py \
  circuit_diagram_v3/make_solder_doc.py \
  circuit_diagram_v3/test_packet_format.py \
  lsd_interface/wireless_bridge_v4.py \
  lsd_interface/test_bridge_e2e.py \
  lsd_interface/test_bridge_concurrency.py

# 2) 패킷 포맷 계약
step "packet-format" python3 circuit_diagram_v3/test_packet_format.py

# 3) 브리지 E2E (단일 클라이언트)
step "bridge-e2e" python3 lsd_interface/test_bridge_e2e.py

# 4) 브리지 동시성 견고성 (다중 클라이언트)
step "bridge-concurrency" python3 lsd_interface/test_bridge_concurrency.py

echo "──────────────────────────────────────────────"
if [ "$fail" -eq 0 ]; then
  echo "🟢 ALL TESTS PASSED"
else
  echo "🔴 SOME TESTS FAILED"
fi
exit "$fail"
