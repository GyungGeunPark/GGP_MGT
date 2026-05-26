#!/bin/bash
# UPower를 이용한 MR-BLESW 배터리(아날로그 다이얼) 모니터링
# 값이 변경될 때만 출력됩니다.
# 사용법: bash monitor_upower.sh

echo "=== UPower 이벤트 기반 모니터 ==="
echo "다이얼을 돌리면 변경 이벤트가 표시됩니다."
echo "종료: Ctrl+C"
echo "---"

upower --monitor-detail 2>/dev/null | while read -r line; do
    # MR-BLESW 관련 이벤트만 필터링
    if echo "$line" | grep -q "E9_F7_6F_28_CA_16\|percentage\|MR-BLESW"; then
        echo "[$(date '+%H:%M:%S')] $line"
    fi
done
