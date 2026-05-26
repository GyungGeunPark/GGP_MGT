#!/bin/bash
# MR-BLESW 배터리(아날로그 다이얼) 값 빠른 확인 스크립트 (멀티 디바이스)
# 사용법: bash check_battery_once.sh

echo "=== MR-BLESW Battery Level Quick Check ==="
echo ""

# 연결된 MR-BLESW 디바이스 찾기
DEVICES=$(bluetoothctl devices Connected 2>/dev/null | grep "MR-BLESW" | awk '{print $2}')

if [ -z "$DEVICES" ]; then
    echo "[오류] 연결된 MR-BLESW 디바이스가 없습니다."
    echo "       bluetoothctl devices Connected 으로 확인하세요."
    exit 1
fi

IDX=1
for MAC in $DEVICES; do
    DBUS_PATH="/org/bluez/hci0/dev_$(echo $MAC | tr ':' '_')"
    UPOWER_PATH="keyboard_dev_$(echo $MAC | tr ':' '_')"

    echo "--- DEV-${IDX}: $MAC ---"
    echo ""

    echo "[1] bluetoothctl info:"
    bluetoothctl info "$MAC" 2>/dev/null | grep -E "Name|Battery"
    echo ""

    echo "[2] D-Bus Battery1 query:"
    dbus-send --system --print-reply \
        --dest=org.bluez \
        "$DBUS_PATH" \
        org.freedesktop.DBus.Properties.Get \
        string:"org.bluez.Battery1" \
        string:"Percentage" 2>/dev/null
    echo ""

    echo "[3] UPower:"
    upower -i "/org/freedesktop/UPower/devices/${UPOWER_PATH}" 2>/dev/null | grep -E "model|percentage|updated"
    echo ""

    IDX=$((IDX + 1))
done

echo "=== 완료 ==="
