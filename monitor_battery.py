#!/usr/bin/env python3
"""
MR-BLESW Bluetooth Analog Dial Monitor (Multi-Device)
======================================================
연결된 모든 MR-BLESW 디바이스의 아날로그 다이얼(Battery Level)을
실시간으로 모니터링하는 스크립트.

사용법:
  python3 monitor_battery.py              # 연결된 모든 MR-BLESW 자동 감지
  python3 monitor_battery.py D3:5B:EE:D6:8D:87   # 특정 디바이스만
  python3 monitor_battery.py E9:F7:6F:28:CA:16 D3:5B:EE:D6:8D:87  # 여러 개 지정
"""

import sys
import time
import signal
import subprocess
from datetime import datetime

import dbus
from dbus.mainloop.glib import DBusGMainLoop
from gi.repository import GLib

# ── 상수 ──────────────────────────────────────────────
GATT_CHAR_IFACE = "org.bluez.GattCharacteristic1"
BATTERY_IFACE = "org.bluez.Battery1"
DEVICE_IFACE = "org.bluez.Device1"
PROPERTIES_IFACE = "org.freedesktop.DBus.Properties"
OBJMGR_IFACE = "org.freedesktop.DBus.ObjectManager"

BATTERY_CHAR_SUFFIX = "/service002e/char002f"
POLL_INTERVAL = 1.0

# ── ANSI 색상 (디바이스 구분용) ───────────────────────
COLORS = [
    "\033[96m",   # cyan
    "\033[93m",   # yellow
    "\033[92m",   # green
    "\033[95m",   # magenta
    "\033[91m",   # red
    "\033[94m",   # blue
]
RESET = "\033[0m"
BOLD = "\033[1m"


class DeviceMonitor:
    """단일 MR-BLESW 디바이스 모니터"""

    def __init__(self, mac, label, color):
        self.mac = mac
        self.label = label
        self.color = color
        self.device_path = "/org/bluez/hci0/dev_" + mac.replace(":", "_")
        self.char_path = self.device_path + BATTERY_CHAR_SUFFIX
        self.last_value = None
        self.change_count = 0
        self.min_value = 100
        self.max_value = 0
        self.values_history = []
        self.notify_active = False

    def make_bar(self, value, width=30):
        filled = int(width * value / 100)
        return "█" * filled + "░" * (width - filled)

    def log_value(self, value, source="poll"):
        now = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        changed = ""

        if self.last_value is not None and value != self.last_value:
            diff = value - self.last_value
            self.change_count += 1
            changed = f"  ← 변화! ({diff:+d}, #{self.change_count})"
        elif self.last_value is not None and value == self.last_value:
            if source in ("poll", "gatt-rd"):
                return
            changed = "  (동일)"

        if value < self.min_value:
            self.min_value = value
        if value > self.max_value:
            self.max_value = value

        self.values_history.append((time.time(), value))
        if len(self.values_history) > 300:
            self.values_history.pop(0)

        bar = self.make_bar(value)
        tag = f"{self.color}{self.label}{RESET}"
        print(
            f"[{now}] {tag} [{source:8s}] 값: {value:3d}% [{bar}]{changed}",
            flush=True,
        )
        self.last_value = value

    def summary(self):
        lines = []
        lines.append(f"  {self.color}{self.label}{RESET} ({self.mac})")
        lines.append(f"    Notification: {'활성' if self.notify_active else '비활성'}")
        lines.append(f"    변화 횟수:   {self.change_count}회")
        lines.append(f"    최솟값:      {self.min_value}%")
        lines.append(f"    최댓값:      {self.max_value}%")
        lines.append(f"    마지막 값:   {self.last_value}%")
        if len(self.values_history) > 1:
            vals = [v for _, v in self.values_history]
            avg = sum(vals) / len(vals)
            lines.append(f"    평균값:      {avg:.1f}%")
        return "\n".join(lines)


def discover_mr_blesw(bus):
    """D-Bus ObjectManager를 통해 연결된 MR-BLESW 디바이스를 자동 감지"""
    devices = []
    try:
        obj = bus.get_object("org.bluez", "/")
        mgr = dbus.Interface(obj, OBJMGR_IFACE)
        objects = mgr.GetManagedObjects()

        for path, interfaces in objects.items():
            if DEVICE_IFACE in interfaces:
                props = interfaces[DEVICE_IFACE]
                name = str(props.get("Name", ""))
                connected = bool(props.get("Connected", False))
                if name == "MR-BLESW" and connected:
                    mac = str(props.get("Address", ""))
                    devices.append(mac)
    except dbus.exceptions.DBusException as e:
        print(f"[오류] 디바이스 탐색 실패: {e}")
    return devices


def start_notify_for_device(bus, dev):
    """디바이스의 GATT Characteristic에 StartNotify 호출"""
    try:
        obj = bus.get_object("org.bluez", dev.char_path)
        char_iface = dbus.Interface(obj, GATT_CHAR_IFACE)
        props = dbus.Interface(obj, PROPERTIES_IFACE)

        notifying = bool(props.Get(GATT_CHAR_IFACE, "Notifying"))
        flags = list(props.Get(GATT_CHAR_IFACE, "Flags"))

        print(f"  {dev.color}{dev.label}{RESET} Flags: {flags}, Notifying: {notifying}")

        if "notify" in flags and not notifying:
            char_iface.StartNotify()
            notifying = bool(props.Get(GATT_CHAR_IFACE, "Notifying"))

        if notifying:
            dev.notify_active = True
            print(f"  {dev.color}{dev.label}{RESET} Notification 활성화 성공!")
            return True
        else:
            print(f"  {dev.color}{dev.label}{RESET} Notification 실패, 폴링 모드")
            return False

    except dbus.exceptions.DBusException as e:
        print(f"  {dev.color}{dev.label}{RESET} StartNotify 오류: {e}")
        return False


def read_initial_value(bus, dev):
    """초기값 읽기"""
    try:
        obj = bus.get_object("org.bluez", dev.char_path)
        char_iface = dbus.Interface(obj, GATT_CHAR_IFACE)
        raw = char_iface.ReadValue({"offset": dbus.UInt16(0)})
        if raw:
            dev.log_value(int(raw[0]), source="init")
    except dbus.exceptions.DBusException:
        try:
            obj = bus.get_object("org.bluez", dev.device_path)
            props = dbus.Interface(obj, PROPERTIES_IFACE)
            val = int(props.Get(BATTERY_IFACE, "Percentage"))
            dev.log_value(val, source="init-b1")
        except dbus.exceptions.DBusException as e2:
            print(f"  {dev.color}{dev.label}{RESET} 초기값 읽기 실패: {e2}")


def make_gatt_signal_handler(dev):
    """GATT Characteristic PropertiesChanged 핸들러 생성"""
    def handler(interface, changed_props, invalidated):
        if interface == GATT_CHAR_IFACE and "Value" in changed_props:
            raw = bytes(changed_props["Value"])
            if raw:
                dev.log_value(raw[0], source="notify")
    return handler


def make_battery1_signal_handler(dev):
    """Battery1 PropertiesChanged 핸들러 생성"""
    def handler(interface, changed_props, invalidated):
        if interface == BATTERY_IFACE and "Percentage" in changed_props:
            dev.log_value(int(changed_props["Percentage"]), source="bat1-sig")
    return handler


def make_poll_func(bus, dev):
    """폴링 함수 생성"""
    def poll():
        try:
            obj = bus.get_object("org.bluez", dev.char_path)
            char_iface = dbus.Interface(obj, GATT_CHAR_IFACE)
            raw = char_iface.ReadValue({"offset": dbus.UInt16(0)})
            if raw:
                dev.log_value(int(raw[0]), source="gatt-rd")
        except dbus.exceptions.DBusException as e:
            if "InProgress" not in str(e):
                print(f"  {dev.color}{dev.label}{RESET} [폴링 오류] {e}", flush=True)
        return True
    return poll


def main():
    start_time = time.time()

    # D-Bus 설정
    DBusGMainLoop(set_as_default=True)
    bus = dbus.SystemBus()

    # 대상 디바이스 결정
    if len(sys.argv) > 1:
        target_macs = sys.argv[1:]
    else:
        target_macs = discover_mr_blesw(bus)

    if not target_macs:
        print("[오류] 연결된 MR-BLESW 디바이스를 찾을 수 없습니다.")
        print("       블루투스 연결 상태를 확인하세요: bluetoothctl devices Connected")
        sys.exit(1)

    # DeviceMonitor 인스턴스 생성
    devices = []
    for i, mac in enumerate(target_macs):
        color = COLORS[i % len(COLORS)]
        short = mac[-5:].replace(":", "")
        label = f"[DEV-{i+1} {short}]"
        devices.append(DeviceMonitor(mac, label, color))

    # 헤더 출력
    print("=" * 76)
    print(f"  MR-BLESW 아날로그 다이얼 실시간 모니터 ({len(devices)}대)")
    print("=" * 76)
    for dev in devices:
        print(f"  {dev.color}{dev.label}{RESET} {dev.mac}  →  {dev.char_path}")
    print(f"  시작: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    print("-" * 76)
    print("  다이얼을 돌려보세요! 종료: Ctrl+C")
    print("-" * 76)

    # 각 디바이스 설정
    for dev in devices:
        # 시그널 등록
        bus.add_signal_receiver(
            make_gatt_signal_handler(dev),
            signal_name="PropertiesChanged",
            dbus_interface=PROPERTIES_IFACE,
            path=dev.char_path,
        )
        bus.add_signal_receiver(
            make_battery1_signal_handler(dev),
            signal_name="PropertiesChanged",
            dbus_interface=PROPERTIES_IFACE,
            path=dev.device_path,
        )

    # StartNotify
    print("\n[시스템] Notification 활성화...")
    for dev in devices:
        start_notify_for_device(bus, dev)

    # 초기값 읽기
    print()
    for dev in devices:
        read_initial_value(bus, dev)

    # 폴링 등록
    for dev in devices:
        GLib.timeout_add(int(POLL_INTERVAL * 1000), make_poll_func(bus, dev))

    print(f"\n[시스템] 모니터링 시작! (폴링 {POLL_INTERVAL}초 간격)\n")

    # 메인 루프
    loop = GLib.MainLoop()

    def signal_handler(sig, frame):
        print("\n\n[시스템] 종료 중...")
        for dev in devices:
            if dev.notify_active:
                try:
                    obj = bus.get_object("org.bluez", dev.char_path)
                    dbus.Interface(obj, GATT_CHAR_IFACE).StopNotify()
                except Exception:
                    pass
        loop.quit()

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    try:
        loop.run()
    except KeyboardInterrupt:
        pass
    finally:
        elapsed = time.time() - start_time
        print("\n" + "=" * 76)
        print(f"  모니터링 결과 요약 (총 {elapsed:.1f}초)")
        print("=" * 76)
        for dev in devices:
            print(dev.summary())
        print("=" * 76)


if __name__ == "__main__":
    main()
