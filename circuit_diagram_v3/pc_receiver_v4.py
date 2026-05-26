#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
JRT U81 무선 거리측정 — PC 수신 소프트웨어 v4
============================================================================
option_b_firmware_v4.ino 와 짝을 이루는 PC 측 수신기.
  - BLE 모드 : NimBLE NUS 알림(notify) 수신 + 선택적 명령 전송
  - UDP 모드 : WiFi UDP 패킷 수신

펌웨어 패킷 (15바이트, 리틀엔디안):
    오프셋  타입      필드
    0       uint8     sensor id (0x01)
    1..4    float32   distance_mm
    5..8    uint32    timestamp_ms (ESP millis)
    9..12   float32   battery_v
    13..14  uint16    signal_quality (SQ, 낮을수록 좋음; 0xFFFF=미상)
  → struct 포맷: '<B f I f H'  (pad 없음, 총 15B)  — 펌웨어 memcpy 순서와 동일

설치:
    pip install bleak        # BLE 모드에만 필요 (UDP는 표준 라이브러리만 사용)

사용 예:
    python3 pc_receiver_v4.py udp                 # UDP 4210 수신
    python3 pc_receiver_v4.py udp --port 4210 --csv log.csv
    python3 pc_receiver_v4.py ble                 # "JRT-U81-LDS" 검색·연결
    python3 pc_receiver_v4.py ble --laser-on --csv log.csv
============================================================================
"""
import argparse
import csv
import datetime as dt
import struct
import sys

# ── 펌웨어와 합의된 상수 ──
PACKET_FMT  = "<BfIfH"            # 15바이트 (v4: SQ 추가)
PACKET_SIZE = struct.calcsize(PACKET_FMT)   # == 15
assert PACKET_SIZE == 15, f"패킷 크기 불일치: {PACKET_SIZE}"
SQ_UNKNOWN  = 0xFFFF             # 신호품질 미상 센티넬

UDP_PORT_DEFAULT = 4210
BLE_NAME_DEFAULT = "JRT-U81-LDS"
NUS_TX_UUID = "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"  # ESP→PC (notify)
NUS_RX_UUID = "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"  # PC→ESP (write)

LOW_BATT_WARN = 2.90   # 펌웨어 경고 임계값과 동일(표시용)


# ─────────────────────────── 공통 ───────────────────────────
class Sink:
    """수신 데이터 출력 + (선택) CSV 로깅."""
    def __init__(self, csv_path=None):
        self.count = 0
        self.writer = None
        self.fh = None
        if csv_path:
            self.fh = open(csv_path, "w", newline="", encoding="utf-8")
            self.writer = csv.writer(self.fh)
            self.writer.writerow(["pc_time_iso", "seq", "distance_mm", "sq", "esp_ts_ms", "battery_v"])
            print(f"[로그] CSV 기록: {csv_path}")

    def feed(self, rec):
        self.count += 1
        now = dt.datetime.now().isoformat(timespec="milliseconds")
        warn = "  ⚠저전압" if (0 < rec["battery_v"] < LOW_BATT_WARN) else ""
        sq = rec.get("sq", -1)
        sq_s = "--" if sq < 0 else str(sq)
        if rec["distance_mm"] < 0:
            dist_s = "측정실패"
        else:
            dist_s = f"{rec['distance_mm']:8.1f} mm ({rec['distance_mm']/1000:.3f} m)"
        print(f"[{now}] #{self.count:<5} id={rec['id']} "
              f"dist={dist_s}  SQ={sq_s}  batt={rec['battery_v']:.2f}V  esp_ts={rec['esp_ts_ms']}ms{warn}")
        if self.writer:
            self.writer.writerow([now, self.count, f"{rec['distance_mm']:.1f}",
                                  sq, rec["esp_ts_ms"], f"{rec['battery_v']:.3f}"])
            self.fh.flush()

    def close(self):
        if self.fh:
            self.fh.close()


def parse_packet(data: bytes):
    """15바이트 패킷 → dict. 크기/ID 불일치 시 None. SQ 미상은 -1."""
    if len(data) != PACKET_SIZE:
        return None
    sid, dist_mm, ts_ms, batt, sq = struct.unpack(PACKET_FMT, data)
    if sid != 0x01:
        return None
    return {"id": sid, "distance_mm": dist_mm, "esp_ts_ms": ts_ms,
            "battery_v": batt, "sq": (-1 if sq == SQ_UNKNOWN else sq)}


# ─────────────────────────── UDP 모드 ───────────────────────────
def run_udp(args):
    import socket
    sink = Sink(args.csv)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(("0.0.0.0", args.port))
    print(f"[UDP] 0.0.0.0:{args.port} 수신 대기 (Ctrl-C 종료)")
    try:
        while True:
            data, addr = sock.recvfrom(64)
            rec = parse_packet(data)
            if rec is None:
                print(f"[UDP] 무시: {addr}에서 {len(data)}B (형식 불일치)")
                continue
            sink.feed(rec)
    except KeyboardInterrupt:
        print("\n[UDP] 종료")
    finally:
        sock.close()
        sink.close()


# ─────────────────────────── BLE 모드 ───────────────────────────
def run_ble(args):
    try:
        import asyncio
        from bleak import BleakClient, BleakScanner
    except ImportError:
        sys.exit("BLE 모드에는 bleak가 필요합니다: pip install bleak")

    sink = Sink(args.csv)

    async def main_async():
        print(f"[BLE] '{args.name}' 검색 중...")
        device = await BleakScanner.find_device_by_name(args.name, timeout=args.scan_timeout)
        if device is None:
            sys.exit(f"[BLE] '{args.name}' 미발견 (펌웨어 광고 중인지 확인)")
        print(f"[BLE] 발견: {device.address} → 연결")

        def on_notify(_char, data: bytearray):
            rec = parse_packet(bytes(data))
            if rec:
                sink.feed(rec)
            else:
                print(f"[BLE] 무시: {len(data)}B (형식 불일치)")

        async with BleakClient(device) as client:
            print("[BLE] 연결됨. 알림 구독.")
            await client.start_notify(NUS_TX_UUID, on_notify)
            if args.laser_on:
                await client.write_gatt_char(NUS_RX_UUID, b"O", response=False)
                print("[BLE] 레이저 ON 명령('O') 전송")
            try:
                while client.is_connected:
                    await asyncio.sleep(0.5)
            except asyncio.CancelledError:
                pass
            finally:
                try:
                    await client.stop_notify(NUS_TX_UUID)
                except Exception:
                    pass

    try:
        import asyncio
        asyncio.run(main_async())
    except KeyboardInterrupt:
        print("\n[BLE] 종료")
    finally:
        sink.close()


# ─────────────────────────── 진입점 ───────────────────────────
def build_parser():
    p = argparse.ArgumentParser(description="JRT U81 무선 거리측정 PC 수신기 v4")
    sub = p.add_subparsers(dest="mode", required=True)

    pu = sub.add_parser("udp", help="WiFi UDP 수신")
    pu.add_argument("--port", type=int, default=UDP_PORT_DEFAULT, help=f"UDP 포트 (기본 {UDP_PORT_DEFAULT})")
    pu.add_argument("--csv", help="CSV 로그 경로")

    pb = sub.add_parser("ble", help="BLE NUS 수신")
    pb.add_argument("--name", default=BLE_NAME_DEFAULT, help=f"BLE 장치 이름 (기본 {BLE_NAME_DEFAULT})")
    pb.add_argument("--scan-timeout", type=float, default=10.0, help="검색 타임아웃(초)")
    pb.add_argument("--laser-on", action="store_true", help="연결 시 레이저 ON('O') 전송")
    pb.add_argument("--csv", help="CSV 로그 경로")
    return p


def main():
    args = build_parser().parse_args()
    if args.mode == "udp":
        run_udp(args)
    elif args.mode == "ble":
        run_ble(args)


if __name__ == "__main__":
    main()
