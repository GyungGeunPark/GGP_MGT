#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
JRT U81 무선(BLE/WiFi) → 웹 UI 브리지  (v4)
============================================================================
circuit_diagram_v3/option_b_firmware_v4.ino 가 보내는 13바이트 무선 패킷을
기존 lsd_interface/static 웹 UI(app.js)가 기대하는 WebSocket 스키마로 변환해
**같은 UI를 무선으로 재사용**한다. ROS2 노드(main.py)와 독립된 별도 진입점.

데이터 흐름:
  ESP32-C3 ──[BLE NUS notify | WiFi UDP]──▶ 이 브리지 ──/ws(WebSocket)──▶ static UI

UI WebSocket 스키마(app.js 기준):
  수신(서버→UI): {type:'status'|'distance'|'ack'|'voltage'|'error', ...}
  송신(UI→서버): {cmd:'power_on'|'power_off'|'measure_once'|
                       'continuous_on'|'continuous_off'|'set_mode'|'read_voltage'}

무선 패킷(펌웨어와 합의, 리틀엔디안 15B): '<BfIfH'
  id(u8)=0x01 · distance_mm(f32) · timestamp_ms(u32) · battery_v(f32) · sq(u16)
  ※ sq=0xFFFF → 미상(-1, UI '--'). 배터리 %는 전압→잔량 매핑으로 산출(batt_pct).

실행:
  pip install fastapi "uvicorn[standard]"            # UDP 모드
  pip install bleak                                  # BLE 모드 추가 시
  python3 wireless_bridge_v4.py --mode udp --port 4210
  python3 wireless_bridge_v4.py --mode ble --name JRT-U81-LDS
  브라우저: http://localhost:8000
============================================================================
"""
import argparse
import asyncio
import contextlib
import os
import struct
import time
from pathlib import Path

from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse
from fastapi.staticfiles import StaticFiles

# ── 무선 패킷 (pc_receiver_v4.py 와 동일 계약, v4: 15B + SQ) ──
PACKET_FMT = "<BfIfH"
PACKET_SIZE = struct.calcsize(PACKET_FMT)          # 15
SQ_UNKNOWN = 0xFFFF
LOW_BATT_MV = 2800                                 # UI 저전압 뱃지 임계(=펌웨어 2.8V 종료)
BATT_FULL_V, BATT_EMPTY_V = 3.0, 2.7               # 알카라인 사용범위(UVLO 위)


def batt_pct(v: float) -> int:
    """알카라인 2×AA 전압 → 잔량 %(3.0V=100, 2.7V=0, UVLO 기준)."""
    if v <= 0:
        return 0
    p = (v - BATT_EMPTY_V) / (BATT_FULL_V - BATT_EMPTY_V) * 100.0
    return max(0, min(100, int(round(p))))

STATIC_DIR = Path(__file__).parent / "static"

# 런타임 설정(환경변수로 주입; __main__ 에서 설정)
MODE = os.environ.get("WIRELESS_MODE", "udp")      # 'udp' | 'ble'
UDP_PORT = int(os.environ.get("WIRELESS_UDP_PORT", "4210"))
BLE_NAME = os.environ.get("WIRELESS_BLE_NAME", "JRT-U81-LDS")
NUS_TX_UUID = "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"  # ESP→PC notify
NUS_RX_UUID = "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"  # PC→ESP write


def parse_packet(data: bytes):
    if len(data) != PACKET_SIZE:
        return None
    sid, dist_mm, ts_ms, batt, sq = struct.unpack(PACKET_FMT, data)
    if sid != 0x01:
        return None
    return {"distance_mm": dist_mm, "esp_ts_ms": ts_ms, "battery_v": batt,
            "sq": (-1 if sq == SQ_UNKNOWN else sq)}


# ─────────────────────── 공유 상태 + 브로드캐스트 ───────────────────────
class Hub:
    def __init__(self):
        self.clients: set[WebSocket] = set()
        self.laser_on = True        # 펌웨어는 부팅 시 레이저 ON
        self.streaming = True       # 펌웨어는 자동 연속 측정
        self.last_batt_mv = 0
        self.last_batt_v = 0.0
        self.last_sq = -1
        self.ble_client = None      # BLE 모드에서 명령 전송용
        self.source_label = ""
        self.send_lock = asyncio.Lock()   # ★ 동일 소켓 동시 send 방지(ASGI 규약)

    async def register(self, ws: WebSocket):
        await ws.accept()
        self.clients.add(ws)
        await self.send_status(ws)  # 접속 즉시 상태 1회

    def unregister(self, ws: WebSocket):
        self.clients.discard(ws)

    async def broadcast(self, msg: dict):
        # ★ 모든 아웃바운드 send를 직렬화 → on_packet/ticker/handle_cmd 간 인터리브 방지
        async with self.send_lock:
            dead = []
            for ws in list(self.clients):
                try:
                    await ws.send_json(msg)
                except Exception:
                    dead.append(ws)
            for ws in dead:
                self.clients.discard(ws)

    def status_payload(self):
        return {
            "type": "status",
            "powered": self.laser_on,
            "continuous": self.streaming,
            "port": self.source_label,
            "voltage_mv": self.last_batt_mv or None,
            "battery_pct": (batt_pct(self.last_batt_v) if self.last_batt_v else None),
            "measure_mode": "auto",
        }

    async def send_status(self, ws=None):
        msg = self.status_payload()
        if ws is not None:
            async with self.send_lock:
                with contextlib.suppress(Exception):
                    await ws.send_json(msg)
        else:
            await self.broadcast(msg)

    async def on_packet(self, rec: dict):
        """무선 패킷 1건 → UI 메시지 변환·브로드캐스트."""
        self.last_batt_v = rec["battery_v"]
        self.last_batt_mv = int(round(rec["battery_v"] * 1000))
        self.last_sq = rec.get("sq", -1)
        if not self.streaming:
            return
        dmm = rec["distance_mm"]
        if dmm is None or dmm < 0:
            await self.broadcast({"type": "distance", "success": False, "error": "측정 실패"})
            return
        await self.broadcast({
            "type": "distance",
            "success": True,
            "mm": int(round(dmm)),
            "m": round(dmm / 1000.0, 4),
            "signal_quality": self.last_sq,   # ★v4: 실제 SQ (-1=미상 → UI '--')
            "low_confidence": (self.last_sq >= 1000),  # SQ 높으면 저신뢰
            "timestamp": time.time(),    # PC 벽시계(초)
        })

    async def send_ble(self, ch: bytes) -> bool:
        if self.ble_client is None:
            return False
        with contextlib.suppress(Exception):
            await self.ble_client.write_gatt_char(NUS_RX_UUID, ch, response=False)
            return True
        return False


hub = Hub()


# ─────────────────────── 명령 처리 (UI → 서버) ───────────────────────
async def handle_cmd(data: dict):
    cmd = (data.get("cmd") or "").strip()
    ble = (MODE == "ble")

    async def ack(success, message):
        await hub.broadcast({"type": "ack", "cmd": cmd, "success": success, "message": message})

    if cmd == "power_on":
        ok = await hub.send_ble(b"O") if ble else False
        hub.laser_on = True
        await ack(True, "레이저 ON" + ("" if ok or ble else " (UDP 단방향: 표시만)"))
    elif cmd == "power_off":
        ok = await hub.send_ble(b"C") if ble else False
        hub.laser_on = False
        await ack(True, "레이저 OFF" + ("" if ok or ble else " (UDP 단방향: 표시만)"))
    elif cmd == "measure_once":
        if ble:
            await hub.send_ble(b"D")
            await ack(True, "측정 명령 전송")
        else:
            await ack(False, "UDP 단방향: 펌웨어가 자동 연속 측정합니다")
    elif cmd == "continuous_on":
        hub.streaming = True
        await ack(True, "연속 표시 시작")
    elif cmd == "continuous_off":
        hub.streaming = False
        await ack(True, "연속 표시 중지(스트림 무시)")
    elif cmd == "set_mode":
        await ack(True, "무선 모드는 펌웨어 측정주기 고정(set_mode 무시)")
    elif cmd == "read_voltage":
        mv = hub.last_batt_mv
        if mv:
            pct = batt_pct(hub.last_batt_v)
            await hub.broadcast({"type": "voltage", "success": True,
                                 "voltage_mv": mv, "battery_pct": pct,
                                 "message": f"{mv/1000:.3f} V ({pct}%)"})
        else:
            await hub.broadcast({"type": "voltage", "success": False,
                                 "message": "아직 배터리 데이터 없음"})
    else:
        await hub.broadcast({"type": "error", "message": f"알 수 없는 명령: {cmd}"})

    if cmd in ("power_on", "power_off", "continuous_on", "continuous_off"):
        await hub.send_status()


# ─────────────────────── 무선 소스 태스크 ───────────────────────
class UDPProto(asyncio.DatagramProtocol):
    def __init__(self, loop):
        self.loop = loop

    def datagram_received(self, data, addr):
        rec = parse_packet(data)
        if rec:
            # 콜백은 이벤트 루프 스레드에서 실행 → create_task 가 올바름
            self.loop.create_task(hub.on_packet(rec))


async def run_udp_source():
    loop = asyncio.get_running_loop()
    hub.source_label = f"UDP:{UDP_PORT}"
    transport, _ = await loop.create_datagram_endpoint(
        lambda: UDPProto(loop), local_addr=("0.0.0.0", UDP_PORT))
    print(f"[bridge] UDP 수신 0.0.0.0:{UDP_PORT}")
    try:
        while True:
            await asyncio.sleep(3600)
    finally:
        transport.close()   # ★ 종료 시 소켓 정리(누수 방지)


async def run_ble_source():
    try:
        from bleak import BleakClient, BleakScanner
    except ImportError:
        print("[bridge] BLE 모드에는 bleak 필요: pip install bleak")
        return
    hub.source_label = f"BLE:{BLE_NAME}"
    loop = asyncio.get_running_loop()

    def on_notify(_c, data: bytearray):
        rec = parse_packet(bytes(data))
        if rec:
            asyncio.run_coroutine_threadsafe(hub.on_packet(rec), loop)

    while True:
        print(f"[bridge] BLE '{BLE_NAME}' 검색...")
        dev = await BleakScanner.find_device_by_name(BLE_NAME, timeout=10.0)
        if dev is None:
            await asyncio.sleep(3)
            continue
        try:
            async with BleakClient(dev) as client:
                hub.ble_client = client
                await client.start_notify(NUS_TX_UUID, on_notify)
                print(f"[bridge] BLE 연결: {dev.address}")
                while client.is_connected:
                    await asyncio.sleep(0.5)
        except Exception as e:
            print(f"[bridge] BLE 오류: {e}")
        finally:
            hub.ble_client = None
        await asyncio.sleep(2)  # 재연결 대기


async def run_status_ticker():
    while True:
        await asyncio.sleep(2.0)
        if hub.clients:
            await hub.send_status()


# ─────────────────────── FastAPI 앱 ───────────────────────
@contextlib.asynccontextmanager
async def lifespan(app: FastAPI):
    src = run_ble_source if MODE == "ble" else run_udp_source
    tasks = [asyncio.create_task(src()), asyncio.create_task(run_status_ticker())]
    print(f"[bridge] 모드={MODE} · UI: http://localhost:8000")
    try:
        yield
    finally:
        for t in tasks:
            t.cancel()
        with contextlib.suppress(Exception):
            await asyncio.gather(*tasks, return_exceptions=True)


app = FastAPI(title="JRT U81 무선 브리지 v4", lifespan=lifespan)
app.mount("/static", StaticFiles(directory=str(STATIC_DIR)), name="static")


@app.get("/")
async def index():
    return FileResponse(str(STATIC_DIR / "index.html"))


@app.websocket("/ws")
async def ws_endpoint(ws: WebSocket):
    await hub.register(ws)
    try:
        while True:
            data = await ws.receive_json()
            await handle_cmd(data)
    except WebSocketDisconnect:
        pass
    except Exception:
        pass
    finally:
        hub.unregister(ws)


def main():
    global MODE, UDP_PORT, BLE_NAME
    ap = argparse.ArgumentParser(description="JRT U81 무선 → 웹 UI 브리지 v4")
    ap.add_argument("--mode", choices=["udp", "ble"], default=MODE)
    ap.add_argument("--port", type=int, default=UDP_PORT, help="UDP 수신 포트")
    ap.add_argument("--name", default=BLE_NAME, help="BLE 장치 이름")
    ap.add_argument("--http-port", type=int, default=8000, help="웹 UI 포트")
    args = ap.parse_args()
    MODE, UDP_PORT, BLE_NAME = args.mode, args.port, args.name

    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=args.http_port, log_level="info")


if __name__ == "__main__":
    main()
