#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
wireless_bridge_v4.py 엔드투엔드 테스트 (단일 클라이언트, 하드웨어 없이)
서버 기동 → WS 접속 → 합성 UDP 15B 패킷 주입 → distance(SQ 포함) 수신 →
read_voltage 명령 왕복(battery_pct) 검증. PASS 시 exit 0, 실패 시 exit 1.
"""
import asyncio
import json
import os
import socket
import struct
import subprocess
import sys

import websockets

BR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "wireless_bridge_v4.py")
HTTP_PORT = int(os.environ.get("E2E_HTTP_PORT", "8007"))
UDP_PORT = int(os.environ.get("E2E_UDP_PORT", "4217"))


async def run():
    uri = f"ws://127.0.0.1:{HTTP_PORT}/ws"
    ws = None
    for _ in range(60):
        try:
            ws = await websockets.connect(uri); break
        except Exception:
            await asyncio.sleep(0.25)
    if ws is None:
        return "FAIL: 서버/ws 접속 실패"
    got = {"status": False, "distance_sq": False, "battery_pct": False}
    async with ws:
        first = json.loads(await asyncio.wait_for(ws.recv(), 5))
        got["status"] = (first.get("type") == "status")
        pkt = (struct.pack("<B", 1) + struct.pack("<f", 1234.5) + struct.pack("<I", 5000)
               + struct.pack("<f", 2.95) + struct.pack("<H", 79))
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.sendto(pkt, ("127.0.0.1", UDP_PORT)); s.close()
        for _ in range(25):
            m = json.loads(await asyncio.wait_for(ws.recv(), 5))
            if m.get("type") == "distance":
                assert m["success"] and m["mm"] == 1234 and m["signal_quality"] == 79, m
                got["distance_sq"] = True; break
        await ws.send(json.dumps({"cmd": "read_voltage"}))
        for _ in range(25):
            m = json.loads(await asyncio.wait_for(ws.recv(), 5))
            if m.get("type") == "voltage":
                assert m["success"] and m["voltage_mv"] == 2950 and m["battery_pct"] == 83, m
                got["battery_pct"] = True; break
    return "PASS" if all(got.values()) else f"FAIL: {got}"


def main():
    proc = subprocess.Popen(
        [sys.executable, BR, "--mode", "udp", "--port", str(UDP_PORT), "--http-port", str(HTTP_PORT)],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    try:
        res = asyncio.run(asyncio.wait_for(run(), 35))
    finally:
        proc.terminate()
        try:
            proc.wait(5)
        except Exception:
            proc.kill()
    print("RESULT:", res)
    sys.exit(0 if res == "PASS" else 1)


if __name__ == "__main__":
    main()
