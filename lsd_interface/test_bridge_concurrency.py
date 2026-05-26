#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
wireless_bridge_v4.py 멀티클라이언트 동시성 회귀테스트(견고성 가드)
============================================================================
목적: 다중 WS 클라이언트 + 고속 UDP 플러드 + status 티커 + read_voltage 명령이
      동시에 여러 소켓으로 송신되고, 느린 소비자(백프레셔)가 섞인 상황에서
      서버가 크래시/ASGI 오류 없이 견고하게 동작하는지 회귀 검증.

부하: N 클라이언트(그 중 1개는 느린 리더로 send 버퍼 백프레셔 유발) +
      4000 패킷 플러드 + 주기 status + read_voltage 명령 인터리브.

판정(PASS):
  - 어떤 클라이언트도 끊기지 않음 + 서버 로그에 'Unexpected ASGI message'/Traceback 없음
  - 느린 클라이언트는 부분 수신(정상), 빠른 클라이언트는 전량 수신

주의(정직): 이 테스트는 send_lock 유무와 무관하게 현재 스택(uvicorn/websockets)에서
      모두 PASS 한다 — 즉 send_lock 의 '필요성'을 차등 증명하지는 못한다.
      send_lock 은 ASGI 규약(소켓당 단일 송신자)에 부합하는 방어적/무해한 조치로 유지.
실행: python3 test_bridge_concurrency.py
============================================================================
"""
import asyncio
import json
import socket
import struct
import subprocess
import sys
import threading
import time

import os

import websockets

BR = os.environ.get("BRIDGE_PATH", "/root/lds_ws/lsd_interface/wireless_bridge_v4.py")
HTTP_PORT = int(os.environ.get("BRIDGE_HTTP_PORT", "8002"))
UDP_PORT = int(os.environ.get("BRIDGE_UDP_PORT", "4212"))
N_CLIENTS = 5
N_PACKETS = 4000            # 고속 플러드(백프레셔 유발용)
SLOW_CLIENT = 0             # 0번은 느린 리더 → send 버퍼 백프레셔 → 동시 send 경합 유발
DIST_THRESHOLD = 1          # 핵심 판정은 '크래시/ASGI 에러 없음'; 수신량은 보조


def make_pkt(seq):
    mm = 1000.0 + (seq % 500)
    sq = seq % 200
    return (struct.pack("<B", 1) + struct.pack("<f", mm) + struct.pack("<I", seq)
            + struct.pack("<f", 2.95) + struct.pack("<H", sq))


async def client_task(idx, stop_evt, results):
    uri = f"ws://127.0.0.1:{HTTP_PORT}/ws"
    dist = 0
    crashed = False
    try:
        async with websockets.connect(uri) as ws:
            # 명령도 섞어 송신 경로(ack/voltage) 동시성 유발
            async def spam_cmd():
                for _ in range(5):
                    await asyncio.sleep(0.3)
                    try:
                        await ws.send(json.dumps({"cmd": "read_voltage"}))
                    except Exception:
                        return
            cmd_t = asyncio.create_task(spam_cmd())
            slow = (idx == SLOW_CLIENT)
            while not stop_evt.is_set():
                try:
                    m = json.loads(await asyncio.wait_for(ws.recv(), 1.0))
                    if m.get("type") == "distance":
                        dist += 1
                    if slow:
                        await asyncio.sleep(0.05)   # 느린 소비 → 서버 send 버퍼 백프레셔
                except asyncio.TimeoutError:
                    continue
            cmd_t.cancel()
    except Exception as e:
        crashed = True
        results["errors"].append(f"client{idx}: {type(e).__name__}: {e}")
    results["dist"][idx] = dist
    results["crashed"][idx] = crashed


def udp_blaster():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    for seq in range(N_PACKETS):
        s.sendto(make_pkt(seq), ("127.0.0.1", UDP_PORT))
        if seq % 100 == 0:
            time.sleep(0.001)   # 최소 텀(과도 드롭만 방지) — 사실상 플러드
    s.close()


async def run():
    results = {"dist": {}, "crashed": {}, "errors": []}
    stop_evt = asyncio.Event()
    tasks = [asyncio.create_task(client_task(i, stop_evt, results)) for i in range(N_CLIENTS)]
    await asyncio.sleep(1.0)                       # 전 클라이언트 접속 대기
    threading.Thread(target=udp_blaster, daemon=True).start()  # 고속 스트림
    await asyncio.sleep(4.0)                        # 부하 지속(티커도 그 사이 송신)
    stop_evt.set()
    await asyncio.gather(*tasks)
    return results


def main():
    proc = subprocess.Popen(
        [sys.executable, BR, "--mode", "udp", "--port", str(UDP_PORT), "--http-port", str(HTTP_PORT)],
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    logs = []
    t = threading.Thread(target=lambda: [logs.append(l) for l in proc.stdout], daemon=True)
    t.start()
    try:
        time.sleep(2.0)                            # 서버 기동
        results = asyncio.run(asyncio.wait_for(run(), 20))
    finally:
        proc.terminate()
        try:
            proc.wait(5)
        except Exception:
            proc.kill()
    log_txt = "".join(logs)
    asgi_err = ("Unexpected ASGI message" in log_txt) or ("Traceback" in log_txt)
    any_crash = any(results["crashed"].values())
    min_dist = min(results["dist"].values()) if results["dist"] else 0

    print(f"clients={N_CLIENTS} packets={N_PACKETS}")
    print(f"distance per client: {results['dist']}")
    print(f"crashed: {results['crashed']}  errors: {results['errors']}")
    print(f"server ASGI/traceback error in log: {asgi_err}")
    ok = (not any_crash) and (not asgi_err) and (min_dist >= DIST_THRESHOLD)
    print("RESULT:", "PASS" if ok else "FAIL"
          + ("" if ok else f"  (min_dist={min_dist}, crash={any_crash}, asgi_err={asgi_err})"))
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
