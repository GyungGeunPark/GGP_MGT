#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""펌웨어↔수신기 15바이트 패킷 계약 회귀테스트 (pc_receiver_v4.parse_packet)."""
import importlib.util
import os
import struct
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
spec = importlib.util.spec_from_file_location("rx", os.path.join(HERE, "pc_receiver_v4.py"))
rx = importlib.util.module_from_spec(spec)
spec.loader.exec_module(rx)


def make(mm, ts, batt, sq):
    return (struct.pack("<B", 1) + struct.pack("<f", mm) + struct.pack("<I", ts)
            + struct.pack("<f", batt) + struct.pack("<H", sq))


def main():
    assert rx.PACKET_SIZE == 15, f"PACKET_SIZE={rx.PACKET_SIZE}"
    pkt = make(1234.5, 5000, 2.95, 79)
    r = rx.parse_packet(pkt)
    assert r and r["id"] == 1 and abs(r["distance_mm"] - 1234.5) < 1e-3 \
        and r["esp_ts_ms"] == 5000 and abs(r["battery_v"] - 2.95) < 1e-3 and r["sq"] == 79, r
    # SQ 미상 센티넬 → -1
    assert rx.parse_packet(make(1.0, 1, 3.0, 0xFFFF))["sq"] == -1
    # 잘못된 크기/ID 거부
    assert rx.parse_packet(pkt[:14]) is None
    assert rx.parse_packet(b"\x02" + pkt[1:]) is None
    print("RESULT: PASS (packet 15B '<BfIfH' + SQ + reject-bad)")


if __name__ == "__main__":
    try:
        main()
    except AssertionError as e:
        print("RESULT: FAIL —", e)
        sys.exit(1)
