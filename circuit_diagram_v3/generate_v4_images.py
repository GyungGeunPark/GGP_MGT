#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Option B v4 회로 다이어그램 생성기 (정밀판)
v3 리뷰 개선사항(option_b_v3_리뷰_개선사항.md) 반영. 3종 이미지를 렌더링한다.
  1) option_b_schematic_v4.jpg            전체 회로도
  2) option_b_top_component_side_v4.jpg   상면 부품 배치도 (실제 핀맵·풋프린트)
  3) option_b_bottom_solder_side_v4.jpg   하면 납땜 배선도 (직교 라우팅, 미러)

단일 좌표 모델(PLACE/NETS)을 상면·하면이 공유 → 두 도면이 항상 일치.
matplotlib + NanumGothic (µ/− 글리프는 DejaVu Sans 폴백).
"""
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.font_manager as fm
from matplotlib.patches import FancyBboxPatch, Rectangle, Circle
from matplotlib.lines import Line2D
import os

OUT = os.path.dirname(os.path.abspath(__file__))
FONT = "/usr/share/fonts/truetype/nanum/NanumGothic.ttf"
fm.fontManager.addfont(FONT)
FP = fm.FontProperties(fname=FONT)
plt.rcParams["font.family"] = [FP.get_name(), "DejaVu Sans"]
plt.rcParams["axes.unicode_minus"] = False

C_RED, C_BLACK, C_YELLOW = "#d62728", "#000000", "#e6b800"
C_GREEN, C_BLUE, C_PURPLE = "#2ca02c", "#1f6fd6", "#7a3fbf"
C_GRAY, C_NEW = "#888888", "#cc0000"
NCOL, NROW = 27, 19
ROWS = "ABCDEFGHIJKLMNOPQRS"

# ─────────────────────────────────────────────────────────────────────
# 단일 좌표 모델 — 모든 부품 핀의 (열, 행)
# ─────────────────────────────────────────────────────────────────────
PLACE = {
    "BATp": (2, "B"), "BATn": (4, "B"),
    "POL_VIN": (9, "B"), "POL_GNDi": (9, "D"), "POL_VOUT": (13, "B"), "POL_GNDo": (13, "D"),
    "R1t": (19, "G"), "JUNC": (19, "K"), "R2b": (19, "N"),
    "C3t": (21, "K"), "C3b": (21, "N"),               # ADC 필터(세라믹)
    "C1p": (3, "R"), "C1n": (3, "S"),                 # 센서 디커플링 100µF
    "C2a": (6, "R"), "C2b": (6, "S"),                 # 100nF
    "C4p": (25, "Q"), "C4n": (25, "S"),               # 470µF 벌크
    "C5p": (7, "C"), "C5n": (7, "E"),                 # VIN 벌크 (Pololu 좌측)
    "SEN_VCC": (10, "S"), "SEN_GND": (11, "S"),
    "SEN_TX": (12, "S"), "SEN_RX": (13, "S"), "SEN_EN": (14, "S"),
    # ESP32-C3 (USB 상단). 왼쪽 col6 / 오른쪽 col13, 행 I~P
    "E_5V": (6, "I"), "E_GND": (6, "J"), "E_3V3": (6, "K"), "E_G4": (6, "L"),
    "E_G3": (6, "M"), "E_G2": (6, "N"), "E_G1": (6, "O"), "E_G0": (6, "P"),
    "E_G5": (13, "I"), "E_G6": (13, "J"), "E_G7": (13, "K"), "E_G8": (13, "L"),
    "E_G9": (13, "M"), "E_G10": (13, "N"), "E_G20": (13, "O"), "E_G21": (13, "P"),
}

# 네트(net): (색, 굵기, [세그먼트...]). 각 세그먼트는 직교 경유점 목록
# (연속 점은 같은 행 또는 같은 열 공유 → 대각선 없음).
NETS = {
    "GND":  (C_BLACK, 2.6, [
        [(3, "S"), (25, "S")],                    # 메인 버스(행 S)
        [(4, "S"), (4, "B")],                     # BAT−
        [(9, "S"), (9, "D")], [(13, "S"), (13, "D")],  # Pololu GND
        [(6, "S"), (6, "J")],                     # ESP32 GND
        [(19, "S"), (19, "N")], [(21, "S"), (21, "N")],  # R2−, C3−
        [(7, "S"), (7, "E")]]),                   # C5−
    "V33":  (C_YELLOW, 2.6, [
        [(3, "C"), (25, "C")],                    # 3.3V 버스(행 C)
        [(13, "B"), (13, "C")],                   # VOUT→버스
        [(6, "C"), (6, "K")],                     # ESP32 3V3
        [(10, "C"), (10, "S")],                   # 센서 VCC
        [(3, "C"), (3, "R")],                     # C1+
        [(25, "C"), (25, "Q")]]),                 # C4+
    "BATp": (C_RED, 2.6, [
        [(2, "B"), (9, "B")],                     # BAT+→Pololu VIN
        [(7, "B"), (7, "C")]]),                   # C5+
    "ADC":  (C_PURPLE, 2.2, [
        [(19, "K"), (19, "F"), (5, "F"), (5, "M"), (6, "M")]]),  # JUNC→IO3
    "UTX":  (C_GREEN, 2.2, [[(12, "S"), (12, "Q"), (13, "Q"), (13, "O")]]),  # 센서TX→IO20
    "URX":  (C_BLUE, 2.2, [[(13, "P"), (13, "S")]]),            # IO21→센서RX
    "EN":   (C_GRAY, 2.0, [[(13, "N"), (14, "N"), (14, "S")]]),  # IO10→센서EN
}


def newfig(w, h):
    fig, ax = plt.subplots(figsize=(w, h), dpi=115)
    ax.set_xlim(0, 100); ax.set_ylim(0, 100); ax.axis("off")
    return fig, ax


def perfboard(ax, x0, y0, w, h, mirror=False):
    ax.add_patch(Rectangle((x0, y0), w, h, fc="#cbb89e", ec="#7a6a4f", lw=2, zorder=0))
    dx = w / (NCOL + 1); dy = h / (NROW + 1)
    coords = {}
    for c in range(1, NCOL + 1):
        cc = (NCOL + 1 - c) if mirror else c
        cx = x0 + c * dx
        if cc % 2 == 1:
            ax.text(cx, y0 + h + 0.8, str(cc), fontsize=5.0, ha="center", color="#555")
        for ri, r in enumerate(ROWS):
            cy = y0 + h - (ri + 1) * dy
            ax.add_patch(Circle((cx, cy), min(dx, dy) * 0.15, fc="#5b4d36", ec="none", zorder=1))
            coords[(cc, r)] = (cx, cy)
    for ri, r in enumerate(ROWS):
        cy = y0 + h - (ri + 1) * dy
        ax.text(x0 - 0.8, cy, r, fontsize=5.2, va="center", ha="right", color="#555")
    return coords, dx, dy


def xy(coords, key):
    return coords[PLACE[key]]


def hole(ax, p, color, r=0.55, ring=False):
    ax.add_patch(Circle(p, r, fc=color if not ring else "white", ec=color, lw=1.4, zorder=6))


def lead(ax, p1, p2, color="#777"):
    ax.add_line(Line2D([p1[0], p2[0]], [p1[1], p2[1]], color=color, lw=1.3, zorder=4))


# ════════════════════════ 1) 전체 회로도 ════════════════════════
def box(ax, x, y, w, h, edge, title, lines, fc="white", tfs=10, lfs=8.0, tcol=None, lw=1.6):
    ax.add_patch(FancyBboxPatch((x, y), w, h, boxstyle="round,pad=0.15,rounding_size=0.8",
                 ec=edge, fc=fc, lw=lw, zorder=2))
    cy = y + h - 2.2
    ax.text(x + 1.2, cy, title, fontsize=tfs, fontweight="bold", color=tcol or edge,
            va="top", ha="left", zorder=3)
    cy -= (tfs * 0.20 + 1.6)
    for ln in lines:
        col, fw = ("black", "normal")
        if ln.startswith("★") or ln.startswith("⚠"):
            col, fw = C_NEW, "bold"
        ax.text(x + 1.2, cy, ln, fontsize=lfs, color=col, fontweight=fw, va="top", zorder=3)
        cy -= (lfs * 0.205 + 1.05)


def wire(ax, pts, color, lw=2.4):
    ax.add_line(Line2D([p[0] for p in pts], [p[1] for p in pts], color=color, lw=lw,
                zorder=1, solid_capstyle="round"))


def schematic():
    fig, ax = newfig(13.2, 8.7)
    ax.text(1.5, 99, "Option B: 2×AA 전체 회로도 (Schematic) v4", fontsize=15,
            fontweight="bold", va="top", color="#222")
    ax.text(1.5, 95.0, "JRT U81 + ESP32-C3 Super Mini + Pololu S7V8F3   |   v3 리뷰 개선사항 반영",
            fontsize=8.5, va="top", color="#555")
    ax.add_patch(Rectangle((72, 92.5), 28, 7.5, fc=C_NEW, ec=C_NEW, zorder=2))
    for i, t in enumerate(["v4 변경사항", "ADC필터 세라믹화·분배 100k",
                           "저전압 2.9/2.8V·EN→GPIO10", "벌크캡 추가·SHDN=NC 명시"]):
        ax.text(73, 99 - i * 2.4, t, color="white", fontsize=8.4 if i == 0 else 7.4,
                fontweight="bold" if i == 0 else "normal", va="top")

    box(ax, 2, 74, 19, 15, C_RED, "2×AA 건전지 홀더",
        ["3.0V / 2500mAh", "알카라인", "내장 ON/OFF 스위치", "★ 역극성 통전 전 확인"])
    box(ax, 25, 73, 21, 17, C_BLUE, "Pololu S7V8F3",
        ["Buck-Boost 컨버터", "입력: 2.7~11.8V", "출력: 3.3V 고정 / 최대 1A",
         "효율 >90%", "★ SHDN = NC (상시 ON)"])
    box(ax, 55, 47, 23, 43, "#1a9e8f", "ESP32-C3 Super Mini", ["(핀소켓 탈착식)"], fc="#eafaf6")
    for i, p in enumerate(["GND", "3V3", "GPIO10", "GPIO9", "GPIO8", "GPIO7",
                           "GPIO6", "GPIO5", "GPIO21", "GPIO20"]):
        py = 84 - i * 3.5
        col, fw = "#444", "normal"
        if p == "GPIO20": col, fw = C_GREEN, "bold"
        if p == "GPIO21": col, fw = C_BLUE, "bold"
        if p == "GPIO8": col, fw = C_YELLOW, "bold"
        if p == "GPIO10": col, fw = C_NEW, "bold"
        ax.add_patch(Circle((78, py), 0.4, fc="#666", zorder=4))
        ax.text(79.2, py, p, fontsize=7.4, va="center", color=col, fontweight=fw)
    ax.text(54.7, 70, "GPIO2", fontsize=7.4, va="center", ha="right", color="#999")
    ax.add_patch(Circle((55, 70), 0.4, fc="#666", zorder=4))
    ax.text(54.7, 66, "GPIO3", fontsize=7.4, va="center", ha="right", color=C_PURPLE, fontweight="bold")
    ax.add_patch(Circle((55, 66), 0.4, fc="#666", zorder=4))

    box(ax, 40, 27, 21, 16, C_RED, "JRT U81 센서",
        ["레이저 거리 측정 모듈", "3.3V UART, 19200 bps", "범위: 0.03~20m, ±1mm"])
    box(ax, 2, 30, 24, 30, C_PURPLE, "전압분배기 (배터리 모니터링)",
        ["BAT(+)─[R1: 100kΩ 1/2W ±1%]─┬→ GPIO3", "                            │",
         "          [R2: 100kΩ 1/2W ±1%]", "                            │",
         "★ ADC필터: 100nF~1µF 세라믹", "   (무극성·저누설)",
         "                            │", "                           GND",
         "Vadc = Vbat × 0.5  (3.0V→1.50V)", "★ 전해 100µF 사용 금지(τ·누설 오차)"])
    box(ax, 84.5, 58, 15.2, 17, C_RED, "⚠ UART 크로스 !!",
        ["센서 TX → GPIO20(RX)", "GPIO21(TX) → 센서 RX",
         "19200 bps · 8N1", "미연결 시 통신 불가"], tfs=8.2, lfs=7.0)
    box(ax, 84.5, 38, 15.2, 17, C_YELLOW, "디커플링 캐패시터",
        ["100µF 전해 + 100nF", "세라믹 (센서 VCC-GND,", " 5mm 이내)",
         "★ 센서 모듈 끝단에도", "   100nF 추가 (원격)"], tfs=8.2, lfs=7.0, tcol="#a07d00")
    box(ax, 2, 62, 46, 9.5, C_NEW, "★ v4 전원 안정화 (WiFi 돌입전류 대비)",
        ["3.3V 버스에 470µF 벌크캡 + Pololu VIN에 100~220µF 추가.",
         "알카라인 ESR 상승 시 WiFi 피크에서 브라운아웃/리셋 방지."], tfs=9, lfs=7.8, tcol=C_NEW)
    box(ax, 28, 6, 44, 18, C_NEW, "★ v3 → v4 핵심 변경",
        ["1. ADC 필터: 100µF 전해 → 100nF~1µF 세라믹",
         "2. 전압분배: 220k/220k → 100k/100k (소스 임피던스↓)",
         "3. 저전압: 2.2V → 2.9V경고/2.8V종료 (UVLO 2.7V)",
         "4. 센서 EN: GPIO2(스트래핑) → GPIO10",
         "5. 3.3V 470µF + VIN 100~220µF 벌크캡 추가",
         "6. ADC eFuse 캘리브레이션 / SHDN=NC 명시"], tfs=9.5, lfs=8.0, tcol=C_NEW)

    wire(ax, [(21, 84), (25, 84)], C_RED, 2.6); ax.text(22, 85.4, "BAT+", fontsize=6.6, color=C_RED)
    wire(ax, [(21, 77), (25, 77)], C_BLACK, 2.6)
    wire(ax, [(46, 86), (52, 86), (52, 80.5), (78, 80.5)], C_YELLOW, 2.6)
    wire(ax, [(52, 80.5), (52, 39), (60, 39)], C_YELLOW, 2.6)
    ax.add_patch(Circle((52, 80.5), 0.5, fc=C_YELLOW, zorder=4))
    ax.text(53, 88, "3.3V (VOUT)", fontsize=6.6, color="#a07d00")
    wire(ax, [(26, 45), (50, 45), (50, 66), (55, 66)], C_PURPLE, 2.4)
    wire(ax, [(61, 33), (74, 33), (74, 52.5), (78, 52.5)], C_GREEN, 2.4)
    ax.text(62.5, 34.2, "센서 TX → GPIO20", fontsize=6.4, color=C_GREEN)
    wire(ax, [(78, 56), (76, 56), (76, 30), (61, 30)], C_BLUE, 2.4)
    ax.text(62.5, 28.8, "GPIO21 → 센서 RX", fontsize=6.4, color=C_BLUE)

    items = [(C_RED, "빨강 — BAT(+)"), (C_BLACK, "검정 — GND 공통"),
             (C_YELLOW, "노랑 — 3.3V 버스"), (C_GREEN, "초록 — UART RX(센서TX→G20)"),
             (C_BLUE, "파랑 — UART TX(G21→센서RX)"), (C_PURPLE, "보라 — ADC(분배점→G3)")]
    ax.text(2, 5.4, "■ 배선 색상 범례:", fontsize=9, fontweight="bold", va="top")
    for i, (c, t) in enumerate(items):
        cx = 2 if i < 3 else 52; ry = 2.6 - (i % 3) * 1.9
        ax.add_patch(Rectangle((cx, ry - 0.6), 2.2, 1.3, fc=c, ec="black", lw=0.5, zorder=3))
        ax.text(cx + 3.0, ry, t, fontsize=7.6, va="center")
    fig.savefig(os.path.join(OUT, "option_b_schematic_v4.jpg"),
                bbox_inches="tight", pad_inches=0.15, facecolor="white")
    plt.close(fig); print("saved schematic_v4")


# ════════════════════════ 부품 풋프린트 ════════════════════════
def draw_resistor(ax, coords, k1, k2, label):
    p1, p2 = xy(coords, k1), xy(coords, k2)
    cx, cy = (p1[0] + p2[0]) / 2, (p1[1] + p2[1]) / 2
    ax.add_patch(FancyBboxPatch((cx - 0.9, min(p1[1], p2[1]) + 0.7),
                 1.8, abs(p2[1] - p1[1]) - 1.4, boxstyle="round,pad=0.05,rounding_size=0.3",
                 fc="#f1d873", ec="#9a7b10", lw=1.2, zorder=5))
    lead(ax, p1, (cx, min(p1[1], p2[1]) + 0.7)); lead(ax, (cx, max(p1[1], p2[1]) - 0.7), p2)
    hole(ax, p1, "#9a7b10", ring=True); hole(ax, p2, "#9a7b10", ring=True)
    ax.text(cx + 1.4, cy, label, fontsize=5.6, va="center", color="#5a4500", fontweight="bold")


def draw_elec_cap(ax, coords, kp, kn, label, color=C_RED, lab="right"):
    pp, pn = xy(coords, kp), xy(coords, kn)
    cx, cy = (pp[0] + pn[0]) / 2, (pp[1] + pn[1]) / 2
    ax.add_patch(Circle((cx, cy), 1.4, fc="#e9a3a3", ec=color, lw=1.4, zorder=5))
    ax.text(cx, cy, "+", fontsize=8, ha="center", va="center", color=color, fontweight="bold", zorder=6)
    lead(ax, pp, (cx, cy), color); lead(ax, pn, (cx, cy), color)
    hole(ax, pp, color); hole(ax, pn, "#333", ring=True)
    if lab == "below":
        ax.text(cx, min(pp[1], pn[1]) - 2.0, label, fontsize=5.2, ha="center", color=color, fontweight="bold")
    else:
        ax.text(cx + 1.8, cy, label, fontsize=5.4, va="center", color=color, fontweight="bold")
    ax.text(pn[0] - 1.4, pn[1], "−", fontsize=7, ha="center", va="center", color="#333")


def draw_cer_cap(ax, coords, k1, k2, label, color=C_GREEN):
    p1, p2 = xy(coords, k1), xy(coords, k2)
    cx, cy = (p1[0] + p2[0]) / 2, (p1[1] + p2[1]) / 2
    ax.add_patch(FancyBboxPatch((cx - 1.3, cy - 1.0), 2.6, 2.0,
                 boxstyle="round,pad=0.05,rounding_size=0.4", fc="#bfe6bf", ec=color, lw=1.3, zorder=5))
    lead(ax, p1, (cx, cy), color); lead(ax, p2, (cx, cy), color)
    hole(ax, p1, color, ring=True); hole(ax, p2, color, ring=True)
    ax.text(cx, cy - 2.4, label, fontsize=5.0, ha="center", color="#145214", fontweight="bold")


ESP_LEFT = [("E_5V", "5V", "#999"), ("E_GND", "GND", C_BLACK), ("E_3V3", "3V3", "#a07d00"),
            ("E_G4", "IO4", "#999"), ("E_G3", "IO3·ADC", C_PURPLE), ("E_G2", "IO2✗", "#bbb"),
            ("E_G1", "IO1", "#999"), ("E_G0", "IO0", "#999")]
ESP_RIGHT = [("E_G5", "IO5", "#999"), ("E_G6", "IO6", "#999"), ("E_G7", "IO7", "#999"),
             ("E_G8", "IO8·LED", "#a07d00"), ("E_G9", "IO9", "#999"), ("E_G10", "IO10·EN", C_NEW),
             ("E_G20", "IO20·RX", C_GREEN), ("E_G21", "IO21·TX", C_BLUE)]


def draw_esp32(ax, coords, label_side):
    pL, pP = xy(coords, "E_5V"), xy(coords, "E_G0")
    pR = xy(coords, "E_G5")
    xl, xr = min(pL[0], pR[0]), max(pL[0], pR[0])
    yt, yb = pL[1], pP[1]
    ax.add_patch(FancyBboxPatch((xl - 2.0, yb - 2.0), (xr - xl) + 4.0, (yt - yb) + 4.0,
                 boxstyle="round,pad=0.1,rounding_size=0.6", fc="#d8f3ec", ec="#1a9e8f",
                 lw=1.8, alpha=0.9, zorder=3))
    ax.text((xl + xr) / 2, yt + 3.0, "ESP32-C3 Super Mini (USB↑)", fontsize=6.4, ha="center",
            color="#0c6", fontweight="bold", zorder=6)
    for key, lab, col in ESP_LEFT + ESP_RIGHT:
        p = xy(coords, key); hole(ax, p, col if col != "#999" else "#777", ring=True)
        is_left = key in [k for k, _, _ in ESP_LEFT]
        # label_side: 'out' = 왼쪽핀은 왼쪽, 오른쪽핀은 오른쪽
        dirL = -1 if (is_left ^ (label_side == "mirror")) else 1
        ha = "right" if dirL < 0 else "left"
        ax.text(p[0] + dirL * 1.6, p[1], lab, fontsize=4.7, va="center", ha=ha,
                color=col, fontweight="bold" if col not in ("#999", "#bbb") else "normal")


def draw_pololu(ax, coords):
    a, b = xy(coords, "POL_VIN"), xy(coords, "POL_GNDo")
    xl, xr = min(a[0], b[0]), max(a[0], b[0]); yt, yb = a[1], b[1]
    ax.add_patch(FancyBboxPatch((xl - 1.6, yb - 1.6), (xr - xl) + 3.2, (yt - yb) + 3.2,
                 boxstyle="round,pad=0.1,rounding_size=0.5", fc="#cfe0f7", ec=C_BLUE, lw=1.7, zorder=3))
    ax.text((xl + xr) / 2, yt + 3.0, "Pololu S7V8F3", fontsize=6.2, ha="center",
            color=C_BLUE, fontweight="bold", zorder=6)
    for key, lab, col in [("POL_VIN", "VIN", C_RED), ("POL_GNDi", "GND", C_BLACK),
                          ("POL_VOUT", "VOUT", "#a07d00"), ("POL_GNDo", "GND", C_BLACK)]:
        p = xy(coords, key); hole(ax, p, col, ring=True)
        ax.text(p[0], p[1] + (1.0 if key in ("POL_VIN", "POL_VOUT") else -1.5), lab,
                fontsize=4.8, ha="center", color=col, fontweight="bold")
    ax.text((xl + xr) / 2, (yt + yb) / 2, "SHDN=NC\n(상시 ON)", fontsize=4.6, ha="center",
            va="center", color=C_NEW, fontweight="bold", zorder=6)


def draw_sensor_header(ax, coords):
    for key, lab, col in [("SEN_VCC", "VCC", "#a07d00"), ("SEN_GND", "GND", C_BLACK),
                          ("SEN_TX", "TX", C_GREEN), ("SEN_RX", "RX", C_BLUE), ("SEN_EN", "EN", C_GRAY)]:
        p = xy(coords, key); hole(ax, p, col)
        ax.text(p[0], p[1] - 1.6, lab, fontsize=4.8, ha="center", color=col, fontweight="bold")
    a = xy(coords, "SEN_VCC"); e = xy(coords, "SEN_EN")
    ax.text((a[0] + e[0]) / 2, a[1] + 1.6, "JRT U81 센서 5핀", fontsize=5.2, ha="center", color="#159e8f")


# ════════════════════════ 2) 상면 배치도 ════════════════════════
def top_component():
    fig, ax = newfig(14.2, 9.6)
    ax.text(1.5, 99, "Option B: 만능기판 배치도 (상면 / Component Side) v4", fontsize=14,
            fontweight="bold", va="top", color="#222")
    ax.text(1.5, 95.3, "기판 70×50mm · 실제 핀맵·풋프린트 반영 · v4 개선사항", fontsize=8.2, va="top", color="#555")
    ax.add_patch(Rectangle((72, 92.5), 28, 7.5, fc=C_NEW, zorder=2))
    for i, t in enumerate(["v4 배치 변경", "ADC필터=세라믹·EN→IO10", "R1·R2=100k·470µF 벌크"]):
        ax.text(73, 99 - i * 2.4, t, color="white", fontsize=8.2 if i == 0 else 7.2,
                fontweight="bold" if i == 0 else "normal", va="top")

    coords, dx, dy = perfboard(ax, 3, 40, 52, 50, mirror=False)
    draw_esp32(ax, coords, "out")
    draw_pololu(ax, coords)
    draw_resistor(ax, coords, "R1t", "JUNC", "R1 100k")
    draw_resistor(ax, coords, "JUNC", "R2b", "R2 100k")
    draw_cer_cap(ax, coords, "C3t", "C3b", "C3 ADC필터\n100nF~1µF", color=C_GREEN)
    draw_elec_cap(ax, coords, "C1p", "C1n", "C1 100µF")
    draw_cer_cap(ax, coords, "C2a", "C2b", "C2 100nF", color="#159e8f")
    draw_elec_cap(ax, coords, "C4p", "C4n", "C4 470µF", color=C_PURPLE)
    draw_elec_cap(ax, coords, "C5p", "C5n", "C5 100~220µF (VIN)", color=C_RED, lab="below")
    draw_sensor_header(ax, coords)
    for key, lab, col in [("BATp", "BAT+", C_RED), ("BATn", "BAT−", C_BLACK)]:
        p = xy(coords, key); hole(ax, p, col)
        ax.text(p[0], p[1] + 1.4, lab, fontsize=5.2, ha="center", color=col, fontweight="bold")
    jp = xy(coords, "JUNC"); ax.add_patch(Circle(jp, 0.7, fc=C_PURPLE, zorder=7))
    ax.text(jp[0] + 1.3, jp[1] + 1.4, "분배점→IO3", fontsize=5.0, color=C_PURPLE)

    lx = 58
    ax.text(lx, 92, "■ 부품 범례:", fontsize=9.5, fontweight="bold", va="top")
    leg = [(C_BLUE, "Pololu S7V8F3 (SHDN=NC)"), ("#1a9e8f", "ESP32-C3 (소켓, USB 상단)"),
           ("#f1d873", "R1·R2 = 100kΩ ±1% ★v4"), (C_GREEN, "C3 ADC필터=100nF~1µF 세라믹 ★v4"),
           (C_RED, "C1 100µF·C5 VIN벌크 (전해)"), ("#159e8f", "C2 100nF 세라믹"),
           (C_PURPLE, "C4 470µF 3.3V 벌크 ★v4"), (C_GRAY, "센서 5핀 헤더")]
    yy = 88.5
    for c, t in leg:
        ax.add_patch(Rectangle((lx, yy - 0.8), 2.4, 1.6, fc=c, ec="black", lw=0.5))
        ax.text(lx + 3.2, yy, t, fontsize=7.2, va="center")
        yy -= 2.9

    ax.text(lx, 62, "★ ESP32-C3 핀 할당 (v4):", fontsize=9.2, fontweight="bold", va="top", color=C_NEW)
    pins = [("IO20 (RX) ← 센서 TX  [우측 행O]", C_GREEN), ("IO21 (TX) → 센서 RX  [우측 행P]", C_BLUE),
            ("IO3 (ADC1) ← 분배점   [좌측 행M]", C_PURPLE), ("IO10 → 센서 EN ★변경 [우측 행N]", C_NEW),
            ("3V3 ← Pololu VOUT     [좌측 행K]", "#a07d00"), ("GND ← 공통 버스       [좌측 행J]", "black"),
            ("IO8 = 온보드 LED      [우측 행L]", "#a07d00"), ("IO2 = 미사용(스트래핑) ✗", "#aaa")]
    yy = 58.5
    for t, c in pins:
        ax.text(lx, yy, t, fontsize=7.3, va="top", color=c, fontweight="bold" if "★" in t else "normal")
        yy -= 2.6

    ax.text(lx, 36, "※ 주의:", fontsize=9, fontweight="bold", va="top")
    for i, n in enumerate(["1. C3는 세라믹(무극성). 전해 금지 — 누설 오차",
                           "2. Pololu SHDN 핀 미연결(NC). VIN/VOUT 실크 확인",
                           "3. 전해 C1·C4·C5 극성 확인 ((+)=긴 리드)",
                           "4. ESP32 USB가 상단(행 H쪽). 핀맵 도통 검증",
                           "5. 모든 부품 상면, 납땜은 하면에서 수행"]):
        ax.text(lx, 33 - i * 2.6, n, fontsize=7.2, va="top", color="#333")

    fig.savefig(os.path.join(OUT, "option_b_top_component_side_v4.jpg"),
                bbox_inches="tight", pad_inches=0.15, facecolor="white")
    plt.close(fig); print("saved top_component_v4")


# ════════════════════════ 3) 하면 납땜 배선도 ════════════════════════
def route(ax, coords, pts, color, lw):
    xs = [coords[p][0] for p in pts]; ys = [coords[p][1] for p in pts]
    ax.add_line(Line2D(xs, ys, color=color, lw=lw, zorder=5, solid_capstyle="round",
                solid_joinstyle="miter"))


def bottom_solder():
    fig, ax = newfig(14.2, 9.6)
    ax.text(1.5, 99, "Option B: 만능기판 납땜 배선도 (하면 / Solder Side) v4", fontsize=14,
            fontweight="bold", va="top", color="#222")
    ax.text(1.5, 95.3, "※ 미러 뷰(뒤집은 상태) · 직교 라우팅 · 실제 홀 경로 · v4 반영", fontsize=8.2, va="top", color="#555")
    ax.add_patch(Rectangle((72, 92.5), 28, 7.5, fc=C_NEW, zorder=2))
    for i, t in enumerate(["v4 배선 변경", "EN선→IO10·ADC필터 세라믹", "470µF 벌크 배선 추가"]):
        ax.text(73, 99 - i * 2.4, t, color="white", fontsize=8.2 if i == 0 else 7.2,
                fontweight="bold" if i == 0 else "normal", va="top")

    coords, dx, dy = perfboard(ax, 3, 40, 52, 50, mirror=True)
    # ESP32 핀 위치 가이드(미러) — 라벨만 흐리게
    for key, lab, col in ESP_LEFT + ESP_RIGHT:
        p = coords[PLACE[key]]
        if col not in ("#999", "#bbb"):
            ax.add_patch(Circle(p, 0.5, fc="white", ec=col, lw=1.0, zorder=4))
    # 네트 라우팅 (세그먼트별 직교 경로)
    for name, (color, lw, segs) in NETS.items():
        for seg in segs:
            route(ax, coords, seg, color, lw)
    # ESP32 사용 핀 라벨(미러, 작게)
    esp_lab = {"E_3V3": ("3V3", "#a07d00"), "E_GND": ("GND", C_BLACK),
               "E_G3": ("IO3", C_PURPLE), "E_G8": ("IO8", "#a07d00"),
               "E_G10": ("IO10", C_NEW), "E_G20": ("IO20", C_GREEN), "E_G21": ("IO21", C_BLUE)}
    for key, (lab, col) in esp_lab.items():
        p = coords[PLACE[key]]
        ax.text(p[0], p[1] + 0.9, lab, fontsize=4.2, ha="center", va="bottom",
                color=col, fontweight="bold", zorder=8)
    # 솔더 패드 — 각 부품 핀
    for key in PLACE:
        p = coords[PLACE[key]]
        ax.add_patch(Circle(p, 0.45, fc="#b08d57", ec="#5b4d36", lw=0.6, zorder=6))
    jp = coords[PLACE["JUNC"]]; ax.add_patch(Circle(jp, 0.7, fc=C_PURPLE, zorder=7))

    lx = 58
    ax.text(lx, 92, "■ 배선 색상 범례 (하면 와이어):", fontsize=9.2, fontweight="bold", va="top")
    leg = [(C_RED, "빨강 — BAT(+) → Pololu VIN"), (C_BLACK, "검정 — GND 공통 버스(행 S)"),
           (C_YELLOW, "노랑 — 3.3V 버스(행 C)"), (C_GREEN, "초록 — UART RX (센서TX→IO20)"),
           (C_BLUE, "파랑 — UART TX (IO21→센서RX)"), (C_PURPLE, "보라 — ADC (분배점→IO3)"),
           (C_GRAY, "회색 — EN (IO10→센서EN) ★v4")]
    yy = 88.5
    for c, t in leg:
        ax.add_patch(Rectangle((lx, yy - 0.8), 2.4, 1.6, fc=c, ec="black", lw=0.5))
        ax.text(lx + 3.2, yy, t, fontsize=7.2, va="center",
                color=C_NEW if "★" in t else "black", fontweight="bold" if "★" in t else "normal")
        yy -= 2.9

    ax.text(lx, 64, "★ 납땜 순서 (하면, v4):", fontsize=9.2, fontweight="bold", va="top", color=C_NEW)
    for i, s in enumerate(["① GND 버스(검정 24AWG) — 행 S 따라 먼저",
                           "② 3.3V 버스(노랑 24AWG) — 행 C, VOUT→ESP32·센서·벌크캡",
                           "③ BAT+(빨강 24AWG) — 행 B, 홀더(+)→VIN",
                           "④ 분배 출력(보라 26AWG) — 분배점→IO3",
                           "⑤ UART 크로스(초록/파랑 26AWG) — 반드시 크로스!",
                           "⑥ EN(회색 26AWG, 선택) — IO10→센서EN ★v4",
                           "⑦ 점검 — 쇼트·도통·콜드솔더 / IPA 세척"]):
        ax.text(lx, 60.5 - i * 2.6, s, fontsize=7.2, va="top",
                color=C_NEW if "★" in s else "#222")

    ax.text(lx, 40, "★ v4 배선 변경:", fontsize=8.8, fontweight="bold", va="top", color=C_NEW)
    for i, c in enumerate(["▣ EN: IO2(스트래핑)→IO10 이설",
                           "▣ ADC 필터: 전해→세라믹(무극성)",
                           "▣ 470µF 벌크 (+)→3.3V/(−)→GND 추가",
                           "▣ 분배 220k→100k"]):
        ax.text(lx, 37 - i * 2.5, c, fontsize=7.2, va="top", color="#222")

    ax.text(3, 36, "※ 납땜 팁:", fontsize=8.5, fontweight="bold", va="top", color="#333")
    for i, t in enumerate(["• 전원선 24AWG 실리콘(굵고 짧게), 신호선 26AWG",
                           "• UART 크로스: 센서TX→IO20(RX)/IO21(TX)→센서RX",
                           "• 전해 극성 확인, DuPont 점퍼 금지(진동)"]):
        ax.text(3, 33 - i * 2.5, t, fontsize=7.1, va="top", color="#444")

    fig.savefig(os.path.join(OUT, "option_b_bottom_solder_side_v4.jpg"),
                bbox_inches="tight", pad_inches=0.15, facecolor="white")
    plt.close(fig); print("saved bottom_solder_v4")


if __name__ == "__main__":
    schematic(); top_component(); bottom_solder(); print("ALL DONE")
