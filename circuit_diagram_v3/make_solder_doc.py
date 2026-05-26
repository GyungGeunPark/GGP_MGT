#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
option_b_bottom_solder_side_v4.jpg 와 동일한 좌표 모델(PLACE/NETS)로부터
텍스트 납땜배선도(option_b_solder_wiring_v4.md)를 생성한다.
→ 이미지와 텍스트가 항상 일치(단일 소스).
"""
import os
import generate_v4_images as g   # PLACE, NETS, NCOL, NROW, ROWS 재사용

OUT = os.path.dirname(os.path.abspath(__file__))
PLACE, NETS, NCOL, NROW, ROWS = g.PLACE, g.NETS, g.NCOL, g.NROW, g.ROWS

# 부품 핀 → 사람이 읽는 이름
PIN_NAME = {
    "BATp": "BAT(+) 홀더 빨강", "BATn": "BAT(−) 홀더 검정",
    "POL_VIN": "Pololu VIN", "POL_GNDi": "Pololu GND(in)",
    "POL_VOUT": "Pololu VOUT", "POL_GNDo": "Pololu GND(out)",
    "R1t": "R1 100k 상단(=BAT+측)", "JUNC": "분배점 R1/R2(=ADC)", "R2b": "R2 100k 하단(=GND)",
    "C3t": "C3 ADC필터(+측=분배점)", "C3b": "C3 ADC필터(−측=GND)",
    "C1p": "C1 100µF (+)", "C1n": "C1 100µF (−)",
    "C2a": "C2 100nF", "C2b": "C2 100nF",
    "C4p": "C4 470µF (+)", "C4n": "C4 470µF (−)",
    "C5p": "C5 100~220µF (+)", "C5n": "C5 100~220µF (−)",
    "SEN_VCC": "센서 VCC", "SEN_GND": "센서 GND", "SEN_TX": "센서 TX",
    "SEN_RX": "센서 RX", "SEN_EN": "센서 EN",
    "E_5V": "ESP 5V(미사용)", "E_GND": "ESP GND", "E_3V3": "ESP 3V3", "E_G4": "ESP IO4",
    "E_G3": "ESP IO3(ADC)", "E_G2": "ESP IO2(미사용·스트래핑)", "E_G1": "ESP IO1", "E_G0": "ESP IO0",
    "E_G5": "ESP IO5", "E_G6": "ESP IO6", "E_G7": "ESP IO7", "E_G8": "ESP IO8(LED)",
    "E_G9": "ESP IO9", "E_G10": "ESP IO10(→센서EN)", "E_G20": "ESP IO20(RX)", "E_G21": "ESP IO21(TX)",
}
COORD2NAME = {v: PIN_NAME.get(k, k) for k, v in PLACE.items()}

NET_META = {  # net: (설명, 색, 게이지, 그리드기호)
    "GND":  ("GND 공통 버스 (행 S)",          "검정", "24AWG", "G"),
    "V33":  ("3.3V 전원 버스 (행 C)",          "노랑", "24AWG", "V"),
    "BATp": ("BAT(+) → Pololu VIN (행 B)",     "빨강", "24AWG", "B"),
    "ADC":  ("분배점 → ESP IO3 (ADC)",         "보라", "26AWG", "A"),
    "UTX":  ("센서 TX → ESP IO20 (RX) [크로스]", "초록", "26AWG", "T"),
    "URX":  ("ESP IO21 (TX) → 센서 RX [크로스]", "파랑", "26AWG", "R"),
    "EN":   ("ESP IO10 → 센서 EN (선택)",       "회색", "26AWG", "E"),
}


def ridx(r):
    return ROWS.index(r)


def build_grid():
    """직교 세그먼트를 채워 ASCII 배선 경로 그리드 생성(논리 좌표)."""
    grid = [["·"] * (NCOL + 1) for _ in range(NROW)]  # grid[row][col], col 1..NCOL
    nodes = set(PLACE.values())
    for name, (_c, _lw, segs) in NETS.items():
        ch = NET_META[name][3]
        for seg in segs:
            for (c1, r1), (c2, r2) in zip(seg, seg[1:]):
                if c1 == c2:  # 수직
                    for rr in range(min(ridx(r1), ridx(r2)), max(ridx(r1), ridx(r2)) + 1):
                        cur = grid[rr][c1]
                        grid[rr][c1] = ch if cur in ("·", ch) else "+"
                elif r1 == r2:  # 수평
                    for cc in range(min(c1, c2), max(c1, c2) + 1):
                        cur = grid[ridx(r1)][cc]
                        grid[ridx(r1)][cc] = ch if cur in ("·", ch) else "+"
    # 부품 핀 노드는 대문자 'O'로 강조(빈 곳일 때만, 라우팅 위는 유지)
    for (c, r) in nodes:
        if grid[ridx(r)][c] == "·":
            grid[ridx(r)][c] = "o"
    return grid


def render_grid_mirror(grid):
    """미러 뷰: 좌→우 = 논리 col 27→1."""
    lines = []
    header = "행\\열 " + "".join(f"{c:>2}" for c in range(NCOL, 0, -1))
    lines.append(header)
    for ri, r in enumerate(ROWS):
        cells = "".join(f"{grid[ri][c]:>2}" for c in range(NCOL, 0, -1))
        lines.append(f"  {r}   {cells}")
    return "\n".join(lines)


def seg_str(seg):
    return " → ".join(f"{c}{r}" for (c, r) in seg)


def main():
    grid = build_grid()
    grid_txt = render_grid_mirror(grid)

    # 네트별 세그먼트 표
    net_rows = []
    for name, (desc, color, gauge, ch) in NET_META.items():
        segs = NETS[name][2]
        seg_lines = []
        for seg in segs:
            a = COORD2NAME.get(seg[0], f"{seg[0][0]}{seg[0][1]}")
            b = COORD2NAME.get(seg[-1], f"{seg[-1][0]}{seg[-1][1]}")
            seg_lines.append(f"`{seg_str(seg)}`  ({a} ↔ {b})")
        net_rows.append((name, ch, desc, color, gauge, seg_lines))

    # 부품 핀 좌표 표 (논리 좌표 + 미러 좌표)
    pin_rows = []
    for k in PLACE:
        c, r = PLACE[k]
        mc = NCOL + 1 - c
        pin_rows.append((PIN_NAME.get(k, k), f"{c}{r}", f"{mc}{r}"))
    pin_rows.sort(key=lambda x: (x[0]))

    md = []
    md.append("# Option B v4 — 납땜 배선도 (텍스트판 / Solder Side)")
    md.append("")
    md.append("> `option_b_bottom_solder_side_v4.jpg` 와 **동일한 좌표 모델**에서 자동 생성.")
    md.append("> 기판 70×50mm · 열 1~27(좌→우) · 행 A~S(상→하) · 2.54mm 피치.")
    md.append("> **하면(Solder Side)은 미러**: 기판을 뒤집으면 열 순서가 좌우 반전(27→1)됩니다.")
    md.append("> 생성: `python3 make_solder_doc.py`")
    md.append("")
    md.append("---")
    md.append("")
    md.append("## 1. 네트(net) 배선표")
    md.append("")
    md.append("각 와이어를 **상면 논리 좌표**(열행, 예: `19K`)로 표기. 하면 작업 시 좌우 반전 주의.")
    md.append("")
    for name, ch, desc, color, gauge, seg_lines in net_rows:
        md.append(f"### [{ch}] {desc}  —  {color} {gauge}")
        for sl in seg_lines:
            md.append(f"- {sl}")
        md.append("")
    md.append("---")
    md.append("")
    md.append("## 2. 배선 경로 그리드 (하면 미러 뷰)")
    md.append("")
    md.append("기호: `G`=GND · `V`=3.3V · `B`=BAT+ · `A`=ADC · `T`=UART RX(센서TX) · "
              "`R`=UART TX(센서RX) · `E`=EN · `+`=배선 교차 · `o`=부품 핀(미배선) · `·`=빈 홀")
    md.append("")
    md.append("```")
    md.append(grid_txt)
    md.append("```")
    md.append("")
    md.append("> ⚠ 교차(`+`) 지점은 **서로 다른 네트가 겹쳐 보이는 것**일 뿐, 전기적으로 연결하면 안 됩니다.")
    md.append("> 하면에서 절연(점퍼 띄우기 또는 피복선)으로 분리하세요.")
    md.append("")
    md.append("---")
    md.append("")
    md.append("## 3. 부품 핀 ↔ 홀 좌표")
    md.append("")
    md.append("| 부품 핀 | 상면 좌표 | 하면(미러) 좌표 |")
    md.append("|---------|-----------|-----------------|")
    for nm, lc, mc in pin_rows:
        md.append(f"| {nm} | `{lc}` | `{mc}` |")
    md.append("")
    md.append("---")
    md.append("")
    md.append("## 4. 납땜 순서 (하면, v4)")
    md.append("")
    md.append("```")
    md.append("① GND 버스 (검정 24AWG) — 가장 먼저")
    md.append("   행 S를 따라 col3~col25 메인 버스, 각 GND 핀으로 수직 스터브")
    md.append("   (BAT−, Pololu GND×2, ESP GND, R2−, C3−, C1−, C2−, C4−, C5−)")
    md.append("② 3.3V 버스 (노랑 24AWG)")
    md.append("   행 C를 따라 버스, Pololu VOUT→ESP 3V3·센서VCC·C1+·C4+")
    md.append("③ BAT+ (빨강 24AWG)")
    md.append("   행 B: 홀더(+) → Pololu VIN, 분기 C5+")
    md.append("④ 전압분배 출력 (보라 26AWG) — 분배점 → ESP IO3")
    md.append("⑤ UART 크로스 (초록/파랑 26AWG)")
    md.append("   센서TX→IO20(RX) [초록] / IO21(TX)→센서RX [파랑]  ← 반드시 크로스!")
    md.append("⑥ EN 제어 (회색 26AWG, 선택) — ESP IO10 → 센서 EN")
    md.append("⑦ 전체 점검 — 쇼트·도통·콜드솔더 / IPA 플럭스 세척")
    md.append("```")
    md.append("")
    md.append("---")
    md.append("")
    md.append("## 5. 통전 전 도통/검증 체크리스트")
    md.append("")
    md.append("- [ ] **전원 단락 확인**: 3.3V 버스 ↔ GND 버스 저항이 수십 kΩ 이상(쇼트 아님)")
    md.append("- [ ] **극성**: C1·C4·C5 전해 (+)/(−) 방향, C3는 세라믹(무극성)")
    md.append("- [ ] **분배기**: BAT+ → R1 → 분배점 → R2 → GND 도통, 분배점 = ESP IO3")
    md.append("- [ ] **UART 크로스**: 센서TX 홀 ↔ ESP IO20, ESP IO21 ↔ 센서RX 홀 (절대 직결 금지)")
    md.append("- [ ] **EN**: ESP IO10 ↔ 센서 EN (GPIO2 아님 확인)")
    md.append("- [ ] **SHDN**: Pololu SHDN 핀은 미연결(NC) 유지")
    md.append("- [ ] **전원 순서**: 스위치 ON → VOUT=3.3V±0.1V 확인 → 그 후 ESP·센서 장착")
    md.append("- [ ] **핀맵 도통**: ESP 각 홀이 `ESP32_C3_pinmap.png`와 1:1 (특히 IO3/10/20/21)")
    md.append("")
    md.append("---")
    md.append("")
    md.append("## 6. 와이어 색상 규칙")
    md.append("")
    md.append("| 기호 | 색 | 용도 | 게이지 |")
    md.append("|------|-----|------|--------|")
    for name, ch, desc, color, gauge, _ in net_rows:
        md.append(f"| `{ch}` | {color} | {desc} | {gauge} |")
    md.append("")
    md.append("*생성: `make_solder_doc.py` — 좌표/네트 변경 시 재실행하면 이미지와 자동 동기화.*")

    path = os.path.join(OUT, "option_b_solder_wiring_v4.md")
    with open(path, "w", encoding="utf-8") as f:
        f.write("\n".join(md) + "\n")
    print("wrote", path)


if __name__ == "__main__":
    main()
