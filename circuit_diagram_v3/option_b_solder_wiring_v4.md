# Option B v4 — 납땜 배선도 (텍스트판 / Solder Side)

> `option_b_bottom_solder_side_v4.jpg` 와 **동일한 좌표 모델**에서 자동 생성.
> 기판 70×50mm · 열 1~27(좌→우) · 행 A~S(상→하) · 2.54mm 피치.
> **하면(Solder Side)은 미러**: 기판을 뒤집으면 열 순서가 좌우 반전(27→1)됩니다.
> 생성: `python3 make_solder_doc.py`

---

## 1. 네트(net) 배선표

각 와이어를 **상면 논리 좌표**(열행, 예: `19K`)로 표기. 하면 작업 시 좌우 반전 주의.

### [G] GND 공통 버스 (행 S)  —  검정 24AWG
- `3S → 25S`  (C1 100µF (−) ↔ C4 470µF (−))
- `4S → 4B`  (4S ↔ BAT(−) 홀더 검정)
- `9S → 9D`  (9S ↔ Pololu GND(in))
- `13S → 13D`  (센서 RX ↔ Pololu GND(out))
- `6S → 6J`  (C2 100nF ↔ ESP GND)
- `19S → 19N`  (19S ↔ R2 100k 하단(=GND))
- `21S → 21N`  (21S ↔ C3 ADC필터(−측=GND))
- `7S → 7E`  (7S ↔ C5 100~220µF (−))

### [V] 3.3V 전원 버스 (행 C)  —  노랑 24AWG
- `3C → 25C`  (3C ↔ 25C)
- `13B → 13C`  (Pololu VOUT ↔ 13C)
- `6C → 6K`  (6C ↔ ESP 3V3)
- `10C → 10S`  (10C ↔ 센서 VCC)
- `3C → 3R`  (3C ↔ C1 100µF (+))
- `25C → 25Q`  (25C ↔ C4 470µF (+))

### [B] BAT(+) → Pololu VIN (행 B)  —  빨강 24AWG
- `2B → 9B`  (BAT(+) 홀더 빨강 ↔ Pololu VIN)
- `7B → 7C`  (7B ↔ C5 100~220µF (+))

### [A] 분배점 → ESP IO3 (ADC)  —  보라 26AWG
- `19K → 19F → 5F → 5M → 6M`  (분배점 R1/R2(=ADC) ↔ ESP IO3(ADC))

### [T] 센서 TX → ESP IO20 (RX) [크로스]  —  초록 26AWG
- `12S → 12Q → 13Q → 13O`  (센서 TX ↔ ESP IO20(RX))

### [R] ESP IO21 (TX) → 센서 RX [크로스]  —  파랑 26AWG
- `13P → 13S`  (ESP IO21(TX) ↔ 센서 RX)

### [E] ESP IO10 → 센서 EN (선택)  —  회색 26AWG
- `13N → 14N → 14S`  (ESP IO10(→센서EN) ↔ 센서 EN)

---

## 2. 배선 경로 그리드 (하면 미러 뷰)

기호: `G`=GND · `V`=3.3V · `B`=BAT+ · `A`=ADC · `T`=UART RX(센서TX) · `R`=UART TX(센서RX) · `E`=EN · `+`=배선 교차 · `o`=부품 핀(미배선) · `·`=빈 홀

```
행\열 272625242322212019181716151413121110 9 8 7 6 5 4 3 2 1
  A    · · · · · · · · · · · · · · · · · · · · · · · · · · ·
  B    · · · · · · · · · · · · · · V · · · B B B B B + B B ·
  C    · · V V V V V V V V V V V V V V V V V V + V V + V · ·
  D    · · V · · · · · · · · · · · G · · V G · · V · G V · ·
  E    · · V · · · · · · · · · · · G · · V G · G V · G V · ·
  F    · · V · · · · · A A A A A A + A A + + A + + A G V · ·
  G    · · V · · · · · A · · · · · G · · V G · G V A G V · ·
  H    · · V · · · · · A · · · · · G · · V G · G V A G V · ·
  I    · · V · · · · · A · · · · · G · · V G · G V A G V · ·
  J    · · V · · · · · A · · · · · G · · V G · G + A G V · ·
  K    · · V · · · o · A · · · · · G · · V G · G + A G V · ·
  L    · · V · · · · · · · · · · · G · · V G · G G A G V · ·
  M    · · V · · · · · · · · · · · G · · V G · G + A G V · ·
  N    · · V · · · G · G · · · · E + · · V G · G G · G V · ·
  O    · · V · · · G · G · · · · E + · · V G · G G · G V · ·
  P    · · V · · · G · G · · · · E + · · V G · G G · G V · ·
  Q    · · V · · · G · G · · · · E + T · V G · G G · G V · ·
  R    · · · · · · G · G · · · · E + T · V G · G G · G V · ·
  S    · · G G G G G G G G G G G + + + G + G G G G G G G · ·
```

> ⚠ 교차(`+`) 지점은 **서로 다른 네트가 겹쳐 보이는 것**일 뿐, 전기적으로 연결하면 안 됩니다.
> 하면에서 절연(점퍼 띄우기 또는 피복선)으로 분리하세요.

---

## 3. 부품 핀 ↔ 홀 좌표

| 부품 핀 | 상면 좌표 | 하면(미러) 좌표 |
|---------|-----------|-----------------|
| BAT(+) 홀더 빨강 | `2B` | `26B` |
| BAT(−) 홀더 검정 | `4B` | `24B` |
| C1 100µF (+) | `3R` | `25R` |
| C1 100µF (−) | `3S` | `25S` |
| C2 100nF | `6R` | `22R` |
| C2 100nF | `6S` | `22S` |
| C3 ADC필터(+측=분배점) | `21K` | `7K` |
| C3 ADC필터(−측=GND) | `21N` | `7N` |
| C4 470µF (+) | `25Q` | `3Q` |
| C4 470µF (−) | `25S` | `3S` |
| C5 100~220µF (+) | `7C` | `21C` |
| C5 100~220µF (−) | `7E` | `21E` |
| ESP 3V3 | `6K` | `22K` |
| ESP 5V(미사용) | `6I` | `22I` |
| ESP GND | `6J` | `22J` |
| ESP IO0 | `6P` | `22P` |
| ESP IO1 | `6O` | `22O` |
| ESP IO10(→센서EN) | `13N` | `15N` |
| ESP IO2(미사용·스트래핑) | `6N` | `22N` |
| ESP IO20(RX) | `13O` | `15O` |
| ESP IO21(TX) | `13P` | `15P` |
| ESP IO3(ADC) | `6M` | `22M` |
| ESP IO4 | `6L` | `22L` |
| ESP IO5 | `13I` | `15I` |
| ESP IO6 | `13J` | `15J` |
| ESP IO7 | `13K` | `15K` |
| ESP IO8(LED) | `13L` | `15L` |
| ESP IO9 | `13M` | `15M` |
| Pololu GND(in) | `9D` | `19D` |
| Pololu GND(out) | `13D` | `15D` |
| Pololu VIN | `9B` | `19B` |
| Pololu VOUT | `13B` | `15B` |
| R1 100k 상단(=BAT+측) | `19G` | `9G` |
| R2 100k 하단(=GND) | `19N` | `9N` |
| 분배점 R1/R2(=ADC) | `19K` | `9K` |
| 센서 EN | `14S` | `14S` |
| 센서 GND | `11S` | `17S` |
| 센서 RX | `13S` | `15S` |
| 센서 TX | `12S` | `16S` |
| 센서 VCC | `10S` | `18S` |

---

## 4. 납땜 순서 (하면, v4)

```
① GND 버스 (검정 24AWG) — 가장 먼저
   행 S를 따라 col3~col25 메인 버스, 각 GND 핀으로 수직 스터브
   (BAT−, Pololu GND×2, ESP GND, R2−, C3−, C1−, C2−, C4−, C5−)
② 3.3V 버스 (노랑 24AWG)
   행 C를 따라 버스, Pololu VOUT→ESP 3V3·센서VCC·C1+·C4+
③ BAT+ (빨강 24AWG)
   행 B: 홀더(+) → Pololu VIN, 분기 C5+
④ 전압분배 출력 (보라 26AWG) — 분배점 → ESP IO3
⑤ UART 크로스 (초록/파랑 26AWG)
   센서TX→IO20(RX) [초록] / IO21(TX)→센서RX [파랑]  ← 반드시 크로스!
⑥ EN 제어 (회색 26AWG, 선택) — ESP IO10 → 센서 EN
⑦ 전체 점검 — 쇼트·도통·콜드솔더 / IPA 플럭스 세척
```

---

## 5. 통전 전 도통/검증 체크리스트

- [ ] **전원 단락 확인**: 3.3V 버스 ↔ GND 버스 저항이 수십 kΩ 이상(쇼트 아님)
- [ ] **극성**: C1·C4·C5 전해 (+)/(−) 방향, C3는 세라믹(무극성)
- [ ] **분배기**: BAT+ → R1 → 분배점 → R2 → GND 도통, 분배점 = ESP IO3
- [ ] **UART 크로스**: 센서TX 홀 ↔ ESP IO20, ESP IO21 ↔ 센서RX 홀 (절대 직결 금지)
- [ ] **EN**: ESP IO10 ↔ 센서 EN (GPIO2 아님 확인)
- [ ] **SHDN**: Pololu SHDN 핀은 미연결(NC) 유지
- [ ] **전원 순서**: 스위치 ON → VOUT=3.3V±0.1V 확인 → 그 후 ESP·센서 장착
- [ ] **핀맵 도통**: ESP 각 홀이 `ESP32_C3_pinmap.png`와 1:1 (특히 IO3/10/20/21)

---

## 6. 와이어 색상 규칙

| 기호 | 색 | 용도 | 게이지 |
|------|-----|------|--------|
| `G` | 검정 | GND 공통 버스 (행 S) | 24AWG |
| `V` | 노랑 | 3.3V 전원 버스 (행 C) | 24AWG |
| `B` | 빨강 | BAT(+) → Pololu VIN (행 B) | 24AWG |
| `A` | 보라 | 분배점 → ESP IO3 (ADC) | 26AWG |
| `T` | 초록 | 센서 TX → ESP IO20 (RX) [크로스] | 26AWG |
| `R` | 파랑 | ESP IO21 (TX) → 센서 RX [크로스] | 26AWG |
| `E` | 회색 | ESP IO10 → 센서 EN (선택) | 26AWG |

*생성: `make_solder_doc.py` — 좌표/네트 변경 시 재실행하면 이미지와 자동 동기화.*
