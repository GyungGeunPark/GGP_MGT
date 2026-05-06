#!/usr/bin/env python3
"""
Ablation runner — v6 의 6 variant 를 동일 입력에 대해 차례로 실행.

설계: research/260420_fp_top5_implementation_design.md §7.2.

Variants (preset):
  baseline      : 모든 v6 flag = False (v5.2 와 동일)
  r1            : Rank 1 (depth bilateral denoise) only
  r1_r2         : Rank 1 + 2 (textured + symmetry)
  r1_r2_r5      : + Rank 5 (track_refine_iter=5)
  r1_r2_r3_r5   : (r3 은 deferred — 동등)
  full          : + Rank 4 (hierarchical refine n=2)

CLI:
  python src/nimg_v3/scripts/eval/run_ablation.py \
      --variants baseline,r1,r1_r2,r1_r2_r5,full \
      --pairs 20260323_155751_test4,test1 \
      --max-frames 200 \
      --out-root test_result/260420_v6
"""
from __future__ import annotations

import argparse
import json
import logging
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
SCRIPT = Path(__file__).parent / "video_benchmark.py"

logging.basicConfig(level=logging.INFO, format="%(levelname)s %(name)s: %(message)s")
logger = logging.getLogger("ablation")


def run_variant(variant: str, pairs: str, out_root: Path,
                max_frames: int, stride: int, viz_every: int,
                extra_args: list[str] = None) -> dict:
    out_dir = out_root / variant
    out_dir.mkdir(parents=True, exist_ok=True)
    cmd = [
        sys.executable, str(SCRIPT),
        "--pairs", pairs,
        "--max-frames", str(max_frames),
        "--stride", str(stride),
        "--viz-every", str(viz_every),
        "--variant", variant,
        "--out-root", str(out_dir),
    ]
    if extra_args:
        cmd += extra_args
    logger.info("[%s] cmd: %s", variant, " ".join(cmd))
    t0 = time.time()
    res = subprocess.run(cmd, capture_output=True, text=True)
    dt = time.time() - t0
    ok = (res.returncode == 0)
    if not ok:
        logger.error("[%s] FAIL (exit=%d) stderr tail: %s",
                     variant, res.returncode, res.stderr[-1500:])
    summary_path = out_dir / "all_summaries.json"
    summary = json.loads(summary_path.read_text()) if summary_path.exists() else {}
    return dict(variant=variant, ok=ok, wall_time_s=dt, summary=summary)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--variants",
                    default="baseline,r1,r1_r2,r1_r2_r5,full",
                    help="comma-separated variant preset names")
    ap.add_argument("--pairs", default="20260323_155751_test4,test1",
                    help="comma-separated video pair tags or 'all'")
    ap.add_argument("--max-frames", type=int, default=200)
    ap.add_argument("--stride", type=int, default=1)
    ap.add_argument("--viz-every", type=int, default=1)
    ap.add_argument("--out-root", type=Path,
                    default=ROOT / "test_result" / "260420_v6")
    args = ap.parse_args()

    variants = [v.strip() for v in args.variants.split(",")]
    args.out_root.mkdir(parents=True, exist_ok=True)
    overall = []
    for v in variants:
        r = run_variant(v, args.pairs, args.out_root,
                        args.max_frames, args.stride, args.viz_every)
        overall.append(r)
    (args.out_root / "ablation_index.json").write_text(json.dumps(
        [{k: v for k, v in r.items() if k != "summary"} for r in overall],
        indent=2))
    logger.info("Ablation done: %d variants → %s", len(variants), args.out_root)


if __name__ == "__main__":
    main()
