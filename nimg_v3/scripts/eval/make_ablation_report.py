#!/usr/bin/env python3
"""
Ablation 결과 비교 리포트 생성.

각 variant 의 per_frame.csv 를 읽어 핵심 메트릭 (Δyaw p95, v_p95, uid 수,
score 표준편차) 을 비교한 표 + 차트.
"""
from __future__ import annotations

import argparse
import csv
import json
import statistics as st
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


def metrics_from_csv(p: Path) -> dict:
    rows = list(csv.DictReader(p.open()))
    if not rows:
        return {}
    by_uid: dict = {}
    for r in rows:
        u = r["object_uid"]
        by_uid.setdefault(u, []).append(r)
    n = len(rows)
    yaws = [float(r["rel_yaw_deg"]) for r in rows]
    vs = [float(r["v_ms"]) for r in rows]
    scores = [float(r["score"]) for r in rows]
    d_yaws = [abs(float(r.get("delta_yaw_prev", 0))) for r in rows]
    return dict(
        n_frames=n,
        n_uids=len(by_uid),
        v_p95=float(np.percentile(vs, 95)) if vs else 0,
        delta_yaw_mean=st.mean(d_yaws) if d_yaws else 0,
        delta_yaw_p95=float(np.percentile(d_yaws, 95)) if d_yaws else 0,
        yaw_std=st.stdev(yaws) if len(yaws) > 1 else 0,
        score_std=st.stdev(scores) if len(scores) > 1 else 0,
        score_med=st.median(scores) if scores else 0,
    )


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--root", type=Path,
                    default=Path("test_result/260420_v6"))
    args = ap.parse_args()
    root = args.root
    if not root.exists():
        print(f"[error] {root} not found"); return

    # variant 디렉터리 자동 탐색
    variants = sorted([d.name for d in root.iterdir() if d.is_dir()])
    # 각 variant 의 모든 pair csv 를 합쳐 평균
    aggregated: dict = {}
    for v in variants:
        v_dir = root / v
        # variant 디렉터리 안에 또 variant 폴더 (run_ablation 의 out 구조) 가 있을 수 있음
        # 재귀 탐색 — per_frame.csv 가 있는 가장 가까운 디렉터리 사용
        per_pair: dict = {}
        for csv_p in v_dir.rglob("per_frame.csv"):
            tag = csv_p.parent.name
            per_pair[tag] = metrics_from_csv(csv_p)
        if not per_pair:
            continue
        # 평균
        keys = list(next(iter(per_pair.values())).keys())
        avg = {}
        for k in keys:
            vals = [m[k] for m in per_pair.values()]
            avg[k] = float(np.mean(vals))
        aggregated[v] = dict(per_pair=per_pair, avg=avg)

    # MD report
    md_lines = ["# v6 Ablation Comparison", "",
                f"**root**: `{root}`", "",
                "## Variant 평균 메트릭",
                "",
                "| Variant | Δyaw_prev mean | Δyaw_prev p95 | yaw_std | v_p95 | n_uids | score_std |",
                "|---|---:|---:|---:|---:|---:|---:|"]
    for v, data in aggregated.items():
        a = data["avg"]
        md_lines.append(
            f"| {v} | {a.get('delta_yaw_mean', 0):.2f}° | "
            f"{a.get('delta_yaw_p95', 0):.2f}° | "
            f"{a.get('yaw_std', 0):.1f}° | "
            f"{a.get('v_p95', 0):.2f} m/s | "
            f"{a.get('n_uids', 0):.1f} | "
            f"{a.get('score_std', 0):.4f} |"
        )
    md_lines += ["",
                 "## 차트",
                 "",
                 "- [ablation_chart.png](ablation_chart.png)",
                 ""]
    (root / "ablation_summary.md").write_text("\n".join(md_lines))

    # 차트
    if aggregated:
        names = list(aggregated.keys())
        d_yaw_p95 = [aggregated[v]["avg"].get("delta_yaw_p95", 0) for v in names]
        d_yaw_mean = [aggregated[v]["avg"].get("delta_yaw_mean", 0) for v in names]
        v_p95 = [aggregated[v]["avg"].get("v_p95", 0) for v in names]
        score_std = [aggregated[v]["avg"].get("score_std", 0) for v in names]
        n_uids = [aggregated[v]["avg"].get("n_uids", 0) for v in names]

        fig, axes = plt.subplots(2, 2, figsize=(14, 9))
        x = np.arange(len(names))
        ax = axes[0, 0]; w = 0.35
        ax.bar(x - w/2, d_yaw_mean, w, label="mean", color="#2ca02c")
        ax.bar(x + w/2, d_yaw_p95, w, label="p95", color="#d62728")
        ax.set_xticks(x); ax.set_xticklabels(names, rotation=20)
        ax.set_title("Δyaw_prev (degrees)"); ax.set_ylabel("deg")
        ax.legend(); ax.grid(True, alpha=0.3)

        ax = axes[0, 1]
        ax.bar(names, v_p95, color="#9467bd")
        ax.set_title("Linear velocity p95 (m/s)")
        ax.tick_params(axis="x", rotation=20); ax.grid(True, alpha=0.3)

        ax = axes[1, 0]
        ax.bar(names, score_std, color="#ff7f0e")
        ax.set_title("ScoreNet std (variability — higher = working)")
        ax.tick_params(axis="x", rotation=20); ax.grid(True, alpha=0.3)

        ax = axes[1, 1]
        ax.bar(names, n_uids, color="#1f77b4")
        ax.set_title("Avg unique uids per pair")
        ax.tick_params(axis="x", rotation=20); ax.grid(True, alpha=0.3)

        fig.suptitle(f"v6 Ablation — {len(names)} variants × {len(next(iter(aggregated.values()))['per_pair'])} pairs",
                     fontsize=12)
        fig.tight_layout()
        fig.savefig(root / "ablation_chart.png", dpi=110)
        plt.close(fig)

    (root / "ablation_data.json").write_text(json.dumps(aggregated, indent=2))
    print(f"[ablation_report] {root / 'ablation_summary.md'}")


if __name__ == "__main__":
    main()
