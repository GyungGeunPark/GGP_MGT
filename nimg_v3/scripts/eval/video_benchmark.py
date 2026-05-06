#!/usr/bin/env python3
"""
Video Benchmark — /video/*_{rgb,depth}.mp4 쌍에 대해 엔드투엔드 파이프라인 실행.

architecture §7 에 대응. 출력:
  - test_result/<date>/<tag>/per_frame.csv
  - test_result/<date>/<tag>/viz.mp4    (--save-viz 있을 때)
  - test_result/<date>/<tag>/summary.json
"""
from __future__ import annotations

import argparse
import csv
import json
import logging
import sys
import time
from pathlib import Path
from typing import Dict, Optional

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[4]      # /root/rvc_scan_ws
sys.path.insert(0, str(ROOT / "src" / "nimg_v3"))

from nimg_v3.common import BBox, Frame, PoseHypothesis, RecognizedObject
from nimg_v3.config.system_config import (
    CameraConfig, RecognitionConfig, TrackerConfig, V6OptimizationFlags,
)
from nimg_v3.input import PairedVideoSource, RawDepthReader, discover_video_pairs
from nimg_v3.recognition import (
    FastROI, NOCTISRecognizer, TemplateDB,
)
from nimg_v3.tracker import FoundationPosePlusPlus
from nimg_v3.tracker.baseline_loader import BaselineLoader
from nimg_v3.tracker.fp_estimator import build_fp_estimator

logging.basicConfig(level=logging.INFO, format="%(levelname)s %(name)s: %(message)s")
logger = logging.getLogger("video_benchmark")


# ----------------------------------------------------------------------
# Video writers
# ----------------------------------------------------------------------

class _ImageIoWriter:
    """imageio-ffmpeg 기반 (opencv-ffmpeg 가 실패하는 환경에서도 동작)."""
    def __init__(self, path: Path, size: tuple[int, int], fps: float = 30.0):
        import imageio
        self._w = imageio.get_writer(str(path), fps=fps, codec="libx264",
                                     quality=7, macro_block_size=1)
        self.size = size

    def write(self, img_bgr):
        img_rgb = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB)
        self._w.append_data(img_rgb)

    def release(self):
        self._w.close()

    def isOpened(self):
        return True


class _PngDumper:
    def __init__(self, out_dir: Path):
        self.out_dir = Path(out_dir)
        self.out_dir.mkdir(parents=True, exist_ok=True)
        self._idx = 0

    def write(self, img):
        cv2.imwrite(str(self.out_dir / f"frame_{self._idx:05d}.png"), img)
        self._idx += 1

    def release(self):
        pass

    def isOpened(self):
        return True


# ----------------------------------------------------------------------
# Visualization helpers
# ----------------------------------------------------------------------

def _draw_axes(img: np.ndarray, K: np.ndarray, T: np.ndarray, length: float = 0.04):
    origin = T[:3, 3].reshape(3, 1)
    axes = np.hstack([origin,
                      origin + T[:3, :1] * length,
                      origin + T[:3, 1:2] * length,
                      origin + T[:3, 2:3] * length])   # (3, 4)
    if (axes[2] <= 1e-3).any():
        return
    u = (K[0, 0] * axes[0] / axes[2] + K[0, 2]).astype(int)
    v = (K[1, 1] * axes[1] / axes[2] + K[1, 2]).astype(int)
    o = (int(u[0]), int(v[0]))
    cv2.line(img, o, (int(u[1]), int(v[1])), (0, 0, 255), 3)   # X red
    cv2.line(img, o, (int(u[2]), int(v[2])), (0, 255, 0), 3)   # Y green
    cv2.line(img, o, (int(u[3]), int(v[3])), (255, 0, 0), 3)   # Z blue


# 클래스별 오버레이 색상 (BGR)
_CLASS_COLORS = {
    0: (60, 200, 60),    # housing_M → 초록
    1: (60, 140, 255),   # Wiring_tray → 주황
    -1: (200, 200, 200),
}


def _draw_mask_overlay(img: np.ndarray, mask: np.ndarray, color=(60, 200, 60),
                       alpha: float = 0.42, draw_contour: bool = True):
    if mask is None or not mask.any():
        return
    overlay = img.copy()
    overlay[mask] = color
    cv2.addWeighted(overlay, alpha, img, 1 - alpha, 0, dst=img)
    if draw_contour:
        m_u8 = (mask.astype(np.uint8) * 255)
        contours, _ = cv2.findContours(m_u8, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(img, contours, -1, color, 2)


def _draw_bbox_label(img: np.ndarray, bbox, label: str, color=(60, 200, 60)):
    x1, y1, x2, y2 = bbox.x1, bbox.y1, bbox.x2, bbox.y2
    cv2.rectangle(img, (x1, y1), (x2, y2), color, 2)
    if label:
        font = cv2.FONT_HERSHEY_SIMPLEX
        (tw, th), _ = cv2.getTextSize(label, font, 0.55, 1)
        cv2.rectangle(img, (x1, max(0, y1 - th - 8)), (x1 + tw + 8, y1),
                      color, -1)
        cv2.putText(img, label, (x1 + 4, y1 - 4), font, 0.55, (20, 20, 20),
                    1, cv2.LINE_AA)


def _draw_signal_badge(img: np.ndarray, bbox, signal: int):
    sig_str = {-2: "-2", -1: "-1", 0: "0", 1: "+1", 2: "+2", 99: "none"}.get(signal, "?")
    sig_color = {-2: (30, 30, 220), -1: (50, 100, 230), 0: (50, 200, 50),
                 1: (50, 230, 230), 2: (230, 200, 50), 99: (150, 150, 150)}.get(signal, (255, 255, 255))
    cx, cy = int(bbox.cx), int(bbox.cy)
    cv2.circle(img, (cx, cy), 22, sig_color, -1)
    cv2.circle(img, (cx, cy), 22, (20, 20, 20), 2)
    font = cv2.FONT_HERSHEY_SIMPLEX
    (tw, th), _ = cv2.getTextSize(sig_str, font, 0.7, 2)
    cv2.putText(img, sig_str, (cx - tw // 2, cy + th // 2), font, 0.7, (20, 20, 20),
                2, cv2.LINE_AA)


def _hud(img: np.ndarray, lines: list[str], x: int = 10, y: int = 24,
         color=(255, 255, 255), bg=(0, 0, 0)):
    font = cv2.FONT_HERSHEY_SIMPLEX
    for i, line in enumerate(lines):
        yy = y + i * 22
        (tw, th), _ = cv2.getTextSize(line, font, 0.55, 1)
        cv2.rectangle(img, (x - 4, yy - th - 4), (x + tw + 6, yy + 4), bg, -1)
        cv2.putText(img, line, (x, yy), font, 0.55, color, 1, cv2.LINE_AA)


# ----------------------------------------------------------------------
# Measurement helpers
# ----------------------------------------------------------------------

def _relative_yaw_deg(T: np.ndarray, T_ref: np.ndarray) -> tuple[float, float, float]:
    """
    시계방향(clockwise) = +, 반시계방향(CCW) = - 컨벤션.

    image 좌표 규약에 따라 PCA 기반 yaw 는 이미 CW=+ 의 값을 냄.
    `_mask_to_6dof` 가 쓴 rotation matrix 도 `from_euler("xyz", [roll, pitch, yaw])`
    로 같은 축을 유지하므로, 상대 yaw = cur_yaw - ref_yaw 를 wrap-to-[-180,180] 만
    하면 CW=+ 가 보존된다.
    """
    from scipy.spatial.transform import Rotation as R
    R_rel = T_ref[:3, :3].T @ T[:3, :3]
    roll, pitch, yaw = R.from_matrix(R_rel).as_euler("xyz", degrees=True)
    # wrap to (-180, 180]
    yaw = ((yaw + 180.0) % 360.0) - 180.0
    roll = ((roll + 180.0) % 360.0) - 180.0
    pitch = ((pitch + 180.0) % 360.0) - 180.0
    return float(roll), float(pitch), float(yaw)


def _classify_signal(yaw_deg: float,
                     thresholds: dict = None) -> int:
    """기존 ShapeBasedSignalGenerator 와 동일한 형태: -2..+2, 99=none."""
    # 대칭 경계: ±6° 기준으로 ±12, ±25 로 분류
    t = thresholds or {"s2": 25.0, "s1": 12.0, "s0": 6.0, "none_lo": -36.0, "none_hi": 46.0}
    if yaw_deg < t["none_lo"] or yaw_deg > t["none_hi"]:
        return 99
    if yaw_deg <= -t["s2"]: return -2
    if yaw_deg <= -t["s1"]: return -1
    if yaw_deg < -t["s0"]:  return -1
    if yaw_deg < t["s0"]:   return 0
    if yaw_deg < t["s1"]:   return 1
    if yaw_deg < t["s2"]:   return 1
    return 2


# ----------------------------------------------------------------------
# Main pipeline runner
# ----------------------------------------------------------------------

class VideoPipeline:
    def __init__(self, template_db_path: Path,
                 reco_cfg: RecognitionConfig, tracker_cfg: TrackerConfig,
                 class_mesh_paths: Dict[int, str],
                 save_viz_path: Optional[Path] = None,
                 viz_every: int = 1,
                 optim: Optional[V6OptimizationFlags] = None):
        self.optim = optim or V6OptimizationFlags()
        self.db = TemplateDB.load(template_db_path)
        self.reco_cfg = reco_cfg
        self.tracker_cfg = tracker_cfg
        self.class_meshes = class_mesh_paths
        self.recognizer = NOCTISRecognizer(
            self.db,
            encoder_name=reco_cfg.encoder,
            sam_version=reco_cfg.sam_version,
            sam_cfg=reco_cfg.sam_cfg,
            sam_ckpt=reco_cfg.sam_ckpt,
            yolo_fallback=reco_cfg.yolo_fallback_path,
            match_threshold=reco_cfg.match_threshold,
            cyclic_threshold_step=reco_cfg.cyclic_threshold_step,
            nms_iou=reco_cfg.nms_iou,
            backend=reco_cfg.backend,
            roi=reco_cfg.roi,
        )
        # 멀티트래킹: 각 track 인스턴스마다 독립 FP++ 생성 (팩토리 방식).
        # 이전 버전은 class_id 당 하나의 FP++ 를 공유 → 동일 클래스의 두 객체를
        # 동시에 추적할 수 없었다.
        self.tracker_cfg = tracker_cfg
        self.class_meshes = class_mesh_paths
        self.T_ref: Dict[int, np.ndarray] = {}    # baseline
        self.active: Dict[int, dict] = {}
        self._next_uid = 1

        # Baseline (0deg) T_ref 사전 계산 — 클래스당 1회.
        # YOLO 모델 로딩은 이미 segmenter 내부에서 로드되어 있으므로 재사용.
        yolo_model = getattr(self.recognizer.segmenter, "model", None)
        class_name_by_id = {
            cid: self.db.class_meta.get(cid, {}).get("class_name", f"cls{cid}")
            for cid in class_mesh_paths
        }
        fp_estimators_for_baseline = {
            cid: build_fp_estimator(mesh, tracker_cfg.fp_weights_root)
            for cid, mesh in class_mesh_paths.items()
        }
        self.baseline_loader = BaselineLoader(
            neural_fields_root=Path(reco_cfg.template_db_root),
            class_name_by_id=class_name_by_id,
            yolo_model=yolo_model,
            fp_estimators=fp_estimators_for_baseline,
        )
        for cid in class_mesh_paths:
            T = self.baseline_loader.compute_T_ref(cid)
            if T is not None:
                self.T_ref[cid] = T
                logger.info("Baseline 0° loaded for class %d (%s)",
                            cid, class_name_by_id[cid])

        self.viz_writer: Optional[cv2.VideoWriter] = None
        self.save_viz_path = save_viz_path
        self.viz_every = viz_every

    # ------------------------------------------------------------------
    def run(self, source: PairedVideoSource, out_dir: Path, max_frames: int = -1,
            stride: int = 1):
        out_dir.mkdir(parents=True, exist_ok=True)
        csv_path = out_dir / "per_frame.csv"
        fcsv = csv_path.open("w", newline="")
        wr = csv.writer(fcsv)
        wr.writerow([
            "frame_idx", "ts", "class_id", "class_name", "object_uid",
            "T00","T01","T02","T03","T10","T11","T12","T13",
            "T20","T21","T22","T23","T30","T31","T32","T33",
            "score", "rel_roll_deg", "rel_pitch_deg", "rel_yaw_deg", "signal",
            "v_ms", "w_rads",
            "lat_A_ms", "lat_B_ms", "lat_C_ms", "lat_D_ms", "source",
            # v6 추가 메트릭
            "delta_yaw_prev", "delta_pitch_prev", "delta_roll_prev",
            "delta_t_prev_mm",
        ])
        # v6: per-uid 이전 상태 보관 (delta_*_prev 계산용)
        prev_state: dict = {}    # uid → (rel_roll, rel_pitch, rel_yaw, T)

        viz_w = source.get_K()[0, 2] * 2
        viz_h = source.get_K()[1, 2] * 2
        W = int(viz_w); H = int(viz_h)
        if self.save_viz_path is not None:
            # writer fps = source_fps / viz_every (실제 기록되는 프레임 간격 반영).
            # 소스가 없거나 fps<=0 이면 30 기본.
            src_fps = 30.0
            for attr in ("cfg", "_fps"):
                v = getattr(source, attr, None)
                if isinstance(v, (int, float)) and v > 0:
                    src_fps = float(v); break
                if hasattr(v, "fps") and v.fps > 0:
                    src_fps = float(v.fps); break
            writer_fps = max(1.0, src_fps / max(1, self.viz_every))
            try:
                self.viz_writer = _ImageIoWriter(self.save_viz_path, (W, H),
                                                 fps=writer_fps)
                logger.info("Viz writer: imageio-ffmpeg libx264 fps=%.1f → %s",
                            writer_fps, self.save_viz_path)
            except Exception as e:
                logger.warning("imageio writer failed (%s); falling back to PNG dump", e)
                self.viz_writer = _PngDumper(self.save_viz_path.with_suffix(""))

        n_frames = 0
        lat_stats = {"A": [], "B": [], "C": [], "D": []}
        signals_count: dict = {}
        source_count: dict = {}

        while True:
            fr = source.read()
            if fr is None:
                break
            if stride > 1 and fr.frame_idx % stride != 0:
                continue
            n_frames += 1
            # Layer-A: Fast ROI
            tA = time.perf_counter()
            rois = self.recognizer.fast_roi.process(fr)
            latA = (time.perf_counter() - tA) * 1000

            # Layer-B: 인식을 "매 프레임" 실행 → IoU 로 기존 track 과 재연결
            # (이전 버전은 lost 시만 호출 → mask 가 고정되어 드리프트 발생)
            tB = time.perf_counter()
            current_objs = self.recognizer.recognize(fr, prior_bboxes=rois or None)
            latB = (time.perf_counter() - tB) * 1000

            # active 트랙에 가장 IoU 높은 detection 을 재연결
            # IoU=0 인 경우라도 **같은 클래스 + 중심 거리 ≤ 90 px** 이면 재연결.
            # 빠른 움직임/얇은 객체로 인해 연속 프레임 IoU 가 낮아도 같은 물체일 수 있다.
            assigned: dict[int, RecognizedObject] = {}
            unused_objs = list(current_objs)
            for uid, st in self.active.items():
                prev_bbox = st["last_bbox"]
                best_score = -1.0; best_obj = None
                for o in unused_objs:
                    if o.class_id != st["class_id"]:
                        continue
                    iou = prev_bbox.iou(o.bbox)
                    # 중심 거리 (정규화): 동일 프레임 내 대각 길이로 scale
                    dx = o.bbox.cx - prev_bbox.cx
                    dy = o.bbox.cy - prev_bbox.cy
                    dist = (dx * dx + dy * dy) ** 0.5
                    dist_ok = dist <= 90.0
                    # score = iou (primary) with center-dist tiebreak
                    match_score = iou if iou > 0.02 else (0.2 if dist_ok else -1.0)
                    if match_score > best_score:
                        best_score = match_score; best_obj = o
                if best_obj is not None and best_score >= 0.02:
                    assigned[uid] = best_obj
                    unused_objs.remove(best_obj)

            # 할당 안 된 detection 전부를 신규 track 으로 생성 (멀티트래킹).
            for obj in unused_objs:
                if obj.class_id not in self.class_meshes:
                    continue
                # 동일 bbox IoU > 0.5 로 이미 다른 track 이 그것을 쓰고 있으면 skip
                dup = False
                for uid, st in self.active.items():
                    if st["class_id"] == obj.class_id:
                        if st["last_bbox"].iou(obj.bbox) > 0.4:
                            dup = True; break
                if dup:
                    continue
                # 인스턴스별 FP++ (v6 flags 적용)
                sym = self.baseline_loader.symmetry_tfs(obj.class_id) \
                    if self.optim.use_symmetry_tfs else None
                tr = FoundationPosePlusPlus(
                    self.class_meshes[obj.class_id],
                    self.tracker_cfg,
                    symmetry_tfs=sym,
                    prefer_textured=self.optim.use_textured_mesh,
                    use_hierarchical_refine=self.optim.use_hierarchical_refine,
                    hierarchical_n_hyp=self.optim.hierarchical_n_hyp,
                    hierarchical_sigma_rot_deg=self.optim.hierarchical_sigma_rot_deg,
                )
                hyp = tr.on_recognized(fr, obj)
                uid = self._next_uid; self._next_uid += 1
                self.active[uid] = dict(
                    class_id=obj.class_id, tracker=tr,
                    last_hyp=hyp, last_bbox=obj.bbox, last_mask=obj.mask,
                    class_name=self.db.class_meta.get(obj.class_id, {}).get(
                        "class_name", f"cls{obj.class_id}"),
                    prev_axis_sign=+1.0,   # 부호 연속성 (아래 §3 에서 사용)
                )

            # Layer-C: 추적 (모든 활성) — fresh mask/bbox 전달
            to_delete = []
            results = []
            for uid, st in list(self.active.items()):
                tr: FoundationPosePlusPlus = st["tracker"]
                prev = st["last_hyp"]
                fresh_obj = assigned.get(uid)
                fresh_mask = fresh_obj.mask if fresh_obj is not None else None
                fresh_bbox = fresh_obj.bbox if fresh_obj is not None else None
                tC = time.perf_counter()
                hyp, reinit = tr.track(fr, prev, fresh_mask=fresh_mask,
                                       fresh_bbox=fresh_bbox)
                latC = (time.perf_counter() - tC) * 1000
                # bbox/mask 상태 갱신
                if fresh_bbox is not None:
                    st["last_bbox"] = fresh_bbox
                    st["last_mask"] = fresh_mask
                # Layer-D: 측정
                tD = time.perf_counter()
                class_id = st["class_id"]
                T_ref = self.T_ref.get(class_id, np.eye(4))
                rr, rp, ry = _relative_yaw_deg(hyp.T, T_ref)
                sig = _classify_signal(ry)
                v_norm = float(np.linalg.norm(hyp.velocity)) if hyp.velocity is not None else 0.0
                w_norm = float(np.linalg.norm(hyp.angular_velocity)) if hyp.angular_velocity is not None else 0.0
                latD = (time.perf_counter() - tD) * 1000
                st["last_hyp"] = hyp
                results.append((uid, class_id, hyp, rr, rp, ry, sig, v_norm, w_norm,
                                latC, latD))
                if reinit:
                    to_delete.append(uid)
                    logger.debug("frame %d: uid=%d reinit triggered", fr.frame_idx, uid)
            for uid in to_delete:
                del self.active[uid]

            # CSV write + counters
            for (uid, cid, hyp, rr, rp, ry, sig, vn, wn, lC, lD) in results:
                cls_name = self.db.class_meta.get(cid, {}).get("class_name", f"cls{cid}")
                # v6: delta vs prev frame (같은 uid)
                d_yaw = d_pitch = d_roll = 0.0
                d_t_mm = 0.0
                if uid in prev_state:
                    pr_rr, pr_rp, pr_ry, pr_T = prev_state[uid]
                    def _wrap(a):
                        return ((a + 180.0) % 360.0) - 180.0
                    d_yaw = _wrap(ry - pr_ry)
                    d_pitch = _wrap(rp - pr_rp)
                    d_roll = _wrap(rr - pr_rr)
                    d_t_mm = float(np.linalg.norm(hyp.T[:3, 3] - pr_T[:3, 3])) * 1000.0
                prev_state[uid] = (rr, rp, ry, hyp.T.copy())

                row = [fr.frame_idx, f"{fr.timestamp:.4f}", cid, cls_name, uid]
                row += [f"{hyp.T[i, j]:.6f}" for i in range(4) for j in range(4)]
                row += [f"{hyp.score:.4f}", f"{rr:.2f}", f"{rp:.2f}", f"{ry:.2f}",
                        sig, f"{vn:.4f}", f"{wn:.4f}",
                        f"{latA:.2f}", f"{latB:.2f}", f"{lC:.2f}", f"{lD:.2f}",
                        hyp.source,
                        f"{d_yaw:.3f}", f"{d_pitch:.3f}", f"{d_roll:.3f}",
                        f"{d_t_mm:.2f}"]
                wr.writerow(row)
                signals_count[sig] = signals_count.get(sig, 0) + 1
                source_count[hyp.source] = source_count.get(hyp.source, 0) + 1
            lat_stats["A"].append(latA)
            if latB > 0: lat_stats["B"].append(latB)
            for r in results:
                lat_stats["C"].append(r[9]); lat_stats["D"].append(r[10])

            # Viz — 영상 아래쪽에 별도 status 패널을 붙여 합성.
            if self.viz_writer is not None and (fr.frame_idx % self.viz_every == 0):
                frame_viz = fr.rgb.copy()
                # ROI boxes (얇은 회색)
                for bb in rois[:5]:
                    cv2.rectangle(frame_viz, (bb.x1, bb.y1), (bb.x2, bb.y2),
                                  (90, 90, 90), 1)
                # 미할당 YOLO detection 은 옅은 테두리로
                assigned_set = {id(o) for o in assigned.values()}
                for o in current_objs:
                    if id(o) in assigned_set:
                        continue
                    color = _CLASS_COLORS.get(o.class_id, (200, 200, 200))
                    cv2.rectangle(frame_viz, (o.bbox.x1, o.bbox.y1),
                                  (o.bbox.x2, o.bbox.y2), color, 1)

                hud_lines = [
                    f"frame {fr.frame_idx}    det={len(current_objs)}    track={len(self.active)}",
                ]
                track_colors = []
                for (uid, cid, hyp, rr, rp, ry, sig, vn, wn, lC, lD) in results:
                    cls_name = self.db.class_meta.get(cid, {}).get(
                        "class_name", f"cls{cid}")
                    color = _CLASS_COLORS.get(cid, (60, 200, 60))
                    track_colors.append(color)
                    st = self.active.get(uid, {})
                    bbox = st.get("last_bbox")
                    mask = st.get("last_mask")
                    if mask is not None:
                        _draw_mask_overlay(frame_viz, mask, color=color, alpha=0.42)
                    if bbox is not None:
                        _draw_bbox_label(frame_viz, bbox,
                                         f"{cls_name}#{uid}  s={hyp.score:.2f}",
                                         color=color)
                        _draw_signal_badge(frame_viz, bbox, sig)
                    _draw_axes(frame_viz, fr.K, hyp.T, length=0.05)
                    hud_lines += [
                        f"  {cls_name}#{uid}   sig={sig}   "
                        f"yaw={ry:+6.1f}°   pitch={rp:+6.1f}°   roll={rr:+6.1f}°",
                        f"    score={hyp.score:.2f}   "
                        f"v={vn:5.2f} m/s   w={np.degrees(wn):6.1f} deg/s",
                    ]
                lat_c = float(np.mean([r[9] for r in results])) if results else 0.0
                lat_d = float(np.mean([r[10] for r in results])) if results else 0.0
                hud_lines.append(
                    f"latency  A={latA:.1f}  B={latB:.1f}  C(track)={lat_c:.1f}  D={lat_d:.2f}   ms"
                )
                from nimg_v3.pipeline import compose_with_status_panel
                viz = compose_with_status_panel(frame_viz, hud_lines,
                                                track_colors=track_colors)
                self.viz_writer.write(viz)

            if 0 < max_frames <= n_frames:
                break

        fcsv.close()
        if self.viz_writer is not None:
            self.viz_writer.release()

        # Summary
        def pctl(arr, q):
            return float(np.percentile(arr, q)) if arr else 0.0

        summary = {
            "n_frames": n_frames,
            "signals": signals_count,
            "source": source_count,
            "latency_ms": {
                "A_mean": float(np.mean(lat_stats["A"])) if lat_stats["A"] else 0,
                "B_mean": float(np.mean(lat_stats["B"])) if lat_stats["B"] else 0,
                "C_mean": float(np.mean(lat_stats["C"])) if lat_stats["C"] else 0,
                "C_p95":  pctl(lat_stats["C"], 95),
                "D_mean": float(np.mean(lat_stats["D"])) if lat_stats["D"] else 0,
            },
            "encoder_used": getattr(self.recognizer.encoder, "name", "unknown"),
            "segmenter_used": self.recognizer.seg_source,
            "fp_estimator": sorted({
                getattr(st["tracker"].fp, "source", "unknown")
                for st in self.active.values()
            }) or ["(no active)"],
            "baseline_T_ref_classes": sorted(self.T_ref.keys()),
        }
        with (out_dir / "summary.json").open("w") as f:
            json.dump(summary, f, indent=2)
        logger.info("Summary: %s", summary)
        return summary


# ----------------------------------------------------------------------
# CLI
# ----------------------------------------------------------------------

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--video-dir", type=Path, default=ROOT / "video")
    ap.add_argument("--pairs", default="test1,test3,260112",
                    help="comma-separated tags or 'all'")
    ap.add_argument("--template-db", type=Path,
                    default=ROOT / "src" / "nimg_v3" / "models" / "neural_fields" / "template_db.npz")
    ap.add_argument("--out-root", type=Path,
                    default=ROOT / "test_result" / "260420")
    ap.add_argument("--max-frames", type=int, default=-1,
                    help="per-video cap; -1 = 풀 영상 전체 (기본)")
    ap.add_argument("--stride", type=int, default=1,
                    help="process every k-th frame (기본 1 = 모든 프레임)")
    ap.add_argument("--save-viz", action="store_true", default=True)
    ap.add_argument("--viz-every", type=int, default=1,
                    help="viz 저장 간격 (기본 1 = 매 프레임, "
                         "writer fps = source_fps / viz_every)")
    ap.add_argument("--encoder", default="dinov2-large")
    ap.add_argument("--backend", default="noctis")
    ap.add_argument("--match-threshold", type=float, default=0.30)
    # v6 (Top-5) flags
    ap.add_argument("--variant", default="baseline",
                    help="baseline / r1 / r1_r2 / r1_r2_r5 / r1_r2_r3_r5 / full / custom")
    ap.add_argument("--use-raw-depth", action="store_true")
    ap.add_argument("--depth-denoise-bilateral", action="store_true")
    ap.add_argument("--use-textured-mesh", action="store_true")
    ap.add_argument("--use-symmetry-tfs", action="store_true")
    ap.add_argument("--use-hierarchical-refine", action="store_true")
    ap.add_argument("--hierarchical-n-hyp", type=int, default=2)
    ap.add_argument("--track-refine-iter", type=int, default=3)
    ap.add_argument("--render-resolution", type=int, default=160)
    ap.add_argument("--raw-depth-source", type=Path, default=None,
                    help="(use-raw-depth 시) PNG dir 또는 NPZ 경로")
    ap.add_argument("--K-json", type=Path, default=None,
                    help="(raw-depth 시) per-recording K.json")
    args = ap.parse_args()

    # Variant preset → flags 자동 설정
    variant_presets = {
        "baseline":        dict(),
        "r1":              dict(depth_denoise_bilateral=True),
        "r1_r2":           dict(depth_denoise_bilateral=True, use_textured_mesh=True,
                                use_symmetry_tfs=True),
        "r1_r2_r5":        dict(depth_denoise_bilateral=True, use_textured_mesh=True,
                                use_symmetry_tfs=True, track_refine_iter=5),
        "r1_r2_r3_r5":     dict(depth_denoise_bilateral=True, use_textured_mesh=True,
                                use_symmetry_tfs=True, track_refine_iter=5),
        "full":            dict(depth_denoise_bilateral=True, use_textured_mesh=True,
                                use_symmetry_tfs=True, use_hierarchical_refine=True,
                                track_refine_iter=5),
        "custom":          dict(),    # CLI flags 그대로 사용
    }
    preset = variant_presets.get(args.variant, {})
    # CLI 가 명시되면 preset 무시
    optim = V6OptimizationFlags(
        depth_denoise_bilateral=args.depth_denoise_bilateral
            or preset.get("depth_denoise_bilateral", False),
        use_textured_mesh=args.use_textured_mesh
            or preset.get("use_textured_mesh", False),
        use_symmetry_tfs=args.use_symmetry_tfs
            or preset.get("use_symmetry_tfs", False),
        use_hierarchical_refine=args.use_hierarchical_refine
            or preset.get("use_hierarchical_refine", False),
        hierarchical_n_hyp=args.hierarchical_n_hyp,
        use_raw_depth=args.use_raw_depth or preset.get("use_raw_depth", False),
        track_refine_iter_v6=args.track_refine_iter
            or preset.get("track_refine_iter", 3),
        render_resolution=args.render_resolution,
    )
    logger.info("Variant: %s, optim flags: %s", args.variant, optim)

    # Discover pairs
    all_pairs = discover_video_pairs(args.video_dir)
    if args.pairs == "all":
        pairs = all_pairs
    else:
        want = set([s.strip() for s in args.pairs.split(",")])
        pairs = [(t, r, d) for (t, r, d) in all_pairs if any(w in t for w in want)]
    if not pairs:
        logger.error("No pairs matched. Discovered: %s", [p[0] for p in all_pairs])
        sys.exit(1)

    # Mesh paths per class
    class_meshes: Dict[int, str] = {}
    root_nf = ROOT / "src" / "nimg_v3" / "models" / "neural_fields"
    for cid, name in [(0, "housing_M"), (1, "Wiring_tray")]:
        for cand in ["Part_02.obj", "Part_01.obj"]:
            p = root_nf / name / cand
            if p.exists():
                class_meshes[cid] = str(p); break

    # Configs
    reco_cfg = RecognitionConfig(
        backend=args.backend,
        sam_version="sam2.1",      # 실 ckpt 없음 → YOLO fallback or depth_bbox
        sam_ckpt="",
        encoder=args.encoder,
        match_threshold=args.match_threshold,
        yolo_fallback_path=str(ROOT / "src" / "nimg_v3" / "models" / "yolo" /
                               "yolo26_2class_seg_best_260324.pt"),
        nms_iou=0.4,
    )
    tr_cfg = TrackerConfig(
        backend="fp_plus_plus",
        tracker_2d="iou",
        track_refine_iter=optim.track_refine_iter_v6,
        est_refine_iter=6,
    )

    all_summaries = {}
    for tag, rgb, depth in pairs:
        logger.info("=== Running pair '%s' ===", tag)
        out_dir = args.out_root / tag
        out_dir.mkdir(parents=True, exist_ok=True)
        # depth source: raw (PNG/NPZ) vs MP4-BGR (lossy, default)
        if optim.use_raw_depth and args.raw_depth_source is not None:
            src = RawDepthReader(rgb, args.raw_depth_source,
                                 K_path=args.K_json,
                                 camera_cfg=CameraConfig(), loop=False)
        else:
            src = PairedVideoSource(rgb, depth, CameraConfig(), loop=False,
                                    depth_denoise=optim.depth_denoise_bilateral)
        if not src.start():
            continue
        viz_path = out_dir / "viz.mp4" if args.save_viz else None
        pipe = VideoPipeline(
            template_db_path=args.template_db,
            reco_cfg=reco_cfg, tracker_cfg=tr_cfg,
            class_mesh_paths=class_meshes,
            save_viz_path=viz_path, viz_every=args.viz_every,
            optim=optim,
        )
        t0 = time.time()
        summary = pipe.run(src, out_dir, max_frames=args.max_frames, stride=args.stride)
        summary["wall_time_s"] = time.time() - t0
        all_summaries[tag] = summary
        src.stop()

    with (args.out_root / "all_summaries.json").open("w") as f:
        json.dump(all_summaries, f, indent=2)
    logger.info("All done. outputs at %s", args.out_root)


if __name__ == "__main__":
    main()
