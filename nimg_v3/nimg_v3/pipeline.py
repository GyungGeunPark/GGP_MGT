"""
nimg_v3 v5 통합 파이프라인.

architecture §3 의 4-layer (Input → Fast ROI → Recognition → Pose → Measurement)
를 단일 클래스로 제공. `scripts/eval/video_benchmark.py` 와 streaming 스크립트
둘 다에서 import 해 쓴다.

streaming 모드에서는 max_frames=-1 + loop=True 로 무한 재생이 가능하고,
각 프레임은 (results, viz_image, latency_dict) 를 리턴한다.
"""
from __future__ import annotations

import csv
import json
import logging
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Callable, Dict, List, Optional, Tuple

import cv2
import numpy as np

from .common import BBox, Frame, PoseHypothesis, RecognizedObject
from .config.system_config import RecognitionConfig, TrackerConfig
from .input import InputAdapter
from .recognition import NOCTISRecognizer, TemplateDB
from .tracker import FoundationPosePlusPlus
from .tracker.baseline_loader import BaselineLoader
from .tracker.fp_estimator import build_fp_estimator

logger = logging.getLogger(__name__)


# ----------------------------------------------------------------------
# Visualization helpers
# ----------------------------------------------------------------------

_CLASS_COLORS = {
    0: (60, 200, 60),    # housing_M → 초록
    1: (60, 140, 255),   # Wiring_tray → 주황
    -1: (200, 200, 200),
}


def draw_axes(img: np.ndarray, K: np.ndarray, T: np.ndarray, length: float = 0.04):
    origin = T[:3, 3].reshape(3, 1)
    axes = np.hstack([origin,
                      origin + T[:3, :1] * length,
                      origin + T[:3, 1:2] * length,
                      origin + T[:3, 2:3] * length])
    if (axes[2] <= 1e-3).any():
        return
    u = (K[0, 0] * axes[0] / axes[2] + K[0, 2]).astype(int)
    v = (K[1, 1] * axes[1] / axes[2] + K[1, 2]).astype(int)
    o = (int(u[0]), int(v[0]))
    cv2.line(img, o, (int(u[1]), int(v[1])), (0, 0, 255), 3)   # X red
    cv2.line(img, o, (int(u[2]), int(v[2])), (0, 255, 0), 3)   # Y green
    cv2.line(img, o, (int(u[3]), int(v[3])), (255, 0, 0), 3)   # Z blue


def draw_mask_overlay(img: np.ndarray, mask: np.ndarray,
                      color=(60, 200, 60), alpha: float = 0.42,
                      draw_contour: bool = True):
    if mask is None or not mask.any():
        return
    overlay = img.copy()
    overlay[mask] = color
    cv2.addWeighted(overlay, alpha, img, 1 - alpha, 0, dst=img)
    if draw_contour:
        m_u8 = (mask.astype(np.uint8) * 255)
        contours, _ = cv2.findContours(m_u8, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(img, contours, -1, color, 2)


def draw_bbox_label(img: np.ndarray, bbox, label: str, color=(60, 200, 60)):
    x1, y1, x2, y2 = bbox.x1, bbox.y1, bbox.x2, bbox.y2
    cv2.rectangle(img, (x1, y1), (x2, y2), color, 2)
    if label:
        font = cv2.FONT_HERSHEY_SIMPLEX
        (tw, th), _ = cv2.getTextSize(label, font, 0.55, 1)
        cv2.rectangle(img, (x1, max(0, y1 - th - 8)), (x1 + tw + 8, y1), color, -1)
        cv2.putText(img, label, (x1 + 4, y1 - 4), font, 0.55, (20, 20, 20),
                    1, cv2.LINE_AA)


def draw_signal_badge(img: np.ndarray, bbox, signal: int):
    sig_str = {-2: "-2", -1: "-1", 0: "0", 1: "+1", 2: "+2", 99: "none"}.get(signal, "?")
    sig_color = {-2: (30, 30, 220), -1: (50, 100, 230), 0: (50, 200, 50),
                 1: (50, 230, 230), 2: (230, 200, 50), 99: (150, 150, 150)
                 }.get(signal, (255, 255, 255))
    cx, cy = int(bbox.cx), int(bbox.cy)
    cv2.circle(img, (cx, cy), 22, sig_color, -1)
    cv2.circle(img, (cx, cy), 22, (20, 20, 20), 2)
    font = cv2.FONT_HERSHEY_SIMPLEX
    (tw, th), _ = cv2.getTextSize(sig_str, font, 0.7, 2)
    cv2.putText(img, sig_str, (cx - tw // 2, cy + th // 2), font, 0.7, (20, 20, 20),
                2, cv2.LINE_AA)


def draw_hud(img: np.ndarray, lines: list[str], x: int = 10, y: int = 24,
             color=(255, 255, 255), bg=(0, 0, 0)):
    """HUD 를 이미지 위에 오버레이(레거시). 새 UI 는 `compose_with_status_panel` 사용."""
    font = cv2.FONT_HERSHEY_SIMPLEX
    for i, line in enumerate(lines):
        yy = y + i * 22
        (tw, th), _ = cv2.getTextSize(line, font, 0.55, 1)
        cv2.rectangle(img, (x - 4, yy - th - 4), (x + tw + 6, yy + 4), bg, -1)
        cv2.putText(img, line, (x, yy), font, 0.55, color, 1, cv2.LINE_AA)


def compose_with_status_panel(
    img: np.ndarray,
    hud_lines: list[str],
    track_colors: list[tuple[int, int, int]] | None = None,
    panel_height: int | None = 220,      # 고정 높이 (비디오 writer 일관성).
                                          # None 이면 라인 수 기반 자동.
    panel_bg: tuple[int, int, int] = (28, 28, 32),
    text_color: tuple[int, int, int] = (230, 230, 230),
    accent_color: tuple[int, int, int] = (80, 220, 120),
    line_spacing: int = 22,
    pad_x: int = 14,
    pad_y: int = 18,
    max_lines: int = 8,
) -> np.ndarray:
    """영상 아래쪽에 별도 status 패널 영역을 붙여 합성한다.

    - img: 원본 RGB 프레임 (이미 3축/mask 등은 그려진 상태)
    - hud_lines: 패널에 표시할 텍스트 줄 (첫 줄은 헤더로 강조)
    - track_colors: 각 "  class#uid" 라인에 왼쪽에 붙일 색 박스 (없으면 미사용)
    - panel_height: None 이면 줄 수 기반 자동 계산

    반환: (img_h + panel_h, img_w, 3) 의 새 uint8 BGR 이미지.
    """
    # 최대 라인 수 제한 — 고정 패널에 넘치지 않게 + "+N more" 표시
    if len(hud_lines) > max_lines:
        overflow = len(hud_lines) - max_lines + 1
        hud_lines = hud_lines[:max_lines - 1] + [f"  … (+{overflow} more)"]
    H, W = img.shape[:2]
    n_lines = max(1, len(hud_lines))
    ph = panel_height if panel_height else pad_y * 2 + line_spacing * n_lines
    out_h = H + ph
    out = np.zeros((out_h, W, 3), dtype=np.uint8)
    out[:H] = img
    # 패널 배경
    out[H:] = panel_bg
    # 상단 구분선
    cv2.line(out, (0, H), (W, H), (70, 70, 80), 2)
    # 좌측 세로 accent bar
    cv2.rectangle(out, (0, H), (6, out_h), accent_color, -1)

    font = cv2.FONT_HERSHEY_SIMPLEX
    y = H + pad_y + line_spacing // 2 + 4
    for i, line in enumerate(hud_lines):
        # 트랙별 라인 앞에 색 박스 표시
        x = pad_x
        is_track_header = line.startswith("  ")
        if is_track_header and track_colors:
            # 몇 번째 트랙인지 추정: "  " 시작 라인 개수
            nt = sum(1 for L in hud_lines[:i + 1] if L.startswith("  ") and "sig=" in L)
            if 0 < nt <= len(track_colors):
                c = track_colors[nt - 1]
                cv2.rectangle(out, (pad_x, y - line_spacing // 2 + 4),
                              (pad_x + 12, y + line_spacing // 2 - 2), c, -1)
                x = pad_x + 20
        # 헤더(첫 줄): 더 큰 + accent
        if i == 0:
            cv2.putText(out, line, (x, y), font, 0.62, accent_color, 2, cv2.LINE_AA)
        else:
            # 들여쓰기된 서브-라인은 약간 작게, 일반 정보 라인은 기본 크기
            size = 0.52 if line.startswith("    ") else 0.56
            thickness = 1
            cv2.putText(out, line, (x, y), font, size, text_color, thickness, cv2.LINE_AA)
        y += line_spacing
    return out


# ----------------------------------------------------------------------
# Signal classification & relative pose
# ----------------------------------------------------------------------

def relative_ypr_deg(T: np.ndarray, T_ref: np.ndarray) -> Tuple[float, float, float]:
    """baseline 대비 (roll, pitch, yaw) in degrees, wrap to (-180, 180]."""
    from scipy.spatial.transform import Rotation as R
    R_rel = T_ref[:3, :3].T @ T[:3, :3]
    roll, pitch, yaw = R.from_matrix(R_rel).as_euler("xyz", degrees=True)
    wrap = lambda a: ((a + 180.0) % 360.0) - 180.0
    return wrap(roll), wrap(pitch), wrap(yaw)


def classify_signal(yaw_deg: float,
                    thresholds: dict | None = None) -> int:
    """CW=+ convention. -2..+2, 99=none."""
    t = thresholds or {"s2": 25.0, "s1": 12.0, "s0": 6.0,
                       "none_lo": -36.0, "none_hi": 46.0}
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
# Pipeline (통합)
# ----------------------------------------------------------------------

@dataclass
class FrameResult:
    """한 프레임 처리 결과 — streaming 루프가 per-frame 소비."""
    frame: Frame
    tracks: List[dict]               # [{uid, class_id, class_name, hyp, rr, rp, ry,
                                     #   signal, v_ms, w_rads, bbox, mask}, ...]
    current_objs: List[RecognizedObject]
    rois: List[BBox]
    viz: Optional[np.ndarray]
    latency_ms: dict                 # {A, B, C_mean, D_mean}
    reinit_events: int


class Nimg3Pipeline:
    """v5 통합 파이프라인. `recognize every frame → FP++ tracking → measurement`.

    Streaming 용: `step(frame)` 로 프레임별 처리.
    배치 용: `run(source)` 로 InputAdapter 에서 전체 프레임 소비.
    """

    def __init__(self,
                 template_db_path: Path,
                 reco_cfg: RecognitionConfig,
                 tracker_cfg: TrackerConfig,
                 class_mesh_paths: Dict[int, str],
                 ros2_publisher=None,
                 signal_thresholds: Optional[dict] = None):
        self.db = TemplateDB.load(template_db_path)
        self.reco_cfg = reco_cfg
        self.tracker_cfg = tracker_cfg
        self.class_meshes = class_mesh_paths
        self.ros2 = ros2_publisher
        self.signal_thresholds = signal_thresholds

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
        self.T_ref: Dict[int, np.ndarray] = {}
        self.active: Dict[int, dict] = {}
        self._next_uid = 1

        # Baseline T_ref 사전 계산
        yolo_model = getattr(self.recognizer.segmenter, "model", None)
        class_name_by_id = {
            cid: self.db.class_meta.get(cid, {}).get("class_name", f"cls{cid}")
            for cid in class_mesh_paths
        }
        self.class_name_by_id = class_name_by_id
        fp_for_baseline = {
            cid: build_fp_estimator(mesh, tracker_cfg.fp_weights_root)
            for cid, mesh in class_mesh_paths.items()
        }
        self.baseline_loader = BaselineLoader(
            neural_fields_root=Path(reco_cfg.template_db_root),
            class_name_by_id=class_name_by_id,
            yolo_model=yolo_model,
            fp_estimators=fp_for_baseline,
        )
        for cid in class_mesh_paths:
            T = self.baseline_loader.compute_T_ref(cid)
            if T is not None:
                self.T_ref[cid] = T
                logger.info("Baseline 0° loaded for class %d (%s)",
                            cid, class_name_by_id[cid])

    # ------------------------------------------------------------------
    def step(self, fr: Frame, *, draw: bool = True) -> FrameResult:
        """프레임 한 개 처리 — streaming 루프용."""
        tA = time.perf_counter()
        rois = self.recognizer.fast_roi.process(fr)
        latA = (time.perf_counter() - tA) * 1000

        tB = time.perf_counter()
        current_objs = self.recognizer.recognize(fr, prior_bboxes=rois or None)
        latB = (time.perf_counter() - tB) * 1000

        # IoU + center-dist 재연결
        assigned: Dict[int, RecognizedObject] = {}
        unused = list(current_objs)
        for uid, st in self.active.items():
            prev_bbox = st["last_bbox"]
            best_score = -1.0; best = None
            for o in unused:
                if o.class_id != st["class_id"]:
                    continue
                iou = prev_bbox.iou(o.bbox)
                dx = o.bbox.cx - prev_bbox.cx
                dy = o.bbox.cy - prev_bbox.cy
                dist_ok = (dx * dx + dy * dy) ** 0.5 <= 90.0
                s = iou if iou > 0.02 else (0.2 if dist_ok else -1.0)
                if s > best_score:
                    best_score = s; best = o
            if best is not None and best_score >= 0.02:
                assigned[uid] = best
                unused.remove(best)

        # 신규 track (멀티트래킹)
        for obj in unused:
            if obj.class_id not in self.class_meshes:
                continue
            dup = any(st["class_id"] == obj.class_id and
                      st["last_bbox"].iou(obj.bbox) > 0.4
                      for st in self.active.values())
            if dup:
                continue
            tr = FoundationPosePlusPlus(self.class_meshes[obj.class_id], self.tracker_cfg)
            hyp = tr.on_recognized(fr, obj)
            uid = self._next_uid; self._next_uid += 1
            self.active[uid] = dict(
                class_id=obj.class_id, tracker=tr,
                last_hyp=hyp, last_bbox=obj.bbox, last_mask=obj.mask,
                class_name=self.class_name_by_id.get(obj.class_id, f"cls{obj.class_id}"),
            )

        # Track + Measure
        tracks_out: List[dict] = []
        reinit_events = 0
        lat_c_list, lat_d_list = [], []
        to_delete = []
        for uid, st in list(self.active.items()):
            tr: FoundationPosePlusPlus = st["tracker"]
            prev = st["last_hyp"]
            fresh_obj = assigned.get(uid)
            fresh_mask = fresh_obj.mask if fresh_obj else None
            fresh_bbox = fresh_obj.bbox if fresh_obj else None
            tC = time.perf_counter()
            hyp, reinit = tr.track(fr, prev, fresh_mask=fresh_mask, fresh_bbox=fresh_bbox)
            latC = (time.perf_counter() - tC) * 1000
            lat_c_list.append(latC)

            if fresh_bbox is not None:
                st["last_bbox"] = fresh_bbox
                st["last_mask"] = fresh_mask

            tD = time.perf_counter()
            T_ref = self.T_ref.get(st["class_id"], np.eye(4))
            rr, rp, ry = relative_ypr_deg(hyp.T, T_ref)
            sig = classify_signal(ry, self.signal_thresholds)
            v_norm = float(np.linalg.norm(hyp.velocity)) if hyp.velocity is not None else 0.0
            w_norm = float(np.linalg.norm(hyp.angular_velocity)) if hyp.angular_velocity is not None else 0.0
            latD = (time.perf_counter() - tD) * 1000
            lat_d_list.append(latD)
            st["last_hyp"] = hyp

            tracks_out.append(dict(
                uid=uid, class_id=st["class_id"], class_name=st["class_name"],
                hyp=hyp, rr=rr, rp=rp, ry=ry, signal=sig,
                v_ms=v_norm, w_rads=w_norm,
                bbox=st["last_bbox"], mask=st["last_mask"],
            ))
            # ROS2 publish
            if self.ros2 is not None:
                self.ros2.publish_signal(
                    signal=sig, object_id=uid, confidence=float(hyp.score),
                    relative_yaw=ry, position=hyp.T[:3, 3].tolist(),
                )

            if reinit:
                to_delete.append(uid)
                reinit_events += 1
        for uid in to_delete:
            del self.active[uid]

        lat_c = float(np.mean(lat_c_list)) if lat_c_list else 0.0
        lat_d = float(np.mean(lat_d_list)) if lat_d_list else 0.0

        viz = None
        if draw:
            frame_viz = fr.rgb.copy()
            # ROI 와 미할당 detection bbox 는 영상 위에 그대로 (mask/3축 과 함께)
            for bb in rois[:5]:
                cv2.rectangle(frame_viz, (bb.x1, bb.y1), (bb.x2, bb.y2), (90, 90, 90), 1)
            assigned_set = {id(o) for o in assigned.values()}
            for o in current_objs:
                if id(o) in assigned_set: continue
                color = _CLASS_COLORS.get(o.class_id, (200, 200, 200))
                cv2.rectangle(frame_viz, (o.bbox.x1, o.bbox.y1),
                              (o.bbox.x2, o.bbox.y2), color, 1)

            hud = [f"frame {fr.frame_idx}    det={len(current_objs)}    track={len(tracks_out)}"]
            track_colors = []
            for tr_r in tracks_out:
                color = _CLASS_COLORS.get(tr_r["class_id"], (60, 200, 60))
                track_colors.append(color)
                if tr_r["mask"] is not None:
                    draw_mask_overlay(frame_viz, tr_r["mask"], color=color)
                draw_bbox_label(frame_viz, tr_r["bbox"],
                                f"{tr_r['class_name']}#{tr_r['uid']}  "
                                f"s={tr_r['hyp'].score:.2f}", color=color)
                draw_signal_badge(frame_viz, tr_r["bbox"], tr_r["signal"])
                draw_axes(frame_viz, fr.K, tr_r["hyp"].T, length=0.05)
                hud += [
                    f"  {tr_r['class_name']}#{tr_r['uid']}   sig={tr_r['signal']}   "
                    f"yaw={tr_r['ry']:+6.1f}°   pitch={tr_r['rp']:+6.1f}°   roll={tr_r['rr']:+6.1f}°",
                    f"    score={tr_r['hyp'].score:.2f}   "
                    f"v={tr_r['v_ms']:5.2f} m/s   w={np.degrees(tr_r['w_rads']):6.1f} deg/s",
                ]
            hud.append(
                f"latency  A={latA:.1f}  B={latB:.1f}  C(track)={lat_c:.1f}  D={lat_d:.2f}   ms"
            )
            # 영상 아래쪽에 별도 status 패널 추가
            viz = compose_with_status_panel(frame_viz, hud, track_colors=track_colors)

        return FrameResult(
            frame=fr, tracks=tracks_out, current_objs=current_objs, rois=rois,
            viz=viz,
            latency_ms=dict(A=latA, B=latB, C_mean=lat_c, D_mean=lat_d),
            reinit_events=reinit_events,
        )
