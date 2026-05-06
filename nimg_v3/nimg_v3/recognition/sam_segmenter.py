"""
SAM segmenter — SAM 3.1 / SAM 2.1 우선, YOLO-seg / depth-bbox 자동 폴백.

architecture §4.1 의 `sam_segmenter.py` 에 대응.
현재 환경에 SAM 3.1 / SAM 2 체크포인트가 없는 경우 `YOLODetectorSegmenter` →
`DepthBoxSegmenter` 순으로 폴백한다. 모든 경로는 동일한 인터페이스를 갖는다.
"""
from __future__ import annotations

import logging
from pathlib import Path
from typing import List, Optional

import cv2
import numpy as np

from ..common import BBox, Frame

logger = logging.getLogger(__name__)


class SegmentationProposal:
    """SAM 또는 fallback 이 생성한 마스크 제안 한 개."""

    def __init__(self, mask: np.ndarray, bbox: BBox, score: float = 1.0,
                 source: str = "fallback"):
        self.mask = mask.astype(bool)
        self.bbox = bbox
        self.score = score
        self.source = source


# ----------------------------------------------------------------------
# 1) SAM 3.1 / SAM 2 wrapper (import-time guard)
# ----------------------------------------------------------------------

class SAM2Segmenter:
    """SAM 2 / SAM 3.1 AutomaticMaskGenerator 래퍼. 체크포인트/모듈이 없으면 에러."""

    def __init__(self, cfg_path: str, ckpt_path: str,
                 points_per_side: int = 16, device: str = "cuda:0"):
        try:
            from sam2.build_sam import build_sam2
            from sam2.automatic_mask_generator import SAM2AutomaticMaskGenerator
        except ImportError as e:
            raise RuntimeError(f"SAM2 package not installed: {e}")
        if not Path(ckpt_path).exists():
            raise FileNotFoundError(f"SAM ckpt missing: {ckpt_path}")
        sam = build_sam2(cfg_path, ckpt_path, device=device, apply_postprocessing=False)
        self.gen = SAM2AutomaticMaskGenerator(
            sam,
            points_per_side=points_per_side,
            pred_iou_thresh=0.7,
            stability_score_thresh=0.92,
            min_mask_region_area=500,
        )
        self.source = "sam2"

    def segment(self, frame: Frame,
                prior_bboxes: Optional[List[BBox]] = None) -> List[SegmentationProposal]:
        rgb = cv2.cvtColor(frame.rgb, cv2.COLOR_BGR2RGB)
        results = self.gen.generate(rgb)
        out: List[SegmentationProposal] = []
        for r in results:
            m = r["segmentation"].astype(bool)
            ys, xs = np.where(m)
            if len(xs) < 200:
                continue
            bb = BBox(int(xs.min()), int(ys.min()), int(xs.max()), int(ys.max()),
                      conf=float(r.get("predicted_iou", 0.9)))
            out.append(SegmentationProposal(m, bb, r.get("stability_score", 0.9), self.source))
        return out


# ----------------------------------------------------------------------
# 2) YOLO seg fallback — 기존 학습된 `yolo26_2class_seg_best_*.pt`
# ----------------------------------------------------------------------

class YoloSegSegmenter:
    """ultralytics YOLO-seg 모델을 SAM 대체 segmenter 로 쓰는 fallback."""

    def __init__(self, model_path: str, conf: float = 0.5, device: str = "cuda:0"):
        from ultralytics import YOLO
        if not Path(model_path).exists():
            raise FileNotFoundError(f"YOLO weights missing: {model_path}")
        self.model = YOLO(model_path)
        self.conf = conf
        self.device = device
        self.source = "yolo_seg"

    def segment(self, frame: Frame,
                prior_bboxes: Optional[List[BBox]] = None) -> List[SegmentationProposal]:
        res = self.model.predict(frame.rgb, conf=self.conf, verbose=False, device=self.device)
        out: List[SegmentationProposal] = []
        if not res or res[0].masks is None:
            return out
        r0 = res[0]
        masks = r0.masks.data.cpu().numpy()  # (N, H, W) float
        boxes = r0.boxes.xyxy.cpu().numpy()
        cls_ids = r0.boxes.cls.cpu().numpy().astype(int)
        confs = r0.boxes.conf.cpu().numpy()
        H, W = frame.rgb.shape[:2]
        for m, bb, cid, cf in zip(masks, boxes, cls_ids, confs):
            if m.shape != (H, W):
                m = cv2.resize(m, (W, H), interpolation=cv2.INTER_NEAREST)
            mb = m > 0.5
            x1, y1, x2, y2 = map(int, bb)
            out.append(SegmentationProposal(
                mb, BBox(x1, y1, x2, y2, conf=float(cf)),
                score=float(cf), source=self.source,
            ))
            out[-1].yolo_class_id = int(cid)
        return out


# ----------------------------------------------------------------------
# 3) Depth-bbox fallback — 마지막 안전망
# ----------------------------------------------------------------------

class DepthBoxSegmenter:
    """Fast ROI bbox 그대로 받아 bbox 내 유효 depth 픽셀을 mask 로 반환.

    SAM 부재 시 검증용. mask 품질은 낮으나 파이프라인 전체를 구동 가능.
    """

    def __init__(self, fast_roi):
        self.fast_roi = fast_roi
        self.source = "depth_bbox"

    def segment(self, frame: Frame,
                prior_bboxes: Optional[List[BBox]] = None) -> List[SegmentationProposal]:
        boxes = prior_bboxes if prior_bboxes else self.fast_roi.process(frame)
        out: List[SegmentationProposal] = []
        depth = frame.depth
        H, W = depth.shape
        for bb in boxes:
            m = np.zeros((H, W), dtype=bool)
            x1, y1, x2, y2 = (max(0, bb.x1), max(0, bb.y1),
                              min(W, bb.x2), min(H, bb.y2))
            sub = depth[y1:y2, x1:x2]
            if sub.size == 0:
                continue
            valid_sub = (sub > 0.15) & (sub < 3.5)
            # 전경 판단: sub 내 median ± 0.1 m 구간
            if valid_sub.sum() < 50:
                continue
            med = float(np.median(sub[valid_sub]))
            fg = (sub > med - 0.1) & (sub < med + 0.1) & valid_sub
            m[y1:y2, x1:x2] = fg
            out.append(SegmentationProposal(m, bb, score=bb.conf, source=self.source))
        return out


# ----------------------------------------------------------------------
# 4) Auto factory — 체크포인트 탐색하여 최선 백엔드 선택
# ----------------------------------------------------------------------

def build_segmenter(
    sam_version: str,
    sam_cfg: str,
    sam_ckpt: str,
    yolo_path: str | None,
    fast_roi,
    device: str = "cuda:0",
):
    """SAM → YOLO → DepthBox 순으로 폴백.

    Returns (segmenter, source_tag)
    """
    # 1. SAM
    if sam_version in ("sam3.1", "sam2.1") and sam_ckpt and Path(sam_ckpt).exists():
        try:
            seg = SAM2Segmenter(sam_cfg, sam_ckpt, device=device)
            logger.info("Segmenter: SAM %s (%s)", sam_version, sam_ckpt)
            return seg, f"sam_{sam_version}"
        except Exception as e:
            logger.warning("SAM init failed, trying YOLO: %s", e)
    # 2. YOLO-seg
    if yolo_path and Path(yolo_path).exists():
        try:
            seg = YoloSegSegmenter(yolo_path, device=device)
            logger.info("Segmenter: YOLO-seg fallback (%s)", yolo_path)
            return seg, "yolo_seg"
        except Exception as e:
            logger.warning("YOLO-seg init failed: %s", e)
    # 3. Depth bbox
    logger.warning("Segmenter: falling back to depth-bbox only")
    return DepthBoxSegmenter(fast_roi), "depth_bbox"
