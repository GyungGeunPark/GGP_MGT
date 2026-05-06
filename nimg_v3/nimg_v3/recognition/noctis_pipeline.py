"""
NOCTIS / CNOS 파이프라인: SAM 제안 → crop → DINOv3 → TemplateDB 매칭 → NMS.

architecture §4.1 에 대응. `backend="cnos"` 시 cyclic threshold 없이 단일 임계
매칭으로 퇴화. concept prompt 는 sam_segmenter 에 위임.
"""
from __future__ import annotations

import logging
import time
from pathlib import Path
from typing import List, Optional

import cv2
import numpy as np

from ..common import BBox, Frame, RecognizedObject
from .dinov3_encoder import build_encoder
from .sam_segmenter import SegmentationProposal, build_segmenter
from .fast_roi import FastROI
from .template_db import TemplateDB

logger = logging.getLogger(__name__)


class NOCTISRecognizer:
    def __init__(
        self,
        template_db: TemplateDB,
        encoder_name: str = "dinov3-vitl16",
        sam_version: str = "sam3.1",
        sam_cfg: str = "",
        sam_ckpt: str = "",
        yolo_fallback: str | None = None,
        match_threshold: float = 0.35,
        cyclic_threshold_step: float = 0.05,
        nms_iou: float = 0.5,
        backend: str = "noctis",
        roi: tuple | None = None,
        device: str = "cuda:0",
    ):
        self.db = template_db
        self.match_threshold = match_threshold
        self.cyclic_step = cyclic_threshold_step
        self.nms_iou = nms_iou
        self.backend = backend
        self.roi = roi

        self.fast_roi = FastROI(backend="simple_depth", roi=roi)
        self.segmenter, self.seg_source = build_segmenter(
            sam_version, sam_cfg, sam_ckpt, yolo_fallback, self.fast_roi, device=device,
        )
        self.encoder = build_encoder(encoder_name, device=device)
        self._uid = 0

    # ------------------------------------------------------------------
    def recognize(self, frame: Frame,
                  prior_bboxes: Optional[List[BBox]] = None) -> List[RecognizedObject]:
        t0 = time.perf_counter()
        props: List[SegmentationProposal] = self.segmenter.segment(frame, prior_bboxes)
        if self.roi is not None:
            # ROI 밖 bbox 제거
            rx1, ry1, rx2, ry2 = self.roi
            props = [p for p in props if not (p.bbox.x2 < rx1 or p.bbox.x1 > rx2 or
                                              p.bbox.y2 < ry1 or p.bbox.y1 > ry2)]

        if not props:
            return []

        # crop + resize to 224x224 for encoder
        crops = []
        for p in props:
            x1, y1, x2, y2 = p.bbox.x1, p.bbox.y1, p.bbox.x2, p.bbox.y2
            if x2 - x1 < 8 or y2 - y1 < 8:
                crops.append(np.zeros((224, 224, 3), dtype=np.uint8)); continue
            sub = frame.rgb[y1:y2, x1:x2]
            sub_rgb = cv2.cvtColor(sub, cv2.COLOR_BGR2RGB)
            sub_rgb = cv2.resize(sub_rgb, (224, 224))
            crops.append(sub_rgb)

        feats = self.encoder.encode(crops)    # (M, D)  L2-normalized
        scores, idxs = self.db.search(feats, top_k=1)    # (M, 1)

        # cyclic threshold: 매칭 없으면 threshold 낮춰 재시도 (NOCTIS 모드)
        results: List[RecognizedObject] = []
        tau = self.match_threshold
        while tau > 0.1:
            kept = []
            for i in range(len(props)):
                s = float(scores[i, 0])
                if s < tau:
                    continue
                row = int(idxs[i, 0])
                cls = self.db.class_of(row)
                view = self.db.view_idx_of(row)
                # YOLO seg 가 직접 class 를 준 경우 우선
                yolo_cls = getattr(props[i], "yolo_class_id", None)
                if self.seg_source == "yolo_seg" and yolo_cls is not None:
                    cls = int(yolo_cls)
                obj = RecognizedObject(
                    object_uid=self._new_uid(),
                    class_id=cls, bbox=props[i].bbox, mask=props[i].mask,
                    ref_view_idx=view, match_score=s, source=self.backend,
                )
                kept.append(obj)
            if kept or self.backend != "noctis":
                results = kept
                break
            tau -= self.cyclic_step
            logger.debug("cyclic threshold lowered to %.3f", tau)

        results = self._nms(results)
        dt = (time.perf_counter() - t0) * 1000
        logger.debug("NOCTIS recognize: %d→%d objs, %.1f ms",
                     len(props), len(results), dt)
        return results

    # ------------------------------------------------------------------
    def _nms(self, objs: List[RecognizedObject]) -> List[RecognizedObject]:
        if not objs:
            return []
        # 고신뢰도 우선
        objs = sorted(objs, key=lambda o: -o.match_score)
        kept: List[RecognizedObject] = []
        for o in objs:
            if any(o.bbox.iou(k.bbox) > self.nms_iou for k in kept):
                continue
            kept.append(o)
        return kept

    def _new_uid(self) -> int:
        self._uid += 1
        return self._uid
