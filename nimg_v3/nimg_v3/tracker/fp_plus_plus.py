"""
FoundationPose++ 조합 — 2D tracker + depth sample + pre-filter KF + FP refine.

architecture §4.2, §6.3 에 대응. 실제 FP 가 없으면 FallbackFPEstimator 가 대신
동작하므로, 이 클래스는 항상 실행 가능하다.
"""
from __future__ import annotations

import logging
import time
from pathlib import Path
from typing import Optional, Tuple

import numpy as np

from ..common import Frame, PoseHypothesis, RecognizedObject
from ..config.system_config import TrackerConfig
from .fp_estimator import build_fp_estimator
from .hierarchical_refine import HierarchicalRefiner
from .lost_detector import LostDetector
from .pose_hypothesis_kf import PoseHypothesisKF
from .tracker_2d import IoUTracker2D, build_tracker_2d

logger = logging.getLogger(__name__)


def _robust_depth_sample(depth: np.ndarray, cx: float, cy: float, win: int = 5) -> float:
    H, W = depth.shape
    x1 = max(0, int(cx) - win); x2 = min(W, int(cx) + win + 1)
    y1 = max(0, int(cy) - win); y2 = min(H, int(cy) + win + 1)
    patch = depth[y1:y2, x1:x2]
    valid = patch[(patch > 0.15) & (patch < 3.5)]
    if valid.size < 5:
        return 0.0
    return float(np.median(valid))


class FoundationPosePlusPlus:
    def __init__(self, mesh_path: str, cfg: TrackerConfig,
                 symmetry_tfs: Optional[list] = None,
                 prefer_textured: bool = True,
                 use_hierarchical_refine: bool = False,
                 hierarchical_n_hyp: int = 2,
                 hierarchical_sigma_rot_deg: float = 15.0):
        self.cfg = cfg
        self.mesh_path = mesh_path
        self.fp = build_fp_estimator(mesh_path, cfg.fp_weights_root,
                                     symmetry_tfs=symmetry_tfs,
                                     prefer_textured=prefer_textured)
        self.tracker2d = build_tracker_2d(cfg.tracker_2d, cfg.tracker_2d_ckpt)
        # Rank 4: hierarchical refiner — n_hyp=1 이면 단일 호출과 동등
        self.refiner = HierarchicalRefiner(
            self.fp,
            n_hypotheses=hierarchical_n_hyp if use_hierarchical_refine else 1,
            sigma_rot_deg=hierarchical_sigma_rot_deg,
            track_refine_iter=cfg.track_refine_iter,
        )
        self.pre_kf = PoseHypothesisKF(process_noise=cfg.kf_measurement_noise_scale)
        self.lost = LostDetector(cfg.lost_score_threshold, cfg.lost_score_frames,
                                 cfg.periodic_reinit_frames)
        self._initialized = False
        self._last: Optional[PoseHypothesis] = None

    # ------------------------------------------------------------------
    def on_recognized(self, frame: Frame, obj: RecognizedObject) -> PoseHypothesis:
        mask = obj.mask
        if mask is None:
            import numpy as _np
            mask = _np.zeros(frame.rgb.shape[:2], dtype=bool)
            mask[obj.bbox.y1:obj.bbox.y2, obj.bbox.x1:obj.bbox.x2] = True
        t0 = time.perf_counter()
        try:
            T, score = self.fp.register(frame.rgb, frame.depth, frame.K, mask,
                                        iter_n=self.cfg.est_refine_iter)
        except Exception as e:
            logger.warning("FP register failed (%s); using bbox fallback pose", e)
            import numpy as _np
            from scipy.spatial.transform import Rotation as _R
            z = 1.0
            cx, cy = obj.bbox.cx, obj.bbox.cy
            x = (cx - frame.K[0, 2]) * z / frame.K[0, 0]
            y = (cy - frame.K[1, 2]) * z / frame.K[1, 1]
            T = _np.eye(4); T[:3, 3] = [x, y, z]
            score = 0.2
        self.tracker2d.reset(frame.rgb, obj.bbox)
        self.pre_kf.reset(T, frame.timestamp)
        self.lost.reset()
        hyp = PoseHypothesis(T=T, score=score, source="init",
                             timestamp=frame.timestamp)
        self._last = hyp
        self._initialized = True
        logger.debug("FP++ init: score=%.3f time=%.1f ms", score,
                     (time.perf_counter() - t0) * 1000)
        return hyp

    def track(self, frame: Frame, prev: PoseHypothesis,
              fresh_mask: Optional[np.ndarray] = None,
              fresh_bbox: Optional[object] = None) -> Tuple[PoseHypothesis, bool]:
        """Returns (hypothesis, reinit_needed).

        fresh_mask/fresh_bbox: 상위 파이프라인(YOLO-seg) 이 매 프레임 공급하는
        현재 마스크. 주어지면 CSRT 대신 이것을 translation 소스로 쓰고,
        fp.track_one 에도 넘겨 드리프트를 억제한다.
        """
        if not self._initialized:
            return prev, True

        # 1. bbox 결정: fresh > 2D tracker > prev_T projection
        bbox_cx = bbox_cy = None
        tracker_ok = False
        if fresh_bbox is not None:
            bbox_cx, bbox_cy = fresh_bbox.cx, fresh_bbox.cy
            tracker_ok = True
            try:
                self.tracker2d.reset(frame.rgb, fresh_bbox)
            except Exception:
                pass
        else:
            res = self.tracker2d.update(frame.rgb)
            if res is not None:
                bbox_cx, bbox_cy = res.bbox.cx, res.bbox.cy
                tracker_ok = True
            else:
                # YOLO 도 CSRT 도 실패 — prev_T 를 그대로 사용하고 tracker_ok 는
                # True 로 유지(LostDetector 가 grace 를 줌). KF 예측으로 이어간다.
                z_prev = float(prev.T[2, 3])
                if z_prev > 1e-3:
                    bbox_cx = prev.T[0, 3] * frame.K[0, 0] / z_prev + frame.K[0, 2]
                    bbox_cy = prev.T[1, 3] * frame.K[1, 1] / z_prev + frame.K[1, 2]
                tracker_ok = False   # LostDetector 는 grace 내에서는 OK 처리

        # 2. depth sample
        if tracker_ok and bbox_cx is not None:
            z = _robust_depth_sample(frame.depth, bbox_cx, bbox_cy)
            if z <= 0:
                z = float(prev.T[2, 3])
            x = (bbox_cx - frame.K[0, 2]) * z / frame.K[0, 0]
            y = (bbox_cy - frame.K[1, 2]) * z / frame.K[1, 1]
            q_pred, _t_pred, _ = self.pre_kf.predict(frame.timestamp)
            t_pred = np.array([x, y, z])
            from scipy.spatial.transform import Rotation as R
            T_hyp = np.eye(4)
            T_hyp[:3, :3] = R.from_quat(q_pred).as_matrix()
            T_hyp[:3, 3] = t_pred
        else:
            T_hyp = prev.T.copy()

        # 3. FP refine — hierarchical refiner 위임 (n_hyp=1 이면 단일 호출).
        try:
            T_new, score = self.refiner.refine(
                frame, T_hyp,
                omega=self.pre_kf.omega if self.pre_kf.initialized else None,
                dt=max(1e-3, frame.timestamp - (self.pre_kf.last_ts or frame.timestamp)),
                mask=fresh_mask,
            )
        except Exception as e:
            # FP 내부 singular matrix / bbox edge case → 이전 포즈 유지
            logger.debug("FP track_one failed (%s); holding prev_T", e)
            T_new, score = prev.T.copy(), max(0.3, prev.score * 0.8)

        # 4. 오리엔테이션 스무딩 — fallback 노이즈 억제
        #    fresh_mask 가 있으면 translation 은 신뢰, 방위는 heavy smooth.
        if fresh_mask is not None:
            from scipy.spatial.transform import Rotation as R
            q_new = R.from_matrix(T_new[:3, :3]).as_quat()
            q_prev = R.from_matrix(prev.T[:3, :3]).as_quat()
            q_smoothed = _slerp_quat(q_prev, q_new, 0.25)   # 25% 만 반영
            T_new = T_new.copy()
            T_new[:3, :3] = R.from_quat(q_smoothed).as_matrix()

        # 5. KF update
        self.pre_kf.update(T_new, frame.timestamp)
        hyp = PoseHypothesis(
            T=T_new, score=score, source="track",
            timestamp=frame.timestamp,
            velocity=self.pre_kf.v.copy(),
            angular_velocity=self.pre_kf.omega.copy(),
        )
        self._last = hyp
        reinit = self.lost.update(score, tracker_ok)
        return hyp, reinit


def _slerp_quat(q0: np.ndarray, q1: np.ndarray, t: float) -> np.ndarray:
    import numpy as _np
    dot = float(_np.dot(q0, q1))
    if dot < 0.0:
        q1 = -q1; dot = -dot
    if dot > 0.9995:
        out = q0 + t * (q1 - q0)
    else:
        theta = _np.arccos(_np.clip(dot, -1.0, 1.0))
        s = _np.sin(theta)
        out = (_np.sin((1 - t) * theta) / s) * q0 + (_np.sin(t * theta) / s) * q1
    return out / (_np.linalg.norm(out) + 1e-12)

    @property
    def last(self) -> Optional[PoseHypothesis]:
        return self._last
