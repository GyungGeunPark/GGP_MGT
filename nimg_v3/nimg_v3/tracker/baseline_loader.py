"""
BaselineLoader — reference_config.yaml 의 `baseline_angles[0]` 이미지를
FoundationPose (or fallback) 에 통과시켜 T_ref (0°) 을 사전 계산.

architecture §5.4 에 대응. 이 T_ref 는 런타임 중 모든 상대 포즈(rel_yaw) 계산의
기준점이 된다.
"""
from __future__ import annotations

import logging
from pathlib import Path
from typing import Optional

import cv2
import numpy as np
import yaml

logger = logging.getLogger(__name__)


class BaselineLoader:
    def __init__(self, neural_fields_root: Path,
                 class_name_by_id: dict[int, str],
                 yolo_model=None,
                 fp_estimators: dict[int, object] | None = None,
                 camera_K: np.ndarray | None = None):
        self.root = Path(neural_fields_root)
        self.class_name_by_id = class_name_by_id
        self.yolo = yolo_model
        self.fp_by_class = fp_estimators or {}
        self.K = camera_K
        self._cache: dict[int, np.ndarray] = {}
        self._axis_ref_by_class: dict[int, np.ndarray] = {}
        # symmetry_tfs (4x4 list) per class — reference_config.yaml 의 'symmetry' 필드
        self._symmetry_tfs_by_class: dict[int, list[np.ndarray]] = {}

    # ------------------------------------------------------------------
    def _load_rgb_depth_mask(self, class_id: int) -> tuple[
            np.ndarray, np.ndarray, Optional[np.ndarray]] | None:
        class_name = self.class_name_by_id.get(class_id)
        if not class_name:
            return None
        class_dir = self.root / class_name / "reference_images"
        cfg_path = class_dir / "reference_config.yaml"
        if not cfg_path.exists():
            logger.warning("No reference_config: %s", cfg_path)
            return None
        cfg = yaml.safe_load(cfg_path.read_text())
        baseline_angles = cfg.get("baseline_angles", {}) or {}
        baseline_depths = cfg.get("baseline_depths", {}) or {}
        # Key 는 int 0 또는 str "0"
        rgb_name = baseline_angles.get(0) or baseline_angles.get("0")
        depth_name = baseline_depths.get(0) or baseline_depths.get("0")
        # Wiring_tray 식: baseline_image / signal_mapping[0]
        if not rgb_name:
            rgb_name = cfg.get("baseline_image")
        if not rgb_name:
            sig_map = cfg.get("signal_mapping", {}) or {}
            rgb_name = sig_map.get(0) or sig_map.get("0")
        if not rgb_name:
            logger.warning("baseline reference missing for %s (no baseline_angles[0] / baseline_image / signal_mapping[0])",
                           class_name)
            return None
        img_dir = class_dir / "images"
        rgb_path = img_dir / rgb_name
        if not rgb_path.exists():
            logger.warning("baseline rgb missing: %s", rgb_path)
            return None
        rgb = cv2.imread(str(rgb_path))
        depth = None
        if depth_name:
            depth_path = img_dir / depth_name
            if depth_path.exists():
                raw = cv2.imread(str(depth_path), cv2.IMREAD_UNCHANGED)
                if raw is not None:
                    if raw.dtype == np.uint16:
                        depth = raw.astype(np.float32) * 0.001
                    else:
                        # BGR → gray → 5m 선형 매핑 (기존 디코더와 일관)
                        gray = cv2.cvtColor(raw, cv2.COLOR_BGR2GRAY) \
                            if raw.ndim == 3 else raw
                        depth = gray.astype(np.float32) * (5.0 / 255.0)
        if depth is None:
            # depth 누락 시 기본 1.0m
            depth = np.ones(rgb.shape[:2], dtype=np.float32) * 1.0
        # YOLO mask
        mask = None
        if self.yolo is not None:
            res = self.yolo.predict(rgb, conf=0.3, verbose=False)
            if res and res[0].masks is not None:
                m = res[0].masks.data.cpu().numpy()[0]
                H, W = rgb.shape[:2]
                if m.shape != (H, W):
                    m = cv2.resize(m, (W, H), interpolation=cv2.INTER_NEAREST)
                mask = m > 0.5
        return rgb, depth, mask

    def compute_T_ref(self, class_id: int) -> Optional[np.ndarray]:
        """0deg 이미지에 FP/Fallback 을 실행하여 T_ref 계산. 결과는 cache."""
        if class_id in self._cache:
            return self._cache[class_id]
        loaded = self._load_rgb_depth_mask(class_id)
        if loaded is None:
            return None
        rgb, depth, mask = loaded
        fp = self.fp_by_class.get(class_id)
        if fp is None:
            logger.warning("No FP estimator for class %d — cannot compute T_ref", class_id)
            return None
        if mask is None:
            # mask 없으면 프레임 중앙 1/3 영역으로 fallback
            H, W = rgb.shape[:2]
            mask = np.zeros((H, W), dtype=bool)
            mask[H // 3:2 * H // 3, W // 3:2 * W // 3] = True
        K = self.K if self.K is not None else np.array([[383.883, 0, 320.499],
                                                         [0, 383.883, 237.913],
                                                         [0, 0, 1]], dtype=np.float64)
        T_ref, score = fp.register(rgb, depth, K, mask, iter_n=10)
        self._cache[class_id] = T_ref
        # 기준 yaw 축 기록 → 부호 일관성용
        self._axis_ref_by_class[class_id] = T_ref[:3, :3] @ np.array([1.0, 0.0, 0.0])
        logger.info("T_ref[class=%d] computed (score=%.2f): yaw=%.2f deg",
                    class_id, score,
                    float(np.degrees(np.arctan2(
                        self._axis_ref_by_class[class_id][1],
                        self._axis_ref_by_class[class_id][0]))))
        return T_ref

    def axis_ref(self, class_id: int) -> Optional[np.ndarray]:
        return self._axis_ref_by_class.get(class_id)

    def symmetry_tfs(self, class_id: int) -> list[np.ndarray]:
        """reference_config.yaml 의 `symmetry:` 필드 → 4x4 transforms list.

        YAML 형식 예 (Rank 2.B):
            symmetry:
              - identity              # 항상 첫 entry
              - rotation_z: 180       # axis-angle deg
              - rotation_y: 90
        """
        if class_id in self._symmetry_tfs_by_class:
            return self._symmetry_tfs_by_class[class_id]
        out = [np.eye(4)]
        class_name = self.class_name_by_id.get(class_id)
        if class_name:
            cfg_path = (self.root / class_name / "reference_images"
                        / "reference_config.yaml")
            if cfg_path.exists():
                try:
                    cfg = yaml.safe_load(cfg_path.read_text())
                    sym_list = cfg.get("symmetry", []) or []
                    for entry in sym_list:
                        if entry == "identity":
                            continue
                        if not isinstance(entry, dict):
                            continue
                        T = np.eye(4)
                        from scipy.spatial.transform import Rotation as R
                        for axis, deg in entry.items():
                            ax = axis.replace("rotation_", "")
                            T[:3, :3] = R.from_euler(ax, float(deg),
                                                      degrees=True).as_matrix()
                            break
                        out.append(T)
                    if len(out) > 1:
                        logger.info("Class %s: %d symmetry transforms loaded",
                                    class_name, len(out))
                except Exception as e:
                    logger.warning("symmetry parse failed for %s: %s",
                                   class_name, e)
        self._symmetry_tfs_by_class[class_id] = out
        return out
