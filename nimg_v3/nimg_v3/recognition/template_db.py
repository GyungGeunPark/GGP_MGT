"""
TemplateDB — 클래스별 42뷰 렌더 이미지 + feature 인덱스.

오프라인(`build_templates.py`)에서 빌드하고, 온라인(`noctis_pipeline.py`)에서 로드.
FAISS IndexFlatIP 을 사용하되, 소규모(N_views*N_classes < 1000) 에서는 단순
행렬곱으로도 충분해 FAISS 가 없어도 동작하도록 자동 폴백을 둔다.
"""
from __future__ import annotations

import json
import logging
from pathlib import Path
from typing import List, Optional, Tuple

import numpy as np

logger = logging.getLogger(__name__)


class TemplateDB:
    """모든 클래스의 템플릿 피처를 단일 행렬로 관리.

    Attributes:
        feats:      (N, D) float32, L2-normalized
        class_ids:  (N,) int
        view_idxs:  (N,) int
        view_poses: (N, 4, 4) float64 (카메라 pose in mesh frame)
        class_meta: dict[int, dict] — class_id → {name, mesh_path, encoder}
    """

    def __init__(self):
        self.feats: np.ndarray = np.zeros((0, 0), dtype=np.float32)
        self.class_ids: np.ndarray = np.zeros((0,), dtype=np.int32)
        self.view_idxs: np.ndarray = np.zeros((0,), dtype=np.int32)
        self.view_poses: np.ndarray = np.zeros((0, 4, 4), dtype=np.float64)
        self.class_meta: dict[int, dict] = {}
        self._faiss_idx = None

    # ---------------- build-time ----------------
    def add_class(self, class_id: int, class_name: str, mesh_path: str,
                  encoder_name: str,
                  feats: np.ndarray, view_poses: np.ndarray):
        assert feats.ndim == 2 and view_poses.shape[0] == feats.shape[0]
        D = feats.shape[1]
        if self.feats.shape[0] == 0:
            self.feats = feats.copy()
        else:
            assert self.feats.shape[1] == D, \
                f"encoder dim mismatch: existing {self.feats.shape[1]} new {D}"
            self.feats = np.concatenate([self.feats, feats], axis=0)
        n = feats.shape[0]
        self.class_ids = np.concatenate([self.class_ids, np.full(n, class_id, dtype=np.int32)])
        self.view_idxs = np.concatenate([self.view_idxs, np.arange(n, dtype=np.int32)])
        if self.view_poses.shape[0] == 0:
            self.view_poses = view_poses.astype(np.float64)
        else:
            self.view_poses = np.concatenate([self.view_poses, view_poses.astype(np.float64)], axis=0)
        self.class_meta[int(class_id)] = {
            "class_name": class_name,
            "mesh_path": mesh_path,
            "encoder": encoder_name,
            "n_views": int(n),
        }

    def save(self, out_path: str | Path):
        out_path = Path(out_path)
        out_path.parent.mkdir(parents=True, exist_ok=True)
        np.savez(out_path,
                 feats=self.feats,
                 class_ids=self.class_ids,
                 view_idxs=self.view_idxs,
                 view_poses=self.view_poses)
        with open(str(out_path).replace(".npz", ".json"), "w") as f:
            json.dump(self.class_meta, f, indent=2)
        logger.info("TemplateDB saved: %s (N=%d, D=%d)",
                    out_path, self.feats.shape[0], self.feats.shape[1])

    @classmethod
    def load(cls, in_path: str | Path) -> "TemplateDB":
        in_path = Path(in_path)
        d = np.load(in_path)
        self = cls()
        self.feats = d["feats"].astype(np.float32)
        self.class_ids = d["class_ids"].astype(np.int32)
        self.view_idxs = d["view_idxs"].astype(np.int32)
        self.view_poses = d["view_poses"].astype(np.float64)
        meta_path = Path(str(in_path).replace(".npz", ".json"))
        if meta_path.exists():
            with open(meta_path) as f:
                raw = json.load(f)
            self.class_meta = {int(k): v for k, v in raw.items()}
        try:
            import faiss
            idx = faiss.IndexFlatIP(self.feats.shape[1])
            idx.add(self.feats)
            self._faiss_idx = idx
        except Exception:
            self._faiss_idx = None
        logger.info("TemplateDB loaded: %s (N=%d, D=%d, classes=%s)",
                    in_path, self.feats.shape[0], self.feats.shape[1],
                    list(self.class_meta.keys()))
        return self

    # ---------------- runtime ----------------
    def search(self, q: np.ndarray, top_k: int = 5) -> Tuple[np.ndarray, np.ndarray]:
        """cosine similarity (입력은 이미 L2-normalized 가정).

        Returns (scores (Q, K), indices (Q, K))  — indices 는 self.feats 행 번호.
        """
        if self.feats.shape[0] == 0:
            return (np.zeros((q.shape[0], 0), dtype=np.float32),
                    np.zeros((q.shape[0], 0), dtype=np.int64))
        if self._faiss_idx is not None:
            D, I = self._faiss_idx.search(q.astype(np.float32), top_k)
            return D, I
        # 단순 행렬곱
        sim = q @ self.feats.T          # (Q, N)
        idx = np.argsort(-sim, axis=1)[:, :top_k]
        scores = np.take_along_axis(sim, idx, axis=1)
        return scores.astype(np.float32), idx.astype(np.int64)

    def class_of(self, row: int) -> int:
        return int(self.class_ids[row])

    def view_pose_of(self, row: int) -> np.ndarray:
        return self.view_poses[row]

    def view_idx_of(self, row: int) -> int:
        return int(self.view_idxs[row])
