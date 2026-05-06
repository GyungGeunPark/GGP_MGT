"""
DINOv3 / DINOv2 CLS 인코더 + HSV histogram 폴백.

architecture §5.3 에 대응. transformers 가 없거나 가중치 다운로드 불가 시에도
파이프라인 전체가 동작하도록 `HsvHistEncoder` 폴백을 제공한다.
"""
from __future__ import annotations

import logging
from pathlib import Path
from typing import List

import cv2
import numpy as np

logger = logging.getLogger(__name__)


class _BaseEncoder:
    dim: int = 0
    name: str = "base"

    def encode(self, images_rgb: List[np.ndarray]) -> np.ndarray:
        raise NotImplementedError


class DinoV3Encoder(_BaseEncoder):
    """facebook/dinov3-vitl16 (or dinov2-large) CLS 토큰. transformers + 가중치 필수."""

    def __init__(self, model_name: str = "facebook/dinov3-vitl16", device: str = "cuda:0"):
        from transformers import AutoImageProcessor, AutoModel
        import torch
        self.torch = torch
        self.device = device
        logger.info("Loading DINO encoder: %s", model_name)
        self.proc = AutoImageProcessor.from_pretrained(model_name)
        self.model = AutoModel.from_pretrained(model_name).to(device).eval()
        self.dim = self.model.config.hidden_size
        self.name = model_name

    def encode(self, images_rgb: List[np.ndarray]) -> np.ndarray:
        if len(images_rgb) == 0:
            return np.zeros((0, self.dim), dtype=np.float32)
        # transformers 최신 버전은 negative-stride numpy 를 거부 → 연속 배열로 복사
        images_rgb = [np.ascontiguousarray(img) for img in images_rgb]
        x = self.proc(images=images_rgb, return_tensors="pt").to(self.device)
        with self.torch.no_grad():
            out = self.model(**x)
        cls = out.last_hidden_state[:, 0, :]
        cls = self.torch.nn.functional.normalize(cls, dim=-1)
        return cls.cpu().numpy().astype(np.float32)


class HsvHistEncoder(_BaseEncoder):
    """HSV 3D histogram (8×8×8 = 512-dim). DINOv3 부재 시 검증용 폴백.

    정밀도는 낮지만 (1) 렌더 이미지와 실측 이미지의 색 분포가 비슷하면 잘 맞고
    (2) depth-bbox segmenter 와 결합해 클래스 혼동 (housing vs tray) 정도는 구분한다.
    """
    def __init__(self, bins_per_ch: int = 8):
        self.bins = bins_per_ch
        self.dim = bins_per_ch ** 3
        self.name = "hsvhist"

    def encode(self, images_rgb: List[np.ndarray]) -> np.ndarray:
        feats = []
        for img in images_rgb:
            hsv = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
            h = cv2.calcHist([hsv], [0, 1, 2], None,
                             [self.bins, self.bins, self.bins],
                             [0, 180, 0, 256, 0, 256])
            h = h.flatten().astype(np.float32)
            n = np.linalg.norm(h)
            if n > 0:
                h /= n
            feats.append(h)
        if not feats:
            return np.zeros((0, self.dim), dtype=np.float32)
        return np.stack(feats, axis=0)


def build_encoder(encoder_name: str, device: str = "cuda:0") -> _BaseEncoder:
    """자동 폴백 체인: DINOv3 → DINOv2 → HSV."""
    fallback_order = [encoder_name]
    if encoder_name.startswith("dinov3"):
        fallback_order += ["facebook/dinov2-large", "hsvhist"]
    elif encoder_name.startswith("dinov2"):
        fallback_order += ["hsvhist"]
    else:
        fallback_order = [encoder_name]

    for name in fallback_order:
        try:
            if name == "hsvhist":
                logger.warning("Falling back to HSV-hist encoder (non-semantic)")
                return HsvHistEncoder()
            if not name.startswith("facebook/"):
                name_full = f"facebook/{name}"
            else:
                name_full = name
            return DinoV3Encoder(name_full, device=device)
        except Exception as e:
            logger.warning("Encoder '%s' failed: %s", name, e)
    return HsvHistEncoder()
