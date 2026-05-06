"""SAM + DINOv3 + NOCTIS 인식 파이프라인."""
from .fast_roi import FastROI
from .sam_segmenter import (
    SAM2Segmenter, YoloSegSegmenter, DepthBoxSegmenter,
    SegmentationProposal, build_segmenter,
)
from .dinov3_encoder import DinoV3Encoder, HsvHistEncoder, build_encoder
from .template_db import TemplateDB
from .template_builder import build_templates_for_class, load_reference_images, render_icosphere_views
from .noctis_pipeline import NOCTISRecognizer

__all__ = [
    "FastROI",
    "SAM2Segmenter", "YoloSegSegmenter", "DepthBoxSegmenter", "SegmentationProposal",
    "build_segmenter",
    "DinoV3Encoder", "HsvHistEncoder", "build_encoder",
    "TemplateDB",
    "build_templates_for_class", "load_reference_images", "render_icosphere_views",
    "NOCTISRecognizer",
]
