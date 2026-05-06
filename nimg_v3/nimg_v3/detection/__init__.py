"""
detection 모듈 - 객체 탐지

YOLO 기반 객체 탐지를 제공합니다.
nimg_v2의 yolo_detector를 재사용합니다.
"""

# nimg_v2의 YOLODetector 재사용
try:
    import sys
    from pathlib import Path
    nimg_v2_path = Path(__file__).parents[3] / 'nimg_v2' / 'nimg_v2'
    if nimg_v2_path.exists():
        sys.path.insert(0, str(nimg_v2_path))
        from detection.yolo_detector import YOLODetector, Detection
        __all__ = ['YOLODetector', 'Detection']
except ImportError:
    __all__ = []
