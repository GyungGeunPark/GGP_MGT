"""ChArUco corner detection (OpenCV 4.7+ / 4.6 호환)."""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Optional
import numpy as np


@dataclass
class ChArUcoDetection:
    corners_2d: np.ndarray        # (N,2) image pixel
    corners_3d_board: np.ndarray  # (N,3) board frame
    ids: np.ndarray               # (N,) charuco corner ids
    marker_corners: list = field(default_factory=list)  # aruco markers 원본
    marker_ids: Optional[np.ndarray] = None

    @property
    def count(self) -> int:
        return len(self.ids) if self.ids is not None else 0


def _build_detector_params(aruco_cfg: dict):
    import cv2
    p = cv2.aruco.DetectorParameters() if hasattr(cv2.aruco, 'DetectorParameters') \
        else cv2.aruco.DetectorParameters_create()
    p.adaptiveThreshWinSizeMin  = aruco_cfg.get('adaptive_thresh_win_size_min', 3)
    p.adaptiveThreshWinSizeMax  = aruco_cfg.get('adaptive_thresh_win_size_max', 23)
    p.adaptiveThreshWinSizeStep = aruco_cfg.get('adaptive_thresh_win_size_step', 10)
    p.minMarkerPerimeterRate    = aruco_cfg.get('min_marker_perimeter_rate', 0.02)
    refine_name = aruco_cfg.get('corner_refinement', 'SUBPIX')
    p.cornerRefinementMethod    = getattr(cv2.aruco,
                                          f'CORNER_REFINE_{refine_name}',
                                          cv2.aruco.CORNER_REFINE_SUBPIX)
    p.cornerRefinementWinSize     = aruco_cfg.get('corner_refinement_win_size', 7)
    p.cornerRefinementMaxIterations = aruco_cfg.get('corner_refinement_max_iters', 50)
    return p


def detect_charuco(image, board_def) -> Optional[ChArUcoDetection]:
    """이미지에서 ChArUco 코너 검출.

    Args:
        image: 그레이스케일 또는 BGR numpy array
        board_def: BoardDef
    Returns:
        ChArUcoDetection or None
    """
    import cv2
    if image is None:
        return None
    if image.ndim == 3:
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    else:
        gray = image

    cv_board = board_def.get_cv_charuco_board()
    params = _build_detector_params(board_def.aruco_detector)

    # Path A: OpenCV 4.7+ CharucoDetector API
    if hasattr(cv2.aruco, 'CharucoDetector'):
        ch_params = cv2.aruco.CharucoParameters() \
            if hasattr(cv2.aruco, 'CharucoParameters') else None
        try:
            if ch_params is not None:
                detector = cv2.aruco.CharucoDetector(cv_board, ch_params, params)
            else:
                detector = cv2.aruco.CharucoDetector(cv_board)
            corners, ids, marker_corners, marker_ids = detector.detectBoard(gray)
        except Exception:
            corners = ids = None
            marker_corners, marker_ids = [], None
    else:
        # Path B: OpenCV 4.6 — legacy
        aruco_dict = board_def.get_cv_aruco_dict()
        marker_corners, marker_ids, _ = cv2.aruco.detectMarkers(
            gray, aruco_dict, parameters=params)
        if marker_ids is None or len(marker_ids) == 0:
            return None
        retval, corners, ids = cv2.aruco.interpolateCornersCharuco(
            marker_corners, marker_ids, gray, cv_board)
        if retval <= 0 or corners is None:
            return None

    if corners is None or ids is None or len(ids) < board_def.charuco.min_corners_for_valid:
        return None

    corners = np.asarray(corners, dtype=np.float32).reshape(-1, 2)
    ids_arr = np.asarray(ids, dtype=np.int32).ravel()

    # Board 3D points using matchImagePoints
    try:
        obj_pts, img_pts = cv_board.matchImagePoints(
            corners.reshape(-1, 1, 2).astype(np.float32), ids_arr)
        obj_pts = np.asarray(obj_pts, dtype=np.float64).reshape(-1, 3)
        img_pts = np.asarray(img_pts, dtype=np.float64).reshape(-1, 2)
    except Exception:
        # Fallback: compute directly from grid
        sx, sy = board_def.charuco.squares_x, board_def.charuco.squares_y
        sl = board_def.charuco.square_length_m
        obj_pts_all = np.zeros(((sx - 1) * (sy - 1), 3), dtype=np.float64)
        k = 0
        for j in range(1, sy):
            for i in range(1, sx):
                obj_pts_all[k] = [i * sl, j * sl, 0.0]
                k += 1
        obj_pts = obj_pts_all[ids_arr]
        img_pts = corners.astype(np.float64)

    return ChArUcoDetection(
        corners_2d=img_pts.astype(np.float64),
        corners_3d_board=obj_pts.astype(np.float64),
        ids=ids_arr,
        marker_corners=list(marker_corners) if marker_corners is not None else [],
        marker_ids=np.asarray(marker_ids).ravel() if marker_ids is not None else None,
    )


def charuco_pnp(detection: ChArUcoDetection, K, D):
    """Solve PnP from ChArUco detection. Returns T_cam_to_board (4x4) or None."""
    import cv2
    if detection is None or detection.count < 4:
        return None
    obj = detection.corners_3d_board.astype(np.float64)
    img = detection.corners_2d.astype(np.float64)
    ok, rvec, tvec = cv2.solvePnP(obj, img, K, D,
                                  flags=cv2.SOLVEPNP_ITERATIVE)
    if not ok:
        return None
    from .transforms import se3_from_rvec_tvec
    return se3_from_rvec_tvec(rvec, tvec)
