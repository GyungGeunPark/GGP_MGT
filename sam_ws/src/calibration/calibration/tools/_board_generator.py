"""Shared ChArUco board printable image generator."""
from __future__ import annotations

import os
from typing import Optional

from ..common.board_definition import BoardDef


def generate_charuco_image(board_yaml: str, out_path: str,
                           dpi: int = 300, border_mm: Optional[float] = None) -> str:
    """Generate a printable image for a BoardDef YAML.

    Only the ChArUco pattern is rendered; cones are physical and must be
    mounted afterwards. The output is a PNG whose physical size equals the
    board.outer_size_m (at the requested DPI).
    """
    import cv2
    import numpy as np

    board = BoardDef.load(board_yaml)
    W_mm, H_mm = board.outer_size_m[0] * 1000.0, board.outer_size_m[1] * 1000.0
    dpmm = dpi / 25.4
    W_px = int(round(W_mm * dpmm))
    H_px = int(round(H_mm * dpmm))

    # Compute the ChArUco pattern area
    sx = board.charuco.squares_x
    sy = board.charuco.squares_y
    sl_m = board.charuco.square_length_m
    pat_w_mm = sx * sl_m * 1000.0
    pat_h_mm = sy * sl_m * 1000.0
    pat_w_px = int(round(pat_w_mm * dpmm))
    pat_h_px = int(round(pat_h_mm * dpmm))

    cv_board = board.get_cv_charuco_board()
    # Draw the charuco pattern at pat_w_px × pat_h_px
    pat_img = None
    for attr in ('generateImage', 'draw'):
        fn = getattr(cv_board, attr, None)
        if fn is None:
            continue
        try:
            pat_img = fn((pat_w_px, pat_h_px))
            break
        except Exception:
            continue
    if pat_img is None:
        raise RuntimeError("Unable to render ChArUco pattern (no generateImage/draw method)")

    # Canvas: full board (white)
    canvas = np.full((H_px, W_px), 255, dtype=np.uint8)
    off_x = (W_px - pat_w_px) // 2
    off_y = (H_px - pat_h_px) // 2
    canvas[off_y:off_y + pat_h_px, off_x:off_x + pat_w_px] = pat_img

    # Mark cone base locations with small crosshairs
    for cone in board.cones:
        cx_mm, cy_mm = cone.base_center_m[0] * 1000.0, cone.base_center_m[1] * 1000.0
        cx_px = int(round(cx_mm * dpmm))
        cy_px = int(round(cy_mm * dpmm))
        # Draw circle at cone footprint + crosshair
        r_px = int(round((cone.diameter_m / 2) * 1000 * dpmm))
        cv2.circle(canvas, (cx_px, cy_px), r_px, 128, 2)
        cv2.line(canvas, (cx_px - r_px - 10, cy_px), (cx_px + r_px + 10, cy_px), 128, 1)
        cv2.line(canvas, (cx_px, cy_px - r_px - 10), (cx_px, cy_px + r_px + 10), 128, 1)

    os.makedirs(os.path.dirname(out_path), exist_ok=True) if os.path.dirname(out_path) else None
    cv2.imwrite(out_path, canvas)
    return out_path
