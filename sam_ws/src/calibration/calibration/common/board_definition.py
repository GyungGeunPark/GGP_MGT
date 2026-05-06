"""Board definition loader (Board A r2, Board B r3.1)."""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Optional
import math
import numpy as np

from .io_yaml import load_yaml, sha256_file


@dataclass
class ConeDef:
    id: int
    base_center_m: np.ndarray
    apex_m: np.ndarray
    diameter_m: float
    height_m: float
    half_angle_rad: float
    measured: bool = False


@dataclass
class CharucoParams:
    squares_x: int
    squares_y: int
    square_length_m: float
    marker_length_m: float
    aruco_dict_name: str
    marker_id_range: tuple
    n_inner_corners: int
    min_corners_for_valid: int


@dataclass
class BoardDef:
    name: str
    outer_size_m: tuple
    plane_thickness_m: float
    charuco: CharucoParams
    cones: list
    detection: dict = field(default_factory=dict)
    aruco_detector: dict = field(default_factory=dict)
    yaml_path: Optional[str] = None
    yaml_sha256: Optional[str] = None

    @classmethod
    def load(cls, path: str) -> 'BoardDef':
        data = load_yaml(path)
        ch = data['charuco']
        charuco = CharucoParams(
            squares_x=ch['squares_x'],
            squares_y=ch['squares_y'],
            square_length_m=float(ch['square_length_m']),
            marker_length_m=float(ch['marker_length_m']),
            aruco_dict_name=ch['aruco_dict'],
            marker_id_range=tuple(ch.get('marker_id_range', [0, 0])),
            n_inner_corners=int(ch.get('n_inner_corners',
                                       (ch['squares_x'] - 1) * (ch['squares_y'] - 1))),
            min_corners_for_valid=int(ch.get('min_corners_for_valid', 6)),
        )
        cones = []
        for c in data.get('cones', []):
            cones.append(ConeDef(
                id=int(c['id']),
                base_center_m=np.asarray(c['base_center_m'], dtype=float),
                apex_m=np.asarray(c['apex_m'], dtype=float),
                diameter_m=float(c['diameter_m']),
                height_m=float(c['height_m']),
                half_angle_rad=math.radians(float(c['half_angle_deg'])),
                measured=bool(c.get('measured', False)),
            ))
        return cls(
            name=data['name'],
            outer_size_m=tuple(data['outer_size_m']),
            plane_thickness_m=float(data.get('plane_thickness_m', 0.006)),
            charuco=charuco,
            cones=cones,
            detection=data.get('detection', {}),
            aruco_detector=data.get('aruco_detector', {}),
            yaml_path=path,
            yaml_sha256=sha256_file(path),
        )

    # --- OpenCV helpers -------------------------------------------------
    def get_cv_aruco_dict(self):
        import cv2
        dict_id = getattr(cv2.aruco, self.charuco.aruco_dict_name)
        return cv2.aruco.getPredefinedDictionary(dict_id)

    def get_cv_charuco_board(self):
        import cv2
        d = self.get_cv_aruco_dict()
        # OpenCV 4.5: CharucoBoard_create (function-style)
        if hasattr(cv2.aruco, 'CharucoBoard_create'):
            return cv2.aruco.CharucoBoard_create(
                self.charuco.squares_x, self.charuco.squares_y,
                self.charuco.square_length_m, self.charuco.marker_length_m, d)
        if hasattr(cv2.aruco, 'CharucoBoard'):
            # OpenCV 4.6: CharucoBoard.create(squaresX, squaresY, ...)
            if hasattr(cv2.aruco.CharucoBoard, 'create'):
                return cv2.aruco.CharucoBoard.create(
                    self.charuco.squares_x, self.charuco.squares_y,
                    self.charuco.square_length_m, self.charuco.marker_length_m, d)
            # OpenCV 4.7+: CharucoBoard((squaresX, squaresY), ...)
            return cv2.aruco.CharucoBoard(
                (self.charuco.squares_x, self.charuco.squares_y),
                self.charuco.square_length_m,
                self.charuco.marker_length_m,
                d,
            )
        raise RuntimeError("cv2.aruco.CharucoBoard not available in this OpenCV build.")

    # --- Cone apex as (4,3) numpy ------------------------------------------
    def apex_3d_board(self) -> np.ndarray:
        return np.stack([c.apex_m for c in sorted(self.cones, key=lambda x: x.id)], axis=0)

    def base_center_3d_board(self) -> np.ndarray:
        return np.stack(
            [c.base_center_m for c in sorted(self.cones, key=lambda x: x.id)], axis=0)

    def expected_apex_distances_m(self) -> np.ndarray:
        """4x4 행렬, apex 간 기대 거리."""
        apex = self.apex_3d_board()
        D = np.linalg.norm(apex[:, None, :] - apex[None, :, :], axis=-1)
        return D
