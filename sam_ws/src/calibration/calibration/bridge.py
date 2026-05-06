"""Bridge between mainwindow.py's CameraManager (iface) and the calibration pipeline.

The mainwindow.py module creates a global `iface = CameraManager()`. We import
this lazily so the calibration modules can be tested standalone (e.g., during
CI) without ROS2 dependencies.
"""
from __future__ import annotations

import os
import time
from typing import Dict, Optional


def get_iface():
    """Return the mainwindow.iface singleton (CameraManager). Raises if not
    importable (common in CI / unit tests)."""
    try:
        # Primary import path: source package
        from sensor_cam_main.mainwindow import iface  # type: ignore
    except Exception as e:
        raise RuntimeError(
            "Cannot import sensor_cam_main.mainwindow.iface. "
            "Make sure the Flask server is running or sensor_cam_main is installed. "
            f"(underlying error: {e})")
    return iface


def set_pcl_target_override(override_map: Dict[str, int]) -> None:
    """Set ``iface._pcl_target_override`` so calibration sessions can request a
    specific LiDAR file count (5 s × N files = N×5 s integration)."""
    iface = get_iface()
    iface._pcl_target_override = dict(override_map)


def clear_pcl_target_override() -> None:
    iface = get_iface()
    if hasattr(iface, '_pcl_target_override'):
        iface._pcl_target_override = {}


def wait_pcl_complete(unit: str, timeout_s: float = 90.0,
                      poll_interval_s: float = 0.5) -> str:
    """Poll iface.get_pcl_status(unit) until collection is done. Returns the
    absolute path of the saved merged PCD."""
    iface = get_iface()
    deadline = time.time() + timeout_s
    last_status = None
    while time.time() < deadline:
        st = iface.get_pcl_status(unit)
        last_status = st
        if not st.get('is_collecting') and st.get('last_saved_pcd_path'):
            # mainwindow stores paths relative to iface.saveImgPath
            rel = st['last_saved_pcd_path']
            return os.path.join(iface.saveImgPath, rel)
        time.sleep(poll_interval_s)
    raise TimeoutError(f"LiDAR collect did not complete for {unit} within {timeout_s}s. "
                       f"Last status: {last_status}")
