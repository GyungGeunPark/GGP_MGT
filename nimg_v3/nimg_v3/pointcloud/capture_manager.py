"""
PointCloud Capture and Management

Provides pointcloud capture, save, load, and preprocessing functionality.
"""

import open3d as o3d
import numpy as np
from pathlib import Path
from datetime import datetime
from typing import List, Dict, Optional, Union
import json


class PointCloudCaptureManager:
    """PointCloud Capture, Save, Load, and Preprocessing Manager"""

    def __init__(
        self,
        camera,
        save_dir: str = "./pointcloud_data",
        preprocessing_enabled: bool = True
    ):
        """
        Initialize the capture manager.

        Args:
            camera: RealSenseCameraWrapper instance
            save_dir: Directory to save pointclouds
            preprocessing_enabled: Enable preprocessing by default
        """
        self.camera = camera
        self.save_directory = Path(save_dir)
        self.save_directory.mkdir(parents=True, exist_ok=True)

        self.preprocessing_enabled = preprocessing_enabled
        self.saved_pointclouds: List[Dict] = []

        # Preprocessing parameters
        self.voxel_size = 0.005  # 5mm
        self.sor_nb_neighbors = 20
        self.sor_std_ratio = 2.0
        self.normal_radius = 0.1
        self.normal_max_nn = 30

        print(f"[CaptureManager] Save directory: {self.save_directory}")

    def capture_pointcloud(self) -> o3d.geometry.PointCloud:
        """
        Capture point cloud from camera.

        Returns:
            o3d.geometry.PointCloud: Captured point cloud
        """
        print("[CaptureManager] Capturing point cloud...")

        pcd = self.camera.get_pointcloud()

        print(f"[CaptureManager] Raw points: {len(pcd.points)}")

        if self.preprocessing_enabled:
            pcd = self.preprocess_pointcloud(pcd)
            print(f"[CaptureManager] Preprocessed points: {len(pcd.points)}")

        return pcd

    def save_pointcloud(
        self,
        pcd: o3d.geometry.PointCloud,
        filename: Optional[str] = None,
        format: str = "ply"
    ) -> str:
        """
        Save point cloud to file.

        Args:
            pcd: PointCloud object
            filename: Filename (auto-generated if None)
            format: "ply" or "pcd"

        Returns:
            str: Saved file path
        """
        if filename is None:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"captured_{timestamp}.{format}"

        filepath = self.save_directory / filename

        # Save pointcloud
        write_ascii = (format == "ply")
        success = o3d.io.write_point_cloud(
            str(filepath), pcd, write_ascii=write_ascii
        )

        if not success:
            raise IOError(f"Failed to save point cloud to {filepath}")

        # Update metadata
        metadata = {
            "path": str(filepath),
            "filename": filename,
            "timestamp": datetime.now().isoformat(),
            "num_points": len(pcd.points),
            "format": format,
            "has_colors": pcd.has_colors(),
            "has_normals": pcd.has_normals(),
        }
        self.saved_pointclouds.append(metadata)

        # Save metadata to JSON
        self._save_metadata()

        print(f"[CaptureManager] Saved: {filepath} ({len(pcd.points)} points)")

        return str(filepath)

    def load_pointcloud(self, filepath: str) -> o3d.geometry.PointCloud:
        """
        Load point cloud from file.

        Args:
            filepath: Path to .ply or .pcd file

        Returns:
            o3d.geometry.PointCloud
        """
        print(f"[CaptureManager] Loading: {filepath}")

        pcd = o3d.io.read_point_cloud(filepath)

        if len(pcd.points) == 0:
            raise ValueError(f"Empty point cloud: {filepath}")

        print(f"[CaptureManager] Loaded {len(pcd.points)} points")

        return pcd

    def preprocess_pointcloud(
        self,
        pcd: o3d.geometry.PointCloud,
        voxel_size: Optional[float] = None,
        remove_outliers: bool = True,
        estimate_normals: bool = True
    ) -> o3d.geometry.PointCloud:
        """
        Preprocess point cloud.

        Steps:
            1. Statistical Outlier Removal
            2. Voxel Downsampling
            3. Normal Estimation

        Args:
            pcd: Input point cloud
            voxel_size: Voxel size for downsampling (None uses default)
            remove_outliers: Whether to remove outliers
            estimate_normals: Whether to estimate normals

        Returns:
            o3d.geometry.PointCloud: Preprocessed point cloud
        """
        if voxel_size is None:
            voxel_size = self.voxel_size

        # 1. Statistical Outlier Removal
        if remove_outliers:
            pcd, ind = pcd.remove_statistical_outlier(
                nb_neighbors=self.sor_nb_neighbors,
                std_ratio=self.sor_std_ratio
            )
            print(f"[Preprocess] After SOR: {len(pcd.points)} points")

        # 2. Voxel Downsampling
        if voxel_size > 0:
            pcd = pcd.voxel_down_sample(voxel_size=voxel_size)
            print(f"[Preprocess] After voxel ({voxel_size*1000:.1f}mm): {len(pcd.points)} points")

        # 3. Normal Estimation
        if estimate_normals:
            pcd.estimate_normals(
                search_param=o3d.geometry.KDTreeSearchParamHybrid(
                    radius=self.normal_radius,
                    max_nn=self.normal_max_nn
                )
            )
            print("[Preprocess] Normals estimated")

        return pcd

    def get_saved_list(self) -> List[Dict]:
        """
        Get list of saved point clouds.

        Returns:
            List[Dict]: List of metadata dicts
        """
        # Refresh from file system
        self.saved_pointclouds = []

        # Find all .ply and .pcd files
        patterns = ["captured_*.ply", "captured_*.pcd"]

        for pattern in patterns:
            for filepath in sorted(self.save_directory.glob(pattern)):
                try:
                    pcd = o3d.io.read_point_cloud(str(filepath))

                    self.saved_pointclouds.append({
                        "path": str(filepath),
                        "filename": filepath.name,
                        "timestamp": datetime.fromtimestamp(
                            filepath.stat().st_mtime
                        ).isoformat(),
                        "num_points": len(pcd.points),
                        "format": filepath.suffix[1:],
                        "size_mb": filepath.stat().st_size / 1024 / 1024,
                    })
                except Exception as e:
                    print(f"[CaptureManager] Error reading {filepath}: {e}")

        return self.saved_pointclouds

    def delete_pointcloud(self, filepath: str) -> bool:
        """
        Delete a pointcloud file.

        Args:
            filepath: Path to the file

        Returns:
            bool: True if deleted successfully
        """
        try:
            path = Path(filepath)
            if path.exists():
                path.unlink()
                print(f"[CaptureManager] Deleted: {filepath}")

                # Update saved list
                self.saved_pointclouds = [
                    p for p in self.saved_pointclouds
                    if p['path'] != filepath
                ]
                self._save_metadata()
                return True
        except Exception as e:
            print(f"[CaptureManager] Delete error: {e}")

        return False

    def get_pointcloud_info(self, pcd: o3d.geometry.PointCloud) -> Dict:
        """
        Get point cloud information.

        Args:
            pcd: PointCloud object

        Returns:
            Dict: Info dict
        """
        bbox = pcd.get_axis_aligned_bounding_box()

        return {
            "num_points": len(pcd.points),
            "has_colors": pcd.has_colors(),
            "has_normals": pcd.has_normals(),
            "bounding_box_min": np.asarray(bbox.min_bound).tolist(),
            "bounding_box_max": np.asarray(bbox.max_bound).tolist(),
            "center": np.asarray(pcd.get_center()).tolist(),
        }

    def set_preprocessing_enabled(self, enabled: bool):
        """Enable or disable preprocessing."""
        self.preprocessing_enabled = enabled
        print(f"[CaptureManager] Preprocessing: {'ON' if enabled else 'OFF'}")

    def set_voxel_size(self, voxel_size: float):
        """Set voxel size for downsampling (in meters)."""
        self.voxel_size = voxel_size
        print(f"[CaptureManager] Voxel size: {voxel_size*1000:.1f}mm")

    def _save_metadata(self):
        """Save metadata to JSON file."""
        metadata_path = self.save_directory / "metadata.json"
        try:
            with open(metadata_path, 'w') as f:
                json.dump(self.saved_pointclouds, f, indent=2)
        except Exception as e:
            print(f"[CaptureManager] Metadata save error: {e}")

    def _load_metadata(self):
        """Load metadata from JSON file."""
        metadata_path = self.save_directory / "metadata.json"
        try:
            if metadata_path.exists():
                with open(metadata_path, 'r') as f:
                    self.saved_pointclouds = json.load(f)
        except Exception as e:
            print(f"[CaptureManager] Metadata load error: {e}")

    def cleanup_old_pointclouds(self, max_count: int = 10):
        """
        Delete oldest pointclouds if count exceeds max_count.

        Args:
            max_count: Maximum number of files to keep
        """
        saved_list = self.get_saved_list()

        if len(saved_list) > max_count:
            # Sort by timestamp (oldest first)
            to_delete = sorted(
                saved_list,
                key=lambda x: x['timestamp']
            )[:len(saved_list) - max_count]

            for item in to_delete:
                self.delete_pointcloud(item['path'])
                print(f"[Cleanup] Deleted: {item['filename']}")
