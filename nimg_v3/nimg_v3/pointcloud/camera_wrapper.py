"""
RealSense D455 Camera Wrapper with PointCloud Generation

Provides Intel RealSense D455 camera handling with RGB-D streaming
and pointcloud generation capabilities.
"""

import pyrealsense2 as rs
import numpy as np
import open3d as o3d
from typing import Tuple, Optional, Dict


class RealSenseCameraWrapper:
    """Intel RealSense D455 Camera Handler with PointCloud Support"""

    def __init__(self, width: int = 640, height: int = 480, fps: int = 30):
        """
        Initialize RealSense camera wrapper.

        Args:
            width: Frame width (default: 640)
            height: Frame height (default: 480)
            fps: Frames per second (default: 30)
        """
        self.width = width
        self.height = height
        self.fps = fps
        self.pipeline = None
        self.config = None
        self.align = None
        self.depth_scale = None
        self.intrinsics = None
        self.is_running = False

        # RealSense Filters
        self.pc_filter = rs.pointcloud()
        self.decimation_filter = rs.decimation_filter()
        self.spatial_filter = rs.spatial_filter()
        self.temporal_filter = rs.temporal_filter()

        self._configure_filters()

    def _configure_filters(self):
        """Configure RealSense depth filters"""
        # Decimation: 2x downsampling for performance
        self.decimation_filter.set_option(rs.option.filter_magnitude, 2)

        # Spatial: edge-preserving smoothing
        self.spatial_filter.set_option(rs.option.filter_magnitude, 2)
        self.spatial_filter.set_option(rs.option.filter_smooth_alpha, 0.5)
        self.spatial_filter.set_option(rs.option.filter_smooth_delta, 20)

        # Temporal: temporal smoothing for flicker reduction
        self.temporal_filter.set_option(rs.option.filter_smooth_alpha, 0.4)
        self.temporal_filter.set_option(rs.option.filter_smooth_delta, 20)

    def start(self) -> bool:
        """
        Start the camera pipeline.

        Returns:
            bool: True if started successfully, False otherwise
        """
        print("[Camera] Initializing Intel RealSense D455...")

        self.pipeline = rs.pipeline()
        self.config = rs.config()

        # Try multiple configurations
        configs_to_try = [
            (self.width, self.height, self.width, self.height, self.fps, "Requested"),
            (640, 480, 640, 480, 30, "640x480 @ 30fps"),
            (640, 480, 640, 480, 15, "640x480 @ 15fps"),
        ]

        for color_w, color_h, depth_w, depth_h, fps, desc in configs_to_try:
            try:
                self.pipeline = rs.pipeline()
                self.config = rs.config()

                print(f"[Camera] Trying {desc}")

                self.config.enable_stream(
                    rs.stream.color, color_w, color_h, rs.format.bgr8, fps
                )
                self.config.enable_stream(
                    rs.stream.depth, depth_w, depth_h, rs.format.z16, fps
                )

                profile = self.pipeline.start(self.config)

                self.width = color_w
                self.height = color_h
                self.fps = fps

                # Get depth scale
                depth_sensor = profile.get_device().first_depth_sensor()
                self.depth_scale = depth_sensor.get_depth_scale()
                print(f"[Camera] Depth Scale: {self.depth_scale}")

                # Get intrinsics
                color_profile = profile.get_stream(rs.stream.color)
                intr = color_profile.as_video_stream_profile().get_intrinsics()
                self.intrinsics = {
                    'fx': intr.fx,
                    'fy': intr.fy,
                    'cx': intr.ppx,
                    'cy': intr.ppy,
                    'width': intr.width,
                    'height': intr.height
                }
                print(f"[Camera] Intrinsics: fx={intr.fx:.2f}, fy={intr.fy:.2f}")

                self.align = rs.align(rs.stream.color)
                self.is_running = True
                print(f"[Camera] Started successfully: {self.width}x{self.height} @ {self.fps}fps")
                return True

            except Exception as e:
                print(f"[Camera] Config failed: {e}")
                try:
                    self.pipeline.stop()
                except:
                    pass
                continue

        print("[Camera] All configurations failed!")
        return False

    def read(self) -> Tuple[Optional[np.ndarray], Optional[np.ndarray]]:
        """
        Read RGB and Depth frames from camera.

        Returns:
            Tuple[Optional[np.ndarray], Optional[np.ndarray]]:
                (RGB image [H, W, 3] uint8, Depth image [H, W] float32 in meters)
        """
        if not self.is_running:
            return None, None

        try:
            frames = self.pipeline.wait_for_frames(timeout_ms=5000)
            aligned_frames = self.align.process(frames)

            color_frame = aligned_frames.get_color_frame()
            depth_frame = aligned_frames.get_depth_frame()

            if not color_frame or not depth_frame:
                return None, None

            rgb = np.asanyarray(color_frame.get_data())
            depth = np.asanyarray(depth_frame.get_data())
            depth = depth.astype(np.float32) * self.depth_scale

            return rgb, depth

        except Exception as e:
            print(f"[Camera] Read error: {e}")
            return None, None

    def apply_filters(self, depth_frame: rs.depth_frame) -> rs.depth_frame:
        """
        Apply filters to depth frame.

        Args:
            depth_frame: RealSense depth frame

        Returns:
            rs.depth_frame: Filtered depth frame
        """
        depth_frame = self.decimation_filter.process(depth_frame)
        depth_frame = self.spatial_filter.process(depth_frame)
        depth_frame = self.temporal_filter.process(depth_frame)
        return depth_frame

    def get_pointcloud(self, apply_filter: bool = True) -> o3d.geometry.PointCloud:
        """
        Get PointCloud from current frame.

        Args:
            apply_filter: Whether to apply depth filters

        Returns:
            o3d.geometry.PointCloud: RGB-D point cloud
        """
        if not self.is_running:
            raise RuntimeError("Camera is not running")

        try:
            # Get frames
            frames = self.pipeline.wait_for_frames(timeout_ms=5000)
            aligned_frames = self.align.process(frames)

            color_frame = aligned_frames.get_color_frame()
            depth_frame = aligned_frames.get_depth_frame()

            if not color_frame or not depth_frame:
                raise RuntimeError("Failed to get frames")

            # Apply filters
            if apply_filter:
                depth_frame = self.apply_filters(depth_frame)

            # Generate point cloud using RealSense SDK
            self.pc_filter.map_to(color_frame)
            points = self.pc_filter.calculate(depth_frame)

            # Get vertices
            vertices = np.asanyarray(points.get_vertices())
            vertices = np.array([[v[0], v[1], v[2]] for v in vertices], dtype=np.float64)

            # Filter out invalid points (z <= 0)
            valid_mask = vertices[:, 2] > 0
            vertices = vertices[valid_mask]

            # Get RGB colors
            color_data = np.asanyarray(color_frame.get_data())
            tex_coords = np.asanyarray(points.get_texture_coordinates())
            tex_coords = tex_coords[valid_mask]

            # Map colors to vertices
            colors = []
            for tc in tex_coords:
                u = int(tc[0] * self.width)
                v = int(tc[1] * self.height)
                u = np.clip(u, 0, self.width - 1)
                v = np.clip(v, 0, self.height - 1)

                bgr = color_data[v, u]
                rgb = [bgr[2], bgr[1], bgr[0]]  # BGR -> RGB
                colors.append(rgb)

            colors = np.array(colors, dtype=np.float64) / 255.0  # Normalize to [0, 1]

            # Create Open3D PointCloud
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(vertices)
            pcd.colors = o3d.utility.Vector3dVector(colors)

            return pcd

        except Exception as e:
            print(f"[Camera] PointCloud generation error: {e}")
            raise

    def get_pointcloud_from_rgbd(
        self,
        rgb: np.ndarray,
        depth: np.ndarray
    ) -> o3d.geometry.PointCloud:
        """
        Generate PointCloud from RGB and Depth arrays.

        Args:
            rgb: RGB image [H, W, 3] uint8
            depth: Depth image [H, W] float32 (meters)

        Returns:
            o3d.geometry.PointCloud
        """
        # Convert BGR to RGB if needed
        if rgb.shape[2] == 3:
            rgb_converted = rgb[:, :, ::-1].copy()  # BGR to RGB
        else:
            rgb_converted = rgb

        # Convert to Open3D images
        rgb_o3d = o3d.geometry.Image(rgb_converted.astype(np.uint8))
        depth_o3d = o3d.geometry.Image(depth.astype(np.float32))

        rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(
            rgb_o3d, depth_o3d,
            depth_scale=1.0,  # Already in meters
            depth_trunc=5.0,
            convert_rgb_to_intensity=False
        )

        # Camera intrinsics
        intr = self.get_intrinsics()
        intrinsic = o3d.camera.PinholeCameraIntrinsic(
            width=intr['width'],
            height=intr['height'],
            fx=intr['fx'],
            fy=intr['fy'],
            cx=intr['cx'],
            cy=intr['cy']
        )

        pcd = o3d.geometry.PointCloud.create_from_rgbd_image(
            rgbd_image, intrinsic
        )

        return pcd

    def stop(self):
        """Stop the camera pipeline."""
        if self.is_running:
            try:
                self.pipeline.stop()
            except:
                pass
            self.is_running = False
            print("[Camera] Stopped")

    def get_intrinsics(self) -> Dict:
        """
        Get camera intrinsics.

        Returns:
            Dict: Camera intrinsic parameters
        """
        return self.intrinsics if self.intrinsics else {
            'fx': 383.883,
            'fy': 383.883,
            'cx': 320.499,
            'cy': 237.913,
            'width': self.width,
            'height': self.height
        }

    def __del__(self):
        """Destructor to ensure camera is stopped."""
        self.stop()
