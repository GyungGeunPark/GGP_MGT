"""
PointCloud Visualizer using Open3D

Provides 3D visualization capabilities for point clouds.
"""

import open3d as o3d
import numpy as np
from typing import Optional, List


class PointCloudVisualizer:
    """PointCloud Visualizer using Open3D"""

    def __init__(
        self,
        point_size: float = 1.0,
        coordinate_frame_size: float = 0.3,
        background_color: Optional[List[float]] = None
    ):
        """
        Initialize the visualizer.

        Args:
            point_size: Size of points in viewer
            coordinate_frame_size: Size of coordinate frame axes
            background_color: Background color [R, G, B] (0-1)
        """
        self.point_size = point_size
        self.coordinate_frame_size = coordinate_frame_size
        self.background_color = background_color or [0.1, 0.1, 0.1]

    def visualize_pointcloud(
        self,
        pcd: o3d.geometry.PointCloud,
        window_name: str = "PointCloud Viewer",
        show_coordinate_frame: bool = True,
        width: int = 1280,
        height: int = 720
    ):
        """
        Visualize point cloud (blocking).

        Args:
            pcd: Open3D PointCloud
            window_name: Window title
            show_coordinate_frame: Show coordinate axes
            width: Window width
            height: Window height

        Controls:
            - Left mouse drag: Rotate
            - Mouse wheel: Zoom
            - Right mouse drag: Pan
            - Q / ESC: Exit
            - H: Help
            - R: Reset view
            - +/-: Adjust point size
        """
        vis = o3d.visualization.Visualizer()
        vis.create_window(
            window_name=window_name,
            width=width,
            height=height
        )

        vis.add_geometry(pcd)

        if show_coordinate_frame:
            coord_frame = self.create_coordinate_frame(self.coordinate_frame_size)
            vis.add_geometry(coord_frame)

        # Rendering options
        opt = vis.get_render_option()
        opt.point_size = self.point_size
        opt.background_color = np.asarray(self.background_color)
        opt.show_coordinate_frame = show_coordinate_frame

        # Set viewpoint
        self.set_view_point(vis)

        vis.run()
        vis.destroy_window()

    def visualize_multiple(
        self,
        pointclouds: List[o3d.geometry.PointCloud],
        window_name: str = "Multi PointCloud Viewer",
        colors: Optional[List[List[float]]] = None,
        show_coordinate_frame: bool = True
    ):
        """
        Visualize multiple point clouds.

        Args:
            pointclouds: List of PointCloud objects
            window_name: Window title
            colors: List of colors for each pointcloud
            show_coordinate_frame: Show coordinate axes
        """
        vis = o3d.visualization.Visualizer()
        vis.create_window(window_name=window_name, width=1280, height=720)

        for i, pcd in enumerate(pointclouds):
            # Apply color if specified
            if colors and i < len(colors):
                pcd_colored = o3d.geometry.PointCloud(pcd)
                pcd_colored.paint_uniform_color(colors[i])
                vis.add_geometry(pcd_colored)
            else:
                vis.add_geometry(pcd)

        if show_coordinate_frame:
            coord_frame = self.create_coordinate_frame(self.coordinate_frame_size)
            vis.add_geometry(coord_frame)

        opt = vis.get_render_option()
        opt.point_size = self.point_size
        opt.background_color = np.asarray(self.background_color)

        self.set_view_point(vis)

        vis.run()
        vis.destroy_window()

    def create_coordinate_frame(
        self,
        size: float = 0.3,
        origin: List[float] = None
    ) -> o3d.geometry.TriangleMesh:
        """
        Create coordinate frame.

        Args:
            size: Axis length (meters)
            origin: Origin point [x, y, z]

        Returns:
            o3d.geometry.TriangleMesh: Coordinate frame
            - X axis: Red
            - Y axis: Green
            - Z axis: Blue
        """
        frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=size)

        if origin:
            frame.translate(origin)

        return frame

    def set_view_point(self, vis: o3d.visualization.Visualizer):
        """
        Set camera viewpoint.

        Default View:
            - Position: [0, 0, -1.5] (behind camera)
            - Look at: [0, 0, 0] (origin)
            - Up: [0, -1, 0] (Y-axis down)
        """
        ctr = vis.get_view_control()
        ctr.set_lookat([0, 0, 0])
        ctr.set_front([0, 0, 1])
        ctr.set_up([0, -1, 0])
        ctr.set_zoom(0.8)

    def save_screenshot(
        self,
        pcd: o3d.geometry.PointCloud,
        output_path: str,
        width: int = 1920,
        height: int = 1080
    ):
        """
        Save a screenshot of the point cloud.

        Args:
            pcd: PointCloud to render
            output_path: Path to save the image
            width: Image width
            height: Image height
        """
        vis = o3d.visualization.Visualizer()
        vis.create_window(
            window_name="Screenshot",
            width=width,
            height=height,
            visible=False
        )

        vis.add_geometry(pcd)

        opt = vis.get_render_option()
        opt.point_size = self.point_size
        opt.background_color = np.asarray(self.background_color)

        self.set_view_point(vis)

        vis.poll_events()
        vis.update_renderer()
        vis.capture_screen_image(output_path)
        vis.destroy_window()

        print(f"[Visualizer] Screenshot saved: {output_path}")

    def set_point_size(self, size: float):
        """Set point size for rendering."""
        self.point_size = size

    def set_background_color(self, color: List[float]):
        """Set background color [R, G, B] (0-1)."""
        self.background_color = color


def quick_visualize(
    pcd: o3d.geometry.PointCloud,
    window_name: str = "Quick View"
):
    """
    Quick visualization helper function.

    Args:
        pcd: PointCloud to visualize
        window_name: Window title
    """
    visualizer = PointCloudVisualizer()
    visualizer.visualize_pointcloud(pcd, window_name=window_name)


def quick_visualize_file(
    filepath: str,
    window_name: str = None
):
    """
    Quick visualization from file.

    Args:
        filepath: Path to .ply or .pcd file
        window_name: Window title
    """
    pcd = o3d.io.read_point_cloud(filepath)

    if window_name is None:
        import os
        window_name = os.path.basename(filepath)

    quick_visualize(pcd, window_name)
