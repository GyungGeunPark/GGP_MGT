#!/usr/bin/env python3
"""
PointCloud Capture & Publish Interface
Main Entry Point

FurSys AI Team - nimg_v3
Intel RealSense D455 based pointcloud capture, visualization, and ROS2 publishing.

Usage:
    python pointcloud_interface.py [--save-dir PATH]

Controls:
    - Capture: Capture current pointcloud and save to file
    - Publish: Publish selected pointcloud to ROS2 topic (pc_point)
    - Visualize: Open 3D viewer for selected pointcloud
    - Preprocessing: Toggle noise removal and downsampling
"""

import sys
import os
import argparse
from pathlib import Path

# Add nimg_v3 to path
current_dir = Path(__file__).parent.resolve()
nimg_v3_root = current_dir.parent.parent
sys.path.insert(0, str(nimg_v3_root))

from PyQt5.QtWidgets import QApplication
from nimg_v3.pointcloud.gui import PointCloudInterfaceGUI


def parse_args():
    """Parse command line arguments."""
    parser = argparse.ArgumentParser(
        description="PointCloud Capture & Publish Interface",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
    python pointcloud_interface.py
    python pointcloud_interface.py --save-dir /path/to/data

Controls:
    - Capture button: Capture pointcloud and save as .ply
    - Publish button: Publish selected file to ROS2 topic 'pc_point'
    - Visualize button: Open Open3D 3D viewer
    - Preprocessing checkbox: Enable/disable noise removal and downsampling

ROS2 Integration:
    Topic: pc_point (sensor_msgs/PointCloud2)
    Frame ID: camera_link

    To verify:
        ros2 topic list
        ros2 topic echo /pc_point --no-arr
        rviz2 (Add PointCloud2, topic: /pc_point)
        """
    )

    parser.add_argument(
        '--save-dir',
        type=str,
        default=None,
        help='Directory to save pointcloud files (default: ./pointcloud_data)'
    )

    parser.add_argument(
        '--no-ros2',
        action='store_true',
        help='Disable ROS2 integration'
    )

    return parser.parse_args()


def main():
    """Main entry point."""
    args = parse_args()

    print("=" * 60)
    print("PointCloud Capture & Publish Interface")
    print("FurSys AI Team - nimg_v3")
    print("=" * 60)
    print()
    print("Starting application...")
    print()

    # Determine save directory
    if args.save_dir:
        save_dir = args.save_dir
    else:
        save_dir = str(nimg_v3_root / "pointcloud_data")

    print(f"Save directory: {save_dir}")
    print()

    # Create Qt application
    app = QApplication(sys.argv)
    app.setStyle('Fusion')

    # Create and show main window
    window = PointCloudInterfaceGUI(save_dir=save_dir)
    window.show()

    print("Application started. Use the GUI to capture and publish pointclouds.")
    print("Close the window or press Ctrl+C to exit.")
    print()

    # Run event loop
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
