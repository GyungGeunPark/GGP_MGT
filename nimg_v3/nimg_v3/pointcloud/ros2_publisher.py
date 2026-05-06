"""
ROS2 PointCloud Publisher

Publishes Open3D PointCloud as sensor_msgs/PointCloud2 message.
"""

import numpy as np
import open3d as o3d
import struct
from typing import Optional

# Try to import ROS2 libraries
try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import PointCloud2, PointField
    from std_msgs.msg import Header
    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False
    print("[ROS2Publisher] rclpy not available. ROS2 features disabled.")


class ROS2PointCloudPublisher:
    """ROS2 PointCloud2 Publisher (Singleton)"""

    _instance = None
    _initialized = False

    def __new__(cls, *args, **kwargs):
        if cls._instance is None:
            cls._instance = super().__new__(cls)
        return cls._instance

    def __init__(
        self,
        topic_name: str = "pc_point",
        frame_id: str = "camera_link"
    ):
        """
        Initialize the ROS2 publisher.

        Args:
            topic_name: ROS2 topic name
            frame_id: Frame ID for the pointcloud
        """
        if ROS2PointCloudPublisher._initialized:
            return

        self.topic_name = topic_name
        self.frame_id = frame_id
        self.node = None
        self.publisher = None
        self.is_initialized = False

        if not ROS2_AVAILABLE:
            print("[ROS2Publisher] ROS2 not available")
            return

        ROS2PointCloudPublisher._initialized = True

    def init_ros2_node(self) -> bool:
        """
        Initialize ROS2 node.

        Returns:
            bool: Success
        """
        if not ROS2_AVAILABLE:
            print("[ROS2Publisher] rclpy not available")
            return False

        try:
            if not rclpy.ok():
                rclpy.init()

            self.node = rclpy.create_node('pointcloud_publisher')
            self.publisher = self.node.create_publisher(
                PointCloud2,
                self.topic_name,
                10  # QoS depth
            )

            self.is_initialized = True
            print(f"[ROS2Publisher] Initialized: topic='{self.topic_name}'")
            return True

        except Exception as e:
            print(f"[ROS2Publisher] Init failed: {e}")
            return False

    def publish_pointcloud(self, pcd: o3d.geometry.PointCloud) -> bool:
        """
        Publish point cloud to ROS2 topic.

        Args:
            pcd: Open3D PointCloud

        Returns:
            bool: Success
        """
        if not self.is_initialized:
            print("[ROS2Publisher] Not initialized!")
            return False

        try:
            msg = self.convert_to_pointcloud2_msg(pcd)
            self.publisher.publish(msg)

            rclpy.spin_once(self.node, timeout_sec=0.01)

            print(f"[ROS2Publisher] Published {len(pcd.points)} points to '{self.topic_name}'")
            return True

        except Exception as e:
            print(f"[ROS2Publisher] Publish failed: {e}")
            return False

    def convert_to_pointcloud2_msg(
        self,
        pcd: o3d.geometry.PointCloud
    ) -> 'PointCloud2':
        """
        Convert Open3D PointCloud to sensor_msgs/PointCloud2.

        Args:
            pcd: Open3D PointCloud

        Returns:
            sensor_msgs.msg.PointCloud2
        """
        points = np.asarray(pcd.points)
        colors = np.asarray(pcd.colors) if pcd.has_colors() else None

        msg = PointCloud2()

        # Header
        msg.header = Header()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id

        # Dimensions
        msg.height = 1
        msg.width = len(points)
        msg.is_dense = False
        msg.is_bigendian = False

        # Fields: x, y, z, rgb
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='rgb', offset=12, datatype=PointField.UINT32, count=1),
        ]
        msg.point_step = 16  # 16 bytes per point
        msg.row_step = msg.point_step * msg.width

        # Pack data using numpy for performance
        data = self._pack_pointcloud_data(points, colors)
        msg.data = data

        return msg

    def _pack_pointcloud_data(
        self,
        points: np.ndarray,
        colors: Optional[np.ndarray]
    ) -> bytes:
        """
        Pack pointcloud data into binary format.

        Args:
            points: Nx3 array of xyz coordinates
            colors: Nx3 array of rgb colors (0-1) or None

        Returns:
            bytes: Packed binary data
        """
        n_points = len(points)

        # Create buffer
        buffer = []
        for i in range(n_points):
            x, y, z = points[i]

            if colors is not None:
                r, g, b = (colors[i] * 255).astype(np.uint8)
                rgb = struct.unpack('I', struct.pack('BBBB', b, g, r, 0))[0]
            else:
                rgb = 0

            buffer.append(struct.pack('fffI', x, y, z, rgb))

        return b''.join(buffer)

    def shutdown(self):
        """Shutdown ROS2 node."""
        if self.is_initialized and self.node is not None:
            try:
                self.node.destroy_node()
                rclpy.shutdown()
            except:
                pass
            self.is_initialized = False
            ROS2PointCloudPublisher._initialized = False
            print("[ROS2Publisher] Shutdown")

    def is_ready(self) -> bool:
        """Check if publisher is ready."""
        return self.is_initialized

    @staticmethod
    def is_ros2_available() -> bool:
        """Check if ROS2 is available."""
        return ROS2_AVAILABLE

    def get_topic_name(self) -> str:
        """Get current topic name."""
        return self.topic_name

    def set_frame_id(self, frame_id: str):
        """Set frame ID for published messages."""
        self.frame_id = frame_id

    def __del__(self):
        """Destructor to ensure proper shutdown."""
        self.shutdown()


def create_test_pointcloud() -> o3d.geometry.PointCloud:
    """Create a test pointcloud for debugging."""
    # Create a simple colored cube point cloud
    points = []
    colors = []

    for x in np.linspace(-0.5, 0.5, 20):
        for y in np.linspace(-0.5, 0.5, 20):
            for z in np.linspace(0, 1, 20):
                points.append([x, y, z])
                colors.append([
                    (x + 0.5),
                    (y + 0.5),
                    z
                ])

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(np.array(points))
    pcd.colors = o3d.utility.Vector3dVector(np.array(colors))

    return pcd


if __name__ == "__main__":
    # Test code
    if ROS2_AVAILABLE:
        print("Testing ROS2 PointCloud Publisher...")

        publisher = ROS2PointCloudPublisher(topic_name="pc_point")

        if publisher.init_ros2_node():
            pcd = create_test_pointcloud()
            publisher.publish_pointcloud(pcd)
            print("Published test pointcloud!")
            publisher.shutdown()
    else:
        print("ROS2 is not available. Cannot run test.")
