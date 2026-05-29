"""scan_to_pointcloud_node — opt-in utility node.

Subscribes to /scan, converts each valid beam to a 3D point in lidar_link,
transforms the points into base_link using TF, and publishes a
sensor_msgs/PointCloud2 on /scan_points.

Not started by the default minilab launch file; the student opts in by
adding a Node action to their own launch file or running it directly:
    ros2 run my_robot_perception scan_to_pointcloud_node
"""
import math
import struct

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

import tf2_ros
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
from std_msgs.msg import Header

# Registering tf2_geometry_msgs makes do_transform_point work on PointStamped.
import tf2_geometry_msgs  # noqa: F401


def scan_to_points_in_lidar(msg: LaserScan) -> list[tuple[float, float, float]]:
    """Convert a LaserScan to a list of (x, y, z) points in the sensor's frame.

    Points are kept only when the range is finite and inside
    [range_min, range_max]. z is always 0 (2D scanner sweeps a horizontal
    plane).
    """
    pts: list[tuple[float, float, float]] = []
    for i, r in enumerate(msg.ranges):
        if not math.isfinite(r):
            continue
        if r <= msg.range_min or r >= msg.range_max:
            continue
        angle = msg.angle_min + i * msg.angle_increment
        pts.append((r * math.cos(angle), r * math.sin(angle), 0.0))
    return pts


def _pointcloud2_from_points(points, header: Header) -> PointCloud2:
    cloud = PointCloud2()
    cloud.header = header
    cloud.height = 1
    cloud.width = len(points)
    cloud.fields = [
        PointField(name='x', offset=0,  datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4,  datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8,  datatype=PointField.FLOAT32, count=1),
    ]
    cloud.is_bigendian = False
    cloud.point_step = 12
    cloud.row_step = cloud.point_step * cloud.width
    cloud.is_dense = False
    buffer = bytearray()
    for x, y, z in points:
        buffer.extend(struct.pack('<fff', float(x), float(y), float(z)))
    cloud.data = bytes(buffer)
    return cloud


class ScanToPointCloudNode(Node):

    TARGET_FRAME = 'base_link'
    SOURCE_FRAME = 'lidar_link'

    def __init__(self) -> None:
        super().__init__('scan_to_pointcloud_node')

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self._pub = self.create_publisher(PointCloud2, '/scan_points', 10)
        self.create_subscription(
            LaserScan, '/scan', self._scan_cb, qos_profile_sensor_data
        )

        self.get_logger().info(
            f'scan_to_pointcloud_node ready: publishing /scan_points in {self.TARGET_FRAME}'
        )

    def _scan_cb(self, msg: LaserScan) -> None:
        pts_lidar = scan_to_points_in_lidar(msg)
        if not pts_lidar:
            return

        try:
            transform = self._tf_buffer.lookup_transform(
                self.TARGET_FRAME, self.SOURCE_FRAME, rclpy.time.Time(),
                timeout=Duration(seconds=0.1),
            )
        except (tf2_ros.LookupException,
                tf2_ros.ExtrapolationException,
                tf2_ros.ConnectivityException) as exc:
            self.get_logger().warn(
                f'tf {self.SOURCE_FRAME} -> {self.TARGET_FRAME} unavailable: {exc}',
                throttle_duration_sec=2.0,
            )
            return

        import tf2_geometry_msgs as t2g
        out_points: list[tuple[float, float, float]] = []
        ps = PointStamped()
        ps.header.frame_id = self.SOURCE_FRAME
        ps.header.stamp = msg.header.stamp
        for x, y, z in pts_lidar:
            ps.point.x = x
            ps.point.y = y
            ps.point.z = z
            transformed = t2g.do_transform_point(ps, transform)
            out_points.append((
                transformed.point.x, transformed.point.y, transformed.point.z
            ))

        header = Header()
        header.frame_id = self.TARGET_FRAME
        header.stamp = msg.header.stamp
        cloud = _pointcloud2_from_points(out_points, header)
        self._pub.publish(cloud)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ScanToPointCloudNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
