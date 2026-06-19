#!/usr/bin/env python3
"""scan_fusion_node — fuse the two front-corner LiDARs into one cloud.

The arm mounts just ahead of base_link, occluding the
forward sector of any centre-mounted LiDAR. Two LiDARs at the front corners
(lidar_left_link / lidar_right_link) each see across the arm's blind wedge, so
fusing them gives a complete forward+side view. Each scan is projected into
base_link via TF and the points are concatenated into a single PointCloud2 on
/scan_points (base_link), published on a fixed timer.
"""
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

import tf2_ros
import tf2_geometry_msgs  # noqa: F401  (registers do_transform_point for PointStamped)
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import LaserScan, PointCloud2
from std_msgs.msg import Header

from my_robot_perception.scan_utils import scan_to_points_in_lidar, pointcloud2_from_points


class ScanFusionNode(Node):

    TARGET_FRAME = 'base_link'
    # Chassis half-extents (0.30 x 0.20 box) + a small margin. Returns inside
    # this rectangle in base_link are self-hits (the robot's own body) and are
    # dropped before publishing. The scan plane (z=0.125) sits above the wheel
    # tops (0.10) and below the arm shoulder (0.15), so neither wheels nor arm
    # appear -- this footprint filter only needs to clear the chassis itself.
    FOOTPRINT_X = 0.17
    FOOTPRINT_Y = 0.12

    def __init__(self) -> None:
        super().__init__('scan_fusion_node')

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self._scans: dict[str, LaserScan] = {}

        self.create_subscription(LaserScan, '/scan_left', self._on_scan_left,
                                 qos_profile_sensor_data)
        self.create_subscription(LaserScan, '/scan_right', self._on_scan_right,
                                 qos_profile_sensor_data)

        self._pub = self.create_publisher(PointCloud2, '/scan_points', 10)
        self.create_timer(0.05, self._publish_fused)  # 20 Hz

        self.get_logger().info(
            f'scan_fusion_node ready: fusing /scan_left + /scan_right '
            f'-> /scan_points in {self.TARGET_FRAME}')

    def _on_scan_left(self, msg: LaserScan) -> None:
        self._scans['left'] = msg

    def _on_scan_right(self, msg: LaserScan) -> None:
        self._scans['right'] = msg

    def _project(self, msg: LaserScan) -> list[tuple[float, float, float]]:
        pts = scan_to_points_in_lidar(msg)
        if not pts:
            return []
        source = msg.header.frame_id
        try:
            transform = self._tf_buffer.lookup_transform(
                self.TARGET_FRAME, source, rclpy.time.Time(),
                timeout=Duration(seconds=0.1))
        except (tf2_ros.LookupException, tf2_ros.ExtrapolationException,
                tf2_ros.ConnectivityException) as exc:
            self.get_logger().warn(
                f'tf {source} -> {self.TARGET_FRAME} unavailable: {exc}',
                throttle_duration_sec=2.0)
            return []
        out: list[tuple[float, float, float]] = []
        ps = PointStamped()
        ps.header.frame_id = source
        ps.header.stamp = msg.header.stamp
        for x, y, z in pts:
            ps.point.x, ps.point.y, ps.point.z = x, y, z
            t = tf2_geometry_msgs.do_transform_point(ps, transform)
            out.append((t.point.x, t.point.y, t.point.z))
        return out

    def _publish_fused(self) -> None:
        if not self._scans:
            return
        merged: list[tuple[float, float, float]] = []
        for msg in self._scans.values():
            merged.extend(self._project(msg))
        # Drop self-hits: points inside the chassis footprint (base_link frame).
        merged = [(x, y, z) for (x, y, z) in merged
                  if abs(x) >= self.FOOTPRINT_X or abs(y) >= self.FOOTPRINT_Y]
        if not merged:
            return
        header = Header()
        header.frame_id = self.TARGET_FRAME
        header.stamp = self.get_clock().now().to_msg()
        self._pub.publish(pointcloud2_from_points(merged, header))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ScanFusionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
