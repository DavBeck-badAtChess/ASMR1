import math

import pytest
from sensor_msgs.msg import LaserScan

from my_robot_perception.scan_to_pointcloud_node import scan_to_points_in_lidar


def make_scan(ranges, range_min=0.1, range_max=10.0):
    msg = LaserScan()
    msg.angle_min = -math.pi
    msg.angle_max = math.pi
    msg.angle_increment = 2 * math.pi / len(ranges)
    msg.range_min = range_min
    msg.range_max = range_max
    msg.ranges = list(ranges)
    return msg


def test_scan_to_points_all_invalid_returns_empty():
    ranges = [float('inf')] * 360
    msg = make_scan(ranges)
    pts = scan_to_points_in_lidar(msg)
    assert pts == []


def test_scan_to_points_forward_beam_correct_cartesian():
    ranges = [float('inf')] * 360
    ranges[180] = 1.0  # angle = 0 (forward in lidar frame)
    msg = make_scan(ranges)
    pts = scan_to_points_in_lidar(msg)
    assert len(pts) == 1
    x, y, z = pts[0]
    assert x == pytest.approx(1.0, abs=1e-6)
    assert y == pytest.approx(0.0, abs=1e-6)
    assert z == pytest.approx(0.0)


def test_scan_to_points_left_beam_y_positive():
    ranges = [float('inf')] * 360
    # quarter way around: index 270 -> angle = -pi + 270 * 2pi/360 = pi/2 (left)
    ranges[270] = 1.0
    msg = make_scan(ranges)
    pts = scan_to_points_in_lidar(msg)
    assert len(pts) == 1
    x, y, z = pts[0]
    assert x == pytest.approx(0.0, abs=1e-6)
    assert y == pytest.approx(1.0, abs=1e-6)


def test_scan_to_points_filters_out_of_range():
    ranges = [float('inf')] * 360
    ranges[180] = 0.05  # below range_min
    msg = make_scan(ranges)
    pts = scan_to_points_in_lidar(msg)
    assert pts == []


def test_scan_to_points_multiple_valid():
    ranges = [float('inf')] * 360
    ranges[180] = 1.0
    ranges[270] = 2.0
    msg = make_scan(ranges)
    pts = scan_to_points_in_lidar(msg)
    assert len(pts) == 2
