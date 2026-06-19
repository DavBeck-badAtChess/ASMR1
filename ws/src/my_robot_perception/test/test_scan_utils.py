import math
import pytest
import numpy as np
from sensor_msgs.msg import LaserScan
from my_robot_perception.scan_utils import (
    sector_min, any_below, cloud_sector_min,
)


def make_scan(ranges, range_min=0.1, range_max=10.0):
    msg = LaserScan()
    msg.angle_min = -math.pi
    msg.angle_max = math.pi
    msg.angle_increment = 2 * math.pi / len(ranges)
    msg.range_min = range_min
    msg.range_max = range_max
    msg.ranges = list(ranges)
    return msg


def test_sector_min_forward():
    ranges = [5.0] * 360
    # index for angle 0 (forward): (0 - (-pi)) / (2pi/360) = 180
    ranges[180] = 0.5
    msg = make_scan(ranges)
    assert sector_min(msg, -30.0, 30.0) == pytest.approx(0.5)


def test_sector_min_no_valid_readings():
    ranges = [float('inf')] * 360
    msg = make_scan(ranges)
    assert sector_min(msg, -30.0, 30.0) == float('inf')


def test_sector_min_ignores_out_of_range():
    ranges = [float('inf')] * 360
    ranges[180] = 0.05  # below range_min=0.1
    msg = make_scan(ranges)
    assert sector_min(msg, -30.0, 30.0) == float('inf')


def test_sector_min_right_side():
    # right side: negative angles, index ~90 (angle = -pi/2)
    ranges = [5.0] * 360
    ranges[90] = 0.3  # angle = -pi + 90*(2pi/360) = -pi/2 (right)
    msg = make_scan(ranges)
    assert sector_min(msg, -90.0, -30.0) == pytest.approx(0.3)


def test_any_below_true():
    ranges = [5.0] * 360
    ranges[100] = 0.2
    msg = make_scan(ranges)
    assert any_below(msg, 0.25) is True


def test_any_below_false():
    ranges = [5.0] * 360
    msg = make_scan(ranges)
    assert any_below(msg, 0.25) is False


def test_any_below_ignores_invalid():
    ranges = [5.0] * 360
    ranges[100] = 0.05  # below range_min
    msg = make_scan(ranges)
    assert any_below(msg, 0.25) is False


def test_cloud_sector_min_forward():
    xy = np.array([[0.5, 0.0], [2.0, 0.0], [0.0, 1.0]])  # fwd 0.5 & 2.0, left 1.0
    assert cloud_sector_min(xy, -30.0, 30.0) == pytest.approx(0.5)


def test_cloud_sector_min_empty():
    assert cloud_sector_min(np.empty((0, 2)), -30.0, 30.0) == float('inf')


def test_cloud_sector_min_no_points_in_sector():
    xy = np.array([[0.0, 1.0]])  # +90 deg, outside the forward sector
    assert cloud_sector_min(xy, -30.0, 30.0) == float('inf')


def test_cloud_sector_min_right_side():
    xy = np.array([[0.0, -0.3]])  # -90 deg, range 0.3
    assert cloud_sector_min(xy, -120.0, -60.0) == pytest.approx(0.3)
