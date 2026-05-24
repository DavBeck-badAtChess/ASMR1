import math
import pytest
from nav_msgs.msg import Odometry
from my_robot_perception.odom_utils import get_position, get_yaw


def make_odom(x, y, yaw):
    msg = Odometry()
    msg.pose.pose.position.x = x
    msg.pose.pose.position.y = y
    msg.pose.pose.orientation.w = math.cos(yaw / 2)
    msg.pose.pose.orientation.x = 0.0
    msg.pose.pose.orientation.y = 0.0
    msg.pose.pose.orientation.z = math.sin(yaw / 2)
    return msg


def test_get_position():
    msg = make_odom(1.5, -2.3, 0.0)
    assert get_position(msg) == pytest.approx((1.5, -2.3))


def test_get_position_origin():
    msg = make_odom(0.0, 0.0, 0.0)
    assert get_position(msg) == pytest.approx((0.0, 0.0))


def test_get_yaw_zero():
    msg = make_odom(0.0, 0.0, 0.0)
    assert get_yaw(msg) == pytest.approx(0.0, abs=1e-9)


def test_get_yaw_90():
    msg = make_odom(0.0, 0.0, math.pi / 2)
    assert get_yaw(msg) == pytest.approx(math.pi / 2, abs=1e-6)


def test_get_yaw_negative():
    msg = make_odom(0.0, 0.0, -math.pi / 4)
    assert get_yaw(msg) == pytest.approx(-math.pi / 4, abs=1e-6)
