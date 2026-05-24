import math
from nav_msgs.msg import Odometry


def get_position(msg: Odometry) -> tuple:
    """Return (x, y) position from an Odometry message."""
    return (msg.pose.pose.position.x, msg.pose.pose.position.y)


def get_yaw(msg: Odometry) -> float:
    """Extract yaw (radians) from the quaternion in an Odometry message."""
    q = msg.pose.pose.orientation
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)
