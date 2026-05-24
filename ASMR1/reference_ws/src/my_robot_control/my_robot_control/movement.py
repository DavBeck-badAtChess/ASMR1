import time

from geometry_msgs.msg import Twist


def drive(publisher, linear_x: float, angular_z: float, duration_s: float) -> None:
    msg = Twist()
    msg.linear.x = float(linear_x)
    msg.angular.z = float(angular_z)
    publisher.publish(msg)
    time.sleep(duration_s)


def stop(publisher) -> None:
    publisher.publish(Twist())
