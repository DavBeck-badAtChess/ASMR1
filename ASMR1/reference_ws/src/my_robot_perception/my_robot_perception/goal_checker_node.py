import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, HistoryPolicy, ReliabilityPolicy

from geometry_msgs.msg import Point, PointStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, ColorRGBA, Header
from visualization_msgs.msg import Marker

from my_robot_perception.odom_utils import get_position


def distance_to_goal(robot_x: float, robot_y: float, goal_x: float, goal_y: float) -> float:
    """Euclidean distance from robot position to goal."""
    return math.hypot(robot_x - goal_x, robot_y - goal_y)


class GoalLatch:
    """One-shot 'goal reached' state.

    Transitions from False to True the first time the robot enters the goal
    region (strict `<` against threshold). Stays True afterwards regardless of
    where the robot goes next.
    """

    def __init__(self, threshold: float) -> None:
        self._threshold = threshold
        self._reached = False

    @property
    def reached(self) -> bool:
        return self._reached

    def update(self, robot_x: float, robot_y: float,
               goal_x: float, goal_y: float) -> bool:
        """Update the latch with the latest robot pose.

        Returns True only on the transition False -> True (the single firing edge).
        Returns False on every subsequent call.
        """
        if self._reached:
            return False
        if distance_to_goal(robot_x, robot_y, goal_x, goal_y) < self._threshold:
            self._reached = True
            return True
        return False


class GoalCheckerNode(Node):

    def __init__(self) -> None:
        super().__init__('goal_checker_node')

        self.declare_parameter('goal_x', 0.0)
        self.declare_parameter('goal_y', 0.0)
        self.declare_parameter('goal_threshold', 0.3)

        self._goal_x = self.get_parameter('goal_x').value
        self._goal_y = self.get_parameter('goal_y').value
        self._threshold = self.get_parameter('goal_threshold').value
        self._latch = GoalLatch(self._threshold)
        self._start_time = self.get_clock().now()

        latched_qos = QoSProfile(
            depth=1,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._goal_pub = self.create_publisher(Bool, '/goal_reached', latched_qos)
        self._goal_pub.publish(Bool(data=False))

        # Latched goal coordinate for nav nodes (single source of truth: nav
        # nodes subscribe to this rather than duplicating the parameter).
        self._point_pub = self.create_publisher(PointStamped, '/goal_point', latched_qos)
        point = PointStamped()
        point.header = Header(frame_id='odom')
        point.point = Point(x=float(self._goal_x), y=float(self._goal_y), z=0.0)
        self._point_pub.publish(point)

        # Latched goal marker for RViz (fixed in world frame, frame_id=odom so
        # it doesn't move with the robot). One-shot publish on a latched topic.
        self._marker_pub = self.create_publisher(Marker, '/goal_marker', latched_qos)
        self._marker_pub.publish(self._build_marker())

        self.create_subscription(Odometry, '/odom', self._odom_cb, 10)

        self.get_logger().info(
            f'goal_checker_node: watching for goal at '
            f'({self._goal_x:.2f}, {self._goal_y:.2f}) '
            f'with threshold {self._threshold:.2f} m'
        )

    def _build_marker(self) -> Marker:
        m = Marker()
        m.header = Header(frame_id='odom')
        m.ns = 'goal'
        m.id = 0
        m.type = Marker.CYLINDER
        m.action = Marker.ADD
        m.pose.position = Point(
            x=float(self._goal_x), y=float(self._goal_y), z=0.15,
        )
        m.pose.orientation.w = 1.0
        # Disk: diameter = 2 × threshold so the marker visually equals the
        # goal-reached radius. Height 0.3 m for clear visibility in RViz.
        m.scale.x = 2.0 * self._threshold
        m.scale.y = 2.0 * self._threshold
        m.scale.z = 0.3
        m.color = ColorRGBA(r=0.0, g=1.0, b=1.0, a=0.5)
        return m

    def _odom_cb(self, msg: Odometry) -> None:
        x, y = get_position(msg)
        just_transitioned = self._latch.update(x, y, self._goal_x, self._goal_y)
        if just_transitioned:
            elapsed = (self.get_clock().now() - self._start_time).nanoseconds / 1e9
            self.get_logger().info(
                f'GOAL REACHED at ({x:.2f}, {y:.2f}) after {elapsed:.1f} s'
            )
            self._goal_pub.publish(Bool(data=True))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GoalCheckerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
