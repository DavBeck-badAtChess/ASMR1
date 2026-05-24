"""Velocity controller — hosts the SetVelocity action server.

During MiniLab 1 this node owns the /cmd_vel publisher. The reactive
navigator (my_robot_nav) sends SetVelocity action goals to this node; on
each goal accept, we publish the goal's (linear_x, angular_z) to /cmd_vel
at 10 Hz until the goal is preempted (a new goal arrives) or cancelled.

Two callback groups so the timer doesn't deadlock with action callbacks
when the executor is single-threaded; we also use MultiThreadedExecutor
defensively. ReentrantCallbackGroup is appropriate because both callbacks
are short and stateless w.r.t. each other.
"""
import threading

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from geometry_msgs.msg import Twist
from my_robot_interfaces.action import SetVelocity


class VelocityState:
    """Pure-Python current-velocity state held by the controller.

    No rclpy dependency so the transitions can be unit-tested.
    """

    def __init__(self) -> None:
        self._linear_x: float = 0.0
        self._angular_z: float = 0.0
        self._active: bool = False

    @property
    def linear_x(self) -> float:
        return self._linear_x

    @property
    def angular_z(self) -> float:
        return self._angular_z

    @property
    def active(self) -> bool:
        return self._active

    def set_goal(self, linear_x: float, angular_z: float) -> None:
        self._linear_x = float(linear_x)
        self._angular_z = float(angular_z)
        self._active = True

    def clear(self) -> None:
        self._linear_x = 0.0
        self._angular_z = 0.0
        self._active = False


class VelocityControllerNode(Node):
    """Hosts /set_velocity. Sole publisher of /cmd_vel during MiniLab 1.

    On each accepted goal, internal state is updated to the goal's
    (linear_x, angular_z). A 10 Hz timer publishes a Twist with those
    values to /cmd_vel until a new goal preempts this one or the client
    cancels (which publishes a single zero Twist).
    """

    TICK_HZ = 10.0

    def __init__(self) -> None:
        super().__init__('velocity_controller_node')

        self._state = VelocityState()
        self._state_lock = threading.Lock()
        self._active_goal_handle: ServerGoalHandle | None = None

        cb_group = ReentrantCallbackGroup()

        self._cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self._action_server = ActionServer(
            self,
            SetVelocity,
            '/set_velocity',
            execute_callback=self._execute,
            goal_callback=self._on_goal_request,
            cancel_callback=self._on_cancel_request,
            callback_group=cb_group,
        )
        self._timer = self.create_timer(
            1.0 / self.TICK_HZ, self._tick, callback_group=cb_group
        )

        self.get_logger().info(
            'velocity_controller_node ready: /set_velocity action server up'
        )

    def _on_goal_request(self, goal_request) -> GoalResponse:
        # Always accept. Active goal (if any) is preempted in _execute.
        return GoalResponse.ACCEPT

    def _on_cancel_request(self, goal_handle: ServerGoalHandle) -> CancelResponse:
        return CancelResponse.ACCEPT

    def _execute(self, goal_handle: ServerGoalHandle):
        # Atomically: capture previous handle, install ourselves, update state.
        with self._state_lock:
            previous_handle = self._active_goal_handle
            self._active_goal_handle = goal_handle
            self._state.set_goal(
                linear_x=goal_handle.request.linear_x,
                angular_z=goal_handle.request.angular_z,
            )

        # Preemption is done outside the lock to avoid holding it while the
        # action library does its own internal bookkeeping on abort().
        if previous_handle is not None and previous_handle.is_active:
            try:
                previous_handle.abort()
            except Exception:
                # The handle may have transitioned to terminal between is_active
                # check and abort() call. Safe to ignore.
                pass

        feedback = SetVelocity.Feedback()
        # Spin until cancel or preemption (handle becomes inactive).
        rate = self.create_rate(self.TICK_HZ)
        while rclpy.ok() and goal_handle.is_active:
            if goal_handle.is_cancel_requested:
                with self._state_lock:
                    self._state.clear()
                    if self._active_goal_handle is goal_handle:
                        self._active_goal_handle = None
                self._publish_zero_once()
                goal_handle.canceled()
                return SetVelocity.Result(stopped=True)

            with self._state_lock:
                feedback.current_linear_x = self._state.linear_x
                feedback.current_angular_z = self._state.angular_z
            goal_handle.publish_feedback(feedback)
            rate.sleep()

        # Handle was aborted by a newer goal: the new execute_callback will
        # have replaced the state already; we do not zero /cmd_vel here.
        with self._state_lock:
            if self._active_goal_handle is goal_handle:
                self._active_goal_handle = None
        return SetVelocity.Result(stopped=False)

    def _tick(self) -> None:
        with self._state_lock:
            active = self._state.active
            vx = self._state.linear_x
            wz = self._state.angular_z
        if not active:
            return
        cmd = Twist()
        cmd.linear.x = vx
        cmd.angular.z = wz
        self._cmd_pub.publish(cmd)

    def _publish_zero_once(self) -> None:
        self._cmd_pub.publish(Twist())

    def destroy_node(self) -> bool:
        # Defensive: publish zero on shutdown.
        self._publish_zero_once()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VelocityControllerNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
