#!/usr/bin/env python3
"""Bridge /goal_pose to the two-stage ROBORTS planner.

Flow:
  /goal_pose (PoseStamped)
      -> GlobalPlanner action (A*)
      -> first feedback path
      -> LocalPlanner action (TEB)

Differences from roborts_decision::ChassisExecutor, which this replaces for
the VSLAM navigation stack:

* One local goal per user goal.  TEB copies its pending plan into the active
  plan every cycle (teb_local_planner.cpp: GetPlan), so a single send is
  enough.  A bounded resend happens only if the local action aborts.
* A generation counter guards callbacks: feedback/result belonging to an old
  goal can never trigger motion for a new one.
* A new goal cancels the local goal first (stop the robot immediately), then
  the global goal, using chained futures.  Nothing blocks inside a callback,
  so a SingleThreadedExecutor cannot deadlock.
* The global planner is not called until map->base_link exists.  Its plan
  thread busy-waits for the robot pose while holding the costmap mutex
  (global_planner_node.cpp PlanThread), so sending a goal before RTAB-Map has
  relocalized would stall the costmap.
* With respect_goal_orientation:=false (default) only the goal position is
  used: the last pose handed to TEB finishes facing the direction of travel,
  not the heading RViz attaches to a 2D Goal Pose click.  A heading dragged
  in RViz was forcing TEB to rotate on the spot (often by ~180 deg) before or
  after driving.  Set respect_goal_orientation:=true to restore the old
  behaviour.
* The global planner uses its own A* path (whose last pose carries the goal
  orientation) for the success check, so the global goal is cancelled once
  TEB reports success.
"""

import copy
import math

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.time import Duration, Time

import tf2_ros

from roborts_msgs.action import GlobalPlanner, LocalPlanner


class GoalBridge(Node):
    def __init__(self):
        super().__init__('vslam_goal_bridge')

        self.declare_parameter('goal_topic', '/goal_pose')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('local_retries', 2)
        self.declare_parameter('tf_wait_timeout', 60.0)
        self.declare_parameter('respect_goal_orientation', False)
        # Refuse to start until map -> odom has stopped moving: RTAB-Map
        # refines its initial correction right after relocalization (observed
        # as a ~45 deg step), and a goal sent during that step makes TEB turn
        # to chase a target that moved.
        self.declare_parameter('tf_settle_time', 1.5)
        self.declare_parameter('tf_settle_tolerance', 0.05)

        goal_topic = self.get_parameter('goal_topic').value
        self._map_frame = self.get_parameter('map_frame').value
        self._base_frame = self.get_parameter('base_frame').value
        self._max_local_retries = int(self.get_parameter('local_retries').value)
        self._tf_wait_timeout = float(
            self.get_parameter('tf_wait_timeout').value)
        self._respect_goal_orientation = bool(
            self.get_parameter('respect_goal_orientation').value)
        self._settle_time = float(self.get_parameter('tf_settle_time').value)
        self._settle_tolerance = float(
            self.get_parameter('tf_settle_tolerance').value)

        self._global_client = ActionClient(
            self, GlobalPlanner, '/global_planner_node_action')
        self._local_client = ActionClient(
            self, LocalPlanner, '/local_planner_node_action')

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        # State guarded by the generation counter.
        self._gen = 0
        self._goal_msg = None
        self._global_handle = None
        self._local_handle = None
        self._local_sent = False
        self._last_path = None
        self._local_retries_left = 0
        self._pending_timer = None
        self._pending_since = None

        self.create_subscription(PoseStamped, goal_topic, self._on_goal, 10)

        self.get_logger().info('waiting for planner action servers ...')
        self._global_client.wait_for_server()
        self._local_client.wait_for_server()
        self.get_logger().info(
            f'ready. publish PoseStamped to {goal_topic}')

    # ------------------------------------------------------------------ goal
    def _on_goal(self, msg):
        self._gen += 1
        gen = self._gen
        self._cancel_pending_timer()

        goal = copy.deepcopy(msg)
        if not goal.header.frame_id:
            goal.header.frame_id = self._map_frame
        elif goal.header.frame_id != self._map_frame:
            self.get_logger().warn(
                f'goal frame_id is "{goal.header.frame_id}", expected '
                f'"{self._map_frame}"; the global planner will try to '
                f'transform it')

        self._goal_msg = goal
        self._local_sent = False
        self._last_path = None
        self._local_retries_left = self._max_local_retries
        self._pending_since = None

        p = goal.pose.position
        self.get_logger().info(
            f'new goal: ({p.x:.2f}, {p.y:.2f}) frame={goal.header.frame_id}')

        # Cancel the local goal first so the chassis stops before a new plan
        # is produced, then cancel the global goal, then start planning.
        local_handle = self._local_handle
        self._local_handle = None
        if local_handle is not None:
            local_handle.cancel_goal_async().add_done_callback(
                lambda _future: self._cancel_global_then_start(gen))
        else:
            self._cancel_global_then_start(gen)

    def _cancel_global_then_start(self, gen):
        if gen != self._gen:
            return
        global_handle = self._global_handle
        self._global_handle = None
        if global_handle is not None:
            global_handle.cancel_goal_async().add_done_callback(
                lambda _future: self._start(gen))
        else:
            self._start(gen)

    # ----------------------------------------------------------- global plan
    def _start(self, gen):
        if gen != self._gen:
            return

        if not self._ready_to_plan():
            if self._pending_since is None:
                self._pending_since = self.get_clock().now()
                self._pending_timer = self.create_timer(
                    0.5, lambda: self._retry_start(gen))
                self.get_logger().warn(
                    f'waiting for a stable {self._map_frame}->'
                    f'{self._base_frame} (RTAB-Map relocalization) before '
                    f'sending the goal')
            return

        self._cancel_pending_timer()
        self._send_global(gen)

    def _ready_to_plan(self):
        if not self._tf_buffer.can_transform(
                self._map_frame, self._base_frame, Time()):
            return False
        return self._localization_stable()

    def _localization_stable(self):
        """True when map -> odom has not moved for tf_settle_time seconds.

        The TF buffer keeps history, so compare the current correction with
        the one from tf_settle_time ago; a goal sent while it is still being
        refined makes TEB chase a target that moves under it.
        """
        if self._settle_time <= 0.0:
            return True
        try:
            current = self._tf_buffer.lookup_transform(
                self._map_frame, 'odom', Time())
            previous = self._tf_buffer.lookup_transform(
                self._map_frame, 'odom',
                self.get_clock().now() - Duration(seconds=self._settle_time))
        except Exception:
            # No history yet (or no odom frame): do not block the goal.
            return True

        dx = current.transform.translation.x - previous.transform.translation.x
        dy = current.transform.translation.y - previous.transform.translation.y
        if math.hypot(dx, dy) > self._settle_tolerance:
            return False

        def yaw(q):
            return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                              1.0 - 2.0 * (q.y * q.y + q.z * q.z))

        delta = yaw(current.transform.rotation) - yaw(previous.transform.rotation)
        delta = math.atan2(math.sin(delta), math.cos(delta))
        return abs(delta) <= self._settle_tolerance

    def _retry_start(self, gen):
        if gen != self._gen or self._goal_msg is None:
            self._cancel_pending_timer()
            return
        if self._ready_to_plan():
            self._cancel_pending_timer()
            self._send_global(gen)
            return
        if self._pending_since is not None:
            waited = (self.get_clock().now() - self._pending_since).nanoseconds
            if waited > int(self._tf_wait_timeout * 1e9):
                self._cancel_pending_timer()
                self.get_logger().error(
                    f'timed out waiting for {self._map_frame}->'
                    f'{self._base_frame}; goal dropped')

    def _send_global(self, gen):
        goal = GlobalPlanner.Goal()
        goal.command = 0
        goal.goal = self._goal_msg

        self._global_client.send_goal_async(
            goal,
            feedback_callback=lambda feedback: self._on_feedback(gen, feedback),
        ).add_done_callback(
            lambda future: self._on_global_accepted(gen, future))

    def _on_global_accepted(self, gen, future):
        handle = future.result()
        if gen != self._gen:
            handle.cancel_goal_async()
            return
        if not handle.accepted:
            self.get_logger().error('global goal rejected')
            return
        self._global_handle = handle
        handle.get_result_async().add_done_callback(
            lambda result: self._on_global_result(gen, result))

    def _on_feedback(self, gen, feedback_msg):
        if gen != self._gen:
            return
        feedback = feedback_msg.feedback
        if feedback.error_code != 0:
            self.get_logger().warn(
                f'global feedback error {feedback.error_code}: '
                f'{feedback.error_msg}')
        if not feedback.path.poses:
            return

        path = copy.deepcopy(feedback.path)
        self._last_path = path
        if self._local_sent:
            return

        self._local_sent = True
        self.get_logger().info(
            f'forwarding path with {len(path.poses)} poses')
        self._send_local(gen, self._with_goal_orientation(path))

    def _on_global_result(self, gen, future):
        if gen != self._gen:
            return
        status = future.result().status
        self._global_handle = None
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('global planner reports goal reached')
        elif status in (GoalStatus.STATUS_ABORTED,
                        GoalStatus.STATUS_CANCELED):
            self.get_logger().warn(
                f'global goal finished with status {status}')
            local_handle = self._local_handle
            self._local_handle = None
            if local_handle is not None:
                local_handle.cancel_goal_async()
        else:
            self.get_logger().info(
                f'global goal finished with status {status}')

    # ------------------------------------------------------------ local plan
    def _send_local(self, gen, path):
        goal = LocalPlanner.Goal()
        goal.route = path
        self._local_handle = None
        self._local_client.send_goal_async(goal).add_done_callback(
            lambda future: self._on_local_accepted(gen, future))

    def _on_local_accepted(self, gen, future):
        handle = future.result()
        if gen != self._gen:
            handle.cancel_goal_async()
            return
        if not handle.accepted:
            self.get_logger().error('local goal rejected')
            self._maybe_retry_local(gen)
            return
        self._local_handle = handle
        handle.get_result_async().add_done_callback(
            lambda result: self._on_local_result(gen, result))

    def _on_local_result(self, gen, future):
        if gen != self._gen:
            return
        status = future.result().status
        self._local_handle = None
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('reached the goal')
            # The global planner checks the goal pose in its own A* path,
            # which has yaw 0; when the user asked for another heading it
            # would keep RUNNING forever.  Stop it.
            global_handle = self._global_handle
            self._global_handle = None
            if global_handle is not None:
                self.get_logger().info(
                    'canceling global goal after local success')
                global_handle.cancel_goal_async()
        elif status in (GoalStatus.STATUS_ABORTED,
                        GoalStatus.STATUS_CANCELED):
            self.get_logger().warn(
                f'local goal finished with status {status}')
            self._maybe_retry_local(gen)

    def _maybe_retry_local(self, gen):
        if (gen != self._gen or self._last_path is None
                or self._local_retries_left <= 0
                or self._global_handle is None):
            return
        self._local_retries_left -= 1
        self.get_logger().info(
            f'resending local goal ({self._local_retries_left} retries left)')
        path = self._with_goal_orientation(copy.deepcopy(self._last_path))
        self._send_local(gen, path)

    def _with_goal_orientation(self, path):
        """Set the final pose heading of the path handed to TEB.

        Default (respect_goal_orientation=false): finish facing the direction
        of travel.  RViz's 2D Goal Pose always publishes a heading, and using
        it made TEB rotate in place for many seconds before/after driving.
        """
        if self._goal_msg is None or not path.poses:
            return path
        if self._respect_goal_orientation:
            path.poses[-1].pose.orientation = copy.deepcopy(
                self._goal_msg.pose.orientation)
            return path

        yaw = self._approach_yaw(path)
        if yaw is not None:
            q = path.poses[-1].pose.orientation
            q.x = 0.0
            q.y = 0.0
            q.z = math.sin(yaw * 0.5)
            q.w = math.cos(yaw * 0.5)
        return path

    def _approach_yaw(self, path):
        """Heading of the last >=0.2 m path segment, in the path frame.

        Returns None when the path is too short to give a direction.
        """
        last = path.poses[-1].pose.position
        for pose in reversed(path.poses[:-1]):
            dx = last.x - pose.pose.position.x
            dy = last.y - pose.pose.position.y
            if math.hypot(dx, dy) >= 0.2:
                return math.atan2(dy, dx)

        # Position-only goal essentially on top of the robot: keep the
        # current heading so TEB is not asked to turn.
        try:
            transform = self._tf_buffer.lookup_transform(
                self._map_frame, self._base_frame, Time())
        except Exception:
            return None
        q = transform.transform.rotation
        return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                          1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    # ---------------------------------------------------------------- helpers
    def _cancel_pending_timer(self):
        # cancel() only: this may be called from inside the timer callback,
        # where destroy_timer() can be unsafe.
        if self._pending_timer is not None:
            self._pending_timer.cancel()
            self._pending_timer = None
        self._pending_since = None

    def destroy_node(self):
        self._cancel_pending_timer()
        if self._local_handle is not None:
            self._local_handle.cancel_goal_async()
            self._local_handle = None
        if self._global_handle is not None:
            self._global_handle.cancel_goal_async()
            self._global_handle = None
        super().destroy_node()


def main():
    rclpy.init()
    node = GoalBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
