#!/usr/bin/env python3
"""On-demand visualization for the gimbal perception pipeline.

Subscribes to the annotation stream published by ``person_gesture_node``
(``publish_debug_image:=true``), plus the person / gesture / lock / follow /
gimbal topics, and draws a HUD on top:

  * person id, distance, yaw/pitch and score;
  * the locked target highlighted in red;
  * gesture boxes and names;
  * follow state and current gimbal command.

The annotated frame is published on ``/perception/debug/overlay`` for RViz or
rqt_image_view.  With ``show_window:=true`` it also opens a local OpenCV window
(requires a display).

This node is debug-only.  It is meant to run *instead of* the normal perception
launch when you need to look at the stream; it does not open the camera itself.
"""

import collections
import os
import time

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, UInt32

from roborts_msgs.msg import GestureArray, GimbalAngle, PersonArray

FONT = cv2.FONT_HERSHEY_SIMPLEX


def _text(image, text, origin, scale=0.55, color=(255, 255, 255), thickness=1):
    """Outlined text so it stays readable on any background."""
    cv2.putText(image, text, origin, FONT, scale, (0, 0, 0),
                thickness + 2, cv2.LINE_AA)
    cv2.putText(image, text, origin, FONT, scale, color, thickness,
                cv2.LINE_AA)


class PerceptionView(Node):
    def __init__(self):
        super().__init__('perception_view')

        image_topic = self.declare_parameter(
            'image_topic', '/perception/debug/image_raw').value
        output_topic = self.declare_parameter(
            'output_topic', '/perception/debug/overlay').value
        publish_overlay = bool(
            self.declare_parameter('publish_overlay', True).value)
        show_window = bool(
            self.declare_parameter('show_window', False).value)
        window_name = self.declare_parameter(
            'window_name', 'perception').value
        self._data_timeout = float(
            self.declare_parameter('data_timeout', 0.5).value)

        self._bridge = CvBridge()
        self._publish_overlay = publish_overlay
        self._window_name = window_name
        self._show_window = show_window
        if self._show_window and not os.environ.get('DISPLAY'):
            self.get_logger().warn(
                'show_window requested but DISPLAY is not set; disabling the '
                'local window (the overlay topic still works)')
            self._show_window = False

        image_qos = QoSProfile(
            depth=1, history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.BEST_EFFORT)
        # The input image is best-effort (sensor data).  The overlay is a
        # debug stream consumed by RViz's Image display, which subscribes
        # RELIABLE; depth 1 keeps only the newest frame.
        overlay_qos = QoSProfile(
            depth=1, history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(
            Image, image_topic, self._on_image, image_qos)
        self.create_subscription(
            PersonArray, '/perception/persons', self._on_persons, 10)
        self.create_subscription(
            GestureArray, '/perception/gestures', self._on_gestures, 10)
        self.create_subscription(
            UInt32, '/perception/lock_target', self._on_lock, 1)
        self.create_subscription(
            Bool, '/perception/follow_enable', self._on_follow, 1)
        self.create_subscription(
            GimbalAngle, '/cmd_gimbal_angle', self._on_gimbal, 1)

        self._overlay_pub = None
        if self._publish_overlay:
            self._overlay_pub = self.create_publisher(
                Image, output_topic, overlay_qos)

        self._persons = []
        self._gestures = []
        self._persons_time = 0.0
        self._gestures_time = 0.0
        self._lock_id = 0
        self._follow_enabled = False
        self._gimbal_yaw = None
        self._gimbal_pitch = None
        self._gimbal_time = 0.0
        self._frame_times = collections.deque()
        self._window_ready = False

        self.get_logger().info(
            f'perception_view: {image_topic} -> '
            f'{output_topic if publish_overlay else "(window only)"}'
            f'{", show_window" if self._show_window else ""}')

    # ------------------------------------------------------------- callbacks
    def _on_persons(self, message):
        self._persons = message.persons
        self._persons_time = time.monotonic()

    def _on_gestures(self, message):
        self._gestures = message.gestures
        self._gestures_time = time.monotonic()

    def _on_lock(self, message):
        self._lock_id = message.data

    def _on_follow(self, message):
        self._follow_enabled = message.data

    def _on_gimbal(self, message):
        self._gimbal_yaw = message.yaw_angle
        self._gimbal_pitch = message.pitch_angle
        self._gimbal_time = time.monotonic()

    def _on_image(self, message):
        try:
            image = self._bridge.imgmsg_to_cv2(message, 'bgr8')
        except Exception as error:  # cv_bridge can reject malformed frames
            self.get_logger().warn(f'cannot convert image: {error}')
            return

        annotated = image.copy()
        now = time.monotonic()
        self._update_fps(now)
        self._draw(annotated, now)

        if self._overlay_pub is not None:
            output = self._bridge.cv2_to_imgmsg(annotated, 'bgr8')
            output.header = message.header
            self._overlay_pub.publish(output)

        if self._show_window:
            self._show(annotated)

    # --------------------------------------------------------------- drawing
    def _update_fps(self, now):
        self._frame_times.append(now)
        while self._frame_times and now - self._frame_times[0] > 1.0:
            self._frame_times.popleft()

    @property
    def _fps(self):
        return float(len(self._frame_times))

    def _draw(self, image, now):
        persons = self._persons if now - self._persons_time \
            <= self._data_timeout else []
        gestures = self._gestures if now - self._gestures_time \
            <= self._data_timeout else []
        gimbal_valid = self._gimbal_yaw is not None and \
            now - self._gimbal_time <= 1.0

        height, width = image.shape[:2]
        for person in persons:
            self._draw_person(image, person, width, height)
        for gesture in gestures:
            self._draw_gesture(image, gesture, width, height)
        self._draw_hud(image, persons, gestures, gimbal_valid)

    def _box(self, center, size, width, height):
        x1 = max(0, int(center.x - size.x * 0.5))
        y1 = max(0, int(center.y - size.y * 0.5))
        x2 = min(width - 1, int(center.x + size.x * 0.5))
        y2 = min(height - 1, int(center.y + size.y * 0.5))
        return x1, y1, x2, y2

    def _draw_person(self, image, person, width, height):
        x1, y1, x2, y2 = self._box(person.center, person.size, width, height)
        if x2 <= x1 or y2 <= y1:
            return
        locked = self._lock_id != 0 and person.id == self._lock_id
        color = (0, 0, 255) if locked else (0, 220, 0)
        cv2.rectangle(image, (x1, y1), (x2, y2), color, 3 if locked else 1)
        if locked:
            _text(image, 'LOCK', (x1, max(16, y1 - 8)), 0.6, color, 2)
        label = (f'#{person.id} {person.distance:.1f}m '
                 f'yaw{person.yaw * 57.2958:+.0f} '
                 f'pitch{person.pitch * 57.2958:+.0f}')
        _text(image, label, (x1, min(height - 6, y2 + 18)), 0.5, color)

    def _draw_gesture(self, image, gesture, width, height):
        x1, y1, x2, y2 = self._box(gesture.center, gesture.size, width, height)
        if x2 <= x1 or y2 <= y1:
            return
        color = (255, 255, 0)
        cv2.rectangle(image, (x1, y1), (x2, y2), color, 2)
        label = f'{gesture.name} {gesture.score:.2f} (#{gesture.track_id})'
        _text(image, label, (x1, max(16, y1 - 8)), 0.6, color, 2)

    def _draw_hud(self, image, persons, gestures, gimbal_valid):
        gesture_text = 'gesture: -'
        if gestures:
            best = max(gestures, key=lambda item: item.score)
            gesture_text = f'gesture: {best.name} {best.score:.2f}'
        lock_text = f'lock: #{self._lock_id}' if self._lock_id else 'lock: -'
        follow_text = 'follow: ON' if self._follow_enabled else 'follow: OFF'
        if gimbal_valid:
            gimbal_text = (f'gimbal yaw {self._gimbal_yaw * 57.2958:+.1f} '
                           f'pitch {self._gimbal_pitch * 57.2958:+.1f}')
        else:
            gimbal_text = 'gimbal: n/a'

        lines = [
            f'PERCEPTION  {self._fps:4.1f} FPS',
            f'persons {len(persons)}   {lock_text}   {follow_text}',
            gimbal_text,
            gesture_text,
        ]
        scale = 0.55
        panel_width = max(
            cv2.getTextSize(line, FONT, scale, 1)[0][0] for line in lines) + 24
        panel_height = 14 + len(lines) * 22
        panel = image.copy()
        cv2.rectangle(panel, (8, 8), (8 + panel_width, 8 + panel_height),
                      (0, 0, 0), -1)
        cv2.addWeighted(panel, 0.55, image, 0.45, 0, image)
        for index, line in enumerate(lines):
            color = (255, 255, 255)
            if index == 0:
                color = (0, 255, 0)
            elif index == 1:
                color = (0, 255, 0) if self._follow_enabled else (120, 120, 255)
            _text(image, line, (20, 32 + index * 22), scale, color)

    def _show(self, image):
        try:
            if not self._window_ready:
                cv2.namedWindow(self._window_name, cv2.WINDOW_NORMAL)
                self._window_ready = True
            cv2.imshow(self._window_name, image)
            if (cv2.waitKey(1) & 0xFF) in (27, ord('q')):
                self.get_logger().info('closing window; Ctrl+C to exit')
                self._show_window = False
                cv2.destroyWindow(self._window_name)
        except cv2.error as error:
            self.get_logger().warn(
                f'cannot open display window ({error}); continuing headless')
            self._show_window = False

    def shutdown(self):
        if self._window_ready:
            cv2.destroyAllWindows()
            self._window_ready = False


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = PerceptionView()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.shutdown()
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
