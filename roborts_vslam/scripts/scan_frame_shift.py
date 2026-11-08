#!/usr/bin/env python3
"""Re-express a LaserScan in a rotated frame by shifting its angles.

``depthimage_to_laserscan`` does NOT transform points into ``output_frame``;
it only overwrites ``header.frame_id``.  Its angles are computed in the
camera convention (0 rad = forward, positive = left), which matches
``base_link`` because the D455 is mounted with zero yaw.

The ROBORTS costmap, however, ignores the scan's header and always interprets
the rays in ``base_laser_link`` -- configured as ``sensor_frame`` in
``obstacle_layer_config.prototxt``.  On this robot that frame is rotated +90
degrees from ``base_link`` (``static_tf.launch.py``), so publishing the raw
camera scan on /scan would rotate every obstacle by 90 degrees in the costmap.

A uniform angular shift of -pi/2 converts the camera-convention scan into
base_laser_link without reordering the ranges array (index i still maps to
``angle_min + i * angle_increment``).

Parameters:
  input_topic   (str,   /scan_depth_raw)  scan in the camera convention
  output_topic  (str,   /scan)            scan in base_laser_link
  angle_offset  (float, -1.5708)          angle shift [rad]
  frame_id      (str,   base_laser_link)  header.frame_id of the output
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import LaserScan


class ScanFrameShift(Node):
    def __init__(self):
        super().__init__('scan_frame_shift')

        self.declare_parameter('input_topic', '/scan_depth_raw')
        self.declare_parameter('output_topic', '/scan')
        self.declare_parameter('angle_offset', -1.5708)
        self.declare_parameter('frame_id', 'base_laser_link')

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        self._angle_offset = float(self.get_parameter('angle_offset').value)
        self._frame_id = self.get_parameter('frame_id').value

        # The output publisher is RELIABLE because the costmap obstacle layer
        # subscribes with rclcpp::QoS(50) (reliable).  The input is
        # BEST_EFFORT (qos_profile_sensor_data) so it works with either
        # depthimage_to_laserscan (reliable) or pointcloud_to_laserscan
        # (SensorDataQoS / best effort).
        self._publisher = self.create_publisher(LaserScan, output_topic, 10)
        self._subscription = self.create_subscription(
            LaserScan, input_topic, self._on_scan, qos_profile_sensor_data)

        self.get_logger().info(
            f'scan_frame_shift: {input_topic} -> {output_topic} '
            f'(offset {self._angle_offset:+.4f} rad, frame "{self._frame_id}")')

    def _on_scan(self, msg):
        out = LaserScan()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self._frame_id

        out.angle_min = msg.angle_min + self._angle_offset
        out.angle_max = msg.angle_max + self._angle_offset
        out.angle_increment = msg.angle_increment
        out.time_increment = msg.time_increment
        out.scan_time = msg.scan_time
        out.range_min = msg.range_min
        out.range_max = msg.range_max
        # A uniform shift keeps the range at index i attached to the same
        # physical direction.
        out.ranges = list(msg.ranges)
        out.intensities = list(msg.intensities)

        self._publisher.publish(out)


def main():
    rclpy.init()
    node = ScanFrameShift()
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
