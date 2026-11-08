#!/usr/bin/env python3
"""Republish a latched occupancy grid as a periodic volatile map.

The ROBORTS costmap static layer subscribes to its map topic with
``rclcpp::QoS(1)`` -- RELIABLE + VOLATILE.  Both ``nav2_map_server`` and
RTAB-Map publish their grid as TRANSIENT_LOCAL latched messages, which a
volatile subscriber only receives if it happens to be alive at publish time.
The global planner starts concurrently with the map source, so whether it
ever sees the map is a race; if it loses the race it blocks forever inside
``StaticLayer::OnInitialize()``.

This node subscribes with TRANSIENT_LOCAL (so it always gets the latched
map, even when started later) and republishes it on the downstream topic at
a fixed rate.  The downstream subscriber is volatile but will see the next
tick within one period.

Parameters:
  input_topic   (str,   /map_raw) topic to subscribe to
  output_topic  (str,   /map)     topic to publish on
  publish_rate  (float, 1.0)      republish rate [Hz]
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from nav_msgs.msg import OccupancyGrid


class MapRelay(Node):
    def __init__(self):
        super().__init__('map_relay')

        self.declare_parameter('input_topic', '/map_raw')
        self.declare_parameter('output_topic', '/map')
        self.declare_parameter('publish_rate', 1.0)

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        publish_rate = float(self.get_parameter('publish_rate').value)
        if publish_rate <= 0.0:
            self.get_logger().warn(
                f'publish_rate={publish_rate} is invalid, using 1.0 Hz')
            publish_rate = 1.0

        # TRANSIENT_LOCAL so a late-joining relay still receives the latched
        # grid from map_server / rtabmap.
        input_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)

        self._last_map = None
        self._subscription = self.create_subscription(
            OccupancyGrid, input_topic, self._on_map, input_qos)
        # Default publisher QoS is RELIABLE + VOLATILE, which is what the
        # costmap static layer subscribes with.
        self._publisher = self.create_publisher(OccupancyGrid, output_topic, 1)
        self._timer = self.create_timer(1.0 / publish_rate, self._tick)

        self.get_logger().info(
            f'map_relay: {input_topic} (transient_local) -> {output_topic} '
            f'@ {publish_rate:.1f} Hz')

    def _on_map(self, msg):
        if self._last_map is None:
            self.get_logger().info(
                f'received map {msg.info.width}x{msg.info.height} '
                f'@ {msg.info.resolution:.3f} m/cell, frame "{msg.header.frame_id}"')
        self._last_map = msg

    def _tick(self):
        if self._last_map is not None:
            self._publisher.publish(self._last_map)


def main():
    rclpy.init()
    node = MapRelay()
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
