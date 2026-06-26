from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Lidar mounting: base_link -> base_laser_link
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_base_laser_link_broadcaster',
            arguments=['--x', '0.15', '--y', '0.0', '--z', '0.05',
                       '--yaw', '1.57', '--pitch', '0.0', '--roll', '0.0',
                       '--frame-id', 'base_link', '--child-frame-id', 'base_laser_link'],
        ),
        # RealSense D455 mounting: base_link -> camera_link
        # Adjust x/y/z to your actual D455 installation position (meters)
        # x=forward, y=left, z=up relative to base_link (chassis center, ground level)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_camera_link_broadcaster',
            arguments=['--x', '0.10', '--y', '0.0', '--z', '0.20',
                       '--yaw', '0.0', '--pitch', '0.0', '--roll', '0.0',
                       '--frame-id', 'base_link', '--child-frame-id', 'camera_link'],
        ),
        # camera_link -> camera_color_optical_frame
        # ROS optical frame convention: X-right, Y-down, Z-forward
        # Rotation from camera_link (X-fwd, Y-left, Z-up) to optical frame:
        #   roll=-pi/2, pitch=0, yaw=-pi/2
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_link_color_optical_broadcaster',
            arguments=['--x', '0.0', '--y', '0.0', '--z', '0.0',
                       '--yaw', '-1.5708', '--pitch', '0.0', '--roll', '-1.5708',
                       '--frame-id', 'camera_link', '--child-frame-id', 'camera_color_optical_frame'],
        ),
        # camera_link -> camera_depth_optical_frame (same rotation, D455 color-depth are aligned)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_link_depth_optical_broadcaster',
            arguments=['--x', '0.0', '--y', '0.0', '--z', '0.0',
                       '--yaw', '-1.5708', '--pitch', '0.0', '--roll', '-1.5708',
                       '--frame-id', 'camera_link', '--child-frame-id', 'camera_depth_optical_frame'],
        ),
    ])
