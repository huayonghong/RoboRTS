from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_base_laser_link_broadcaster',
            arguments=['--x', '0.15', '--y', '0.0', '--z', '0.05',
                       '--yaw', '1.57', '--pitch', '0.0', '--roll', '0.0',
                       '--frame-id', 'base_link', '--child-frame-id', 'base_laser_link'],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_camera_link_broadcaster',
            arguments=['--x', '0.0', '--y', '0.0', '--z', '0.0',
                       '--yaw', '0.0', '--pitch', '0.0', '--roll', '0.0',
                       '--frame-id', 'base_link', '--child-frame-id', 'camera'],
        ),
    ])
