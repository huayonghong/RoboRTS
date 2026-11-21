from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # The CSI camera is mounted on the gimbal, so its link hangs off the
    # dynamic `gimbal` frame published by roborts_base.  These offsets are
    # measured from the gimbal pivot: x forward, y left, z up (metres).
    # Adjust them on the robot -- they are launch arguments so a bringup file
    # can override them without editing this file.
    csi_camera_x = DeclareLaunchArgument(
        'csi_camera_x', default_value='0.03',
        description='CSI camera x offset from the gimbal pivot [m]')
    csi_camera_y = DeclareLaunchArgument(
        'csi_camera_y', default_value='0.0',
        description='CSI camera y offset from the gimbal pivot [m]')
    csi_camera_z = DeclareLaunchArgument(
        'csi_camera_z', default_value='0.07',
        description='CSI camera z offset from the gimbal pivot [m]')
    csi_camera_roll = DeclareLaunchArgument(
        'csi_camera_roll', default_value='0.0',
        description='CSI camera roll in gimbal_link [rad]')
    csi_camera_pitch = DeclareLaunchArgument(
        'csi_camera_pitch', default_value='0.0',
        description='CSI camera pitch in gimbal_link [rad]')
    csi_camera_yaw = DeclareLaunchArgument(
        'csi_camera_yaw', default_value='0.0',
        description='CSI camera yaw in gimbal_link [rad]')

    return LaunchDescription([
        csi_camera_x,
        csi_camera_y,
        csi_camera_z,
        csi_camera_roll,
        csi_camera_pitch,
        csi_camera_yaw,
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
        # Gimbal CSI camera mounting: gimbal -> csi_camera_link
        # `gimbal` is published dynamically by roborts_base (base_link -> gimbal),
        # so the camera follows the gimbal yaw/pitch automatically.
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='gimbal_csi_camera_link_broadcaster',
            arguments=['--x', LaunchConfiguration('csi_camera_x'),
                       '--y', LaunchConfiguration('csi_camera_y'),
                       '--z', LaunchConfiguration('csi_camera_z'),
                       '--yaw', LaunchConfiguration('csi_camera_yaw'),
                       '--pitch', LaunchConfiguration('csi_camera_pitch'),
                       '--roll', LaunchConfiguration('csi_camera_roll'),
                       '--frame-id', 'gimbal',
                       '--child-frame-id', 'csi_camera_link'],
        ),
        # csi_camera_link -> csi_camera_optical_frame (REP-103)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='csi_camera_link_optical_broadcaster',
            arguments=['--x', '0.0', '--y', '0.0', '--z', '0.0',
                       '--yaw', '-1.5708', '--pitch', '0.0', '--roll', '-1.5708',
                       '--frame-id', 'csi_camera_link',
                       '--child-frame-id', 'csi_camera_optical_frame'],
        ),
    ])
