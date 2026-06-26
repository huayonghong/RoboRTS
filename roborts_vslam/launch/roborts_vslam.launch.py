"""
Full RoboRTS + VSLAM Launch File (RealSense D455)

Launches the complete robot stack with RGB-D VSLAM localization:
  - Base driver + static TF
  - RealSense D455 (color + depth + IMU)
  - RPLidar (for costmap / obstacle avoidance)
  - RTAB-Map localization + RGB-D odometry
  - EKF fusion (wheel + visual odometry)
  - Global + Local planners

Workflow:
  1. First, build a map:  ros2 launch roborts_vslam vslam_mapping.launch.py
  2. Then, use this launch for navigation with VSLAM localization.

Usage:
  ros2 launch roborts_vslam roborts_vslam.launch.py
"""
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    bringup_dir = get_package_share_directory('roborts_bringup')
    vslam_dir = get_package_share_directory('roborts_vslam')

    base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, 'launch', 'base.launch.py')))

    static_tf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, 'launch', 'static_tf.launch.py')))

    vslam_localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(vslam_dir, 'launch', 'vslam_localization.launch.py')))

    rplidar_node = Node(
        package='rplidar_ros',
        executable='rplidar_node',
        name='rplidar_node',
        output='screen',
        parameters=[{
            'serial_port': '/dev/rplidar',
            'serial_baudrate': 115200,
            'frame_id': 'base_laser_link',
            'inverted': False,
            'angle_compensate': True,
        }],
    )

    global_planner = Node(
        package='roborts_planning',
        executable='global_planner_node',
        name='global_planner_node',
        output='screen',
    )

    local_planner = Node(
        package='roborts_planning',
        executable='local_planner_node',
        name='local_planner_node',
        output='screen',
    )

    return LaunchDescription([
        base_launch,
        static_tf_launch,
        rplidar_node,
        vslam_localization_launch,
        global_planner,
        local_planner,
    ])
