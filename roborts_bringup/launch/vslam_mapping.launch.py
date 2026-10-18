"""
RoboRTS VSLAM Mapping Launch File

Launches the robot base with RTAB-Map mapping using RealSense D455.
Use this mode to build a visual map before switching to vslam.launch.py.

Usage:
  ros2 launch roborts_bringup vslam_mapping.launch.py
  ros2 launch roborts_bringup vslam_mapping.launch.py camera_ns:=/camera
"""
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
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

    vslam_mapping_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(vslam_dir, 'launch', 'vslam_mapping.launch.py')))

    return LaunchDescription([
        base_launch,
        static_tf_launch,
        vslam_mapping_launch,
    ])
