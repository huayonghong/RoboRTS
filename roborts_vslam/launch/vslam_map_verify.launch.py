"""
VSLAM Map Verification

Loads a pre-built RTAB-Map database and visualizes in RViz2 for
verifying the map quality before using it for navigation.

Two modes:
  - offline (default): Loads the database and publishes the stored map
    WITHOUT starting the camera. Useful for quickly viewing the map.
  - online: Also starts the camera + localization so you can drive the
    robot through the map and verify real-time re-localization.

Usage:
  # Offline verification (just view the map):
  ros2 launch roborts_vslam vslam_map_verify.launch.py

  # Online verification (camera + re-localization):
  ros2 launch roborts_vslam vslam_map_verify.launch.py online:=true

  # Use a non-default database path:
  ros2 launch roborts_vslam vslam_map_verify.launch.py db_path:=/path/to/rtabmap.db
"""
import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    vslam_dir = get_package_share_directory('roborts_vslam')
    bringup_dir = get_package_share_directory('roborts_bringup')
    default_db = os.path.join(os.path.expanduser('~'), '.ros', 'rtabmap.db')

    db_path_arg = DeclareLaunchArgument(
        'db_path', default_value=default_db,
        description='Path to the RTAB-Map database file to verify')

    online_arg = DeclareLaunchArgument(
        'online', default_value='false',
        description='If true, start camera and localization for live verification')

    camera_ns_arg = DeclareLaunchArgument(
        'camera_ns', default_value='/camera/camera',
        description='RealSense topic prefix (see vslam_mapping.launch.py)')

    static_tf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, 'launch', 'static_tf.launch.py')))

    # Online mode: full localization stack with camera
    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(vslam_dir, 'launch', 'vslam_localization.launch.py')),
        launch_arguments=[
            ('camera_ns', LaunchConfiguration('camera_ns')),
        ],
        condition=IfCondition(LaunchConfiguration('online')),
    )

    # Offline mode: load database and publish stored map data without camera
    rtabmap_offline = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        condition=UnlessCondition(LaunchConfiguration('online')),
        parameters=[{
            'subscribe_depth': False,
            'subscribe_rgb': False,
            'subscribe_odom': False,
            'frame_id': 'base_link',
            'odom_frame_id': 'vo',
            'map_frame_id': 'map',
            'database_path': LaunchConfiguration('db_path'),
            'Mem/IncrementalMemory': 'false',
            'Mem/InitWMWithAllNodes': 'true',
            'Reg/Force3DoF': 'true',
        }],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(vslam_dir, 'config', 'vslam_mapping.rviz')],
    )

    # Trigger map publication once rtabmap loads the database
    trigger_map = ExecuteProcess(
        cmd=['bash', '-c',
             'sleep 5 && '
             'ros2 service call /rtabmap/publish_map '
             'rtabmap_msgs/srv/PublishMap '
             '"{global_map: true, optimized: true, graph_only: false}"'],
        output='screen',
        condition=UnlessCondition(LaunchConfiguration('online')),
    )

    return LaunchDescription([
        db_path_arg,
        online_arg,
        camera_ns_arg,
        static_tf_launch,
        rtabmap_offline,
        localization_launch,
        rviz_node,
        trigger_map,
    ])
