"""Show the gimbal perception stream with tracking / gesture / target data.

Starts ``person_gesture_node`` with the debug image enabled, plus
``perception_view`` which overlays person ids, distance/yaw/pitch, the locked
target, gestures, follow state and the current gimbal command, and publishes
the result on /perception/debug/overlay.

Display options (pick one, or both):
  use_rviz:=true      RViz2 Image display with config/perception.rviz (default)
  show_window:=true   local OpenCV window (needs DISPLAY)

`rqt_image_view` also works on the overlay topic:
  ros2 run rqt_image_view rqt_image_view /perception/debug/overlay

Usage:
  ros2 launch roborts_perception perception_view.launch.py
  ros2 launch roborts_perception perception_view.launch.py show_window:=true
  ros2 launch roborts_perception perception_view.launch.py use_rviz:=false \\
      show_window:=true
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package_dir = get_package_share_directory('roborts_perception')
    rviz_config = os.path.join(package_dir, 'config', 'perception.rviz')

    debug_image_fps = DeclareLaunchArgument(
        'debug_image_fps', default_value='15.0',
        description='Debug/overlay image rate')
    enable_gesture = DeclareLaunchArgument(
        'enable_gesture', default_value='false',
        description='Run the hand-pose gesture branch')
    show_window = DeclareLaunchArgument(
        'show_window', default_value='false',
        description='Open a local OpenCV window (needs DISPLAY)')
    use_rviz = DeclareLaunchArgument(
        'use_rviz', default_value='true',
        description='Open RViz2 with config/perception.rviz')

    perception = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(package_dir, 'launch', 'person_gesture.launch.py')),
        launch_arguments={
            'publish_debug_image': 'true',
            'debug_image_fps': LaunchConfiguration('debug_image_fps'),
            'enable_gesture': LaunchConfiguration('enable_gesture'),
        }.items(),
    )

    viewer = Node(
        package='roborts_perception',
        executable='perception_view',
        name='perception_view',
        output='screen',
        parameters=[{
            'publish_overlay': True,
            'show_window': ParameterValue(
                LaunchConfiguration('show_window'), value_type=bool),
        }],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='perception_rviz',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_rviz')),
        # Ogre's default render target is blank white on this Jetson.
        additional_env={'OGRE_RTT_MODE': 'Copy'},
        arguments=['-d', rviz_config],
    )

    return LaunchDescription([
        debug_image_fps,
        enable_gesture,
        show_window,
        use_rviz,
        perception,
        viewer,
        rviz,
    ])
