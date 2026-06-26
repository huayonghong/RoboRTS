"""
VSLAM Mapping + Visualization

Launches VSLAM mapping stack with RViz2 (3D map / TF / odometry) plus
rqt_image_view windows for color and depth camera feeds.

Usage:
  ros2 launch roborts_vslam vslam_mapping_viz.launch.py
  # For realsense2_camera 4.54+ or 3.x:
  ros2 launch roborts_vslam vslam_mapping_viz.launch.py camera_ns:=/camera

  # On Jetson, if RViz2 Image plugin still shows blank, force software GL:
  LIBGL_ALWAYS_SOFTWARE=1 ros2 launch roborts_vslam vslam_mapping_viz.launch.py
"""
import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    vslam_dir = get_package_share_directory('roborts_vslam')
    bringup_dir = get_package_share_directory('roborts_bringup')

    camera_ns_arg = DeclareLaunchArgument(
        'camera_ns', default_value='/camera/camera',
        description='RealSense topic prefix (see vslam_mapping.launch.py)')
    camera_ns = LaunchConfiguration('camera_ns')

    show_rqt_arg = DeclareLaunchArgument(
        'show_rqt', default_value='true',
        description='Also launch rqt_image_view windows for color/depth')

    static_tf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, 'launch', 'static_tf.launch.py')))

    mapping_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(vslam_dir, 'launch', 'vslam_mapping.launch.py')),
        launch_arguments=[
            ('use_rtabmap_gui', 'false'),
            ('camera_ns', LaunchConfiguration('camera_ns')),
        ],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(vslam_dir, 'config', 'vslam_mapping.rviz')],
    )

    # rqt_image_view as fallback (works everywhere including Jetson)
    color_viewer = ExecuteProcess(
        cmd=['ros2', 'run', 'rqt_image_view', 'rqt_image_view',
             [camera_ns, '/color/image_raw']],
        output='screen',
        condition=IfCondition(LaunchConfiguration('show_rqt')),
    )

    depth_viewer = ExecuteProcess(
        cmd=['ros2', 'run', 'rqt_image_view', 'rqt_image_view',
             [camera_ns, '/aligned_depth_to_color/image_raw']],
        output='screen',
        condition=IfCondition(LaunchConfiguration('show_rqt')),
    )

    return LaunchDescription([
        camera_ns_arg,
        show_rqt_arg,
        static_tf_launch,
        mapping_launch,
        rviz_node,
        color_viewer,
        depth_viewer,
    ])
