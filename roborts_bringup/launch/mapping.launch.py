import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    bringup_dir = get_package_share_directory('roborts_bringup')

    base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup_dir, 'launch', 'base.launch.py'))
    )

    static_tf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup_dir, 'launch', 'static_tf.launch.py'))
    )

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
            'scan_mode': 'Standard',
        }],
    )

    slam_toolbox = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[{
            'odom_frame': 'odom',
            'map_frame': 'map',
            'base_frame': 'base_link',
            'scan_topic': '/scan',
            'use_sim_time': False,
            'mode': 'mapping',
        }],
    )

    return LaunchDescription([
        base_launch,
        static_tf_launch,
        rplidar_node,
        slam_toolbox,
    ])
