import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    roborts_base_dir = get_package_share_directory('roborts_base')

    return LaunchDescription([
        Node(
            package='roborts_base',
            executable='roborts_base_node',
            name='roborts_base_node',
            output='screen',
            parameters=[os.path.join(roborts_base_dir, 'config', 'roborts_base_parameter.yaml')],
            respawn=True,
        ),
    ])
