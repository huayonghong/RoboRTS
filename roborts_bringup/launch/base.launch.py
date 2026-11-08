import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    roborts_base_dir = get_package_share_directory('roborts_base')

    # When another node owns odom -> base_link (visual odometry in the VSLAM
    # navigation stack), pass publish_tf:=false so the two do not fight over
    # the same transform.
    publish_tf_arg = DeclareLaunchArgument(
        'publish_tf', default_value='true',
        description='Let the chassis publish odom -> base_link.')

    return LaunchDescription([
        publish_tf_arg,
        Node(
            package='roborts_base',
            executable='roborts_base_node',
            name='roborts_base_node',
            output='screen',
            parameters=[
                os.path.join(roborts_base_dir, 'config', 'roborts_base_parameter.yaml'),
                {'publish_tf': ParameterValue(LaunchConfiguration('publish_tf'),
                                              value_type=bool)},
            ],
            respawn=True,
        ),
    ])
