import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    bringup_dir = get_package_share_directory('roborts_bringup')
    localization_dir = get_package_share_directory('roborts_localization')

    map_arg = DeclareLaunchArgument('map', default_value='icra2019')

    base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup_dir, 'launch', 'base.launch.py'))
    )

    static_tf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup_dir, 'launch', 'static_tf.launch.py'))
    )

    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'yaml_filename': os.path.join(bringup_dir, 'maps', 'icra2019.yaml'),
            'use_sim_time': False,
        }],
    )

    map_server_lifecycle = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        output='screen',
        parameters=[{
            'autostart': True,
            'node_names': ['map_server'],
        }],
    )

    localization_node = Node(
        package='roborts_localization',
        executable='localization_node',
        name='localization_node',
        output='screen',
        parameters=[
            os.path.join(localization_dir, 'config', 'localization.yaml'),
            os.path.join(localization_dir, 'amcl', 'config', 'amcl.yaml'),
        ],
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
        map_arg,
        base_launch,
        static_tf_launch,
        map_server,
        map_server_lifecycle,
        localization_node,
        global_planner,
        local_planner,
    ])
