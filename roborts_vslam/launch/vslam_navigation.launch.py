"""
VSLAM Navigation Launch -- autonomous goal navigation on the scanned 2D map.

Stack:
  - roborts_base            : odom -> base_link, subscribes /cmd_vel_acc
  - RealSense D455 + RTAB-Map localization (vslam_localization.launch.py)
                              : map -> odom, relocalizes in ~/.ros/rtabmap_run3.db.
                              With use_visual_odom:=true the RGB-D visual
                              odometry owns odom -> base_link instead (it
                              matches the recording odometry, but resets
                              itself when tracking is lost).
  - nav2_map_server         : loads roborts_bringup/maps/<map>.yaml
  - map_relay               : /map_raw -> /map @ 1 Hz (costmap QoS workaround)
  - /scan producer (one of):
      sensor_mode:=depth (default) + depth_source:=depthimage
          depthimage_to_laserscan -> scan_frame_shift -> /scan
      sensor_mode:=depth + depth_source:=pointcloud
          pointcloud_to_laserscan -> /scan_pc_raw
              -> scan_qos_relay (best-effort -> reliable) -> /scan
      sensor_mode:=lidar
          rplidar_node -> /scan
  - global_planner_node (A*) + local_planner_node (TEB)
  - vslam_goal_bridge       : /goal_pose -> GlobalPlanner -> LocalPlanner

All /scan producers publish in base_laser_link, which is the frame the
costmap obstacle layer is configured with (sensor_frame).  The global static
costmap uses the exported PGM map, not RTAB-Map's grid (the latter is
remapped to /rtabmap/map).

Prerequisites:
  - The robot base is connected (or the chassis node stays in respawn).
  - RealSense D455 connected.
  - A map database produced by the mapping launch; the default picks
    ~/.ros/rtabmap.db when present, otherwise ~/.ros/rtabmap_run3.db (the
    database that produced maps/vslam_d455.pgm).

Usage:
  ros2 launch roborts_vslam vslam_navigation.launch.py
  ros2 launch roborts_vslam vslam_navigation.launch.py sensor_mode:=lidar
  ros2 launch roborts_vslam vslam_navigation.launch.py depth_source:=pointcloud
  ros2 launch roborts_vslam vslam_navigation.launch.py map:=vslam_d455 \\
      database_path:=$HOME/.ros/rtabmap_run3.db
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration, PathJoinSubstitution, PythonExpression)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from ament_index_python.packages import get_package_share_directory


def _default_database_path():
    """Prefer the canonical RTAB-Map db, else the one behind vslam_d455.pgm."""
    candidates = (
        os.path.expanduser('~/.ros/rtabmap.db'),
        os.path.expanduser('~/.ros/rtabmap_run3.db'),
    )
    for path in candidates:
        if os.path.exists(path):
            return path
    return candidates[0]


def generate_launch_description():
    bringup_dir = get_package_share_directory('roborts_bringup')
    vslam_dir = get_package_share_directory('roborts_vslam')
    rviz_config = os.path.join(
        vslam_dir, 'config', 'vslam_navigation.rviz')

    # ------------------------------------------------------------- arguments
    sensor_mode_arg = DeclareLaunchArgument(
        'sensor_mode', default_value='depth', choices=['depth', 'lidar'],
        description='depth: synthesize /scan from the D455; '
                    'lidar: use the RPLidar.')
    depth_source_arg = DeclareLaunchArgument(
        'depth_source', default_value='depthimage',
        choices=['depthimage', 'pointcloud'],
        description='Depth backend. depthimage is installed by default; '
                    'pointcloud needs ros-$ROS_DISTRO-pointcloud-to-laserscan.')
    camera_ns_arg = DeclareLaunchArgument(
        'camera_ns', default_value='/camera/camera',
        description='RealSense topic prefix.')
    database_path_arg = DeclareLaunchArgument(
        'database_path', default_value=_default_database_path(),
        description='RTAB-Map database used for relocalization.')
    map_arg = DeclareLaunchArgument(
        'map', default_value='vslam_d455',
        description='Map name (without extension) under roborts_bringup/maps.')
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz', default_value='true',
        description='Open RViz2 with the navigation configuration.')
    # IncludeLaunchDescription applies its launch_arguments to the *parent*
    # context, so passing use_rviz:=false to vslam_localization below would
    # also disable this launch's RViz.  Capture the user's value first.
    use_nav_rviz_arg = DeclareLaunchArgument(
        'use_nav_rviz', default_value=LaunchConfiguration('use_rviz'),
        description='Open the navigation RViz (defaults to use_rviz).')

    # Off by default.  The RGB-D visual odometry matches the database, but on
    # this Jetson it loses tracking and resets itself (Odom/ResetCountdown),
    # which teleports odom -> base_link.  The chassis wheels are the stable
    # source; enable this only when the visual odometry has been verified.
    use_visual_odom_arg = DeclareLaunchArgument(
        'use_visual_odom', default_value='false',
        description='Use RGB-D visual odometry as the odom source. '
                    'false (default) uses the chassis /odom.')

    # RViz's 2D Goal Pose publishes a heading as well as a position.  By
    # default only the position is used, so the robot does not rotate on the
    # spot to the (often arbitrary) heading the user dragged.
    respect_goal_orientation_arg = DeclareLaunchArgument(
        'respect_goal_orientation', default_value='false',
        description='Finish at the heading carried by the RViz goal pose.')

    scan_height_arg = DeclareLaunchArgument(
        'scan_height', default_value='20',
        description='depthimage backend: number of centre rows averaged.')
    scan_range_min_arg = DeclareLaunchArgument(
        'scan_range_min', default_value='0.45')
    scan_range_max_arg = DeclareLaunchArgument(
        'scan_range_max', default_value='4.0')
    min_height_arg = DeclareLaunchArgument(
        'min_height', default_value='0.05',
        description='pointcloud backend: min height in base_laser_link.')
    max_height_arg = DeclareLaunchArgument(
        'max_height', default_value='0.95',
        description='pointcloud backend: max height in base_laser_link.')
    # The camera looks along base_laser_link -Y (that frame is yaw +90 deg
    # from base_link), so the forward FOV is centred on -90 deg, not 0.
    pc_angle_min_arg = DeclareLaunchArgument(
        'pc_angle_min', default_value='-2.356',
        description='pointcloud backend: angle_min in base_laser_link [rad].')
    pc_angle_max_arg = DeclareLaunchArgument(
        'pc_angle_max', default_value='-0.785',
        description='pointcloud backend: angle_max in base_laser_link [rad].')
    pc_angle_increment_arg = DeclareLaunchArgument(
        'pc_angle_increment', default_value='0.0087')

    sensor_mode = LaunchConfiguration('sensor_mode')
    depth_source = LaunchConfiguration('depth_source')
    camera_ns = LaunchConfiguration('camera_ns')

    is_lidar = IfCondition(PythonExpression(
        ["'", sensor_mode, "' == 'lidar'"]))
    is_depthimage = IfCondition(PythonExpression(
        ["'", sensor_mode, "' == 'depth' and '",
         depth_source, "' == 'depthimage'"]))
    is_pointcloud = IfCondition(PythonExpression(
        ["'", sensor_mode, "' == 'depth' and '",
         depth_source, "' == 'pointcloud'"]))
    enable_pointcloud = PythonExpression(
        ["'", sensor_mode, "' == 'depth' and '",
         depth_source, "' == 'pointcloud'"])

    # When visual odometry is the odom source the chassis must not publish
    # odom -> base_link, and RTAB-Map must consume the visual odometry topic
    # exactly like it did while the database was recorded.
    base_publish_tf = PythonExpression(
        ["'false' if '", LaunchConfiguration('use_visual_odom'),
         "' == 'true' else 'true'"])
    rtabmap_odom_topic = PythonExpression(
        ["'/rgbd_odometry/odom' if '",
         LaunchConfiguration('use_visual_odom'),
         "' == 'true' else '/odom'"])

    # ------------------------------------------------------------------ base
    base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, 'launch', 'base.launch.py')),
        launch_arguments=[('publish_tf', base_publish_tf)])

    # ---------------------------------------------------------- localization
    # vslam_localization.launch.py already includes static_tf.launch.py; do
    # not include it a second time here.
    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(vslam_dir, 'launch', 'vslam_localization.launch.py')),
        launch_arguments=[
            ('database_path', LaunchConfiguration('database_path')),
            ('camera_ns', camera_ns),
            ('use_rviz', 'false'),
            # Exactly one node owns odom -> base_link: visual odometry by
            # default (matching how the database was recorded), otherwise the
            # chassis.  RTAB-Map owns map -> odom either way.
            ('vo_publish_tf', LaunchConfiguration('use_visual_odom')),
            ('odom_frame', 'odom'),
            ('odom_topic', rtabmap_odom_topic),
            # Only the pointcloud backend needs the realsense pointcloud.
            ('enable_pointcloud', enable_pointcloud),
            # nav2_map_server owns /map; keep RTAB-Map's grid separate.
            ('map_topic', '/rtabmap/map'),
        ])

    # ------------------------------------------------------------ static map
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'yaml_filename': PathJoinSubstitution([
                bringup_dir, 'maps',
                [LaunchConfiguration('map'), '.yaml']]),
            'frame_id': 'map',
            'topic_name': '/map_raw',
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

    # The costmap static layer subscribes /map with VOLATILE QoS and latched
    # maps are TRANSIENT_LOCAL; republish periodically so the planner always
    # sees it regardless of startup order.
    map_relay = Node(
        package='roborts_vslam',
        executable='map_relay.py',
        name='map_relay',
        output='screen',
        parameters=[{
            'input_topic': '/map_raw',
            'output_topic': '/map',
            'publish_rate': 1.0,
        }],
    )

    # ------------------------------------------------------------- perception
    rplidar_node = Node(
        package='rplidar_ros',
        executable='rplidar_node',
        name='rplidar_node',
        output='screen',
        condition=is_lidar,
        parameters=[{
            'serial_port': '/dev/rplidar',
            'serial_baudrate': 115200,
            'frame_id': 'base_laser_link',
            'inverted': False,
            'angle_compensate': True,
            'scan_mode': 'Standard',
        }],
    )

    # depthimage_to_laserscan only relabels the header; its angles are in the
    # camera convention (0 = forward) which matches base_link.  The shift
    # node converts them to base_laser_link (yaw +90 deg) before /scan.
    depthimage_node = Node(
        package='depthimage_to_laserscan',
        executable='depthimage_to_laserscan_node',
        name='depthimage_to_laserscan',
        output='screen',
        condition=is_depthimage,
        remappings=[
            ('depth', [camera_ns, '/aligned_depth_to_color/image_raw']),
            ('depth_camera_info', [camera_ns, '/aligned_depth_to_color/camera_info']),
            ('scan', '/scan_depth_raw'),
        ],
        parameters=[{
            'output_frame': 'base_link',
            'scan_height': ParameterValue(
                LaunchConfiguration('scan_height'), value_type=int),
            'range_min': ParameterValue(
                LaunchConfiguration('scan_range_min'), value_type=float),
            'range_max': ParameterValue(
                LaunchConfiguration('scan_range_max'), value_type=float),
            'scan_time': 0.033,
        }],
    )

    scan_frame_shift = Node(
        package='roborts_vslam',
        executable='scan_frame_shift.py',
        name='scan_frame_shift',
        output='screen',
        condition=is_depthimage,
        parameters=[{
            'input_topic': '/scan_depth_raw',
            'output_topic': '/scan',
            'angle_offset': -1.5708,
            'frame_id': 'base_laser_link',
        }],
    )

    pointcloud_node = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        output='screen',
        condition=is_pointcloud,
        remappings=[
            ('cloud_in', [camera_ns, '/depth/color/points']),
            # SensorDataQoS publisher: pass it through the relay so the
            # costmap's reliable subscription can receive it.
            ('scan', '/scan_pc_raw'),
        ],
        parameters=[{
            'target_frame': 'base_laser_link',
            'transform_tolerance': 0.05,
            'min_height': ParameterValue(
                LaunchConfiguration('min_height'), value_type=float),
            'max_height': ParameterValue(
                LaunchConfiguration('max_height'), value_type=float),
            'angle_min': ParameterValue(
                LaunchConfiguration('pc_angle_min'), value_type=float),
            'angle_max': ParameterValue(
                LaunchConfiguration('pc_angle_max'), value_type=float),
            'angle_increment': ParameterValue(
                LaunchConfiguration('pc_angle_increment'), value_type=float),
            'scan_time': 0.033,
            'range_min': ParameterValue(
                LaunchConfiguration('scan_range_min'), value_type=float),
            'range_max': ParameterValue(
                LaunchConfiguration('scan_range_max'), value_type=float),
            'use_inf': True,
        }],
    )

    # pointcloud_to_laserscan publishes /scan with SensorDataQoS (best
    # effort) while the costmap subscribes reliably.  Re-emit the scan with
    # the default reliable QoS; angle_offset 0 because the cloud was already
    # transformed into base_laser_link by target_frame.
    scan_qos_relay = Node(
        package='roborts_vslam',
        executable='scan_frame_shift.py',
        name='scan_qos_relay',
        output='screen',
        condition=is_pointcloud,
        parameters=[{
            'input_topic': '/scan_pc_raw',
            'output_topic': '/scan',
            'angle_offset': 0.0,
            'frame_id': 'base_laser_link',
        }],
    )

    # -------------------------------------------------------------- planning
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

    goal_bridge = Node(
        package='roborts_vslam',
        executable='vslam_goal_bridge.py',
        name='vslam_goal_bridge',
        output='screen',
        parameters=[{
            'goal_topic': '/goal_pose',
            'respect_goal_orientation': ParameterValue(
                LaunchConfiguration('respect_goal_orientation'),
                value_type=bool),
        }],
    )

    # ------------------------------------------------------------------- viz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_nav_rviz')),
        # Ogre's default render target is blank white on this Jetson.
        additional_env={'OGRE_RTT_MODE': 'Copy'},
        arguments=['-d', rviz_config],
    )

    return LaunchDescription([
        sensor_mode_arg,
        depth_source_arg,
        camera_ns_arg,
        database_path_arg,
        map_arg,
        use_rviz_arg,
        use_nav_rviz_arg,
        use_visual_odom_arg,
        respect_goal_orientation_arg,
        scan_height_arg,
        scan_range_min_arg,
        scan_range_max_arg,
        min_height_arg,
        max_height_arg,
        pc_angle_min_arg,
        pc_angle_max_arg,
        pc_angle_increment_arg,
        LogInfo(msg=['RTAB-Map database: ',
                     LaunchConfiguration('database_path')]),
        base_launch,
        localization_launch,
        map_server,
        map_server_lifecycle,
        map_relay,
        rplidar_node,
        depthimage_node,
        scan_frame_shift,
        pointcloud_node,
        scan_qos_relay,
        global_planner,
        local_planner,
        goal_bridge,
        rviz_node,
    ])
