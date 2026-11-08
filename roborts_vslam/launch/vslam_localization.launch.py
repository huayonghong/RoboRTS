"""
VSLAM Localization Launch File (RealSense D455 RGB-D)

Loads a previously built RTAB-Map database and localizes the robot inside it.
The map is loaded read-only (Mem/IncrementalMemory=false), so this never
modifies the database that vslam_mapping.launch.py produced.

Prerequisites:
  - Map database exists at ~/.ros/rtabmap.db (from the mapping phase)
  - RealSense D455 connected

Usage:
  ros2 launch roborts_vslam vslam_localization.launch.py
  ros2 launch roborts_vslam vslam_localization.launch.py use_rviz:=false
  ros2 launch roborts_vslam vslam_localization.launch.py database_path:=/path/to/rtabmap.db

  # With wheel odometry + EKF (requires the base driver publishing /odom):
  ros2 launch roborts_vslam vslam_localization.launch.py use_ekf:=true \\
      odom_topic:=/odometry/filtered odom_frame:=odom vo_publish_tf:=false

Notes:
  - By default localization is visual-only: RGB-D odometry publishes the
    vo -> base_link transform, exactly like the mapping launch did, so the
    loaded map and the live odometry share the same frame conventions.
  - The D455 on this Jetson is sensitive to initial_reset=true (depth stream
    start failure) and clip_distance > 0 (depth zeroed beyond that range),
    which is why both are kept at the values used for mapping.
"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    vslam_dir = get_package_share_directory('roborts_vslam')
    bringup_dir = get_package_share_directory('roborts_bringup')
    localization_params = os.path.join(
        vslam_dir, 'config', 'rtabmap_localization_params.yaml')
    ekf_config = os.path.join(vslam_dir, 'config', 'ekf_fusion.yaml')
    rviz_config = os.path.join(vslam_dir, 'config', 'vslam_mapping.rviz')

    camera_ns_arg = DeclareLaunchArgument(
        'camera_ns', default_value='/camera/camera',
        description='RealSense topic prefix (see vslam_mapping.launch.py)')

    use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='false')

    database_path_arg = DeclareLaunchArgument(
        'database_path', default_value=os.path.expanduser('~/.ros/rtabmap.db'),
        description='RTAB-Map database built during the mapping phase')

    use_ekf_arg = DeclareLaunchArgument(
        'use_ekf', default_value='false',
        description='Fuse wheel odometry (/odom) with visual odometry through '
                    'robot_localization. Requires the base driver to be running.')

    odom_topic_arg = DeclareLaunchArgument(
        'odom_topic', default_value='/rgbd_odometry/odom',
        description='Odometry topic fed to RTAB-Map. Use /odometry/filtered with use_ekf:=true')

    odom_frame_arg = DeclareLaunchArgument(
        'odom_frame', default_value='vo',
        description='Odometry frame expected by RTAB-Map. Use odom with use_ekf:=true')

    vo_publish_tf_arg = DeclareLaunchArgument(
        'vo_publish_tf', default_value='true',
        description='Let RGB-D odometry publish <odom_frame> -> base_link. '
                    'Must be false when another node (chassis or EKF) owns it.')

    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz', default_value='true',
        description='Open RViz2 to visualize the localization on the loaded map')

    # Realsense pointcloud output. Only needed by the depth-based obstacle
    # sensing path (pointcloud_to_laserscan) in vslam_navigation.launch.py.
    # Default false: identical to the previous behaviour, no
    # /depth/color/points topic is produced.
    enable_pointcloud_arg = DeclareLaunchArgument(
        'enable_pointcloud', default_value='false',
        description='Enable realsense pointcloud for depth-based obstacle sensing.')

    # Topic on which RTAB-Map publishes its occupancy grid. Default 'map'
    # preserves the existing behaviour. vslam_navigation.launch.py sets this
    # to /rtabmap/map so that nav2_map_server can own /map.
    map_topic_arg = DeclareLaunchArgument(
        'map_topic', default_value='map',
        description='Topic for the RTAB-Map occupancy grid output.')

    camera_ns = LaunchConfiguration('camera_ns')

    static_tf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, 'launch', 'static_tf.launch.py')))

    realsense_node = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        name='camera',
        output='screen',
        parameters=[{
            'enable_color': True,
            'enable_depth': True,
            'enable_infra1': False,
            'enable_infra2': False,
            'enable_gyro': True,
            'enable_accel': True,
            'unite_imu_method': 2,
            'rgb_camera.color_profile': '640x480x30',
            'depth_module.depth_profile': '640x480x30',
            'align_depth.enable': True,
            # 0.0 disables clipping. A non-zero clip_distance zeroes out all
            # depth pixels beyond that range, which starves RGB-D odometry.
            'clip_distance': 0.0,
            # Disable realsense internal TF publishing — camera_link -> camera_*
            # optical_frame transforms come from static_tf.launch.py.
            'publish_tf': False,
            # initial_reset=true triggers a firmware reset that on this
            # Jetson/D455 combo raises "Depth stream start failure".
            'initial_reset': False,
            'enable_sync': True,
            # Pointcloud output for the depth-based /scan synthesis path.
            # Off by default so the localization launch behaves exactly as
            # before when used on its own.
            #
            # realsense2_camera names the parameter after the filter's
            # RS2_CAMERA_INFO_NAME.  On this build (4.57.7 + librealsense
            # NEON) the pointcloud filter is called "Pointcloud (NEON)", so
            # the sanitised name is pointcloud__neon_, not pointcloud.
            # If the camera build changes, find the right key with:
            #   ros2 param list /camera/camera | grep -i pointcloud
            'pointcloud__neon_.enable': ParameterValue(
                LaunchConfiguration('enable_pointcloud'), value_type=bool),
        }],
    )

    # Same RGB-D odometry configuration as vslam_mapping.launch.py so that the
    # odometry behaves exactly like it did while the map was recorded.
    rgbd_odometry = Node(
        package='rtabmap_odom',
        executable='rgbd_odometry',
        name='rgbd_odometry',
        output='screen',
        parameters=[{
            'subscribe_depth': True,
            'subscribe_rgb': True,
            'frame_id': 'base_link',
            # Keep the TF frame name in sync with odom_frame so the same node
            # works both in the mapping/standalone stack (vo) and in the
            # navigation stack (odom, where visual odometry replaces the
            # chassis as the odom -> base_link owner).
            'odom_frame_id': LaunchConfiguration('odom_frame'),
            'publish_tf': ParameterValue(
                LaunchConfiguration('vo_publish_tf'), value_type=bool),
            'approx_sync': True,
            'approx_sync_max_interval': 0.02,
            'sync_queue_size': 10,
            'wait_for_transform': 0.5,
            'qos_image': 1,
            'qos_camera_info': 1,
            'use_sim_time': ParameterValue(
                LaunchConfiguration('use_sim_time'), value_type=bool),
            'Odom/Strategy': '0',
            'Odom/GuessMotion': 'true',
            # Without this, frame-to-map odometry stays permanently lost after
            # a bad frame (it never re-acquires). Reset after ~15 failed frames
            # (~2 s at 7 Hz) so tracking can recover and RTAB-Map relocalizes.
            'Odom/ResetCountdown': '15',
            'Vis/CorType': '0',
            'OdomF2M/MaxSize': '2000',
            'OdomF2M/MaxNewFeatures': '1000',
            'Vis/MinInliers': '10',
            'Reg/Force3DoF': 'true',
        }],
        remappings=[
            ('rgb/image', [camera_ns, '/color/image_raw']),
            ('rgb/camera_info', [camera_ns, '/color/camera_info']),
            ('depth/image', [camera_ns, '/aligned_depth_to_color/image_raw']),
            # Keep the visual odometry off /odom: in the navigation stack the
            # chassis owns odom -> base_link (vo_publish_tf:=false), and a
            # second publisher on /odom (frame "vo") corrupts the odometry
            # input of RTAB-Map and TEB.  The default visual-only mode already
            # reads this topic (odom_topic defaults to /rgbd_odometry/odom).
            ('odom', '/rgbd_odometry/odom'),
        ],
    )

    rtabmap_node = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=[
            localization_params,
            {
                'use_sim_time': ParameterValue(
                    LaunchConfiguration('use_sim_time'), value_type=bool),
                'database_path': LaunchConfiguration('database_path'),
                'odom_frame_id': LaunchConfiguration('odom_frame'),
            },
        ],
        remappings=[
            ('rgb/image', [camera_ns, '/color/image_raw']),
            ('rgb/camera_info', [camera_ns, '/color/camera_info']),
            ('depth/image', [camera_ns, '/aligned_depth_to_color/image_raw']),
            ('odom', LaunchConfiguration('odom_topic')),
            # Keep RTAB-Map's grid off /map when nav2_map_server owns it.
            ('map', LaunchConfiguration('map_topic')),
        ],
    )

    # Odom relay for the optional EKF fusion (visual odom -> /visual_odom).
    odom_relay = Node(
        package='roborts_vslam',
        executable='vslam_odom_relay',
        name='vslam_odom_relay',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_ekf')),
        parameters=[{
            'input_odom_topic': '/rgbd_odometry/odom',
            'output_odom_topic': '/visual_odom',
            'publish_tf': False,
        }],
    )

    # EKF fusion: wheel odom + visual odom
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_ekf')),
        parameters=[ekf_config],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_rviz')),
        # On Jetson (NVIDIA EGL), RViz2's Ogre render window defaults to a
        # blank white panel. Copying the render target fixes it without
        # falling back to slow software GL.
        additional_env={'OGRE_RTT_MODE': 'Copy'},
        arguments=['-d', rviz_config],
    )

    return LaunchDescription([
        camera_ns_arg,
        use_sim_time,
        database_path_arg,
        use_ekf_arg,
        odom_topic_arg,
        odom_frame_arg,
        vo_publish_tf_arg,
        use_rviz_arg,
        enable_pointcloud_arg,
        map_topic_arg,
        static_tf_launch,
        realsense_node,
        rgbd_odometry,
        rtabmap_node,
        odom_relay,
        ekf_node,
        rviz_node,
    ])
