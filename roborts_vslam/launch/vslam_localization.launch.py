"""
VSLAM Localization Launch File (RealSense D455 RGB-D)

Launches RTAB-Map in localization mode using a pre-built map database,
with EKF fusion of visual odometry and wheel odometry.

Prerequisites:
  - Map database exists at ~/.ros/rtabmap.db (from mapping phase)
  - RealSense D455 connected
  - Wheel odometry on /odom
  - Proper TF chain

Usage:
  ros2 launch roborts_vslam vslam_localization.launch.py
  # For realsense2_camera 4.54+ or 3.x:
  ros2 launch roborts_vslam vslam_localization.launch.py camera_ns:=/camera
"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    vslam_dir = get_package_share_directory('roborts_vslam')
    ekf_config = os.path.join(vslam_dir, 'config', 'ekf_fusion.yaml')

    camera_ns_arg = DeclareLaunchArgument(
        'camera_ns', default_value='/camera/camera',
        description='RealSense topic prefix. /camera/camera for 4.51-4.53, /camera for 4.54+ or 3.x')
    camera_ns = LaunchConfiguration('camera_ns')

    use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='false')

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
            'clip_distance': 4.0,
            'publish_tf': False,
            'initial_reset': True,
            'enable_sync': True,
        }],
    )

    rtabmap_node = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=[{
            'subscribe_depth': True,
            'subscribe_rgb': True,
            'subscribe_odom': True,
            'frame_id': 'base_link',
            'odom_frame_id': 'odom',
            'map_frame_id': 'map',
            'approx_sync': True,
            'sync_queue_size': 10,
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'Mem/IncrementalMemory': 'false',
            'Mem/InitWMWithAllNodes': 'true',
            'Reg/Strategy': '0',
            'Reg/Force3DoF': 'true',
        }],
        remappings=[
            ('rgb/image', [camera_ns, '/color/image_raw']),
            ('rgb/camera_info', [camera_ns, '/color/camera_info']),
            ('depth/image', [camera_ns, '/aligned_depth_to_color/image_raw']),
            ('odom', '/odom'),
        ],
    )

    rtabmap_odom = Node(
        package='rtabmap_odom',
        executable='rgbd_odometry',
        name='rgbd_odometry',
        output='screen',
        parameters=[{
            'subscribe_depth': True,
            'subscribe_rgb': True,
            'frame_id': 'base_link',
            'odom_frame_id': 'vo',
            'publish_tf': False,
            'approx_sync': True,
            'sync_queue_size': 10,
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'Odom/Strategy': '0',
            'Odom/GuessMotion': 'true',
            'OdomF2M/MaxSize': '1000',
            'Reg/Force3DoF': 'true',
        }],
        remappings=[
            ('rgb/image', [camera_ns, '/color/image_raw']),
            ('rgb/camera_info', [camera_ns, '/color/camera_info']),
            ('depth/image', [camera_ns, '/aligned_depth_to_color/image_raw']),
        ],
    )

    # Odom relay for EKF fusion
    odom_relay = Node(
        package='roborts_vslam',
        executable='vslam_odom_relay',
        name='vslam_odom_relay',
        output='screen',
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
        parameters=[ekf_config],
    )

    return LaunchDescription([
        camera_ns_arg,
        use_sim_time,
        realsense_node,
        rtabmap_odom,
        rtabmap_node,
        odom_relay,
        ekf_node,
    ])
