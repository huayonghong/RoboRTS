"""
VSLAM Mapping Launch File (RealSense D455 RGB-D)

Launches RTAB-Map in mapping mode using Intel RealSense D455 depth camera.
Builds a 3D visual + depth map of the environment.

Prerequisites:
  - RealSense D455 connected via USB 3.0
  - ros-${ROS_DISTRO}-realsense2-camera installed
  - Proper TF: base_link -> camera_link (set in static_tf.launch.py)

Usage:
  ros2 launch roborts_vslam vslam_mapping.launch.py
  # Override camera topic prefix for realsense2_camera 4.54+ or 3.x:
  ros2 launch roborts_vslam vslam_mapping.launch.py camera_ns:=/camera
"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    vslam_dir = get_package_share_directory('roborts_vslam')
    mapping_params = os.path.join(
        vslam_dir, 'config', 'rtabmap_mapping_params.yaml')

    # RealSense topic prefix varies by realsense2_camera version:
    #   4.51-4.53: internal hardcoded ns "/camera" + name "camera" → /camera/camera/...
    #   4.54+:     relative ns → /camera/...
    #   3.x:       /camera/...
    # Run `ros2 topic list | grep color` after launching the camera to verify,
    # then override camera_ns if the default doesn't match.
    camera_ns_arg = DeclareLaunchArgument(
        'camera_ns', default_value='/camera/camera',
        description='RealSense topic prefix. /camera/camera for 4.51-4.53, /camera for 4.54+ or 3.x')

    camera_ns = LaunchConfiguration('camera_ns')

    use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='false')

    use_rtabmap_gui = DeclareLaunchArgument(
        'use_rtabmap_gui', default_value='true',
        description='If true, launch RTAB-Map standalone GUI (rtabmap_viz) in addition to SLAM nodes.')

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
            # Disable realsense internal TF publishing — we publish the required
            # camera_link → camera_*_optical_frame transforms in static_tf.launch.py
            # to guarantee they exist before rgbd_odometry starts looking them up.
            # This avoids version-dependent issues (4.51-4.53: rate=0 disables TF;
            # 4.54+: rate=0 uses StaticTransformBroadcaster but may race with startup).
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
        parameters=[
            mapping_params,
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
        remappings=[
            ('rgb/image', [camera_ns, '/color/image_raw']),
            ('rgb/camera_info', [camera_ns, '/color/camera_info']),
            ('depth/image', [camera_ns, '/aligned_depth_to_color/image_raw']),
            ('odom', '/rgbd_odometry/odom'),
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
            'publish_tf': True,
            'approx_sync': True,
            'sync_queue_size': 10,
            'wait_for_transform': 0.5,
            'qos_image': 1,
            'qos_camera_info': 1,
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'Odom/Strategy': '0',
            'Odom/GuessMotion': 'true',
            'Vis/CorType': '0',
            'OdomF2M/MaxSize': '1000',
            'OdomF2M/MaxNewFeatures': '150',
            'Reg/Force3DoF': 'true',
        }],
        remappings=[
            ('rgb/image', [camera_ns, '/color/image_raw']),
            ('rgb/camera_info', [camera_ns, '/color/camera_info']),
            ('depth/image', [camera_ns, '/aligned_depth_to_color/image_raw']),
            ('odom', '/rgbd_odometry/odom'),
        ],
    )

    rtabmap_viz = Node(
        package='rtabmap_viz',
        executable='rtabmap_viz',
        name='rtabmap_viz',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_rtabmap_gui')),
        parameters=[{
            'subscribe_depth': True,
            'subscribe_rgb': True,
            'subscribe_odom': True,
            'frame_id': 'base_link',
            'odom_frame_id': 'vo',
            'approx_sync': True,
            'sync_queue_size': 10,
        }],
        remappings=[
            ('rgb/image', [camera_ns, '/color/image_raw']),
            ('rgb/camera_info', [camera_ns, '/color/camera_info']),
            ('depth/image', [camera_ns, '/aligned_depth_to_color/image_raw']),
            ('odom', '/rgbd_odometry/odom'),
        ],
    )

    return LaunchDescription([
        camera_ns_arg,
        use_sim_time,
        use_rtabmap_gui,
        realsense_node,
        rtabmap_odom,
        rtabmap_node,
        rtabmap_viz,
    ])
