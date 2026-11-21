"""Launch the gimbal CSI perception node (P3 capture, P4 persons, P5 gestures).

The CSI camera is opened inside ``person_gesture_node`` -- there is no
separate camera node and no image topic unless ``publish_debug_image`` is set.
P4 needs ``models/yolo11n-pose_fp16.engine``; P5 additionally needs
``models/hand_pose_fp16.engine`` and ``enable_gesture:=true``.

Usage:
  ros2 launch roborts_perception person_gesture.launch.py
  ros2 launch roborts_perception person_gesture.launch.py \
      publish_debug_image:=true
  ros2 launch roborts_perception person_gesture.launch.py \
      enable_gesture:=true
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package_dir = get_package_share_directory('roborts_perception')
    models_dir = os.path.join(package_dir, 'models')

    sensor_id = DeclareLaunchArgument(
        'sensor_id', default_value='0',
        description='Argus sensor id for nvarguscamerasrc')
    image_width = DeclareLaunchArgument(
        'image_width', default_value='1920',
        description='Capture width (must match the calibration file)')
    image_height = DeclareLaunchArgument(
        'image_height', default_value='1080',
        description='Capture height (must match the calibration file)')
    framerate = DeclareLaunchArgument(
        'framerate', default_value='30',
        description='Capture frame rate')
    publish_debug_image = DeclareLaunchArgument(
        'publish_debug_image', default_value='false',
        description='Publish /perception/debug/image_raw and camera_info')
    debug_image_fps = DeclareLaunchArgument(
        'debug_image_fps', default_value='10.0',
        description='Debug image publish rate (0 = every frame)')
    camera_info_path = DeclareLaunchArgument(
        'camera_info_path',
        default_value=os.path.join(package_dir, 'config', 'camera_csi.yaml'),
        description='Camera intrinsics YAML')
    enable_person = DeclareLaunchArgument(
        'enable_person', default_value='true',
        description='Run yolo11n-pose on TensorRT and publish persons')
    enable_gesture = DeclareLaunchArgument(
        'enable_gesture', default_value='false',
        description='Run the hand-pose engine and publish gestures')
    person_engine_path = DeclareLaunchArgument(
        'person_engine_path',
        default_value=os.path.join(
            models_dir, 'yolo11n-pose_fp16.engine'),
        description='Person pose TensorRT engine')
    hand_engine_path = DeclareLaunchArgument(
        'hand_engine_path',
        default_value=os.path.join(models_dir, 'hand_pose_fp16.engine'),
        description='Hand pose TensorRT engine')

    node = Node(
        package='roborts_perception',
        executable='person_gesture_node',
        name='person_gesture_node',
        output='screen',
        parameters=[{
            'sensor_id': ParameterValue(
                LaunchConfiguration('sensor_id'), value_type=int),
            'image_width': ParameterValue(
                LaunchConfiguration('image_width'), value_type=int),
            'image_height': ParameterValue(
                LaunchConfiguration('image_height'), value_type=int),
            'framerate': ParameterValue(
                LaunchConfiguration('framerate'), value_type=int),
            'publish_debug_image': ParameterValue(
                LaunchConfiguration('publish_debug_image'), value_type=bool),
            'debug_image_fps': ParameterValue(
                LaunchConfiguration('debug_image_fps'), value_type=float),
            'camera_info_path': LaunchConfiguration('camera_info_path'),
            'enable_person': ParameterValue(
                LaunchConfiguration('enable_person'), value_type=bool),
            'enable_gesture': ParameterValue(
                LaunchConfiguration('enable_gesture'), value_type=bool),
            'person_engine_path': LaunchConfiguration('person_engine_path'),
            'hand_engine_path': LaunchConfiguration('hand_engine_path'),
        }],
    )

    return LaunchDescription([
        sensor_id,
        image_width,
        image_height,
        framerate,
        publish_debug_image,
        debug_image_fps,
        camera_info_path,
        enable_person,
        enable_gesture,
        person_engine_path,
        hand_engine_path,
        node,
    ])
