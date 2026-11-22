"""P3-P7: one integrated gimbal perception and robot-control stack.

Starts everything needed to control the robot from the gimbal camera:

  person_gesture_node      P3 CSI capture, P4 person detection, P5 gestures
  gimbal_follow_node       P6 absolute-angle visual following
  gesture_command_bridge   P7 open hand -> fist locks a person + starts
                              following, fist -> open hand releases + stops,
                              thumbs_up forward
  cmd_vel_mux              P7 safety > gesture > planner -> /cmd_vel_acc

Tracking is idle at start.  Show an open hand (palm, or pointing_up/victory if
that is how the model labels it), then close it into a fist: the bridge locks
that person's track id and enables following.  Open the hand again
(fist -> open hand) to release the lock and stop following.

Forward motion with thumbs_up is separately gated by gesture mode:
  ros2 service call /gesture_command_bridge/enable std_srvs/srv/SetBool \\
      "{data: true}"

`cmd_vel_mux` writes /cmd_vel_acc, so a planner running at the same time must
publish /cmd_vel_acc_planner instead.  ``vslam_navigation.launch.py
use_gesture_control:=true`` does that remap automatically and includes this
launch.

Requirements:
  * By default this launch also starts roborts_base (start_base:=true), which
    publishes base_link -> gimbal and consumes /cmd_gimbal_angle and
    /cmd_vel_acc.  If a bringup already starts the base, pass
    start_base:=false or the serial port will be opened twice.
  * The main bringup normally already publishes the static TF
    gimbal -> csi_camera_link -> csi_camera_optical_frame.  Set
    publish_csi_tf:=true to add it here for a standalone run (do not do both).

Usage:
  ros2 launch roborts_perception perception_control.launch.py
  ros2 launch roborts_perception perception_control.launch.py use_viewer:=true
  ros2 launch roborts_perception perception_control.launch.py \\
      start_base:=false gesture_enabled:=true follow_enabled:=false
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    AndSubstitution, LaunchConfiguration, OrSubstitution)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package_dir = get_package_share_directory('roborts_perception')
    base_dir = get_package_share_directory('roborts_base')
    models_dir = os.path.join(package_dir, 'models')
    rviz_config = os.path.join(package_dir, 'config', 'perception.rviz')

    # ------------------------------------------------------------- arguments
    start_base = DeclareLaunchArgument(
        'start_base', default_value='true',
        description='Start roborts_base here (false when a bringup already '
                    'starts it)')
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
    camera_info_path = DeclareLaunchArgument(
        'camera_info_path',
        default_value=os.path.join(package_dir, 'config', 'camera_csi.yaml'),
        description='Camera intrinsics YAML')
    publish_debug_image = DeclareLaunchArgument(
        'publish_debug_image', default_value='false',
        description='Publish /perception/debug/image_raw and camera_info')
    debug_image_fps = DeclareLaunchArgument(
        'debug_image_fps', default_value='10.0',
        description='Debug image publish rate (0 = every frame)')
    enable_person = DeclareLaunchArgument(
        'enable_person', default_value='true',
        description='Run person detection (P4); needed by gestures')
    enable_gesture = DeclareLaunchArgument(
        'enable_gesture', default_value='true',
        description='Run hand gestures (P5); the P6/P7 tracking switch needs '
                    'this')
    person_engine_path = DeclareLaunchArgument(
        'person_engine_path',
        default_value=os.path.join(models_dir, 'yolo11n-pose_fp16.engine'),
        description='Person pose TensorRT engine')
    hand_engine_path = DeclareLaunchArgument(
        'hand_engine_path',
        default_value=os.path.join(models_dir, 'hand_pose_fp16.engine'),
        description='Hand pose TensorRT engine')

    follow_enabled = DeclareLaunchArgument(
        'follow_enabled', default_value='false',
        description='Follow the best-scoring person before any gesture lock '
                    '(normally off; open hand->fist starts locked tracking)')
    transition_timeout = DeclareLaunchArgument(
        'transition_timeout_s', default_value='1.5',
        description='Seconds between the open hand and fist for a tracking edge')
    allow_chassis_follow = DeclareLaunchArgument(
        'allow_chassis_follow', default_value='false',
        description='Allow the chassis to follow when gimbal yaw hits its limit')
    yaw_sign = DeclareLaunchArgument(
        'yaw_sign', default_value='-1.0',
        description='Camera yaw sign relative to gimbal yaw (bench test)')
    pitch_sign = DeclareLaunchArgument(
        'pitch_sign', default_value='-1.0',
        description='Camera pitch sign relative to gimbal pitch (bench test)')
    yaw_gain = DeclareLaunchArgument(
        'yaw_gain', default_value='0.9',
        description='Proportional yaw gain (higher = more aggressive)')
    pitch_gain = DeclareLaunchArgument(
        'pitch_gain', default_value='0.9',
        description='Proportional pitch gain')
    deadband_deg = DeclareLaunchArgument(
        'deadband_deg', default_value='0.8',
        description='Ignore bearing errors below this angle')
    max_rate_deg_s = DeclareLaunchArgument(
        'max_rate_deg_s', default_value='180.0',
        description='Maximum commanded gimbal rate (deg/s)')
    publish_rate = DeclareLaunchArgument(
        'publish_rate', default_value='50.0',
        description='Gimbal command/control loop rate (Hz)')
    set_control_mode = DeclareLaunchArgument(
        'set_control_mode', default_value='true',
        description='Call /set_gimbal_mode(CODE_CONTROL) on startup')
    gesture_enabled = DeclareLaunchArgument(
        'gesture_enabled', default_value='false',
        description='Allow thumbs_up forward motion (tracking switch works '
                    'without it)')
    use_mux = DeclareLaunchArgument(
        'use_mux', default_value='true',
        description='Arbitrate /cmd_vel_acc through cmd_vel_mux')

    use_viewer = DeclareLaunchArgument(
        'use_viewer', default_value='false',
        description='Start perception_view to draw the tracking HUD/overlay')
    use_rviz = DeclareLaunchArgument(
        'use_rviz', default_value='true',
        description='With use_viewer:=true, open RViz (perception.rviz)')
    show_window = DeclareLaunchArgument(
        'show_window', default_value='false',
        description='With use_viewer:=true, also open a local OpenCV window')

    publish_csi_tf = DeclareLaunchArgument(
        'publish_csi_tf', default_value='false',
        description='Publish gimbal -> csi_camera_link -> optical frame here. '
                    'Leave false when the main bringup already publishes it.')
    csi_camera_x = DeclareLaunchArgument(
        'csi_camera_x', default_value='0.03',
        description='CSI camera x offset from the gimbal pivot [m]')
    csi_camera_y = DeclareLaunchArgument(
        'csi_camera_y', default_value='0.0',
        description='CSI camera y offset from the gimbal pivot [m]')
    csi_camera_z = DeclareLaunchArgument(
        'csi_camera_z', default_value='0.07',
        description='CSI camera z offset from the gimbal pivot [m]')
    csi_camera_roll = DeclareLaunchArgument(
        'csi_camera_roll', default_value='0.0',
        description='CSI camera roll in the gimbal frame [rad]')
    csi_camera_pitch = DeclareLaunchArgument(
        'csi_camera_pitch', default_value='0.0',
        description='CSI camera pitch in the gimbal frame [rad]')
    csi_camera_yaw = DeclareLaunchArgument(
        'csi_camera_yaw', default_value='0.0',
        description='CSI camera yaw in the gimbal frame [rad]')

    # ---------------------------------------------------------- base / robot
    # roborts_base owns the chassis, the dynamic base_link -> gimbal TF and
    # the /set_gimbal_mode service.  It must run for the P6 follower to work.
    base = Node(
        package='roborts_base',
        executable='roborts_base_node',
        name='roborts_base_node',
        output='screen',
        condition=IfCondition(LaunchConfiguration('start_base')),
        parameters=[
            os.path.join(base_dir, 'config', 'roborts_base_parameter.yaml'),
            {'publish_tf': True},
        ],
        respawn=True,
    )

    # --------------------------------------------------------- P3/P4/P5 nodes
    perception = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(package_dir, 'launch', 'person_gesture.launch.py')),
        launch_arguments={
            'sensor_id': LaunchConfiguration('sensor_id'),
            'image_width': LaunchConfiguration('image_width'),
            'image_height': LaunchConfiguration('image_height'),
            'framerate': LaunchConfiguration('framerate'),
            'camera_info_path': LaunchConfiguration('camera_info_path'),
            # The viewer needs the debug image; use_viewer implies it.
            'publish_debug_image': OrSubstitution(
                LaunchConfiguration('publish_debug_image'),
                LaunchConfiguration('use_viewer')),
            'debug_image_fps': LaunchConfiguration('debug_image_fps'),
            'enable_person': LaunchConfiguration('enable_person'),
            'enable_gesture': LaunchConfiguration('enable_gesture'),
            'person_engine_path': LaunchConfiguration('person_engine_path'),
            'hand_engine_path': LaunchConfiguration('hand_engine_path'),
        }.items(),
    )

    # ----------------------------------------------------------- P6/P7 nodes
    control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(package_dir, 'launch', 'gesture_control.launch.py')),
        launch_arguments={
            'follow_enabled': LaunchConfiguration('follow_enabled'),
            'transition_timeout_s': LaunchConfiguration('transition_timeout_s'),
            'allow_chassis_follow': LaunchConfiguration('allow_chassis_follow'),
            'yaw_sign': LaunchConfiguration('yaw_sign'),
            'pitch_sign': LaunchConfiguration('pitch_sign'),
            'yaw_gain': LaunchConfiguration('yaw_gain'),
            'pitch_gain': LaunchConfiguration('pitch_gain'),
            'deadband_deg': LaunchConfiguration('deadband_deg'),
            'max_rate_deg_s': LaunchConfiguration('max_rate_deg_s'),
            'publish_rate': LaunchConfiguration('publish_rate'),
            'set_control_mode': LaunchConfiguration('set_control_mode'),
            'gesture_enabled': LaunchConfiguration('gesture_enabled'),
            'use_mux': LaunchConfiguration('use_mux'),
        }.items(),
    )

    # ------------------------------------------------------------- optional viz
    viewer = Node(
        package='roborts_perception',
        executable='perception_view',
        name='perception_view',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_viewer')),
        parameters=[{
            'publish_overlay': True,
            'show_window': ParameterValue(
                LaunchConfiguration('show_window'), value_type=bool),
        }],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='perception_rviz',
        output='screen',
        condition=IfCondition(AndSubstitution(
            LaunchConfiguration('use_viewer'),
            LaunchConfiguration('use_rviz'))),
        # Ogre's default render target is blank white on this Jetson.
        additional_env={'OGRE_RTT_MODE': 'Copy'},
        arguments=['-d', rviz_config],
    )

    # --------------------------------------------- optional CSI static TF (P3)
    csi_tf_condition = IfCondition(LaunchConfiguration('publish_csi_tf'))
    csi_camera_link = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='gimbal_csi_camera_link_broadcaster',
        output='screen',
        condition=csi_tf_condition,
        arguments=['--x', LaunchConfiguration('csi_camera_x'),
                   '--y', LaunchConfiguration('csi_camera_y'),
                   '--z', LaunchConfiguration('csi_camera_z'),
                   '--yaw', LaunchConfiguration('csi_camera_yaw'),
                   '--pitch', LaunchConfiguration('csi_camera_pitch'),
                   '--roll', LaunchConfiguration('csi_camera_roll'),
                   '--frame-id', 'gimbal',
                   '--child-frame-id', 'csi_camera_link'],
    )
    csi_optical = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='csi_camera_link_optical_broadcaster',
        output='screen',
        condition=csi_tf_condition,
        arguments=['--x', '0.0', '--y', '0.0', '--z', '0.0',
                   '--yaw', '-1.5708', '--pitch', '0.0', '--roll', '-1.5708',
                   '--frame-id', 'csi_camera_link',
                   '--child-frame-id', 'csi_camera_optical_frame'],
    )

    return LaunchDescription([
        # base / robot
        start_base,
        # camera / P3
        sensor_id, image_width, image_height, framerate, camera_info_path,
        publish_debug_image, debug_image_fps,
        # P4/P5
        enable_person, enable_gesture, person_engine_path, hand_engine_path,
        # P6
        follow_enabled, allow_chassis_follow, yaw_sign, pitch_sign,
        yaw_gain, pitch_gain, deadband_deg, max_rate_deg_s, publish_rate,
        set_control_mode,
        # P7
        transition_timeout, gesture_enabled, use_mux,
        # optional viz / TF
        use_viewer, use_rviz, show_window, publish_csi_tf,
        csi_camera_x, csi_camera_y, csi_camera_z,
        csi_camera_roll, csi_camera_pitch, csi_camera_yaw,
        # nodes
        base,
        perception,
        control,
        viewer,
        rviz,
        csi_camera_link,
        csi_optical,
    ])
