"""P6/P7: visual gimbal following, gesture commands and the velocity mux.

This is the P6/P7 subset.  For the full P3-P7 stack in one launch (camera,
persons, gestures, following and the mux), use perception_control.launch.py.

Nodes:
  gimbal_follow_node       : /perception/persons -> /cmd_gimbal_angle
  gesture_command_bridge   : /perception/gestures -> /perception/{follow_enable,
                             lock_target} and /cmd_vel_gesture
  cmd_vel_mux              : safety > gesture > planner -> /cmd_vel_acc

Tracking is idle at start.  Show an open hand (palm, or pointing_up/victory if
that is how the model labels it), then close it into a fist: the bridge locks
that person's track id and enables following.  Open the hand again
(fist -> open hand) to release the lock and stop following.  This works
whenever the bridge runs; no service call is needed.

Forward motion with thumbs_up additionally needs gesture mode, entered
explicitly:
  ros2 service call /gesture_command_bridge/enable std_srvs/srv/SetBool \
      "{data: true}"

The camera/perception side (person_gesture_node with enable_gesture:=true) must
run separately, e.g.:
  ros2 launch roborts_perception perception_view.launch.py enable_gesture:=true

IMPORTANT: when the mux runs, the planner must not publish /cmd_vel_acc
directly.  Launch the navigation stack with ``use_gesture_control:=true`` so
local_planner is remapped to /cmd_vel_acc_planner (see
roborts_vslam/launch/vslam_navigation.launch.py).

Usage:
  ros2 launch roborts_perception gesture_control.launch.py
  ros2 launch roborts_perception gesture_control.launch.py follow_enabled:=true
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    follow_enabled = DeclareLaunchArgument(
        'follow_enabled', default_value='false',
        description='Follow the best-scoring person before any gesture lock '
                    '(normally off; open hand->fist starts locked tracking)')
    transition_timeout = DeclareLaunchArgument(
        'transition_timeout_s', default_value='1.5',
        description='Seconds between the open hand and fist for a tracking edge')
    allow_chassis_follow = DeclareLaunchArgument(
        'allow_chassis_follow', default_value='false',
        description='Allow the chassis to follow when yaw hits its limit')
    yaw_sign = DeclareLaunchArgument(
        'yaw_sign', default_value='-1.0',
        description='Camera yaw sign relative to gimbal yaw (bench test)')
    pitch_sign = DeclareLaunchArgument(
        'pitch_sign', default_value='-1.0',
        description='Camera pitch sign relative to gimbal pitch')
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
        description='Start in gesture-control mode (can also use service)')
    use_mux = DeclareLaunchArgument(
        'use_mux', default_value='true',
        description='Arbitrate /cmd_vel_acc through cmd_vel_mux')

    gimbal_follow = Node(
        package='roborts_perception',
        executable='gimbal_follow_node',
        name='gimbal_follow_node',
        output='screen',
        parameters=[{
            'follow_enabled': ParameterValue(
                LaunchConfiguration('follow_enabled'), value_type=bool),
            'allow_chassis_follow': ParameterValue(
                LaunchConfiguration('allow_chassis_follow'), value_type=bool),
            'yaw_sign': ParameterValue(
                LaunchConfiguration('yaw_sign'), value_type=float),
            'pitch_sign': ParameterValue(
                LaunchConfiguration('pitch_sign'), value_type=float),
            'yaw_gain': ParameterValue(
                LaunchConfiguration('yaw_gain'), value_type=float),
            'pitch_gain': ParameterValue(
                LaunchConfiguration('pitch_gain'), value_type=float),
            'deadband_deg': ParameterValue(
                LaunchConfiguration('deadband_deg'), value_type=float),
            'max_rate_deg_s': ParameterValue(
                LaunchConfiguration('max_rate_deg_s'), value_type=float),
            'publish_rate': ParameterValue(
                LaunchConfiguration('publish_rate'), value_type=float),
            'set_control_mode': ParameterValue(
                LaunchConfiguration('set_control_mode'), value_type=bool),
        }],
    )

    gesture_bridge = Node(
        package='roborts_perception',
        executable='gesture_command_bridge',
        name='gesture_command_bridge',
        output='screen',
        parameters=[{
            'enabled': ParameterValue(
                LaunchConfiguration('gesture_enabled'), value_type=bool),
            'transition_timeout_s': ParameterValue(
                LaunchConfiguration('transition_timeout_s'), value_type=float),
            # Keep the bridge's initial follow state in sync with the gimbal
            # node.  Normally both are off until the open hand->fist edge.
            'follow_enabled_on_start': ParameterValue(
                LaunchConfiguration('follow_enabled'), value_type=bool),
        }],
    )

    mux = Node(
        package='roborts_perception',
        executable='cmd_vel_mux',
        name='cmd_vel_mux',
        output='screen',
        parameters=[{
            'planner_topic': '/cmd_vel_acc_planner',
            'gesture_topic': '/cmd_vel_gesture',
            'output_topic': '/cmd_vel_acc',
        }],
        condition=IfCondition(LaunchConfiguration('use_mux')),
    )

    return LaunchDescription([
        follow_enabled,
        transition_timeout,
        allow_chassis_follow,
        yaw_sign,
        pitch_sign,
        yaw_gain,
        pitch_gain,
        deadband_deg,
        max_rate_deg_s,
        publish_rate,
        set_control_mode,
        gesture_enabled,
        use_mux,
        gimbal_follow,
        gesture_bridge,
        mux,
    ])
