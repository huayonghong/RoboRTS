"""P6/P7: visual gimbal following, gesture commands and the velocity mux.

Nodes:
  gimbal_follow_node       : /perception/persons -> /cmd_gimbal_angle
  gesture_command_bridge   : /perception/gestures -> /cmd_vel_gesture,
                             /perception/{follow_enable,lock_target}
  cmd_vel_mux              : safety > gesture > planner -> /cmd_vel_acc

IMPORTANT: when the mux runs, the planner must not publish /cmd_vel_acc
directly.  Launch the navigation stack with ``use_gesture_control:=true`` so
local_planner is remapped to /cmd_vel_acc_planner (see
roborts_vslam/launch/vslam_navigation.launch.py).

Gesture mode is off by default; enter it explicitly:
  ros2 service call /gesture_command_bridge/enable std_srvs/srv/SetBool \
      "{data: true}"

Usage:
  ros2 launch roborts_perception gesture_control.launch.py
  ros2 launch roborts_perception gesture_control.launch.py follow_enabled:=false
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    follow_enabled = DeclareLaunchArgument(
        'follow_enabled', default_value='true',
        description='Start with visual gimbal following enabled')
    allow_chassis_follow = DeclareLaunchArgument(
        'allow_chassis_follow', default_value='false',
        description='Allow the chassis to follow when yaw hits its limit')
    yaw_sign = DeclareLaunchArgument(
        'yaw_sign', default_value='-1.0',
        description='Camera yaw sign relative to gimbal yaw (bench test)')
    pitch_sign = DeclareLaunchArgument(
        'pitch_sign', default_value='-1.0',
        description='Camera pitch sign relative to gimbal pitch')
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
            # Keep the bridge's toggle state in sync with the gimbal node.
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
        allow_chassis_follow,
        yaw_sign,
        pitch_sign,
        set_control_mode,
        gesture_enabled,
        use_mux,
        gimbal_follow,
        gesture_bridge,
        mux,
    ])
