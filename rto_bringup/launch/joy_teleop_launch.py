import os

from ament_index_python.packages import get_package_share_path
from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from rto_bringup.joy_bindings import load_joy_config


def launch_teleop(context: LaunchContext):
    joy_vel = context.perform_substitution(LaunchConfiguration('joy_vel'))
    if not joy_vel:
        # With the safety layer commands go through teleop_safety and collision_monitor
        safety = context.perform_substitution(LaunchConfiguration('safety')) == 'true'
        joy_vel = '/robotino/cmd_vel_teleop' if safety else '/robotino/cmd_vel'
    # Buttons and axes are bound by name in joy_teleop.yaml: convert them to parameters
    params = load_joy_config(context.perform_substitution(LaunchConfiguration('config_filepath')))
    use_sim_time = {'use_sim_time': LaunchConfiguration('use_sim_time')}

    return [
        Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_twist_joy_node',
            parameters=[params['teleop_twist_joy_node'], use_sim_time],
            remappings=[('/cmd_vel', joy_vel)],
        ),
        Node(
            package='rto_teleop',
            executable='gripper_joy',
            name='gripper_joy',
            namespace='robotino',
            parameters=[params['gripper_joy'], use_sim_time],
            remappings=[('joy', '/joy')],
            condition=IfCondition(LaunchConfiguration('gripper')),
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'joy_dev',
            default_value='0',
            description='Gamepad device id',
        ),
        DeclareLaunchArgument(
            'joy_vel',
            default_value='',
            description='Topic of velocity commands, empty: /robotino/cmd_vel_teleop with the '
                        'safety layer, /robotino/cmd_vel without it',
        ),
        DeclareLaunchArgument(
            'safety',
            default_value='true',
            choices=['true', 'false'],
            description='Start the safety layer: slowdown near obstacles and emergency stop',
        ),
        DeclareLaunchArgument(
            'laser',
            default_value='true',
            choices=['true', 'false'],
            description='Safety layer uses the laser scan, false for a robot without the laser',
        ),
        DeclareLaunchArgument(
            'gripper',
            default_value='true',
            choices=['true', 'false'],
            description='Start the gripper and the status LEDs on the I/O interface, '
                        'open / close the gripper with the gamepad',
        ),
        DeclareLaunchArgument(
            'gripper_params_file',
            default_value=os.path.join(
                get_package_share_path('rto_bringup'), 'config', 'gripper.yaml'
            ),
            description='Channels of the I/O interface used by the gripper and the status LEDs',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            choices=['true', 'false'],
            description='Use the Gazebo clock, true for the simulation',
        ),
        DeclareLaunchArgument(
            'config_filepath',
            default_value=os.path.join(
                get_package_share_path('rto_bringup'), 'config', 'joy_teleop.yaml'
            ),
            description='Gamepad bindings: buttons and axes of the commands, speeds',
        ),
        # game_controller_node maps axes and buttons of any gamepad known to SDL to one layout,
        # so joy_teleop.yaml does not depend on the gamepad model and its driver.
        Node(
            package='joy',
            executable='game_controller_node',
            name='game_controller_node',
            parameters=[{
                'device_id': LaunchConfiguration('joy_dev'),
                'deadzone': 0.1,
                'autorepeat_rate': 20.0,
            }],
        ),
        OpaqueFunction(function=launch_teleop),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_path('rto_bringup'), 'teleop_safety_launch.py'
            )),
            launch_arguments={
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'laser': LaunchConfiguration('laser'),
                'joy_config': LaunchConfiguration('config_filepath'),
            }.items(),
            condition=IfCondition(LaunchConfiguration('safety')),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_path('rto_bringup'), 'gripper_launch.py'
            )),
            launch_arguments={
                'params_file': LaunchConfiguration('gripper_params_file'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }.items(),
            condition=IfCondition(LaunchConfiguration('gripper')),
        ),
    ])
