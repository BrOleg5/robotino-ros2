import os

from ament_index_python.packages import get_package_share_path
from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_teleop(context: LaunchContext):
    joy_vel = context.perform_substitution(LaunchConfiguration('joy_vel'))
    if not joy_vel:
        # With the safety layer commands go through teleop_safety and collision_monitor
        safety = context.perform_substitution(LaunchConfiguration('safety')) == 'true'
        joy_vel = '/robotino/cmd_vel_teleop' if safety else '/robotino/cmd_vel'

    return [
        Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_twist_joy_node',
            parameters=[
                LaunchConfiguration('config_filepath'),
                {'use_sim_time': LaunchConfiguration('use_sim_time')},
            ],
            remappings=[('/cmd_vel', joy_vel)],
        )
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
            description='teleop_twist_joy parameters: gamepad axes, buttons and speed scales',
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
            }.items(),
            condition=IfCondition(LaunchConfiguration('safety')),
        ),
    ])
