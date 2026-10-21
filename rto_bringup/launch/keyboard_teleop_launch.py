import os

from ament_index_python.packages import get_package_share_path
from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_teleop(context: LaunchContext):
    # With the safety layer commands go through teleop_safety and collision_monitor
    safety = context.perform_substitution(LaunchConfiguration('safety')) == 'true'
    remappings = [] if safety else [('/robotino/cmd_vel_teleop', '/robotino/cmd_vel')]

    return [
        Node(
            package='rto_teleop',
            executable='keyboard_teleop',
            name='keyboard_teleop',
            output='screen',
            parameters=[
                LaunchConfiguration('config_filepath'),
                {'use_sim_time': LaunchConfiguration('use_sim_time')},
            ],
            remappings=remappings,
        ),
    ]


def generate_launch_description():
    # keyboard_teleop opens a window; keyboard_teleop_tty needs a terminal and cannot be launched
    return LaunchDescription([
        DeclareLaunchArgument(
            'safety',
            default_value='true',
            choices=['true', 'false'],
            description='Start the safety layer: slowdown near obstacles and emergency stop; '
                        'false: commands go directly to /robotino/cmd_vel',
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
                get_package_share_path('rto_bringup'), 'config', 'keyboard_teleop.yaml'
            ),
            description='keyboard_teleop parameters: key bindings and speeds',
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
