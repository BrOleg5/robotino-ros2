import os

from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Topics and services of the nodes are in the robotino namespace, next to the I/O interface
    # of robotino_node: /robotino/gpio_states, /robotino/gpio_commands, /robotino/gripper/...
    params_file = LaunchConfiguration('params_file')
    use_sim_time = {'use_sim_time': LaunchConfiguration('use_sim_time')}

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
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
        Node(
            package='rto_io',
            executable='gripper',
            name='gripper',
            namespace='robotino',
            output='screen',
            parameters=[params_file, use_sim_time],
        ),
        Node(
            package='rto_io',
            executable='status_leds',
            name='status_leds',
            namespace='robotino',
            output='screen',
            parameters=[params_file, use_sim_time],
            remappings=[('collision_monitor_state', '/collision_monitor_state')],
        ),
    ])
