import os

from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config_filepath = LaunchConfiguration('config_filepath')

    return LaunchDescription([
        DeclareLaunchArgument(
            'joy_dev',
            default_value='0',
            description='Gamepad device id',
        ),
        DeclareLaunchArgument(
            'joy_vel',
            default_value='/robotino/cmd_vel',
            description='Topic of velocity commands',
        ),
        DeclareLaunchArgument(
            'config_filepath',
            default_value=os.path.join(
                get_package_share_path('rto_bringup'), 'config', 'joy_teleop.yaml'
            ),
            description='teleop_twist_joy parameters: gamepad axes, buttons and speed scales',
        ),
        # game_controller_node maps axes and buttons of any gamepad known to SDL to a single layout,
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
        Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_twist_joy_node',
            parameters=[config_filepath],
            remappings=[('/cmd_vel', LaunchConfiguration('joy_vel'))],
        ),
    ])
