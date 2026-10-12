import os

from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # robot_description and TF are published by rto_bringup_launch.py
    default_rviz_config_path = os.path.join(
        get_package_share_path('rto_bringup'), 'rto_config.rviz'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'rviz_config_file',
            default_value=default_rviz_config_path,
            description='Full path to the RViz config file to use',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', LaunchConfiguration('rviz_config_file')],
        ),
    ])
