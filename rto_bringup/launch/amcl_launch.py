import os

from ament_index_python.packages import get_package_share_path
from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_localization(context: LaunchContext):
    map_file = context.perform_substitution(LaunchConfiguration('map'))
    if not map_file:
        raise RuntimeError('AMCL requires an occupancy grid map: map:=<path>.yaml')
    map_file = os.path.abspath(os.path.expanduser(map_file))
    if not os.path.isfile(map_file):
        raise RuntimeError(f"Map file '{map_file}' does not exist")

    params_file = LaunchConfiguration('params_file')
    use_sim_time = {'use_sim_time': LaunchConfiguration('use_sim_time')}
    global_localization = context.perform_substitution(
        LaunchConfiguration('global_localization')) == 'true'
    amcl_overrides = {'set_initial_pose': False} if global_localization else {}

    actions = [
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[params_file, use_sim_time, {'yaml_filename': map_file}],
        ),
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[params_file, use_sim_time, amcl_overrides],
        ),
        # map_server and amcl are lifecycle nodes: configure and activate them
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[params_file, use_sim_time],
        ),
    ]
    if global_localization:
        # Spread the particles over the free space of the map once amcl is active
        # (it has received the map by then: map_server is activated first)
        actions.append(ExecuteProcess(
            cmd=['bash', '-c',
                 'until ros2 lifecycle get /amcl 2>/dev/null | grep -q "^active"; do sleep 1; done; '
                 'ros2 service call /reinitialize_global_localization std_srvs/srv/Empty'],
            output='screen',
        ))
    return actions


def generate_launch_description():
    # Laser scans and odom -> base_link TF are published by rto_bringup_launch.py
    # or by robotino_gazebo gazebo_sim.launch.py
    rto_bringup_path = get_package_share_path('rto_bringup')

    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value='',
            description='Occupancy grid map (saved by "Save Map"): <path>.yaml',
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(rto_bringup_path, 'config', 'amcl.yaml'),
            description='map_server, amcl and lifecycle_manager parameters',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            choices=['true', 'false'],
            description='Use the Gazebo clock, true for the simulation',
        ),
        DeclareLaunchArgument(
            'global_localization',
            default_value='false',
            choices=['true', 'false'],
            description='Spread the particles over the whole map instead of the initial pose',
        ),
        DeclareLaunchArgument(
            'rviz',
            default_value='true',
            choices=['true', 'false'],
            description='Start RViz with the map and the particle cloud',
        ),
        DeclareLaunchArgument(
            'rviz_config_file',
            default_value=os.path.join(rto_bringup_path, 'amcl.rviz'),
            description='Full path to the RViz config file to use',
        ),
        OpaqueFunction(function=launch_localization),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', LaunchConfiguration('rviz_config_file')],
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
            condition=IfCondition(LaunchConfiguration('rviz')),
        ),
    ])
