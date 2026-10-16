import os
import tempfile

from ament_index_python.packages import get_package_share_path
from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml


def launch_slam_toolbox(context: LaunchContext):
    mode = context.perform_substitution(LaunchConfiguration('mode'))
    map_name = context.perform_substitution(LaunchConfiguration('map'))
    params_file = context.perform_substitution(LaunchConfiguration('slam_params_file'))
    if not params_file:
        params_file = os.path.join(
            get_package_share_path('rto_bringup'), 'config', f'slam_toolbox_{mode}.yaml'
        )

    if map_name:
        # slam_toolbox appends .posegraph and .data to map_file_name itself
        map_name = os.path.abspath(os.path.expanduser(map_name))
        map_name = os.path.splitext(map_name)[0] if map_name.endswith('.posegraph') else map_name
        if not os.path.isfile(f'{map_name}.posegraph'):
            raise RuntimeError(f"Serialized map '{map_name}.posegraph' does not exist")
        with open(params_file) as f:
            params = yaml.safe_load(f)
        params['slam_toolbox']['ros__parameters'].update({
            'map_file_name': map_name,
            'map_start_pose': [0.0, 0.0, 0.0],
        })
        with tempfile.NamedTemporaryFile(
            'w', prefix='slam_toolbox_', suffix='.yaml', delete=False
        ) as f:
            yaml.safe_dump(params, f)
            params_file = f.name
    elif mode == 'localization':
        raise RuntimeError('Localization mode requires a serialized map: map:=<path>')

    launch_file = 'online_async_launch.py' if mode == 'mapping' else 'localization_launch.py'
    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_path('slam_toolbox'), 'launch', launch_file)
            ),
            launch_arguments={
                'slam_params_file': params_file,
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }.items(),
        )
    ]


def generate_launch_description():
    # Laser scans and odom -> base_link TF are published by rto_bringup_launch.py
    # or by robotino_gazebo gazebo_sim.launch.py
    return LaunchDescription([
        DeclareLaunchArgument(
            'mode',
            default_value='mapping',
            choices=['mapping', 'localization'],
            description='Build a map or localize in a serialized map without changing it',
        ),
        DeclareLaunchArgument(
            'map',
            default_value='',
            description='Serialized pose graph without extension (saved by "Serialize Map"): '
                        'mapping continues it, localization requires it',
        ),
        DeclareLaunchArgument(
            'slam_params_file',
            default_value='',
            description='slam_toolbox parameters, '
                        'empty: rto_bringup/config/slam_toolbox_<mode>.yaml',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            choices=['true', 'false'],
            description='Use the Gazebo clock, true for the simulation',
        ),
        DeclareLaunchArgument(
            'rviz',
            default_value='true',
            choices=['true', 'false'],
            description='Start RViz with the map and the slam_toolbox panel',
        ),
        DeclareLaunchArgument(
            'rviz_config_file',
            default_value=os.path.join(get_package_share_path('rto_bringup'), 'slam.rviz'),
            description='Full path to the RViz config file to use',
        ),
        OpaqueFunction(function=launch_slam_toolbox),
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
