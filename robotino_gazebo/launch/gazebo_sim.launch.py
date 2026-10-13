import os

from ament_index_python.packages import get_package_share_path
from launch import LaunchContext, LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def launch_robot(context: LaunchContext, robot_model):
    robot_model_str = context.perform_substitution(robot_model)
    gazebo_path = get_package_share_path('robotino_gazebo')
    base_model = 'robotino3' if robot_model_str.startswith('robotino3') else 'robotino4'
    controllers_file = os.path.join(gazebo_path, 'config', f'{base_model}_controllers.yaml')
    xacro_model_path = os.path.join(gazebo_path, 'urdf', 'robotino_sim.urdf.xacro')

    robot_description = ParameterValue(
        Command([
            'xacro ', xacro_model_path,
            ' robot_model:=', robot_model_str,
            ' controllers_file:=', controllers_file,
        ]),
        value_type=str,
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'robotino',
            '-x', LaunchConfiguration('x'),
            '-y', LaunchConfiguration('y'),
            '-z', '0.01',
            '-Y', LaunchConfiguration('yaw'),
        ],
    )

    # gz_ros2_control starts the controller manager when the robot is spawned
    spawn_joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        arguments=['joint_state_broadcaster', '--controller-manager-timeout', '60'],
        parameters=[{'use_sim_time': True}],
    )
    spawn_drive_controller = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        arguments=[
            'omni_wheel_drive_controller',
            '--param-file', controllers_file,
            '--controller-ros-args', '-r /omni_wheel_drive_controller/odom:=/robotino/odom',
            '--controller-manager-timeout', '60',
        ],
        parameters=[{'use_sim_time': True}],
    )

    return [
        robot_state_publisher,
        spawn_robot,
        RegisterEventHandler(
            OnProcessExit(
                target_action=spawn_robot,
                on_exit=[spawn_joint_state_broadcaster, spawn_drive_controller],
            )
        ),
    ]


def launch_gazebo(context: LaunchContext):
    world = context.perform_substitution(LaunchConfiguration('world'))
    # A bare name refers to a world shipped with this package
    if os.sep not in world:
        name = world if world.endswith('.sdf') else f'{world}.sdf'
        world = os.path.join(get_package_share_path('robotino_gazebo'), 'worlds', name)
    if not os.path.isfile(world):
        raise RuntimeError(f"World file '{world}' does not exist")

    gz_args = ['-r', '-v', context.perform_substitution(LaunchConfiguration('gz_verbosity'))]
    if context.perform_substitution(LaunchConfiguration('gui')) != 'true':
        gz_args += ['-s', '--headless-rendering']
    gz_args.append(world)

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_path('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
            ),
            launch_arguments={'gz_args': ' '.join(gz_args), 'on_exit_shutdown': 'true'}.items(),
        )
    ]


def generate_launch_description():
    gazebo_path = get_package_share_path('robotino_gazebo')
    description_path = get_package_share_path('robotino_description')

    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_model',
            default_value='robotino4',
            choices=[
                'robotino3',
                'robotino4',
                'robotino3_with_tower',
                'robotino4_with_tower',
            ],
            description='Robot model from robotino_description',
        ),
        DeclareLaunchArgument(
            'world',
            default_value='empty',
            description='World name from robotino_gazebo/worlds (empty, robotino_test) '
                        'or path to a gz-sim world file',
        ),
        DeclareLaunchArgument(
            'gui', default_value='true', choices=['true', 'false'],
            description='Start the Gazebo GUI (false runs the server headless)',
        ),
        DeclareLaunchArgument(
            'rviz', default_value='true', choices=['true', 'false'],
            description='Start RViz',
        ),
        DeclareLaunchArgument(
            'rvizconfig',
            default_value=os.path.join(gazebo_path, 'rviz', 'sim.rviz'),
            description='Absolute path to rviz config file',
        ),
        DeclareLaunchArgument(
            'gz_verbosity', default_value='2', description='gz-sim verbosity 0..4'
        ),
        DeclareLaunchArgument('x', default_value='0.0', description='Spawn x position [m]'),
        DeclareLaunchArgument('y', default_value='0.0', description='Spawn y position [m]'),
        DeclareLaunchArgument('yaw', default_value='0.0', description='Spawn yaw [rad]'),

        # Lets gz resolve meshes referenced by package:// URIs (e.g. realsense2_description)
        AppendEnvironmentVariable(
            'GZ_SIM_RESOURCE_PATH', os.path.dirname(os.path.dirname(str(description_path)))
        ),
        AppendEnvironmentVariable(
            'GZ_SIM_RESOURCE_PATH',
            os.path.dirname(str(get_package_share_path('realsense2_description'))),
        ),

        OpaqueFunction(function=launch_gazebo),
        OpaqueFunction(function=launch_robot, args=[LaunchConfiguration('robot_model')]),

        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            output='screen',
            parameters=[{
                'config_file': os.path.join(gazebo_path, 'config', 'bridge.yaml'),
                'use_sim_time': True,
            }],
        ),
        Node(
            package='ros_gz_image',
            executable='image_bridge',
            output='screen',
            arguments=['robotino/sim/camera/image'],
            remappings=[('robotino/sim/camera/image', '/camera/image_raw')],
            parameters=[{'use_sim_time': True}],
        ),
        Node(
            package='robotino_gazebo',
            executable='sim_adapter.py',
            output='screen',
            parameters=[{'use_sim_time': True}],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            arguments=['-d', LaunchConfiguration('rvizconfig')],
            parameters=[{'use_sim_time': True}],
            condition=IfCondition(LaunchConfiguration('rviz')),
        ),
    ])
