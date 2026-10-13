import os

from ament_index_python.packages import get_package_share_path
from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def launch_robot_state_publisher(context: LaunchContext, robot_model):
    robot_model_str = context.perform_substitution(robot_model)
    xacro_model_path = os.path.join(
        get_package_share_path('robotino_description'), 'urdf', f'{robot_model_str}.urdf.xacro'
    )
    robot_description = ParameterValue(Command(['xacro ', xacro_model_path]), value_type=str)

    return [
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
        )
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'hostname',
            default_value='172.26.1.1',
            description='Robotino IP Address',
        ),
        DeclareLaunchArgument(
            'robot_model',
            default_value='robotino3',
            choices=[
                'robotino3',
                'robotino4',
                'robotino3_with_tower',
                'robotino4_with_tower',
            ],
            description='Robot model from robotino_description',
        ),
        Node(
            package='robotino_node',
            executable='robotino_node',
            name='robotino',
            parameters=[{'hostname': LaunchConfiguration('hostname')}],
            remappings=[('/robotino/joint_states', '/joint_states')],
        ),
        OpaqueFunction(
            function=launch_robot_state_publisher,
            args=[LaunchConfiguration('robot_model')],
        ),
    ])
