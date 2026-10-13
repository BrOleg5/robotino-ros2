import os

from ament_index_python.packages import get_package_share_path
from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
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


def launch_camera_node(context: LaunchContext, robot_model, camera_frame_id):
    camera_frame_id_str = context.perform_substitution(camera_frame_id)
    if not camera_frame_id_str:
        # Robotino 3: Logitech C920 from robotino_description;
        # Robotino 4: RGB module of RealSense D435, the frame is published by realsense2_camera
        robot_model_str = context.perform_substitution(robot_model)
        if robot_model_str.startswith('robotino4'):
            camera_frame_id_str = 'camera_color_optical_frame'
        else:
            camera_frame_id_str = 'camera_optical_frame'

    return [
        Node(
            package='robotino_node',
            executable='robotino_camera_node',
            name='camera',
            parameters=[{
                'hostname': LaunchConfiguration('hostname'),
                'frame_id': camera_frame_id_str,
                'camera_info_url': LaunchConfiguration('camera_info_url'),
            }],
            condition=IfCondition(LaunchConfiguration('camera')),
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
            'camera',
            default_value='true',
            choices=['true', 'false'],
            description='Publish images of the Robotino camera',
        ),
        DeclareLaunchArgument(
            'camera_frame_id',
            default_value='',
            description='Frame of camera images, empty: optical frame of robot_model camera',
        ),
        DeclareLaunchArgument(
            'camera_info_url',
            default_value='',
            description='Camera calibration URL, empty means ~/.ros/camera_info/camera.yaml',
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
        OpaqueFunction(
            function=launch_camera_node,
            args=[LaunchConfiguration('robot_model'), LaunchConfiguration('camera_frame_id')],
        ),
    ])
