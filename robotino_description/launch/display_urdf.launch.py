import os
from ament_index_python.packages import get_package_share_path

from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node


def launch_robot_state_publisher(context: LaunchContext, robot_model):
    """source: https://answers.ros.org/question/396345/ros2-launch-file-how-to-convert-launchargument-to-string/"""
    robot_model_str = context.perform_substitution(robot_model)
    xacro_model_path = os.path.join(
        get_package_share_path("robotino_description"),
        "urdf",
        "{}.urdf.xacro".format(robot_model_str),
    )
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": Command(["xacro ", xacro_model_path])}],
    )
    return [robot_state_publisher_node]


def generate_launch_description():
    robotino_description_path = get_package_share_path("robotino_description")
    default_rviz_config_path = os.path.join(robotino_description_path, "rviz/urdf.rviz")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                name="gui",
                default_value="true",
                choices=["true", "false"],
                description="Flag to enable joint_state_publisher_gui",
            ),
            DeclareLaunchArgument(
                name="rvizconfig",
                default_value=str(default_rviz_config_path),
                description="Absolute path to rviz config file",
            ),
            DeclareLaunchArgument(
                name="robot_model",
                default_value="robotino4",
                choices=[
                    "robotino3",
                    "robotino4",
                    "robotino3_with_tower",
                    "robotino4_with_tower",
                ],
            ),
            OpaqueFunction(
                function=launch_robot_state_publisher,
                args=[LaunchConfiguration("robot_model")],
            ),
            # Depending on gui parameter, either launch joint_state_publisher or joint_state_publisher_gui
            Node(
                package="joint_state_publisher",
                executable="joint_state_publisher",
                condition=UnlessCondition(LaunchConfiguration("gui")),
            ),
            Node(
                package="joint_state_publisher_gui",
                executable="joint_state_publisher_gui",
                condition=IfCondition(LaunchConfiguration("gui")),
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", LaunchConfiguration("rvizconfig")],
            ),
        ]
    )
