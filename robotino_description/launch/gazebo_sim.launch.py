import os
from launch import LaunchDescription, LaunchContext
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    ExecuteProcess,
    OpaqueFunction,
)
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_path

from launch.launch_description_sources import PythonLaunchDescriptionSource


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
                "use_sim_time",
                default_value="false",
                description="Use simulation (Gazebo) clock if true",
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
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        os.path.join(
                            get_package_share_path("gazebo_ros"),
                            "launch",
                            "gazebo.launch.py",
                        )
                    ]
                )
            ),
            # ExecuteProcess(
            #     cmd=["gazebo", "--verbose", "-s", "libgazebo_ros_factory.so"],
            #     output="screen",
            # ),
            OpaqueFunction(
                function=launch_robot_state_publisher,
                args=[LaunchConfiguration("robot_model")],
            ),
            Node(
                package="joint_state_publisher",
                executable="joint_state_publisher",
                name="joint_state_publisher",
                output="screen",
                parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
            ),
            Node(
                package="gazebo_ros",
                executable="spawn_entity.py",
                name="urdf_spawner",
                output="screen",
                arguments=["-topic", "/robot_description", "-entity", "robotino"],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", default_rviz_config_path],
            ),
        ]
    )
