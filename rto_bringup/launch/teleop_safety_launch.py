import math
import os

from ament_index_python.packages import get_package_share_path
from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml

# Robotino 3 and 4 are circles of this radius around base_link
ROBOT_RADIUS = 0.225
# Number of velocity directions of the omnidirectional robot with own zone polygons
SECTORS = 24


def points_str(points):
    return '[' + ', '.join(f'[{x:.3f}, {y:.3f}]' for x, y in points) + ']'


def velocity_polygons(name: str, distance: float):
    """
    Generate sub-polygons of a holonomic velocity_polygon of nav2 collision_monitor.

    Every sector of motion directions gets a polygon covering obstacles that the round robot can
    hit when it moves in a direction of the sector: the part of the circle of radius
    ROBOT_RADIUS + distance ahead of the robot, within the robot width widened by the sector width.
    Obstacles behind and beside the robot are not covered, so it can move away from an obstacle
    or along a wall. Rotation in place gets a polygon inside the body: it is never limited.
    """
    radius = ROBOT_RADIUS + distance
    half_sector = math.pi / SECTORS
    half_width = min(ROBOT_RADIUS + radius * math.sin(half_sector), radius)
    edge = math.asin(half_width / radius)
    arc = [-edge + 2 * edge * i / 12 for i in range(13)]
    shape = ([(0.0, -half_width)] + [(radius * math.cos(a), radius * math.sin(a)) for a in arc]
             + [(0.0, half_width)])

    sub_polygons = ['rotation']
    params = {
        f'{name}.rotation.points': points_str(
            [(0.01, 0.0), (0.0, 0.01), (-0.01, 0.0), (0.0, -0.01)]),
        f'{name}.rotation.linear_min': 0.0,
        f'{name}.rotation.linear_max': 0.01,
        f'{name}.rotation.theta_min': -10.0,
        f'{name}.rotation.theta_max': 10.0,
    }
    for i in range(SECTORS):
        direction = math.atan2(math.sin(2 * half_sector * i), math.cos(2 * half_sector * i))
        sub = f'direction{i}'
        sub_polygons.append(sub)
        cos_d, sin_d = math.cos(direction), math.sin(direction)
        start = direction - half_sector
        end = direction + half_sector
        params.update({
            f'{name}.{sub}.points': points_str(
                [(x * cos_d - y * sin_d, x * sin_d + y * cos_d) for x, y in shape]),
            f'{name}.{sub}.linear_min': 0.0,
            f'{name}.{sub}.linear_max': 10.0,
            f'{name}.{sub}.theta_min': -10.0,
            f'{name}.{sub}.theta_max': 10.0,
            f'{name}.{sub}.direction_start_angle': math.atan2(math.sin(start), math.cos(start)),
            f'{name}.{sub}.direction_end_angle': math.atan2(math.sin(end), math.cos(end)),
        })
    params[f'{name}.velocity_polygons'] = sub_polygons
    return params


def launch_collision_monitor(context: LaunchContext):
    params_file = LaunchConfiguration('params_file')
    use_sim_time = {'use_sim_time': LaunchConfiguration('use_sim_time')}
    stop_distance = float(context.perform_substitution(LaunchConfiguration('stop_distance')))
    slow_distance = float(context.perform_substitution(LaunchConfiguration('slow_distance')))
    sources = {}
    if context.perform_substitution(LaunchConfiguration('laser')) != 'true':
        # Without the laser source collision_monitor does not stop the robot for missing scans
        with open(context.perform_substitution(params_file)) as f:
            params = yaml.safe_load(f)['collision_monitor']['ros__parameters']
        sources['observation_sources'] = [
            source for source in params['observation_sources']
            if params[source]['type'] != 'scan']

    return [
        Node(
            package='nav2_collision_monitor',
            executable='collision_monitor',
            name='collision_monitor',
            output='screen',
            parameters=[
                params_file,
                use_sim_time,
                velocity_polygons('StopZone', stop_distance),
                velocity_polygons('SlowZone', slow_distance),
                sources,
            ],
        ),
    ]


def generate_launch_description():
    # Velocity commands of the teleop are expected on /robotino/cmd_vel_teleop:
    # teleop_safety -> /robotino/cmd_vel_safe -> collision_monitor -> /robotino/cmd_vel
    params_file = LaunchConfiguration('params_file')
    use_sim_time = {'use_sim_time': LaunchConfiguration('use_sim_time')}

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(
                get_package_share_path('rto_bringup'), 'config', 'teleop_safety.yaml'
            ),
            description='collision_monitor and teleop_safety parameters',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            choices=['true', 'false'],
            description='Use the Gazebo clock, true for the simulation',
        ),
        DeclareLaunchArgument(
            'laser',
            default_value='true',
            choices=['true', 'false'],
            description='Use the laser scan /laser_scan, false for a robot without the laser: '
                        'only the IR sensors see obstacles',
        ),
        DeclareLaunchArgument(
            'stop_distance',
            default_value='0.075',
            description='Stop when an obstacle ahead is closer to the body [m]',
        ),
        DeclareLaunchArgument(
            'slow_distance',
            default_value='0.425',
            description='Limit the speed when an obstacle ahead is closer to the body [m]',
        ),
        Node(
            package='rto_teleop',
            executable='teleop_safety',
            name='teleop_safety',
            output='screen',
            parameters=[params_file, use_sim_time],
            remappings=[
                ('cmd_vel_in', '/robotino/cmd_vel_teleop'),
                ('cmd_vel_out', '/robotino/cmd_vel_safe'),
                ('bumper', '/robotino/bumper'),
                ('contact', '/robotino/gripper/contact'),
                ('joy', '/joy'),
                ('emergency_stop', '/robotino/safety/emergency_stop'),
                ('motion_blocked', '/robotino/safety/motion_blocked'),
                ('reset', '/robotino/safety/reset'),
            ],
        ),
        OpaqueFunction(function=launch_collision_monitor),
        # collision_monitor is a lifecycle node: configure and activate it
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_teleop_safety',
            output='screen',
            parameters=[params_file, use_sim_time],
        ),
    ])
