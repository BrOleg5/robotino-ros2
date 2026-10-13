# RTO-ROS 2

This repository contains code for packages that enable ROS 2 support for the **Festo Robotino 3/4**.

Supported ROS 2 distributions: **Lyrical** (default, Ubuntu 26.04) and **Jazzy** (Ubuntu 24.04).

The original code was taken from [dietriro's robotino repository](https://github.com/dietriro/rto_core) and adapted for ROS2

**Note:** Not all functionality has been enabled. North Star support, BHA support, and gripper support have been disabled.

## Development in a dev container (recommended)

The [.devcontainer](.devcontainer) directory provides an image with ROS 2, Robotino API2 and all build dependencies.

Host requirements:
- Linux with X11, amd64 (Robotino API2 is available only for amd64)
- Docker and [NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html) (for GPU-accelerated RViz)
- VS Code with the Dev Containers extension

Usage:
- Open the repository in VS Code and run **Dev Containers: Reopen in Container**.
  On the first start the image is built, then `rosdep install` and `colcon build` run automatically.
- The repository is mounted to `/home/ros/ws/src/robotino-ros2`; colcon `build`, `install` and `log`
  directories live in `/home/ros/ws` (Docker volumes that survive container rebuilds).
- Rebuild manually: `cd ~/ws && colcon build --symlink-install`, then open a new terminal or `source ~/ws/install/setup.bash`.
- The container uses the host network, so it can reach the Robotino by IP and talk to other ROS 2 nodes via DDS.
  Set `ROS_DOMAIN_ID` on the host before opening the container to change the domain.
- To switch the ROS 2 distribution, change `ROS_DISTRO` in [devcontainer.json](.devcontainer/devcontainer.json),
  run **Dev Containers: Rebuild Container** and clean the old build: `rm -rf ~/ws/build/* ~/ws/install/* ~/ws/log/*`.

## Installation without a container

On Ubuntu 26.04 with ROS 2 Lyrical (or Ubuntu 24.04 with ROS 2 Jazzy):
- Download `robotino-dev`, `rec-rpc` and `robotino-api2` .deb files for Ubuntu 20.04 from the
  [Robotino OS page](https://wiki.openrobotino.org/index.php?title=Robotino_OS).
- These packages depend on `libqt5core5a`, `libqt5network5`, `libqt5xml5`, `libqt5gui5`, which are named
  `libqt5core5t64`, `libqt5network5t64`, `libqt5xml5t64`, `libqt5gui5t64` in newer Ubuntu releases.
  Rewrite the `Depends` field before installing, as done in [.devcontainer/Dockerfile](.devcontainer/Dockerfile):
  ```bash
  dpkg-deb -R robotino-api2_1.1.14_amd64.deb robotino-api2
  sed -i -E '/^Depends:/ { s/libqt5core5a\b/libqt5core5t64/; s/libqt5(gui5|network5|xml5)\b/libqt5\1t64/g }' robotino-api2/DEBIAN/control
  dpkg-deb -b robotino-api2 robotino-api2_1.1.14_amd64_fixed.deb
  sudo apt install ./robotino-api2_1.1.14_amd64_fixed.deb
  ```
- Clone this repo into the `src` directory of a ROS 2 workspace, then run
  `rosdep install --from-paths src --ignore-src -y` and `colcon build`.
- Source the workspace.

## Usage
- To bring up the robot, use `ros2 launch rto_bringup rto_bringup_launch.py hostname:=<robotino ip address> robot_model:=robotino3`
- To launch RViz visualization, use `ros2 launch rto_bringup rto_rviz_launch.py`
- To view the robot model only, use `ros2 launch robotino_description display_urdf.launch.py robot_model:=robotino4`
- **Note:** The robotino teleop package was not ported fully. To use keyboard teleop, run
  `ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/robotino/cmd_vel`
  (`teleop_twist_keyboard` is preinstalled in the dev container).
- To drive the robot with a gamepad, see [Gamepad teleop](#gamepad-teleop).

### Gamepad teleop

`ros2 launch rto_bringup joy_teleop_launch.py` starts `game_controller_node` of the `joy` package and
`teleop_twist_joy`, which publish
velocity commands to `/robotino/cmd_vel`. It works with the real robot and the simulation alike.

| Control | Action |
| --- | --- |
| LB (hold) | Enable motion; releasing it stops the robot |
| RB (hold) | Turbo: higher speed |
| Left stick | Drive forward/backward and sideways |
| Right stick, horizontal | Rotate |

| Argument | Default | Description |
| --- | --- | --- |
| `joy_dev` | `0` | Gamepad device id, if several gamepads are connected |
| `joy_vel` | `/robotino/cmd_vel` | Topic of velocity commands |
| `config_filepath` | `rto_bringup/config/joy_teleop.yaml` | Axes, buttons and speed scales |

`game_controller_node` maps every gamepad known to SDL to the same layout of axes and buttons, so
`joy_teleop.yaml` does not depend on the gamepad model (buttons are named as on an Xbox gamepad).
`joy_node` publishes the raw layout of the gamepad driver instead, which does not match this file.
To change the mapping, check axis and button numbers with `ros2 run joy game_controller_node` and
`ros2 topic echo /joy`, then pass your own file with `config_filepath:=<path>`.

The dev container gets the host's `/dev/input`, so gamepads connected to the host are available inside
it, including ones plugged in after the container start. The container user needs read access to
`/dev/input/event*` on the host; if the gamepad is not found, check the permissions of its device.

## Simulation in Gazebo

The `robotino_gazebo` package simulates Robotino in Gazebo (gz-sim) with ros2_control.
The simulated robot exposes the same topics and frames as `robotino_node`, so the rest of the stack
works with the simulation and the real robot alike.

- Start the simulation:
  `ros2 launch robotino_gazebo gazebo_sim.launch.py robot_model:=robotino4`
  - `world:=robotino_test`: a room with obstacles; the default `empty` is an empty world.
    A path to any gz-sim `.sdf` world file is accepted too
  - `gui:=false` runs Gazebo headless, `rviz:=false` disables RViz
  - `x:=`, `y:=`, `yaw:=` set the spawn pose
- Drive the robot:
  `ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/robotino/cmd_vel`

Wheels are velocity controlled by `omni_wheel_drive_controller`, which also computes the wheel odometry.
The omniwheel rollers are modeled by anisotropic friction: wheels grip only in the rolling direction.

| Topic | Type | Description |
| --- | --- | --- |
| `/robotino/cmd_vel` | `geometry_msgs/Twist` | Velocity command |
| `/robotino/odom` | `nav_msgs/Odometry` | Wheel odometry, also published as TF `odom` → `base_link` |
| `/joint_states` | `sensor_msgs/JointState` | Wheel joints |
| `/laser_scan` | `sensor_msgs/LaserScan` | Hokuyo laser range finder |
| `/robotino/ir0` … `/robotino/ir8` | `sensor_msgs/Range` | Infrared distance sensors |
| `/robotino/bumper` | `std_msgs/Bool` | Bumper contact |
| `/image_raw`, `/camera_info` | `sensor_msgs/Image`, `sensor_msgs/CameraInfo` | RGB camera |

The camera is a generic RGB camera (`generic_camera` macro in
[robotino_gazebo/urdf/generic_camera.urdf.xacro](robotino_gazebo/urdf/generic_camera.urdf.xacro)):
its resolution, field of view, frame rate and noise are macro parameters set in
[robotino_sim.urdf.xacro](robotino_gazebo/urdf/robotino_sim.urdf.xacro).
