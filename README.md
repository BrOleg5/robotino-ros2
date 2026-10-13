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

### I/O interface

`robotino_node` exposes the [I/O interface](https://ip.festo-didactic.com/Infoportal/Robotino/Hardware/Interfaces/EN/IOInterface.html)
with `control_msgs/msg/DynamicInterfaceGroupValues`, the message type of `gpio_controllers/GpioCommandController`:

| Topic | Description |
| --- | --- |
| `/robotino/gpio_states` | States of all channels, published when any of them changes |
| `/robotino/gpio_commands` | Commands for digital outputs and relays |

| Group | Channels | Values | Access |
| --- | --- | --- | --- |
| `digital_inputs` | `DI1` … `DI8` | 0.0 / 1.0 | read |
| `analog_inputs` | `AI1` … `AI8` | 0–10 V | read |
| `digital_outputs` | `DQ1` … `DQ8` | 0.0 / 1.0 | read / write |
| `relays` | `R1`, `R2` | 0.0 / 1.0 | read / write |

A command may contain only some groups and channels; the other channels keep their last commanded value
(0.0 after the node start). For example, to switch on digital output 1 and relay 2:
```bash
ros2 topic pub --once /robotino/gpio_commands control_msgs/msg/DynamicInterfaceGroupValues \
  "{interface_groups: [digital_outputs, relays],
    interface_values: [{interface_names: [DQ1], values: [1.0]}, {interface_names: [R2], values: [1.0]}]}"
```

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
