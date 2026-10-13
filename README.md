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
  - `camera:=false` disables the camera node
  - `camera_frame_id:=<frame>` overrides the frame of camera images. By default it is `camera_optical_frame`
    (Logitech C920) for Robotino 3 and `camera_color_optical_frame` (RGB module of RealSense D435, published
    by `realsense2_camera`) for Robotino 4
  - `camera_info_url:=file:///path/to/calibration.yaml` sets the camera calibration file
- To launch RViz visualization, use `ros2 launch rto_bringup rto_rviz_launch.py`
- To view the robot model only, use `ros2 launch robotino_description display_urdf.launch.py robot_model:=robotino4`
- **Note:** The robotino teleop package was not ported fully. To use keyboard teleop, run
  `ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/robotino/cmd_vel`
  (`teleop_twist_keyboard` is preinstalled in the dev container).

### Camera

`robotino_camera_node` receives images of the Robotino camera via Robotino API2 and publishes them
in the `camera` namespace with the topic names of `image_transport`:

| Topic / service | Type | Description |
| --- | --- | --- |
| `/camera/image_raw` | `sensor_msgs/Image` | Image, `rgb8` |
| `/camera/image_raw/compressed` | `sensor_msgs/CompressedImage` | JPEG image, `compressed` transport of `image_transport`. Use it over Wi-Fi and for `ros2 bag` |
| `/camera/camera_info` | `sensor_msgs/CameraInfo` | Calibration |
| `/camera/robotino_camera/set_camera_info` | `sensor_msgs/SetCameraInfo` | Saves calibration to `camera_info_url` |

JPEG images transferred by Robotino (`image_format:=mjpg`) are published to `image_raw/compressed` as is,
without decoding and re-encoding. They are decoded (libjpeg-turbo) only when `image_raw` has subscribers.
Raw images (`image_format:=raw`) are JPEG-encoded only when `image_raw/compressed` has subscribers.
Without subscribers the node does no image processing at all.

The node keeps running while Robotino is unreachable and reconnects automatically,
camera settings are applied again on every connection.
Run it alone with
`ros2 run robotino_node robotino_camera_node --ros-args -r __ns:=/camera -p hostname:=<robotino ip address>`.

| Parameter | Default | Description |
| --- | --- | --- |
| `hostname` | `192.168.0.1` | Robotino IP address |
| `camera_number` | `0` | Camera number, if Robotino has several cameras |
| `frame_id` | `camera_optical_frame` | Frame of published messages |
| `image_width`, `image_height` | `0` | Requested resolution, `0` keeps the current camera settings |
| `image_format` | `mjpg` | Transfer format for the requested resolution: `mjpg` or `raw` |
| `jpeg_quality` | `80` | JPEG quality of `image_raw/compressed` if Robotino transfers raw images |
| `camera_name` | `camera` | Camera name in the calibration file |
| `camera_info_url` | `""` | Calibration file, empty means `~/.ros/camera_info/<camera_name>.yaml` |
| `timer_period_ms` | `20` | Period of polling Robotino API2 events |

Without calibration `/camera/camera_info` contains only the image size. To calibrate, run
`ros2 run camera_calibration cameracalibrator --size 8x6 --square 0.025 --ros-args -r image:=/camera/image_raw -r camera/set_camera_info:=/camera/robotino_camera/set_camera_info`
(set `--size` and `--square` for your checkerboard)
and press **Commit**: the calibration is saved to `camera_info_url`.

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
| `/camera/image_raw`, `/camera/camera_info` | `sensor_msgs/Image`, `sensor_msgs/CameraInfo` | RGB camera, compressed transports as on the real robot |

The camera is a generic RGB camera (`generic_camera` macro in
[robotino_gazebo/urdf/generic_camera.urdf.xacro](robotino_gazebo/urdf/generic_camera.urdf.xacro)):
its resolution, field of view, frame rate and noise are macro parameters set in
[robotino_sim.urdf.xacro](robotino_gazebo/urdf/robotino_sim.urdf.xacro).
