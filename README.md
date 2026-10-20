# RTO-ROS 2

This repository contains code for packages that enable ROS 2 support for the **Festo Robotino 3/4**.

Supported ROS 2 distributions: **Lyrical** (default, Ubuntu 26.04) and **Jazzy** (Ubuntu 24.04).

The original code was taken from [dietriro's robotino repository](https://github.com/dietriro/rto_core) and adapted for ROS2

**Note:** Not all functionality has been enabled. North Star support and BHA support have been disabled.
The gripper is supported when it is connected to the [I/O interface](#gripper-and-status-leds).

## Running in Docker

The quickest way to use the packages: [compose.yaml](compose.yaml) builds an image with ROS 2, Robotino API2
and this workspace ([docker/Dockerfile](docker/Dockerfile)) and runs commands in it. For development use
the [dev container](#development-in-a-dev-container-recommended) instead.

Host requirements:
- Linux with X11, amd64 (Robotino API2 is available only for amd64)
- [Docker Engine](https://docs.docker.com/engine/install/ubuntu/) with the Compose plugin
  (`docker compose version` should work)

Setup:
```bash
git clone https://github.com/BrOleg5/robotino-ros2.git
cd robotino-ros2
docker compose build  # takes a while; rerun after updating the repository
```

Run any command from [Usage](#usage) with `docker compose run --rm robotino <command>` in the repository
directory. Allow the container to open windows (RViz, Gazebo, keyboard teleop) once per login session:
`xhost +local:docker`. Examples:
```bash
# Real robot
docker compose run --rm robotino ros2 launch rto_bringup rto_bringup_launch.py hostname:=<robotino ip address> robot_model:=robotino3
docker compose run --rm robotino ros2 launch rto_bringup rto_rviz_launch.py
docker compose run --rm robotino ros2 launch rto_bringup joy_teleop_launch.py

# Simulation
docker compose run --rm robotino ros2 launch robotino_gazebo gazebo_sim.launch.py robot_model:=robotino4

# Interactive shell with the workspace sourced
docker compose run --rm robotino
```

Each command runs in its own container; they see each other and other ROS 2 nodes on the network,
because the containers use the host network. Notes:
- Settings are taken from environment variables of the host (or an `.env` file next to `compose.yaml`):
  - `ROS_DOMAIN_ID` (default `0`): ROS 2 domain;
  - `ROBOTINO_ROS_DISTRO` (default `lyrical`): ROS 2 distribution of the image, `lyrical` or `jazzy`;
    run `docker compose build` after changing it.
- Gamepads connected to the host (also later) are available in the container.
- RViz and Gazebo render on Intel/AMD GPUs through `/dev/dri`. With an NVIDIA GPU install the
  [NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html)
  and add the override file: `docker compose -f compose.yaml -f docker/compose.nvidia.yaml run --rm robotino <command>`.

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

Requirements: Ubuntu 26.04 with ROS 2 Lyrical or Ubuntu 24.04 with ROS 2 Jazzy, amd64
(Robotino API2 is available only for amd64).

1. Install ROS 2 following the official guide
   ([Lyrical](https://docs.ros.org/en/lyrical/Get-Started/Installation/Ubuntu-Install-Debs.html),
   [Jazzy](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)),
   the `ros-<distro>-desktop` variant is enough.
2. Create a workspace and clone this repository into it:
   ```bash
   mkdir -p ~/ws/src && cd ~/ws/src
   git clone https://github.com/BrOleg5/robotino-ros2.git
   ```
3. Install the dependencies:
   ```bash
   source /opt/ros/<distro>/setup.bash
   ~/ws/src/robotino-ros2/scripts/install_dependencies.sh
   ```
   The script (uses `sudo`):
   - downloads `rec-rpc`, `robotino-dev` and `robotino-api2` from the
     [openrobotino.org package repository](https://packages2.openrobotino.org/pool/focal/main/r/)
     and checks their SHA-256 sums;
   - rewrites their `Depends` field: the packages are built for Ubuntu 20.04 and depend on `libqt5core5a`,
     `libqt5gui5`, `libqt5network5`, `libqt5xml5`, which are named `libqt5core5t64`, `libqt5gui5t64`,
     `libqt5network5t64`, `libqt5xml5t64` in Ubuntu 24.04 and newer;
   - installs them to `/opt/robotino` and checks that `librec_robotino_api2.so` resolves all its libraries;
   - initializes rosdep if needed and installs the ROS dependencies of all packages in this repository.

   Run `install_dependencies.sh --help` for options (`--api2-only`, `--rosdistro <distro>`).
   The [dev container](.devcontainer/Dockerfile) installs Robotino API2 with the same script.
4. Build and source the workspace:
   ```bash
   cd ~/ws
   colcon build --symlink-install
   source install/setup.bash
   ```
   Add `source ~/ws/install/setup.bash` to `~/.bashrc` to make it permanent.

## Usage
- To bring up the robot, use `ros2 launch rto_bringup rto_bringup_launch.py hostname:=<robotino ip address> robot_model:=robotino3`
  - `camera:=false` disables the camera node
  - `camera_frame_id:=<frame>` overrides the frame of camera images. By default it is `camera_optical_frame`
    (Logitech C920) for Robotino 3 and `camera_color_optical_frame` (RGB module of RealSense D435, published
    by `realsense2_camera`) for Robotino 4
  - `camera_info_url:=file:///path/to/calibration.yaml` sets the camera calibration file
- To launch RViz visualization, use `ros2 launch rto_bringup rto_rviz_launch.py`
- To view the robot model only, use `ros2 launch robotino_description display_urdf.launch.py robot_model:=robotino4`
- To drive the robot with a gamepad or the keyboard, see [Gamepad teleop](#gamepad-teleop),
  [Keyboard teleop](#keyboard-teleop) and [Teleop safety](#teleop-safety).
- To build a map and localize in it, see [SLAM](#slam) and [Localization with AMCL](#localization-with-amcl).

### Gamepad teleop

`ros2 launch rto_bringup joy_teleop_launch.py` starts `game_controller_node` of the `joy` package,
`teleop_twist_joy` and the [safety layer](#teleop-safety). It works with the real robot and the simulation
alike (add `use_sim_time:=true` for the simulation).

| Control | Action |
| --- | --- |
| LB (hold) | Enable motion; releasing it stops the robot |
| RB (hold) | Turbo: higher speed |
| Left stick | Drive forward/backward and sideways |
| Right stick, horizontal | Rotate |
| Back + Start (hold 1 s) | Reset the emergency stop |

| Argument | Default | Description |
| --- | --- | --- |
| `joy_dev` | `0` | Gamepad device id, if several gamepads are connected |
| `joy_vel` | `""` | Topic of velocity commands, empty: `/robotino/cmd_vel_teleop` with the safety layer, `/robotino/cmd_vel` without it |
| `safety` | `true` | Start the [safety layer](#teleop-safety) |
| `laser` | `true` | The safety layer uses the laser scan; `false` for a robot without the laser |
| `use_sim_time` | `false` | `true` for the simulation |
| `config_filepath` | `rto_bringup/config/joy_teleop.yaml` | Axes, buttons and speed scales |

`game_controller_node` maps every gamepad known to SDL to the same layout of axes and buttons, so
`joy_teleop.yaml` does not depend on the gamepad model (buttons are named as on an Xbox gamepad).
`joy_node` publishes the raw layout of the gamepad driver instead, which does not match this file.
To change the mapping, check axis and button numbers with `ros2 run joy game_controller_node` and
`ros2 topic echo /joy`, then pass your own file with `config_filepath:=<path>`.

The dev container gets the host's `/dev/input`, so gamepads connected to the host are available inside
it, including ones plugged in after the container start. The container user needs read access to
`/dev/input/event*` on the host; if the gamepad is not found, check the permissions of its device.

### Keyboard teleop

`keyboard_teleop` of the `rto_teleop` package opens a small window and drives the robot while it has the
keyboard focus. Start it together with the [safety layer](#teleop-safety):
```bash
ros2 launch rto_bringup teleop_safety_launch.py   # use_sim_time:=true for the simulation
ros2 run rto_teleop keyboard_teleop               # in another terminal
```
`joy_teleop_launch.py` starts the safety layer too, so with it running only `keyboard_teleop` is needed.

| Key | Action |
| --- | --- |
| `W` / `S` | Forward / backward |
| `A` / `D` | Left / right |
| `Q` / `E` | Rotate left / right |
| Shift | Turbo |
| Space | Stop |
| Ctrl+R | Reset the emergency stop |

Held keys are combined: `W`+`Q` drives an arc, `W`+`D` drives diagonally (at the same speed as straight),
opposite keys cancel each other. The robot moves only while keys are held and stops when the window loses
focus. Keys are recognized by their position, so they work with any keyboard layout. The window shows the
command and the state of the emergency stop.

Without a display (e.g. over SSH) use the terminal version `ros2 run rto_teleop keyboard_teleop_tty`
with the same keys (lowercase, Shift + key for turbo) and Ctrl+C to quit. A terminal reports only the
last held key, so keys are not combined there: the robot moves while the terminal repeats the key and stops
0.15 s after the repeats stop (0.6 s after the first press, to wait for the autorepeat delay).

Both nodes publish to `/robotino/cmd_vel_teleop`; without the safety layer run them with
`--ros-args -r /robotino/cmd_vel_teleop:=/robotino/cmd_vel`.

| Parameter | Default | Description |
| --- | --- | --- |
| `linear_speed` | `0.2` | Linear speed [m/s] |
| `angular_speed` | `0.6` | Angular speed [rad/s] |
| `turbo_scale` | `2.0` | Speed factor with Shift |
| `rate` | `20.0` | Rate of commands while keys are held [Hz] |
| `key_timeout`, `first_key_timeout` | `0.15`, `0.6` | `keyboard_teleop_tty` only: stop if no key repeat comes within this time [s] |

### Teleop safety

`teleop_safety_launch.py` (started by `joy_teleop_launch.py` by default) protects the robot during
teleoperation with [nav2 collision_monitor](https://docs.nav2.org/rolling/tutorials/general_tutorials/using_collision_monitor/using_collision_monitor/)
and the `teleop_safety` node of `rto_teleop`:
```
teleop -> /robotino/cmd_vel_teleop -> teleop_safety -> /robotino/cmd_vel_safe
       -> collision_monitor -> /robotino/cmd_vel
```

- **Slowdown**: an obstacle closer than `slow_distance` (0.425 m) to the body in the direction of motion
  limits the speed to 0.2 m/s and 0.6 rad/s.
- **Stop**: an obstacle closer than `stop_distance` (7.5 cm) to the body in the direction of motion stops
  the robot. Commands in other directions still work, so the robot can move away from an obstacle or along
  a wall; rotation in place is never limited (the robot is round).
- Obstacles are taken from the laser scan `/laser_scan` and the IR sensors `/robotino/ir0..8`. The laser
  sees 240° ahead; behind the robot only the IR sensors see obstacles, up to 30 cm from the body.
  Without laser scans for 1 s collision_monitor stops the robot ("Robot to stop due to invalid source"
  in its log). On a robot without the laser start the safety layer with `laser:=false`: then only the IR
  sensors see obstacles, up to 30 cm from the body, so the slow zone works only near obstacles.
- **Emergency stop**: when the bumper is pressed, `teleop_safety` stops the robot and drops all commands
  until the emergency stop is reset by holding Back + Start of the gamepad for 1 s, Ctrl+R in
  `keyboard_teleop` / `keyboard_teleop_tty` or
  `ros2 service call /robotino/safety/reset std_srvs/srv/Trigger`. After a reset the robot can move away
  while the bumper is still pressed; commands pass after the first zero command, so a deflected stick
  does not jerk the robot.

The zones depend on the direction of the commanded velocity: `teleop_safety_launch.py` generates
`velocity_polygon`s of collision_monitor for 24 directions. RViz (`rto_config.rviz`, `sim.rviz`) shows the
current zones (group **Safety zones**).

| Name | Type | Description |
| --- | --- | --- |
| `/robotino/cmd_vel_teleop` | `geometry_msgs/Twist` | Commands of the teleop |
| `/robotino/safety/emergency_stop` | `std_msgs/Bool` | Emergency stop is active, transient local |
| `/robotino/safety/reset` | `std_srvs/Trigger` (service) | Reset the emergency stop |
| `/collision_monitor_state` | `nav2_msgs/CollisionMonitorState` | Current action of collision_monitor |
| `/collision_monitor/stop_zone`, `/collision_monitor/slow_zone` | `geometry_msgs/PolygonStamped` | Current zones |

| Argument | Default | Description |
| --- | --- | --- |
| `params_file` | [rto_bringup/config/teleop_safety.yaml](rto_bringup/config/teleop_safety.yaml) | collision_monitor and teleop_safety parameters: speed limits, sensors, reset buttons |
| `laser` | `true` | Use the laser scan; `false` for a robot without the laser |
| `stop_distance` | `0.075` | Stop zone around the body [m] |
| `slow_distance` | `0.425` | Slow zone around the body [m] |
| `use_sim_time` | `false` | `true` for the simulation |

Limitations:
- The edges of the laser field of view hit the robot itself; these points are masked by an exclusion
  zone of collision_monitor, which Jazzy does not support: on Jazzy they may block motion in some directions.
- An IR sensor reports a range beyond its 30 cm span when it sees nothing. IR sources have no timeout,
  so a silent IR sensor is not detected.

### Battery, gyroscope and shutdown

| Name | Type | Description |
| --- | --- | --- |
| `/robotino/battery_state` | `sensor_msgs/BatteryState` | Battery voltage and current consumed by the robot (negative). `power_supply_technology` is derived from the battery type reported by Robotino: `LION` for the Festool batteries of Robotino 4, `NIMH`, or `UNKNOWN` for lead-acid ones. With Festool batteries `percentage` is the mean charge of the packs and `power_supply_status` is `CHARGING` while any pack is charging, `NOT_CHARGING` with the external power supply plugged in and `DISCHARGING` otherwise. Without them `percentage` is not reported and the status is `CHARGING` while the external power supply is plugged in |
| `/robotino/batteries/accu0` … `accu7` | `sensor_msgs/BatteryState` | Festool battery packs of Robotino 4 by slot, the topic appears once a pack is found in the slot: voltage, `percentage`, `present`. `power_supply_status` is `CHARGING` while the pack is charging, `DISCHARGING` while it powers the robot and `NOT_CHARGING` otherwise: the charger electronics switch the robot between the packs |
| `/robotino/battery_low` | `std_msgs/Bool` | Low battery warning: Robotino shuts down soon. Published on change with transient local durability |
| `/robotino/imu` | `sensor_msgs/Imu` | Yaw and yaw rate of the gyroscope in `base_link`. Linear acceleration is not provided |
| `/robotino/joint_states` | `sensor_msgs/JointState` | Wheel positions, velocities and torques. Torque in N·m is estimated from the motor current, without gearbox losses |
| `/robotino/shutdown` | `std_srvs/Trigger` (service) | Shuts down the Robotino computer |

To shut down the robot:
```bash
ros2 service call /robotino/shutdown std_srvs/srv/Trigger
```

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

### Gripper and status LEDs

`gripper_launch.py` (started by `joy_teleop_launch.py` by default) starts two nodes of the `rto_io` package
that work with the gripper and the status LEDs connected to the [I/O interface](#io-interface):
```bash
ros2 launch rto_bringup gripper_launch.py
```

- `gripper` opens and closes the gripper by two outputs and reports the states of its sensors. It checks
  interlocks: the gripper opens only on contact of a roller lever switch and when it is not open yet; it closes
  only when it is open, on contact and with an object between the fingers. A refused command returns
  `success: false` with the reason.
- `status_leds`: green in normal operation; red when collision_monitor slows down or stops the robot;
  blue when the gripper is ready to grip (contact and an object between the fingers). During an emergency
  stop only the red LED is on. The LEDs are switched off when the node stops.

| Name | Type | Description |
| --- | --- | --- |
| `/robotino/gripper/open`, `/robotino/gripper/close` | `std_srvs/Trigger` (service) | Open / close the gripper |
| `/robotino/gripper/is_open` | `std_msgs/Bool` | The gripper is open, transient local |
| `/robotino/gripper/object_detected` | `std_msgs/Bool` | An object is between the fingers, transient local |
| `/robotino/gripper/contact` | `std_msgs/Bool` | A roller lever switch is pressed, transient local |

The wiring is set in [rto_bringup/config/gripper.yaml](rto_bringup/config/gripper.yaml); the defaults are the
wiring of the Robotino View program `gamepad_control_gripper_safety.rvwx`:

| Parameter | Default | Description |
| --- | --- | --- |
| `open_output`, `close_output` | `DQ1`, `DQ2` | Outputs opening / closing the gripper (`DQ1`..`DQ8`, `R1`, `R2`) |
| `outputs_active_low` | `false` | The outputs are active at 0 |
| `pulse_duration` | `0.1` | Pulse on the output [s]; `0.0`: keep it on until the next command |
| `open_sensor` | `DI1` | The gripper is open, `active_low: false` |
| `object_sensor` | `DI2` | An object is between the fingers, `active_low: true` |
| `contact_sensors` | `[DI3, DI4]` | Roller lever switches, contact if any is pressed, `active_low: true` |
| `require_contact`, `require_object_to_close` | `true`, `true` | Interlocks |
| `red_led`, `green_led`, `blue_led` | `DQ4`, `DQ5`, `DQ6` | Outputs of the LEDs (`status_leds`) |
| `leds_active_low` | `false` | The LEDs are on at 0 |

An empty channel (`""`, `[""]` for `contact_sensors`) means that the signal is not connected; the interlocks
using a missing sensor are skipped. For example, a gripper on the relays without the open sensor:
```bash
ros2 launch rto_bringup joy_teleop_launch.py gripper_params_file:=/path/to/my_gripper.yaml
```
with `open_output: "R1"`, `close_output: "R2"`, `open_sensor: ""` in the copy of `gripper.yaml`.

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
- Drive the robot: `ros2 launch rto_bringup joy_teleop_launch.py use_sim_time:=true` or
  [keyboard teleop](#keyboard-teleop) with `teleop_safety_launch.py use_sim_time:=true`

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

## SLAM

2D SLAM is done by [slam_toolbox](https://github.com/SteveMacenski/slam_toolbox) (Karto scan matching with
pose graph optimization and loop closure) from the Hokuyo laser range finder `/laser_scan` and the wheel
odometry TF `odom` → `base_link`. slam_toolbox publishes the map `/map` and the TF `map` → `odom`,
which corrects the odometry drift. The same launch file works with the real robot and the simulation.

- Real robot: start `rto_bringup_launch.py`, then
  `ros2 launch rto_bringup slam_launch.py`
- Simulation: start `ros2 launch robotino_gazebo gazebo_sim.launch.py world:=robotino_test rviz:=false`, then
  `ros2 launch rto_bringup slam_launch.py use_sim_time:=true`

Drive the robot slowly around the area (e.g. with [gamepad teleop](#gamepad-teleop)) and return to visited
places to close loops. RViz shows the map in the `map` frame, the pose graph (display **Pose graph**) and
the **SlamToolboxPlugin** panel. In the panel:
- **Save Map** writes an occupancy grid `<name>.pgm` + `<name>.yaml` for `nav2_map_server`;
- **Serialize Map** writes the pose graph `<name>.posegraph` + `<name>.data`, which slam_toolbox can load
  to continue mapping or to localize.

File names without a directory are saved to the working directory of slam_toolbox, so give a full path
(e.g. `/home/ros/maps/lab`). The same from the command line:
```bash
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "{name: {data: /home/ros/maps/lab}}"
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph "{filename: /home/ros/maps/lab}"
```

To localize in a serialized map without changing it, run
`ros2 launch rto_bringup slam_launch.py mode:=localization map:=/home/ros/maps/lab`.
The robot is expected to start at the origin of the map; otherwise set its pose with
**2D Pose Estimate** in RViz.

| Argument | Default | Description |
| --- | --- | --- |
| `mode` | `mapping` | `mapping` builds a map, `localization` localizes in `map` without changing it |
| `map` | `""` | Serialized pose graph without extension: `mapping` continues it, `localization` requires it |
| `use_sim_time` | `false` | `true` for the simulation |
| `slam_params_file` | `""` | slam_toolbox parameters, empty: [rto_bringup/config/slam_toolbox_\<mode\>.yaml](rto_bringup/config) |
| `rviz` | `true` | Start RViz |
| `rviz_config_file` | `rto_bringup/rviz/slam.rviz` | RViz config |

The parameters are tuned for the Hokuyo URG-04LX-UG01 (240°, guaranteed range 4 m): the map is built from
ranges up to 4 m, and a scan is added to the pose graph every 0.2 m or 0.2 rad of travel. Long featureless corridors
and open spaces wider than the laser range are hard for scan matching; there the map relies on the odometry.

## Localization with AMCL

[AMCL](https://docs.nav2.org/configuration/packages/configuring-amcl.html) of Nav2 localizes the robot in an
occupancy grid map saved by **Save Map** (see [SLAM](#slam)) with a particle filter. Each particle is a
hypothesis of the robot pose, moved by the odometry and weighted by how well the laser scan matches the map.
Unlike the localization mode of slam_toolbox, AMCL keeps many hypotheses, so it recovers from a wrong initial
pose and the map can be edited in an image editor, but the pose is less precise.
`map_server` publishes the map on `/map`, `amcl` publishes `/amcl_pose` and the TF `map` → `odom`.

- Real robot: start `rto_bringup_launch.py`, then
  `ros2 launch rto_bringup amcl_launch.py map:=/home/ros/maps/lab.yaml`
- Simulation: start `ros2 launch robotino_gazebo gazebo_sim.launch.py world:=robotino_test rviz:=false`, then
  `ros2 launch rto_bringup amcl_launch.py map:=/home/ros/maps/lab.yaml use_sim_time:=true`

Do not run it together with `slam_launch.py`: both publish `map` → `odom`.
The robot is expected to start at the origin of the map; otherwise set its pose with **2D Pose Estimate**
in RViz and drive a bit: AMCL updates the pose only after the robot moves 0.1 m or 0.1 rad.
RViz shows the estimated pose with its covariance (display **AMCL pose**).

| Argument | Default | Description |
| --- | --- | --- |
| `map` | `""` | Occupancy grid map `<path>.yaml`, required |
| `params_file` | [rto_bringup/config/amcl.yaml](rto_bringup/config/amcl.yaml) | `map_server`, `amcl` and `lifecycle_manager` parameters |
| `use_sim_time` | `false` | `true` for the simulation |
| `rviz` | `true` | Start RViz |
| `rviz_config_file` | `rto_bringup/rviz/amcl.rviz` | RViz config |

AMCL uses the omnidirectional motion model `nav2_amcl::OmniMotionModel` and the likelihood field laser model
with ranges up to 4 m.
