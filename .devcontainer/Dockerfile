# Development image for robotino-ros2.
# ROS_DISTRO selects the ROS 2 distribution (lyrical by default, jazzy is supported too).
# Robotino API2 is only available for amd64 (the platform is set in devcontainer.json).
ARG ROS_DISTRO=lyrical
FROM osrf/ros:${ROS_DISTRO}-desktop

ARG ROS_DISTRO
ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=${USER_UID}

ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-o", "pipefail", "-c"]

# Development tools and ROS packages used by this repository.
# The base image may lag behind the ROS apt repository: upgrade it first, otherwise newly installed
# packages (e.g. ros_gz, gz_ros2_control) fail with undefined symbols of older installed libraries.
RUN apt-get update && apt-get full-upgrade -y \
    && apt-get install -y --no-install-recommends \
        build-essential \
        clang-format \
        cmake \
        gdb \
        git \
        mesa-utils \
        python3-colcon-common-extensions \
        python3-rosdep \
        ros-dev-tools \
        sudo \
        wget \
        ros-${ROS_DISTRO}-controller-manager \
        ros-${ROS_DISTRO}-gz-ros2-control \
        ros-${ROS_DISTRO}-image-transport-plugins \
        ros-${ROS_DISTRO}-joint-state-broadcaster \
        ros-${ROS_DISTRO}-joint-state-publisher \
        ros-${ROS_DISTRO}-joint-state-publisher-gui \
        ros-${ROS_DISTRO}-joy \
        ros-${ROS_DISTRO}-nav2-amcl \
        ros-${ROS_DISTRO}-nav2-collision-monitor \
        ros-${ROS_DISTRO}-nav2-lifecycle-manager \
        ros-${ROS_DISTRO}-nav2-map-server \
        ros-${ROS_DISTRO}-omni-wheel-drive-controller \
        ros-${ROS_DISTRO}-realsense2-camera-msgs \
        ros-${ROS_DISTRO}-realsense2-description \
        ros-${ROS_DISTRO}-robot-state-publisher \
        ros-${ROS_DISTRO}-ros-gz \
        ros-${ROS_DISTRO}-ros2controlcli \
        ros-${ROS_DISTRO}-slam-toolbox \
        ros-${ROS_DISTRO}-teleop-twist-joy \
        ros-${ROS_DISTRO}-xacro \
    && rm -rf /var/lib/apt/lists/*

# Robotino API2 (C++), see the script for details.
COPY scripts/install_dependencies.sh /tmp/install_dependencies.sh
RUN /tmp/install_dependencies.sh --api2-only \
    && rm -rf /tmp/install_dependencies.sh /var/lib/apt/lists/*

# Non-root user matching the host UID/GID (Ubuntu 24.04+ images ship a default "ubuntu" user with UID 1000).
RUN if id -u ubuntu >/dev/null 2>&1; then userdel -r ubuntu; fi \
    && groupadd --gid ${USER_GID} ${USERNAME} \
    && useradd --uid ${USER_UID} --gid ${USER_GID} -m -s /bin/bash ${USERNAME} \
    && echo "${USERNAME} ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME} \
    && chmod 0440 /etc/sudoers.d/${USERNAME}

USER ${USERNAME}
ENV ROS_DISTRO=${ROS_DISTRO}
RUN mkdir -p /home/${USERNAME}/ws/src /home/${USERNAME}/ws/build /home/${USERNAME}/ws/install /home/${USERNAME}/ws/log \
    && rosdep update --rosdistro ${ROS_DISTRO} \
    && { echo "source /opt/ros/${ROS_DISTRO}/setup.bash"; \
         echo "[ -f ~/ws/install/setup.bash ] && source ~/ws/install/setup.bash"; \
         echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash 2>/dev/null || true"; \
       } >> /home/${USERNAME}/.bashrc

WORKDIR /home/${USERNAME}/ws
