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
        ros-${ROS_DISTRO}-omni-wheel-drive-controller \
        ros-${ROS_DISTRO}-realsense2-camera-msgs \
        ros-${ROS_DISTRO}-realsense2-description \
        ros-${ROS_DISTRO}-robot-state-publisher \
        ros-${ROS_DISTRO}-ros-gz \
        ros-${ROS_DISTRO}-ros2controlcli \
        ros-${ROS_DISTRO}-teleop-twist-joy \
        ros-${ROS_DISTRO}-teleop-twist-keyboard \
        ros-${ROS_DISTRO}-xacro \
    && rm -rf /var/lib/apt/lists/*

# Robotino API2 (C++). Upstream packages exist only for Ubuntu 20.04 (focal).
# Their libraries need only Qt5 Core/Network/Xml, which newer Ubuntu releases ship
# under the *t64 package names, so the Depends field is rewritten before installing.
ARG ROBOTINO_PKG_URL=https://packages2.openrobotino.org/pool/focal/main/r
RUN apt-get update && apt-get install -y --no-install-recommends \
        libqt5core5t64 libqt5gui5t64 libqt5network5t64 libqt5xml5t64 \
    && mkdir -p /tmp/robotino/orig /tmp/robotino/fixed && cd /tmp/robotino/orig \
    && wget -q \
        ${ROBOTINO_PKG_URL}/rec-rpc/rec-rpc_1.6.1_amd64.deb \
        ${ROBOTINO_PKG_URL}/robotino-dev/robotino-dev_1.0.3_amd64.deb \
        ${ROBOTINO_PKG_URL}/robotino-api2/robotino-api2_1.1.14_amd64.deb \
    && printf '%s\n' \
        "a4fdb236707cf2211fe8ec3c86c5caa2b7e30e6ebfdbe6a198f3b216860dafd5  rec-rpc_1.6.1_amd64.deb" \
        "704b07e475fec1de694fef0d6af668591831c48c04650694bdd84609a99c8fcc  robotino-dev_1.0.3_amd64.deb" \
        "a3371fa9caddc7a26226d75a36f52975c36885ca11d3815c3b5c02af008db896  robotino-api2_1.1.14_amd64.deb" \
        | sha256sum -c - \
    && for deb in *.deb; do \
        dpkg-deb -R "${deb}" "/tmp/robotino/${deb%.deb}" \
        && sed -i -E '/^Depends:/ { s/libqt5core5a\b/libqt5core5t64/; s/libqt5(gui5|network5|xml5)\b/libqt5\1t64/g }' \
            "/tmp/robotino/${deb%.deb}/DEBIAN/control" \
        && dpkg-deb -b "/tmp/robotino/${deb%.deb}" "/tmp/robotino/fixed/${deb}"; \
    done \
    && apt-get install -y --no-install-recommends /tmp/robotino/fixed/*.deb \
    && ldconfig \
    && ! ldd /opt/robotino/lib/librec_robotino_api2.so | grep "not found" \
    && rm -rf /tmp/robotino /var/lib/apt/lists/*

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
