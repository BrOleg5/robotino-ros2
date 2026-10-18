# Runtime image for robotino-ros2: the workspace is built into the image, see "Running in Docker" in README.md.
# Build from the repository root: docker compose build (or docker build -f docker/Dockerfile .).
# ROS_DISTRO selects the ROS 2 distribution (lyrical by default, jazzy is supported too).
# Robotino API2 is only available for amd64.
ARG ROS_DISTRO=lyrical

# Only the package manifests, so that rosdep install below is rerun only when they change.
FROM busybox:1.36 AS manifests
COPY . /src
RUN cd /src && for f in $(find . -name package.xml); do \
        mkdir -p "/manifests/$(dirname "$f")" && cp "$f" "/manifests/$f"; \
    done

FROM osrf/ros:${ROS_DISTRO}-desktop

ARG ROS_DISTRO
ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=${USER_UID}

ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-o", "pipefail", "-c"]

# The base image may lag behind the ROS apt repository: upgrade it first, otherwise newly installed
# packages fail with undefined symbols of older installed libraries.
RUN apt-get update && apt-get full-upgrade -y \
    && apt-get install -y --no-install-recommends python3-colcon-common-extensions python3-rosdep \
    && rm -rf /var/lib/apt/lists/*

# Robotino API2 (C++), see the script for details.
COPY scripts/install_dependencies.sh /tmp/install_dependencies.sh
RUN /tmp/install_dependencies.sh --api2-only \
    && rm -rf /tmp/install_dependencies.sh /var/lib/apt/lists/*

# ROS dependencies of the packages.
COPY --from=manifests /manifests /tmp/manifests
RUN apt-get update \
    && { [ -f /etc/ros/rosdep/sources.list.d/20-default.list ] || rosdep init; } \
    && rosdep update --rosdistro ${ROS_DISTRO} \
    && rosdep install --from-paths /tmp/manifests --ignore-src -y --rosdistro ${ROS_DISTRO} \
    && rm -rf /tmp/manifests /root/.ros /var/lib/apt/lists/*

# Non-root user (Ubuntu 24.04+ images ship a default "ubuntu" user with UID 1000).
RUN if id -u ubuntu >/dev/null 2>&1; then userdel -r ubuntu; fi \
    && groupadd --gid ${USER_GID} ${USERNAME} \
    && useradd --uid ${USER_UID} --gid ${USER_GID} -m -s /bin/bash ${USERNAME}

USER ${USERNAME}
WORKDIR /home/${USERNAME}/ws
COPY --chown=${USER_UID}:${USER_GID} . src/robotino-ros2
RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
    && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release \
    && rm -rf build log \
    && { echo "source /opt/ros/${ROS_DISTRO}/setup.bash"; \
         echo "source /home/${USERNAME}/ws/install/setup.bash"; \
       } >> /home/${USERNAME}/.bashrc

ENV ROS_DISTRO=${ROS_DISTRO}
COPY docker/entrypoint.sh /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
