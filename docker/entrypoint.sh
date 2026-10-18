#!/bin/bash
# Sources ROS 2 and the workspace, then runs the command.
set -e
source "/opt/ros/${ROS_DISTRO}/setup.bash"
source "${HOME}/ws/install/setup.bash"
exec "$@"
