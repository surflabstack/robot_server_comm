#!/bin/bash
# Loads the ROS 2 environment and the workspace, then runs the given command.
set -e
source "/opt/ros/${ROS_DISTRO}/setup.bash"
source "/ws/install/setup.bash"
exec "$@"
