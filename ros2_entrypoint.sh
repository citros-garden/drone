#!/bin/bash
set -e

source /opt/ros/$ROS_DISTRO/setup.bash
source /workspaces/drone/ros2_ws/install/local_setup.bash

exec "$@"