#!/bin/bash
set -e

source /opt/ros/$ROS_DISTRO/setup.bash
source /workspaces/citros_px4/ros2_ws/install/local_setup.bash

exec "$@"