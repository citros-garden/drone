#!/bin/bash
WORKSPACE_DIR="/workspaces/citros_px4/"

cd $WORKSPACE_DIR

# Adding PX4 as submodule
git submodule add https://github.com/PX4/PX4-Autopilot.git
git submodule update --init --recursive

# Installing all PX4 deps
PX4-Autopilot/Tools/setup/ubuntu.sh --no-nuttx

# Installing Gazebo
sudo apt-get install -y gazebo
pip install setuptools --upgrade