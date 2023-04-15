#!/bin/bash
WORKSPACE_DIR="/workspaces/drone/"

cd $WORKSPACE_DIR

# Adding PX4 as submodule
git submodule add https://github.com/PX4/PX4-Autopilot.git
git submodule update --init --recursive
cd PX4-Autopilot
git checkout release/1.14
cd ..
git submodule update --init --recursive

cd PX4-Autopilot

# Installing all PX4 deps
Tools/setup/ubuntu.sh --no-nuttx
make px4_sitl gazebo
# Installing Gazebo
sudo apt-get install -y gazebo
pip install setuptools==58.2.0

cd ..
cd ros2_ws
colcon build
cd ..
mkdir /tmp/px4

echo "Done installing, ready to develop!"