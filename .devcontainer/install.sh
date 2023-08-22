#!/bin/bash
WORKSPACE_DIR="/workspaces/drone/"

cd $WORKSPACE_DIR

# Adding PX4
git clone -b release/1.14 https://github.com/PX4/PX4-Autopilot.git
cd PX4-Autopilot
git submodule update --init --recursive

# Installing Gazebo
sudo apt-get install -y gazebo

# Installing all PX4 deps and build SITL
Tools/setup/ubuntu.sh --no-nuttx
make px4_sitl gazebo

pip install setuptools==58.2.0

# compile ROS workspace
cd ..
cd ros2_ws
colcon build
cd ..
sudo mkdir /tmp/px4

echo "
# ==============================================
#   ██████╗██╗████████╗██████╗  ██████╗ ███████╗
#  ██╔════╝██║╚══██╔══╝██╔══██╗██╔═══██╗██╔════╝
#  ██║     ██║   ██║   ██████╔╝██║   ██║███████╗
#  ██║     ██║   ██║   ██╔══██╗██║   ██║╚════██║
#  ╚██████╗██║   ██║   ██║  ██║╚██████╔╝███████║
#   ╚═════╝╚═╝   ╚═╝   ╚═╝  ╚═╝ ╚═════╝ ╚══════╝                                        
# =============================================="

echo "Done installing, ready to develop!"