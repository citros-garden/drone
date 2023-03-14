#!/bin/bash
WORKSPACE_DIR="/workspaces/citros_px4/"

cd $WORKSPACE_DIR

# Adding PX4 as submodule
git submodule add https://github.com/PX4/PX4-Autopilot.git
git submodule update --init --recursive

# Installing all PX4 deps
PX4-Autopilot/Tools/setup/ubuntu.sh --no-nuttx

# Adding microros_ws as submodule
mkdir microros_ws
cd microros_ws
mkdir src
git submodule add -b humble https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup

# Building microros_ws
colcon build
source install/local_setup.bash
ros2 run micro_ros_setup create_agent_ws.sh
ros2 run micro_ros_setup build_agent.sh
sudo rosdep init
rosdep update

# Adding a second ROS workspace for the offboard example
cd $WORKSPACE_DIR
mkdir example_ws
cd example_ws
mkdir src
git submodule add https://github.com/PX4/px4_msgs.git src/px4_msgs
git submodule add https://github.com/PX4/px4_ros_com.git src/px4_ros_com
git submodule add https://github.com/Jaeyoung-Lim/px4-offboard.git src/px4-offboard
colcon build