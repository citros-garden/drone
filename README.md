# CITROS Simulation with PX4

exposing different parameters for PX4 SITL (Gazebo and ROS2 with XRCE-DDS) to CITROS

# Install

Clone the repository:

             git clone git@github.com:citros-garden/drone.git

Open the repository with VSCode and then `reopen in container`. the `devcontainer` contains all the development requirements.

# Run

Use the pre-configure VSCode tasks to `build` and `launch` the simulation.

# Dockers

build:

        docker build -t citros/drone:latest .

run:

        docker run --rm -it citros/drone:latest ros2 launch px4_offboard offboard_position_control.launch.py
