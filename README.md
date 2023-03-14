# CITROS Simulation with PX4

# Run

# first terminal:

        make px4_sitl gazebo_iris

# second terminal:

        source install/local_setup.bash
        ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888

# third terminal:

        source install/local_setup.bash
        ros2 launch px4_offboard offboard_position_control.launch.py