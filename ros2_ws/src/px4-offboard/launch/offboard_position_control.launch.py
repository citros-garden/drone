from launch import LaunchDescription, launch_description_sources
import subprocess
import time
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, EmitEvent, IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler, LogInfo, TimerAction
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import FrontendLaunchDescriptionSource
from launch.event_handlers import OnExecutionComplete, OnProcessExit, OnProcessIO, OnProcessStart, OnShutdown
from launch.events import Shutdown, process
import os

def generate_launch_description():

    proc_px4 = ExecuteProcess(
        cmd=['bash', '-c', 'cd /workspaces/citros_px4/PX4-Autopilot && HEADLESS=1 make px4_sitl gazebo'],
        cwd='/tmp/px4',
        output='screen',
        emulate_tty=True
    )

    node_offboard = Node(
            package='px4_offboard',
            namespace='px4_offboard',
            executable='offboard_control',
            output='screen',
            name='control',
            emulate_tty=True
    )
    
    node_dds_agent = Node(
            package='micro_ros_agent',
            namespace='px4_offboard',
            executable='micro_ros_agent',
            name='micro_ros',
            output='screen',
            emulate_tty=True,
            arguments=['udp4', '--port', '8888']
    )

    bridge_dir = get_package_share_directory('rosbridge_server')
    node_rosbridge =  IncludeLaunchDescription(launch_description_sources.FrontendLaunchDescriptionSource(bridge_dir + '/launch/rosbridge_websocket_launch.xml'))

    ld = LaunchDescription([proc_px4, node_offboard, node_dds_agent, node_rosbridge])

    return ld
