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
            cmd=[
                    '/workspaces/citros_px4/PX4-Autopilot/build/px4_sitl_default/bin/px4',
                    '/workspaces/citros_px4/PX4-Autopilot/ROMFS/px4fmu_common/',
                    '-s',
                    '/workspaces/citros_px4/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/rcS'
            ],
            cwd='/tmp/px4',
            output='screen')

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

    ld = LaunchDescription([node_offboard, node_dds_agent, node_rosbridge])

    return ld
