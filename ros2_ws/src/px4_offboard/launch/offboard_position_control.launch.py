from launch import LaunchDescription, launch_description_sources
import time
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, EmitEvent, IncludeLaunchDescription, RegisterEventHandler, LogInfo
from ament_index_python.packages import get_package_share_directory
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
import launch.logging
import logging
import os
import sys

sys.path.insert(0,'/workspaces/drone/ros2_ws/src/px4_offboard/launch')

from sdf_modifier import  Modifier as SDFModifier
from px4_modifier import  Modifier as PX4Modifier

class bcolors:
    OKBLUE = '\033[94m'
    OKGREEN = '\033[92m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    WARNING = '\033[93m'
    
print('''\n\n==============================================
 ██████╗██╗████████╗██████╗  ██████╗ ███████╗
██╔════╝██║╚══██╔══╝██╔══██╗██╔═══██╗██╔════╝
██║     ██║   ██║   ██████╔╝██║   ██║███████╗
██║     ██║   ██║   ██╔══██╗██║   ██║╚════██║
╚██████╗██║   ██║   ██║  ██║╚██████╔╝███████║
 ╚═════╝╚═╝   ╚═╝   ╚═╝  ╚═╝ ╚═════╝ ╚══════╝                                        
==============================================\n\n''')

try:
	headless = False if sys.argv[4].split(":=")[1] == 'False' else True
except:
	print(f"{bcolors.WARNING}Gazebo mode not selected, running default with GUI{bcolors.ENDC}")
	headless = True

mode = 'HEADLESS=1' if headless else ''

offboard_parameters = os.path.join(
    get_package_share_directory('px4_offboard'),
    'config',
    'params.yaml'
    )

config_file = '/workspaces/drone/ros2_ws/src/px4_offboard/launch/config.json'
iris_parameters_file = '/workspaces/drone/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/10015_gazebo-classic_iris'
px4_parameters_file = '/workspaces/drone/ros2_ws/src/px4_config/config/params.yaml'

SDFModifier.change_sdf_parameters(config_file)
PX4Modifier.change_px4_parameters(iris_parameters_file, px4_parameters_file)

launch.logging.launch_config.level = logging.INFO

time.sleep(1.0)

def generate_launch_description():

    proc_px4 = ExecuteProcess(
        cmd=['bash', '-c', f'cd /workspaces/drone/PX4-Autopilot && {mode} make px4_sitl gazebo-classic_iris__windy'],
        cwd='/tmp/px4',
        output='screen',
        emulate_tty=True
    )

    node_offboard = Node(
        package='px4_offboard',
        namespace='offboard_control',
        executable='offboard_control',
        output='screen',
        name='offboard_control',
        parameters=[offboard_parameters],
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

    recorder = ExecuteProcess(
        cmd=['ros2', 'bag', 'record', '-a', '-s', 'mcap','-o', f'db/{time.strftime("%Y-%m-%d_%H:%M:%S", time.gmtime())}'],
	)

    sys_shut_down = RegisterEventHandler(OnProcessExit(
		target_action=node_offboard,
        on_exit=[
                    LogInfo(msg=(f'{bcolors.OKGREEN}The Scenario has ended!{bcolors.ENDC}')),
                    EmitEvent(event=Shutdown(
                        reason='Finished'))
		        ]		
	))

    bridge_dir = get_package_share_directory('rosbridge_server')
    node_rosbridge =  IncludeLaunchDescription(launch_description_sources.FrontendLaunchDescriptionSource(bridge_dir + '/launch/rosbridge_websocket_launch.xml'))

    ld = LaunchDescription([proc_px4, node_offboard, node_dds_agent, node_rosbridge,recorder, sys_shut_down])
    return ld