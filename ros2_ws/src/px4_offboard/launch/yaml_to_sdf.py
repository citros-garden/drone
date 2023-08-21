from .sdf_modifier import Modifier
import sdf_parameters

DEBUG = True

class SetParameters():
    
    def __init__(self):
        self.mod = Modifier()

        yaml_parameters_rigid_body = '/workspaces/drone/ros2_ws/src/rigid_body_config/config/params.yaml'
        yaml_parameters_sensors = '/workspaces/drone/ros2_ws/src/sensors_config/config/params.yaml'
        yaml_parameters_world = '/workspaces/drone/ros2_ws/src/world_config/config/params.yaml'

        self.yaml_files = [yaml_parameters_rigid_body, yaml_parameters_sensors, yaml_parameters_world]
        
        if DEBUG:
            sdf_rigid_body = '/workspaces/drone/ros2_ws/src/rigid_body_config/rigid_body_config/iris.sdf'
            sdf_imu = '/workspaces/drone/ros2_ws/src/rigid_body_config/rigid_body_config/iris.sdf'
            sdf_gps = '/workspaces/drone/ros2_ws/src/sensors_config/sensors_config/gps.sdf'
            sdf_world = '/workspaces/drone/ros2_ws/src/world_config/world_config/empty.world'

        else:
            sdf_rigid_body = '/workspaces/drone/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/iris/iris.sdf'
            sdf_imu = '/workspaces/drone/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/iris/iris.sdf'
            sdf_gps = '/workspaces/drone/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/gps/gps.sdf'
            sdf_world = '/workspaces/drone/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/worlds/empty.world'

        self.files = {
                "rigid_body": 
                       {"yaml": yaml_parameters_rigid_body,
                        "sdf": sdf_rigid_body},

                 "imu": {"yaml": yaml_parameters_sensors,
                         "sdf": sdf_imu},

                 "gps": {"yaml": yaml_parameters_sensors,
                         "sdf": sdf_gps},

                 "world": {"yaml": yaml_parameters_world,
                           "sdf": sdf_world}
                }