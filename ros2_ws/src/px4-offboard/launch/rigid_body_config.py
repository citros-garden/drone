import yaml

class Parser():
    def __init__(self):

        self.parameter_file_path = '/workspaces/citros_px4/ros2_ws/src/rigid_body_config/config/params.yaml'
        self.iris_file_path = '/workspaces/citros_px4/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/iris/iris.sdf'

        with open(self.parameter_file_path, "r") as stream:
            try:
                    parameters_raw = yaml.safe_load(stream)
            except yaml.YAMLError as exc:
                    print(exc)

        parameters = parameters_raw['rigid_body_config']['ros__parameters']

        m   = f"        <mass>{parameters['m']}</mass>\n"
        ixx = f"          <ixx>{parameters['i_xx']}</ixx>\n"
        ixy = f"          <ixy>{parameters['i_xy']}</ixy>\n"
        ixz = f"          <ixz>{parameters['i_xz']}</ixz>\n"
        iyy = f"          <iyy>{parameters['i_yy']}</iyy>\n"
        iyz = f"          <iyz>{parameters['i_yz']}</iyz>\n"
        izz = f"          <izz>{parameters['i_zz']}</izz>\n"

        self.config = {
        '7':   m,
        '9':   ixx,
        '10':  ixy,
        '11':  ixz,
        '12':  iyy,
        '13':  iyz,
        '14':  izz
        }

    def parse(self):
        with open(self.iris_file_path, "r") as file:
            lines = file.readlines()

        for key, value in self.config.items():
            lines[int(key)] = value
            print_value = value.split("\n")[0]
            print(f"[rigid-body-parameters] Writing to line {key}: {print_value}")

        with open(self.iris_file_path, "w") as file:
            file.writelines(lines)

def main():
     parser = Parser()
     parser.parse()

if __name__ == "__main__":
     main()



