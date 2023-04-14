import yaml

parameter_file_path = '/workspaces/citros_px4/ros2_ws/src/rigid_body_config/config/params.yaml'
iris_file_path = '/workspaces/citros_px4/ros2_ws/src/rigid_body_config/rigid_body_config/iris.sdf'
with open(parameter_file_path, "r") as stream:
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

CONFIG = {
    '7':   m,
    '9':   ixx,
    '10':  ixy,
    '11':  ixz,
    '12':  iyy,
    '13':  iyz,
    '14':  izz
}

with open(iris_file_path, "r") as file:
    lines = file.readlines()

for key, value in CONFIG.items():
    print(f"writing {value} to line {key}")
    lines[int(key)] = value

with open(iris_file_path, "w") as file:
    file.writelines(lines)



