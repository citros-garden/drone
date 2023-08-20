import xmltodict
import json

class ParameterConvertor():

    def __init__(self, sdf_file_path):
        with open(sdf_file_path, "rt") as file:
            buffer = file.read()
            self.sdf_json = xmltodict.parse(buffer)
            print(self.sdf_json)

        with open("output.sdf", 'w') as file:
            file.write(xmltodict.unparse(self.sdf_json, pretty=True))

if __name__ == "__main__":
    sdf_file_path = "/workspaces/drone/ros2_ws/src/rigid_body_config/rigid_body_config/iris_modified.sdf"
    pc = ParameterConvertor(sdf_file_path)

    