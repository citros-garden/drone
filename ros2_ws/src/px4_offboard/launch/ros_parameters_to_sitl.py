import xmltodict

class ParameterConvertor():
    @staticmethod
    def _set_nested_value(json_obj, path, value):
        current_level = json_obj
        for p in path[:-1]:
            if isinstance(current_level, dict) and p in current_level:
                current_level = current_level[p]
            elif isinstance(current_level, list) and isinstance(p, int) and p < len(current_level):
                current_level = current_level[p]
        current_level[path[-1]] = value
        return json_obj

    @staticmethod
    def _convert_to_json(sdf_file_path):
        try:
            with open(sdf_file_path, "rt") as file:
                buffer = file.read()
                sdf_json = xmltodict.parse(buffer)
            return sdf_json
        except Exception as e:
            print(e)
            return 1
    
    @staticmethod
    def _save_sdf(sdf_file_path, sdf_json):
        try:
            with open(sdf_file_path.replace(".sdf", "_modified.sdf"), 'w') as file:
                file.write(xmltodict.unparse(sdf_json, pretty=True))
                return 0
        except Exception as e:
            print(e)
            return 1
        
    @classmethod
    def change_parameter(cls, sdf_file_path, p_path, p_value):
        sdf_json = cls._convert_to_json(sdf_file_path)
        current_level = sdf_json
        for p in p_path:
            if isinstance(current_level, dict) and p in current_level:
                current_level = current_level[p]
            elif isinstance(current_level, list) and isinstance(p, int) and p < len(current_level):
                current_level = current_level[p]
            else:
                print(f"Error: '{p}' not found in the current JSON level.")
                break

        if isinstance(current_level, (str, int, float)):
            sdf_json = cls._set_nested_value(sdf_json, p_path, p_value)

            if cls._save_sdf(sdf_file_path, sdf_json) == 0:
                print("SDF file modified and saved successfully.")
            else:
                print("Failed to save modified SDF file.")
        else:
            print("Target parameter is not a valid scalar value.")

if __name__ == "__main__":
    sdf_file_path = "/workspaces/drone/ros2_ws/src/rigid_body_config/rigid_body_config/iris_modified.sdf"
    parameter_path = ["sdf", "model", "link", 0, "inertial", "inertia", "ixx"]
    parameter_value = 10.0
    ParameterConvertor.change_parameter(sdf_file_path, parameter_path, parameter_value)

    