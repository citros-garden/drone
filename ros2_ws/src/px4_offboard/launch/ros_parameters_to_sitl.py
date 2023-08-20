import xmltodict

class ParameterConvertor():
    """
    A utility class to convert SDF files, modify specified parameters, and save the changes.

    Usage:
        ParameterConvertor.change_parameter(sdf_file_path, parameter_path, parameter_value)

    Attributes:
        sdf_file_path (str): Path to the SDF file.
        p_path (list): List of keys representing the path to the parameter to be modified.
        p_value (float/int/str): New value to set for the parameter.
    """
    @staticmethod
    def _set_nested_value(json_obj, path, value):
        """
        Set a nested value in a JSON-like object.

        Args:
            json_obj (dict/list): The JSON-like object.
            path (list): List of keys representing the path to the parameter.
            value (float/int/str): The new value to set.

        Returns:
            dict/list: The modified JSON-like object.
        """
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
        """
        Convert an SDF file to a JSON-like structure using xmltodict.

        Args:
            sdf_file_path (str): Path to the SDF file.

        Returns:
            dict: The JSON-like representation of the SDF content.
        """
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
        """
        Save a JSON-like structure as an SDF file.

        Args:
            sdf_file_path (str): Path to the original SDF file.
            sdf_json (dict): The JSON-like structure to be saved.

        Returns:
            int: 0 if successful, 1 if failed.
        """
        try:
            with open(sdf_file_path.replace(".sdf", "_modified.sdf"), 'w') as file:
                file.write(xmltodict.unparse(sdf_json, pretty=True))
                return 0
        except Exception as e:
            print(e)
            return 1
        
    @classmethod
    def change_parameter(cls, sdf_file_path, p_path, p_value):
        """
        Change a parameter value in the SDF file and save the modified SDF.

        Args:
            sdf_file_path (str): Path to the SDF file.
            p_path (list): List of keys representing the path to the parameter.
            p_value (float/int/str): New value to set for the parameter.

        Returns:
            None
        """
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