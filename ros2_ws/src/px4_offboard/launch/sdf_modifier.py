import xmltodict
import logging

class Modifier():
    """
    A utility class to convert SDF files, modify specified parameters, and save the changes.

    Usage:
        ParameterConvertor.change_parameter(sdf_file_path, parameter_path, parameter_value)

    Attributes:
        sdf_file_path (str): Path to the SDF file.
        p_path (list): List of keys representing the path to the parameter to be modified.
        p_value (float/int/str): New value to set for the parameter.
    """

    def __init__(self):
        logging.basicConfig(
            # filename='app.log',
            level=logging.DEBUG,
            format='%(asctime)s - %(levelname)s - %(message)s',
            datefmt='%Y-%m-%d %H:%M:%S'
        )
        self.logger = logging.getLogger('Modifier')
        self.logger.setLevel(logging.INFO)

    def _convert_to_dict(self, sdf_file_path):
        """
        Convert an SDF file to a dict-like structure using xmltodict.

        Args:
            sdf_file_path (str): Path to the SDF file.

        Returns:
            dict: The dict representation of the SDF content.
        """
        try:
            with open(sdf_file_path, "rt") as file:
                buffer = file.read()
                sdf_dict = xmltodict.parse(buffer)
            return sdf_dict
        except Exception as e:
            self.logger.error(e)
            return 1
    
    def _save_sdf(self, sdf_file_path, sdf_dict):
        """
        Save a dict structure as an SDF file.

        Args:
            sdf_file_path (str): Path to the original SDF file.
            sdf_dict (dict): The sdf_dict structure to be saved.

        Returns:
            int: 0 if successful, 1 if failed.
        """
        try:
            with open(sdf_file_path.replace(".sdf", "_modified.sdf"), 'w') as file:
                file.write(xmltodict.unparse(sdf_dict, pretty=True))
                return 0
        except Exception as e:
            self.logger.error(e)
            return 1
        
    def change_parameter(self, sdf_file_path, p_path, p_value):
        """
        Change a parameter value in the SDF file and save the modified SDF.

        Args:
            sdf_file_path (str): Path to the SDF file.
            p_path (list): List of keys representing the path to the parameter.
            p_value (float/int/str): New value to set for the parameter.

        Returns:
            None
        """
        sdf_dict = self._convert_to_dict(sdf_file_path)
        current_level = sdf_dict

        for p in p_path[:-1]:
            if isinstance(current_level, dict) and p in current_level:
                current_level = current_level[p]
            elif isinstance(current_level, list):
                for attr in current_level:
                    if p in attr["@name"]:
                        current_level = attr
                        break
            else:
                self.logger.error(f"Error: '{p}' not found in the current dict level.")
                self.logger.error(f"{current_level}")
                break

        self.logger.info(f"Setting {p_path[-1]} to {p_value}")

        if isinstance(p_value, (str, int, float)):
            current_level[p_path[-1]] = p_value
                
        elif isinstance(p_value, list):
            self.logger.debug(f"Element {p_path[-1]} element is a list")
            current_level[p_path[-1]] = ' '.join(str(x) for x in p_value)

        else:
            self.logger.error("Target parameter is not a valid scalar value.")
            return

        if self._save_sdf(sdf_file_path, sdf_dict) == 0:
            self.logger.debug(f"{p_path[-1]} Saved in the SDF file successfully.")
        else:
            self.logger.error("Failed to save the SDF file.")

if __name__ == "__main__":
    sdf_file_path = "/workspaces/drone/ros2_ws/src/rigid_body_config/rigid_body_config/iris_modified.sdf"
    parameter_path = ["sdf", "model", "link", "base_link", "inertial", "inertia", "ixx"]
    parameter_value = 5.0
    mod = Modifier()
    mod.change_parameter(sdf_file_path, parameter_path, parameter_value)