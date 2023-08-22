import xmltodict
import logging
import yaml
import json

class Modifier():
    """
    A utility class for modifying SDF files using YAML-based parameter configuration.

    This class provides methods to convert SDF files to dictionary-like structures,
    change parameter values within the SDF structure, and apply parameters from YAML files.

    Usage:
        Modifier.yaml_to_sdf(config_json)

    Attributes:
        None

    Methods:
        _parse_json(config_json):
            Parse a JSON file and return the parsed content as a dictionary.

        _convert_to_dict(sdf_file_path):
            Convert an SDF file to a dictionary-like structure using xmltodict.

        _save_sdf(sdf_file_path, sdf_dict):
            Save a dictionary-like structure as an SDF file.

        _change_parameter(sdf_file_path, p_path, p_value):
            Change a parameter value in the SDF file and save the modified SDF.

        change_sdf_parameters(config_json):
            Convert parameters from YAML files and apply them to an SDF file.

    Notes:
        - The Modifier class uses xmltodict for XML to dictionary conversion.
        - This class logs errors and information during its operations.
    """
    logging.basicConfig(
        level=logging.DEBUG,
        format='%(asctime)s - %(levelname)s - %(message)s',
        datefmt='%Y-%m-%d %H:%M:%S'
    )
    _logger = logging.getLogger('Modifier')
    _logger.setLevel(logging.INFO)

    @staticmethod
    def _parse_json(config_json):
        try:
            with open(config_json, 'r') as file:
                return json.load(file)
        except ValueError as e:
            Modifier._logger.error(e)
            return 1

    @staticmethod
    def _convert_to_dict(sdf_file_path):
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
            Modifier._logger.error(e)
            return 1
    
    @staticmethod
    def _save_sdf(sdf_file_path, sdf_dict):
        """
        Save a dict structure as an SDF file.

        Args:
            sdf_file_path (str): Path to the original SDF file.
            sdf_dict (dict): The sdf_dict structure to be saved.

        Returns:
            int: 0 if successful, 1 if failed.
        """
        try:
            with open(sdf_file_path, 'w') as file:
                file.write(xmltodict.unparse(sdf_dict, pretty=True))
                return 0
        except Exception as e:
            Modifier._logger.error(e)
            return 1
    
    @classmethod
    def _change_parameter(cls, sdf_file_path, p_path, p_value):
        """
        Change a parameter value in the SDF file and save the modified SDF.

        Args:
            sdf_file_path (str): Path to the SDF file.
            p_path (list): List of keys representing the path to the parameter.
            p_value (float/int/str): New value to set for the parameter.

        Returns:
            None
        """
        sdf_dict = cls._convert_to_dict(sdf_file_path)
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
                cls._logger.error(f"{current_level}: '{p}' not found in the current dict level.")
                break

        cls._logger.info(f"Setting {p_path[-1]} to {p_value}")

        if isinstance(p_value, (str, int, float)):
            current_level[p_path[-1]] = p_value
                
        elif isinstance(p_value, list):
            cls._logger.debug(f"Element {p_path[-1]} element is a list")
            current_level[p_path[-1]] = ' '.join(str(x) for x in p_value)

        else:
            cls._logger.error("Target parameter is not a valid scalar value.")
            return

        if cls._save_sdf(sdf_file_path, sdf_dict) == 0:
            cls._logger.debug(f"{p_path[-1]} Saved in the SDF file successfully.")
        else:
            cls._logger.error("Failed to save the SDF file.")

    @classmethod
    def change_sdf_parameters(cls, config_json):
        """
    Convert parameters from YAML files and apply them to an SDF file.

    This method takes a configuration JSON object, which contains information
    about YAML files and their associated paths within an SDF structure.
    It reads YAML files, extracts parameter values, and updates the specified
    SDF paths with these values.

    Args:
        config_json (dict): A dictionary containing configuration information.
            Each key is associated with a YAML file path and an SDF structure path.

    Returns:
        None

    Notes:
        - The configuration JSON format:
          {
            "sdf_key_1": {
                "yaml": "path_to_yaml_file_1",
                "sdf": "path_to_sdf_file_1",
                "path": {
                    "parameter_key_1": ["sdf_path_to_parameter_1"],
                    "parameter_key_2": ["sdf_path_to_parameter_2", "sdf_path_to_parameter_3"]
                }
            },
            "sdf_key_2": {
                ...
            },
            ...
          }
        - This method will log any encountered exceptions as errors.

    Raises:
        Exception: If there's an issue with parsing the YAML or updating the SDF.
        """
        for key, val in cls._parse_json(config_json).items():
            with open(val["yaml"], "r") as stream:
                try:
                    yaml_parameters = yaml.safe_load(stream)[key]['ros__parameters']
                except yaml.YAMLError as exc:
                    print(exc)
            for k, v in yaml_parameters.items():
                try:
                    cls._change_parameter(val["sdf"], val["path"][k], v)
                except Exception as e:
                    cls._logger.error(e)

def main():
     Modifier.change_sdf_parameters("/workspaces/drone/ros2_ws/src/px4_offboard/launch/config.json")

if __name__ == "__main__":
    main()