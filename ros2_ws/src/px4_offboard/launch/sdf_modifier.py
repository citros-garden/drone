import xmltodict
import logging
import config as cfg
import yaml

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


    logging.basicConfig(
        level=logging.DEBUG,
        format='%(asctime)s - %(levelname)s - %(message)s',
        datefmt='%Y-%m-%d %H:%M:%S'
    )
    _logger = logging.getLogger('Modifier')
    _logger.setLevel(logging.INFO)

    @classmethod
    def _convert_to_dict(cls, sdf_file_path):
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
            cls._logger.error(e)
            return 1
    
    @classmethod
    def _save_sdf(cls, sdf_file_path, sdf_dict):
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
            cls._logger.error(e)
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
    def yaml_to_sdf(cls):
        for key, val in cfg.files.items():
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
     Modifier.yaml_to_sdf()

if __name__ == "__main__":
    main()