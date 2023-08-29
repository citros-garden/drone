import yaml
import logging
from typing import List

class Modifier():
    """
    A class for modifying and managing parameters in PX4 configuration files using ROS-YAML input.
    """
    logging.basicConfig(
        level=logging.DEBUG,
        format='%(asctime)s - %(levelname)s - %(message)s',
        datefmt='%Y-%m-%d %H:%M:%S'
    )
    _logger = logging.getLogger('Modifier')
    _logger.setLevel(logging.INFO)

    @staticmethod
    def _set_parameters(yaml_parameters: dict) -> dict:
        """
    Generate lines for setting PX4 parameters based on the provided YAML parameters.

    Parameters:
        yaml_parameters (dict): A dictionary containing YAML parameters.

    Returns:
        dict: A dictionary of lines to set PX4 parameters in the format:
              {
                  "parameter_name": "param set-default parameter_name value",
                  ...
              }
        """
        lines_to_be_writter = {
            "MC_PITCHRATE_P": f"param set-default MC_PITCHRATE_P {yaml_parameters['MC_PITCHRATE_P']}",
            "MC_PITCHRATE_I": f"param set-default MC_PITCHRATE_I {yaml_parameters['MC_PITCHRATE_I']}",
            "MC_PITCHRATE_D": f"param set-default MC_PITCHRATE_D {yaml_parameters['MC_PITCHRATE_D']}",
            "MC_PITCHRATE_K": f"param set-default MC_PITCHRATE_K {yaml_parameters['MC_PITCHRATE_K']}",
            "MC_PITCH_P": f"param set-default MC_PITCH_P {yaml_parameters['MC_PITCH_P']}",

            "MC_ROLLRATE_P": f"param set-default MC_ROLLRATE_P {yaml_parameters['MC_ROLLRATE_P']}",
            "MC_ROLLRATE_I": f"param set-default MC_ROLLRATE_I {yaml_parameters['MC_ROLLRATE_I']}",
            "MC_ROLLRATE_D": f"param set-default MC_ROLLRATE_D {yaml_parameters['MC_ROLLRATE_D']}",
            "MC_ROLLRATE_K": f"param set-default MC_ROLLRATE_K {yaml_parameters['MC_ROLLRATE_K']}",
            "MC_ROLL_P": f"param set-default MC_ROLL_P {yaml_parameters['MC_ROLL_P']}"
        }
        return lines_to_be_writter

    @staticmethod
    def _parse_px4_yaml(yaml_file_path: str) -> dict:
        """
    Parse a PX4 YAML configuration file and extract ROS parameters.

    Parameters:
        yaml_file_path (str): Path to the PX4 YAML configuration file.

    Returns:
        dict: A dictionary containing the extracted ROS parameters from the YAML file.
              An empty dictionary is returned if there is an error during parsing.
        """
        with open(yaml_file_path, "r") as stream:
            try:
                parameters_raw = yaml.safe_load(stream)
                parameters = parameters_raw['px4_config']['ros__parameters']
                return parameters
            except yaml.YAMLError as exc:
                Modifier._logger.error(exc)
                return {}

    @staticmethod
    def _parse_px4_parameters(parameter_file: str) -> List[str]:
        """
    Parse a PX4 parameter file and return its lines as a list.

    Parameters:
        parameter_file (str): Path to the PX4 parameter file.

    Returns:
        List[str]: A list of strings representing the lines of the parameter file.
                   An empty list is returned if there is an error during parsing.
        """
        try:
            with open(parameter_file) as file:
                lines = [line.rstrip() for line in file]
                return lines
        except Exception as e:
             Modifier._logger.error(e)
             return []
            
    @staticmethod
    def _check_for_existing_parameters(lines: List[str]) -> int:
        """
    Check if any of the lines in the list contain the string "PITCH".

    Parameters:
        lines (List[str]): A list of strings representing lines from a file.

    Returns:
        bool: True if any line contains the string "PITCH", False otherwise.
        """
        return any(["PITCH" in x for x in lines])
    
    @staticmethod 
    def _get_parameters_lines(lines: List[str], lines_to_be_written: dict) -> dict:
        """
    Extract information about existing parameter lines that need to be changed.

    Parameters:
        lines (List[str]): A list of strings representing lines from a file.
        lines_to_be_written (dict): A dictionary containing lines to be written.

    Returns:
        dict: A dictionary containing information about parameters to be changed in the format:
              {
                  "parameter_name": {
                      "value": "parameter_value",
                      "line": line_number
                  },
                  ...
              }
        """
        parameters = {}
        parameters_to_be_changed = lines_to_be_written.keys()
        for counter, line in enumerate(lines):
            try:
                parameter = line.split("param set-default ")[1].split(" ")[0]
                if parameter in parameters_to_be_changed:
                    value = line.split("param set-default ")[1].split(" ")[1]
                    parameters[parameter] = {"value": value, "line": counter}
                    Modifier._logger.info(f"Parameter {parameter} will be set to {value} in line {counter + 1}")
            except Exception:
                pass
        return parameters

    @classmethod
    def change_px4_parameters(cls, parameter_file: str, yaml_file: str) -> None:
        """
    Modify PX4 parameters in a parameter file based on YAML input.

    Parameters:
        parameter_file (str): Path to the PX4 parameter file to be modified.
        yaml_file (str): Path to the YAML input file containing parameters.

    Returns:
        None
        """
        lines = cls._parse_px4_parameters(parameter_file)

        yaml_parameters = cls._parse_px4_yaml(yaml_file)
        lines_to_be_written = cls._set_parameters(yaml_parameters)

        if cls._check_for_existing_parameters(lines):
            # Parameters already exist in the file, change the value
            parameters_lines = cls._get_parameters_lines(lines, lines_to_be_written)
            for k,v in parameters_lines.items():
                lines[v["line"]] = lines_to_be_written[k]
            
            with open(parameter_file, "w") as file:
                for counter, line in enumerate(lines):
                    if counter == 0:
                        file.write(line)
                    else:
                        file.write("\n" + line)

        else:
            # Parameters not exist, append to the tail of the file
            with open(parameter_file, "a") as file:
                for k, v in lines_to_be_written.items():
                    file.write('\n' + v)
         
def main():
     parameter_file = '/workspaces/drone/ros2_ws/src/px4_offboard/launch/10015_gazebo-classic_iris'
     yaml_file = '/workspaces/drone/ros2_ws/src/px4_offboard/config/params.yaml'
     Modifier.change_px4_parameters(parameter_file, yaml_file)

if __name__ == "__main__":
     main()



