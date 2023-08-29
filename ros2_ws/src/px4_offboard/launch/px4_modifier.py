import yaml
import json
import logging
from typing import List

class Modifier():
    logging.basicConfig(
        level=logging.DEBUG,
        format='%(asctime)s - %(levelname)s - %(message)s',
        datefmt='%Y-%m-%d %H:%M:%S'
    )
    _logger = logging.getLogger('Modifier')
    _logger.setLevel(logging.INFO)

    @staticmethod
    def _parse_px4_yaml(yaml_file_path: str) -> dict:
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
        try:
            with open(parameter_file) as file:
                lines = [line.rstrip() for line in file]
                return lines
        except Exception as e:
             Modifier._logger.error(e)
             return []
            
    @staticmethod
    def _check_for_existing_parameters(lines: List[str]) -> int:
         return any(["PITCH" in x for x in lines])
    
    @staticmethod 
    def _get_parameters_lines(lines: List[str]) -> List[int]:
        stripped_lines = {}
        parameters_to_be_changed = ["MC_PITCHRATE_P", 
                          "MC_PITCHRATE_I", 
                          "MC_PITCHRATE_D", 
                          "MC_PITCHRATE_K", 
                          "MC_ROLLRATE_P", 
                          "MC_ROLLRATE_I", 
                          "MC_ROLLRATE_D", 
                          "MC_ROLLRATE_K",
                          "MC_PITCH_P", 
                          "MC_ROLL_P"]
        for counter, line in enumerate(lines):
            try:
                parameter = line.split("param set-default ")[1].split(" ")[0]
                if parameter in parameters_to_be_changed:
                    value = line.split("param set-default ")[1].split(" ")[1]
                    stripped_lines[parameter] = {"value": value, "line": counter+1}
            except Exception:
                pass
        print(stripped_lines)

    @classmethod
    def change_px4_parameters(cls, parameter_file: str) -> None:
        lines = cls._parse_px4_parameters(parameter_file)
        if cls._check_for_existing_parameters(lines):
            # print(lines)
            parameters_lines = cls._get_parameters_lines(lines)

        # else:
        #     with open(parameter_file, "a") as file:
        #         for line in lines:
        #             file.write('\n' + line)
         
def main():
     parameter_file = '/workspaces/drone/ros2_ws/src/px4_offboard/launch/10015_gazebo-classic_iris'
     yaml_file = '/workspaces/drone/ros2_ws/src/px4_offboard/config/params.yaml'
     Modifier.change_px4_parameters(parameter_file)

if __name__ == "__main__":
     main()



