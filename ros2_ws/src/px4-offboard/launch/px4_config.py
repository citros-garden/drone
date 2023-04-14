import yaml

class Parser():
    def __init__(self):

        self.parameter_file_path = '/workspaces/citros_px4/ros2_ws/src/px4_config/config/params.yaml'
        self.iris_file_path = '/workspaces/citros_px4/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/10016_gazebo-classic_iris'

        with open(self.parameter_file_path, "r") as stream:
            try:
                    parameters_raw = yaml.safe_load(stream)
            except yaml.YAMLError as exc:
                    print(exc)

        parameters = parameters_raw['px4_config']['ros__parameters']
        print(parameters)

        mc_pitchrate_p = f"param set-default MC_PITCHRATE_P {parameters['MC_PITCHRATE_P']}\n"
        mc_pitchrate_i = f"param set-default MC_PITCHRATE_I {parameters['MC_PITCHRATE_I']}\n"
        mc_pitchrate_d = f"param set-default MC_PITCHRATE_D {parameters['MC_PITCHRATE_D']}\n"
        mc_pitchrate_k = f"param set-default MC_PITCHRATE_K {parameters['MC_PITCHRATE_K']}\n"
        mc_pitch_p     = f"param set-default MC_PITCH_P {parameters['MC_PITCH_P']}\n"

        mc_rollrate_p = f"param set-default MC_ROLLRATE_P {parameters['MC_ROLLRATE_P']}\n"
        mc_rollrate_i = f"param set-default MC_ROLLRATE_I {parameters['MC_ROLLRATE_I']}\n"
        mc_rollrate_d = f"param set-default MC_ROLLRATE_D {parameters['MC_ROLLRATE_D']}\n"
        mc_rollrate_k = f"param set-default MC_ROLLRATE_K {parameters['MC_ROLLRATE_K']}\n"
        mc_roll_p     = f"param set-default MC_ROLL_P {parameters['MC_ROLL_P']}\n"

        self.config = {
        '28':   mc_pitchrate_p,
        '29':   mc_pitchrate_i,
        '30':   mc_pitchrate_d,
        '31':   mc_pitchrate_k,
        '32':   mc_pitch_p,

        '34':  mc_rollrate_p,
        '35':  mc_rollrate_i,
        '36':  mc_rollrate_d,
        '37':  mc_rollrate_k,
        '38':  mc_roll_p,
        }

    def parse(self):
        with open(self.iris_file_path, "r") as file:
            lines = file.readlines()

        for key, value in self.config.items():
            lines[int(key)] = value
            print_value = value.split("\n")[0]
            print(f"[px4-config-parameters] Writing to line {key}: {print_value}")

        with open(self.iris_file_path, "w") as file:
            file.writelines(lines)

def main():
     parser = Parser()
     parser.parse()

if __name__ == "__main__":
     main()



