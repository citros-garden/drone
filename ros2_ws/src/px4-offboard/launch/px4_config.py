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

        mc_pitchrate_p = f"\nparam set-default MC_PITCHRATE_P {parameters['MC_PITCHRATE_P']}\n"
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
        '1':   mc_pitchrate_p,
        '2':   mc_pitchrate_i,
        '3':   mc_pitchrate_d,
        '4':   mc_pitchrate_k,
        '5':   mc_pitch_p,

        '6':  mc_rollrate_p,
        '7':  mc_rollrate_i,
        '8':  mc_rollrate_d,
        '9':  mc_rollrate_k,
        '0':  mc_roll_p,
        }

    def parse(self):
        new_lines = [x for x in self.config.values()]

        with open(self.iris_file_path, "a") as file:
            file.writelines(new_lines)

def main():
     parser = Parser()
     parser.parse()

if __name__ == "__main__":
     main()



