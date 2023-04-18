import yaml

class Parser():
    def __init__(self):

        self.imu_parameter_file_path = '/workspaces/drone/ros2_ws/src/sensors_config/config/params.yaml'
        self.iris_file_path = '/workspaces/drone/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/iris/iris.sdf'
        self.gps_file_path = '/workspaces/drone/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/gps/gps.sdf'

        with open(self.imu_parameter_file_path, "r") as stream:
            try:
                    parameters_raw = yaml.safe_load(stream)
            except yaml.YAMLError as exc:
                    print(exc)

        parameters = parameters_raw['sensors_config']['ros__parameters']

        gyroscopeNoiseDensity = f"      <gyroscopeNoiseDensity>{parameters['gyroscopeNoiseDensity']}</gyroscopeNoiseDensity>\n"
        gyroscopeRandomWalk = f"      <gyroscopeRandomWalk>{parameters['gyroscopeRandomWalk']}</gyroscopeRandomWalk>\n"
        gyroscopeBiasCorrelationTime = f"      <gyroscopeBiasCorrelationTime>{parameters['gyroscopeBiasCorrelationTime']}</gyroscopeBiasCorrelationTime>\n"
        gyroscopeTurnOnBiasSigma = f"      <gyroscopeTurnOnBiasSigma>{parameters['gyroscopeTurnOnBiasSigma']}</gyroscopeTurnOnBiasSigma>\n"
        accelerometerNoiseDensity = f"      <accelerometerNoiseDensity>{parameters['accelerometerNoiseDensity']}</accelerometerNoiseDensity>\n"
        accelerometerRandomWalk = f"      <accelerometerRandomWalk>{parameters['accelerometerRandomWalk']}</accelerometerRandomWalk>\n"
        accelerometerBiasCorrelationTime = f"      <accelerometerBiasCorrelationTime>{parameters['accelerometerBiasCorrelationTime']}</accelerometerBiasCorrelationTime>\n"
        accelerometerTurnOnBiasSigma = f"      <accelerometerTurnOnBiasSigma>{parameters['accelerometerTurnOnBiasSigma']}</accelerometerTurnOnBiasSigma>\n"
    
        self.imu_config = {
        '570':  gyroscopeNoiseDensity,
        '571':  gyroscopeRandomWalk,
        '572':  gyroscopeBiasCorrelationTime,
        '573':  gyroscopeTurnOnBiasSigma,
        '574':  accelerometerNoiseDensity,
        '575':  accelerometerRandomWalk,
        '576':  accelerometerBiasCorrelationTime,
        '577':  accelerometerTurnOnBiasSigma
        }

        gpsXYRandomWalk = f"          <gpsXYRandomWalk>{parameters['gpsXYRandomWalk']}</gpsXYRandomWalk>\n"
        gpsZRandomWalk = f"          <gpsZRandomWalk>{parameters['gpsZRandomWalk']}</gpsZRandomWalk>\n"
        gpsXYNoiseDensity = f"          <gpsXYNoiseDensity>{parameters['gpsXYNoiseDensity']}</gpsXYNoiseDensity>\n"
        gpsZNoiseDensity = f"          <gpsZNoiseDensity>{parameters['gpsZNoiseDensity']}</gpsZNoiseDensity>\n"
        gpsVXYNoiseDensity = f"          <gpsVXYNoiseDensity>{parameters['gpsVXYNoiseDensity']}</gpsVXYNoiseDensity>\n"
        gpsVZNoiseDensity = f"          <gpsVZNoiseDensity>{parameters['gpsVZNoiseDensity']}</gpsVZNoiseDensity>\n"

        self.gps_config = {
        '81':  gpsXYRandomWalk,
        '82':  gpsZRandomWalk,
        '83':  gpsXYNoiseDensity,
        '84':  gpsZNoiseDensity,
        '85':  gpsVXYNoiseDensity,
        '86':  gpsVZNoiseDensity
        }

    def parse(self):
        with open(self.iris_file_path, "r") as file:
            imu_lines = file.readlines()

        with open(self.gps_file_path, "r") as file:
            gps_lines = file.readlines()

        for key, value in self.imu_config.items():
            imu_lines[int(key)] = value
            print_value = value.split("\n")[0]
            print(f"[imu-parameters] Writing to line {key}: {print_value}")

        for key, value in self.gps_config.items():
            gps_lines[int(key)] = value
            print_value = value.split("\n")[0]
            print(f"[gps-parameters] Writing to line {key}: {print_value}")

        with open(self.iris_file_path, "w") as file:
            file.writelines(imu_lines)

        with open(self.gps_file_path, "w") as file:
            file.writelines(gps_lines)

def main():
     parser = Parser()
     parser.parse()

if __name__ == "__main__":
     main()



