import yaml

class Parser():
    def __init__(self):

        self.parameter_file_path = '/workspaces/drone/ros2_ws/src/world_config/config/params.yaml'
        self.world_file_path = '/workspaces/drone/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/worlds/empty.world'

        with open(self.parameter_file_path, "r") as stream:
            try:
                    parameters_raw = yaml.safe_load(stream)
            except yaml.YAMLError as exc:
                    print(exc)

        parameters = parameters_raw['world_config']['ros__parameters']

        windVelocityMean = f"      <windVelocityMean>{parameters['windVelocityMean']}</windVelocityMean>\n"
        windVelocityMax = f"      <windVelocityMax>{parameters['windVelocityMax']}</windVelocityMax>\n"
        windVelocityVariance = f"      <windVelocityVariance>{parameters['windVelocityVariance']}</windVelocityVariance>\n"
        windDirectionMean = f"      <windDirectionMean>{parameters['windDirectionMean_x']} {parameters['windDirectionMean_y']} {parameters['windDirectionMean_z']}</windDirectionMean>\n"
        windDirectionVariance = f"      <windDirectionVariance>{parameters['windDirectionVariance']}</windDirectionVariance>\n"
        windGustStart = f"      <windGustStart>{parameters['windGustStart']}</windGustStart>\n"
        windGustDuration = f"      <windGustDuration>{parameters['windGustDuration']}</windGustDuration>\n"
        windGustVelocityMean = f"      <windGustVelocityMean>{parameters['windGustVelocityMean']}</windGustVelocityMean>\n"
        windGustVelocityMax = f"      <windGustVelocityMax>{parameters['windGustVelocityMax']}</windGustVelocityMax>\n"
        windGustVelocityVariance = f"      <windGustVelocityVariance>{parameters['windGustVelocityVariance']}</windGustVelocityVariance>\n"
        windGustDirectionMean = f"      <windGustDirectionMean>{parameters['windGustDirectionMean_x']} {parameters['windGustDirectionMean_y']} {parameters['windGustDirectionMean_z']}</windGustDirectionMean>\n"
        windGustDirectionVariance = f"      <windGustDirectionVariance>{parameters['windGustDirectionVariance']}</windGustDirectionVariance>\n"        

        self.config = {
        '17':  windVelocityMean,
        '18':  windVelocityMax,
        '19':  windVelocityVariance,
        '20':  windDirectionMean,
        '21':  windDirectionVariance,
        '22':  windGustStart,
        '23':  windGustDuration,
        '24':  windGustVelocityMean,
        '25':  windGustVelocityMax,
        '26':  windGustVelocityVariance,
        '27':  windGustDirectionMean,
        '28':  windGustDirectionVariance,
        }

    def parse(self):
        with open(self.world_file_path, "r") as file:
            lines = file.readlines()

        for key, value in self.config.items():
            lines[int(key)] = value
            print_value = value.split("\n")[0]
            print(f"[world-parameters] Writing to line {key}: {print_value}")

        with open(self.world_file_path, "w") as file:
            file.writelines(lines)

def main():
     parser = Parser()
     parser.parse()

if __name__ == "__main__":
     main()



