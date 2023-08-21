from sdf_modifier import Modifier
import config as cfg
import yaml

class SetParameters():
        
    @classmethod
    def yaml_to_sdf(cls):
        mod = Modifier()
        for key, val in cfg.files.items():
            with open(val["yaml"], "r") as stream:
                try:
                    yaml_parameters = yaml.safe_load(stream)[key]['ros__parameters']
                except yaml.YAMLError as exc:
                    print(exc)
            for k, v in yaml_parameters.items():
                try:
                    mod.change_parameter(val["sdf"], val["path"][k], v)
                except Exception as e:
                    mod.logger.error(e)

def main():
     SetParameters.yaml_to_sdf()

if __name__ == "__main__":
    main()