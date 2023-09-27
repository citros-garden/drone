import sys
sys.path.insert(0,'/workspaces/drone/ros2_ws/src/px4_offboard/launch')

from px4_modifier import  Modifier as PX4Modifier

def main():
    PX4Modifier.change_dds_qos_profile()
    PX4Modifier.replace_dds_topics_yaml()
    PX4Modifier.create_px4_folder()
    print("Change DDS settings!")

if __name__ == "__main__":
    main()