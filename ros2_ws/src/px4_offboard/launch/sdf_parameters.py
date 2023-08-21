RIGID_BODY = {
        "ixx": ["sdf", "model", "link", "base_link", "inertial", "inertia", "ixx"],
        "ixy": ["sdf", "model", "link", "base_link", "inertial", "inertia", "ixy"],
        "ixz": ["sdf", "model", "link", "base_link", "inertial", "inertia", "ixz"],
        "iyy": ["sdf", "model", "link", "base_link", "inertial", "inertia", "iyy"],
        "iyz": ["sdf", "model", "link", "base_link", "inertial", "inertia", "iyz"],
        "izz": ["sdf", "model", "link", "base_link", "inertial", "inertia", "izz"],
        "mass":  ["sdf", "model", "link", "base_link", "inertial", "inertia", "mass"]
}

WORLD = {
        "windVelocityMean": ["sdf", "world", "plugin", "wind_plugin", "windVelocityMean"],
        "windVelocityMax": ["sdf", "world", "plugin", "wind_plugin", "windVelocityMax"],
        "windVelocityVariance": ["sdf", "world", "plugin", "wind_plugin", "windVelocityVariance"],
        "windDirectionMean": ["sdf", "world", "plugin", "wind_plugin", "windDirectionMean"],
        "windDirectionVariance": ["sdf", "world", "plugin", "wind_plugin", "windDirectionVariance"],
        "windGustStart": ["sdf", "world", "plugin", "wind_plugin", "windGustStart"],
        "windGustDuration": ["sdf", "world", "plugin", "wind_plugin", "windGustDuration"],
        "windGustVelocityMean": ["sdf", "world", "plugin", "wind_plugin", "windGustVelocityMean"],
        "windGustVelocityMax": ["sdf", "world", "plugin", "wind_plugin", "windGustVelocityMax"],
        "windGustVelocityVariance": ["sdf", "world", "plugin", "wind_plugin", "windGustVelocityVariance"],
        "windGustDirectionMean": ["sdf", "world", "plugin", "wind_plugin", "windGustDirectionMean"],
        "windGustDirectionVariance": ["sdf", "world", "plugin", "wind_plugin", "windGustDirectionVariance"]
}

IMU = {
        "gyroscopeNoiseDensity":  ["sdf", "model", "plugin", "rotors_gazebo_imu_plugin", "gyroscopeNoiseDensity"],
        "gyroscopeRandomWalk":  ["sdf", "model", "plugin", "rotors_gazebo_imu_plugin", "gyroscopeRandomWalk"],
        "gyroscopeBiasCorrelationTime":  ["sdf", "model", "plugin", "rotors_gazebo_imu_plugin", "gyroscopeBiasCorrelationTime"],
        "gyroscopeTurnOnBiasSigma":  ["sdf", "model", "plugin", "rotors_gazebo_imu_plugin", "gyroscopeTurnOnBiasSigma"],
        "accelerometerNoiseDensity":  ["sdf", "model", "plugin", "rotors_gazebo_imu_plugin", "accelerometerNoiseDensity"],
        "accelerometerRandomWalk":  ["sdf", "model", "plugin", "rotors_gazebo_imu_plugin", "accelerometerRandomWalk"],
        "accelerometerBiasCorrelationTime":  ["sdf", "model", "plugin", "rotors_gazebo_imu_plugin", "accelerometerBiasCorrelationTime"],
        "accelerometerTurnOnBiasSigma":  ["sdf", "model", "plugin", "rotors_gazebo_imu_plugin", "accelerometerTurnOnBiasSigma"]
}

GPS = {
        "gpsNoise":  ["sdf", "model", "link", "plugin", "gps_plugin", "gpsNoise"],
        "gpsXYRandomWalk":  ["sdf", "model", "link", "plugin", "gps_plugin", "gpsXYRandomWalk"],
        "gpsZRandomWalk":  ["sdf", "model", "link", "plugin", "gps_plugin", "gpsZRandomWalk"],
        "gpsXYNoiseDensity":  ["sdf", "model", "link", "plugin", "gps_plugin", "gpsXYNoiseDensity"],
        "gpsZNoiseDensity":  ["sdf", "model", "link", "plugin", "gps_plugin", "gpsZNoiseDensity"],
        "gpsVXYNoiseDensity":  ["sdf", "model", "link", "plugin", "gps_plugin", "gpsVXYNoiseDensity"],
        "gpsVZNoiseDensity":  ["sdf", "model", "link", "plugin", "gps_plugin", "gpsVZNoiseDensity"]
}