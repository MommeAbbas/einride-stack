# SLAM and localization

Mapping and localization integration for ROS 2. Mapping uses `slam_toolbox`;
saved-map localization uses Nav2 AMCL and map server. The package also estimates
dock poses from fiducial tags, saves mapping snapshots, and provides hardware
wheel/IMU odometry.

- `config/mapping.yaml` and `config/localisation.yaml`: maintained backend setup.
- `config/slam.yaml`: dock mapping and snapshot parameters.
- `config/wheel_odometry.yaml`: encoder and gyro odometry parameters.
- `launch/slam.launch.py`: package launch entry point.

```bash
ros2 launch slam slam.launch.py
colcon test --packages-select slam
```
