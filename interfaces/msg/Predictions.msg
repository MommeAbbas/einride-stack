# Prediction collection carried on /perception/predictions.
std_msgs/Header header # frame_id=map; stamp equals the FIRST point of every trajectory.
uint64 map_revision # Same correction epoch as DockMap and PlannedTrajectory.
builtin_interfaces/Duration time_step # Uniform dt [s, ns], configured by ROS parameter.
builtin_interfaces/Duration horizon # T [s, ns]; >= risk_horizon + max_sensor_lag.
interfaces/RobotPrediction[3] robots # Always three, including in empty-arena tests.
