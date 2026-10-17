# PROVISIONAL timed own-robot state, including braking, WAIT and LOOK.
builtin_interfaces/Time stamp # Absolute ROS time [s, ns], strictly increasing along trajectory.
float64 x # map [m]
float64 y # map [m]
float64 yaw # Robot base heading about map +z [rad].
float64 linear_velocity # Forward speed [m/s].
float64 angular_velocity # Yaw rate [rad/s].
float64[9] covariance # Row-major x,y,yaw covariance [m^2, m*rad, rad^2]; symmetric PSD.
