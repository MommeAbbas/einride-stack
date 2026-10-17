# Dock landmark representation; tag pose is NOT the docking pose.
int8 dock_id -1 # Also tag36h11 ID; 0..7, -1 invalid.
bool valid
geometry_msgs/PoseWithCovariance tag_pose # map, [m, rad], standard ROS covariance order.
geometry_msgs/PoseWithCovariance approach_pose # Derived using parameterised offsets.
geometry_msgs/PoseWithCovariance docking_pose # Derived using parameterised offsets.
builtin_interfaces/Time last_observed # Tag observation capture time [s, ns].
