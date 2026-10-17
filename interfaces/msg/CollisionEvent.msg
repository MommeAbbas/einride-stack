# PROVISIONAL /sim/collisions. One event per contact onset using oriented footprints.
uint8 MINOR=1
uint8 MAJOR=2
std_msgs/Header header # world frame, ROS simulation time.
uint64 event_id
uint8 first_robot
uint8 second_robot
uint8 severity
float64 relative_speed # Norm of true relative center velocity [m/s].
bool speed_valid # False after teleport; forced-test contact must not claim measured speed.
bool forced_test
