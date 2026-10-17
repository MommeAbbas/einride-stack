# PROVISIONAL topic /perception/health; zero-initialised message fails closed.
uint8 FAILED=0
uint8 NOMINAL=1
uint8 DEGRADED=2
std_msgs/Header header # stamp=status publication time, frame_id=map.
uint8 status
string reason # No tokens or credentials.
builtin_interfaces/Time prediction_stamp # First point of corresponding prediction output.
uint64 map_revision
uint64 dropped_tf # Measurements dropped since this process started [count].
uint64 dropped_late # Measurements dropped since this process started [count].
bool has_camera_observation
interfaces/CameraObservation last_camera_observation
