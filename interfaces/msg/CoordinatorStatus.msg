# PROVISIONAL topic /control/status. Acknowledged/canceled does not mean physically stopped.
uint8 UNKNOWN=0
uint8 ACKNOWLEDGED=1
uint8 STOPPING=2
uint8 ACTIVE=3
uint8 HOLDING=4
uint8 SUCCEEDED=5
uint8 FAILED=6
uint8 CANCELED=7
uint8 REJECTED=8
std_msgs/Header header # Status publication time, frame_id=map.
string request_id # Echo the request; late results must never reactivate motion.
string goal_id
uint64 path_version
uint64 map_revision
uint8 mode # CoordinatorRequest WAIT/DRIVE/LOOK.
uint8 status
bool stopped # Independently confirmed measured translation AND rotation below tolerances.
float64 progress # DRIVE path fraction or LOOK turn fraction [0,1].
float64 remaining_heading_error # LOOK signed angular error [rad].
bool terminal
builtin_interfaces/Time completion_stamp # Terminal event ROS time; zero if not terminal.
float64 final_heading # Camera-facing heading in header.frame_id [rad], valid only below.
bool final_heading_valid
string reason # Includes timeout, invalid TF, superseded goal, safety override as applicable.
