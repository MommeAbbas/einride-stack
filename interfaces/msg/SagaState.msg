# PROVISIONAL topic /planning/saga_state; ego robot only. Never includes a token.
string NOT_STARTED=not_started
string RUNNING=running
string STOPPED=stopped
string LOADING_PENDING=loading_pending
string LOADING_IN_PROGRESS=loading_in_progress
string LOADING_COMPLETE=loading_complete
string UNLOADING_IN_PROGRESS=unloading_in_progress
string UNLOADING_COMPLETE=unloading_complete
std_msgs/Header header # stamp=last successful GET response in ROS time, NOT republish time.
bool available # Have ever received a valid response; consumers additionally check stamp age.
uint8 truck_id # Mission identity 1..4; production consumes the ego robot only.
string event_state # not_started/running/stopped; empty is unknown and requires WAIT.
bool has_route # False for Saga route:null; route fields then ignored.
int64 route_id # Server route identity, meaningful only if has_route.
string route_status # One of the five Saga status strings; empty if no route.
int8 source_dock -1 # 0..7; -1 absent.
int8 destination_dock -1 # 0..7; -1 absent.
int8 next_dock -1 # -1 is null (hold), never dock A.
int8[] held_docks # Destinations excluded for other robots, NOT physical free-space claims.
