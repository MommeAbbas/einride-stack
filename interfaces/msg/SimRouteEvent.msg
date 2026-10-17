# PROVISIONAL /sim/routes, organiser audit. Simulation evaluation only, not a planning input.
std_msgs/Header header # ROS simulation time; world frame.
uint8 truck_id # Saga 1..4.
int64 route_id
int8 source_dock -1
int8 destination_dock -1
string status
string action # snapshot, arrived_at_source, loading_complete, arrived_at_destination, unloading_complete.
uint32 completed_routes
float64 dwell_sim # Continuous dock dwell before successful completion [s].
float64 dwell_wall # Wall time since arrival acknowledgement [s].
