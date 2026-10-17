# PROVISIONAL topic /perception/dock_states.
std_msgs/Header header # Belief evaluation time; frame_id=map.
uint64 map_revision
interfaces/DockState[8] docks # Exactly one record per dock 0..7.
