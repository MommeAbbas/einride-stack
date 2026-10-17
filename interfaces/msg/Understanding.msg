# PROVISIONAL topic /perception/understanding; tied to the timed plan being evaluated.
std_msgs/Header header # Evaluation time; frame_id=map.
uint64 map_revision
string goal_id
uint64 path_version
interfaces/UnderstandingRegion[] regions
