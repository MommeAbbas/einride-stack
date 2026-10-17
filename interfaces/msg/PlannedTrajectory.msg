# PROVISIONAL topic /planning/trajectory; not an untimed nav_msgs/Path.
std_msgs/Header header # frame_id=map; stamp equals first point time.
string goal_id # Unique mission goal identity, changes on Saga route/leg changes.
uint64 path_version # Increment on path/timing changes within a goal.
uint64 map_revision
interfaces/TrajectoryPoint[] points
