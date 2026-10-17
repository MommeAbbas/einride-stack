# PROVISIONAL topic /slam/docks; consistent landmark/map correction snapshot.
std_msgs/Header header # frame_id=map; correction effective time.
uint64 map_revision # Increment on map-coordinate corrections; invalidate older plans/views.
bool ready # Only after all eight tag/dock estimates satisfy configured readiness checks.
interfaces/DockLandmark[8] docks
