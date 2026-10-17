# Usable-view receipt; motion success alone is insufficient.
std_msgs/Header header # frame_id=map; stamp is image CAPTURE time, never completion time.
uint64 map_revision
geometry_msgs/Pose camera_pose # Capture-time optical pose in map [m, quaternion].
float64 horizontal_fov # Usable horizontal field of view after edge margin [rad].
float64 max_range # Usable detection/negative-evidence range [m].
bool processed # Detector actually processed this image, even if no detections.
bool negative_evidence_usable # TF, turn rate, sensor health and visibility checks passed.
uint32 detection_count # Count in this processed frame; zero is not detector failure.
bool negative_evidence_applied # Saw-nothing update ACTUALLY applied; eligibility alone is insufficient.
builtin_interfaces/Time predictions_after_frame # First-point stamp of published predictions incorporating this frame; must be > header.stamp.
