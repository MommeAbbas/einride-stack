# PROVISIONAL physical occupancy, not Saga reservation; derived from the belief.
int8 dock_id -1 # A..H = 0..7; -1 invalid.
bool valid # False means unknown; never infer free from a default message.
float64 occupancy_probability # Probability any other robot occupies the dock [0,1].
bool physically_free # Only meaningful when valid; threshold is a ROS parameter.
