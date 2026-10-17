# Coordinator request carried on /control/request.
uint8 WAIT=0
uint8 DRIVE=1
uint8 LOOK=2
std_msgs/Header header # frame_id=map; request issue time.
string request_id # Nonempty globally unique ID; repeated identical requests are idempotent.
string goal_id # Empty allowed for unconditional WAIT.
uint64 map_revision
uint8 mode # Default WAIT; unknown modes rejected and held.
builtin_interfaces/Time valid_until # Absolute lease expiry; heartbeat may only extend this.
string reason
interfaces/PlannedTrajectory trajectory # DRIVE only; goal/revision must match outer fields.
float64 speed_limit # DRIVE absolute nonnegative limit [m/s], NOT percent.
float64 target_heading # LOOK absolute desired camera-facing map heading [rad].
builtin_interfaces/Duration look_timeout # LOOK execution timeout [s, ns].
