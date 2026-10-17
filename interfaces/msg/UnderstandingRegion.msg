# Observation candidate from belief, not a risk decision.
geometry_msgs/Point center # map [m]; z=0 for planar region.
float64 radius # Region extent [m].
float64 unconfirmed_mass # Belief mass relevant to this region [0,1], not collision probability.
float64 target_heading # Suggested absolute camera-facing map heading [rad].
uint8[] track_ids # Anonymous track labels whose uncertainty supports the candidate.
