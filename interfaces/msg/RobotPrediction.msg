# PROVISIONAL encoding. Track labels are NOT Saga truck IDs or true-pose identities.
uint8 track_id # Stable anonymous label 0..2; label swaps do not change safety.
bool ever_confirmed # False means unknown age, not a fresh observation.
builtin_interfaces/Duration confirmation_age # Since last supporting measurement [s, ns].
interfaces/PredictedTrajectory[] groups # At most max_groups (ROS parameter).
interfaces/PredictedTrajectory leftover # Always a full timed trajectory, even at zero mass.
