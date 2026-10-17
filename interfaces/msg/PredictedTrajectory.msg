# PROVISIONAL encoding; one destination/route-mode group, or the explicit leftover.
float64 probability # Particle mass [0,1]; groups plus leftover sum to 1 per robot.
int8 destination_dock -1 # A..H = 0..7; -1 only for mixed-destination leftover.
string route_mode # Opaque route-mode label; empty only for leftover.
interfaces/PredictionPoint[] points # Fixed dt through horizon, including both ends.
