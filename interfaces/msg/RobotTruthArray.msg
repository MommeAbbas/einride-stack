# PROVISIONAL /sim/truth, frame_id=world. True world poses, NOT wheel odometry.
std_msgs/Header header
interfaces/RobotTruth[] robots # 1..4 physical robots; unlike predictions this varies with traffic_count.
