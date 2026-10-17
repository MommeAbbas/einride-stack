# PROVISIONAL SIMULATION ONLY. Never feed this to production perception/planning.
uint8 robot_id # 0=ours, 1..3=traffic; Saga truck_id=robot_id+1.
string model_name
geometry_msgs/Pose pose # True world pose [m, quaternion].
geometry_msgs/Twist twist # True finite-difference world velocity [m/s, rad/s].
bool velocity_valid # False for first pose or discontinuity/teleport.
