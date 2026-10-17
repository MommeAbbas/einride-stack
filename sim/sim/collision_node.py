"""Publish true collision onset events; never commands a robot."""
from std_msgs.msg import Float64, Bool
from interfaces.msg import CollisionEvent, RobotTruthArray
from .collision import CollisionMonitor
from .ros_support import SimNode, yaw, run


class CollisionNode(SimNode):
    def __init__(self):
        super().__init__('sim_collision')
        self.monitor=CollisionMonitor(self.p('collision_footprint_length'),self.p('collision_footprint_width'),
                                      self.p('major_collision_speed'),self.p('collision_release_distance'))
        self.forced=False
        self.pub=self.create_publisher(CollisionEvent,'/sim/collisions',self.p('qos_depth'))
        self.distance=self.create_publisher(Float64,'/sim/minimum_separation',self.p('qos_depth'))
        self.create_subscription(RobotTruthArray,'/sim/truth',self.receive,self.p('qos_depth'))
        self.create_subscription(Bool,'/sim/forced_collision',self.force,self.p('qos_depth'))

    def force(self,message):
        self.forced=message.data

    def receive(self,message):
        robots={r.robot_id:((r.pose.position.x,r.pose.position.y,yaw(r.pose.orientation)),
                            (r.twist.linear.x,r.twist.linear.y),r.velocity_valid) for r in message.robots}
        events,distance=self.monitor.update(robots)
        self.distance.publish(Float64(data=distance))
        for event in events:
            result=CollisionEvent(**event); result.header=message.header; result.forced_test=self.forced
            self.pub.publish(result)
            self.get_logger().info(f'Collision {result.event_id}: {result.first_robot}/{result.second_robot}, '
                                   f'severity={result.severity}, relative_speed={result.relative_speed:.3f} m/s')


def main():
    run(CollisionNode)
