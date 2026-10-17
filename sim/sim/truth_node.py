"""Convert Gazebo world model poses into explicit simulation-only truth records."""
import math
from builtin_interfaces.msg import Time
from gz.transport13 import Node as GazeboNode
from gz.msgs10.pose_v_pb2 import Pose_V
from interfaces.msg import RobotTruth, RobotTruthArray
from .geometry import wrap
from .ros_support import SimNode, seconds, yaw, run


class TruthNode(SimNode):
    def __init__(self):
        super().__init__('sim_truth')
        self.previous={}; self.last_publish=-math.inf
        self.names={'einride_mini_truck':0, **{f'traffic_{i}':i for i in range(1,self.p('traffic_count')+1)}}
        self.pub=self.create_publisher(RobotTruthArray,'/sim/truth',self.p('qos_depth'))
        self.gz=GazeboNode(); self.latest=None
        self.gz.subscribe(Pose_V,'/world/arena/dynamic_pose/info',self.capture)
        self.create_timer(1/self.p('truth_rate'),self.flush)

    def capture(self,message):
        self.latest=message

    def flush(self):
        message=self.latest; self.latest=None
        if message is not None:
            self.receive(message)

    def receive(self,message):
        transforms={p.name:p for p in message.pose if p.name in self.names}
        if not transforms:
            return
        stamp=Time(sec=message.header.stamp.sec,nanosec=message.header.stamp.nsec)
        now=seconds(stamp)
        if now < self.last_publish:
            self.previous.clear(); self.last_publish=-math.inf
        if now-self.last_publish < 1/self.p('truth_rate'):
            return
        result=RobotTruthArray(); result.header.stamp=stamp; result.header.frame_id='world'
        for name,t in transforms.items():
            robot=RobotTruth(robot_id=self.names[name],model_name=name)
            robot.pose.position.x=t.position.x
            robot.pose.position.y=t.position.y
            robot.pose.position.z=t.position.z
            robot.pose.orientation.x=t.orientation.x; robot.pose.orientation.y=t.orientation.y
            robot.pose.orientation.z=t.orientation.z; robot.pose.orientation.w=t.orientation.w
            current=(now,robot.pose.position.x,robot.pose.position.y,yaw(robot.pose.orientation))
            old=self.previous.get(name)
            if old and now>old[0]:
                dt=now-old[0]; vx=(current[1]-old[1])/dt; vy=(current[2]-old[2])/dt
                robot.velocity_valid=math.hypot(vx,vy)<self.p('truth_jump_speed')
                if robot.velocity_valid:
                    robot.twist.linear.x=vx; robot.twist.linear.y=vy
                    robot.twist.angular.z=wrap(current[3]-old[3])/dt
            self.previous[name]=current
            result.robots.append(robot)
        self.last_publish=now; self.pub.publish(result)


def main():
    run(TruthNode)
