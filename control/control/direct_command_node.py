"""Standalone Nav2 command gate for map/dock profiles; collision monitor remains last."""
import math
from geometry_msgs.msg import Twist
from .ros_support import AreaNode, run
from .hardware_motion import hardware_command


class DirectCommandNode(AreaNode):
    def __init__(self):
        super().__init__('control_direct_command')
        self.last=-math.inf
        self.command=Twist()
        self.pub=self.create_publisher(Twist,'/control/guard_input',10)
        self.create_subscription(Twist,'/control/nav2_cmd',self.receive,10)
        self.create_timer(1/self.p('coordinator_rate'),self.tick)

    def receive(self,message):
        self.last=self.now();self.command=message

    def tick(self):
        output=Twist()
        if self.now()-self.last<self.p('command_timeout'):
            v,w=self.command.linear.x,self.command.angular.z
            if self.p('hardware_minimum_commands'):
                v,w=hardware_command(v,w,self.p('direct_forward_limit'),self.p('reverse_speed_limit'),
                    self.p('navigation_turn_limit'),self.p('minimum_linear_command'),self.p('minimum_angular_command'),
                    self.p('linear_compensation_threshold'),self.p('angular_compensation_threshold'))
            output.linear.x=v;output.angular.z=w
        self.pub.publish(output)


def main():run(DirectCommandNode)
