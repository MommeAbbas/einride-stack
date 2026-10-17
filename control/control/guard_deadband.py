"""Executable candidates, checked again by installed Nav2 before publication.

No geometry here: two installed CollisionMonitor instances own sensor freshness,
Stop and Approach collision checking. The final gate can only pass or zero the
second monitor's result; it never increases that checked command.
"""
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String


def candidate(v,w,linear,angular):
    if not all(math.isfinite(x) for x in (v,w)):return 0.,0.
    if v and abs(v)<linear:
        scale=linear/abs(v);return math.copysign(linear,v),w*scale
    if not v and w and abs(w)<angular:return 0.,math.copysign(angular,w)
    return v,w


def executable(v,w,linear,angular):
    return all(math.isfinite(x) for x in (v,w)) and (abs(v)>=linear or v==0. and (w==0. or abs(w)>=angular))


class GuardDeadband(Node):
    def __init__(self):
        super().__init__('control_guard_deadband')
        for key,value in [('minimum_linear_command',.15),('minimum_angular_command',.35),('navigation_turn_limit',.8),('command_timeout',.5),('guard_watchdog_rate',5.)]:self.declare_parameter(key,value)
        self.linear=self.get_parameter('minimum_linear_command').value;self.angular=self.get_parameter('minimum_angular_command').value
        self.angular_cap=self.get_parameter('navigation_turn_limit').value
        if not 0<self.linear or not 0<self.angular<=self.angular_cap:raise ValueError('Invalid executable velocity bounds')
        self.proposal=self.create_publisher(Twist,'/control/guard_proposal',1)
        self.output=self.create_publisher(Twist,'/cmd_vel',1)
        self.state=self.create_publisher(String,'/control/guard_deadband_state',1)
        self.last=-math.inf;self.last_state=None;self.moving=False
        self.create_subscription(Twist,'/control/guard_reduced',self.propose,1)
        self.create_subscription(Twist,'/control/guard_checked',self.checked,1)
        self.create_timer(1/self.get_parameter('guard_watchdog_rate').value,self.watchdog)

    def propose(self,m):
        v,w=candidate(m.linear.x,m.angular.z,self.linear,self.angular)
        # Curvature-preserving proposal must not exceed the existing turn cap.
        # A forbidden proposal becomes zero, not a different untested curve.
        if abs(w)>self.angular_cap:v,w=0.,0.
        out=Twist();out.linear.x=v;out.angular.z=w;self.proposal.publish(out)

    def checked(self,m):
        self.last=self.get_clock().now().nanoseconds/1e9
        valid=executable(m.linear.x,m.angular.z,self.linear,self.angular)
        out=m if valid else Twist();self.output.publish(out)
        self.moving=bool(out.linear.x or out.angular.z)
        state='validated executable command' if self.moving else 'zero' if valid else 'validated result below deadband: zero'
        if state!=self.last_state:self.state.publish(String(data=state));self.last_state=state

    def watchdog(self):
        if self.moving and self.get_clock().now().nanoseconds/1e9-self.last>self.get_parameter('command_timeout').value:
            self.output.publish(Twist());self.moving=False;self.state.publish(String(data='validation receipt expired: zero'))

    def destroy_node(self):
        if rclpy.ok(context=self.context):self.output.publish(Twist())
        super().destroy_node()


def main():
    from .ros_support import run
    run(GuardDeadband)


if __name__=='__main__':main()
