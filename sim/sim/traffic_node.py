"""One Saga client/A* driver per traffic robot; publishes only its namespaced velocity."""
from concurrent.futures import ThreadPoolExecutor
import json
import math
from pathlib import Path
import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from interfaces.msg import RobotTruthArray
from .grid import Grid
from .traffic import command, mode_command, yield_command
from .saga_client import SagaClient
from .ros_support import SimNode, seconds, yaw, run


class TrafficNode(SimNode):
    def __init__(self):
        super().__init__('sim_traffic')
        self.declare_parameter('robot_id',1); self.declare_parameter('saga_token','')
        self.robot_id=self.p('robot_id')
        if self.robot_id not in range(1,4):
            raise ValueError('Traffic robot_id must be 1..3; ego commands are Control-only')
        self.mode=self.p('traffic_modes')[self.robot_id-1]
        if self.mode not in ('ignore','stop_near','random_wander','yield'):
            raise ValueError('Invalid traffic mode')
        self.api=SagaClient(self.p('saga_url'),self.p('http_timeout'),[(self.robot_id+1,self.p('saga_token'))])
        self.worker=ThreadPoolExecutor(max_workers=1); self.future=None; self.route=None; self.route_time=-math.inf
        self.robots={}; self.pose_time=-math.inf; self.path=[]; self.goal_key=None; self.wander_until=0.
        self.rng=np.random.default_rng(self.p('route_seed')+self.robot_id)
        data=np.load(Path(self.p('run_dir'))/'map.npz')
        self.grid=Grid(data['occupied'],float(data['resolution']),data['origin'])
        self.docks=json.loads((Path(self.p('run_dir'))/'geometry.json').read_text())['docks']
        self.radius=math.hypot(self.p('robot_length'),self.p('robot_width'))/2+self.p('obstacle_clearance')
        self.pub=self.create_publisher(Twist,f'/traffic_{self.robot_id}/cmd_vel',self.p('qos_depth'))
        self.create_subscription(RobotTruthArray,'/sim/truth',self.truth,self.p('qos_depth'))
        self.create_timer(self.p('saga_poll_period'),self.poll)
        self.create_timer(1/self.p('control_rate'),self.tick)

    def truth(self,message):
        self.robots={r.robot_id:(r.pose.position.x,r.pose.position.y,yaw(r.pose.orientation)) for r in message.robots}
        self.pose_time=seconds(message.header.stamp)

    def poll(self):
        if self.future is not None and self.future.done():
            try:
                self.route=self.future.result(); self.route_time=self.now()
            except Exception:
                self.get_logger().warning('Saga polling failed; holding until a fresh response')
                self.route=None
            self.future=None
        if self.future is None:
            self.future=self.worker.submit(self.api.route,self.robot_id+1)

    def tick(self):
        v,w=0.,0.; now=self.now(); pose=self.robots.get(self.robot_id)
        if pose and now-self.pose_time<=self.p('pose_stale_timeout') and self.route and self.route['event_state']=='running' and now-self.route_time<=self.p('saga_stale_timeout'):
            route=self.route['route']; dock=route['next_dock'] if route else None
            if self.mode=='random_wander':
                if now>=self.wander_until and not self.path:
                    letter=str(self.rng.choice(list(self.docks)))
                    self.path=self.grid.plan(pose[:2],self.docks[letter][:2],self.radius)
                dock=True
            elif dock:
                key=(route['route_id'],dock['letter'])
                if key!=self.goal_key:
                    self.path=self.grid.plan(pose[:2],self.docks[dock['letter']][:2],self.radius)
                    self.goal_key=key if self.path else None
            if dock and self.path:
                v,w,arrived=command(pose,self.path,self.p('traffic_speed'),self.p('traffic_max_turn_rate'),
                    self.p('heading_gain'),self.p('turn_in_place_angle'),self.p('waypoint_tolerance'),self.p('goal_tolerance'))
                if arrived and self.mode=='random_wander':
                    self.path=[]; self.wander_until=now+self.p('wander_dwell')
            v,w=mode_command(self.mode,(v,w),pose,self.robots.get(0),self.p('stop_near_distance'))
            if self.mode=='yield':v,w=yield_command((v,w),pose,[p for ident,p in self.robots.items() if ident!=self.robot_id],self.p('traffic_yield_distance'),self.p('traffic_yield_half_angle'))
        msg=Twist(); msg.linear.x=v; msg.angular.z=w; self.pub.publish(msg)

    def destroy_node(self):
        if rclpy.ok():
            self.pub.publish(Twist())
        self.worker.shutdown(wait=True,cancel_futures=True)
        return super().destroy_node()


def main():
    run(TrafficNode)
