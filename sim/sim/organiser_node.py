"""Local simulation organiser: authenticated real Saga buttons, no server mutations outside API."""
from concurrent.futures import ThreadPoolExecutor
import json
import math
from pathlib import Path
import time
from interfaces.msg import RobotTruthArray, SimRouteEvent
from .organiser import Organiser, target_dock
from .saga_client import SagaClient
from .ros_support import SimNode, seconds, run


class OrganiserNode(SimNode):
    def __init__(self):
        super().__init__('sim_organiser')
        self.declare_parameter('admin_username','admin'); self.declare_parameter('admin_password','')
        tokens=[]
        for truck in range(1,5):
            key=f'saga_token_{truck}'; self.declare_parameter(key,''); tokens.append((truck,self.p(key)))
        self.api=SagaClient(self.p('saga_url'),self.p('http_timeout'),tokens,
                            (self.p('admin_username'),self.p('admin_password')))
        self.logic=Organiser(self.p('dwell_seconds'),self.p('wall_lock_seconds'))
        self.docks=json.loads((Path(self.p('run_dir'))/'geometry.json').read_text())['docks']
        self.robots={}; self.pose_time=-math.inf
        self.worker=ThreadPoolExecutor(max_workers=1); self.future=None
        self.pub=self.create_publisher(SimRouteEvent,'/sim/routes',self.p('qos_depth'))
        self.create_subscription(RobotTruthArray,'/sim/truth',self.truth,self.p('qos_depth'))
        self.create_timer(self.p('saga_poll_period'),self.tick)

    def truth(self,message):
        self.robots={r.robot_id:r for r in message.robots}; self.pose_time=seconds(message.header.stamp)

    def cycle(self,sim_now,robots):
        events=[]
        for truck in range(1,5):
            data=self.api.route(truck)
            if data['event_state']=='not_started':
                if robots and sim_now>=self.p('event_start_delay'):
                    self.api.start()
                return events
            route=data['route']
            if not route:
                continue
            robot=robots.get(truck-1); at=False
            dock=target_dock(route)
            if robot:
                at=(math.dist((robot.pose.position.x,robot.pose.position.y),self.docks[dock['letter']][:2])<=self.p('arrival_tolerance')
                    and robot.velocity_valid
                    and math.hypot(robot.twist.linear.x,robot.twist.linear.y)<=self.p('stopped_speed')
                    and abs(robot.twist.angular.z)<=self.p('stopped_turn_rate'))
            now=time.monotonic()
            action=self.logic.decide(truck,route,at,sim_now,now) if data['event_state']=='running' else None
            dwell=self.logic.dwells.get(truck)
            ds=sim_now-dwell.sim_start if dwell else 0.; dw=now-dwell.wall_start if dwell else 0.
            succeeded=bool(action and self.api.action(truck,action,route['status']))
            if succeeded:
                self.logic.acknowledged(truck,route,action,self.now(),time.monotonic())
            status=self.api.status(truck)
            events.append(dict(truck_id=truck,route_id=route['route_id'],source_dock=route['source']['tag_id'],
                destination_dock=route['destination']['tag_id'],status=route['status'],
                action=action if succeeded else 'snapshot',completed_routes=status['completed_routes'],
                dwell_sim=ds if succeeded and action in ('loading_complete','unloading_complete') else 0.,
                dwell_wall=dw if succeeded and action in ('loading_complete','unloading_complete') else 0.))
        return events

    def tick(self):
        if self.future is not None and self.future.done():
            try:
                for fields in self.future.result():
                    msg=SimRouteEvent(**fields); msg.header.stamp=self.get_clock().now().to_msg(); msg.header.frame_id='world'
                    self.pub.publish(msg)
                    if msg.action!='snapshot':
                        self.get_logger().info(f'Truck {msg.truck_id} route {msg.route_id}: {msg.action}; '
                                              f'dwell sim={msg.dwell_sim:.2f}s wall={msg.dwell_wall:.2f}s')
            except Exception as exc:
                self.get_logger().warning('Saga organiser retry: '+type(exc).__name__)
            self.future=None
        if self.future is None:
            robots=self.robots.copy() if self.now()-self.pose_time<=self.p('pose_stale_timeout') else {}
            if robots:
                self.future=self.worker.submit(self.cycle,self.now(),robots)
            else:
                # A gap in true-pose coverage cannot establish continuous stopped dwell.
                for dwell in self.logic.dwells.values():
                    dwell.sim_start=self.now()

    def destroy_node(self):
        self.worker.shutdown(wait=True,cancel_futures=True)
        return super().destroy_node()


def main():
    run(OrganiserNode)
