"""Coordinator glue only: Nav2 implements Wait, BackUp and collision checking."""
import random
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Point,Twist
from nav2_msgs.action import Wait,BackUp
from rclpy.action import ActionClient
from std_srvs.srv import Trigger


def duration(seconds):
    ns=round(seconds*1e9)
    return Duration(sec=ns//10**9,nanosec=ns%10**9)


class Encounter:
    def __init__(self,node):
        self.node=node;self.state='IDLE';self.generation=0;self.handle=None;self.future=None
        self.wait=ActionClient(node,Wait,'/wait');self.backup=ActionClient(node,BackUp,'/backup')
        self.replan=node.create_client(Trigger,node.p('encounter_replan_service'))
        self.rng=random.Random(node.p('encounter_seed'));self.after_backup=False
        self.raw=None;self.raw_time=-float('inf');self.detail=''
        node.create_subscription(Twist,'/control/behavior_cmd',self.command,10)

    def command(self,message):
        if self.state=='BACKUP':self.raw=message;self.raw_time=self.node.now()

    def cancel(self):
        self.generation+=1
        if self.handle is not None:self.handle.cancel_goal_async()
        self.handle=None;self.future=None;self.state='IDLE';self.raw=None

    def start(self):
        self.cancel();self.state='WAIT_START';self.after_backup=False

    def send(self,client,goal,state,next_state):
        if not client.server_is_ready():return
        self.state=state;generation=self.generation
        self.future=client.send_goal_async(goal)
        def accepted(future):
            try:handle=future.result()
            except Exception:
                if generation==self.generation:self.state='WAIT_START'
                return
            if generation!=self.generation:
                if handle.accepted:handle.cancel_goal_async()
                return
            self.future=None
            if not handle.accepted:self.state='WAIT_START';self.detail=state+' rejected';return
            self.handle=handle
            def done(result):
                if generation!=self.generation:return
                self.handle=None;self.raw=None
                try:self.detail=f'{state} result={result.result().status}'
                except Exception:self.detail=state+' result unavailable'
                self.state=next_state
            handle.get_result_async().add_done_callback(done)
        self.future.add_done_callback(accepted)

    def tick(self):
        n=self.node
        if self.state=='WAIT_START':
            seconds=n.p('encounter_wait')+self.rng.uniform(0.,n.p('encounter_jitter'))
            self.after_backup=False
            self.send(self.wait,Wait.Goal(time=duration(seconds)),'WAIT','REPLAN')
        elif self.state=='REPLAN' and self.future is None and self.replan.service_is_ready():
            generation=self.generation;self.future=self.replan.call_async(Trigger.Request())
            self.service_started=n.now()
            def replanned(future):
                if generation!=self.generation or self.state!='REPLAN':return
                self.future=None
                try:ok=future.result().success;self.detail=future.result().message
                except Exception:ok=False;self.detail='Replan service failed'
                if ok:self.state='READY'
                elif self.after_backup:self.state='WAIT_START'
                else:self.state='BACKUP_START'
            self.future.add_done_callback(replanned)
        elif self.state=='REPLAN' and self.future is not None and n.now()-self.service_started>n.p('encounter_service_timeout'):
            self.generation+=1;self.future=None;self.state='WAIT_START';self.detail='Replan timed out'
        elif self.state=='BACKUP_START':
            self.after_backup=True
            # Jazzy BackUp has no disable_collision_checks field: its costmap
            # checker is always used. Its velocity still goes through the raw guard.
            goal=BackUp.Goal(target=Point(x=-n.p('encounter_backup_distance')),speed=n.p('encounter_backup_speed'),time_allowance=duration(n.p('encounter_backup_timeout')))
            self.send(self.backup,goal,'BACKUP','REPLAN')
        if self.state=='BACKUP' and self.raw is not None and n.now()-self.raw_time<n.p('command_timeout'):
            return self.raw
        return Twist()
