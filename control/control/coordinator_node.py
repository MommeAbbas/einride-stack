"""Control-owned action coordinator; every command passes Nav2 collision monitor."""
import math
from collections import deque
import rclpy
from rclpy.action import ActionClient
from rclpy.time import Time
from rclpy.qos import qos_profile_sensor_data
from tf2_ros import Buffer,TransformListener,TransformException
from nav2_msgs.action import FollowPath
from nav2_msgs.msg import SpeedLimit,CollisionMonitorState
from nav_msgs.msg import Odometry,Path
from geometry_msgs.msg import Twist
from interfaces.msg import CoordinatorRequest,CoordinatorStatus,Predictions,PerceptionHealth,DockMap
from .ros_support import AreaNode,seconds,yaw,latched,run
from .coordinator import validate,turn_command,wrap
from .encounter import Encounter


class CoordinatorNode(AreaNode):
    def __init__(self):
        super().__init__('control_coordinator');self.buffer=Buffer();self.listener=TransformListener(self.buffer,self)
        self.request=None;self.odom=None;self.raw=None;self.raw_time=-math.inf;self.predictions=None;self.health=None;self.revision=None
        self.started=0.;self.turning=False;self.terminal=False;self.completion=None;self.goal_handle=None;self.pending=None;self.sent_id=None;self.result=None
        self.progress=0.;self.initial_heading_error=None;self.guard=None
        self.create_subscription(CollisionMonitorState,'/control/collision_state',lambda m:setattr(self,'guard',m),10)
        self.paths=deque(maxlen=self.p('qos_depth'))
        self.action=ActionClient(self,FollowPath,'/follow_path')
        self.create_subscription(CoordinatorRequest,'/control/request',self.on_request,10)
        self.create_subscription(Odometry,'/odom',lambda m:setattr(self,'odom',m),qos_profile_sensor_data)
        self.create_subscription(Twist,'/control/mppi_cmd',self.on_raw,10)
        self.create_subscription(Path,'/planning/path',lambda m:self.paths.append(m),10)
        self.create_subscription(Predictions,'/perception/predictions',lambda m:setattr(self,'predictions',m),10)
        self.create_subscription(PerceptionHealth,'/perception/health',lambda m:setattr(self,'health',m),10)
        self.create_subscription(DockMap,'/slam/docks',lambda m:setattr(self,'revision',m.map_revision),latched())
        self.pub=self.create_publisher(Twist,'/control/guard_input',10)
        self.status_pub=self.create_publisher(CoordinatorStatus,'/control/status',10)
        self.limit_pub=self.create_publisher(SpeedLimit,'/speed_limit',10)
        self.encounter=Encounter(self)
        self.create_timer(1/self.p('coordinator_rate'),self.tick)

    def on_raw(self,m):self.raw=m;self.raw_time=self.now()

    def cancel(self):
        if self.goal_handle is not None:self.goal_handle.cancel_goal_async()
        self.goal_handle=None;self.sent_id=None;self.raw=None
        if hasattr(self,'encounter'):self.encounter.cancel()

    def on_request(self,m):
        now=self.now();error=validate(m.mode,m.header.frame_id,seconds(m.valid_until),now,m.goal_id,m.map_revision,m.trajectory.map_revision,m.speed_limit)
        if error:
            self.cancel();self.request=m;self.terminal=True;self.result=(CoordinatorStatus.REJECTED,error);self.completion=self.get_clock().now().to_msg();return
        if self.request is not None and self.request.request_id==m.request_id:
            # A heartbeat can only extend a lease, never mutate active mode/goal/path.
            if (m.mode,m.goal_id,m.map_revision,m.trajectory.path_version,m.target_heading)!=(self.request.mode,self.request.goal_id,self.request.map_revision,self.request.trajectory.path_version,self.request.target_heading):
                self.cancel();self.terminal=True;self.result=(CoordinatorStatus.REJECTED,'request ID mutated');return
            self.request.valid_until=m.valid_until;return
        if self.request is not None and not self.terminal:
            old=self.request;status=CoordinatorStatus(request_id=old.request_id,goal_id=old.goal_id,map_revision=old.map_revision,path_version=old.trajectory.path_version,mode=old.mode,status=CoordinatorStatus.CANCELED,terminal=True,reason='Superseded by a new request; stopping is confirmed separately')
            status.header.frame_id='map';status.header.stamp=self.get_clock().now().to_msg();status.completion_stamp=status.header.stamp
            self.status_pub.publish(status)
        self.progress=0.;self.initial_heading_error=None
        if self.request is not None and self.request.mode==m.mode==CoordinatorRequest.DRIVE and self.request.goal_id==m.goal_id and self.request.map_revision==m.map_revision and not self.request.reason.startswith('ENCOUNTER:') and not m.reason.startswith('ENCOUNTER:'):
            self.sent_id=None # FollowPath accepts preemption; do not inject a stop on every refreshed path.
        else:self.cancel()
        self.request=m;self.started=now;self.turning=False;self.terminal=False;self.completion=None;self.result=None

    def predictions_valid(self):
        if not self.p('require_predictions'):return True
        p,h=self.predictions,self.health;now=self.now()
        return p is not None and h is not None and h.status!=h.FAILED and p.map_revision==self.revision and h.map_revision==self.revision and now-seconds(p.header.stamp)<self.p('prediction_stale_timeout') and now-seconds(h.header.stamp)<self.p('prediction_stale_timeout') and all(g.points and seconds(g.points[-1].stamp)>=now+self.p('mppi_horizon') for r in p.robots for g in [*r.groups,r.leftover] if g.probability>0)

    def send_path(self):
        req=self.request
        path=next((p for p in reversed(self.paths) if p.header.stamp==req.trajectory.header.stamp),None)
        if path is None or not path.poses or not self.action.server_is_ready():return
        goal=FollowPath.Goal(path=path,controller_id='FollowPath',goal_checker_id='goal_checker')
        ident=req.request_id;self.sent_id=ident
        self.limit_pub.publish(SpeedLimit(speed_limit=req.speed_limit,percentage=False))
        length=sum(math.hypot(b.pose.position.x-a.pose.position.x,b.pose.position.y-a.pose.position.y) for a,b in zip(path.poses,path.poses[1:]))
        def feedback(message):
            if self.request is not None and self.request.request_id==ident and length>0:
                self.progress=max(0.,min(1.,1-message.feedback.distance_to_goal/length))
        self.pending=self.action.send_goal_async(goal,feedback_callback=feedback)
        def accepted(future):
            handle=future.result()
            if self.request is None or self.request.request_id!=ident:
                if handle.accepted:handle.cancel_goal_async()
                return
            if not handle.accepted:self.result=(CoordinatorStatus.FAILED,'FollowPath rejected');self.terminal=True;return
            self.goal_handle=handle
            def completed(done):
                if self.request is None or self.request.request_id!=ident:return
                result=done.result();self.terminal=True;self.completion=self.get_clock().now().to_msg()
                self.result=(CoordinatorStatus.SUCCEEDED if result.status==4 else CoordinatorStatus.FAILED,'FollowPath terminal')
                self.raw=None
            handle.get_result_async().add_done_callback(completed)
        self.pending.add_done_callback(accepted)

    def tick(self):
        now=self.now();command=Twist();req=self.request
        odom_fresh=self.odom is not None and now-seconds(self.odom.header.stamp)<=self.p('odom_stale_timeout')
        stopped=odom_fresh and abs(self.odom.twist.twist.linear.x)<self.p('stopped_speed') and abs(self.odom.twist.twist.angular.z)<self.p('stopped_turn_rate')
        heading=0.;heading_valid=False;error=0.;state=CoordinatorStatus.HOLDING;reason='No request'
        try:
            transform=self.buffer.lookup_transform('map','base_footprint',Time());t=transform.transform;heading=yaw(t.rotation)
            heading_valid=now-seconds(transform.header.stamp)<=self.p('pose_stale_timeout')
        except TransformException:pass
        if req is not None:
            reason=req.reason
            invalid=seconds(req.valid_until)<=now or (req.mode!=req.WAIT and (not heading_valid or not odom_fresh or req.map_revision!=self.revision))
            if invalid:
                self.cancel();self.terminal=True;self.result=(CoordinatorStatus.FAILED,'expired lease, stale odom, TF or map revision')
                if self.completion is None:self.completion=self.get_clock().now().to_msg()
            elif req.mode==req.DRIVE and req.reason.startswith('ENCOUNTER:') and not self.terminal:
                if self.encounter.state=='IDLE':self.encounter.start()
                command=self.encounter.tick();state=CoordinatorStatus.ACTIVE
                reason='ENCOUNTER '+self.encounter.state+': '+self.encounter.detail
            elif req.mode==req.DRIVE and not self.terminal:
                if self.predictions_valid():
                    if self.sent_id!=req.request_id:self.send_path()
                    if self.goal_handle is not None and self.raw is not None and now-self.raw_time<self.p('command_timeout'):
                        command=self.raw;state=CoordinatorStatus.ACTIVE
                    else:state=CoordinatorStatus.ACKNOWLEDGED
                else:self.cancel();state=CoordinatorStatus.STOPPING;reason='Unusable prediction: WAIT'
            elif req.mode==req.LOOK and not self.terminal:
                error=wrap(req.target_heading-heading-self.p('camera_yaw_offset'))
                if self.initial_heading_error is None:self.initial_heading_error=abs(error)
                self.progress=1. if self.initial_heading_error==0 else max(0.,min(1.,1-abs(error)/self.initial_heading_error))
                if self.predictions_valid():
                    w,label=turn_command(error,stopped,self.turning,now-self.started,{k:self.p(k) for k in ('look_timeout_limit','heading_tolerance','look_turn_rate','look_gain')})
                    if now-self.started>seconds(req.look_timeout):w=0.;label='FAILED'
                    command.angular.z=w;state=getattr(CoordinatorStatus,label)
                    if label=='ACTIVE':self.turning=True
                    if label in ('SUCCEEDED','FAILED'):
                        self.terminal=True;self.result=(state,'LOOK '+label);self.completion=self.get_clock().now().to_msg()
                else:
                    state=CoordinatorStatus.STOPPING;reason='Unusable predictions: LOOK held'
                    if now-self.started>min(seconds(req.look_timeout),self.p('look_timeout_limit')):
                        self.terminal=True;self.result=(CoordinatorStatus.FAILED,'LOOK prediction wait timed out');self.completion=self.get_clock().now().to_msg()
            elif req.mode==req.WAIT:state=CoordinatorStatus.HOLDING if stopped else CoordinatorStatus.STOPPING
            if self.terminal:
                command=Twist();state,reason=self.result or (CoordinatorStatus.FAILED,'terminal without result')
        if self.guard is not None and self.guard.action_type==CollisionMonitorState.STOP and not self.terminal:
            # Do not latch the previous result into the input. Nav2 evaluates
            # this candidate's direction and remains the sole final publisher.
            if self.encounter.state=='IDLE':state=CoordinatorStatus.STOPPING;reason='Collision guard STOP'
        self.pub.publish(command)
        status=CoordinatorStatus();status.header.stamp=self.get_clock().now().to_msg();status.header.frame_id='map'
        if req is not None:
            status.request_id=req.request_id;status.goal_id=req.goal_id;status.mode=req.mode;status.path_version=req.trajectory.path_version;status.map_revision=req.map_revision
        status.status=state;status.stopped=stopped;status.terminal=self.terminal;status.reason=reason
        status.progress=1. if state==CoordinatorStatus.SUCCEEDED else self.progress
        status.remaining_heading_error=error;status.final_heading=heading;status.final_heading_valid=heading_valid
        if self.completion:status.completion_stamp=self.completion
        self.status_pub.publish(status)

    def destroy_node(self):
        if rclpy.ok():
            self.cancel();self.pub.publish(Twist())
        super().destroy_node()


def main():run(CoordinatorNode)
