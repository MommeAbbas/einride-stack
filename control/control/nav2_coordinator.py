"""Thin leased NavigateToPose / Spin adapter. Nav2 owns planning and recoveries."""
import math
import rclpy
from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import PoseStamped,Twist
from std_msgs.msg import Bool
from nav2_msgs.action import NavigateToPose,Spin,DockRobot,UndockRobot
from nav2_msgs.msg import SpeedLimit,BehaviorTreeLog
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from tf2_ros import Buffer,TransformListener,TransformException
from interfaces.msg import CoordinatorRequest,CoordinatorStatus,Predictions,PerceptionHealth,DockMap
from .ros_support import AreaNode,seconds,yaw,latched,run
from .coordinator import validate,wrap
from .hardware_motion import hardware_command


class Nav2Coordinator(AreaNode):
    def __init__(self):
        super().__init__('control_coordinator')
        self.buffer=Buffer();self.listener=TransformListener(self.buffer,self)
        self.navigate=ActionClient(self,NavigateToPose,'/navigate_to_pose');self.spin=ActionClient(self,Spin,'/spin')
        self.dock_client=ActionClient(self,DockRobot,'/dock_robot');self.undock_client=ActionClient(self,UndockRobot,'/undock_robot')
        self.mapping_active=self.p('hardware_mapping_passthrough')
        self.create_subscription(Bool,'/slam/mapping_complete',lambda m:setattr(self,'mapping_active',False) if m.data else None,latched())
        self.request=None;self.revision=None;self.odom=None;self.predictions=None;self.health=None
        self.goal_handle=None;self.pending=None;self.generation=0;self.cancel_pending=[];self.raw=None;self.raw_time=-math.inf
        self.terminal=False;self.result=None;self.completion=None;self.action_result=None;self.progress=0.;self.remaining=0.;self.initial_distance=None;self.bt_state=''
        self.create_subscription(CoordinatorRequest,'/control/request',self.on_request,10)
        self.create_subscription(Odometry,'/odom',lambda m:setattr(self,'odom',m),qos_profile_sensor_data)
        self.create_subscription(DockMap,'/slam/docks',lambda m:setattr(self,'revision',m.map_revision),latched())
        self.create_subscription(Predictions,'/perception/predictions',lambda m:setattr(self,'predictions',m),10)
        self.create_subscription(PerceptionHealth,'/perception/health',lambda m:setattr(self,'health',m),10)
        # Both Nav2 command producers are mutually exclusive in its tree; LOOK cancels navigation first.
        self.create_subscription(Twist,'/control/nav2_cmd',self.command,10)
        self.create_subscription(BehaviorTreeLog,'/behavior_tree_log',self.tree,10)
        self.pub=self.create_publisher(Twist,'/control/guard_input',10)
        self.status_pub=self.create_publisher(CoordinatorStatus,'/control/status',10)
        self.limit_pub=self.create_publisher(SpeedLimit,'/speed_limit',10)
        self.create_timer(1/self.p('coordinator_rate'),self.tick)

    def command(self,m):
        if self.goal_handle is not None or self.mapping_active:self.raw=m;self.raw_time=self.now()
        # Decision/status timer may be 5Hz; never decimate Nav2's active controller.
        # Recheck the lease and sensor freshness on every event before the guard.
        self.pub.publish(self.current_command())

    def current_command(self):
        out=Twist();now=self.now();req=self.request
        if self.raw is None or now-self.raw_time>=self.p('command_timeout'):return out
        if self.mapping_active:
            out.linear.x,out.angular.z=hardware_command(self.raw.linear.x,self.raw.angular.z,
                self.p('direct_forward_limit'),self.p('reverse_speed_limit'),self.p('navigation_turn_limit'),
                self.p('minimum_linear_command'),self.p('minimum_angular_command'),
                self.p('linear_compensation_threshold'),self.p('angular_compensation_threshold'))
            return out
        if (req is None or req.mode==req.WAIT or self.terminal or self.action_result is not None
            or self.goal_handle is None or self.cancel_pending or seconds(req.valid_until)<=now
            or req.map_revision!=self.revision or self.odom is None
            or now-seconds(self.odom.header.stamp)>self.p('odom_stale_timeout')
            or not self.predictions_valid()):return out
        try:
            tr=self.buffer.lookup_transform('map','base_footprint',Time())
            if now-seconds(tr.header.stamp)>self.p('pose_stale_timeout'):return out
        except TransformException:return out
        angular=self.p('look_turn_rate') if req.mode==req.LOOK else self.p('navigation_turn_limit')
        out.linear.x=max(-self.p('reverse_speed_limit'),min(req.speed_limit,self.raw.linear.x)) if req.mode==req.DRIVE else 0.
        out.angular.z=max(-angular,min(angular,self.raw.angular.z))
        if self.p('hardware_minimum_commands'):
            out.linear.x,out.angular.z=hardware_command(out.linear.x,out.angular.z,req.speed_limit,
                self.p('reverse_speed_limit'),angular,self.p('minimum_linear_command'),self.p('minimum_angular_command'),
                self.p('linear_compensation_threshold'),self.p('angular_compensation_threshold'))
        return out

    def tree(self,m):
        active=[x.node_name for x in m.event_log if x.current_status=='RUNNING']
        if active:self.bt_state=', '.join(active)

    def cancel(self):
        self.generation+=1;self.raw=None
        if self.goal_handle is not None:
            self.cancel_pending.append(self.goal_handle.get_result_async())
            self.goal_handle.cancel_goal_async()
        if self.pending is not None:self.cancel_pending.append(self.pending)
        self.goal_handle=None;self.pending=None

    def on_request(self,m):
        error=validate(m.mode,m.header.frame_id,seconds(m.valid_until),self.now(),m.goal_id,m.map_revision,m.trajectory.map_revision,m.speed_limit)
        if m.mode==m.DRIVE and (not m.trajectory.points or not all(math.isfinite(v) for p in m.trajectory.points for v in (p.x,p.y,p.yaw))):error='DRIVE target missing/nonfinite'
        if self.request is not None and m.request_id==self.request.request_id:
            old=self.request
            if (m.mode,m.goal_id,m.map_revision,m.trajectory,m.target_heading,m.look_timeout,m.speed_limit)!=(old.mode,old.goal_id,old.map_revision,old.trajectory,old.target_heading,old.look_timeout,old.speed_limit):error='request ID mutated'
            if not error:
                if seconds(m.valid_until)>seconds(old.valid_until):self.request.valid_until=m.valid_until
                return
        self.cancel();self.request=m;self.terminal=bool(error);self.result=(CoordinatorStatus.REJECTED,error) if error else None
        self.completion=self.get_clock().now().to_msg() if error else None;self.action_result=None;self.progress=0.;self.initial_distance=None;self.bt_state=''

    def predictions_valid(self):
        if not self.p('require_predictions'):return True
        p,h=self.predictions,self.health;now=self.now()
        if p is None or h is None or h.status==h.FAILED or p.map_revision!=self.revision or h.map_revision!=self.revision:return False
        if now-seconds(p.header.stamp)>self.p('prediction_stale_timeout') or now-seconds(h.header.stamp)>self.p('prediction_stale_timeout'):return False
        return seconds(p.header.stamp)+seconds(p.horizon)>=now+self.p('mppi_horizon')

    def send(self,client,goal):
        if not client.server_is_ready():return
        generation=self.generation
        def feedback(m):
            if generation!=self.generation:return
            f=m.feedback
            if hasattr(f,'distance_remaining'):
                if self.initial_distance is None:self.initial_distance=f.distance_remaining
                self.progress=1.-min(1.,f.distance_remaining/self.initial_distance) if self.initial_distance else 0.
            elif hasattr(f,'angular_distance_traveled'):self.progress=abs(f.angular_distance_traveled)
        self.pending=client.send_goal_async(goal,feedback_callback=feedback)
        def accepted(f):
            try:handle=f.result()
            except Exception:
                if generation==self.generation:self.action_result=GoalStatus.STATUS_ABORTED;self.pending=None
                return
            if generation!=self.generation:
                if handle.accepted:
                    self.cancel_pending.append(handle.get_result_async());handle.cancel_goal_async()
                return
            self.pending=None
            if not handle.accepted:self.action_result=GoalStatus.STATUS_ABORTED;return
            self.goal_handle=handle
            def completed(f):
                if generation!=self.generation:return
                self.goal_handle=None;self.raw=None
                try:self.action_result=f.result().status
                except Exception:self.action_result=GoalStatus.STATUS_ABORTED
            handle.get_result_async().add_done_callback(completed)
        self.pending.add_done_callback(accepted)

    def tick(self):
        now=self.now();req=self.request;command=Twist();heading=0.;valid_tf=False
        if self.mapping_active:
            if self.raw is not None and now-self.raw_time<self.p('command_timeout'):
                command.linear.x,command.angular.z=hardware_command(self.raw.linear.x,self.raw.angular.z,
                    self.p('direct_forward_limit'),self.p('reverse_speed_limit'),self.p('navigation_turn_limit'),
                    self.p('minimum_linear_command'),self.p('minimum_angular_command'),
                    self.p('linear_compensation_threshold'),self.p('angular_compensation_threshold'))
            self.pub.publish(command)
            return  # Mapping manager alone owns goals; mission WAIT cannot cancel its Nav2 action.
        try:
            tr=self.buffer.lookup_transform('map','base_footprint',Time());heading=yaw(tr.transform.rotation)
            valid_tf=now-seconds(tr.header.stamp)<=self.p('pose_stale_timeout')
        except TransformException:pass
        fresh=self.odom is not None and now-seconds(self.odom.header.stamp)<=self.p('odom_stale_timeout')
        stopped=fresh and abs(self.odom.twist.twist.linear.x)<self.p('stopped_speed') and abs(self.odom.twist.twist.angular.z)<self.p('stopped_turn_rate')
        self.cancel_pending=[f for f in self.cancel_pending if not f.done()]
        state=CoordinatorStatus.HOLDING;reason='No request';self.remaining=0.
        if req is not None:
            reason=req.reason
            valid=seconds(req.valid_until)>now and (req.mode==req.WAIT or (valid_tf and fresh and req.map_revision==self.revision))
            if not valid or (req.mode!=req.WAIT and not self.predictions_valid()):
                if self.goal_handle is not None or self.pending is not None:self.cancel()
                state=CoordinatorStatus.STOPPING;reason='WAIT: invalid lease/localisation/odometry/predictions'
            elif req.mode==req.WAIT:state=CoordinatorStatus.HOLDING if stopped else CoordinatorStatus.STOPPING
            elif self.terminal:state,reason=self.result
            else:
                if req.mode==req.LOOK:self.remaining=wrap(req.target_heading-heading-self.p('camera_yaw_offset'))
                if self.action_result is not None:
                    if self.action_result!=GoalStatus.STATUS_SUCCEEDED:
                        self.terminal=True;self.result=(CoordinatorStatus.FAILED,f'Nav2 action result={self.action_result}')
                    elif stopped:
                        ok=req.mode!=req.LOOK or abs(self.remaining)<=self.p('heading_tolerance')
                        self.terminal=True;self.result=(CoordinatorStatus.SUCCEEDED if ok else CoordinatorStatus.FAILED,'Nav2 action completed; measured stopped' if ok else 'Spin final heading outside tolerance')
                    if self.terminal:self.completion=self.get_clock().now().to_msg();state,reason=self.result
                    else:state=CoordinatorStatus.STOPPING;reason='Nav2 action complete; awaiting measured stop'
                elif self.goal_handle is None and self.pending is None and not self.cancel_pending:
                    if req.mode==req.DRIVE:
                        point=req.trajectory.points[-1];pose=PoseStamped();pose.header.frame_id='map';pose.header.stamp=self.get_clock().now().to_msg()
                        pose.pose.position.x=point.x;pose.pose.position.y=point.y;pose.pose.orientation.z=math.sin(point.yaw/2);pose.pose.orientation.w=math.cos(point.yaw/2)
                        self.limit_pub.publish(SpeedLimit(speed_limit=req.speed_limit,percentage=False))
                        if self.p('opennav_docking') and req.reason.startswith('DockRobot:'):
                            self.send(self.dock_client,DockRobot.Goal(use_dock_id=True,dock_id=req.reason.split(':')[1]))
                        elif self.p('opennav_docking') and req.reason.startswith('UndockRobot:'):
                            self.send(self.undock_client,UndockRobot.Goal(dock_type='dock_'+req.reason.split(':')[1]))
                        else:self.send(self.navigate,NavigateToPose.Goal(pose=pose))
                    elif stopped:
                        allowance=min(seconds(req.look_timeout),self.p('look_timeout_limit'));ns=round(allowance*1e9)
                        self.send(self.spin,Spin.Goal(target_yaw=self.remaining,time_allowance=Duration(sec=ns//10**9,nanosec=ns%10**9)))
                    state=CoordinatorStatus.ACKNOWLEDGED;reason='Nav2 goal submission / awaiting measured stop'
                else:
                    state=CoordinatorStatus.ACTIVE
                    reason='Nav2 Spin' if req.mode==req.LOOK else 'Nav2 '+req.reason if req.reason.startswith(('DockRobot:','UndockRobot:')) else 'Nav2 NavigateToPose: '+self.bt_state
                    if self.goal_handle is not None and self.raw is not None and now-self.raw_time<self.p('command_timeout'):
                        command.linear.x=max(-self.p('reverse_speed_limit'),min(req.speed_limit,self.raw.linear.x)) if req.mode==req.DRIVE else 0.
                        angular=self.p('look_turn_rate') if req.mode==req.LOOK else self.p('navigation_turn_limit')
                        command.angular.z=max(-angular,min(angular,self.raw.angular.z))
                        if self.p('hardware_minimum_commands'):
                            command.linear.x,command.angular.z=hardware_command(
                                command.linear.x,command.angular.z,req.speed_limit,
                                self.p('reverse_speed_limit'),angular,
                                self.p('minimum_linear_command'),self.p('minimum_angular_command'),
                    self.p('linear_compensation_threshold'),self.p('angular_compensation_threshold'))
        self.pub.publish(command)
        status=CoordinatorStatus(status=state,reason=reason,stopped=stopped,terminal=self.terminal,progress=max(0.,min(1.,self.progress)),remaining_heading_error=self.remaining,final_heading=heading+self.p('camera_yaw_offset'),final_heading_valid=valid_tf)
        status.header.frame_id='map';status.header.stamp=self.get_clock().now().to_msg()
        if req:
            status.request_id=req.request_id;status.goal_id=req.goal_id;status.map_revision=req.map_revision;status.path_version=req.trajectory.path_version;status.mode=req.mode
        if self.completion:status.completion_stamp=self.completion
        self.status_pub.publish(status)

    def destroy_node(self):
        if rclpy.ok(context=self.context):
            self.cancel();self.pub.publish(Twist())
        super().destroy_node()


def main():run(Nav2Coordinator)
