"""Command arbitration for specified one-track escape; Nav2 retains its goal."""
import math
from pathlib import Path
import numpy as np
import yaml
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.parameter import Parameter
from rclpy.parameter_client import AsyncParameterClient
from rclpy.qos import qos_profile_sensor_data
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool,String
from tf2_ros import Buffer,TransformListener,TransformException
from .ros_support import seconds,yaw,run
from .escape import pivot,clearance,front_clearance,swept_clear


class EscapeNode(Node):
    def __init__(self):
        super().__init__('control_escape')
        config=Path(get_package_share_directory('control'))/'config/escape.yaml'
        for k,v in yaml.safe_load(config.read_text())['/**']['ros__parameters'].items():self.declare_parameter(k,v)
        if any(not math.isfinite(self.p(k)) or self.p(k)<=0 for k in yaml.safe_load(config.read_text())['/**']['ros__parameters']):raise ValueError('Escape parameters must be finite and positive')
        if self.p('escape_min_valid_fraction')>1:raise ValueError('Invalid observed scan fraction')
        self.state='NORMAL';self.started=0.;self.motion_since=None
        self.preheld=False;self.prehold_latched=False
        self.held_seconds=0.;self.hold_anchor=None;self.hold_tick=None;self.was_held=False
        self.last_veto=None;self.scan_reason="scan unavailable"
        self.normal=Twist();self.normal_time=-math.inf;self.final=Twist();self.final_time=-math.inf
        self.scan=None;self.odom=None;self.active=False;self.active_time=-math.inf;self.paused=False;self.pause_time=-math.inf
        self.change=None;self.approach_disabled=False;self.side=1.;self.pivot_yaw=0.;self.last_yaw=None
        self.buffer=Buffer();self.listener=TransformListener(self.buffer,self)
        self.guard=AsyncParameterClient(self,'/collision_monitor')
        self.pub=self.create_publisher(Twist,'/control/guard_input',1)
        self.hold=self.create_publisher(Bool,'/control/escape_active',1)
        self.status=self.create_publisher(String,'/control/escape_state',10)
        self.create_subscription(Twist,'/control/normal_cmd',self.normal_command,1)
        self.create_subscription(Twist,'/cmd_vel',self.final_command,1)
        self.create_subscription(LaserScan,'/scan',lambda m:setattr(self,'scan',m),qos_profile_sensor_data)
        self.create_subscription(Odometry,'/odom',self.odometry,qos_profile_sensor_data)
        self.create_subscription(Bool,'/control/navigation_active',self.navigation,1)
        self.create_subscription(Bool,'/control/navigation_paused',self.pause,1)
        self.create_timer(1/self.p('escape_rate'),self.tick)

    def p(self,k):return self.get_parameter(k).value
    def now(self):return self.get_clock().now().nanoseconds/1e9
    def navigation(self,m):self.active=m.data;self.active_time=self.now()
    def pause(self,m):self.paused=m.data;self.pause_time=self.now()
    def final_command(self,m):self.final=m;self.final_time=self.now()
    def normal_command(self,m):
        self.normal=m;self.normal_time=self.now()
        if self.state=='NORMAL':
            active=self.active and self.now()-self.active_time<self.p('escape_heartbeat_timeout')
            held=False
            if active and (m.linear.x!=0. or m.angular.z!=0.):
                points=self.points()
                held=points is None or not swept_clear(points,m.linear.x,m.angular.z,
                    self.p('escape_horizon'),self.p('escape_step'),self.p('escape_stop_half_length'),
                    self.p('escape_stop_half_width'),self.p('escape_margin'))
            if held and not self.preheld:
                reason=self.scan_reason if points is None else 'swept Stop envelope obstructed'
                self.status.publish(String(data='HOLD: '+reason))
                self.get_logger().info(f'escape vetoed [PRESTOP]: {reason}; held_credit={self.held_seconds:.2f}s')
            self.preheld=held
            if held:self.prehold_latched=True
            self.pub.publish(Twist() if held else m)
    def odometry(self,m):
        a=yaw(m.pose.pose.orientation)
        if self.last_yaw is not None and self.state=='PIVOT':self.pivot_yaw+=math.atan2(math.sin(a-self.last_yaw),math.cos(a-self.last_yaw))
        self.last_yaw=a;self.odom=m
    def transition(self,state,reason):
        self.state=state;self.motion_since=None;self.last_veto=None
        label={'ARM':'triggered','PIVOT':'pivot','BACKUP':'reverse','CREEP':'creep','NORMAL':'finished'}.get(state,state.lower())
        self.status.publish(String(data=f'{state}: {reason}'));self.get_logger().info(f'escape {label} [{state}]: {reason}')
        if state=='PIVOT':self.pivot_yaw=0.
        if state=='BACKUP':self.backup_start=(self.odom.pose.pose.position.x,self.odom.pose.pose.position.y)
    def veto(self,reason):
        key=(self.state,reason)
        if key!=self.last_veto:
            self.last_veto=key
            text=f'escape vetoed [{self.state}]: {reason}'
            self.get_logger().info(text);self.status.publish(String(data='VETOED: '+text))

    def held_motion(self,now,fresh,alive,desired,zero):
        # A coverage replacement changes goal identity, not the physical trap.
        if fresh:
            p=self.odom.pose.pose.position;position=(p.x,p.y)
            if self.hold_anchor is None:self.hold_anchor=position
            elif math.dist(position,self.hold_anchor)>self.p('escape_intent_reset_distance'):
                self.hold_anchor=position;self.held_seconds=0.;self.prehold_latched=False;self.was_held=False
        held=fresh and alive and (self.prehold_latched or (desired and zero and self.stationary()))
        if held and self.was_held and self.hold_tick is not None:
            delta=now-self.hold_tick
            if 0<=delta<=self.p('escape_heartbeat_timeout'):self.held_seconds+=delta
        self.hold_tick=now;self.was_held=held
        return held and self.held_seconds>=self.p('escape_trigger')

    def set_approach(self,enabled):
        if self.change is not None:
            if not self.change.done():return False
            try:ok=all(r.successful for r in self.change.result().results)
            except Exception:ok=False
            self.change=None
            if ok:self.approach_disabled=not enabled;return True
        if self.guard.services_are_ready():self.change=self.guard.set_parameters([Parameter('Approach.enabled',value=enabled)])
        return False
    def points(self):
        self.scan_reason='scan missing or stale'
        if self.scan is None or not 0<=self.now()-seconds(self.scan.header.stamp)<=self.p('escape_scan_timeout'):return None
        m=self.scan;r=np.asarray(m.ranges);angles=m.angle_min+np.arange(len(r))*m.angle_increment
        good=np.isfinite(r)&(r>=m.range_min)&(r<=m.range_max)
        if not len(r) or np.mean(good)<self.p('escape_min_valid_fraction'):
            self.scan_reason='insufficient finite lidar returns';return None
        try:tr=self.buffer.lookup_transform_full('base_footprint',Time(),m.header.frame_id,Time(seconds=seconds(m.header.stamp)),'odom').transform
        except TransformException as error:
            self.scan_reason=f'scan transform unavailable: {type(error).__name__}';return None
        a=yaw(tr.rotation);x=r[good]*np.cos(angles[good]);y=r[good]*np.sin(angles[good])
        return np.c_[np.cos(a)*x-np.sin(a)*y+tr.translation.x,np.sin(a)*x+np.cos(a)*y+tr.translation.y]
    def stationary(self):
        v=self.odom.twist.twist
        return abs(v.linear.x)<self.p('escape_motion_linear') and abs(v.angular.z)<self.p('escape_motion_angular')
    def safe(self,points,v,w):
        return swept_clear(points,v,w,self.p('escape_horizon'),self.p('escape_step'),self.p('escape_half_length'),self.p('escape_half_width'),self.p('escape_margin'))
    def tick(self):
        busy=self.state!='NORMAL';self.command_out=Twist()
        self.step()
        if busy or self.state!='NORMAL':self.pub.publish(self.command_out)

    def step(self):
        now=self.now();alive=self.active and now-self.active_time<self.p('escape_heartbeat_timeout')
        fresh=self.odom is not None and 0<=now-seconds(self.odom.header.stamp)<self.p('escape_odom_timeout')
        self.hold.publish(Bool(data=self.state!='NORMAL'))
        if self.state=='NORMAL':
            if now-self.normal_time>=self.p('escape_command_timeout'):self.pub.publish(Twist())
            desired=now-self.normal_time<self.p('escape_command_timeout') and (abs(self.normal.linear.x)>self.p('escape_motion_linear') or abs(self.normal.angular.z)>self.p('escape_motion_angular'))
            zero=now-self.final_time>self.p('escape_command_timeout') or (abs(self.final.linear.x)<self.p('escape_motion_linear') and abs(self.final.angular.z)<self.p('escape_motion_angular'))
            if self.held_motion(now,fresh,alive,desired,zero):
                self.started=now;self.transition('ARM',f'{self.held_seconds:.2f}s accumulated blocked navigation; pre-stop={self.prehold_latched}')
                self.pub.publish(Twist());self.hold.publish(Bool(data=True))
            return
        if self.state=='RESTORE':
            if self.set_approach(True):
                self.held_seconds=0.;self.hold_anchor=None;self.was_held=False;self.prehold_latched=False;self.transition('NORMAL','Approach restored; Nav2 replan');self.hold.publish(Bool(data=False))
            else:self.veto('waiting for Approach restore acknowledgement')
            return
        if not alive or now-self.started>self.p('escape_sequence_timeout'):
            if self.change is not None and not self.change.done():return
            self.change=None;self.transition('RESTORE','goal revoked or sequence timeout');return
        if not fresh:self.veto("odometry missing or stale");return
        points=self.points()
        if points is None:self.veto(self.scan_reason);return
        half_l=self.p('escape_half_length');half_w=self.p('escape_half_width')
        front=front_clearance(points,half_l,half_w,self.p('escape_front_half_angle'))
        if front is None:self.veto("no observed front-sector points");return
        if self.state=='ARM':
            if not self.paused or now-self.pause_time>=self.p('escape_heartbeat_timeout'):self.veto('waiting for Nav2 pause acknowledgement');return
            if not self.set_approach(False):self.veto('waiting for Approach disable acknowledgement');return
            near=points[np.argmin(clearance(points,half_l,half_w))]
            self.side=1. if near[1]>=0 else -1.
            self.transition('PIVOT',f'obstacle side={self.side:+.0f}; one track reverse');return
        if self.state=='PIVOT':
            if front>=self.p('escape_pivot_clearance') or abs(self.pivot_yaw)>=self.p('escape_max_yaw'):
                if front>=self.p('escape_creep_front'):self.transition('CREEP','front >=0.5m')
                else:self.transition('BACKUP','pivot complete but creep blocked; reverse then retry pivot')
                return
            v,w=pivot(self.side,self.p('escape_wheel_speed'),self.p('escape_track'))
        elif self.state=='BACKUP':
            p=self.odom.pose.pose.position
            if math.dist((p.x,p.y),self.backup_start)>=self.p('escape_backup_distance'):
                self.transition('PIVOT','measured 0.15m backup complete');return
            v,w=-self.p('escape_wheel_speed'),0.
        elif self.state=='CREEP':
            if np.min(clearance(points,half_l,half_w))>=self.p('escape_clearance'):
                self.transition('RESTORE','all round >=0.35m');return
            if front<self.p('escape_creep_front'):
                self.transition('RESTORE','front no longer clear; replan');return
            v,w=self.p('escape_wheel_speed'),0.
        else:return
        stopped=self.stationary()
        if not stopped:self.motion_since=None
        elif self.motion_since is None:self.motion_since=now
        blocked=self.motion_since is not None and now-self.motion_since>=self.p('escape_stall_timeout')
        safe=self.safe(points,v,w)
        if not safe or blocked:
            self.veto('lidar swept footprint obstructed' if not safe else 'measured no motion despite command; downstream guard or chassis stopped')
            if self.state=='PIVOT':self.transition('BACKUP','pivot swept path or hard guard blocked')
            elif blocked:self.transition('RESTORE','escape blocked; restore guard and replan')
            return
        self.last_veto=None
        out=Twist();out.linear.x=v;out.angular.z=w;self.command_out=out

    def destroy_node(self):
        if rclpy.ok(context=self.context):self.pub.publish(Twist())
        super().destroy_node()


def main():run(EscapeNode)
