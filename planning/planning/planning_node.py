"""Reactive mission tree and idempotent leased coordinator client. No velocity publisher."""
from concurrent.futures import ThreadPoolExecutor
import math
import time
import uuid
import numpy as np
from builtin_interfaces.msg import Time as Stamp,Duration
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid,Path
from nav2_msgs.msg import CollisionMonitorState
from std_msgs.msg import String,Float64
from std_srvs.srv import Trigger
from tf2_ros import Buffer,TransformListener,TransformException
from rclpy.time import Time
from interfaces.msg import (DockMap,SagaState,Predictions,PerceptionHealth,DockStates,
    Understanding,PlannedTrajectory,TrajectoryPoint,CoordinatorRequest,CoordinatorStatus)
from .ros_support import AreaNode,seconds,yaw,latched,run
from .grid import Grid
from .risk import trajectory_risk,confirmed_look
from .trajectory import timed_path,densify
from .saga import SagaClient
from .behaviour import Situation,decide
from .observation import observation_candidate
from .exploration import frontier_path,tag_view_path


def stamp(t):
    n=max(0,round(t*1e9));return Stamp(sec=n//10**9,nanosec=n%10**9)


class PlanningNode(AreaNode):
    def __init__(self):
        super().__init__('planning_tree')
        self.buffer=Buffer();self.listener=TransformListener(self.buffer,self)
        self.live_grid=None;self.live_stamp=-math.inf;self.recovery_target=None;self.guard_probe_until=-math.inf;self.ready_seen=False
        self.grid=None;self.docks=None;self.saga=SagaState();self.predictions=None;self.health=None;self.dock_states=None;self.understanding=None;self.status=None
        self.guard=None
        self.create_subscription(CollisionMonitorState,'/control/collision_state',lambda m:setattr(self,'guard',m),10)
        self.worker=ThreadPoolExecutor(max_workers=1);self.future=None
        self.client=SagaClient(self.p('saga_url'),self.p('saga_token'),self.p('http_timeout'))
        self.at_dock_latched=False;self.docking_started=False;self.dock_observations=None
        self.create_subscription(DockMap,'/perception/dock_observations',lambda m:setattr(self,'dock_observations',m),10)
        self.path=[];self.goal='';self.version=0;self.revision=0;self.last_plan=-math.inf
        self.visited=[];self.exploration_target=None;self.raw_map=None;self.scan_index=0;self.exploration_look=None;self.exploration_tag=None;self.tag_look_started=None;self.tag_retry={}
        self.request=None;self.last_look=-math.inf;self.look_completion=None;self.wait_since=None
        self.create_subscription(OccupancyGrid,'/map',self.on_map,latched())
        self.create_subscription(OccupancyGrid,self.p('global_costmap_topic'),self.on_costmap,latched())
        self.create_service(Trigger,self.p('encounter_replan_service'),self.replan_encounter)
        self.create_subscription(DockMap,'/slam/docks',self.on_docks,latched())
        for topic,kind,attr in [('/perception/predictions',Predictions,'predictions'),('/perception/health',PerceptionHealth,'health'),('/perception/dock_states',DockStates,'dock_states'),('/perception/understanding',Understanding,'understanding'),('/control/status',CoordinatorStatus,'status')]:
            self.create_subscription(kind,topic,lambda m,a=attr:setattr(self,a,m),10)
        self.request_pub=self.create_publisher(CoordinatorRequest,'/control/request',10)
        self.plan_pub=self.create_publisher(PlannedTrajectory,'/planning/trajectory',10)
        self.path_pub=self.create_publisher(Path,'/planning/path',10)
        self.saga_pub=self.create_publisher(SagaState,'/planning/saga_state',10)
        self.state_pub=self.create_publisher(String,'/planning/behaviour',10)
        self.time_pub=self.create_publisher(Float64,'/planning/time_ms',10)
        self.risk_pub=self.create_publisher(Float64,'/planning/risk',10)
        self.create_timer(self.p('saga_poll_period'),self.poll)
        self.create_timer(1/self.p('tick_rate'),self.tick)

    def on_map(self,m):
        raw=np.array(m.data).reshape(m.info.height,m.info.width);self.raw_map=raw
        self.grid=Grid((raw<0)|(raw>=self.p('occupied_threshold')),m.info.resolution,[m.info.origin.position.x,m.info.origin.position.y])
        self.last_plan=-math.inf

    def on_costmap(self,m):
        if m.header.frame_id!='map':return
        raw=np.array(m.data).reshape(m.info.height,m.info.width)
        self.live_grid=Grid((raw<0)|(raw>=self.p('occupied_threshold')),m.info.resolution,[m.info.origin.position.x,m.info.origin.position.y])
        self.live_stamp=seconds(m.header.stamp)

    def mission_grid(self,now):
        return self.live_grid if now-self.live_stamp<=self.p('costmap_stale_timeout') else None

    def replan_encounter(self,request,response):
        grid=self.mission_grid(self.now())
        try:pose=self.pose()
        except TransformException:
            response.message='No fresh pose';return response
        if grid is None or self.recovery_target is None:
            response.message='No fresh Nav2 global costmap or mission target';return response
        self.path=densify(grid.plan(pose[:2],self.recovery_target,self.p('robot_radius')+self.p('obstacle_clearance')),self.p('path_spacing'))
        self.version+=1;self.last_plan=self.now()
        response.success=bool(self.path);response.message='Live-costmap path ready' if self.path else 'No live-costmap path'
        return response

    def on_docks(self,m):
        if self.docks is None or (self.docks.ready,self.docks.map_revision)!=(m.ready,m.map_revision):
            self.get_logger().info(f'DockMap received: ready={m.ready} revision={m.map_revision}')
        self.docks=m
        if m.map_revision!=self.revision:
            if self.revision:
                self.exploration_target=None;self.exploration_tag=None;self.scan_index=4
            self.at_dock_latched=False
            self.revision=m.map_revision;self.path=[];self.request=None;self.last_plan=-math.inf;self.look_completion=None

    def poll(self):
        if self.future is not None:
            if not self.future.done():return
            try:
                data=self.future.result();self.saga=SagaState(**data,available=True)
                self.saga.header.stamp=stamp(self.now());self.saga.header.frame_id='map'
            except Exception:
                pass # Keep last successful receipt stamp; freshness check below enters WAIT.
            self.future=None
        if self.p('saga_token'): self.future=self.worker.submit(self.client.poll)
        self.saga_pub.publish(self.saga)

    def pose(self):
        transform=self.buffer.lookup_transform('map','base_footprint',Time())
        if self.now()-seconds(transform.header.stamp)>self.p('pose_stale_timeout'):raise TransformException('Stale localisation TF')
        t=transform.transform
        return np.array([t.translation.x,t.translation.y,yaw(t.rotation)])

    def valid_predictions(self,now):
        p,h=self.predictions,self.health
        return p is not None and h is not None and h.status!=h.FAILED and p.map_revision==self.revision and h.map_revision==self.revision and now-seconds(p.header.stamp)<=self.p('prediction_stale_timeout') and now-seconds(h.header.stamp)<=self.p('prediction_stale_timeout')

    def risk(self,own,now):
        if not self.p('risk_check'):return 0.
        if not self.valid_predictions(now):return 1.
        groups=[]
        for robot in self.predictions.robots:
            groups.append([(g.probability,[[seconds(p.stamp),p.x,p.y,*p.covariance] for p in g.points]) for g in [*robot.groups,robot.leftover]])
        times=now+np.arange(len(own))*self.p('trajectory_dt')
        return trajectory_risk(own[:,:2],times,groups,self.p('robot_radius'),self.p('safety_margin'),self.p('risk_samples_power'),self.p('probability_epsilon'),self.p('own_pose_sigma'))

    def issue(self,mode,reason,trajectory,heading=0.):
        now=self.now();kind=getattr(CoordinatorRequest,mode)
        # Identical active requests only extend their lease. A new path or mode gets a new ID.
        changed=self.request is None or self.request.mode!=kind or self.request.goal_id!=self.goal or self.request.map_revision!=self.revision or (kind==CoordinatorRequest.DRIVE and self.request.trajectory.path_version!=trajectory.path_version) or (kind==CoordinatorRequest.WAIT and self.request.reason!=reason) or (self.request is not None and self.request.reason.startswith('ENCOUNTER:')!=reason.startswith('ENCOUNTER:'))
        if changed:
            self.request=CoordinatorRequest(request_id=str(uuid.uuid4()),goal_id=self.goal,map_revision=self.revision,mode=kind,reason=reason,trajectory=trajectory,speed_limit=self.p('speed_limit'),target_heading=heading)
            self.request.header.frame_id='map';self.request.header.stamp=stamp(now)
            self.request.look_timeout=Duration(sec=int(self.p('look_timeout')),nanosec=round(self.p('look_timeout')%1*1e9))
        self.request.valid_until=stamp(now+self.p('request_lease'));self.request_pub.publish(self.request)
        self.state_pub.publish(String(data=f'{mode}: {reason}; goal={self.goal}; provisional=true'))

    def best_region(self,now):
        u=self.understanding
        if u is None or u.goal_id!=self.goal or u.map_revision!=self.revision or now-seconds(u.header.stamp)>self.p('prediction_stale_timeout'):return None
        return max(u.regions,key=lambda r:r.unconfirmed_mass,default=None)

    def tick(self):
        started=time.perf_counter();now=self.now();s=Situation();reason=''
        s.guard=self.guard is not None and self.guard.action_type==CollisionMonitorState.STOP
        try: pose=self.pose()
        except TransformException:
            self.issue('WAIT','Localisation unavailable',PlannedTrajectory());return
        manual=self.p('manual_goal')
        running=self.p('mission_enabled') and (manual>=0 or (self.saga.available and self.saga.event_state=='running' and now-seconds(self.saga.header.stamp)<self.p('saga_stale_timeout')))
        dock=manual if manual>=0 else self.saga.next_dock
        goal=f'manual:{dock}' if manual>=0 else f'{self.saga.route_id}:{dock}'
        if goal!=self.goal and self.docks is not None and self.docks.ready:self.goal=goal;self.at_dock_latched=False;self.docking_started=False;self.path=[];self.last_plan=-math.inf;self.look_completion=None
        s.map_ready=self.grid is not None and self.docks is not None and self.docks.ready
        s.event_running=running;s.next_dock=dock
        if not s.map_ready and self.grid is not None and self.p('explore'):
            self.explore(pose,now);return
        recovery=self.status.reason if self.status is not None else ''
        ready=recovery.startswith('ENCOUNTER READY:')
        if ready and not self.ready_seen:self.guard_probe_until=now+self.p('encounter_drive_probe')
        self.ready_seen=ready
        recovering=recovery.startswith('ENCOUNTER ') and not ready
        target=None;target_heading=pose[2];tag_valid=True
        grid=self.mission_grid(now)
        if s.map_ready and dock>=0:
            landmark=self.docks.docks[dock]
            d=landmark.docking_pose.pose;ap=landmark.approach_pose.pose
            target_heading=yaw(d.orientation)
            s.dock_free=not self.p('risk_check') or (self.dock_states is not None and self.dock_states.map_revision==self.revision and self.dock_states.docks[dock].valid and self.dock_states.docks[dock].physically_free)
            heading_error=abs((target_heading-pose[2]+math.pi)%(2*math.pi)-math.pi)
            if np.linalg.norm(pose[:2]-[ap.position.x,ap.position.y])<self.p('goal_tolerance') and heading_error<self.p('dock_heading_tolerance'):
                if not self.docking_started:self.last_plan=-math.inf
                self.docking_started=True
            if self.docking_started:
                obs=self.dock_observations
                tag_valid=obs is not None and obs.map_revision==self.revision and obs.docks[dock].valid and now-seconds(obs.docks[dock].last_observed)<self.p('dock_observation_timeout')
                if tag_valid:
                    candidate=obs.docks[dock].docking_pose.pose
                    tag_valid=math.hypot(candidate.position.x-d.position.x,candidate.position.y-d.position.y)<self.p('dock_observation_gate')
                    if tag_valid:d=candidate;target_heading=yaw(d.orientation)
                target=np.array([d.position.x,d.position.y])
            else:target=np.array([ap.position.x,ap.position.y])
            s.at_dock=self.docking_started and np.linalg.norm(pose[:2]-target)<self.p('goal_tolerance') and abs((target_heading-pose[2]+math.pi)%(2*math.pi)-math.pi)<self.p('dock_heading_tolerance')
            self.at_dock_latched=self.at_dock_latched or (s.at_dock and tag_valid)
            s.at_dock=self.at_dock_latched
            s.final_approach=self.docking_started
            self.recovery_target=target
            if running and not s.at_dock and not recovering and now-self.last_plan>=self.p('plan_period'):
                path=grid.plan(pose[:2],target,self.p('robot_radius')+self.p('obstacle_clearance')) if grid is not None else []
                if path:
                    region=self.best_region(now)
                    if self.p('observation_curves') and region is not None and region.unconfirmed_mass>=self.p('understanding_threshold'):
                        curve=observation_candidate(grid,path,[region.center.x,region.center.y],self.p('robot_radius')+self.p('obstacle_clearance'),self.p('observation_offset'),self.p('observation_hfov'),self.p('path_spacing'),self.p('speed_limit'),self.p('observation_turn_limit'),pose)
                        if curve is not None:path=curve
                    self.path=densify(path,self.p('path_spacing'));self.version+=1
                else:self.path=[]
                self.last_plan=now
        if running and s.map_ready and dock>=0 and not s.at_dock and (recovering or (s.guard and now>=self.guard_probe_until)):
            self.issue('DRIVE','ENCOUNTER: recovery',PlannedTrajectory(map_revision=self.revision,goal_id=self.goal))
            return
        # Final Collision Monitor handles each proposed direction; a stale STOP
        # cannot veto handing a fresh, clearance-checked path back to Control.
        s.guard=False
        own=timed_path(self.path,pose,self.p('speed_limit'),self.p('turn_rate'),self.p('trajectory_dt'),self.p('risk_horizon'))
        trajectory=PlannedTrajectory(goal_id=self.goal,path_version=self.version,map_revision=self.revision)
        trajectory.header.frame_id='map';trajectory.header.stamp=stamp(now)
        for i,row in enumerate(own):
            trajectory.points.append(TrajectoryPoint(stamp=stamp(now+i*self.p('trajectory_dt')),x=float(row[0]),y=float(row[1]),yaw=float(row[2]),linear_velocity=float(row[3]),angular_velocity=float(row[4])))
        self.plan_pub.publish(trajectory)
        path_msg=Path(header=trajectory.header)
        for i,point in enumerate(self.path):
            p=PoseStamped(header=trajectory.header);p.pose.position.x=float(point[0]);p.pose.position.y=float(point[1])
            angle=math.atan2(self.path[min(i+1,len(self.path)-1)][1]-point[1],self.path[min(i+1,len(self.path)-1)][0]-point[0]) if i<len(self.path)-1 else target_heading
            p.pose.orientation.z=math.sin(angle/2);p.pose.orientation.w=math.cos(angle/2);path_msg.poses.append(p)
        self.path_pub.publish(path_msg)
        s.valid=grid is not None and tag_valid and bool(self.path) and (not self.p('risk_check') or self.valid_predictions(now));s.risk=self.risk(own,now)
        region=self.best_region(now)
        s.look_useful=region is not None and region.unconfirmed_mass>=self.p('understanding_threshold') and now-self.last_look>=self.p('look_cooldown')
        mode,reason=decide(s,self.p('risk_tolerance'),self.p('look'))
        # A LOOK owns the request until terminal status, and successful motion alone never resumes DRIVE.
        if self.request is not None and self.request.mode==CoordinatorRequest.LOOK and self.request.goal_id==self.goal and running:
            status=self.status
            if status is None or status.request_id!=self.request.request_id or not status.terminal:
                mode='LOOK';reason='LOOK in progress'
            elif status.status==CoordinatorStatus.SUCCEEDED:
                completion=seconds(status.completion_stamp);h=self.health;receipt=h.last_camera_observation if h and h.has_camera_observation else None
                confirmed=receipt is not None and confirmed_look(completion,seconds(receipt.header.stamp),seconds(receipt.predictions_after_frame),receipt.processed,receipt.negative_evidence_applied,receipt.map_revision==self.revision)
                if not confirmed:
                    mode='LOOK';reason='LOOK complete: awaiting applied observation'
                    if now-completion>self.p('look_observation_timeout'): mode='WAIT';reason='LOOK observation timed out'
                else:
                    self.last_look=now;self.last_plan=-math.inf
                    mode='WAIT';reason='Observation confirmed: replan before explicit DRIVE'
            else:mode='WAIT';reason='LOOK failed/cancelled';self.last_look=now
        if mode=='LOOK' and (self.request is None or self.request.mode!=CoordinatorRequest.LOOK): self.last_look=now
        if mode=='WAIT' and running and dock>=0 and not s.at_dock:
            if self.wait_since is None:self.wait_since=now
            elif now-self.wait_since>self.p('deadlock_timeout'):self.last_plan=-math.inf;reason='ENCOUNTER: recovery'
        else:self.wait_since=None
        if mode=='WAIT' and running and s.map_ready and dock>=0 and not s.at_dock and not self.path:reason='ENCOUNTER: recovery'
        if reason.startswith('ENCOUNTER:'):mode='DRIVE';trajectory=PlannedTrajectory(map_revision=self.revision,goal_id=self.goal)
        self.issue(mode,reason,trajectory,region.target_heading if region else pose[2])
        self.risk_pub.publish(Float64(data=s.risk));self.time_pub.publish(Float64(data=(time.perf_counter()-started)*1000))

    def explore(self,pose,now):
        self.goal='exploration';trajectory=PlannedTrajectory(goal_id=self.goal,map_revision=self.revision,path_version=self.version)
        trajectory.header.frame_id='map';trajectory.header.stamp=stamp(now)
        terminal=self.status is not None and self.request is not None and self.status.request_id==self.request.request_id and self.status.terminal
        if self.exploration_target is not None and terminal and self.status.status!=CoordinatorStatus.SUCCEEDED:
            self.visited.append(self.exploration_target)
            if self.exploration_tag is not None:self.tag_retry[self.exploration_tag]=now+self.p('tag_revisit_cooldown')
            self.exploration_target=None;self.exploration_tag=None;self.request=None;self.scan_index=0
        # Once a tag is known, visit its estimated approach pose for repeat calibrated
        # observations. This uses SLAM output only, never Gazebo dock coordinates.
        if self.exploration_target is None:
            if self.request is not None and self.request.mode==CoordinatorRequest.LOOK:
                if not terminal:
                    self.issue('LOOK','Map: frontier scan',trajectory,self.request.target_heading);return
                self.scan_index+=1;self.request=None
            path=[]
            if self.scan_index>=4:
                candidates=[]
                if self.docks is not None:
                    for d in self.docks.docks:
                        if d.valid or seconds(d.last_observed)<=0 or now<self.tag_retry.get(d.dock_id,0):continue
                        point=d.tag_pose.pose.position;xy=np.array([point.x,point.y])
                        candidates.append((float(np.linalg.norm(xy-pose[:2])),d.dock_id,xy))
                for _,ident,xy in sorted(candidates,key=lambda x:x[0]):
                    path=tag_view_path(self.grid,pose,xy,self.p('robot_radius')+self.p('obstacle_clearance'),self.p('tag_revisit_standoff'),self.p('tag_view_samples'))
                    if path:self.exploration_tag=ident;break
            if not path and self.scan_index<4:
                self.issue('LOOK','Map: frontier scan',trajectory,self.scan_index*math.pi/2);return
            if not path:
                self.exploration_tag=None
                path=frontier_path(self.raw_map,self.grid,pose,self.visited,self.p('robot_radius')+self.p('obstacle_clearance'),self.p('frontier_band'),self.p('frontier_spacing'),self.p('frontier_stride'))
            if not path:
                self.issue('WAIT','Map: no reachable unexplored frontier',trajectory);return
            self.path=densify(path,self.p('path_spacing'));self.exploration_target=np.array(path[-1]);self.version+=1;self.tag_look_started=None
        if np.linalg.norm(pose[:2]-self.exploration_target)<self.p('goal_tolerance'):
            if self.exploration_tag is not None:
                d=self.docks.docks[self.exploration_tag]
                tag=d.tag_pose.pose.position;heading=math.atan2(tag.y-pose[1],tag.x-pose[0])
                if self.request is None or self.request.mode!=CoordinatorRequest.LOOK:
                    self.request=None;self.tag_look_started=now
                observed=d.valid and now-self.tag_look_started>=self.p('tag_revisit_dwell')
                timeout=now-self.tag_look_started>self.p('tag_revisit_timeout')
                if not observed and not timeout:
                    self.issue('LOOK','Map: repeat dock-tag observation',trajectory,heading);return
                if timeout:self.tag_retry[self.exploration_tag]=now+self.p('tag_revisit_cooldown')
                self.scan_index=4 # Known incomplete tags get priority before another full sweep.
            else:self.scan_index=0
            self.visited.append(self.exploration_target);self.exploration_target=None;self.exploration_tag=None;self.request=None
            self.issue('WAIT','Map: observation point reached',trajectory);return
        trajectory.path_version=self.version
        path_msg=Path(header=trajectory.header)
        for i,point in enumerate(self.path):
            p=PoseStamped(header=trajectory.header);p.pose.position.x=float(point[0]);p.pose.position.y=float(point[1])
            delta=np.asarray(self.path[min(i+1,len(self.path)-1)])-point
            heading=math.atan2(delta[1],delta[0]) if i<len(self.path)-1 else pose[2]
            if self.exploration_tag is not None and i==len(self.path)-1:
                tag=self.docks.docks[self.exploration_tag].tag_pose.pose.position;heading=math.atan2(tag.y-point[1],tag.x-point[0])
            p.pose.orientation.z=math.sin(heading/2);p.pose.orientation.w=math.cos(heading/2);path_msg.poses.append(p)
        self.path_pub.publish(path_msg)
        self.issue('DRIVE','Map: explore or revisit incomplete tag',trajectory)

    def destroy_node(self):
        self.worker.shutdown(wait=False,cancel_futures=True);super().destroy_node()


def main():run(PlanningNode)
