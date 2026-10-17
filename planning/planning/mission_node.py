"""Saga mission adapter. Installed Nav2 owns path search and recovery."""
from concurrent.futures import ThreadPoolExecutor
import math
import uuid
import numpy as np
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import LaserScan,PointCloud2
from sensor_msgs_py.point_cloud2 import read_points_numpy
from rclpy.qos import qos_profile_sensor_data
from builtin_interfaces.msg import Time as Stamp,Duration
from nav_msgs.msg import Path,OccupancyGrid
from geometry_msgs.msg import Pose
from std_msgs.msg import String,Float64
from vision_msgs.msg import Detection3DArray
from tf2_ros import Buffer,TransformListener,TransformException
from rclpy.time import Time
from interfaces.msg import DockMap,SagaState,Predictions,PerceptionHealth,PlannedTrajectory,TrajectoryPoint,CoordinatorRequest,CoordinatorStatus,Understanding
from .ros_support import AreaNode,seconds,yaw,latched,run
from .saga import SagaClient
from .risk import trajectory_risk,confirmed_look
from .risk_gate import RiskGate
from .trajectory import timed_path
from .nav2_exploration import ExplorationMap
from .explore_adapter import ExploreAdapter
from .dock_occupancy import DockAdmission,dynamic_points,occupied,confirmed_track_points,standoff,costmap_cues


def stamp(t):
    ns=round(t*1e9);return Stamp(sec=ns//10**9,nanosec=ns%10**9)


class MissionNode(AreaNode):
    def __init__(self):
        super().__init__('planning_tree')
        self.buffer=Buffer();self.listener=TransformListener(self.buffer,self)
        self.worker=ThreadPoolExecutor(max_workers=1);self.future=None
        self.client=SagaClient(self.p('saga_url'),self.p('saga_token'),self.p('http_timeout'))
        self.saga=SagaState();self.docks=None;self.status=None;self.predictions=None;self.health=None
        self.docked_at=None;self.leaving=False
        self.request=None;self.goal='';self.phase='approach';self.arrived=False;self.revision=0;self.version=0;self.path=[]
        self.understanding=None
        self.camera_tracks=None;self.occupancy_cues=[];self.source_points={}
        self.create_subscription(LaserScan,'/scan',lambda m:self.on_source('lidar',m),qos_profile_sensor_data)
        self.create_subscription(PointCloud2,'/control/depth_points',lambda m:self.on_source('depth',m),qos_profile_sensor_data)
        self.create_subscription(Detection3DArray,'/perception/camera_tracks',lambda m:setattr(self,'camera_tracks',m),10)
        self.live_costmap=None;self.static_map=None;self.dynamic_obstacles=np.empty((0,2))
        self.admission=DockAdmission(self.p('dock_clear_time'));self.dock_occupied=None;self.standoff_active=False
        self.create_subscription(OccupancyGrid,self.p('global_costmap_topic'),self.on_costmap,latched())
        self.risk_gate=RiskGate(self.p('risk_wait_limit'),self.p('plan_period'),self.p('look_cooldown'),self.p('look_observation_timeout'))
        self.create_subscription(Understanding,'/perception/understanding',lambda m:setattr(self,'understanding',m),10)
        self.exploration_map=None;self.visited=[];self.exploration_target=None;self.exploration_tag=None
        self.scan_index=0;self.tag_retry={};self.tag_look_started=None
        self.explorer=ExploreAdapter(self) if self.p('explore') else None
        self.create_subscription(OccupancyGrid,'/map',self.on_map,latched())
        self.create_subscription(DockMap,'/slam/docks',self.on_docks,latched())
        self.create_subscription(CoordinatorStatus,'/control/status',lambda m:setattr(self,'status',m),10)
        self.create_subscription(Predictions,'/perception/predictions',lambda m:setattr(self,'predictions',m),10)
        self.create_subscription(PerceptionHealth,'/perception/health',lambda m:setattr(self,'health',m),10)
        self.create_subscription(Path,'/plan',self.on_plan,10)
        self.request_pub=self.create_publisher(CoordinatorRequest,'/control/request',10)
        self.saga_pub=self.create_publisher(SagaState,'/planning/saga_state',10)
        self.state_pub=self.create_publisher(String,'/planning/behaviour',10)
        self.path_pub=self.create_publisher(Path,'/planning/path',10)
        self.plan_pub=self.create_publisher(PlannedTrajectory,'/planning/trajectory',10)
        self.risk_pub=self.create_publisher(Float64,'/planning/risk',10)
        self.get_logger().info(f'System Nav2 mission adapter; timed_path_risk_enabled={self.p("risk_check")}')
        self.create_timer(self.p('saga_poll_period'),self.poll)
        self.create_timer(1/self.p('tick_rate'),self.tick)

    def on_docks(self,m):
        if self.docks is None or self.revision!=m.map_revision:
            self.get_logger().info(f'DockMap received: ready={m.ready} revision={m.map_revision}')
            self.request=None;self.phase='approach';self.arrived=False;self.version+=1;self.risk_gate.reset();self.admission.reset();self.standoff_active=False
        self.docks=m;self.revision=m.map_revision

    def poll(self):
        if self.future is not None:
            if not self.future.done():return
            try:
                self.saga=SagaState(**self.future.result(),available=True)
                self.saga.header.frame_id='map';self.saga.header.stamp=stamp(self.now())
            except Exception:pass # Preserve last receipt; stale Saga enters WAIT.
            self.future=None
        if self.p('saga_token'):self.future=self.worker.submit(self.client.poll)
        self.saga_pub.publish(self.saga)

    def on_source(self,source,m):
        try:
            tr=self.buffer.lookup_transform('map',m.header.frame_id,Time.from_msg(m.header.stamp)).transform
            q=tr.rotation;rotation=Rotation.from_quat([q.x,q.y,q.z,q.w]).as_matrix()
            if source=='lidar':
                ranges=np.array(m.ranges);angles=m.angle_min+np.arange(len(ranges))*m.angle_increment
                valid=np.isfinite(ranges)&(ranges>=m.range_min)&(ranges<=self.p('dock_source_range'))
                points=np.c_[ranges[valid]*np.cos(angles[valid]),ranges[valid]*np.sin(angles[valid]),np.zeros(valid.sum())]
            else:
                points=read_points_numpy(m,field_names=['x','y','z'],skip_nans=True)
                base=self.buffer.lookup_transform('base_footprint',m.header.frame_id,Time.from_msg(m.header.stamp)).transform
                b=base.rotation;r=Rotation.from_quat([b.x,b.y,b.z,b.w]).as_matrix();height=(points@r.T)[:,2]+base.translation.z
                points=points[(height>=self.p('dock_depth_min_height'))&(height<=self.p('dock_depth_max_height'))&(np.linalg.norm(points,axis=1)<=self.p('dock_source_range'))]
            xy=(points@rotation.T+np.array([tr.translation.x,tr.translation.y,tr.translation.z]))[:,:2]
            self.source_points[source]=(seconds(m.header.stamp),xy)
        except TransformException:pass # Retained costmap marks remain occupied and are logged unattributed.

    def on_costmap(self,m):
        self.live_costmap=m
        if self.static_map is not None:self.dynamic_obstacles=dynamic_points(m,self.static_map,self.p('dock_static_margin'))

    def on_map(self,m):
        self.static_map=m
        if self.live_costmap is not None:self.dynamic_obstacles=dynamic_points(self.live_costmap,m,self.p('dock_static_margin'))
        self.exploration_map=ExplorationMap(m,self.p('occupied_threshold'))

    def on_plan(self,m):
        self.path=[[p.pose.position.x,p.pose.position.y] for p in m.poses]
        self.path_pub.publish(m)

    def pose(self):
        tr=self.buffer.lookup_transform('map','base_footprint',Time())
        if self.now()-seconds(tr.header.stamp)>self.p('pose_stale_timeout'):raise TransformException('Stale map TF')
        return np.array([tr.transform.translation.x,tr.transform.translation.y,yaw(tr.transform.rotation)])

    def publish_trajectory(self,pose,now):
        own=timed_path(self.path,pose,self.p('speed_limit'),self.p('turn_rate'),self.p('trajectory_dt'),self.p('risk_horizon'))
        m=PlannedTrajectory(goal_id=self.goal,map_revision=self.revision,path_version=self.version)
        m.header.frame_id='map';m.header.stamp=stamp(now)
        for i,row in enumerate(own):m.points.append(TrajectoryPoint(stamp=stamp(now+i*self.p('trajectory_dt')),x=float(row[0]),y=float(row[1]),yaw=float(row[2]),linear_velocity=float(row[3]),angular_velocity=float(row[4])))
        self.plan_pub.publish(m)
        if not self.p('risk_check'):return 0.
        p,h=self.predictions,self.health
        if p is None or h is None or h.status==h.FAILED or p.map_revision!=self.revision or h.map_revision!=self.revision or now-seconds(p.header.stamp)>self.p('prediction_stale_timeout') or now-seconds(h.header.stamp)>self.p('prediction_stale_timeout'):return 1.
        groups=[[(g.probability,[[seconds(x.stamp),x.x,x.y,*x.covariance] for x in g.points]) for g in [*r.groups,r.leftover]] for r in p.robots]
        return trajectory_risk(own[:,:2],now+np.arange(len(own))*self.p('trajectory_dt'),groups,self.p('robot_radius'),self.p('safety_margin'),self.p('risk_samples_power'),self.p('probability_epsilon'),self.p('own_pose_sigma'))

    def issue(self,mode,reason,target=None,heading=None):
        now=self.now();kind=getattr(CoordinatorRequest,mode)
        changed=self.request is None or (self.request.mode,self.request.goal_id,self.request.map_revision,self.request.reason)!=(kind,self.goal,self.revision,reason)
        if changed:
            self.version+=1
            target_message=PlannedTrajectory(goal_id=self.goal,map_revision=self.revision,path_version=self.version)
            target_message.header.frame_id='map';target_message.header.stamp=stamp(now)
            if target is not None:
                target_message.points=[TrajectoryPoint(stamp=stamp(now),x=target.position.x,y=target.position.y,yaw=yaw(target.orientation))]
            self.request=CoordinatorRequest(request_id=str(uuid.uuid4()),goal_id=self.goal,map_revision=self.revision,mode=kind,reason=reason,trajectory=target_message,speed_limit=self.p('speed_limit'))
            self.request.header.frame_id='map';self.request.header.stamp=stamp(now)
            self.request.look_timeout=Duration(sec=int(self.p('look_timeout')))
            if heading is not None:self.request.target_heading=float(heading)
        self.request.valid_until=stamp(now+self.p('request_lease'));self.request_pub.publish(self.request)
        state=self.status.reason if self.status is not None else 'awaiting coordinator'
        self.state_pub.publish(String(data=f'{mode}: {reason}; goal={self.goal}; Nav2={state}; timed_risk={self.p("risk_check")}; dock_occupied={str(self.dock_occupied).lower()}; standoff_active={str(self.standoff_active).lower()}; occupancy_cues={','.join(self.occupancy_cues) or 'none'}'))

    def tick(self):
        now=self.now()
        try:pose=self.pose()
        except TransformException:
            self.issue('WAIT','Localisation unavailable');return
        if self.docks is None or not self.docks.ready:
            if self.p('explore') and self.exploration_map is not None and self.docks is not None:self.explore(pose,now)
            else:self.issue('WAIT','Map/docks not ready; Nav2 mission held')
            return
        if self.explorer and not self.explorer.done:self.explorer.stop()
        manual=self.p('manual_goal');running=self.p('mission_enabled') and (manual>=0 or (self.saga.available and self.saga.event_state=='running' and now-seconds(self.saga.header.stamp)<self.p('saga_stale_timeout')))
        if not running:self.issue('WAIT','Event: hold');return
        dock=manual if manual>=0 else self.saga.next_dock
        if dock<0:self.issue('WAIT','Mission: loading/unloading hold');return
        goal=f'manual:{dock}' if manual>=0 else f'{self.saga.route_id}:{dock}'
        if goal!=self.goal:
            self.goal=goal;self.phase='approach';self.arrived=False;self.request=None;self.path=[];self.risk_gate.reset();self.admission.reset();self.standoff_active=False
        d=self.docks.docks[dock]
        if self.p('opennav_docking') and self.docked_at==dock:self.arrived=True
        if self.p('opennav_docking') and self.docked_at is not None and self.docked_at!=dock:
            self.package_docking(d,dock,pose,now,undock=True);return
        if not self.arrived and self.p('dock_occupancy_enabled') and self.dock_admission(d,pose,now):
            self.publish_trajectory(pose,now);return
        if self.p('opennav_docking'):
            self.package_docking(d,dock,pose,now);return
        target=d.approach_pose.pose if self.phase=='approach' else d.docking_pose.pose
        at=lambda t: math.hypot(pose[0]-t.position.x,pose[1]-t.position.y)<self.p('goal_tolerance') and abs(math.atan2(math.sin(pose[2]-yaw(t.orientation)),math.cos(pose[2]-yaw(t.orientation))))<self.p('dock_heading_tolerance')
        if self.phase=='approach' and at(target):
            self.phase='dock';self.request=None;self.path=[];target=d.docking_pose.pose
        if self.phase=='dock' and at(target):self.arrived=True
        risk=self.publish_trajectory(pose,now);self.risk_pub.publish(Float64(data=risk))
        if self.arrived:self.issue('WAIT','Mission: wait for organiser');return
        if self.p('risk_check') and self.path:
            region=None;u=self.understanding
            if self.p('look') and u is not None and u.goal_id==self.goal and u.map_revision==self.revision and now-seconds(u.header.stamp)<=self.p('prediction_stale_timeout'):
                region=max(u.regions,key=lambda r:r.unconfirmed_mass,default=None)
                if region is not None and region.unconfirmed_mass<self.p('understanding_threshold'):region=None
            heading=math.atan2(region.center.y-pose[1],region.center.x-pose[0]) if region is not None else None
            terminal=None;confirmed=False;completion=None
            if self.request and self.request.mode==CoordinatorRequest.LOOK and self.status and self.status.request_id==self.request.request_id and self.status.terminal:
                terminal=self.status.status==CoordinatorStatus.SUCCEEDED
                completion=seconds(self.status.completion_stamp)
                h=self.health;r=h.last_camera_observation if h and h.has_camera_observation else None
                confirmed=r is not None and confirmed_look(completion,seconds(r.header.stamp),seconds(r.predictions_after_frame),r.processed,r.negative_evidence_applied,r.map_revision==self.revision)
            mode,reason,heading=self.risk_gate.step(now,risk>self.p('risk_tolerance'),heading,terminal,confirmed,completion)
            if mode!='DRIVE':
                self.issue(mode,reason,heading=heading);return

        # A completed/failed Nav2 action can be retried for the still-active Saga goal;
        # recovery choices remain entirely inside Nav2's standard tree.
        if self.request and self.status and self.status.request_id==self.request.request_id and self.status.terminal:self.request=None
        self.issue('DRIVE','Nav2 final docking' if self.phase=='dock' else 'Nav2 approach',target)

    def package_docking(self,d,dock,pose,now,undock=False):
        if not undock and self.docked_at==dock:self.arrived=True
        terminal=self.request is not None and self.status is not None and self.status.request_id==self.request.request_id and self.status.terminal
        if terminal:
            if self.status.status==CoordinatorStatus.SUCCEEDED:
                if undock:self.docked_at=None;self.request=None;return
                self.arrived=True;self.docked_at=dock
            self.request=None  # Retry failed package action while the same Saga target remains active.
        self.publish_trajectory(pose,now)
        if self.arrived:self.issue('WAIT','Mission: wait for organiser');return
        self.issue('DRIVE',f'UndockRobot:{self.docked_at}' if undock else f'DockRobot:{dock}',d.approach_pose.pose)

    def dock_admission(self,dock,pose,now):
        valid=self.live_costmap is not None and self.static_map is not None
        valid=valid and now-seconds(self.live_costmap.header.stamp)<=self.p('costmap_stale_timeout')
        centre=[dock.docking_pose.pose.position.x,dock.docking_pose.pose.position.y]
        self.dock_occupied=occupied(self.dynamic_obstacles,centre,self.p('dock_occupancy_radius')) if valid else None
        self.occupancy_cues=costmap_cues(self.dynamic_obstacles,np.array(centre),self.p('dock_occupancy_radius'),self.source_points,now,self.p('costmap_stale_timeout'),self.live_costmap.info.resolution) if self.dock_occupied else []
        prediction=self.predictions
        if self.p('dock_use_predictions') and prediction is not None and prediction.map_revision==self.revision and now-seconds(prediction.header.stamp)<=self.p('prediction_stale_timeout'):
            support=confirmed_track_points(prediction,self.p('dock_track_max_age'),self.camera_tracks,now,self.p('dock_track_max_sigma'))
            if occupied(support,centre,self.p('dock_occupancy_radius')):self.dock_occupied=True;self.occupancy_cues.append('camera_supported_track')
        allowed=self.admission.update(now,bool(self.dock_occupied),valid)
        if allowed:
            if self.standoff_active:self.request=None;self.phase='approach';self.path=[]
            self.standoff_active=False
            return False
        self.standoff_active=True
        if not valid:
            self.issue('WAIT','Dock admission: live costmap unavailable');return True
        if self.admission.target is None:
            a=dock.approach_pose.pose
            self.admission.target=standoff([a.position.x,a.position.y],yaw(a.orientation),self.p('dock_standoff_back'),self.p('dock_standoff_side'),pose,self.live_costmap,self.p('robot_radius')+self.p('obstacle_clearance'))
        point=self.admission.target
        if point is None:
            self.issue('WAIT','Occupied dock: no clear off-exit standoff');return True
        if np.linalg.norm(pose[:2]-point)<self.p('goal_tolerance'):
            self.issue('WAIT','Occupied dock: off-exit standoff; require continuous clear interval');return True
        target=Pose();target.position.x=float(point[0]);target.position.y=float(point[1]);target.orientation=dock.approach_pose.pose.orientation
        if self.request and self.status and self.status.request_id==self.request.request_id and self.status.terminal:self.request=None
        self.issue('DRIVE','Occupied dock: Nav2 to off-exit standoff',target)
        return True

    def explore(self,pose,now):
        if self.explorer and self.explorer.tick():return
        self.goal='dock-validation'
        terminal=self.request is not None and self.status is not None and self.status.request_id==self.request.request_id and self.status.terminal
        if self.exploration_target is not None and terminal and self.status.status!=CoordinatorStatus.SUCCEEDED:
            self.get_logger().info(f'Exploration goal failed: marking {self.exploration_target.tolist()} and selecting next')
            self.visited.append(self.exploration_target);self.exploration_target=None
            if self.exploration_tag is not None:self.tag_retry[self.exploration_tag]=now+self.p('tag_revisit_cooldown')
            self.exploration_tag=None;self.request=None;self.scan_index=0
        if self.exploration_target is None:
            if self.request is not None and self.request.mode==CoordinatorRequest.LOOK:
                if not terminal:
                    self.issue('LOOK','Map: frontier scan',heading=self.request.target_heading);return
                self.scan_index+=1;self.request=None
            if self.scan_index<4: # Four cardinal views, inherited exploration policy.
                self.issue('LOOK','Map: frontier scan',heading=self.scan_index*math.pi/2);return
            radius=self.p('robot_radius')+self.p('obstacle_clearance')
            candidates=[]
            for d in self.docks.docks:
                if d.valid or seconds(d.last_observed)<=0 or now<self.tag_retry.get(d.dock_id,0):continue
                xy=np.array([d.tag_pose.pose.position.x,d.tag_pose.pose.position.y])
                candidates.append((np.linalg.norm(xy-pose[:2]),d.dock_id,xy))
            for _,ident,xy in sorted(candidates,key=lambda c:c[0]):
                target=self.exploration_map.tag_view(pose,xy,self.visited,radius,self.p('tag_revisit_standoff'),self.p('tag_view_samples'),self.p('frontier_spacing'))
                if target is not None:self.exploration_target=target;self.exploration_tag=ident;break
            if self.exploration_target is None:
                self.exploration_target=self.exploration_map.coverage_view(pose,self.visited,self.p('coverage_wall_distance'),self.p('coverage_spacing'),self.p('frontier_stride'))
                self.exploration_tag=None
            if self.exploration_target is None:
                self.issue('WAIT','Map: dock viewing candidates exhausted');return
            self.request=None;self.tag_look_started=None
        arrived=(terminal and self.status.status==CoordinatorStatus.SUCCEEDED) or np.linalg.norm(pose[:2]-self.exploration_target)<self.p('goal_tolerance')
        if arrived:
            if self.exploration_tag is not None:
                d=self.docks.docks[self.exploration_tag]
                if self.tag_look_started is None:self.tag_look_started=now;self.request=None
                observed=d.valid and now-self.tag_look_started>=self.p('tag_revisit_dwell')
                timeout=now-self.tag_look_started>self.p('tag_revisit_timeout')
                if not observed and not timeout:
                    tag=d.tag_pose.pose.position
                    self.issue('LOOK','Map: repeat dock-tag observation',heading=math.atan2(tag.y-pose[1],tag.x-pose[0]));return
                if timeout:self.tag_retry[self.exploration_tag]=now+self.p('tag_revisit_cooldown')
                self.scan_index=4
            else:self.scan_index=0
            self.visited.append(self.exploration_target);self.exploration_target=None;self.exploration_tag=None;self.request=None
            self.issue('WAIT','Map: observation point reached');return
        target=Pose();target.position.x=float(self.exploration_target[0]);target.position.y=float(self.exploration_target[1])
        if self.exploration_tag is not None:
            tag=self.docks.docks[self.exploration_tag].tag_pose.pose.position
            heading=math.atan2(tag.y-target.position.y,tag.x-target.position.x)
        else:heading=math.atan2(target.position.y-pose[1],target.position.x-pose[0])
        # The target heading is immutable during a lease; the adapter rejects mutations.
        target.orientation.z=math.sin(heading/2);target.orientation.w=math.cos(heading/2)
        self.issue('DRIVE','Map: Nav2 frontier/tag-view goal',target)

    def destroy_node(self):
        self.worker.shutdown(wait=False,cancel_futures=True);super().destroy_node()


def main():run(MissionNode)
