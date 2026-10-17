"""Lidar exploration then camera next-best-view; all paths/actions belong to Nav2."""
import json,math
from pathlib import Path
import numpy as np
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.time import Time
from rclpy.action import ActionServer,ActionClient,GoalResponse,CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.task import Future
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid,Odometry
from nav2_msgs.action import NavigateToPose,ComputePathToPose
from interfaces.msg import DockMap
from explore_lite_msgs.msg import ExploreStatus
from std_msgs.msg import Bool,String
from tf2_ros import Buffer,TransformListener,TransformException
from .camera_coverage import WallCoverage,validation_hold
from .nav2_exploration import ExplorationMap
from .ros_support import latched,run,seconds,yaw


class CoverageNode(Node):
    def __init__(self):
        super().__init__('mapping_coverage')
        source=Path(get_package_share_directory('planning'))/'config/coverage.yaml'
        for key,value in yaml.safe_load(source.read_text())['/**']['ros__parameters'].items():
            if key!='use_sim_time':self.declare_parameter(key,value)
        self.p={k:self.get_parameter(k).value for k in self._parameters if k!='use_sim_time'}
        self.buffer=Buffer();self.listener=TransformListener(self.buffer,self)
        self.world=None;self.live=None;self.docks=None;self.odom=None;self.start=None
        self.phase='LIDAR';self.remaining=True;self.paused=False;self.cooldown=[];self.goal=None;self.goal_pending=None
        self.frontier=None;self.frontier_backend=None;self.frontier_pending=None;self.retiring=[]
        self.candidates=[];self.reachable=[];self.path_pending=None;self.hold_since=None;self.hold_frames=0;self.last_hold_frame=-math.inf;self.target=None;self.target_docks=[]
        self.nav=ActionClient(self,NavigateToPose,'/navigate_to_pose');self.planner=ActionClient(self,ComputePathToPose,'/compute_path_to_pose')
        self.resume=self.create_publisher(Bool,'/explore/resume',10)
        self.done=self.create_publisher(Bool,'/slam/mapping_complete',latched())
        self.search_done=self.create_publisher(Bool,'/slam/mapping_search_done',latched())
        self.status=self.create_publisher(String,'/slam/coverage_status',latched())
        self.create_subscription(OccupancyGrid,'/map',self.on_map,latched())
        self.create_subscription(OccupancyGrid,'/global_costmap/costmap',lambda m:setattr(self,'live',ExplorationMap(m,self.p['coverage_occupied_threshold'])),latched())
        self.create_subscription(DockMap,'/slam/docks',self.on_docks,latched())
        self.create_subscription(Odometry,'/odom',lambda m:setattr(self,'odom',m),qos_profile_sensor_data)
        self.create_subscription(String,'/slam/tag_frame',self.frame,10)
        self.create_subscription(ExploreStatus,'/explore/status',self.explore_status,latched())
        self.create_subscription(Bool,'/explore/frontiers_remaining',lambda m:setattr(self,'remaining',m.data),latched())
        self.create_subscription(String,'/slam/status',self.slam_status,10)
        self.create_subscription(Bool,'/slam/mapping_pause',self.pause,10)
        self.create_subscription(Bool,'/explore/resume',lambda m:self.resume.publish(Bool(data=False)) if m.data and self.phase in ('FINISHING','DONE') else None,10)
        self.server=ActionServer(self,NavigateToPose,'/exploration/navigate_to_pose',execute_callback=self.explore_goal,
            goal_callback=lambda g:GoalResponse.ACCEPT if self.phase=='LIDAR' and not self.paused else GoalResponse.REJECT,
            cancel_callback=lambda h:CancelResponse.ACCEPT,callback_group=ReentrantCallbackGroup())
        self.create_timer(self.p['coverage_tick_period'],self.tick)
        self.done.publish(Bool(data=False));self.search_done.publish(Bool(data=False))

    def on_docks(self,m):
        self.docks=m
        if self.phase in ('DRIVE','HOLD') and self.target_docks and all(self.valid(m.docks[i]) for i in self.target_docks):
            self.retire(self.goal);self.goal=None;self.phase='CAMERA'
            self.get_logger().info(f'View completed from processed frames; qualified docks={self.target_docks}; no extra dwell/revisit')
            self.target_docks=[]
        if m.ready and self.phase not in ('FINISHING','DONE'):
            self.end_frontier();self.retire(self.goal);self.goal=None
            self.phase='DONE';self.resume.publish(Bool(data=False));self.done.publish(Bool(data=True))
            self.get_logger().info('Dock map already ready; mapping goal sources disabled')

    def now(self):return self.get_clock().now().nanoseconds/1e9
    def on_map(self,m):self.world=WallCoverage(m,self.p,self.world)

    def pose(self):
        tr=self.buffer.lookup_transform('map','base_footprint',Time()).transform
        return np.array([tr.translation.x,tr.translation.y,yaw(tr.rotation)])

    def stopped(self):
        return self.odom is not None and self.now()-seconds(self.odom.header.stamp)<self.p['coverage_frame_stale'] and abs(self.odom.twist.twist.linear.x)<self.p['coverage_stop_linear'] and abs(self.odom.twist.twist.angular.z)<self.p['coverage_stop_angular']

    def frame(self,m):
        f=json.loads(m.data)
        if self.world is None or self.now()-f['stamp_s']>self.p['coverage_frame_stale']:return
        self.world.observe(f)
        if self.phase=='HOLD' and self.stopped() and self.hold_since is not None and f['stamp_s']>=self.hold_since and f['stamp_s']-self.last_hold_frame>=self.p['coverage_sighting_interval']:
            self.hold_frames+=1;self.last_hold_frame=f['stamp_s']

    def uncertain(self):
        if self.docks is None:return []
        return [d for d in self.docks.docks if not self.valid(d) and seconds(d.last_observed)>0]

    def valid(self,d):
        c=d.tag_pose.covariance
        return d.valid and np.linalg.eigvalsh([[c[0],c[1]],[c[6],c[7]]])[-1]<=self.p['coverage_max_position_variance'] and c[35]<=self.p['coverage_max_yaw_variance']

    def slam_status(self,m):
        if self.phase=='FINISHING' and 'snapshot_saved=True' in m.data:
            self.phase='DONE';self.done.publish(Bool(data=True));self.get_logger().info('Mapping saved; explorer and tag search inactive; mission may own goals')

    def explore_status(self,m):
        if m.status==m.EXPLORATION_COMPLETE and self.phase=='LIDAR':
            self.phase='CAMERA';self.resume.publish(Bool(data=False))
            self.get_logger().info(f'Lidar exploration stopped; frontiers_remaining={self.remaining}; starting tag coverage')

    def retire(self,handle):
        if handle is not None and handle.accepted:
            self.retiring.append(handle.get_result_async());handle.cancel_goal_async()

    def end_frontier(self,success=False,error_code=0,error_msg=''):
        if self.frontier:
            handle,future=self.frontier;self.frontier=None
            if handle.is_active:
                if handle.is_cancel_requested:handle.canceled()
                elif success:handle.succeed()
                else:handle.abort()
            if not future.done():future.set_result(NavigateToPose.Result(error_code=error_code,error_msg=error_msg))
        self.retire(self.frontier_backend);self.frontier_backend=None

    async def explore_goal(self,handle):
        self.end_frontier();future=Future();self.frontier=(handle,future)
        return await future

    def frontier_tick(self):
        if self.frontier is None:return
        handle,_=self.frontier
        if handle.is_cancel_requested:self.end_frontier();return
        if self.frontier_backend or self.frontier_pending or self.retiring or not self.nav.server_is_ready():return
        p=handle.request.pose.pose.position
        if self.world is None or not self.world.frontier_clear([p.x,p.y],self.p['coverage_min_clearance']):
            self.get_logger().warning(f'Reject frontier inside obstacle clearance: ({p.x:.3f},{p.y:.3f})')
            self.end_frontier(error_code=ComputePathToPose.Result.GOAL_OCCUPIED,error_msg='Frontier violates known-obstacle clearance');return
        future=self.nav.send_goal_async(handle.request);self.frontier_pending=future
        def accepted(f):
            self.frontier_pending=None;backend=f.result()
            if self.frontier is None or self.frontier[0] is not handle or self.phase!='LIDAR' or self.paused:self.retire(backend);return
            if not backend.accepted:self.end_frontier();return
            self.frontier_backend=backend
            def completed(f):
                if self.frontier and self.frontier[0] is handle:self.frontier_backend=None;self.end_frontier(f.result().status==4)
            backend.get_result_async().add_done_callback(completed)
        future.add_done_callback(accepted)

    def pause(self,m):
        if self.phase in ('FINISHING','DONE'):return
        self.paused=m.data
        if m.data:self.end_frontier();self.retire(self.goal);self.goal=None;self.resume.publish(Bool(data=False))
        elif self.phase=='LIDAR':self.resume.publish(Bool(data=True))
        else:self.phase='CAMERA';self.target=None

    def finish(self,reason):
        self.phase='FINISHING';self.end_frontier();self.retire(self.goal);self.goal=None;self.resume.publish(Bool(data=False))
        self.search_done.publish(Bool(data=True))
        missing=[i for i in range(8) if self.docks is None or not self.valid(self.docks.docks[i])]
        self.get_logger().warning(f'Mapping search finished: {reason}; missing/uncertain docks={missing}; frontiers_remaining={self.remaining}; saving found docks')

    def pose_message(self,c):
        m=PoseStamped();m.header.frame_id='map';m.header.stamp=self.get_clock().now().to_msg()
        m.pose.position.x=c[1];m.pose.position.y=c[2];m.pose.orientation.z=math.sin(c[3]/2);m.pose.orientation.w=math.cos(c[3]/2)
        return m

    def check_candidate(self):
        if self.path_pending is not None or not self.planner.server_is_ready():return
        if not self.candidates:
            if self.reachable:self.drive(max(self.reachable))
            else:self.phase='CAMERA'
            return
        candidate=self.candidates.pop(0)
        self.path_pending=self.planner.send_goal_async(ComputePathToPose.Goal(goal=self.pose_message(candidate),planner_id='GridBased',use_start=False))
        def accepted(f):
            handle=f.result()
            if not handle.accepted:self.path_pending=None;return
            self.path_pending=handle.get_result_async()
            def complete(f):
                self.path_pending=None
                if self.phase!='PLAN' or self.paused:return
                r=f.result()
                if r.status==4 and r.result.path.poses:
                    points=np.array([[x.pose.position.x,x.pose.position.y] for x in r.result.path.poses]);length=np.linalg.norm(np.diff(points,axis=0),axis=1).sum()
                    self.reachable.append((candidate[4]-self.p['coverage_travel_weight']*length,*candidate[1:]))
                else:self.cooldown.append(((candidate[1],candidate[2]),self.now()+self.p['coverage_retry_cooldown']))
            self.path_pending.add_done_callback(complete)
        self.path_pending.add_done_callback(accepted)

    def drive(self,candidate):
        if not self.nav.server_is_ready():return
        # Keep explicit tag identity through Nav2 reachability scoring; wall raster
        # thickness must not discard it. Recheck after planning's async callbacks.
        targeted=candidate[5]
        self.target_docks=[i for i in targeted if self.docks is None or not self.valid(self.docks.docks[i])]
        if targeted and not self.target_docks:self.phase='CAMERA';return
        self.phase='DRIVE';self.target=candidate;self.goal_started=self.now()
        self.get_logger().info(f'Camera next-best-view target={candidate[1:4]} score={candidate[0]:.2f} unqualified_targets={self.target_docks}')
        self.goal_pending=self.nav.send_goal_async(NavigateToPose.Goal(pose=self.pose_message(candidate)))
        def accepted(f):
            self.goal_pending=None;handle=f.result()
            if self.phase!='DRIVE' or self.paused:self.retire(handle);return
            if not handle.accepted:self.phase='CAMERA';return
            self.goal=handle
            def complete(f):
                if self.goal is not handle:return
                self.goal=None
                if self.phase!='DRIVE' or self.paused:return
                if f.result().status==4:
                    self.phase='HOLD';self.hold_since=None;self.hold_frames=0;self.last_hold_frame=-math.inf;self.arrived_at=self.now()
                else:self.phase='CAMERA'
                self.cooldown.append(((candidate[1],candidate[2]),self.now()+self.p['coverage_retry_cooldown']))
            handle.get_result_async().add_done_callback(complete)
        self.goal_pending.add_done_callback(accepted)

    def tick(self):
        now=self.now();self.retiring=[f for f in self.retiring if not f.done()]
        if self.world is None:return
        if self.start is None:self.start=now
        valid=sum(self.valid(d) for d in self.docks.docks) if self.docks else 0
        self.status.publish(String(data=json.dumps(dict(phase=self.phase,paused=self.paused,valid_docks=int(valid),walls=len(self.world.seen),camera_seen=int(self.world.seen.sum()),frontiers_remaining=self.remaining,elapsed_s=now-self.start))))
        if self.phase in ('FINISHING','DONE') or self.paused:return
        if now-self.start>=self.p['coverage_total_timeout']:self.finish('overall time limit');return
        if not self.remaining and valid==8:self.finish('no lidar frontiers and all eight covariance-qualified docks');return
        if self.phase=='LIDAR':self.frontier_tick();return
        self.end_frontier()
        if self.p['tag_search']=='off':return  # Lidar-only fallback; observed dock validation still counts.
        if self.phase=='CAMERA' and len(self.world.seen) and self.world.seen.all() and not self.uncertain():
            self.finish('every exposed wall cell camera-seen; unavailable docks remain');return
        if self.phase=='HOLD':
            if not self.uncertain() and self.hold_frames>=self.p['coverage_required_sightings']:self.phase='CAMERA';return
            if self.stopped():
                if self.hold_since is None:self.hold_since=now
                if self.hold_frames>=self.p['coverage_required_sightings'] and now-self.hold_since>=validation_hold(self.p):self.phase='CAMERA'
            else:self.hold_since=None;self.hold_frames=0
            if now-self.arrived_at>self.p['coverage_dwell_timeout']:self.phase='CAMERA'
            return
        if self.phase=='DRIVE':
            if now-self.goal_started>self.p['coverage_goal_timeout']:
                self.retire(self.goal);self.goal=None;self.cooldown.append(((self.target[1],self.target[2]),now+self.p['coverage_retry_cooldown']));self.phase='CAMERA'
            return
        if self.retiring or self.goal_pending or self.frontier_pending:return
        if self.phase=='PLAN':self.check_candidate();return
        if self.live is None:return
        try:pose=self.pose()
        except TransformException:return
        self.candidates=self.world.candidates(pose,self.live,self.cooldown,now,self.uncertain(),[d for d in self.docks.docks if self.valid(d)] if self.docks else []);self.reachable=[]
        if self.candidates:self.phase='PLAN';self.check_candidate()


def main():run(CoverageNode)
