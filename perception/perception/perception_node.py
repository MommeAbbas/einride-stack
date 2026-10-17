"""Capture-time ROS adapter. Production inputs exclude /sim/truth and /sim/routes."""
from collections import deque
from copy import deepcopy
import math
import time
import json
from rclpy.clock import Clock,ClockType
import cv2
import numpy as np
from scipy.spatial.transform import Rotation
from scipy.ndimage import distance_transform_edt
import rclpy
from rclpy.time import Time
from rclpy.qos import qos_profile_sensor_data
from tf2_ros import Buffer,TransformListener,TransformException
from nav_msgs.msg import OccupancyGrid,Odometry
from sensor_msgs.msg import Image,CameraInfo,CompressedImage,LaserScan
from vision_msgs.msg import Detection2DArray,Detection3DArray,Detection3D,ObjectHypothesisWithPose
from std_msgs.msg import String
from builtin_interfaces.msg import Time as Stamp,Duration
from interfaces.msg import (DockMap,SagaState,Predictions,RobotPrediction,PredictedTrajectory,
    PredictionPoint,PerceptionHealth,CameraObservation,DockStates,DockState,Understanding,
    UnderstandingRegion,PlannedTrajectory)
from planning.grid import Grid
from planning.geometry import FrameGrid
from .ros_support import AreaNode,seconds,yaw,latched,run
from .filter import Belief
from .motion import route_fields
from .timing import MeasurementQueue
from .measurements import camera_cues,surprise_groups
from .visibility import ray_ranges,lidar_clear_mask,static_return_mask
from .output import predictions,understanding_region,moments


def stamp(t):
    ns=max(0,round(t*1e9)); return Stamp(sec=ns//10**9,nanosec=ns%10**9)


def image_array(msg):
    dtype=np.dtype('<f4' if msg.encoding=='32FC1' else '<u2' if msg.encoding=='16UC1' else 'u1')
    if msg.is_bigendian: dtype=dtype.newbyteorder('>')
    channels=3 if msg.encoding in ('rgb8','bgr8') else 1
    a=np.ndarray((msg.height,msg.width,channels) if channels==3 else (msg.height,msg.width),
                 dtype=dtype,buffer=bytes(msg.data),strides=(msg.step,channels*dtype.itemsize,dtype.itemsize) if channels==3 else (msg.step,dtype.itemsize))
    return a.astype(float)*.001 if msg.encoding=='16UC1' else a.copy()


class PerceptionNode(AreaNode):
    def __init__(self):
        super().__init__('perception_filter')
        self.params={k:self.p(k) for k in self._parameters if k!='use_sim_time'}
        if self.p('particles_per_robot')<1:raise ValueError('particles_per_robot must be positive')
        self.update_count=0;self.runtime_start=time.monotonic();self.runtime_cpu=time.process_time();self.last_cpu_log=self.runtime_start
        self.runtime_pub=self.create_publisher(String,'/perception/runtime',10)
        self.runtime_clock=Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(self.p('runtime_report_period'),self.report_runtime,clock=self.runtime_clock)
        self.buffer=Buffer(); self.listener=TransformListener(self.buffer,self)
        self.map_signature=None; self.map=None; self.docks=None; self.saga=None; self.belief=None; self.filter_time=None
        self.camera=None; self.depths=deque(maxlen=self.p('qos_depth')); self.odometry=deque(maxlen=self.p('odom_history_size'))
        self.scans=deque(maxlen=self.p('qos_depth'))
        self.queue=MeasurementQueue(['camera','lidar'] if self.p('lidar') else ['camera'],self.p('max_sensor_lag'))
        self.camera_records={}; self.receipt=None; self.dropped_tf=0; self.last_output=-math.inf; self.boxes=None; self.plan=None
        self.false_lidar=0; self.camera_detections=0; self.last_debug=-math.inf
        self.create_subscription(OccupancyGrid,'/map',self.on_map,latched())
        self.create_subscription(DockMap,'/slam/docks',self.on_docks,latched())
        self.create_subscription(SagaState,'/planning/saga_state',self.on_saga,10)
        self.create_subscription(PlannedTrajectory,'/planning/trajectory',lambda m:setattr(self,'plan',m),10)
        self.create_subscription(Odometry,'/odom',lambda m:self.odometry.append(m),qos_profile_sensor_data)
        self.create_subscription(CameraInfo,'/oak/rgb/camera_info',lambda m:setattr(self,'camera',m),qos_profile_sensor_data)
        self.create_subscription(Image,self.p('depth_topic'),lambda m:self.depths.append(m),qos_profile_sensor_data)
        self.create_subscription(Detection2DArray,self.p('detector_topic') if self.p('detector_source')=='emulator' else self.p('external_detector_topic'),self.on_boxes,10)
        self.create_subscription(LaserScan,'/scan',self.on_scan,qos_profile_sensor_data)
        self.create_subscription(Image,'/oak/rgb/image_raw',self.on_image,qos_profile_sensor_data)
        self.camera_tracks=self.create_publisher(Detection3DArray,'/perception/camera_tracks',10)
        self.pub=self.create_publisher(Predictions,'/perception/predictions',10)
        self.health=self.create_publisher(PerceptionHealth,'/perception/health',10)
        self.dock_pub=self.create_publisher(DockStates,'/perception/dock_states',10)
        self.understanding=self.create_publisher(Understanding,'/perception/understanding',10)
        self.debug=self.create_publisher(CompressedImage,'/perception/image/compressed',10)
        self.diagnostics=self.create_publisher(String,'/perception/diagnostics',10)
        self.update_timer=self.create_timer(1/self.p('filter_rate'),self.sensor_update)
        self.update_timer.cancel()  # Arrival-triggered coalescing; no periodic idle filter work.

    def on_saga(self,m):
        self.saga=m
        if m.event_state!='running':
            self.update_timer.cancel();self.scans.clear()
            self.queue=MeasurementQueue(['camera','lidar'] if self.p('lidar') else ['camera'],self.p('max_sensor_lag'))

    def wake(self):
        if self.saga and self.saga.event_state=='running' and self.update_timer.is_canceled():self.update_timer.reset()

    def sensor_update(self):
        self.update_timer.cancel();self.tick()
        # Bounded capture-time ordering may retain the newest frame until its
        # companion sensor arrives or max_sensor_lag expires. Drain only then.
        if self.queue.pending or self.scans:self.wake()

    def report_runtime(self):
        now=time.monotonic();elapsed=max(now-self.runtime_start,1e-9)
        cpu=100*(time.process_time()-self.runtime_cpu)/elapsed
        state='updating' if self.update_count else ('waiting for running Saga' if not self.saga or self.saga.event_state!='running' else 'waiting for map/docks/TF')
        report=dict(particles_per_robot=self.p('particles_per_robot'),cpu_percent_one_core=cpu,
                    update_hz=self.update_count/elapsed,state=state,window_seconds=elapsed)
        self.runtime_pub.publish(String(data=json.dumps(report)))
        if now-self.last_cpu_log>=self.p('cpu_log_period'):
            self.get_logger().info('perception_runtime '+json.dumps(report))
            self.last_cpu_log=now;self.runtime_start=now;self.runtime_cpu=time.process_time();self.update_count=0

    def tf(self,target,source,t):
        tf=self.buffer.lookup_transform(target,source,Time(seconds=t)).transform
        q=tf.rotation; rot=Rotation.from_quat([q.x,q.y,q.z,q.w]).as_matrix()
        return np.array([tf.translation.x,tf.translation.y,tf.translation.z]),rot

    def on_map(self,m):
        signature=(m.info.width,m.info.height,m.info.resolution,m.info.origin.position.x,m.info.origin.position.y,bytes(np.asarray(m.data,dtype=np.int8)))
        if signature==self.map_signature:return
        self.map_signature=signature
        a=np.array(m.data).reshape(m.info.height,m.info.width)
        self.map=Grid((a<0)|(a>=self.p('occupied_threshold')),m.info.resolution,[m.info.origin.position.x,m.info.origin.position.y])
        self.map_clearance=distance_transform_edt(~self.map.occupied)*self.map.resolution
        if self.belief is not None: self.refresh_map()

    def on_docks(self,m):
        previous=self.docks; self.docks=m
        if self.belief is not None and (previous is None or previous.map_revision!=m.map_revision):
            self.receipt=None; self.refresh_map()

    def refresh_map(self):
        if self.map is None or self.docks is None or not self.docks.ready: return False
        try:
            t,r=self.tf('map','odom',0.); grid=FrameGrid(self.map,[t[0],t[1],math.atan2(r[1,0],r[0,0])])
            docks=grid.to_odom([[d.docking_pose.pose.position.x,d.docking_pose.pose.position.y] for d in self.docks.docks])
            if self.belief is None:
                ego,_=self.tf('odom','base_footprint',0.); self.belief=Belief(grid,docks,ego[:2],self.params)
            else:
                self.belief.grid=grid; self.belief.docks=docks
                self.belief.fields,self.belief.blocked=route_fields(grid,docks,self.p('robot_radius'))
                self.belief.free=np.array([grid.point(c) for c in zip(*np.where(~self.belief.blocked))])
            return True
        except TransformException: return False

    def on_boxes(self,m):
        self.boxes=m
        if self.saga and self.saga.event_state=='running':
            self.queue.push('camera',seconds(m.header.stamp),m);self.wake()

    def sync_frame(self,t=0.):
        # Localisation can refine map->odom while a static map keeps its revision.
        # Particles stay in odom; only map queries, dock targets and sampling convert.
        tr,r=self.tf('map','odom',t)
        grid=FrameGrid(self.map,[tr[0],tr[1],math.atan2(r[1,0],r[0,0])])
        self.belief.grid=grid
        self.belief.docks=grid.to_odom([[d.docking_pose.pose.position.x,d.docking_pose.pose.position.y] for d in self.docks.docks])
        self.belief.free=grid.points(np.argwhere(~self.belief.blocked))
        ego,_=self.tf('odom','base_footprint',t)
        self.belief.repair_map_support(ego[:2])

    def on_scan(self,m):
        if self.p('lidar') and self.saga and self.saga.event_state=='running':self.scans.append(m);self.wake()

    def process_camera(self,t,m):
        if self.camera is None: return
        origin,rot=self.tf('odom',m.header.frame_id,t)
        heading=math.atan2(rot[1,2],rot[0,2]); measurements=[]; occluders=[]
        depth=min(self.depths,key=lambda x:abs(seconds(x.header.stamp)-t)) if self.depths else None
        depth=image_array(depth) if depth and abs(seconds(depth.header.stamp)-t)<=self.p('depth_sync_tolerance') else None
        boxes=[(d.bbox.center.position.x,d.bbox.center.position.y,d.bbox.size_x,d.bbox.size_y) for d in m.detections]
        k=self.camera.k; intrinsics=(k[0],k[4],k[2],k[5])
        for j,box in enumerate(boxes):
            # Conservative bottom occlusion by another box; static-map endpoint check below.
            bottom=box[1]+box[3]/2
            hidden=any(i!=j and abs(b[0]-box[0])<(b[2]+box[2])/2 and b[1]-b[3]/2<=bottom<=b[1]+b[3]/2 for i,b in enumerate(boxes))
            value=camera_cues(box,intrinsics,(self.camera.height,self.camera.width),depth,self.params,hidden)
            if value is None: continue
            bearing,distance,sigma=value
            point=origin+rot@np.array([-distance*math.sin(bearing),0.,distance*math.cos(bearing)])
            if not self.belief.grid.clear_line(origin[:2],point[:2]):
                value=camera_cues(box,intrinsics,(self.camera.height,self.camera.width),depth,self.params,True)
                if value is None: continue
                bearing,distance,sigma=value
                point=origin+rot@np.array([-distance*math.sin(bearing),0.,distance*math.cos(bearing)])
            delta=point[:2]-origin[:2]
            measurements.append((origin[:2],math.atan2(delta[1],delta[0]),float(np.linalg.norm(delta)),sigma,self.p('detect_max_range')))
            occluders.append((bearing,2*math.atan2(box[2]/2,k[0]),distance))
        odom=min(self.odometry,key=lambda o:abs(seconds(o.header.stamp)-t)) if self.odometry else None
        usable=odom is not None and abs(seconds(odom.header.stamp)-t)<self.p('pose_time_tolerance') and abs(odom.twist.twist.angular.z)<=self.p('max_look_turn_rate')
        applied=self.belief.update(measurements,t,(origin[:2],heading,occluders),negative=usable and self.p('camera_negative_evidence'))
        for i,capture in enumerate(self.belief.last_camera_support):
            if capture!=t:continue
            mean,cov=moments(self.belief.clouds[i][:,:2],self.belief.weights[i],self.p('covariance_epsilon'))
            mean=self.belief.grid.to_map(mean);cov=self.belief.grid.rot@cov@self.belief.grid.rot.T
            detection=Detection3D(id=str(i));detection.header.frame_id='map';detection.header.stamp=stamp(t)
            result=ObjectHypothesisWithPose();result.hypothesis.class_id='robot';result.hypothesis.score=1.0
            result.pose.pose.position.x=float(mean[0]);result.pose.pose.position.y=float(mean[1]);result.pose.pose.orientation.w=1.
            result.pose.covariance[0]=float(cov[0,0]);result.pose.covariance[1]=float(cov[0,1]);result.pose.covariance[6]=float(cov[1,0]);result.pose.covariance[7]=float(cov[1,1])
            detection.results=[result];self.camera_records[i]=detection
        self.camera_detections+=len(measurements)
        receipt=CameraObservation(); receipt.header=deepcopy(m.header); receipt.header.frame_id='map'
        receipt.map_revision=self.docks.map_revision; receipt.processed=True
        receipt.negative_evidence_usable=usable; receipt.negative_evidence_applied=applied
        receipt.detection_count=len(measurements); receipt.horizontal_fov=self.p('camera_hfov')-2*self.p('fov_edge_margin'); receipt.max_range=self.p('detect_max_range')
        mt,mr=self.tf('map',m.header.frame_id,t); q=Rotation.from_matrix(mr).as_quat()
        receipt.camera_pose.position.x,receipt.camera_pose.position.y,receipt.camera_pose.position.z=map(float,mt)
        receipt.camera_pose.orientation.x,receipt.camera_pose.orientation.y,receipt.camera_pose.orientation.z,receipt.camera_pose.orientation.w=map(float,q)
        self.receipt=receipt

    def prepare_scans(self,now):
        # Extract first, then enqueue EACH group at its own capture time. The filter
        # advances in one common camera/lidar order; scan start is not group time.
        while self.scans:
            m=self.scans[0];t=seconds(m.header.stamp)
            end_time=t+max(0,len(m.ranges)-1)*m.time_increment
            try:
                origin,rot=self.tf('odom',m.header.frame_id,t)
                self.tf('odom',m.header.frame_id,end_time)
            except TransformException:
                if now-end_time<self.p('max_sensor_lag'):break
                self.scans.popleft();self.dropped_tf+=1;continue
            self.scans.popleft()
            try:self.sync_frame(t)
            except TransformException:self.dropped_tf+=1;continue
            heading=math.atan2(rot[1,0],rot[0,0])
            angles=m.angle_min+np.arange(len(m.ranges))*m.angle_increment
            expected=ray_ranges(self.belief.grid,origin[:2],angles+heading,self.p('lidar_max_range')+self.p('lidar_surprise'))
            static=static_return_mask(self.belief.grid,origin[:2],angles+heading,m.ranges,self.map_clearance,self.p('lidar_static_gate'))
            groups=surprise_groups(m.ranges,expected,m.angle_increment,self.params,static)
            for i,distance,end in groups:
                capture=t+i*m.time_increment
                try:group_origin,group_rot=self.tf('odom',m.header.frame_id,capture)
                except TransformException:self.dropped_tf+=1;continue
                bearing=math.atan2(group_rot[1,0],group_rot[0,0])+angles[i]
                sigma=math.sqrt(self.p('lidar_sigma')**2+self.p('pose_sigma_xy')**2+(end*self.p('pose_sigma_yaw'))**2)
                self.queue.push('lidar',capture,('positive',(group_origin[:2],bearing,distance,sigma,end)))
            self.false_lidar+=len(groups)
            odom=min(self.odometry,key=lambda o:abs(seconds(o.header.stamp)-t)) if self.odometry else None
            stopped=odom is not None and abs(seconds(odom.header.stamp)-t)<self.p('pose_time_tolerance') and abs(odom.twist.twist.linear.x)<self.p('lidar_negative_max_speed') and abs(odom.twist.twist.angular.z)<self.p('lidar_negative_max_turn_rate')
            value=('negative',(origin[:2],heading,np.asarray(m.ranges),m.angle_min,m.angle_increment)) if self.p('lidar_negative_evidence') and not groups and stopped else ('watermark',None)
            self.queue.push('lidar',end_time,value)

    def process_lidar(self,t,value):
        kind,measurement=value
        if kind=='positive':self.belief.update([measurement],t)
        elif kind=='negative':
            masks=[lidar_clear_mask(c[:,:2],*measurement,self.belief.grid,self.params) for c in self.belief.clouds]
            self.belief.update([],t,negative=True,negative_masks=masks,detection_probability=self.p('lidar_p_detect'))

    def tick(self):
        now=self.now()
        if not self.saga or self.saga.event_state!='running': return
        if self.belief is None:
            if not self.refresh_map():
                self.queue.pending.clear();self.scans.clear();return
            self.filter_time=now-self.p('max_sensor_lag')
        held=self.saga.held_docks if now-seconds(self.saga.header.stamp)<self.p('saga_stale_timeout') else []
        self.belief.reservations(held)
        self.prepare_scans(now)
        processed=False
        for t,sensor,m in self.queue.ready(now):
            if t<self.filter_time: self.queue.dropped+=1; continue
            try:
                self.sync_frame(t)
                self.belief.predict(t-self.filter_time); self.filter_time=t
                if sensor=='camera': self.process_camera(t,m)
                else: self.process_lidar(t,m)
                processed=True
            except TransformException: self.dropped_tf+=1
        if not processed:return
        self.update_count+=1
        if now-self.last_output>=1/self.p('output_rate'):
            try:self.sync_frame();self.publish(now);self.last_output=now
            except TransformException:self.dropped_tf+=1

    def publish(self,now):
        output=Predictions(); output.header.frame_id='map'; output.header.stamp=stamp(now)
        output.map_revision=self.docks.map_revision
        output.time_step=Duration(sec=int(self.p('prediction_dt')),nanosec=round(self.p('prediction_dt')%1*1e9))
        output.horizon=Duration(sec=int(self.p('horizon')),nanosec=round(self.p('horizon')%1*1e9))
        self.belief.expire(self.filter_time)
        records=predictions(self.belief,max(0,now-self.filter_time),now)
        grid=self.belief.grid
        for i,(groups,leftover) in enumerate(records):
            robot=RobotPrediction(track_id=i); support=self.belief.last_support[i]
            robot.ever_confirmed=support is not None
            age=max(0,now-support) if support is not None else 0.
            robot.confirmation_age=Duration(sec=int(age),nanosec=round(age%1*1e9))
            trajectories=[]
            for mass,dock,mode,points in groups+[leftover]:
                group=PredictedTrajectory(probability=mass,destination_dock=dock,route_mode=mode)
                for j,(mean,cov) in enumerate(points):
                    xy=grid.to_map(mean); covariance=grid.rot@cov@grid.rot.T
                    group.points.append(PredictionPoint(stamp=stamp(now+j*self.p('prediction_dt')),x=float(xy[0]),y=float(xy[1]),covariance=covariance.ravel().tolist()))
                trajectories.append(group)
            robot.groups=trajectories[:-1]; robot.leftover=trajectories[-1]; output.robots[i]=robot
        self.pub.publish(output)
        # Per-detection header is the actual camera capture; pose is frozen at that
        # capture. Planning uses only this receipt to qualify current Predictions.
        receipt=Detection3DArray(header=output.header)
        receipt.detections=[d for i,d in self.camera_records.items() if self.belief.last_camera_support[i] is not None and seconds(d.header.stamp)==self.belief.last_camera_support[i]]
        self.camera_tracks.publish(receipt)
        health=PerceptionHealth(); health.header=output.header; health.map_revision=output.map_revision
        health.prediction_stamp=output.header.stamp; health.dropped_tf=self.dropped_tf; health.dropped_late=self.queue.dropped
        fresh=self.receipt is not None and now-seconds(self.receipt.header.stamp)<self.p('input_stale_timeout')
        health.status=health.NOMINAL if fresh else health.DEGRADED; health.reason='particle filter; provisional models' if fresh else 'camera receipt stale'
        if self.receipt is not None:
            health.has_camera_observation=True
            if now>seconds(self.receipt.header.stamp): self.receipt.predictions_after_frame=output.header.stamp
            health.last_camera_observation=self.receipt
        self.health.publish(health)
        docks=DockStates(); docks.header=output.header; docks.map_revision=output.map_revision
        for j,dock in enumerate(self.belief.docks):
            masses=[float(w@(np.linalg.norm(c[:,:2]-dock,axis=1)<self.p('dock_radius'))) for c,w in zip(self.belief.clouds,self.belief.weights)]
            occupancy=1-float(np.prod(1-np.array(masses)))
            docks.docks[j]=DockState(dock_id=j,valid=True,occupancy_probability=occupancy,physically_free=occupancy<self.p('dock_free_threshold'))
        self.dock_pub.publish(docks)
        understanding=Understanding(); understanding.header=output.header; understanding.map_revision=output.map_revision
        if self.plan is not None:
            understanding.goal_id=self.plan.goal_id; understanding.path_version=self.plan.path_version
            try: ego,_=self.tf('map','base_footprint',0.)
            except TransformException: ego=np.zeros(3)
            for point in self.plan.points:
                local=understanding_region(self.belief,np.array([point.x,point.y]),now)
                if local is not None:
                    target,mass,labels=local
                    region=UnderstandingRegion(radius=self.p('understanding_radius'),unconfirmed_mass=mass,target_heading=math.atan2(target[1]-ego[1],target[0]-ego[0]),track_ids=labels)
                    region.center.x,region.center.y=map(float,target); understanding.regions.append(region)
        self.understanding.publish(understanding)
        self.diagnostics.publish(String(data=f'camera_measurements={self.camera_detections}; lidar_groups={self.false_lidar}; dropped_tf={self.dropped_tf}; dropped_late={self.queue.dropped}; deleted_tracks={self.belief.deleted_tracks}'))

    def on_image(self,m):
        t=seconds(m.header.stamp)
        if t-self.last_debug<1/self.p('debug_image_rate'): return
        self.last_debug=t; a=image_array(m)
        if m.encoding=='rgb8': a=cv2.cvtColor(a,cv2.COLOR_RGB2BGR)
        if self.boxes and abs(seconds(self.boxes.header.stamp)-t)<self.p('debug_box_tolerance'):
            for d in self.boxes.detections:
                b=d.bbox; x,y=b.center.position.x,b.center.position.y
                cv2.rectangle(a,(int(x-b.size_x/2),int(y-b.size_y/2)),(int(x+b.size_x/2),int(y+b.size_y/2)),(0,255,0),2)
        ok,data=cv2.imencode('.jpg',a,[cv2.IMWRITE_JPEG_QUALITY,self.p('debug_jpeg_quality')])
        if ok: self.debug.publish(CompressedImage(header=m.header,format='jpeg',data=data.tobytes()))


def main(): run(PerceptionNode)
