"""Stable public interfaces over slam_toolbox/AMCL; tags estimate docks, never ego pose."""
from copy import deepcopy
import json
import math
from pathlib import Path
import cv2
import numpy as np
from scipy.spatial.transform import Rotation
from rclpy.time import Time
from rclpy.qos import qos_profile_sensor_data
from tf2_ros import Buffer, TransformListener, TransformException
from geometry_msgs.msg import PoseWithCovarianceStamped,PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from nav2_msgs.srv import SaveMap
from sensor_msgs.msg import Image, CameraInfo, LaserScan
from std_msgs.msg import String,Bool
from interfaces.msg import DockMap, DockLandmark
from .ros_support import AreaNode, seconds, yaw, latched, run
from .landmarks import detect_tags
from .snapshot import load_snapshot, sanitise_ranges, fuse_landmark,landmark_valid,tag_covariance


def image_array(m):
    channels=3 if m.encoding in ('rgb8','bgr8') else 1
    a=np.frombuffer(m.data,dtype=np.uint8).reshape(m.height,m.step)[:,:m.width*channels]
    return a.reshape(m.height,m.width,channels).copy() if channels==3 else a.copy()


class SlamNode(AreaNode):
    def __init__(self):
        super().__init__('slam_adapter')
        self.buffer=Buffer();self.listener=TransformListener(self.buffer,self)
        self.pending_image=None;self.camera=None;self.last_image=-math.inf;self.landmarks={};self.revision=1
        self.dropped_tf=0;self.invalid_bins=0;self.backend_pose=None;self.map=None
        self.stopped_since=None;self.saved=False;self.save_future=None;self.save_error=''
        self.frozen=bool(self.p('load_snapshot'));self.snapshot=None
        self.mapping_complete=not self.p('mapping_managed') or self.frozen
        self.frontiers_remaining=self.p('mapping_managed') and not self.frozen
        self.create_subscription(Bool,'/explore/frontiers_remaining',lambda m:setattr(self,'frontiers_remaining',m.data),latched())
        self.create_subscription(Bool,'/slam/mapping_search_done',lambda m:setattr(self,'mapping_complete',m.data),latched())
        self.live_docks={i:self.create_publisher(PoseStamped,f'/slam/live_docks/dock_{i}',10) for i in range(8)}
        self.frame_pub=self.create_publisher(String,'/slam/tag_frame',10)
        if self.frozen:
            self.snapshot=load_snapshot(self.p('load_snapshot'))
            self.landmarks={int(k):dict(v,xy=np.array(v['xy']),cov=np.array(v['cov'])) for k,v in self.snapshot['landmarks'].items()}
        self.dock_pub=self.create_publisher(DockMap,'/slam/docks',latched())
        self.pose_pub=self.create_publisher(PoseWithCovarianceStamped,'/slam/pose',10)
        self.diag=self.create_publisher(String,'/slam/status',10)
        self.scan_pub=self.create_publisher(LaserScan,'/slam/scan',qos_profile_sensor_data)
        self.initial_pub=self.create_publisher(PoseWithCovarianceStamped,'/initialpose',10)
        self.save_client=self.create_client(SaveMap,'/map_saver/save_map')
        self.create_subscription(LaserScan,'/scan',self.on_scan,qos_profile_sensor_data)
        self.create_subscription(OccupancyGrid,'/map',lambda m:setattr(self,'map',m),latched())
        self.create_subscription(PoseWithCovarianceStamped,'/amcl_pose' if self.frozen else '/pose',self.on_pose,10)
        self.create_subscription(CameraInfo,'/oak/rgb/camera_info',lambda m:setattr(self,'camera',m),qos_profile_sensor_data)
        self.create_subscription(Image,'/oak/rgb/image_raw',self.on_image,qos_profile_sensor_data)
        self.create_subscription(Odometry,'/odom',self.on_odom,qos_profile_sensor_data)
        self.create_timer(1/self.p('status_rate'),self.publish)
        self.image_wait_timer=self.create_timer(self.p('tag_tf_check_period'),self.finish_image_wait)
        self.image_wait_timer.cancel()

    def on_scan(self,m):
        # Karto rejects below-min ranges; AMCL turns them into ignored max-range beams.
        # NaN/+inf are NOT evidence of free space. Raw /scan stays untouched for others.
        out=deepcopy(m);out.ranges,count=sanitise_ranges(m.ranges,m.range_min,m.range_max)
        self.invalid_bins+=count;self.scan_pub.publish(out)

    def on_pose(self,m):
        if not np.isfinite([m.pose.pose.position.x,m.pose.pose.position.y,*m.pose.covariance]).all():return
        self.backend_pose=m;self.pose_pub.publish(m) # Actual backend estimate and covariance, no fixed substitute.

    def on_odom(self,m):
        stopped=abs(m.twist.twist.linear.x)<=self.p('snapshot_stop_linear') and abs(m.twist.twist.angular.z)<=self.p('snapshot_stop_angular')
        if not stopped:self.stopped_since=None
        elif self.stopped_since is None:self.stopped_since=seconds(m.header.stamp)

    def on_image(self,m):
        t=seconds(m.header.stamp)
        if self.pending_image is not None or self.camera is None or t-self.last_image<1/self.p('tag_rate'):return
        self.last_image=t
        # Installed tf2 waits without blocking the executor that receives /tf.
        future=self.buffer.wait_for_transform_async('map',m.header.frame_id,Time(seconds=t))
        self.pending_image=(m,future,self.now())
        self.image_wait_timer.reset()
        self.finish_image_wait()

    def finish_image_wait(self):
        if self.pending_image is None:return
        m,future,started=self.pending_image
        if not future.done():
            if self.now()-started>self.p('tag_tf_wait_timeout'):
                self.pending_image=None;self.image_wait_timer.cancel();future.cancel();self.dropped_tf+=1
            return
        self.pending_image=None;self.image_wait_timer.cancel()
        try:tr=future.result().transform
        except (TransformException,AttributeError):self.dropped_tf+=1;return
        self.process_image(m,tr)

    def process_image(self,m,tr):
        t=seconds(m.header.stamp)
        q=tr.rotation;rot=Rotation.from_quat([q.x,q.y,q.z,q.w]).as_matrix()
        origin=np.array([tr.translation.x,tr.translation.y,tr.translation.z])
        a=image_array(m);gray=cv2.cvtColor(a,cv2.COLOR_RGB2GRAY if m.encoding=='rgb8' else cv2.COLOR_BGR2GRAY)
        for ident,position,orientation,error in detect_tags(gray,np.array(self.camera.k).reshape(3,3),np.array(self.camera.d),self.p('tag_size'),self.p('tag_reprojection_gate'),include_rejected=True):
            xyz=origin+rot@position;normal=rot@orientation[:,2];heading=math.atan2(normal[1],normal[0])
            distance=float(np.linalg.norm(position));incidence=math.acos(float(np.clip(abs(orientation[:,2]@(-position))/distance,0.,1.)))
            close=distance<=self.p('tag_mapping_max_range') and (not self.p('tag_geometry_covariance') or math.degrees(incidence)<=self.p('tag_max_incidence_deg'))
            covariance,yaw_variance=tag_covariance(distance,incidence,{k:self.p(k) for k in self._parameters}) if self.p('tag_geometry_covariance') else (None,None)
            if error<=self.p('tag_reprojection_gate') and ident in self.live_docks:
                live=PoseStamped();live.header.frame_id='map';live.header.stamp=m.header.stamp
                live.pose.position.x=float(xyz[0]);live.pose.position.y=float(xyz[1]);live.pose.position.z=float(xyz[2])
                live.pose.orientation.z=math.sin((heading+math.pi)/2);live.pose.orientation.w=math.cos((heading+math.pi)/2)
                self.live_docks[ident].publish(live)  # Live receipt stays live even after map/docks freeze.
            old=self.landmarks.get(ident)
            value=fuse_landmark(old,xyz,heading,t,close,{k:self.p(k) for k in self._parameters},covariance,yaw_variance) if error<=self.p('tag_reprojection_gate') else None
            self.get_logger().info('tag_detection '+json.dumps(dict(dock_id=ident,distance_m=distance,incidence_deg=math.degrees(incidence),decision_margin=None,decision_margin_reason='OpenCV aruco does not expose AprilTag decision margin',reprojection_rms_px=error,image_used=dict(stamp_s=t,frame=m.header.frame_id,encoding=m.encoding),accepted=value is not None,validation_geometry=close)))
            if value is not None and not self.frozen and not self.saved:self.landmarks[ident]=value

        self.frame_pub.publish(String(data=json.dumps(dict(stamp_s=t,origin=origin.tolist(),rotation=rot.tolist(),width=m.width,height=m.height,fx=self.camera.k[0],fy=self.camera.k[4],cx=self.camera.k[2],cy=self.camera.k[5]))))

    def start_or_finish_save(self,ready):
        if self.frozen or self.saved or not self.p('save_snapshot') or not ready:return
        if self.save_future is not None:
            if not self.save_future.done():return
            try:
                if not self.save_future.result().result:raise RuntimeError('Nav2 map saver returned false')
                # No motion is authorised after E0 readiness. Save the settled TF pose.
                tr=self.buffer.lookup_transform('map',self.p('base_frame'),Time()).transform
                q=tr.rotation
                data={'format_version':1,'backend':'slam_toolbox','partial':self.frontiers_remaining or not all(i in self.landmarks and landmark_valid(self.landmarks[i],{k:self.p(k) for k in self._parameters}) for i in range(8)),'map_yaml':self.map_url.with_suffix('.yaml').name,
                      'final_pose':[tr.translation.x,tr.translation.y,yaw(q)],
                      'final_pose_covariance':list(self.backend_pose.pose.covariance),
                      'covariance_stamp_s':seconds(self.backend_pose.header.stamp),
                      'saved_at_sim_s':self.now(),'landmarks':{str(i):{k:v.tolist() if isinstance(v,np.ndarray) else v for k,v in d.items()} for i,d in self.landmarks.items()}}
                path=Path(self.p('save_snapshot'));tmp=path.with_suffix('.tmp');tmp.write_text(json.dumps(data,indent=2,allow_nan=False)+'\n');tmp.replace(path)
                self.saved=True;self.get_logger().info(f'Snapshot saved: map + {len(self.landmarks)} found docks + settled final pose; partial={data["partial"]}')
            except Exception as e:self.save_error=str(e);self.get_logger().error(self.save_error)
            self.save_future=None
            return
        if self.backend_pose is None or self.map is None or self.stopped_since is None or self.now()-self.stopped_since<self.p('snapshot_settle'):return
        if not self.save_client.service_is_ready():return
        output=Path(self.p('save_snapshot')).resolve();output.parent.mkdir(parents=True,exist_ok=True);self.map_url=output.parent/'map'
        request=SaveMap.Request(map_topic='/map',map_url=str(self.map_url),image_format=self.p('map_image_format'),map_mode=self.p('map_mode'),free_thresh=self.p('map_free_thresh'),occupied_thresh=self.p('map_occupied_thresh'))
        self.save_future=self.save_client.call_async(request)

    def publish(self):
        if self.frozen and self.backend_pose is None and self.map is not None:
            # Retry until AMCL is active and has processed the initial-pose message.
            initial=PoseWithCovarianceStamped();initial.header.frame_id='map';initial.header.stamp=self.get_clock().now().to_msg()
            x,y,a=self.snapshot['final_pose'];initial.pose.pose.position.x=x;initial.pose.pose.position.y=y
            initial.pose.pose.orientation.z=math.sin(a/2);initial.pose.pose.orientation.w=math.cos(a/2)
            initial.pose.covariance=self.snapshot['final_pose_covariance'];self.initial_pub.publish(initial)
        docks=DockMap(map_revision=self.revision);docks.header.frame_id='map';docks.header.stamp=self.get_clock().now().to_msg()
        docks.ready=(self.saved if self.p('mapping_managed') and not self.frozen else self.mapping_complete) and not (self.frozen and self.snapshot.get('partial',False)) and not self.frontiers_remaining and len(self.landmarks)==8 and all(landmark_valid(d,{k:self.p(k) for k in self._parameters}) for d in self.landmarks.values())
        for i in range(8):
            dock=DockLandmark(dock_id=i)
            if i in self.landmarks:
                d=self.landmarks[i];dock.valid=landmark_valid(d,{k:self.p(k) for k in self._parameters})
                # A loaded snapshot's historic observations are not fresh runtime detections.
                stamp=0. if self.frozen else d['stamp'];dock.last_observed=Time(seconds=stamp).to_msg()
                for field,offset in [('tag_pose',0.),('approach_pose',self.p('approach_standoff')),('docking_pose',self.p('dock_standoff'))]:
                    pose=getattr(dock,field);xy=d['xy']+offset*np.array([math.cos(d['heading']),math.sin(d['heading'])])
                    pose.pose.position.x,pose.pose.position.y=map(float,xy);pose.pose.position.z=d['z'] if field=='tag_pose' else 0.
                    heading=d['heading'] if field=='tag_pose' else d['heading']+math.pi
                    pose.pose.orientation.z=math.sin(heading/2);pose.pose.orientation.w=math.cos(heading/2)
                    pose.covariance[0]=float(d['cov'][0,0]);pose.covariance[1]=float(d['cov'][0,1]);pose.covariance[6]=float(d['cov'][1,0]);pose.covariance[7]=float(d['cov'][1,1]);pose.covariance[35]=d.get('yaw_var',self.p('tag_yaw_sigma')**2)
            docks.docks[i]=dock
        self.dock_pub.publish(docks);self.start_or_finish_save(self.mapping_complete if self.p('mapping_managed') else docks.ready)
        backend='nav2_amcl' if self.frozen else 'slam_toolbox'
        self.diag.publish(String(data=f'backend={backend}; tags={len(self.landmarks)}/8 ready={docks.ready}; snapshot_saved={self.saved}; invalid_scan_bins_rejected={self.invalid_bins}; revision={self.revision}; dropped_tf={self.dropped_tf}; save_error={self.save_error}'))


def main():run(SlamNode)
