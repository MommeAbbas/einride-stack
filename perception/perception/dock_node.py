"""PROVISIONAL final-dock tag observations, independent of the robot detector.
Uses calibrated real camera pixels and capture-time TF; never a simulator dock pose.
"""
import math
import cv2
import numpy as np
from scipy.spatial.transform import Rotation
from rclpy.time import Time
from rclpy.qos import qos_profile_sensor_data
from tf2_ros import Buffer,TransformListener,TransformException
from sensor_msgs.msg import Image,CameraInfo
from interfaces.msg import DockMap,DockLandmark
from .ros_support import AreaNode,seconds,latched,run
from .dock_geometry import detect_tags
from .perception_node import image_array


class DockNode(AreaNode):
    def __init__(self):
        super().__init__('perception_dock_alignment');self.buffer=Buffer();self.listener=TransformListener(self.buffer,self)
        self.camera=None;self.revision=None;self.last=-math.inf;self.observations={}
        self.create_subscription(CameraInfo,'/oak/rgb/camera_info',lambda m:setattr(self,'camera',m),qos_profile_sensor_data)
        self.create_subscription(DockMap,'/slam/docks',self.on_map,latched())
        self.create_subscription(Image,'/oak/rgb/image_raw',self.image,qos_profile_sensor_data)
        self.pub=self.create_publisher(DockMap,'/perception/dock_observations',10)
    def on_map(self,m):
        if self.revision!=m.map_revision:self.observations.clear()
        self.revision=m.map_revision
    def image(self,m):
        t=seconds(m.header.stamp)
        if self.camera is None or self.revision is None or t-self.last<1/self.p('dock_tag_rate'):return
        self.last=t
        try:tr=self.buffer.lookup_transform('map',m.header.frame_id,Time(seconds=t)).transform
        except TransformException:return
        q=tr.rotation;r=Rotation.from_quat([q.x,q.y,q.z,q.w]).as_matrix();origin=np.array([tr.translation.x,tr.translation.y,tr.translation.z])
        a=image_array(m);gray=cv2.cvtColor(a,cv2.COLOR_RGB2GRAY if m.encoding=='rgb8' else cv2.COLOR_BGR2GRAY)
        out=DockMap(map_revision=self.revision);out.header.frame_id='map';out.header.stamp=m.header.stamp
        for ident,position,orientation,_ in detect_tags(gray,np.array(self.camera.k).reshape(3,3),np.array(self.camera.d),self.p('dock_tag_size'),self.p('dock_tag_reprojection_gate')):
            xyz=origin+r@position;normal=r@orientation[:,2];heading=math.atan2(normal[1],normal[0]);dock=DockLandmark(dock_id=ident,valid=True,last_observed=m.header.stamp)
            for field,offset in [('tag_pose',0.),('approach_pose',self.p('dock_approach_standoff')),('docking_pose',self.p('dock_standoff'))]:
                p=getattr(dock,field);xy=xyz[:2]+offset*np.array([math.cos(heading),math.sin(heading)])
                p.pose.position.x,p.pose.position.y=map(float,xy);p.pose.position.z=float(xyz[2]) if field=='tag_pose' else 0.
                angle=heading if field=='tag_pose' else heading+math.pi;p.pose.orientation.z=math.sin(angle/2);p.pose.orientation.w=math.cos(angle/2)
                p.covariance[0]=p.covariance[7]=self.p('dock_tag_sigma')**2;p.covariance[35]=self.p('dock_tag_yaw_sigma')**2
            self.observations[ident]=dock
        for ident,dock in self.observations.items():
            if t-seconds(dock.last_observed)<self.p('dock_observation_timeout'):out.docks[ident]=dock
        self.pub.publish(out)


def main():run(DockNode)
