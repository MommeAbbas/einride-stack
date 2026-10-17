"""Publish bounded raw depth clouds to Nav2; missing input remains a missing guard source."""
import numpy as np
from sensor_msgs.msg import Image,CameraInfo,PointCloud2,PointField
from rclpy.qos import qos_profile_sensor_data
from .ros_support import AreaNode,seconds,run
from .depth import project_depth


class DepthNode(AreaNode):
    def __init__(self):
        super().__init__('control_depth_guard');self.camera=None;self.last=-float('inf')
        self.pub=self.create_publisher(PointCloud2,'/control/depth_points',qos_profile_sensor_data)
        self.create_subscription(CameraInfo,'/oak/rgb/camera_info',lambda m:setattr(self,'camera',m),qos_profile_sensor_data)
        self.create_subscription(Image,self.p('depth_topic'),self.on_depth,qos_profile_sensor_data)
    def on_depth(self,m):
        t=seconds(m.header.stamp)
        if self.camera is None or t-self.last<1/self.p('depth_rate'):return
        if m.encoding not in ('32FC1','16UC1'):return
        self.last=t;dtype=np.dtype('<f4' if m.encoding=='32FC1' else '<u2')
        if m.is_bigendian:dtype=dtype.newbyteorder('>')
        a=np.ndarray((m.height,m.width),dtype=dtype,buffer=bytes(m.data),strides=(m.step,dtype.itemsize)).astype(float)
        if m.encoding=='16UC1':a*=.001
        points=project_depth(a,self.camera.k,self.p('depth_pixel_stride'),self.p('depth_min'),self.p('depth_max'))
        out=PointCloud2(header=m.header,height=1,width=len(points),is_bigendian=False,point_step=12,row_step=len(points)*12,is_dense=True,data=points.tobytes())
        out.fields=[PointField(name=name,offset=i*4,datatype=PointField.FLOAT32,count=1) for i,name in enumerate(('x','y','z'))];self.pub.publish(out)


def main():run(DepthNode)
