"""Raw guard projection uses metres for both hardware and Gazebo depth encodings."""
import numpy as np
import rclpy
from sensor_msgs.msg import Image, CameraInfo
from control.depth_node import DepthNode


def test_depth_guard_mm_equals_metres_with_padding_and_endianness():
    rclpy.init(args=['--ros-args','-p','use_sim_time:=false','-p','depth_pixel_stride:=1'])
    node=DepthNode();outputs=[]
    class Publisher:
        def publish(self,message):outputs.append(message)
    node.pub=Publisher()
    node.camera=CameraInfo(k=[100.,0.,0.,0.,100.,0.,0.,0.,1.])
    try:
        mm=np.array([[1000,2000,99],[0,500,99]],dtype='>u2')
        f=np.array([[1.,2.,99],[0,.5,99]],dtype='<f4')
        for encoding,array,step in [('16UC1',mm,6),('32FC1',f,12)]:
            node.last=-float('inf')
            node.on_depth(Image(height=2,width=2,encoding=encoding,is_bigendian=encoding=='16UC1',step=step,data=array.tobytes()))
        assert len(outputs)==2 and outputs[0].width==3
        assert outputs[0].data==outputs[1].data
        assert np.array_equal(np.frombuffer(outputs[0].data,dtype='<f4').reshape(-1,3)[:,2],[1.,2.,.5])
    finally:
        node.destroy_node();rclpy.shutdown()
