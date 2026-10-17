"""Hardware and simulation encoding equivalence, including row padding/endianness."""
import numpy as np
import rclpy
from sensor_msgs.msg import Image
from perception.perception_node import image_array
from perception import detector_node


def test_depth_mm_and_float_m_are_equivalent():
    # Two padded rows; ROS big-endian payload on the mm path.
    mm = np.array([[1000, 2500, 99], [0, 500, 99]], dtype='>u2')
    m = Image(height=2, width=2, encoding='16UC1', is_bigendian=True, step=6, data=mm.tobytes())
    metres = np.array([[1., 2.5, 99.], [0., .5, 99.]], dtype='<f4')
    f = Image(height=2, width=2, encoding='32FC1', step=12, data=metres.tobytes())
    assert np.array_equal(image_array(m), image_array(f))
    assert np.array_equal(image_array(m), [[1., 2.5], [0., .5]])


def test_detector_converts_bgr_and_rgb_to_identical_model_input(monkeypatch):
    inputs=[]
    class Backend:
        def __init__(self, *args):pass
        def __call__(self, tensor):
            inputs.append(tensor.copy());return np.zeros((1,5,8400),np.float32)
        def close(self):pass
    monkeypatch.setattr(detector_node,'load_backend',lambda *args:(Backend(),'onnx',None))
    rclpy.init(args=['--ros-args','-p','backend:=onnx','-p','use_sim_time:=false'])
    node=detector_node.DetectorNode()
    try:
        # BGR red pixel, plus row padding; simulation's RGB equivalent.
        for encoding,data in [('bgr8',bytes([0,0,255,99,99,99])),('rgb8',bytes([255,0,0,99,99,99]))]:
            node.last=-float('inf')
            node.on_image(Image(height=1,width=1,encoding=encoding,step=6,data=data))
        assert len(inputs)==2
        assert np.array_equal(inputs[0],inputs[1])
        assert inputs[0][0,0,320,320]==1 and inputs[0][0,2,320,320]==0
    finally:
        node.destroy_node();rclpy.shutdown()
