"""Process emulated detections at RGB capture stamps; never leaks truth identities."""
from collections import deque
import csv
from pathlib import Path
import numpy as np
import rclpy.time
from sensor_msgs.msg import CameraInfo, Image
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from tf2_ros import Buffer, TransformListener, TransformException
from interfaces.msg import RobotTruthArray
from .grid import Grid
from .history import PoseHistory
from .detector import transform, emulate
from .ros_support import SimNode, seconds, defaults, run


def pose_matrix(p):
    return transform([p.position.x,p.position.y,p.position.z],
                     [p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w])


class DetectorNode(SimNode):
    def __init__(self):
        super().__init__('sim_detector')
        self.params={key:self.p(key) for key in defaults()}
        self.history=PoseHistory(self.p('detector_history_seconds'))
        self.frames=deque(); self.info=None; self.last=-float('inf')
        self.rng=np.random.default_rng(self.p('route_seed'))
        self.tf=Buffer(); self.listener=TransformListener(self.tf,self)
        data=np.load(Path(self.p('run_dir'))/'map.npz')
        self.grid=Grid(data['occupied'],float(data['resolution']),data['origin'])
        self.audit_file=(Path(self.p('run_dir'))/'detector_visibility.csv').open('w',newline='')
        self.audit=csv.DictWriter(self.audit_file,fieldnames=['stamp','robot_id','in_fov','in_range','unoccluded','eligible','box_emitted','reason'])
        self.audit.writeheader()
        self.pub=self.create_publisher(Detection2DArray,'/sim/detections',self.p('qos_depth'))
        self.create_subscription(CameraInfo,'/oak/rgb/camera_info',self.camera,self.p('qos_depth'))
        self.create_subscription(Image,'/oak/rgb/image_raw',self.frame,self.p('qos_depth'))
        self.create_subscription(RobotTruthArray,'/sim/truth',self.truth,self.p('qos_depth'))
        self.create_timer(1/self.p('detector_rate'),self.process)

    def camera(self,message):
        self.info=message

    def frame(self,message):
        stamp=seconds(message.header.stamp)
        if stamp-self.last>=1/self.p('detector_rate'):
            self.frames.append((message.header,message.width,message.height)); self.last=stamp
        while self.frames and stamp-seconds(self.frames[0][0].stamp)>self.p('detector_history_seconds'):
            self.frames.popleft()

    def truth(self,message):
        self.history.add(seconds(message.header.stamp),{r.robot_id:pose_matrix(r.pose) for r in message.robots})

    def process(self):
        if not self.p('detector_enabled') or self.p('detector_source')!='emulator' or not self.info:
            return
        while self.frames:
            header,width,height=self.frames[0]; stamp=seconds(header.stamp)
            if not self.history.samples or self.history.samples[-1][0]<stamp:
                break
            self.frames.popleft()
            poses=self.history.at(stamp,self.p('detector_pose_tolerance'))
            if not poses or 0 not in poses:
                self.log_frame(stamp,[{'reason':'missing_truth'}]);continue
            try:
                tf=self.tf.lookup_transform('base_footprint',self.info.header.frame_id,rclpy.time.Time.from_msg(header.stamp))
            except TransformException:
                self.log_frame(stamp,[{'reason':'missing_camera_tf'}]);continue
            t,q=tf.transform.translation,tf.transform.rotation
            camera=poses[0]@transform([t.x,t.y,t.z],[q.x,q.y,q.z,q.w])
            audit=[]
            boxes=emulate(poses,camera,np.array(self.info.k).reshape(3,3),[width,height],
                           [self.p('robot_length'),self.p('robot_width'),self.p('robot_height')],
                           self.params,self.grid,self.rng,audit)
            self.log_frame(stamp,audit)
            result=Detection2DArray(); result.header=header; result.header.frame_id=self.info.header.frame_id
            for box in boxes:
                detection=Detection2D(); detection.header=result.header
                detection.bbox.center.position.x=float((box[0]+box[2])/2)
                detection.bbox.center.position.y=float((box[1]+box[3])/2)
                detection.bbox.size_x=float(box[2]-box[0]); detection.bbox.size_y=float(box[3]-box[1])
                hypothesis=ObjectHypothesisWithPose(); hypothesis.hypothesis.class_id='robot'; hypothesis.hypothesis.score=1.
                detection.results=[hypothesis]; result.detections.append(detection)
            self.pub.publish(result) # An empty processed frame is intentional evidence, not a failed detector.

    def log_frame(self,stamp,records):
        for record in records or [{'reason':'no_traffic'}]:self.audit.writerow({'stamp':stamp,**record})
        self.audit_file.flush()

    def destroy_node(self):
        self.audit_file.close()
        return super().destroy_node()


def main():
    run(DetectorNode)
