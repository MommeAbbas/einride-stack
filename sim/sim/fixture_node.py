"""Evaluation-only isolation fixture; it is not sensor-built mapping evidence."""
import json
import math
from pathlib import Path
import numpy as np
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster
from interfaces.msg import DockMap,DockLandmark
from .ros_support import SimNode,latched,run


class FixtureNode(SimNode):
    def __init__(self):
        super().__init__('TEMPORARY_sim_map_fixture')
        data=np.load(Path(self.p('run_dir'))/'map.npz');self.m=OccupancyGrid();self.m.header.frame_id='map';self.m.info.resolution=float(data['resolution']);self.m.info.height,self.m.info.width=data['occupied'].shape
        self.m.info.origin.position.x,self.m.info.origin.position.y=map(float,data['origin']);self.m.info.origin.orientation.w=1.;self.m.data=(data['occupied']*100).astype(int).ravel().tolist()
        self.map_pub=self.create_publisher(OccupancyGrid,'/map',latched());self.dock_pub=self.create_publisher(DockMap,'/slam/docks',latched())
        self.docks=DockMap(map_revision=1,ready=True);self.docks.header.frame_id='map'
        geometry=json.loads((Path(self.p('run_dir'))/'geometry.json').read_text())
        for i,letter in enumerate('ABCDEFGH'):
            xy=geometry['docks'][letter];heading=math.atan2(xy[1],0.) if abs(xy[1])>abs(xy[0]) else (0. if xy[0]>0 else math.pi);dock=DockLandmark(dock_id=i,valid=True)
            for name in ('tag_pose','docking_pose','approach_pose'):
                offset=-self.p('fixture_approach_offset') if name=='approach_pose' else 0.
                pose=getattr(dock,name);pose.pose.position.x=float(xy[0])+offset*math.cos(heading);pose.pose.position.y=float(xy[1])+offset*math.sin(heading);pose.pose.orientation.z=math.sin(heading/2);pose.pose.orientation.w=math.cos(heading/2)
                pose.covariance[0]=pose.covariance[7]=self.p('metric_covariance_epsilon')
            self.docks.docks[i]=dock
        self.tf=StaticTransformBroadcaster(self);t=TransformStamped();t.header.frame_id='map';t.child_frame_id='odom';t.transform.rotation.w=1.;self.tf.sendTransform(t)
        self.create_timer(1/self.p('metrics_rate'),self.publish);self.publish()
        self.get_logger().warning('TEMPORARY oracle fixture: not SLAM or exploration evidence')
    def publish(self):
        self.m.header.stamp=self.get_clock().now().to_msg();self.docks.header.stamp=self.m.header.stamp
        self.map_pub.publish(self.m);self.dock_pub.publish(self.docks)


def main():run(FixtureNode)
