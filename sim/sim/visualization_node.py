"""Recorded Foxglove markers: truth, estimated ego, trajectory modes, probabilities and ellipses."""
import math
from copy import deepcopy
import numpy as np
from geometry_msgs.msg import Point,TransformStamped
from visualization_msgs.msg import Marker,MarkerArray
from interfaces.msg import Predictions,RobotTruthArray
from tf2_ros import Buffer,TransformListener,StaticTransformBroadcaster,TransformException
from rclpy.time import Time
from builtin_interfaces.msg import Duration
from .ros_support import SimNode,run


class VisualizationNode(SimNode):
    def __init__(self):
        super().__init__('sim_visualization');self.buffer=Buffer();self.listener=TransformListener(self.buffer,self)
        self.truth_pub=self.create_publisher(MarkerArray,'/visualization/robots',10);self.pred_pub=self.create_publisher(MarkerArray,'/visualization/predictions',10)
        self.create_subscription(RobotTruthArray,'/sim/truth',self.truth,10);self.create_subscription(Predictions,'/perception/predictions',self.predictions,10)
        self.static=StaticTransformBroadcaster(self);tr=TransformStamped();tr.header.frame_id='world';tr.child_frame_id='map';tr.transform.rotation.w=1.;self.static.sendTransform(tr)
        # Simulation origin alignment only. Production localisation never consumes true poses.
    def marker(self,header,namespace,ident,kind,color):
        m=Marker(header=deepcopy(header),ns=namespace,id=ident,type=kind,action=Marker.ADD);m.pose.orientation.w=1.
        m.color.r,m.color.g,m.color.b,m.color.a=map(float,color)
        m.lifetime=Duration(sec=int(self.p('marker_lifetime')),nanosec=round(self.p('marker_lifetime')%1*1e9));return m
    def truth(self,message):
        markers=[]
        for robot in message.robots:
            color=(.2,1.,.2,.8) if robot.robot_id==0 else (1.,.65,.1,.8)
            m=self.marker(message.header,'true_robot',robot.robot_id,Marker.CUBE,color);m.pose=deepcopy(robot.pose);m.pose.position.z+=self.p('robot_height')/2
            m.scale.x=self.p('robot_length');m.scale.y=self.p('robot_width');m.scale.z=self.p('robot_height');markers.append(m)
            text=self.marker(message.header,'true_label',robot.robot_id,Marker.TEXT_VIEW_FACING,color);text.pose.position=deepcopy(m.pose.position);text.pose.position.z+=self.p('marker_text_height');text.scale.z=self.p('marker_text_size');text.text=f'TRUE ego' if robot.robot_id==0 else f'TRUE traffic {robot.robot_id}';markers.append(text)
        try:
            tr=self.buffer.lookup_transform('map','base_footprint',Time()).transform
            m=self.marker(message.header,'estimated_ego',0,Marker.ARROW,(.15,.5,1.,1.));m.header.frame_id='map';m.pose.position.x=tr.translation.x;m.pose.position.y=tr.translation.y;m.pose.position.z=self.p('marker_text_height');m.pose.orientation=tr.rotation
            m.scale.x=self.p('robot_length');m.scale.y=self.p('marker_line_width')*3;m.scale.z=self.p('marker_line_width')*3;markers.append(m)
        except TransformException:pass
        self.truth_pub.publish(MarkerArray(markers=markers))
    def predictions(self,message):
        markers=[Marker(action=Marker.DELETEALL)];ident=0
        colors=[(.9,.2,.6,1.),(.3,.8,1.,1.),(.7,.3,1.,1.)]
        for robot in message.robots:
            for group in [*robot.groups,robot.leftover]:
                if group.probability<=0:continue
                color=colors[robot.track_id%len(colors)];m=self.marker(message.header,'prediction_path',ident,Marker.LINE_STRIP,color);m.scale.x=self.p('marker_line_width')
                m.points=[Point(x=p.x,y=p.y,z=self.p('marker_path_height')) for p in group.points];markers.append(m)
                if group.points:
                    p=group.points[0];text=self.marker(message.header,'prediction_probability',ident,Marker.TEXT_VIEW_FACING,color);text.pose.position=Point(x=p.x,y=p.y,z=self.p('marker_text_height'));text.scale.z=self.p('marker_text_size')
                    text.text=f'track {robot.track_id}: {group.probability:.1%} '+('leftover' if group.destination_dock<0 else chr(65+group.destination_dock)+' / '+group.route_mode);markers.append(text)
                for j,p in enumerate(group.points[::self.p('marker_ellipse_stride')]):
                    covariance=np.array(p.covariance).reshape(2,2);eig,vec=np.linalg.eigh(covariance)
                    if not np.isfinite(eig).all() or eig.min()<0:continue
                    angles=np.linspace(0,2*math.pi,self.p('marker_ellipse_points')+1)
                    for sigma in (1,2):
                        ellipse=(np.c_[np.cos(angles),np.sin(angles)]*np.sqrt(eig)*sigma)@vec.T+[p.x,p.y]
                        e=self.marker(message.header,f'ellipse_{sigma}sigma',ident*100+j,Marker.LINE_STRIP,(*color[:3],.7 if sigma==1 else .35));e.scale.x=self.p('marker_line_width')/2
                        e.points=[Point(x=float(x),y=float(y),z=self.p('marker_path_height')) for x,y in ellipse];markers.append(e)
                ident+=1
        self.pred_pub.publish(MarkerArray(markers=markers))


def main():run(VisualizationNode)
