#!/usr/bin/env python3
"""Trace a recorded costmap mark to depth endpoints; truth only diagnoses height."""
import argparse,json
from pathlib import Path
import numpy as np
from rclpy.time import Time
from rclpy.duration import Duration
from tf2_ros import Buffer,TransformException
from sensor_msgs_py.point_cloud2 import read_points_numpy
from area_check import messages,seconds
from sim.detector import transform
from sim.detector_node import pose_matrix
from sim.history import PoseHistory

def matrix(tr):
 t=tr.translation;q=tr.rotation;return transform([t.x,t.y,t.z],[q.x,q.y,q.z,q.w])
def report(bag,xy,output):
 tf=Buffer(cache_time=Duration(seconds=10000));truth=PoseHistory(10000.)
 for topic,m,_ in messages(bag,['/tf','/tf_static','/sim/truth']):
  if topic=='/sim/truth':truth.add(seconds(m.header.stamp),{r.robot_id:pose_matrix(r.pose) for r in m.robots})
  else:
   for tr in m.transforms:
    if topic=='/tf_static':tf.set_transform_static(tr,'bag')
    else:tf.set_transform(tr,'bag')
 rows=[]
 for _,m,_ in messages(bag,['/control/depth_points']):
  t=seconds(m.header.stamp);xyz=read_points_numpy(m,field_names=['x','y','z'],skip_nans=True)
  try:a=matrix(tf.lookup_transform('map',m.header.frame_id,Time(seconds=t)).transform)
  except TransformException:continue
  mapped=xyz@a[:3,:3].T+a[:3,3];near=np.linalg.norm(mapped[:,:2]-xy,axis=1)<.06
  if not near.any():continue
  b=matrix(tf.lookup_transform('base_footprint',m.header.frame_id,Time(seconds=t)).transform)
  base=xyz[near]@b[:3,:3].T+b[:3,3];poses=truth.at(t,.2)
  if not poses:continue
  physical=poses[0]@b;world=xyz[near]@physical[:3,:3].T+physical[:3,3]
  marking=(mapped[near,2]>=.04)&(mapped[near,2]<=.5)
  rows.append(dict(stamp=t,near_endpoints=int(near.sum()),marking_endpoints=int(marking.sum()),map_z_min=float(mapped[near,2].min()),map_z_max=float(mapped[near,2].max()),base_z_min=float(base[:,2].min()),base_z_max=float(base[:,2].max()),world_reprojection_z_min=float(world[:,2].min()),world_reprojection_z_max=float(world[:,2].max()),base_xy_min=base[:,:2].min(axis=0).tolist(),base_xy_max=base[:,:2].max(axis=0).tolist()))
 result={'mark_xy':xy,'samples':rows,'note':'0.06 m endpoint matching radius for evaluation only; heights compare ROS TF and full simulator model orientation; reprojected sensor points remain noisy, not ground-truth surface hits.'}
 Path(output).write_text(json.dumps(result,indent=2)+'\n');print(json.dumps({'matched_frames':len(rows),'marking_frames':[r for r in rows if r['marking_endpoints']]}))
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('bag');p.add_argument('output');p.add_argument('--xy',nargs=2,type=float,required=True);a=p.parse_args();report(a.bag,a.xy,a.output)
