#!/usr/bin/env python3
"""Recorded map/ego TF errors including times AMCL did not publish a new pose."""
import argparse,json,math
from pathlib import Path
import numpy as np
from rclpy.duration import Duration
from rclpy.time import Time
from tf2_ros import Buffer,TransformException
from area_check import messages,seconds
from sim.detector import transform
from sim.detector_node import pose_matrix

def report(bag,output):
 tf=Buffer(cache_time=Duration(seconds=10000));rows=[];last_pose=None
 for topic,m,_ in messages(bag,['/tf','/tf_static']):
  for tr in m.transforms:
   if topic=='/tf_static':tf.set_transform_static(tr,'bag')
   else:tf.set_transform(tr,'bag')
 last=-1.
 for topic,m,_ in messages(bag,['/sim/truth','/slam/pose']):
  if topic=='/slam/pose':last_pose=seconds(m.header.stamp);continue
  t=seconds(m.header.stamp)
  if t-last<1.:continue
  last=t
  try:tr=tf.lookup_transform('map','base_footprint',Time(seconds=t)).transform
  except TransformException:continue
  p=tr.translation;q=tr.rotation;estimate=transform([p.x,p.y,p.z],[q.x,q.y,q.z,q.w]);truth=pose_matrix(next(r.pose for r in m.robots if r.robot_id==0))
  h=math.atan2(estimate[1,0],estimate[0,0])-math.atan2(truth[1,0],truth[0,0]);h=math.atan2(math.sin(h),math.cos(h))
  rows.append(dict(stamp=t,position_error_m=float(np.linalg.norm(estimate[:2,3]-truth[:2,3])),heading_error_rad=h,last_slam_pose_stamp=last_pose,map_xy=estimate[:2,3].tolist(),world_xy=truth[:2,3].tolist()))
 result={'samples':rows,'maximum_position_error_m':max(r['position_error_m'] for r in rows),'maximum_heading_error_rad':max(abs(r['heading_error_rad']) for r in rows),'note':'Map is the saved sensor map aligned to arena at E0; compare propagated map/base TF, not only sporadic AMCL pose updates.'}
 Path(output).write_text(json.dumps(result,indent=2)+'\n');print(json.dumps({k:v for k,v in result.items() if k!='samples'}));print(json.dumps([min(rows,key=lambda r:abs(r['stamp']-t)) for t in (20,35,45,65,90)]))
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('bag');p.add_argument('output');a=p.parse_args();report(a.bag,a.output)
