#!/usr/bin/env python3
"""Offline production-node replay, no simulator run; sampled recorded depth only."""
import argparse,json,yaml
from pathlib import Path
from types import SimpleNamespace
import numpy as np
import rclpy
from rclpy.duration import Duration
from tf2_ros import Buffer
from sensor_msgs.msg import Image
from sensor_msgs_py.point_cloud2 import read_points_numpy
from rosidl_runtime_py.convert import message_to_ordereddict
from area_check import messages,seconds
from perception.perception_node import PerceptionNode


def replay(directory,output,until=None):
 rclpy.init();node=PerceptionNode()
 # Historical boxes were generated with their recorded prism height.
 node.params['robot_height']=yaml.safe_load((Path(directory)/'perception.yaml').read_text())['/**']['ros__parameters']['robot_height']
 node.buffer=Buffer(cache_time=Duration(seconds=10000))
 for topic,m,_ in messages(directory,['/tf','/tf_static']):
  for tr in m.transforms:
   if topic=='/tf_static':node.buffer.set_transform_static(tr,'bag')
   else:node.buffer.set_transform(tr,'bag')
 current=[0.];node.now=lambda:current[0];original_tf=node.tf
 node.tf=lambda target,source,t:original_tf(target,source,t or current[0])
 full_depth=any(x['topic_metadata']['name']=='/oak/stereo/image_raw' and x['message_count']>0 for x in yaml.safe_load((Path(directory)/'bag/metadata.yaml').read_text())['rosbag2_bagfile_information']['topics_with_message_count'])
 results=[];captured=[];old_process=node.process_camera
 def capture(t,m):
  diagnostics=[]
  old_update=node.belief.update if node.belief else None
  if old_update:
   def update(items,*args,**kwargs):
    before=[{'mean':node.belief.grid.to_map(w@c[:,:2]).tolist(),'support':node.belief.last_support[i]} for i,(c,w) in enumerate(zip(node.belief.clouds,node.belief.weights))]
    result=old_update(items,*args,**kwargs)
    diagnostics.append({'measurements':[node.belief.grid.to_map(o+d*np.array([np.cos(b),np.sin(b)])).tolist() for o,b,d,s,e in items if d is not None],'before':before})
    return result
   node.belief.update=update
  try:old_process(t,m)
  finally:
   if old_update:node.belief.update=old_update
  if m.detections and node.belief:
   from perception.output import moments
   captured.append({'stamp':t,'diagnostics':diagnostics,'tracks':[{'mean':node.belief.grid.to_map(moments(c[:,:2],w,node.p('covariance_epsilon'))[0]).tolist(),'confirmed':node.belief.last_support[i]==t} for i,(c,w) in enumerate(zip(node.belief.clouds,node.belief.weights))]})
 node.process_camera=capture
 node.pub=SimpleNamespace(publish=lambda m:results.append(message_to_ordereddict(m)))
 for name in ('camera_tracks','health','dock_pub','understanding','debug','diagnostics'):setattr(node,name,SimpleNamespace(publish=lambda m:None))
 last=-1.;topics=['/map','/slam/docks','/planning/saga_state','/planning/trajectory','/odom','/oak/rgb/camera_info','/control/depth_points','/oak/stereo/image_raw','/scan','/sim/detections','/clock']
 try:
  for topic,m,t in messages(directory,topics):
   if until is not None and t>until:break
   current[0]=t
   if topic=='/map':node.on_map(m)
   elif topic=='/slam/docks':node.on_docks(m)
   elif topic=='/planning/saga_state':node.saga=m
   elif topic=='/planning/trajectory':node.plan=m
   elif topic=='/odom':node.odometry.append(m)
   elif topic=='/oak/rgb/camera_info':node.camera=m
   elif topic=='/scan':node.on_scan(m)
   elif topic=='/sim/detections':node.on_boxes(m)
   elif topic=='/oak/stereo/image_raw':node.depths.append(m)
   elif topic=='/control/depth_points' and node.camera and not full_depth:
    k=node.camera.k;xyz=read_points_numpy(m,field_names=['x','y','z'],skip_nans=True)
    uv=np.rint(xyz[:,:2]/xyz[:,2,None]*[k[0],k[4]]+[k[2],k[5]]).astype(int)
    valid=(uv[:,0]>=0)&(uv[:,0]<node.camera.width)&(uv[:,1]>=0)&(uv[:,1]<node.camera.height)
    depth=np.full((node.camera.height,node.camera.width),np.nan,dtype='<f4');depth[uv[valid,1],uv[valid,0]]=xyz[valid,2]
    image=Image(header=m.header,height=node.camera.height,width=node.camera.width,encoding='32FC1',step=node.camera.width*4,data=depth.tobytes());node.depths.append(image)
   elif topic=='/clock' and t-last>=1/node.p('filter_rate'):
    node.tick();last=t
  output=Path(output);output.parent.mkdir(parents=True,exist_ok=True)
  output.write_text(json.dumps(results)+'\n');output.with_suffix('.camera.json').write_text(json.dumps(captured)+'\n')
  print(json.dumps({'predictions':len(results),'camera_frames_with_boxes':len(captured),'dropped_tf':node.dropped_tf,'dropped_late':node.queue.dropped,'output':str(output)}))
 finally:node.destroy_node();rclpy.shutdown()

if __name__=='__main__':
 a=argparse.ArgumentParser();a.add_argument('directory');a.add_argument('output');a.add_argument('--until',type=float);args=a.parse_args();replay(args.directory,args.output,args.until)
