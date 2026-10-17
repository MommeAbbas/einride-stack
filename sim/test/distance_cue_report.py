#!/usr/bin/env python3
"""Evaluate recorded detector range cues; truth is used ONLY to score/associate boxes.
Old bags lack raw depth; sampled guard cloud is projected back to sparse image.
"""
import argparse,json,math,bisect,csv
from pathlib import Path
import numpy as np
import yaml
from rclpy.duration import Duration
from rclpy.time import Time
from tf2_ros import Buffer
from sensor_msgs_py.point_cloud2 import read_points_numpy
from scipy.spatial.transform import Rotation
from scipy.optimize import linear_sum_assignment
from area_check import messages,seconds
from perception.measurements import camera_cues
from perception.perception_node import image_array
from sim.detector import project_box,transform
from sim.history import PoseHistory
from sim.detector_node import pose_matrix


def report(directory,output,replay=None):
 p=Path(directory);out=Path(output);out.mkdir(parents=True,exist_ok=True)
 params=yaml.safe_load((p/'perception.yaml').read_text())['/**']['ros__parameters']
 sim=yaml.safe_load((p/'params.yaml').read_text())['/**']['ros__parameters']
 tf=Buffer(cache_time=Duration(seconds=10000));history=PoseHistory(10000.);boxes=[];clouds=[];camera=None;predictions=[]
 for topic,m,_ in messages(p,['/tf','/tf_static','/sim/truth','/sim/detections','/control/depth_points','/oak/rgb/camera_info','/perception/predictions']):
  if topic in ('/tf','/tf_static'):
   for tr in m.transforms:
    if topic=='/tf_static':tf.set_transform_static(tr,'bag')
    else:tf.set_transform(tr,'bag')
  elif topic=='/sim/truth':history.add(seconds(m.header.stamp),{r.robot_id:pose_matrix(r.pose) for r in m.robots})
  elif topic=='/sim/detections' and m.detections:boxes.append(m)
  elif topic=='/control/depth_points':clouds.append(m)
  elif topic=='/oak/rgb/camera_info':camera=m
  elif topic=='/perception/predictions':predictions.append(m)
 times=[seconds(c.header.stamp) for c in clouds];rows=[];skipped=0
 # Stream full depth on new recordings; retain only the adjacent captures.
 raw=iter(messages(p,['/oak/stereo/image_raw']));previous=None
 following=next(raw,None);raw_samples=0;sampled_samples=0
 boxes.sort(key=lambda m:seconds(m.header.stamp))
 k=np.array(camera.k).reshape(3,3);intr=(k[0,0],k[1,1],k[0,2],k[1,2])
 for msg in boxes:
  t=seconds(msg.header.stamp);poses=history.at(t,sim['detector_pose_tolerance'])
  if not poses or 0 not in poses:skipped+=1;continue
  tr=tf.lookup_transform('base_footprint',msg.header.frame_id,Time.from_msg(msg.header.stamp)).transform
  q=tr.rotation;v=tr.translation;extr=transform([v.x,v.y,v.z],[q.x,q.y,q.z,q.w]);cam=poses[0]@extr
  ids=[];references=[]
  for ident,pose in poses.items():
   if not ident:continue
   b=project_box(pose,cam,k,[camera.width,camera.height],[sim['robot_length'],sim['robot_width'],sim['robot_height']],sim['detector_hfov'],sim['detector_vfov'],sim['detector_min_range'],sim['detector_max_range'])
   if b is not None:ids.append(ident);references.append([(b[0]+b[2])/2,(b[1]+b[3])/2])
  actual=np.array([[d.bbox.center.position.x,d.bbox.center.position.y] for d in msg.detections])
  if not references:skipped+=1;continue
  ii,jj=linear_sum_assignment(np.linalg.norm(actual[:,None,:]-np.array(references)[None,:,:],axis=2))
  near=bisect.bisect_left(times,t);candidates=[i for i in (near-1,near) if 0<=i<len(times)]
  ci=min(candidates,key=lambda i:abs(times[i]-t)) if candidates else None
  depth=None
  if ci is not None and abs(times[ci]-t)<=params['depth_sync_tolerance']:
   cloud=clouds[ci];xyz=read_points_numpy(cloud,field_names=['x','y','z'],skip_nans=True)
   uv=np.rint(xyz[:,:2]/xyz[:,2,None]*[intr[0],intr[1]]+[intr[2],intr[3]]).astype(int)
   valid=(uv[:,0]>=0)&(uv[:,0]<camera.width)&(uv[:,1]>=0)&(uv[:,1]<camera.height)
   depth=np.full((camera.height,camera.width),np.nan);depth[uv[valid,1],uv[valid,0]]=xyz[valid,2]
  while following is not None and seconds(following[1].header.stamp)<t:
   previous=following;following=next(raw,None)
  adjacent=[item[1] for item in (previous,following) if item is not None]
  nearest=min(adjacent,key=lambda m:abs(seconds(m.header.stamp)-t)) if adjacent else None
  used_raw=nearest is not None and abs(seconds(nearest.header.stamp)-t)<=params['depth_sync_tolerance']
  if used_raw:depth=image_array(nearest);raw_samples+=1
  elif depth is not None:sampled_samples+=1
  for i,j in zip(ii,jj):
   d=msg.detections[i];box=(d.bbox.center.position.x,d.bbox.center.position.y,d.bbox.size_x,d.bbox.size_y);truth=poses[ids[j]][:2,3]
   row=dict(stamp=t,robot_id=ids[j],box_pixel_match=float(np.linalg.norm(actual[i]-references[j])),true_range=float(np.linalg.norm(truth-cam[:2,3])),depth_sync_error=abs(seconds(nearest.header.stamp)-t) if used_raw else abs(times[ci]-t) if ci is not None else None,depth_source='raw_image' if used_raw else 'sampled_cloud')
   for cue in ('floor','height','depth','fused'):
    cfg=dict(params)
    for name in ('floor','height','depth'):cfg[name+'_cue']=cue in (name,'fused')
    result=camera_cues(box,intr,(camera.height,camera.width),depth,cfg)
    if result is None:row[cue+'_error_m']=None;continue
    bearing,distance,sigma=result
    point=cam[:3,3]+cam[:3,:3]@np.array([-distance*math.sin(bearing),0.,distance*math.cos(bearing)])
    row[cue+'_error_m']=float(np.linalg.norm(point[:2]-truth));row[cue+'_range_m']=distance
   rows.append(row)
 summary={'recording':str(p),'boxes':len(rows),'skipped_frames':skipped,'raw_depth_frames':raw_samples,'sampled_depth_frames':sampled_samples,'depth_limit':'Use raw depth when recorded and synchronised, else stride-8 cloud fallback. Floor/height exact box replay; sampled-depth evidence is not full-image replay.','cues':{}}
 for cue in ('floor','height','depth','fused'):
  errors=[r[cue+'_error_m'] for r in rows if r[cue+'_error_m'] is not None]
  summary['cues'][cue]={'samples':len(errors),'position_rmse_m':float(np.sqrt(np.mean(np.square(errors)))) if errors else None,'median_error_m':float(np.median(errors)) if errors else None}
 if replay:
  from interfaces.msg import Predictions
  from rosidl_runtime_py.set_message import set_message_fields
  predictions=[]
  for value in json.loads(Path(replay).read_text()):
   m=Predictions();set_message_fields(m,value);predictions.append(m)
 matched=[];confirmed=[];prediction_rows=[]
 for pred in predictions:
  t=seconds(pred.header.stamp);recent=[r for r in rows if 0<=t-r['stamp']<=sim['metric_detection_frame_max_age']]
  if not recent:continue
  observed={r['robot_id'] for r in recent};poses=history.at(t,sim['detector_pose_tolerance'])
  if not poses:continue
  means=[]
  for robot in pred.robots:
   groups=[*robot.groups,robot.leftover]
   means.append(sum(g.probability*np.array([g.points[0].x,g.points[0].y]) for g in groups))
  tr=tf.lookup_transform('map','base_footprint',Time(seconds=t)).transform
  q=tr.rotation;v=tr.translation;world_map=poses[0]@np.linalg.inv(transform([v.x,v.y,v.z],[q.x,q.y,q.z,q.w]))
  means=[world_map[:2,:2]@mean+world_map[:2,3] for mean in means]
  ids=sorted(i for i in poses if i);cost=np.array([[np.linalg.norm(mean-poses[i][:2,3]) for i in ids] for mean in means])
  ii,jj=linear_sum_assignment(cost)
  while following is not None and seconds(following[1].header.stamp)<t:
   previous=following;following=next(raw,None)
  adjacent=[item[1] for item in (previous,following) if item is not None]
  nearest=min(adjacent,key=lambda m:abs(seconds(m.header.stamp)-t)) if adjacent else None
  used_raw=nearest is not None and abs(seconds(nearest.header.stamp)-t)<=params['depth_sync_tolerance']
  if used_raw:depth=image_array(nearest);raw_samples+=1
  elif depth is not None:sampled_samples+=1
  for i,j in zip(ii,jj):
   if ids[j] in observed:
    matched.append(float(cost[i,j]));prediction_rows.append(dict(stamp=t,track=int(i),truth_id=ids[j],error_m=float(cost[i,j]),confirmed=pred.robots[i].ever_confirmed))
    if pred.robots[i].ever_confirmed:confirmed.append(float(cost[i,j]))
 summary['detected_robot_prediction_error']={'samples':len(matched),'rmse_m':float(np.sqrt(np.mean(np.square(matched)))) if matched else None,'confirmed_samples':len(confirmed),'confirmed_rmse_m':float(np.sqrt(np.mean(np.square(confirmed)))) if confirmed else None,'note':'Map/world alignment matches metrics_node; all-track assignment to truth, then only truth identities observed in latest camera interval. Distinct from camera measurement error.'}
 with (out/'detected_prediction_samples.csv').open('w') as f:
  w=csv.DictWriter(f,fieldnames=['stamp','track','truth_id','error_m','confirmed']);w.writeheader();w.writerows(prediction_rows)
 (out/'distance_cues.json').write_text(json.dumps(summary,indent=2)+'\n')
 with (out/'distance_cues.csv').open('w') as f:
  fields=sorted(set().union(*(r.keys() for r in rows)));w=csv.DictWriter(f,fieldnames=fields);w.writeheader();w.writerows(rows)
 print(json.dumps(summary,indent=2))

if __name__=='__main__':
 a=argparse.ArgumentParser();a.add_argument('directory');a.add_argument('output');a.add_argument('--predictions');args=a.parse_args();report(args.directory,args.output,args.predictions)
