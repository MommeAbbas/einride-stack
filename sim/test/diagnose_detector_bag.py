#!/usr/bin/env python3
"""Reconstruct legacy detector frame eligibility from bag truth and static TF.

Usage: after normal ROS setup, python3 diagnose_detector_bag.py RUN_DIRECTORY.
Evaluation only; the recorded CameraInfo may be absent, so this checks centre
FOV/range/map visibility, not historical pixel clipping or stochastic misses.
"""
import sys,csv,json
from pathlib import Path
import numpy as np,yaml
sys.path.insert(0,str(Path(__file__).resolve().parent))
from area_check import messages,seconds
from sim.detector import transform
from sim.detector_node import pose_matrix
from sim.history import PoseHistory
from sim.grid import Grid
p=Path(sys.argv[1]);params=yaml.safe_load((p/'params.yaml').read_text())['/**']['ros__parameters']
h=PoseHistory(1e9);edges={};frames=[]
for topic,m,_ in messages(p,['/sim/truth','/tf_static','/sim/detections']):
 if topic=='/sim/truth':h.add(seconds(m.header.stamp),{r.robot_id:pose_matrix(r.pose) for r in m.robots})
 elif topic=='/sim/detections':frames.append((seconds(m.header.stamp),m.header.frame_id,len(m.detections)))
 else:
  for tr in m.transforms:
   a,b=tr.header.frame_id,tr.child_frame_id;t,q=tr.transform.translation,tr.transform.rotation
   mat=transform([t.x,t.y,t.z],[q.x,q.y,q.z,q.w]);edges.setdefault(a,{})[b]=mat;edges.setdefault(b,{})[a]=np.linalg.inv(mat)
def lookup(frame):
 todo=[('base_footprint',np.eye(4))];seen=set()
 while todo:
  a,mat=todo.pop()
  if a==frame:return mat
  if a in seen:continue
  seen.add(a);todo.extend((b,mat@v) for b,v in edges.get(a,{}).items() if b not in seen)
 raise ValueError(frame)
a=np.load(p/'map.npz');grid=Grid(a['occupied'],float(a['resolution']),a['origin'])
rows=[]
for stamp,frame,boxes in frames:
 poses=h.at(stamp,params['detector_pose_tolerance'])
 if not poses or 0 not in poses:continue
 camera=poses[0]@lookup(frame)
 for ident,pose in poses.items():
  if not ident:continue
  centre=np.linalg.inv(camera)@pose@np.array([0,0,params['robot_height']/2,1])
  fov=centre[2]>0 and abs(np.arctan2(centre[0],centre[2]))<=params['detector_hfov']/2 and abs(np.arctan2(centre[1],centre[2]))<=params['detector_vfov']/2
  in_range=centre[2]>params['detector_min_range'] and np.linalg.norm(centre[:3])<=params['detector_max_range']
  clear=not params['detector_map_occlusion'] or grid.clear_line(camera[:2,3],pose[:2,3])
  rows.append(dict(stamp=stamp,robot_id=ident,in_fov=bool(fov),in_range=bool(in_range),unoccluded=bool(clear),visible=bool(fov and in_range and clear),frame_boxes=boxes))
with (p/'detector_visibility_reconstructed.csv').open('w') as f:
 w=csv.DictWriter(f,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)
summary={'frames':len(frames),'evaluated_robot_frames':len(rows),'visible_robot_frames':sum(r['visible'] for r in rows),'visible_without_boxes':sum(r['visible'] and not r['frame_boxes'] for r in rows),'in_fov':sum(r['in_fov'] for r in rows),'in_range':sum(r['in_range'] for r in rows),'unoccluded':sum(r['unoccluded'] for r in rows),'limitation':'Historical bag lacks CameraInfo; this reconstructs centre FOV/range/map visibility, not pixel-box clipping.'}
(p/'detector_visibility_summary.json').write_text(json.dumps(summary,indent=2)+'\n');print(summary)
