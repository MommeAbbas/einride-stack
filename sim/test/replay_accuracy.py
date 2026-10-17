#!/usr/bin/env python3
"""Score offline posterior replay against recorded truth; not a production input."""
import argparse,csv,json
from pathlib import Path
import numpy as np
from scipy.optimize import linear_sum_assignment
from area_check import messages,seconds
from sim.history import PoseHistory
from sim.detector_node import pose_matrix
from sim.detector import transform
from tf2_ros import Buffer
from rclpy.duration import Duration
from rclpy.time import Time

def report(bag,cues,replay,out):
 history=PoseHistory(10000.);tf=Buffer(cache_time=Duration(seconds=10000))
 for topic,m,_ in messages(bag,['/tf','/tf_static']):
  for tr in m.transforms:
   if topic=='/tf_static':tf.set_transform_static(tr,'bag')
   else:tf.set_transform(tr,'bag')
 for _,m,_ in messages(bag,['/sim/truth']):history.add(seconds(m.header.stamp),{r.robot_id:pose_matrix(r.pose) for r in m.robots})
 rows=list(csv.DictReader(Path(cues).open())); frames=json.loads(Path(replay).with_suffix('.camera.json').read_text());errors=[]
 for f in frames:
  t=f['stamp'];truth=history.at(t,.2)
  if truth is None:continue
  observed={int(r['robot_id']) for r in rows if abs(float(r['stamp'])-t)<.001}
  tr=tf.lookup_transform('map','base_footprint',Time(seconds=t)).transform
  q=tr.rotation;v=tr.translation;world_map=truth[0]@np.linalg.inv(transform([v.x,v.y,v.z],[q.x,q.y,q.z,q.w]))
  ids=sorted(i for i in truth if i);tracks=f['tracks']
  means=[world_map[:2,:2]@np.array(r['mean'])+world_map[:2,3] for r in tracks]
  cost=np.array([[np.linalg.norm(mean-truth[i][:2,3]) for i in ids] for mean in means])
  ii,jj=linear_sum_assignment(cost)
  for i,j in zip(ii,jj):
   if ids[j] in observed and tracks[i]['confirmed']:errors.append({'stamp':t,'track':int(i),'truth_id':ids[j],'error_m':float(cost[i,j])})
 a=np.array([r['error_m'] for r in errors]);result={'replay':str(replay),'samples':len(a),'camera_confirmed_posterior_rmse_m':float(np.sqrt(np.mean(a*a))) if len(a) else None,'median_m':float(np.median(a)) if len(a) else None,'max_m':float(a.max()) if len(a) else None,'limitation':'Map-to-world evaluation alignment matches metrics_node (true ego / estimated map ego); sample-depth offline replay with preloaded TF; immediate camera posteriors, not real-time end-to-end predictions.'}
 out=Path(out);out.parent.mkdir(parents=True,exist_ok=True);out.write_text(json.dumps(result,indent=2)+'\n');out.with_suffix('.csv').write_text('stamp,track,truth_id,error_m\n'+''.join(','.join(str(r[k]) for k in ('stamp','track','truth_id','error_m'))+'\n' for r in errors));print(json.dumps(result,indent=2))
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('bag');p.add_argument('cues');p.add_argument('replay');p.add_argument('output');a=p.parse_args();report(a.bag,a.cues,a.replay,a.output)
