#!/usr/bin/env python3
"""Read-only bag diagnosis using unchanged A* parameters; never commands a robot."""
import sys,json,collections,math,bisect
from pathlib import Path
import numpy as np,yaml
from scipy.ndimage import distance_transform_edt
sys.path.insert(0,str(Path(__file__).resolve().parent))
from area_check import messages,seconds
from planning.grid import Grid
p=Path(sys.argv[1]);params=yaml.safe_load((p/'planning.yaml').read_text())['/**']['ros__parameters'];result=json.loads((p/'result.json').read_text());end=result['start_sim_s']+result['duration_sim_s']
latest={};transforms={};counts=collections.defaultdict(collections.Counter);samples=[];next_sample=10.;start=None;current_reason='';cmd_counts=collections.defaultdict(collections.Counter)
def grid(m):
 a=np.asarray(m.data).reshape(m.info.height,m.info.width);return Grid((a<0)|(a>=params['occupied_threshold']),m.info.resolution,[m.info.origin.position.x,m.info.origin.position.y]),a
for topic,m,t in messages(p,['/tf','/map','/global_costmap/costmap','/slam/pose','/slam/docks','/planning/saga_state','/planning/behaviour','/control/status','/control/request','/cmd_vel','/control/guard_input','/control/behavior_cmd','/control/collision_state','/sim/truth']):
 if t>end:continue
 latest[topic]=(m,t)
 if topic=='/tf':
  for tr in m.transforms:transforms[(tr.header.frame_id,tr.child_frame_id)]=tr
 if topic in ('/planning/behaviour','/control/status'):
  reason=m.data if topic=='/planning/behaviour' else m.reason;counts[topic][reason]+=1
 if topic in ('/cmd_vel','/control/guard_input','/control/behavior_cmd'):
  cmd_counts[topic]['samples']+=1;cmd_counts[topic]['nonzero']+=abs(m.linear.x)+abs(m.angular.z)>1e-6
 if topic!='/global_costmap/costmap' or t<next_sample:continue
 required=['/map','/slam/pose','/slam/docks','/planning/saga_state']
 if any(k not in latest for k in required):continue
 next_sample=t+10.
 if any(k not in transforms for k in [('map','odom'),('odom','base_footprint')]):continue
 a=transforms[('map','odom')];b=transforms[('odom','base_footprint')]
 angle=2*math.atan2(a.transform.rotation.z,a.transform.rotation.w)
 xy=np.array([a.transform.translation.x,a.transform.translation.y])+np.array([[math.cos(angle),-math.sin(angle)],[math.sin(angle),math.cos(angle)]])@np.array([b.transform.translation.x,b.transform.translation.y])
 yaw=angle+2*math.atan2(b.transform.rotation.z,b.transform.rotation.w)
 dock=latest['/planning/saga_state'][0].next_dock
 if dock<0:continue
 docks=latest['/slam/docks'][0];target=docks.docks[dock].approach_pose.pose.position;target=np.array([target.x,target.y])
 g,raw=grid(m);s,static=grid(latest['/map'][0]);cell=g.cell(xy);radius=params['robot_radius']+params['obstacle_clearance'];clearance=distance_transform_edt(~g.occupied)*g.resolution
 y,x=np.mgrid[:raw.shape[0],:raw.shape[1]];near=np.hypot(g.origin[0]+(x+.5)*g.resolution-xy[0],g.origin[1]+(y+.5)*g.resolution-xy[1])<=.6
 r={'sim_s':round(t,3),'pose_map_xy_yaw':[float(xy[0]),float(xy[1]),yaw],'target_dock':chr(65+dock),'target_approach_xy':target.tolist(),'live_map_age_s':t-seconds(m.header.stamp),'tf_latest_common_age_s':t-min(seconds(a.header.stamp),seconds(b.header.stamp)),'ego_cell_occupied':bool(g.occupied[cell]),'ego_clearance_m':float(clearance[cell]),'required_clearance_m':radius,'live_path_to_approach':bool(g.plan(xy,target,radius)),'static_path_to_approach':bool(s.plan(xy,target,radius)),'local_radius_m':.6,'near_free_cells':int(np.sum(near & ~g.occupied)),'near_blocked_cells':int(np.sum(near & g.occupied)),'near_unknown_cells':int(np.sum(near & (raw<0)))}
 if g.occupied.shape==s.occupied.shape and np.allclose(g.origin,s.origin) and g.resolution==s.resolution:r['near_added_blocked_vs_static']=int(np.sum(near & g.occupied & ~s.occupied))
 if '/control/status' in latest:r['control_reason']=latest['/control/status'][0].reason
 if '/planning/behaviour' in latest:r['planning_reason']=latest['/planning/behaviour'][0].data
 if '/sim/truth' in latest:
  ego=next(z for z in latest['/sim/truth'][0].robots if z.robot_id==0).pose
  r['truth_world_xy']=[ego.position.x,ego.position.y]
 samples.append(r)
out={'notes':'Read-only bag reconstruction. Pose composes latest recorded map/odom and odom/base TF (receipt-order approximation). Path tests use existing A* and unchanged clearance to current Saga dock approach; samples are not exact service-invocation times. Nearby cells counted within diagnostic 0.6 m radius. Restricted to requested run window, excludes shutdown.','reason_samples':{k:dict(v) for k,v in counts.items()},'command_samples':{k:dict(v) for k,v in cmd_counts.items()},'costmap_pose_samples':samples}
(p/'navigation_evidence.json').write_text(json.dumps(out,indent=2)+'\n')
print(json.dumps({'reasons':{k:v.most_common(8) for k,v in counts.items()},'commands':out['command_samples'],'snapshots':samples[::max(1,len(samples)//4)]},indent=2))
