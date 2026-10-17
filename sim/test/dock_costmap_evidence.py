#!/usr/bin/env python3
"""Offline occupied-dock evidence; simulator truth only for evaluation."""
import argparse,json,math
from pathlib import Path
import numpy as np
from scipy.ndimage import distance_transform_edt
from area_check import messages,seconds,localisation_summary
from planning.dock_occupancy import dynamic_points,sample


def report(directory,dock_id,times,output):
    static=None;docks=None;truth=None;pose=None;rows=[];index=0
    for topic,m,t in messages(directory,['/map','/slam/docks','/sim/truth','/slam/pose','/global_costmap/costmap']):
        if topic=='/map':static=m
        elif topic=='/slam/docks':docks=m
        elif topic=='/sim/truth':truth=m
        elif topic=='/slam/pose':pose=m.pose.pose
        elif static is not None and docks is not None and index<len(times) and t>=times[index]:
            centre=np.array([docks.docks[dock_id].docking_pose.pose.position.x,docks.docks[dock_id].docking_pose.pose.position.y])
            points=dynamic_points(m,static);points=points[np.linalg.norm(points-centre,axis=1)<=.5]
            raw=np.asarray(static.data).reshape(static.info.height,static.info.width)
            distances=distance_transform_edt(raw<50)*static.info.resolution
            xy=(points-[static.info.origin.position.x,static.info.origin.position.y])/static.info.resolution
            cells=np.floor(xy).astype(int);dd=distances[cells[:,1],cells[:,0]] if len(points) else []
            rows.append(dict(sim_time=t,dock_map=centre.tolist(),lethal_free_static_cells=len(points),
                marks=points.tolist(),distance_to_static_wall_m=list(map(float,dd)),
                ego_map=[pose.position.x,pose.position.y] if pose else None,
                true_robots=[[r.robot_id,r.pose.position.x,r.pose.position.y] for r in truth.robots] if truth else []))
            index+=1
    result={'snapshots':rows,'localisation':localisation_summary(directory),'note':'0.5 m diagnostic radius matches dock admission. Map/world points retained explicitly; only scoring uses truth.'}
    Path(output).write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2))

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('directory');p.add_argument('output');p.add_argument('--dock',type=int,required=True);p.add_argument('--times',type=float,nargs='+',required=True);a=p.parse_args();report(a.directory,a.dock,a.times,a.output)
