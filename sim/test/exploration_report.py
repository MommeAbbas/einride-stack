#!/usr/bin/env python3
"""Offline E0 acceptance and coverage timing; oracle map is evaluation ONLY."""
import argparse,json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from scipy.ndimage import binary_fill_holes
from area_check import messages,seconds,localisation_summary
from costmap_evidence import aligned_static

def report(directory,output):
    p=Path(directory);out=Path(output);out.mkdir(parents=True,exist_ok=True)
    result=json.loads((p/('result.json' if (p/'result.json').exists() else 'partial_result.json')).read_text());start=result['start_sim_s']
    oracle=None;last=None;coverage=[];traj=[];goals=[];full=None;eight=None;frontiers=None;map_msg=None
    for topic,m,t in messages(p,['/sim/map','/map','/slam/docks','/sim/truth','/control/request','/explore/status']):
        if topic=='/sim/map':oracle=m
        elif topic=='/map' and oracle is not None:
            map_msg=m
            truth=np.asarray(oracle.data).reshape(oracle.info.height,oracle.info.width)
            # Only enclosed arena interior, excluding outside the wall rectangle.
            interior=binary_fill_holes(truth>0)&(truth==0)
            mapped,valid=aligned_static(oracle,m)
            fraction=float(np.mean((mapped[interior]>=0)&valid[interior]))
            coverage.append([t,fraction])
            if full is None and fraction>=.99:full=t
        elif topic=='/slam/docks':
            last=m
            if m.ready and eight is None:eight=seconds(m.header.stamp)
        elif topic=='/sim/truth':
            for r in m.robots:
                if r.robot_id==0:traj.append([r.pose.position.x,r.pose.position.y])
        elif topic=='/control/request' and m.mode==m.DRIVE and m.trajectory.points:
            q=m.trajectory.points[-1]
            if not goals or goals[-1][:2]!=[q.x,q.y]:goals.append([q.x,q.y,m.reason])
        elif topic=='/explore/status' and m.status==m.EXPLORATION_COMPLETE and frontiers is None:frontiers=t
    log=(p/'stack.log').read_text(errors='replace')
    summary={'passed':bool(last and last.ready and (p/'slam_snapshot.json').is_file()),'duration_sim_s':result['duration_sim_s'],'rtf':result['real_time_factor'],
        'time_to_full_map_99pct_sim_s':full-start if full else None,'coverage_definition':'99% of oracle free interior cells known in aligned sensor map; evaluation only, not geometric accuracy',
        'max_known_free_interior_fraction':max((c[1] for c in coverage),default=None),'time_to_frontier_exhaustion_sim_s':frontiers-start if frontiers else None,
        'time_to_eight_valid_docks_sim_s':eight-start if eight else None,'valid_docks':[d.dock_id for d in last.docks if d.valid] if last else [],
        'observed_docks':[d.dock_id for d in last.docks if seconds(d.last_observed)>0] if last else [],'snapshot_saved':(p/'slam_snapshot.json').is_file(),
        'nav2_progress_failures':log.count('[controller_server]: Failed to make progress'),'drive_goals':len(goals),'localisation':localisation_summary(p)}
    (out/(p.parent.name+'.json')).write_text(json.dumps(summary,indent=2)+'\n')
    fig,ax=plt.subplots(figsize=(8,8))
    if map_msg:
        i=map_msg.info;data=np.asarray(map_msg.data).reshape(i.height,i.width)
        ax.imshow(data,origin='lower',extent=[i.origin.position.x,i.origin.position.x+i.width*i.resolution,i.origin.position.y,i.origin.position.y+i.height*i.resolution],cmap='gray_r',vmin=-1,vmax=100)
    if traj:
        xy=np.array(traj);ax.plot(xy[:,0],xy[:,1],label='ego truth (evaluation)',linewidth=1)
    if goals:
        xy=np.array([g[:2] for g in goals]);ax.scatter(xy[:,0],xy[:,1],marker='x',label='Nav2 goals')
    if last:
        for d in last.docks:
            if seconds(d.last_observed)>0:ax.text(d.tag_pose.pose.position.x,d.tag_pose.pose.position.y,chr(65+d.dock_id),color='green' if d.valid else 'red')
    ax.set(xlabel='x [m]',ylabel='y [m]',title=p.parent.name);ax.legend();ax.set_aspect('equal');fig.tight_layout();fig.savefig(out/(p.parent.name+'.png'),dpi=160);plt.close(fig)
    print(json.dumps(summary,indent=2))

if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('directory');parser.add_argument('output');args=parser.parse_args();report(args.directory,args.output)
