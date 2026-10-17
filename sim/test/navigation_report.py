#!/usr/bin/env python3
"""Recorded Nav2 progress evidence. Truth is used here only for evaluation."""
import argparse,csv,json,math,re
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from area_check import messages,seconds
from costmap_evidence import aligned_static


def report(directory,output):
    p=Path(directory);out=Path(output);out.mkdir(parents=True,exist_ok=True)
    trajectories={i:[] for i in range(4)};truth={};pose=None;goal=None;goal_id='';goals=[]
    behaviour='';plan=None;plan_at=-math.inf;goal_at=-math.inf;planner_failed=-math.inf
    failures=[];collisions=[];map_msg=None;last_request=None;path_samples=[]
    command=None;raw_command=None;guard=None;goal_yaw=None;pose_yaw=None;extra_points=np.empty((0,2))
    def state(t):
        ego=truth.get(0);xy=pose if pose is not None else ego
        near=min((math.dist(ego,v) for i,v in truth.items() if i and ego),default=None)
        def flag(name):
            match=re.search(name+r'=(true|false)',behaviour)
            return match.group(1)=='true' if match else None
        return dict(sim_time=t,our_x=xy[0] if xy else None,our_y=xy[1] if xy else None,
            goal_id=goal_id,goal_x=goal[0] if goal else None,goal_y=goal[1] if goal else None,
            nearest_traffic_m=near,dock_occupied=flag('dock_occupied'),
            planner_returned_path=bool(plan is not None and len(plan.poses)>0 and plan_at>=goal_at and plan_at>planner_failed),
            plan_age_s=t-plan_at if math.isfinite(plan_at) else None,standoff_active=flag('standoff_active'),behaviour=behaviour,
            goal_distance_m=math.dist(xy,goal) if xy and goal else None,
            goal_heading_error_rad=math.atan2(math.sin(goal_yaw-pose_yaw),math.cos(goal_yaw-pose_yaw)) if goal_yaw is not None and pose_yaw is not None else None,
            final_command=command,nav2_command=raw_command,guard_action=guard,
            extra_lethal_cells=len(extra_points),nearest_extra_lethal_m=float(np.min(np.linalg.norm(extra_points-np.asarray(xy),axis=1))) if xy and len(extra_points) else None)
    topics=['/cmd_vel','/control/nav2_cmd','/control/collision_state','/global_costmap/costmap','/map','/sim/truth','/slam/pose','/control/request','/planning/behaviour','/plan','/rosout','/behavior_tree_log','/sim/collisions']
    for topic,m,t in messages(p,topics):
        if topic=='/cmd_vel':command=[m.linear.x,m.angular.z]
        elif topic=='/control/nav2_cmd':raw_command=[m.linear.x,m.angular.z]
        elif topic=='/control/collision_state':guard=m.action_type
        elif topic=='/global_costmap/costmap' and map_msg is not None:
            ref,_=aligned_static(m,map_msg);data=np.asarray(m.data).reshape(ref.shape)
            yy,xx=np.nonzero((data==100)&(ref==0));q=m.info.origin.orientation;a=2*math.atan2(q.z,q.w)
            x=(xx+.5)*m.info.resolution;y=(yy+.5)*m.info.resolution
            extra_points=np.c_[m.info.origin.position.x+math.cos(a)*x-math.sin(a)*y,m.info.origin.position.y+math.sin(a)*x+math.cos(a)*y]
        elif topic=='/map':map_msg=m
        elif topic=='/sim/truth':
            for r in m.robots:
                xy=[r.pose.position.x,r.pose.position.y];truth[r.robot_id]=xy;trajectories.setdefault(r.robot_id,[]).append([t,*xy])
        elif topic=='/slam/pose':
            pose=[m.pose.pose.position.x,m.pose.pose.position.y];q=m.pose.pose.orientation;pose_yaw=2*math.atan2(q.z,q.w)
        elif topic=='/control/request':
            goal_id=m.goal_id
            if m.mode==m.DRIVE and m.trajectory.points:
                q=m.trajectory.points[-1];new=[q.x,q.y];goal_yaw=q.yaw
                if new!=goal:goal=new;goal_at=t;goals.append([t,*goal,m.reason])
            last_request=m
        elif topic=='/planning/behaviour':behaviour=m.data
        elif topic=='/plan':plan=m;plan_at=t
        elif topic=='/behavior_tree_log':
            for event in m.event_log:
                if event.node_name=='ComputePathToPose' and event.current_status=='FAILURE':planner_failed=t
        elif topic=='/rosout' and 'Failed to make progress' in m.msg and 'controller_server' in m.name:
            row=state(t);row['marker']='F'+str(len(failures)+1);failures.append(row)
            if plan:path_samples.append([(q.pose.position.x,q.pose.position.y) for q in plan.poses])
            else:path_samples.append([])
        elif topic=='/sim/collisions' and not m.forced_test:
            # All collisions are numbered; distinguish ego from traffic-only.
            xy=truth.get(m.first_robot)
            collisions.append(dict(marker='C'+str(len(collisions)+1),sim_time=seconds(m.header.stamp),x=xy[0] if xy else None,y=xy[1] if xy else None,robots=[m.first_robot,m.second_robot]))
    log_count=(p/'stack.log').read_text(errors='replace').count('[controller_server]: Failed to make progress')
    summary={'run':p.parent.name,'progress_failure_count_in_log':log_count,'progress_failure_rows_from_rosout':len(failures),
        'evidence_complete':len(failures)==log_count,'failures':failures,'collisions':collisions,
        'goal_proximity_summary':{'failures_within_0_2m':sum(r['goal_distance_m'] is not None and r['goal_distance_m']<.2 for r in failures),'threshold_note':'0.2 m is an offline diagnostic bin, not a runtime or acceptance tolerance'},
        'definition':'Positions/goals use map frame; nearest traffic uses synchronous latest true poses. Path yes means a nonempty plan received for current coordinates after latest ComputePathToPose failure. Null occupancy means no decision was recorded. Timestamps are rosbag ROS-clock receipt times.'}
    (out/(p.parent.name+'_navigation.json')).write_text(json.dumps(summary,indent=2)+'\n')
    if failures:
        with (out/(p.parent.name+'_failures.csv')).open('w') as f:
            writer=csv.DictWriter(f,fieldnames=list(failures[0]));writer.writeheader();writer.writerows(failures)
    else:(out/(p.parent.name+'_failures.csv')).write_text('marker,sim_time,our_x,our_y,goal_id,goal_x,goal_y,nearest_traffic_m,dock_occupied,planner_returned_path,plan_age_s,standoff_active,behaviour\n')
    lines=['| Marker | Sim s | Ego position m | Current goal m | Nearest traffic m | Dock occupied | Path returned | Standoff |','|---|---:|---|---|---:|---|---|---|']
    def yn(v):return 'unknown' if v is None else 'yes' if v else 'no'
    def num(v):return 'unknown' if v is None else f'{v:.3f}'
    for r in failures:
        lines.append(f"| {r['marker']} | {r['sim_time']:.3f} | {num(r['our_x'])}, {num(r['our_y'])} | {num(r['goal_x'])}, {num(r['goal_y'])} | {num(r['nearest_traffic_m'])} | {yn(r['dock_occupied'])} | {yn(r['planner_returned_path'])} | {yn(r['standoff_active'])} |")
    (out/(p.parent.name+'_failures.md')).write_text('\n'.join(lines)+'\n')
    fig,ax=plt.subplots(figsize=(11,9))
    if map_msg:
        i=map_msg.info;data=np.asarray(map_msg.data).reshape(i.height,i.width)
        ax.imshow(data,origin='lower',extent=[i.origin.position.x,i.origin.position.x+i.width*i.resolution,i.origin.position.y,i.origin.position.y+i.height*i.resolution],cmap='gray_r',vmin=-1,vmax=100,alpha=.6)
    colors=['tab:blue','tab:orange','tab:green','tab:purple']
    for ident,values in trajectories.items():
        if not values:continue
        xy=np.array(values);ax.plot(xy[:,1],xy[:,2],color=colors[ident%4],alpha=.85 if ident==0 else .5,linewidth=1.4 if ident==0 else .8,label='ego' if ident==0 else f'traffic {ident}')
    if goals:
        xy=np.array([g[1:3] for g in goals]);ax.scatter(xy[:,0],xy[:,1],marker='*',s=100,color='navy',label='ego goals')
    for j,row in enumerate(failures):
        if row['our_x'] is None:continue
        ax.scatter(row['our_x'],row['our_y'],marker='x',color='black',s=30)
        ax.annotate(row['marker'],(row['our_x'],row['our_y']),xytext=(8+(j%3)*17,8+(j%5)*11),textcoords='offset points',fontsize=7,arrowprops=dict(arrowstyle='-',lw=.4))
    for row in collisions:
        if row['x'] is None:continue
        ax.scatter(row['x'],row['y'],facecolors='none',edgecolors='red',s=100)
        ax.annotate(row['marker'],(row['x'],row['y']),xytext=(-25,-15),textcoords='offset points',color='red',fontsize=8)
    ax.set(xlabel='map x [m]',ylabel='map y [m]',title=f'{p.parent.name}: F = progress failure, C = collision');ax.set_aspect('equal');ax.legend(loc='upper left',bbox_to_anchor=(1.01,1));fig.tight_layout();fig.savefig(out/(p.parent.name+'_trajectories.png'),dpi=160);plt.close(fig)
    print(json.dumps({'log_failures':log_count,'rows':len(failures),'collisions':len(collisions),'output':str(out)},indent=2))
    return summary

if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('directory');parser.add_argument('output');args=parser.parse_args();report(args.directory,args.output)
