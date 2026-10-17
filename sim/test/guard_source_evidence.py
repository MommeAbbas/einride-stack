#!/usr/bin/env python3
"""Offline coincidence audit: measured/commanded stop with in-footprint returns.

Installed tf2 reprojects each recorded source to the command-time base frame,
through odom, matching base-shift correction. No live ROS node or motion output.
Intervals are observations at guard-input stamps, not proof of causality.
"""
import argparse, ast, json, math
from pathlib import Path
import numpy as np
import yaml
from rclpy.duration import Duration
from rclpy.time import Time
from tf2_ros import Buffer, TransformException
from sensor_msgs_py.point_cloud2 import read_points_numpy
from area_check import messages, seconds


def transform_points(points, transform):
    q=transform.rotation
    x,y,z,w=q.x,q.y,q.z,q.w
    r=np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
    t=transform.translation
    return points @ r.T + [t.x,t.y,t.z]


def inside_polygon(points, polygon):
    # Same odd/even crossing convention as installed Polygon::isPointInside.
    x,y=points[:,0],points[:,1];inside=np.zeros(len(points),dtype=bool)
    previous=polygon[-1]
    for current in polygon:
        x1,y1=current;x2,y2=previous
        if y1!=y2:
            crossing=((y1>y)!=(y2>y)) & (x < (x2-x1)*(y-y1)/(y2-y1)+x1)
            inside ^= crossing
        previous=current
    return inside


def check(directory, control_config):
    p=Path(directory);meta=json.loads((p/'result.json').read_text())
    params=yaml.safe_load((p/'params.yaml').read_text())['/**']['ros__parameters']
    config=yaml.safe_load(Path(control_config).read_text())
    c=config['collision_monitor']['ros__parameters'];own=config['/**']['ros__parameters']
    local=config['local_costmap']['local_costmap']['ros__parameters']
    padding=local['footprint_padding']
    polygon=[[x+math.copysign(padding,x),y+math.copysign(padding,y)] for x,y in ast.literal_eval(local['footprint'])]
    begin=meta['start_sim_s'];end=begin+meta['duration_sim_s']
    buffer=Buffer(cache_time=Duration(seconds=end+c['source_timeout']))
    for topic,m,_ in messages(p,['/tf','/tf_static']):
        for tr in m.transforms:
            if topic=='/tf_static':buffer.set_transform_static(tr,'recorded')
            else:buffer.set_transform(tr,'recorded')
    last={};points={};intervals=[];active=None;unresolved={'lidar':0,'depth':0,'odom':0,'command':0}
    inspected=0;source_frames={'lidar':0,'depth':0};stopped_samples=0;commanded_zero_stopped_samples=0
    for topic,m,t in messages(p,['/scan','/control/depth_points','/odom','/cmd_vel','/control/guard_input']):
        last[topic]=m
        if topic=='/scan':
            ranges=np.asarray(m.ranges);angles=m.angle_min+np.arange(len(ranges))*m.angle_increment
            good=np.isfinite(ranges)&(ranges>=m.range_min)&(ranges<=m.range_max)
            points['lidar']=np.c_[ranges[good]*np.cos(angles[good]),ranges[good]*np.sin(angles[good]),np.zeros(np.sum(good))]
            source_frames['lidar']+=1
        elif topic=='/control/depth_points':
            points['depth']=read_points_numpy(m,field_names=['x','y','z'],skip_nans=True)
            source_frames['depth']+=1
        if topic!='/control/guard_input' or not begin<=t<=end:continue
        inspected+=1
        odom=last.get('/odom');command=last.get('/cmd_vel')
        if odom is None or t-seconds(odom.header.stamp)>own['odom_stale_timeout']:
            unresolved['odom']+=1;stopped=False
        elif command is None:
            unresolved['command']+=1;stopped=False
        else:
            stopped=abs(odom.twist.twist.linear.x)<own['stopped_speed'] and abs(odom.twist.twist.angular.z)<own['stopped_turn_rate']
        commanded_zero=command is not None and abs(command.linear.x)<=params['metric_zero_linear_speed'] and abs(command.angular.z)<=params['metric_zero_angular_speed']
        counts={};missing=[]
        if stopped:
            stopped_samples+=1
            commanded_zero_stopped_samples+=commanded_zero
            for source,topic in [('lidar','/scan'),('depth','/control/depth_points')]:
                msg=last.get(topic)
                if msg is None or not 0<=t-seconds(msg.header.stamp)<=c['source_timeout']:
                    unresolved[source]+=1;missing.append(source);continue
                try:
                    transform=buffer.lookup_transform_full(c['base_frame_id'],Time(nanoseconds=round(t*1e9)),msg.header.frame_id,Time.from_msg(msg.header.stamp),c['odom_frame_id']).transform
                except TransformException:
                    unresolved[source]+=1;missing.append(source);continue
                xyz=transform_points(points[source],transform)
                if source=='depth':xyz=xyz[(xyz[:,2]>=c['depth']['min_height'])&(xyz[:,2]<=c['depth']['max_height'])]
                counts[source]=int(np.sum(inside_polygon(xyz,polygon)))
        sources=tuple(sorted(s for s,n in counts.items() if n>=c['Approach']['min_points']))
        key=(sources,tuple(sorted(missing))) if sources else None
        if active and active['key']!=key:
            active['until_sim_s']=t;active['duration_s']=t-active['from_sim_s'];active.pop('key');intervals.append(active);active=None
        if key:
            if active is None:active={'key':key,'sources':list(sources),'unresolved_sources':missing,'from_sim_s':t,'last_observed_sim_s':t,'samples':0,'commanded_zero_samples':0,'maximum_inside_points':dict(counts),'max_requested_linear_mps':0.,'max_requested_angular_rps':0.,'max_final_linear_mps':0.,'max_final_angular_rps':0.}
            active['samples']+=1;active['last_observed_sim_s']=t
            active['commanded_zero_samples']+=commanded_zero
            active['max_final_linear_mps']=max(active['max_final_linear_mps'],abs(command.linear.x))
            active['max_final_angular_rps']=max(active['max_final_angular_rps'],abs(command.angular.z))
            for s,n in counts.items():active['maximum_inside_points'][s]=max(n,active['maximum_inside_points'].get(s,0))
            active['max_requested_linear_mps']=max(active['max_requested_linear_mps'],abs(m.linear.x))
            active['max_requested_angular_rps']=max(active['max_requested_angular_rps'],abs(m.angular.z))
    if active:
        active['until_sim_s']=active['last_observed_sim_s'];active['duration_s']=active['until_sim_s']-active['from_sim_s'];active.pop('key');intervals.append(active)
    report={'intervals':intervals,'guard_input_samples':inspected,'stopped_samples':stopped_samples,'commanded_zero_stopped_samples':commanded_zero_stopped_samples,'source_frames':source_frames,'unresolved_samples':unresolved,'padded_footprint_m':polygon,
            'measured_stop_thresholds':{'linear_mps':own['stopped_speed'],'angular_rps':own['stopped_turn_rate']},
            'note':'Offline receipt-ordered samples, sensor capture TF through odom to guard-input time; fresh measured odometry below Control stop thresholds required, even when final command is nonzero. Zero-command subset reported separately. Source attribution establishes coincidence, not causality; callback ordering may differ from the guard. No source frames or unresolved TF means absence cannot be established.'}
    (p/'guard_source_evidence.json').write_text(json.dumps(report,indent=2)+'\n')
    return report


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('directory');parser.add_argument('--control-config',required=True);args=parser.parse_args()
    print(json.dumps(check(args.directory,args.control_config),indent=2))
