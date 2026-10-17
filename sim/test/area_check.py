#!/usr/bin/env python3
"""Phase 2 recorded-data acceptance: explicit fixture runs are allowed here only."""
import argparse
from collections import Counter
import json
import math
import yaml
import numpy as np
from pathlib import Path
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def messages(directory,topics):
    reader=rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(Path(directory)/'bag'),storage_id='mcap'),rosbag2_py.ConverterOptions('',''))
    types={t.name:get_message(t.type) for t in reader.get_all_topics_and_types() if t.name in topics}
    if not types:return # Empty rosbag filter means all topics, not zero matches.
    reader.set_filter(rosbag2_py.StorageFilter(topics=list(types)))
    while reader.has_next():
        topic,data,stamp=reader.read_next()
        yield topic,deserialize_message(data,types[topic]),stamp/1e9


def seconds(t):return t.sec+t.nanosec*1e-9


def localisation_summary(directory):
    """Evaluation only: absolute map estimate versus interpolated simulator truth."""
    truth=[];poses=[];docks=None
    def xy_yaw(p):return [p.position.x,p.position.y,2*math.atan2(p.orientation.z,p.orientation.w)]
    for topic,m,_ in messages(directory,['/sim/truth','/slam/pose','/slam/docks']):
        if topic=='/sim/truth':truth.append([seconds(m.header.stamp),*xy_yaw(m.robots[0].pose)])
        elif topic=='/slam/pose':poses.append([seconds(m.header.stamp),*xy_yaw(m.pose.pose)])
        else:docks=m
    if not truth or not poses:return {'samples':0}
    a=np.asarray(truth);b=np.asarray(poses);b=b[(b[:,0]>=a[0,0])&(b[:,0]<=a[-1,0])]
    reference=np.c_[np.interp(b[:,0],a[:,0],a[:,1]),np.interp(b[:,0],a[:,0],a[:,2]),np.interp(b[:,0],a[:,0],np.unwrap(a[:,3]))]
    error=b[:,1:]-reference;error[:,2]=np.arctan2(np.sin(error[:,2]),np.cos(error[:,2]))
    report={'samples':len(b),'position_rmse_m':float(np.sqrt(np.mean(np.sum(error[:,:2]**2,axis=1)))),'heading_rmse_rad':float(np.sqrt(np.mean(error[:,2]**2))),'max_heading_error_rad':float(np.max(np.abs(error[:,2])))}
    geometry=json.loads((Path(directory)/'geometry.json').read_text())
    if docks is not None:
        report['mapped_dock_goal_error_m']={chr(65+d.dock_id):math.dist([d.docking_pose.pose.position.x,d.docking_pose.pose.position.y],geometry['docks'][chr(65+d.dock_id)][:2]) for d in docks.docks if d.valid}
    return report


def behaviour_summary(directory):
    modes=Counter();reasons=Counter();look_ids=set();moves=0;stops=0;last=None;start=None;maximum=0.
    for topic,m,t in messages(directory,['/control/status','/planning/behaviour','/cmd_vel','/control/collision_state']):
        if topic=='/control/status':
            modes[m.mode]+=1
            if m.mode==2:look_ids.add(m.request_id)
        elif topic=='/planning/behaviour':
            reason=m.data.split(';')[0];reasons[reason]+=1
            if reason!=last:
                if start is not None:maximum=max(maximum,t-start)
                last=reason;start=t
            else:maximum=max(maximum,t-start)
        elif topic=='/cmd_vel':moves+=abs(m.linear.x)>0.
        else:stops+=m.action_type==m.STOP
    return {'mode_samples':dict(modes),'behaviour_samples':dict(reasons),'look_request_ids':len(look_ids),'nonzero_linear_samples':moves,'guard_stop_state_samples':stops,'longest_same_behaviour_s':maximum}


def check_slam(directory):
    counts=Counter();last=None;docks=None;looks=0
    for topic,m,_ in messages(directory,['/map','/slam/pose','/slam/status','/slam/docks','/control/status']):
        counts[topic]+=1
        if topic=='/slam/status':last=m.data
        if topic=='/slam/docks':docks=m
        if topic=='/control/status' and m.mode==2 and m.status==m.SUCCEEDED:
            assert m.stopped and m.final_heading_valid
            looks+=1
    assert all(counts[t]>0 for t in ['/map','/slam/pose','/slam/status','/slam/docks'])
    assert docks is not None and any(d.valid for d in docks.docks)
    assert looks>0,'No completed measured-stop LOOK'
    return {'topic_counts':dict(counts),'last_status':last,'valid_tags':sum(d.valid for d in docks.docks),'ready':docks.ready,'note':'Area smoke check only; E0 additionally requires all eight and readiness.'}


def check_perception(directory):
    counts=Counter();receipts=0;last=None;groups=0
    for topic,m,_ in messages(directory,['/perception/predictions','/perception/health','/perception/diagnostics','/perception/image/compressed','/visualization/predictions']):
        counts[topic]+=1
        if topic=='/perception/predictions':
            assert len(m.robots)==3
            for r in m.robots:
                assert math.isclose(sum(g.probability for g in r.groups)+r.leftover.probability,1.,abs_tol=1e-6)
                assert len(r.leftover.points)>=2
                groups+=len(r.groups)
        elif topic=='/perception/health' and m.has_camera_observation:
            r=m.last_camera_observation
            if r.negative_evidence_applied:
                assert r.processed and r.negative_evidence_usable
                assert seconds(r.predictions_after_frame)>seconds(r.header.stamp)
                receipts+=1
        elif topic=='/perception/diagnostics':last=m.data
    assert all(counts[t]>0 for t in ['/perception/predictions','/perception/health','/perception/image/compressed','/visualization/predictions'])
    assert receipts>0 and last is not None
    assert 'camera_measurements=0;' in last and 'lidar_groups=0;' in last,last
    return {'topic_counts':dict(counts),'applied_receipts':receipts,'named_groups':groups,'last_diagnostics':last}


def check_drive(directory):
    params=yaml.safe_load((Path(__file__).parents[1]/'config/sim.yaml').read_text())['/**']['ros__parameters']
    params.update(yaml.safe_load((Path(directory)/'params.yaml').read_text())['/**']['ros__parameters'])
    last={};holds=0;moving=0;guard=0;guard_stops=0;arrived_at=None;end=0
    for topic,m,t in messages(directory,['/sim/truth','/cmd_vel','/planning/behaviour','/control/collision_state','/perception/dock_observations']):
        last[topic]=m;end=t
        if topic=='/control/collision_state':guard=m.action_type
        if topic=='/cmd_vel':
            moving+=abs(m.linear.x)>params['stopped_speed']
            if guard==1:
                assert m.linear.x==0. and m.angular.z==0.,'Guard STOP leaked motion'
                guard_stops+=1
        if topic=='/planning/behaviour':
            if 'wait for organiser' in m.data:
                holds+=1
                if arrived_at is None:arrived_at=t
            elif arrived_at is not None and 'Collision guard' not in m.data:
                raise AssertionError('Arrival hold resumed navigation for the same manual goal')
    pos=last['/sim/truth'].robots[0].pose.position
    target=json.loads((Path(directory)/'geometry.json').read_text())['docks']['A']
    distance=math.hypot(pos.x-target[0],pos.y-target[1])
    assert distance<params['arrival_tolerance'],(distance,'outside organiser tolerance')
    assert moving>0 and holds>0 and end-arrived_at>=params['acceptance_hold_seconds'],'No sustained dock hold'
    assert '/perception/dock_observations' in last
    return {'final_true_distance_m':distance,'nonzero_commands':moving,'hold_messages':holds,'hold_seconds':end-arrived_at,'guard_zero_samples':guard_stops}


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--slam',required=True);p.add_argument('--perception',required=True);p.add_argument('--drive',required=True);p.add_argument('--output',required=True);a=p.parse_args()
    result={'slam':check_slam(a.slam),'perception':check_perception(a.perception),'planning_control':check_drive(a.drive),'passed':True}
    Path(a.output).write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
