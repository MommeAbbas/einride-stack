#!/usr/bin/env python3
"""Formal E0–E4 acceptance. Oracle fixtures can never satisfy a stage check."""
import argparse
import csv
import json
import math
from pathlib import Path
import yaml
from area_check import messages,check_drive,check_perception,localisation_summary,behaviour_summary
from behaviour_audit import audit


def check(stage,directory,look_evidence=None):
    p=Path(directory);result=json.loads((p/'result.json').read_text())
    params=yaml.safe_load((p/'params.yaml').read_text())['/**']['ros__parameters']
    assert result['stack_stage']==stage
    assert result['map_source']!='fixture','Oracle fixture cannot establish integration acceptance'
    report={'stage':stage,'real_time_factor':result['real_time_factor'],'duration_sim_s':result['duration_sim_s'],'last_metrics':result['last_metrics']}
    report['localisation']=localisation_summary(p)
    report['behaviour']=behaviour_summary(p)
    planning=yaml.safe_load((p/'planning.yaml').read_text())['/**']['ros__parameters']
    report['integration_evidence']=audit(p,planning)
    if stage!='E0':
        from slam.snapshot import load_snapshot
        snapshot=load_snapshot(params['map_snapshot'])
        initial=next((m for _,m,_ in messages(p,['/initialpose'])),None)
        assert initial is not None,'No automatic AMCL final-pose handover recorded'
        pose=initial.pose.pose
        actual=[pose.position.x,pose.position.y,2*math.atan2(pose.orientation.z,pose.orientation.w)]
        assert all(math.isclose(a,b,abs_tol=1e-9) for a,b in zip(actual,snapshot['final_pose'])),'AMCL did not receive final mapping pose'
        assert list(initial.pose.covariance)==snapshot['final_pose_covariance']
        assert any(True for _ in messages(p,['/amcl_pose'])),'No AMCL estimate recorded'
        report['amcl_initial_pose']=actual
    if stage=='E0':
        assert result['map_source']=='sensors'
        last=None
        for _,m,_ in messages(p,['/slam/docks']):last=m
        assert last is not None and last.ready and all(d.valid for d in last.docks),'All eight usable tags required'
        from slam.snapshot import load_snapshot
        snapshot=load_snapshot(p/'slam_snapshot.json')
        assert (p/'sim_handover.json').is_file(),'Physical simulation handover missing'
        assert snapshot['backend']=='slam_toolbox'
        report['final_mapping_pose']=snapshot['final_pose']
        report['ready_tags']=[d.dock_id for d in last.docks]
    elif stage=='E1':report['driving']=check_drive(p)
    elif stage=='E2':
        assert int(result['last_metrics']['routes_ego'])>0,'Ego completed no Saga route'
        events=list(csv.DictReader((p/'events.csv').open()))
        dwells=[e for e in events if e['kind']=='route' and e['truck_or_robot']=='1' and e['action_or_severity'] in ('loading_complete','unloading_complete')]
        assert dwells,'No completed dwell'
        for e in dwells:
            assert float(e['dwell_sim_s'])>=params['dwell_seconds']
            assert float(e['dwell_wall_s'])>=params['wall_lock_seconds']
        before=0
        for _,m,t in messages(p,['/cmd_vel']):
            if t<params['event_start_delay']:
                assert m.linear.x==0. and m.angular.z==0.,'Motion before Saga Start'
                before+=1
        assert before>0,'Hold-before-start was not exercised'
        report['completed_ego_dwells']=len(dwells);report['prestart_zero_commands']=before
    elif stage=='E3':
        assert params['traffic_count']==0
        report['perception']=check_perception(p)
        assert int(result['last_metrics']['detections'])==0
        evidence=report['integration_evidence']
        if look_evidence is not None:
            extra=Path(look_evidence)
            extra_result=json.loads((extra/'result.json').read_text())
            extra_params=yaml.safe_load((extra/'params.yaml').read_text())['/**']['ros__parameters']
            extra_planning=yaml.safe_load((extra/'planning.yaml').read_text())['/**']['ros__parameters']
            assert extra_result['stack_stage']=='E3' and extra_params['traffic_count']==0
            assert extra_params['map_source']=='snapshot' and extra_params['map_snapshot']==params['map_snapshot']
            assert extra_params['route_seed']==params['route_seed']
            for feature in ('planning_risk','planning_look','planning_observation_curves','control_prediction_critic'):
                assert extra_params[feature] and extra_params[feature]==params[feature]
            assert 0<=extra_planning['risk_tolerance']<=planning['risk_tolerance'],'LOOK coverage must not loosen risk'
            report['look_coverage_perception']=check_perception(extra)
            evidence=audit(extra,extra_planning)
            report['look_coverage']={'directory':str(extra),'risk_tolerance':extra_planning['risk_tolerance'],'evidence':evidence}
        if planning['risk_check']:
            assert evidence['risk_decreased_after_look']>0,'No observed risk reduction after confirmed LOOK'
            assert evidence['drive_resumptions_after_confirmed_look']>0,'Never resumed DRIVE after a confirmed LOOK'
        else:
            report['risk_look_coverage']='DISABLED by v1 configuration; no LOOK acceptance claimed'
        assert int(result['last_metrics']['routes_ego'])>0,'Empty-scene mission made no route progress'
    elif stage=='E4':
        assert result['duration_sim_s']>=params['duration']
        assert params['duration']>=300.,'Five simulation minutes required'
        m=result['last_metrics']
        assert int(m['truth_count'])==params['traffic_count']+1
        assert int(m['calibration_samples'])>0
        assert all(int(m[f'prediction_samples_{n}s'])>0 for n in (1,2,3))
        assert m['minimum_separation_m']!='' and m['planning_time_ms']!=''
        assert int(m['invalid_predictions'])==0
        assert int(m['routes_ego'])>0,'No ego route completed; recording duration does not establish navigation'
        evidence=report['integration_evidence']
        assert evidence['longest_continuous_risk_wait_s']<=planning['deadlock_timeout'],'Risk WAIT exceeded configured deadlock timeout'
        assert evidence['longest_continuous_guard_wait_s']<=planning['deadlock_timeout'],'Guard standstill exceeded configured deadlock timeout; safe recovery not demonstrated'
        assert not evidence['nonzero_commands_after_guard_stop'],'Nonzero command while raw guard reported STOP'
    else:raise ValueError(stage)
    report['passed']=True
    (p/'stage_check.json').write_text(json.dumps(report,indent=2)+'\n')
    return report


if __name__=='__main__':
    a=argparse.ArgumentParser();a.add_argument('stage',choices=['E0','E1','E2','E3','E4']);a.add_argument('directory');a.add_argument('--look-evidence',help='Separate empty E3 run with stricter threshold for LOOK branch coverage');args=a.parse_args();print(json.dumps(check(args.stage,args.directory,args.look_evidence),indent=2))
