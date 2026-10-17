#!/usr/bin/env python3
"""E4 acceptance: progress, commanded-motion contacts and unbroken holds.

A new ladder label cannot reset a physical zero-command interval. This audit
uses final guarded /cmd_vel, including rotation, and the capture-stamped contact.
"""
import argparse,bisect,json
from pathlib import Path
import yaml
from area_check import messages,seconds


def motion_audit(commands,contacts,start,end,linear_epsilon,angular_epsilon):
    commands=sorted(commands);times=[c[0] for c in commands]
    moving=lambda c:abs(c[1])>linear_epsilon or abs(c[2])>angular_epsilon
    initial=bisect.bisect_right(times,start)-1
    current=commands[initial] if initial>=0 else (start,0.,0.)
    since=None if moving(current) else start;longest=0.
    for c in commands:
        if c[0]<=start:continue
        if c[0]>end:break
        if moving(c) and since is not None:longest=max(longest,c[0]-since);since=None
        elif not moving(c) and since is None:since=c[0]
    if since is not None:longest=max(longest,end-since)
    impacts=[]
    for t,ident in contacts:
        i=bisect.bisect_right(times,t)-1
        impacts.append({'event_id':ident,'stamp':t,'command':list(commands[i][1:]) if i>=0 else None,'moving':moving(commands[i]) if i>=0 else None})
    return {'longest_zero_command_s':longest,'ego_contacts':impacts,'contacts_while_commanded_motion':sum(c['moving'] is True for c in impacts),'contacts_without_command_evidence':sum(c['moving'] is None for c in impacts)}


def check(directory,standard_nav2=False):
    p=Path(directory);result=json.loads((p/'result.json').read_text());params=yaml.safe_load((p/'params.yaml').read_text())['/**']['ros__parameters']
    commands=[];contacts=[];start=None;phases={}
    for topic,m,t in messages(p,['/cmd_vel','/sim/collisions','/planning/saga_state','/control/status']):
        if topic=='/cmd_vel':commands.append((t,m.linear.x,m.angular.z))
        elif topic=='/sim/collisions' and not m.forced_test and 0 in (m.first_robot,m.second_robot):contacts.append((seconds(m.header.stamp),m.event_id))
        elif topic=='/planning/saga_state' and m.available and m.event_state=='running' and start is None:start=t
        elif topic=='/control/status' and m.reason.startswith('ENCOUNTER '):
            key=m.reason.split(':')[0];phases[key]=phases.get(key,0)+1
    start=max(start if start is not None else result['start_sim_s'],result['start_sim_s'])
    audit=motion_audit(commands,contacts,start,result['start_sim_s']+result['duration_sim_s'],params['metric_zero_linear_speed'],params['metric_zero_angular_speed'])
    m=result['last_metrics'];reasons=[]
    if result['duration_sim_s']<300:reasons.append('Less than 300 sim seconds')
    if int(m['routes_ego'])<1:reasons.append('No ego route')
    if audit['contacts_while_commanded_motion']:reasons.append('Collision while commanded motion nonzero')
    if audit['contacts_without_command_evidence']:reasons.append('Collision command evidence missing')
    if not standard_nav2 and audit['longest_zero_command_s']>params['acceptance_wait_limit']:reasons.append('Continuous zero-command hold exceeds 15 s')
    coverage={}
    for category in ('all','detection_frame','coasting'):
        prefix='' if category=='all' else category+'_'
        a=m.get(prefix+'inside_1sigma','');b=m.get(prefix+'inside_2sigma','')
        count=m['calibration_samples'] if category=='all' else m.get(prefix+'samples',0)
        coverage[category]={'samples':int(count),'inside_1sigma':float(a) if a!='' else None,'inside_2sigma':float(b) if b!='' else None,'target_met':bool(a!='' and b!='' and .25<=float(a)<=.55 and float(b)>=.75)}
    report={'passed':not reasons,'failures':reasons,'command_and_wait_audit':audit,'encounter_phase_samples':phases,'calibration':coverage,'note':'Detection/coasting is frame-level using latest processed camera receipt; not per-track detector recall. Calibration targets are reported separately from navigation acceptance.'}
    report['acceptance_policy']='Standard-Nav2 v1: full duration, ego route, no commanded-motion contact' if standard_nav2 else 'Original encounter ladder including 15 s hold limit'
    (p/'encounter_check.json').write_text(json.dumps(report,indent=2)+'\n');return report


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('directory');parser.add_argument('--standard-nav2',action='store_true',help='Use the standard-Nav2 acceptance; still report continuous hold duration');args=parser.parse_args()
    report=check(args.directory,args.standard_nav2);print(json.dumps(report,indent=2));raise SystemExit(0 if report['passed'] else 1)
