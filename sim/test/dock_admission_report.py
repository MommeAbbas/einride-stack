#!/usr/bin/env python3
"""Recorded dock admission branch coverage, distinct from collision acceptance."""
import argparse,json,re
from pathlib import Path
from area_check import messages


def report(directory,output):
    p=Path(directory);entries=[];previous=None;start=None;end=None;intervals=[]
    occupied_samples=0;stand_drive=0;stand_wait=0;release=0;was_active=False;cue_counts={}
    for topic,m,t in messages(p,['/planning/behaviour']):
        occupied='dock_occupied=true' in m.data;active='standoff_active=true' in m.data
        occupied_samples+=occupied
        cue=re.search(r'occupancy_cues=([^;]*)',m.data)
        if occupied and cue:
            for name in cue.group(1).split(','):cue_counts[name]=cue_counts.get(name,0)+1
        stand_drive+=active and m.data.startswith('DRIVE: Occupied dock')
        stand_wait+=active and m.data.startswith('WAIT: Occupied dock')
        if was_active and not active and m.data.startswith('DRIVE: Nav2'):release+=1
        was_active=active
        reason=m.data.split('; Nav2=')[0]
        if reason!=previous:
            if previous is not None:intervals.append(dict(start=start,end=t,reason=previous))
            previous=reason;start=t
        end=t
    if previous is not None:intervals.append(dict(start=start,end=end,reason=previous))
    result=dict(occupied_samples=occupied_samples,occupied_cue_samples=cue_counts,standoff_drive_samples=stand_drive,
        standoff_wait_samples=stand_wait,drive_resumptions=release,
        branch_exercised=bool(occupied_samples and stand_drive and stand_wait and release),
        intervals=[r for r in intervals if 'Occupied dock' in r['reason']],
        note='Branch coverage from recorded real-sensor decisions, not proof of perfect dock occupancy recall. See navigation truth plots and collision audit for outcome.')
    path=Path(output);path.parent.mkdir(parents=True,exist_ok=True);path.write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps({k:v for k,v in result.items() if k!='intervals'},indent=2))
    return result

if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('directory');parser.add_argument('output');args=parser.parse_args();report(args.directory,args.output)
