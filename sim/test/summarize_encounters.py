#!/usr/bin/env python3
"""Collect the fixed verification matrix; bags remain ignored."""
import argparse,csv,json,re
from pathlib import Path
parser=argparse.ArgumentParser(description='Summarise completed encounter checks without changing acceptance.')
parser.add_argument('run_root',type=Path);parser.add_argument('output',type=Path)
args=parser.parse_args();root=args.run_root;out=args.output;out.mkdir(parents=True,exist_ok=True)
rows=[];checks={};visibility={}
for name in ['E4_1_ignore','E4_3_ignore','E4_3_yield','E3','E3_LOOK','E4_3_yield_system','E4_3_yield_fixed','E3_discovery','E3_LOOK_discovery']:
 p=root/name/'run_000'
 if not (p/'result.json').exists():
  if p.exists():
   rows.append({'run':name,'acceptance':'INCOMPLETE_NO_RESULT'})
   log=p.parent/'check.log'
   checks[name]={'passed':False,'failure':'No completed result.json','check_log':log.read_text() if log.exists() else 'No completed recording'}
  continue
 d=json.loads((p/'result.json').read_text());r={'run':name,'duration_sim_s':d['duration_sim_s'],'duration_wall_s':d['duration_wall_s'],'real_time_factor':d['real_time_factor']}
 rates=re.findall(r'average rate: ([0-9.]+)',(p/'scan_hz.log').read_text());r['scan_hz_sample']=rates[-1] if rates else ''
 r.update(d['last_metrics'])
 check=p/('encounter_check.json' if name.startswith('E4') else 'stage_check.json')
 if check.exists():
  c=json.loads(check.read_text());checks[name]=c;r['acceptance']='PASS' if c['passed'] else 'FAIL'
  if name.startswith('E4'):
   r.update({k:v for k,v in c['command_and_wait_audit'].items() if k!='ego_contacts'})
 elif (p.parent/'check.log').exists():
  r['acceptance']='FAIL_OR_INCOMPLETE_CHECK';checks[name]={'passed':False,'check_log':(p.parent/'check.log').read_text()}
 elif name.startswith('E3_LOOK') and (root/name.replace('E3_LOOK','E3')/'run_000/stage_check.json').exists():r['acceptance']='PASS_coverage'
 else:r['acceptance']='PENDING'
 rows.append(r)
 if (p/'detector_visibility.csv').exists():
  v=list(csv.DictReader((p/'detector_visibility.csv').open()))
  counts={}
  for frame in v:counts[frame['reason']]=counts.get(frame['reason'],0)+1
  visibility[name]={'frame_count':len({f['stamp'] for f in v}),'robot_frame_rows':len(v),'eligible':sum(f['eligible']=='True' for f in v),'boxes':sum(f['box_emitted']=='True' for f in v),'reasons':counts}
if rows:
 fields=list(dict.fromkeys(k for r in rows for k in r))
 with (out/'summary.csv').open('w',newline='') as f:w=csv.DictWriter(f,fieldnames=fields,lineterminator='\n');w.writeheader();w.writerows(rows)
(out/'checks.json').write_text(json.dumps(checks,indent=2)+'\n')
(out/'visibility.json').write_text(json.dumps(visibility,indent=2)+'\n')
print('Report rows:',[r['run'] for r in rows])
