#!/usr/bin/env python3
"""Read a completed evaluation comparison once; retain acceptance, LOOK and log evidence."""
import argparse,json,re
from pathlib import Path
import yaml
from encounter_check import check
from behaviour_audit import audit


def report(directory):
    p=Path(directory);result=json.loads((p/'result.json').read_text())
    planning=yaml.safe_load((p/'planning.yaml').read_text())['/**']['ros__parameters']
    acceptance=check(p,standard_nav2=True)
    behaviour=audit(p,planning)
    log=(p/'stack.log').read_text(errors='replace')
    m=result['last_metrics'];rates=re.findall(r'average rate: ([0-9.]+)',(p/'scan_hz.log').read_text())
    row={'run':p.parent.name,'duration_sim_s':result['duration_sim_s'],'rtf':result['real_time_factor'],'scan_hz':float(rates[-1]) if rates else None,
         'ego_routes':int(m['routes_ego']),'fleet_routes':int(m['routes_fleet']),'ego_collisions':int(m['collisions_ego']),
         'moving_command_contacts':acceptance['command_and_wait_audit']['contacts_while_commanded_motion'],
         'nav2_progress_failures':log.count('[controller_server]: Failed to make progress'),
         'nav2_optimizer_failures':log.count('[controller_server]: Optimizer fail to compute path'),
         'looks':int(m['looks']),'confirmed_looks':int(m['looks_confirmed']),
         'detection_frame_samples':int(m['detection_frame_samples']),
         'detection_frame_position_rmse_m':float(m['detection_frame_position_rmse_m']) if m['detection_frame_position_rmse_m'] else None,
         'overall_position_rmse_m':float(m['perception_position_rmse_m']) if m['perception_position_rmse_m'] else None,
         'longest_zero_command_s':acceptance['command_and_wait_audit']['longest_zero_command_s'],
         'passed':acceptance['passed'],'failures':acceptance['failures'],
         'coordinator_shutdown_error':"publisher's context is invalid" in log}
    (p/'followup_report.json').write_text(json.dumps({'summary':row,'acceptance':acceptance,'behaviour':behaviour},indent=2)+'\n')
    return row


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('directory');args=parser.parse_args()
    print(json.dumps(report(args.directory),indent=2))
