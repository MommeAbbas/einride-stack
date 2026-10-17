"""Compose installed Nav2 checks; no collision geometry or thresholds changed."""
from copy import deepcopy


def executable_guard_configuration(data):
    primary=data['collision_monitor']['ros__parameters']
    primary['cmd_vel_out_topic']='/control/guard_reduced'
    data['collision_validation']=deepcopy(data['collision_monitor'])
    checked=data['collision_validation']['ros__parameters']
    checked.update(cmd_vel_in_topic='/control/guard_proposal',cmd_vel_out_topic='/control/guard_checked',state_topic='/control/validation_state')
    for name in checked['polygons']:checked[name]['visualize']=False
    return data
