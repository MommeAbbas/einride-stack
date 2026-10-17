import math
import pytest
from control.guard_deadband import candidate,executable


@pytest.mark.parametrize('v,w,expected',[(0.,0.,(0.,0.)),(.1,.2,(.15,.3)),(-.1,-.2,(-.15,-.3)),(0.,.2,(0.,.35)),(0.,-.2,(0.,-.35)),(.2,.05,(.2,.05)),(math.nan,0.,(0.,0.))])
def test_candidate_preserves_direction_and_driving_curvature(v,w,expected):
    assert candidate(v,w,.15,.35)==pytest.approx(expected)


@pytest.mark.parametrize('v,w,valid',[(0.,0.,True),(.15,.01,True),(-.15,.01,True),(.149,.3,False),(0.,.349,False),(0.,.35,True),(0.,-.35,True),(math.nan,0.,False)])
def test_final_gate_cannot_raise_a_checked_command(v,w,valid):
    assert executable(v,w,.15,.35)==valid


def test_both_checks_retain_identical_stop_approach_and_sources():
    from control.guard_configuration import executable_guard_configuration
    from copy import deepcopy
    p={'cmd_vel_in_topic':'/control/guard_input','cmd_vel_out_topic':'/cmd_vel',
       'polygons':['Approach','Stop'],'observation_sources':['scan','depth'],
       'Approach':dict(action_type='approach',time_before_collision=.6,visualize=True),
       'Stop':dict(action_type='stop',points='[[1,1],[1,-1],[-1,-1],[-1,1]]',min_points=1,visualize=True),
       'depth':dict(min_height=.04,max_height=.50),'source_timeout':.5}
    data=executable_guard_configuration({'collision_monitor':{'ros__parameters':deepcopy(p)}})
    first=data['collision_monitor']['ros__parameters'];last=data['collision_validation']['ros__parameters']
    assert first['cmd_vel_out_topic']=='/control/guard_reduced' and last['cmd_vel_in_topic']=='/control/guard_proposal'
    assert last['cmd_vel_out_topic']=='/control/guard_checked'
    for key in ('observation_sources','depth','source_timeout'):assert first[key]==last[key]==p[key]
    for name in p['polygons']:
        assert {k:v for k,v in last[name].items() if k!='visualize'}=={k:v for k,v in p[name].items() if k!='visualize'}
