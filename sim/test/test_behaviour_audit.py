"""Acceptance audit must not manufacture LOOK benefit from an older risk sample."""
from types import SimpleNamespace as N
import behaviour_audit
from interfaces.msg import CoordinatorRequest,CoordinatorStatus


def test_receipt_and_request_tick_risk(monkeypatch):
    stamp=lambda t:N(sec=int(t),nanosec=round(t%1*1e9))
    req=CoordinatorRequest(request_id='look',goal_id='route',mode=CoordinatorRequest.LOOK,map_revision=1)
    status=CoordinatorStatus(request_id='look',mode=CoordinatorRequest.LOOK,status=CoordinatorStatus.SUCCEEDED,completion_stamp=stamp(3.))
    receipt=N(processed=True,negative_evidence_applied=False,header=N(stamp=stamp(3.5)),predictions_after_frame=stamp(4.),map_revision=1)
    applied=N(**{**receipt.__dict__,'negative_evidence_applied':True})
    samples=[('/planning/risk',N(data=.1),1.),('/control/request',req,2.),
        ('/planning/risk',N(data=.8),2.),('/control/status',status,3.),
        ('/perception/health',N(has_camera_observation=True,last_camera_observation=receipt),4.),
        ('/planning/risk',N(data=.7),4.),
        ('/perception/health',N(has_camera_observation=True,last_camera_observation=applied),4.5),
        ('/planning/risk',N(data=.2),4.6)]
    monkeypatch.setattr(behaviour_audit,'messages',lambda *args:iter(samples))
    report=behaviour_audit.audit('unused',{'deadlock_timeout':8.})
    assert report['confirmed_looks']==1 and report['risk_decreased_after_look']==1
    assert report['risk_before_after_look'][0]['before']==.8
    assert report['risk_before_after_look'][0]['after']==.2


def test_guard_standstill_duration_includes_silent_stop_until_bag_end(monkeypatch):
    zero=N(linear=N(x=0.),angular=N(z=0.))
    samples=[('/control/collision_state',N(action_type=1,STOP=1),2.),
             ('/cmd_vel',zero,3.),
             ('/control/collision_state',N(action_type=0,STOP=1),5.),
             ('/control/collision_state',N(action_type=1,STOP=1),7.),
             ('/planning/behaviour',N(data='WAIT: Collision guard stop'),20.)]
    monkeypatch.setattr(behaviour_audit,'messages',lambda *args:iter(samples))
    report=behaviour_audit.audit('unused',{'deadlock_timeout':8.})
    assert report['longest_guard_stop_s']==13.
    assert report['nonzero_commands_after_guard_stop']==[]


def test_guard_behaviour_wait_is_distinct_from_sparse_raw_state(monkeypatch):
    samples=[('/planning/behaviour',N(data='WAIT: Collision guard stop; goal=a'),2.),
             ('/control/collision_state',N(action_type=1,STOP=1),3.),
             ('/control/collision_state',N(action_type=0,STOP=1),4.),
             ('/planning/behaviour',N(data='WAIT: Collision guard stop; goal=a'),10.),
             ('/planning/behaviour',N(data='DRIVE: Navigate'),12.),
             ('/planning/behaviour',N(data='WAIT: Collision guard stop'),14.),
             ('/planning/risk',N(data=.1),30.)]
    monkeypatch.setattr(behaviour_audit,'messages',lambda *args:iter(samples))
    report=behaviour_audit.audit('unused',{'deadlock_timeout':8.})
    assert report['longest_guard_stop_s']==1.
    assert report['longest_continuous_guard_wait_s']==16.
