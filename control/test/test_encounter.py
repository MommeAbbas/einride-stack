from concurrent.futures import Future
from types import SimpleNamespace as N
import yaml
from pathlib import Path
from control import encounter as module


class Client:
    def __init__(self,*args):self.calls=[]
    def server_is_ready(self):return True
    def service_is_ready(self):return True
    def send_goal_async(self,goal):return self.call_async(goal)
    def call_async(self,request):
        f=Future();self.calls.append((request,f));return f


class Handle:
    accepted=True
    def __init__(self):self.result=Future();self.cancelled=False
    def get_result_async(self):return self.result
    def cancel_goal_async(self):self.cancelled=True


def make(monkeypatch):
    monkeypatch.setattr(module,'ActionClient',Client)
    # Historical module regression fixture; these parameters are no longer runtime config.
    params=dict(encounter_wait=3.,encounter_jitter=2.,encounter_seed=7,
                encounter_backup_distance=.25,encounter_backup_speed=.1,
                encounter_backup_timeout=8.,encounter_service_timeout=2.,
                encounter_replan_service='/planning/replan_encounter')
    node=N(p=params.__getitem__,now=lambda:10.,create_client=lambda *a:Client(),create_subscription=lambda *a:None)
    return module.Encounter(node)


def finish(client):
    h=Handle();client.calls[-1][1].set_result(h);h.result.set_result(N(status=4));return h


def test_ladder_repeats_with_jitter_and_backup_then_ready(monkeypatch):
    e=make(monkeypatch);e.start();e.tick()
    first=e.wait.calls[-1][0].time;wait=first.sec+first.nanosec/1e9
    assert 3<=wait<=5 and e.state=='WAIT'
    finish(e.wait);e.tick();e.replan.calls[-1][1].set_result(N(success=False,message='blocked'))
    e.tick();goal=e.backup.calls[-1][0]
    assert goal.target.x==-.25 and goal.speed<=.1+1e-8
    finish(e.backup);e.tick();e.replan.calls[-1][1].set_result(N(success=False,message='blocked'))
    e.tick();second=e.wait.calls[-1][0].time
    assert (second.sec,second.nanosec)!=(first.sec,first.nanosec)
    finish(e.wait);e.tick();e.replan.calls[-1][1].set_result(N(success=True,message='path'))
    assert e.state=='READY'


def test_cancel_rejects_late_action_acceptance(monkeypatch):
    e=make(monkeypatch);e.start();e.tick();future=e.wait.calls[-1][1]
    e.cancel();handle=Handle();future.set_result(handle)
    assert handle.cancelled and e.state=='IDLE'


def test_directional_zones_keep_dimensions_and_allow_escape():
    import ast
    cfg=yaml.safe_load((Path(__file__).parents[1]/'config/control.yaml').read_text())['collision_monitor']['ros__parameters']
    for name,length,width in [('Slow',.62,.48)]:
        zone=cfg[name];assert zone['min_points']==1
        for direction in zone['velocity_polygons']:
            points=ast.literal_eval(zone[direction]['points']);xs=[p[0] for p in points];ys=[p[1] for p in points]
            assert abs(max(xs)-min(xs)-length)<1e-10
            assert abs(max(ys)-min(ys)-width)<1e-10
        assert max(p[0] for p in ast.literal_eval(zone['reverse']['points']))<.2
        assert min(p[0] for p in ast.literal_eval(zone['forward']['points']))>-.2
    assert cfg['Slow']['slowdown_ratio']==.3


def test_approach_uses_existing_footprint_and_threshold():
    cfg=yaml.safe_load((Path(__file__).parents[1]/'config/control.yaml').read_text())['collision_monitor']['ros__parameters']
    assert cfg['polygons']==['Approach','Slow']
    assert cfg['Approach']['action_type']=='approach'
    assert cfg['Approach']['footprint_topic']=='/local_costmap/published_footprint'
    assert cfg['Approach']['time_before_collision']==1.2 # Baseline simulation horizon.
    assert cfg['Approach']['min_points']==1
    assert cfg['depth']['min_height']==.04 and cfg['depth']['max_height']==.5
