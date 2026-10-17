from pathlib import Path
import yaml
from sim.traffic import yield_command,mode_command
from sim.collision import CollisionMonitor


def test_yield_front_all_robots_but_not_behind_or_ignore():
    assert yield_command((.3,.1),(0,0,0),[(.3,0,0)],.35,1.5707963267948966)==(0.,0.)
    assert yield_command((.3,.1),(0,0,0),[(-.3,0,0)],.35,1.5707963267948966)==(.3,.1)
    assert yield_command((.3,.1),(0,0,0),[(.5,0,0)],.35,1.5707963267948966)==(.3,.1)
    assert mode_command('ignore',(.3,.1),(0,0,0),(.01,0,0),.35)==(.3,.1)


def test_evaluation_rectangle_covers_front_wheels_and_detects_old_gap():
    p=yaml.safe_load((Path(__file__).parents[1]/'config/sim.yaml').read_text())['/**']['ros__parameters']
    assert p['collision_footprint_length']/2>=.12596
    assert p['collision_footprint_width']/2>=.11521
    monitor=CollisionMonitor(p['collision_footprint_length'],p['collision_footprint_width'],p['major_collision_speed'],p['collision_release_distance'])
    events,gap=monitor.update({0:((0,0,0),(0,0),True),1:((.25,0,0),(0,0),True)})
    assert len(events)==1 and gap==0


def test_detector_audit_separates_visible_miss_from_outside_view():
    import numpy as np
    from types import SimpleNamespace as N
    from sim.detector import emulate
    p=yaml.safe_load((Path(__file__).parents[1]/'config/sim.yaml').read_text())['/**']['ros__parameters']
    p['detector_miss_rate']=1.;pose=np.eye(4);pose[2,3]=2.
    audit=[]
    boxes=emulate({1:pose},np.eye(4),np.array([[500,0,320],[0,500,240],[0,0,1]]),[640,480],[.245,.23,.19],p,N(clear_line=lambda *a:True),np.random.default_rng(1),audit)
    assert not boxes and audit[0]['eligible'] and audit[0]['reason']=='configured_miss'
    pose[2,3]=-2.;audit=[]
    emulate({1:pose},np.eye(4),np.eye(3),[640,480],[.245,.23,.19],p,N(clear_line=lambda *a:True),np.random.default_rng(1),audit)
    assert not audit[0]['in_fov'] and not audit[0]['eligible']
