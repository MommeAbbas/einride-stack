import numpy as np
from planning.grid import Grid
from planning.risk import trajectory_risk,confirmed_look
from planning.trajectory import timed_path
from planning.saga import normalise
from planning.behaviour import Situation,decide


def group(start,end,mass=1.):
    return mass,[[0,*start,.001,0,0,.001],[3,*end,.001,0,0,.001]]


def test_risk_absolute_time_leftover_and_broad_mass():
    times=np.linspace(0,3,13); own=np.c_[times,np.zeros(13)]
    crossing=[[group([1.5,-1],[1.5,1])],[group([9,9],[9,9])],[group([-9,-9],[-9,-9])]]
    assert trajectory_risk(own,times,crossing,.168,.2,7,1e-6,0)>.9
    shifted=times+1
    assert trajectory_risk(own,shifted,crossing,.168,.2,7,1e-6,0)==1
    crossing[0]=[group([9,9],[9,9],.5),group([1.5,-1],[1.5,1],.5)]
    assert .4<trajectory_risk(own,times,crossing,.168,.2,7,1e-6,0)<.6


def test_look_requires_applied_post_completion_frame_and_later_predictions():
    assert confirmed_look(1,2,3,True,True,True)
    assert not confirmed_look(2,2,3,True,True,True)
    assert not confirmed_look(1,2,2,True,True,True)
    assert not confirmed_look(1,2,3,True,False,True)
    assert not confirmed_look(1,2,3,True,True,False)


def test_risk_rejects_invalid_mass_time_and_asymmetric_covariance():
    times=np.linspace(0,3,13);own=np.zeros((13,2))
    for mass in (-.1,float('nan'),float('inf'),1.1):
        assert trajectory_risk(own,times,[[group([9,9],[9,9],mass)]],.168,.2,7,1e-6,0)==1.
    for column,value in ((0,float('nan')),(4,.1)):
        invalid=group([9,9],[9,9]);invalid[1][0][column]=value
        assert trajectory_risk(own,times,[[invalid]],.168,.2,7,1e-6,0)==1.


def test_probability_roundoff_does_not_become_certain_collision():
    times=np.linspace(0,3,13);own=np.zeros((13,2))
    clear=[[group([9,9],[9,9],np.nextafter(1.,2.))]]
    assert trajectory_risk(own,times,clear,.168,.2,7,1e-6,0)==0.
    clear[0][0]=(1.01,clear[0][0][1])
    assert trajectory_risk(own,times,clear,.168,.2,7,1e-6,0)==1.


def test_clearance_no_corner_cut_and_unknown():
    a=np.zeros((20,20),bool); a[5:15,8:12]=True
    g=Grid(a,.1,[0,0]); path=g.plan([.2,1],[1.8,1],.15)
    assert len(path)>2
    from scipy.ndimage import distance_transform_edt
    blocked=distance_transform_edt(~a)*.1<=.15
    assert all(g.clear_line(a,b,blocked) for a,b in zip(path,path[1:]))


def test_timing_includes_initial_turn_and_stationary_wait():
    points=timed_path([[0,0],[1,0]],[0,0,np.pi],.3,.6,.25,3.)
    assert points[0,3]==0 and abs(points[0,4])==.6
    assert np.allclose(timed_path([],[1,2,0],.3,.6,.25,3.)[:,:2],[1,2])


def test_rolling_timing_starts_from_actual_path_progress():
    from planning.trajectory import densify
    path=densify([[0.,0.],[2.,0.]],.04)
    points=timed_path(path,[1.,0.,0.],.3,.6,.25,3.)
    assert points[0,3]==.3 and points[0,4]==0.
    assert np.all(np.diff(points[:,0])>=0.) and points[-1,0]>1.5
    # Project onto segments, not nearest vertices: a sparse path must not stop
    # halfway simply because the final vertex is now closest.
    sparse=timed_path([[0.,0.],[2.,0.]],[1.1,0.,0.],.3,.6,.25,1.)
    assert sparse[-1,0]>sparse[0,0]


def test_tree_branches_and_reservations():
    s=Situation(map_ready=True,event_running=True,next_dock=1,valid=True,risk=.2,dock_free=True)
    assert decide(s,.4,True)[0]=='DRIVE'
    s.guard=True; assert decide(s,.4,True)[0]=='WAIT'
    s.guard=False;s.risk=.8;s.look_useful=True;assert decide(s,.4,True)[0]=='LOOK'
    s.look_useful=False;assert decide(s,.4,True)[0]=='WAIT'
    data={'truck_id':1,'event_state':'running','route':{'route_id':2,'source':{'tag_id':0},'destination':{'tag_id':1},'status':'loading_complete','next_dock':{'tag_id':1}}}
    assert normalise(data)['held_docks']==[1]


def test_dense_nav2_preview_retains_segment_geometry():
    from planning.trajectory import densify
    path=densify([[0,0],[3,0]],.04)
    assert len(path)==76 and np.allclose(path[-1],[3,0])
    assert max(np.linalg.norm(np.asarray(b)-a) for a,b in zip(path,path[1:]))<.040001


def test_observation_curve_checks_view_clearance_and_turn_rate():
    from planning.observation import observation_candidate
    from scipy.ndimage import distance_transform_edt
    grid=Grid(np.zeros((120,120),bool),.1,[-6,-6])
    path=[[-2.,0.],[2.,0.]]
    # Looking backwards initially, a forward low-curvature pass opens the target view.
    curve=observation_candidate(grid,path,[3,0],.2,.3,1.05,.04,.3,.25,[-2,0,np.pi])
    assert curve is not None
    assert observation_candidate(grid,path,[3,0],.2,.3,1.05,.04,.3,.25,[-2,0,0]) is None
    assert observation_candidate(grid,path,[3,0],.2,.3,1.05,.04,.3,.001,[-2,0,np.pi]) is None
    blocked=distance_transform_edt(~grid.occupied)*grid.resolution<=.2
    assert all(grid.clear_line(a,b,blocked) for a,b in zip(curve,curve[1:]))


def test_tentative_tag_view_ignores_bad_normal_and_keeps_clearance():
    from planning.exploration import tag_view_path
    from scipy.ndimage import distance_transform_edt
    occupied=np.zeros((60,60),bool);occupied[[0,-1],:]=True;occupied[:,[0,-1]]=True
    grid=Grid(occupied,.1,[-3,-3])
    # A distant initial PnP estimate can lie beyond the wall; view from free space.
    path=tag_view_path(grid,np.array([0.,0.,0.]),[-1.5,3.5],.3,1.4,16)
    assert path and np.linalg.norm(np.asarray(path[-1])-[-1.5,3.5])<1.400001
    blocked=distance_transform_edt(~grid.occupied)*grid.resolution<=.3
    assert all(grid.clear_line(a,b,blocked) for a,b in zip(path,path[1:]))
