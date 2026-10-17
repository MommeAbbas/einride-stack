from pathlib import Path
import numpy as np
import yaml
from planning.grid import Grid
from perception.measurements import fuse_cues,surprise_groups,likelihood,camera_cues
from perception.association import marginal_associations
from perception.timing import MeasurementQueue
from perception.filter import Belief,systematic
from perception.output import predictions
from perception.visibility import visible


def params():
    return yaml.safe_load((Path(__file__).parents[1]/'config/perception.yaml').read_text())['/**']['ros__parameters']


def fixture():
    p=params(); p.update(particles_per_robot=24,horizon=0.5,prediction_dt=0.25)
    occupied=np.zeros((24,24),bool); occupied[[0,-1],:]=True; occupied[:,[0,-1]]=True
    grid=Grid(occupied,.2,[-2.4,-2.4]); docks=[[-1.6,-1.6],[-.6,-1.6],[.6,-1.6],[1.6,-1.6],[-1.6,1.6],[-.6,1.6],[.6,1.6],[1.6,1.6]]
    return p,grid,docks


def test_correlated_cue_bound():
    value,sigma=fuse_cues([(2.,.1),(2.2,.2)],.7)
    assert np.isclose(value,2.04) and np.isclose(sigma,.12)
    assert fuse_cues([],1.) is None


def test_gap_does_not_invent_wall_range():
    p=params(); groups=surprise_groups([1.,np.nan,1.], [1.,1.,1.],.01,p)
    assert groups==[(1,None,1.)]


def test_static_edge_association_keeps_robot_returns_and_nan_gap_evidence():
    from scipy.ndimage import distance_transform_edt
    from perception.visibility import static_return_mask
    p=params();a=np.zeros((40,40),bool);a[:,30]=True
    grid=Grid(a,.04,[-.4,-.8]);clearance=distance_transform_edt(~a)*grid.resolution
    ranges=np.array([.77,.4,np.nan]);angles=np.zeros(3)
    static=static_return_mask(grid,[0,0],angles,ranges,clearance,p['lidar_static_gate'])
    assert static.tolist()==[True,False,False]
    # A one-cell wall-edge mismatch is not a robot, but the clear-space return
    # and missing return still form genuine surprise evidence.
    assert surprise_groups(ranges,[1.4,1.4,1.],.01,p,static)==[(2,.4,1.)]


def test_heavy_tail_and_bearing_interval():
    p=params(); cloud=np.array([[.2,0],[.9,0],[4.,4.]])
    values=likelihood(cloud,[0,0],0,None,.1,1.,p)
    assert values[0]==values[1] and values[2]>0


def test_ambiguous_assignments_are_split():
    a=marginal_associations(np.ones((3,2)))
    assert np.allclose(a,1/3) and np.allclose(a.sum(axis=0),1)
    assert np.array_equal(marginal_associations(np.ones((3,2)),first=True),[[1,0],[0,1],[0,0]])


def test_time_order_watermarks_timeout_and_late():
    q=MeasurementQueue(['camera','lidar'],.2)
    q.push('camera',1.1,'b'); q.push('lidar',1.,'a')
    assert [v for _,_,v in q.ready(1.15)]==['a']
    assert [v for _,_,v in q.ready(1.31)]==['b']
    assert not q.push('camera',1.05,'late') and q.dropped==1


def test_negative_evidence_and_reservations():
    p,g,d=fixture(); p['resample_threshold']=0.; b=Belief(g,d,[0,0],p)
    b.clouds[0][:,:2]=np.array([[1.,0],[-1.,0]]*12)
    assert b.update([],1.,([0,0],0.,[]),negative=True)
    assert b.weights[0][0]<b.weights[0][1]
    b.reservations([0,1]); assert all(not np.isin(c[:,5],[0,1]).any() for c in b.clouds)
    assert not b.update([],2.,([0,0],0.,[]),negative=False)


def test_resampling_and_injection_geometry():
    p,g,d=fixture(); b=Belief(g,d,[0,0],p)
    assert np.array_equal(systematic(np.array([0.,1.,0.]),b.rng),[1,1,1])
    b.view=([0,0],0.,[]); fresh=b.fresh(100)
    assert not visible(fresh[:,:2],[0,0],0.,g,p).any()
    assert all(not b.blocked[g.cell(x)] for x in fresh[:,:2])


def test_prediction_mass_full_leftover_and_no_posterior_mutation():
    p,g,d=fixture(); b=Belief(g,d,[0,0],p); before=b.clouds[0].copy()
    outputs=predictions(b,.1)
    assert len(outputs)==3 and np.array_equal(before,b.clouds[0])
    for groups,leftover in outputs:
        assert np.isclose(sum(x[0] for x in groups)+leftover[0],1)
        assert len(leftover[3])==3
        assert all(np.linalg.eigvalsh(c).min()>0 for _,c in leftover[3])


def test_output_probability_bounds_survive_floating_point_normalisation():
    p,g,d=fixture();p['max_groups']=0;b=Belief(g,d,[0,0],p)
    b.weights=[w*np.nextafter(1.,2.) for w in b.weights]
    for groups,leftover in predictions(b,0.):
        assert groups==[] and 0.<=leftover[0]<=1.
        assert np.isclose(leftover[0],1.)


def test_stationary_plan_looks_toward_uncertain_support_not_ego():
    from planning.geometry import FrameGrid
    from perception.output import understanding_region
    p,g,d=fixture();b=Belief(g,d,[0,0],p);b.grid=FrameGrid(g,[0,0,0])
    for cloud in b.clouds:cloud[:,:2]=[0.,.3]
    target,mass,labels=understanding_region(b,np.array([0.,0.]),2.)
    assert np.allclose(target,[0.,.3]) and mass==1. and labels==[0,1,2]
    assert np.isclose(np.arctan2(target[1],target[0]),np.pi/2)
    b.last_support=[2.]*3
    assert understanding_region(b,np.array([0.,0.]),2.) is None


def test_map_correction_repairs_trapped_support_without_moving_valid_particles():
    from planning.geometry import FrameGrid
    p,g,d=fixture();b=Belief(g,d,[0,0],p)
    # A localisation correction moves a previously admissible sample into the
    # inflated right wall. Most samples stay valid in exactly the same odom pose.
    b.clouds[0][:,:2]=[0.,1.]
    b.clouds[0][0,:2]=[1.9,0.]
    before=b.clouds[0].copy();weights=b.weights[0].copy()
    b.last_support[0]=1.
    b.grid=FrameGrid(g,[.4,0.,0.])
    b.free=b.grid.points(np.argwhere(~b.blocked))
    b.view=([0,0],0.,[])
    b.repair_map_support([0,0])
    assert np.array_equal(b.clouds[0][1:],before[1:])
    assert not np.array_equal(b.clouds[0][0],before[0])
    assert not b.blocked[b.grid.cell(b.clouds[0][0,:2])]
    assert not visible(b.clouds[0][:1,:2],[0,0],0.,b.grid,p).any()
    assert np.array_equal(b.weights[0],weights) and b.last_support[0] is None


def test_occluded_and_cropped_camera_cues():
    p=params(); p.update(depth_cue=False,height_cue=False)
    assert camera_cues((100,190,20,20),(100,100,100,100),(200,200),None,p) is None
    assert camera_cues((100,150,20,20),(100,100,100,100),(200,200),None,p,True) is None


def test_scan_groups_interleave_with_camera_at_capture_time():
    q=MeasurementQueue(['camera','lidar'],.2)
    q.push('lidar',1.01,'first group');q.push('lidar',1.08,'second group')
    q.push('lidar',1.1,'end of scan');q.push('camera',1.05,'camera frame')
    assert [v for _,_,v in q.ready(1.31)]==['first group','camera frame','second group','end of scan']


def test_motion_stops_at_static_wall_and_preserves_dock_reservations():
    from perception.motion import advance
    p,g,d=fixture();b=Belief(g,d,[0,0],p)
    p.update(dock_steering=False,speed_noise=0.,turn_noise=0.)
    states=np.array([[2.,0.,0.,p['max_speed'],0.,2.]])
    out=advance(states,1.,b.rng,p,g,np.array(d),b.fields,b.blocked,[2,3],noise=False)
    assert np.array_equal(out[0,:2],states[0,:2]) and out[0,3]==0.
    assert np.array_equal(states[0,:2],[2.,0.])


def test_dock_tag_identity_depth_and_no_detection_on_blank_image():
    import cv2
    from perception.dock_geometry import detect_tags
    k=np.array([[400.,0,200],[0,400,200],[0,0,1]])
    image=np.full((400,400),255,np.uint8)
    assert detect_tags(image,k,np.zeros(5),.1,2.)==[]
    dictionary=cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    image[150:250,150:250]=cv2.aruco.drawMarker(dictionary,3,100)
    observations=detect_tags(image,k,np.zeros(5),.1,2.)
    assert len(observations)==1 and observations[0][0]==3
    assert abs(observations[0][1][2]-.4)<.02


def test_lidar_update_preserves_camera_exclusion_and_gaps_are_not_clear():
    from perception.visibility import lidar_clear_mask
    p,g,d=fixture();b=Belief(g,d,[0,0],p)
    b.update([],1.,([0,0],0.,[]),negative=True)
    view=b.view;b.update([],1.1)
    assert b.view is view
    assert not visible(b.fresh(100)[:,:2],[0,0],0.,g,p).any()
    points=np.array([[.5,0.],[0.,.5],[-.5,0.]])
    rays=np.array([2.,float('nan'),.4,2.])
    mask=lidar_clear_mask(points,[0,0],0.,rays,0.,np.pi/2,g,p)
    assert mask.tolist()==[True,False,False]


def test_gate_rejects_all_zero_pairs_without_uniform_fallback():
    assert not marginal_associations(np.zeros((3,2)),unmatched=.0012).any()
    e=np.array([[1.,1.],[0.,0.],[0.,0.]])
    a=marginal_associations(e,unmatched=.0012)
    assert not a[1:].any() and a[0].sum()<=1.


def test_coasting_covariance_growth_and_track_expiry_preserve_unknown_slot():
    p,g,d=fixture();p['max_groups']=0;b=Belief(g,d,[0,0],p)
    b.expire(1.);b.last_support[0]=1.
    early=predictions(b,0.,1.)[0][1][3][0][1]
    late=predictions(b,0.,3.)[0][1][3][0][1]
    assert np.allclose(late-early,np.eye(2)*2*p['coasting_variance_rate'])
    old=b.clouds[0].copy();b.expire(1.+p['unsupported_track_timeout']+.1)
    assert b.last_support[0] is None and not np.array_equal(old,b.clouds[0])
    assert np.isclose(b.weights[0].sum(),1.) and len(b.clouds)==3


def test_tight_gate_does_not_refresh_far_confirmed_track():
    p,g,d=fixture();b=Belief(g,d,[0,0],p);b.expire(1.)
    for i in range(3):b.clouds[i][:,:2]=[1.,0.];b.last_support[i]=1.
    b.update([([0,0],np.pi,1.,.1,4.)],1.1)
    assert b.last_support==[1.,1.,1.]


def test_camera_birth_recovers_missing_local_support_and_keeps_map_constraints():
    p,g,d=fixture();p['injection']=False;b=Belief(g,d,[0,0],p)
    for c in b.clouds:c[:,:2]=[-1.,-1.]
    b.update([([0,0],0.,1.,.1,4.)],1.,([0,0],0.,[]))
    confirmed=[i for i,t in enumerate(b.last_camera_support) if t==1.]
    assert len(confirmed)==1
    i=confirmed[0]
    assert np.linalg.norm(b.weights[i]@b.clouds[i][:,:2]-[1.,0.])<.2
    assert all(not b.blocked[g.cell(x)] for x in b.clouds[i][:,:2])
    b.expire(1.+p['unsupported_track_timeout']+.1)
    assert b.last_camera_support[i] is None


def test_camera_birth_does_not_reset_an_existing_supported_track():
    p,g,d=fixture();b=Belief(g,d,[0,0],p)
    b.clouds[0][:,:2]=[1.,0.];b.last_support[0]=1.;b.last_camera_support[0]=1.
    before=[c.copy() for c in b.clouds]
    b.camera_births([([0,0],0.,1.,.1,4.)],1.1)
    assert all(np.array_equal(a,c) for a,c in zip(before,b.clouds))


def test_lidar_confirmation_cannot_block_first_camera_localisation():
    p,g,d=fixture();p['injection']=False;b=Belief(g,d,[0,0],p)
    for c in b.clouds:c[:,:2]=[-1.,-1.]
    b.last_support=[1.]*3
    b.update([([0,0],0.,1.,.1,4.)],1.1,([0,0],0.,[]))
    confirmed=[i for i,t in enumerate(b.last_camera_support) if t==1.1]
    assert len(confirmed)==1
    i=confirmed[0]
    assert np.linalg.norm(b.weights[i]@b.clouds[i][:,:2]-[1.,0.])<.2
