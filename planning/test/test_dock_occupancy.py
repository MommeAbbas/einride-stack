import numpy as np
from nav_msgs.msg import OccupancyGrid
from planning.dock_occupancy import dynamic_points,occupied,standoff,DockAdmission


def grid(raw,origin=(-2.,-2.)):
    m=OccupancyGrid();m.info.resolution=.1;m.info.height,m.info.width=raw.shape
    m.info.origin.position.x,m.info.origin.position.y=origin;m.info.origin.orientation.w=1.
    m.data=raw.ravel().tolist();return m


def test_wall_and_inflation_are_not_dock_occupancy():
    static=np.zeros((40,40),int);static[20,20]=100
    live=static.copy();live[20,21]=99;live[20,22]=75
    assert not occupied(dynamic_points(grid(live),grid(static)),[0,0],.5)
    live[21,20]=100
    assert occupied(dynamic_points(grid(live),grid(static)),[0,0],.5)
    assert not occupied(dynamic_points(grid(live),grid(static)),[1,1],.5)


def test_shifted_live_grid_is_aligned_to_static():
    static=np.zeros((40,40),int);static[20,20]=100
    live=np.zeros((40,40),int);live[20,19]=100
    assert len(dynamic_points(grid(live,(-1.9,-2)),grid(static)))==0


def test_clear_interval_restarts_on_reoccupation_and_stale_data():
    policy=DockAdmission(2.)
    assert not policy.update(0,True)
    assert not policy.update(1,False)
    assert not policy.update(2.9,False)
    assert not policy.update(3,True)
    assert not policy.update(4,False)
    assert not policy.update(6,False,valid=False)
    assert not policy.update(7,False)
    assert policy.update(9,False)
    assert policy.target is None


def test_standoff_uses_clear_side_outside_exit_corridor():
    raw=np.zeros((40,40),int);raw[22:27,9:14]=100
    # Goal heading +x, behind approach by .8, sideways .4. Upper side blocked.
    point=standoff([0,0],0,.8,.4,[0,0],grid(raw),.168)
    assert np.allclose(point,[-.8,-.4])
    raw[13:18,9:14]=100
    assert standoff([0,0],0,.8,.4,[0,0],grid(raw),.168) is None

def test_only_recent_camera_confirmed_precise_mixture_claims_occupancy():
    from types import SimpleNamespace as NS
    from builtin_interfaces.msg import Time,Duration
    from planning.dock_occupancy import confirmed_track_points
    group=NS(probability=1.,points=[NS(x=.1,y=.1,covariance=[.01,0.,0.,.01])])
    robot=NS(track_id=0,ever_confirmed=False,confirmation_age=Duration(),groups=[],leftover=group)
    prediction=NS(robots=[robot]);receipts=NS(detections=[NS(id='0',header=NS(stamp=Time(sec=1)))])
    query=lambda:confirmed_track_points(prediction,2.,receipts,2.,.3)
    assert query()==[]
    robot.ever_confirmed=True
    assert occupied(query(),[0,0],.5)
    # A new lidar confirmation cannot refresh the camera receipt.
    receipts.detections[0].header.stamp.sec=0
    assert confirmed_track_points(prediction,2.,receipts,2.1,.3)==[]
    receipts.detections[0].header.stamp.sec=1
    group.points[0].covariance=[.1,0.,0.,.01]
    assert query()==[]
    # Two precise modes far apart do not count as one precise mean.
    group.points[0].covariance=[.01,0.,0.,.01];group.probability=.5
    robot.groups=[NS(probability=.5,points=[NS(x=2.,y=2.,covariance=[.01,0.,0.,.01])])]
    assert query()==[]


def test_costmap_source_attribution_does_not_invent_persistent_provenance():
    from planning.dock_occupancy import costmap_cues
    points=np.array([[.05,.05]])
    sources={'lidar':(1.,points.copy()),'depth':(1.,points.copy())}
    assert costmap_cues(points,[0,0],.5,sources,1.1,1.,.04)==['costmap_lidar','costmap_depth']
    assert costmap_cues(points,[0,0],.5,sources,3.,1.,.04)==['costmap_retained_unattributed']


def test_full_standoff_envelope_must_fit_known_map():
    # Candidate centres inside, but both robot envelopes overlap the edge.
    raw=np.zeros((40,40),int)
    assert standoff([-1.1,0],0,.8,.4,[0,0],grid(raw),.168) is None


def test_invalid_observation_never_releases_even_before_first_detection():
    policy=DockAdmission(2.)
    assert not policy.update(0,False,False)
    assert policy.update(1,False,True)


def test_unknown_static_cells_cannot_be_claimed_new_obstacles():
    static=np.zeros((40,40),int);static[20,20]=-1
    live=np.zeros((40,40),int);live[20,20]=100
    assert len(dynamic_points(grid(live),grid(static)))==0


def test_scan_map_boundary_alias_does_not_hold_an_empty_dock():
    static=np.zeros((40,40),int);static[:,5]=100
    live=static.copy();live[15:25,6]=100
    assert len(dynamic_points(grid(live),grid(static),.11))==0
    live[20,9]=100
    assert len(dynamic_points(grid(live),grid(static),.11))==1
