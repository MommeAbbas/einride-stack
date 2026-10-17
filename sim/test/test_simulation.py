"""Phase 1 logic checks: geometry, routes, two-clock dwell, camera and scoring contracts."""
from pathlib import Path
import math
import xml.etree.ElementTree as ET
import numpy as np
import pytest
import yaml
from ament_index_python.packages import get_package_share_directory
from sim.collision import CollisionMonitor
from sim.geometry import rectangle, overlap, separation
from sim.grid import Grid
from sim.world import generate
from sim.organiser import Organiser
from sim.detector import project_box, emulate
from sim.history import PoseHistory
from sim.metrics import mixture_moments, ellipse_hits, interpolate_points, look_confirmed
from sim.traffic import command


@pytest.fixture
def params():
    return yaml.safe_load((Path(__file__).parents[1]/'config/sim.yaml').read_text())['/**']['ros__parameters']


def test_oriented_contact_not_bounding_circle():
    a=rectangle(0,0,0,.4,.1); b=rectangle(0,.15,0,.4,.1)
    assert not overlap(a,b)
    assert separation(a,b)==pytest.approx(.05)
    assert overlap(a,rectangle(0,.15,math.pi/2,.4,.1))


def test_contact_onset_major_minor_and_rearm():
    monitor=CollisionMonitor(.245,.23,.35,.015)
    a=((0,0,0),(0,0),True); b=((.1,0,0),(.4,0),True)
    events,_=monitor.update({0:a,1:b})
    assert len(events)==1 and events[0]['severity']==2
    assert not monitor.update({0:a,1:b})[0]
    monitor.update({0:a,1:((1,0,0),(0,0),True)})
    assert monitor.update({0:a,1:((.1,0,0),(0,0),False)})[0][0]['speed_valid'] is False


def test_astar_avoids_obstacle_and_checks_segment_clearance():
    occupied=np.zeros((20,20),bool); occupied[5:15,10]=True
    grid=Grid(occupied,.1,[0,0])
    path=grid.plan([.3,1],[1.7,1],.12)
    assert len(path)>2
    from scipy.ndimage import distance_transform_edt
    blocked=distance_transform_edt(~occupied)*.1<=.12
    assert all(grid.clear_line(a,b,blocked) for a,b in zip(path,path[1:]))
    assert not grid.plan([1.05,1.05],[1.7,1],.12)


@pytest.mark.parametrize('count',[0,1,2,3])
def test_world_topics_no_traffic_sensors_and_all_docks_reachable(tmp_path,params,count):
    params['traffic_count']=count
    grid,docks=generate(get_package_share_directory('einride_mini_truck_gazebo'),
                        get_package_share_directory('einride_mini_truck_description'),tmp_path,params)
    world=ET.parse(tmp_path/'arena.sdf').getroot().find('world')
    traffic=[m for m in world.findall('model') if m.get('name','').startswith('traffic_')]
    assert len(traffic)==count
    for i,model in enumerate(traffic,1):
        assert not model.findall('.//sensor')
        assert len(model.findall('plugin'))==1
        assert model.findtext('plugin/topic')==f'/traffic_{i}/cmd_vel'
        assert model.findtext('plugin/odom_topic')==f'/traffic_{i}/odom'
    radius=math.hypot(params['robot_length'],params['robot_width'])/2+params['obstacle_clearance']
    for dock,pose in docks.items():
        assert grid.plan([0,0],pose[:2],radius),dock
    for i in range(count):
        start=params['traffic_spawns'][3*i:3*i+2]
        assert grid.plan(start,docks['A'][:2],radius)


def test_snapshot_respawn_uses_separate_physical_handover(tmp_path,params):
    import json
    params.update(map_source='snapshot',map_snapshot=str(tmp_path/'slam_snapshot.json'),traffic_count=0)
    physical=[1.2,-.8,.01,0.,0.,.9]
    (tmp_path/'sim_handover.json').write_text(json.dumps({'world_pose':physical}))
    generate(get_package_share_directory('einride_mini_truck_gazebo'),get_package_share_directory('einride_mini_truck_description'),tmp_path,params)
    world=ET.parse(tmp_path/'arena.sdf').getroot().find('world')
    ego=next(i for i in world.findall('include') if i.findtext('uri')=='model://einride_mini_truck')
    assert [float(v) for v in ego.findtext('pose').split()]==physical


def route(status):
    return {'route_id':1,'status':status}


def test_dwell_requires_both_clocks_and_stopped_presence():
    organiser=Organiser(3.,3.)
    r=route('loading_pending')
    assert organiser.decide(2,r,False,10,100) is None
    assert organiser.decide(2,r,True,10,100)=='arrived_at_source'
    organiser.acknowledged(2,r,'arrived_at_source',10,100)
    r=route('loading_in_progress')
    assert organiser.decide(2,r,True,12,110) is None # Slow sim: wall lock expired, sim dwell not done.
    assert organiser.decide(2,r,True,14,102) is None # Fast sim: sim dwell done, wall lock not done.
    assert organiser.decide(2,r,True,14,110)=='loading_complete'
    assert organiser.decide(2,r,False,14,110) is None
    assert organiser.decide(2,r,True,16,112) is None
    assert organiser.decide(2,r,True,17,113)=='loading_complete'


def test_dwell_clock_rewind_and_new_route_restart():
    organiser=Organiser(3.,3.)
    r=route('unloading_in_progress')
    assert organiser.decide(1,r,True,10,100) is None
    assert organiser.decide(1,r,True,1,101) is None
    assert organiser.decide(1,r,True,4,104)=='unloading_complete'
    r['route_id']=2
    assert organiser.decide(1,r,True,5,105) is None


def camera_setup():
    # Optical z faces world +X; x right=-Y; y down=-Z.
    camera=np.array([[0,0,1,0],[-1,0,0,0],[0,-1,0,.124],[0,0,0,1]],dtype=float)
    robot=np.eye(4); robot[0,3]=2
    intrinsics=np.array([[916.,0,640],[0,916.,360],[0,0,1]])
    return camera,robot,intrinsics


def test_camera_projection_real_optical_axes_and_fov():
    camera,robot,k=camera_setup()
    box=project_box(robot,camera,k,[1280,720],[.245,.23,.19],1.219,.748,.2,8)
    assert box is not None and box[0]<640<box[2] and box[1]<360<box[3]
    robot[1,3]=3
    assert project_box(robot,camera,k,[1280,720],[.245,.23,.19],1.219,.748,.2,8) is None


def test_detector_misses_occlusion_empty_frames_and_seed(params):
    camera,robot,k=camera_setup(); grid=Grid(np.zeros((80,80),bool),.1,[-4,-4])
    params.update(detector_miss_rate=0.,detector_box_noise=1.)
    args=({1:robot},camera,k,[1280,720],[.245,.23,.19],params,grid)
    boxes=emulate(*args,np.random.default_rng(7)); assert len(boxes)==1
    assert np.array_equal(boxes,emulate(*args,np.random.default_rng(7)))
    params['detector_miss_rate']=1.; assert not emulate(*args,np.random.default_rng(7))
    params['detector_miss_rate']=0.; grid.occupied[:,50]=True
    assert not emulate(*args,np.random.default_rng(7))
    params['detector_map_occlusion']=False; assert len(emulate(*args,np.random.default_rng(7)))==1
    assert not emulate({},camera,k,[1280,720],[.245,.23,.19],params,grid,np.random.default_rng(7))


def test_capture_history_no_extrapolation_or_large_gap():
    history=PoseHistory(2.)
    a=np.eye(4); b=np.eye(4); b[0,3]=1.
    history.add(1.,{0:a}); history.add(2.,{0:b})
    assert history.at(1.5,1.1)[0][0,3]==pytest.approx(.5)
    assert history.at(3.,1.1) is None
    assert history.at(1.5,.1) is None


def test_mixture_includes_between_mode_uncertainty_and_leftover():
    result=mixture_moments([(.5,np.array([-1.,0]),np.eye(2)),(.5,np.array([1.,0]),np.eye(2))],1e-6)
    assert np.allclose(result[0],[0,0]) and np.allclose(result[1],np.diag([2,1]))
    assert mixture_moments([(.5,np.zeros(2),np.eye(2))],1e-6) is None
    assert mixture_moments([(1.,np.zeros(2),-np.eye(2))],1e-6) is None


def test_ellipse_expected_coverage_is_two_dimensional():
    one,two,p1,p2=ellipse_hits([.5,0],np.zeros(2),np.eye(2),1e-9)
    assert one==two==1
    assert p1==pytest.approx(.3934693403) and p2==pytest.approx(.8646647168)
    assert ellipse_hits([1.5,0],np.zeros(2),np.eye(2),1e-9)[:2]==(0,1)


def test_predictions_interpolate_absolute_times_never_extrapolate():
    points=[(10.,np.zeros(2),np.eye(2)),(12.,np.ones(2)*2,np.eye(2)*3)]
    assert np.allclose(interpolate_points(points,11)[0],[1,1])
    assert interpolate_points(points,9) is None
    assert interpolate_points(points,13) is None


def test_look_receipt_strict_order_and_applied_evidence():
    assert look_confirmed(10,11,True,True,12)
    assert not look_confirmed(10,10,True,True,12)
    assert not look_confirmed(10,11,True,False,12)
    assert not look_confirmed(10,11,False,True,12)
    assert not look_confirmed(10,11,True,True,11)


def test_traffic_rotation_and_goal_hold():
    v,w,done=command([0,0,math.pi],[[1,0]],.4,1.2,2.5,.55,.08,.08)
    assert v==0 and abs(w)<=1.2 and not done
    assert command([1,0,0],[[1,0]],.4,1.2,2.5,.55,.08,.08)==(0.,0.,True)


@pytest.mark.parametrize('status,expected', [('loading_pending','A'),('loading_in_progress','A'),
    ('loading_complete','B'),('unloading_in_progress','B')])
def test_organiser_uses_correct_dock_after_loading(status,expected):
    from sim.organiser import target_dock
    assert target_dock({'source':'A','destination':'B','status':status})==expected


def test_runner_ros_parameter_overrides_scenario_defaults(tmp_path):
    import rclpy
    from sim.runner import ExperimentRunner
    rclpy.init(args=['--ros-args','-p','duration:=15.0'])
    node=ExperimentRunner({'parameters':{'duration':110.}},str(tmp_path),str(tmp_path))
    try:
        assert node.p('duration')==15.
    finally:
        node.destroy_node(); rclpy.shutdown()


@pytest.mark.parametrize('mode,expected',[('ignore',(.4,.1)),('stop_near',(0.,0.)),('random_wander',(.4,.1))])
def test_only_stop_near_reacts_to_ego(mode,expected):
    from sim.traffic import mode_command
    assert mode_command(mode,(.4,.1),(0,0,0),(.1,0,0),.6)==expected


def test_saga_http_200_can_still_mean_button_rejected():
    from sim.saga_client import SagaClient
    class Response:
        headers={'HX-Trigger':'{"saga-error":"wait before confirming"}'}
        def raise_for_status(self):
            pass
    class Session:
        def post(self,*args,**kwargs):
            return Response()
    api=SagaClient('http://127.0.0.1:8000',.5)
    api.session=Session()
    assert not api.action(2,'loading_complete','loading_in_progress')
    with pytest.raises(ValueError):
        SagaClient('https://example.com',.5)


def test_snapshot_spawn_overlap_rejected_before_physics(tmp_path,params):
    import json
    params.update(map_source='snapshot',map_snapshot=str(tmp_path/'slam_snapshot.json'),traffic_count=1)
    (tmp_path/'sim_handover.json').write_text(json.dumps({'world_pose':[1.039556,-.958686,0.,0.,0.,-.001724]}))
    args=(get_package_share_directory('einride_mini_truck_gazebo'),get_package_share_directory('einride_mini_truck_description'),tmp_path,params)
    with pytest.raises(ValueError,match='Initial robot footprint overlap'):generate(*args)
    params['traffic_spawns'][0]=.4
    generate(*args)


def test_competition_camera_private_world(tmp_path,params):
    params.update(traffic_count=0,ego_camera_width=640,ego_camera_height=360,ego_camera_rate=5.)
    generate(get_package_share_directory('einride_mini_truck_gazebo'),get_package_share_directory('einride_mini_truck_description'),tmp_path,params)
    world=ET.parse(tmp_path/'arena.sdf').getroot().find('world')
    ego=world.find("model[@name='einride_mini_truck']")
    assert ego is not None
    for name in ('oak_rgb','oak_stereo'):
        camera=ego.find(f".//sensor[@name='{name}']")
        assert camera.findtext('camera/image/width')=='640'
        assert camera.findtext('camera/image/height')=='360'
        assert float(camera.findtext('update_rate'))==5.
