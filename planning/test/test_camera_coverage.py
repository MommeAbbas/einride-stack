from pathlib import Path
import math
import numpy as np
import yaml
from nav_msgs.msg import OccupancyGrid
from planning.camera_coverage import WallCoverage,validation_hold


def parameters():return yaml.safe_load((Path(__file__).parents[1]/'config/coverage.yaml').read_text())['/**']['ros__parameters']


def room():
    grid=np.zeros((30,30),dtype=int);grid[[0,-1],:]=100;grid[:,[0,-1]]=100
    msg=OccupancyGrid();msg.info.height=msg.info.width=30;msg.info.resolution=.1;msg.info.origin.orientation.w=1.;msg.data=grid.ravel().tolist();return msg


def test_range_incidence_occlusion_and_seen_persistence():
    p=parameters();world=WallCoverage(room(),p)
    visible=world.visible([1.5,1.5],0.)
    assert len(visible)>0 and np.all(world.points[visible,0]>2.8)
    assert not len(world.visible([1.5,1.5],math.pi/2)[np.isin(world.visible([1.5,1.5],math.pi/2),visible)])
    p['coverage_detection_range']=.5;assert not len(world.visible([1.5,1.5],0.));p['coverage_detection_range']=2.5
    world.seen[visible]=True
    grown=WallCoverage(room(),p,world);assert np.array_equal(world.seen,grown.seen)
    message=room();a=np.array(message.data).reshape(30,30);a[:,20]=100;message.data=a.ravel().tolist()
    obstructed=WallCoverage(message,p)
    assert np.all(obstructed.points[obstructed.visible([1.5,1.5],0.),0]<2.2)


def test_hold_observes_validator_spacing_not_only_seven_hz_camera():
    p=parameters();assert validation_hold(p)>p['coverage_required_sightings']/7.
    assert validation_hold(p)>=(p['coverage_required_sightings']-1)*p['coverage_sighting_interval']


def test_mapping_handoff_requires_no_frontiers_valid_docks_and_saved_snapshot():
    import rclpy
    from std_msgs.msg import String
    from interfaces.msg import DockMap
    from planning.coverage_node import CoverageNode
    rclpy.init();node=CoverageNode()
    try:
        node.world=WallCoverage(room(),node.p)
        node.docks=DockMap()
        for d in node.docks.docks:
            d.valid=True;d.tag_pose.covariance[0]=d.tag_pose.covariance[7]=.005;d.tag_pose.covariance[35]=.01
        node.remaining=True;node.tick();assert node.phase=='LIDAR'
        node.remaining=False;node.docks.docks[7].tag_pose.covariance[0]=.1
        node.tick();assert node.phase=='LIDAR'  # High-covariance dock is not completion.
        node.docks.docks[7].tag_pose.covariance[0]=.005
        node.tick();assert node.phase=='FINISHING'
        node.slam_status(String(data='snapshot_saved=False'));assert node.phase=='FINISHING'
        node.slam_status(String(data='snapshot_saved=True'));assert node.phase=='DONE'
        node.tick();assert node.phase=='DONE'
    finally:node.destroy_node();rclpy.shutdown()


def test_processed_camera_frustum_not_robot_heading_marks_seen():
    p=parameters();world=WallCoverage(room(),p)
    # Optical x right=-world y; optical y down=-world z; optical z forward=world x.
    frame=dict(stamp_s=1.,origin=[1.5,1.5,.15],rotation=[[0,0,1],[-1,0,0],[0,-1,0]],width=1280,height=720,fx=920.,fy=920.,cx=640.,cy=360.)
    count=world.observe(frame)
    assert count>0 and world.seen.sum()==count
    assert np.all(world.points[world.seen,0]>2.8)


def test_ready_dock_map_disables_search_and_cannot_be_resumed():
    import rclpy
    from std_msgs.msg import Bool
    from interfaces.msg import DockMap
    from planning.coverage_node import CoverageNode
    rclpy.init(); node=CoverageNode()
    try:
        node.phase='CAMERA'; node.on_docks(DockMap(ready=True))
        assert node.phase=='DONE'
        node.pause(Bool(data=False))
        node.explore_status(type('Status',(),{'status':3,'EXPLORATION_COMPLETE':3})())
        assert node.phase=='DONE' and node.frontier is None and node.goal is None
    finally:node.destroy_node();rclpy.shutdown()


def test_qualified_only_view_has_no_gain_and_new_camera_frame_is_idempotent():
    from interfaces.msg import DockLandmark
    p=parameters();world=WallCoverage(room(),p)
    frame=dict(stamp_s=1.,origin=[1.5,1.5,.15],rotation=[[0,0,1],[-1,0,0],[0,-1,0]],width=1280,height=720,fx=920.,fy=920.,cx=640.,cy=360.)
    world.observe(frame);seen=world.seen.copy();world.observe(frame)
    assert np.array_equal(world.seen,seen)
    # All remaining unseen cells belong to a qualified dock's neighbourhood.
    world.seen[:]=True;near=np.linalg.norm(world.points-[2.95,1.5],axis=1)<.5;world.seen[near]=False
    dock=DockLandmark();dock.valid=True;dock.tag_pose.pose.position.x=2.95;dock.tag_pose.pose.position.y=1.5
    assert world.candidates(np.array([1.5,1.5,0.]),world,[],0.,qualified=[dock])==[]


def test_validation_while_driving_cancels_view_without_extra_hold():
    import rclpy
    from interfaces.msg import DockMap
    from planning.coverage_node import CoverageNode
    rclpy.init();node=CoverageNode()
    try:
        node.phase='DRIVE';node.target_docks=[2,3];m=DockMap()
        for ident in [2,3]:
            d=m.docks[ident];d.valid=True;d.tag_pose.covariance[0]=d.tag_pose.covariance[7]=.005;d.tag_pose.covariance[35]=.01
        node.on_docks(m)
        assert node.phase=='CAMERA' and node.target_docks==[] and node.goal is None
    finally:node.destroy_node();rclpy.shutdown()


def test_explicit_dock_identity_survives_raster_occlusion_and_qualification():
    from interfaces.msg import DockMap
    p=parameters();world=WallCoverage(room(),p);world.seen[:]=True
    m=DockMap();d=m.docks[2];d.dock_id=2
    d.tag_pose.pose.position.x=2.95;d.tag_pose.pose.position.y=1.5
    d.tag_pose.pose.orientation.z=1.;d.tag_pose.pose.orientation.w=0.  # outward normal toward room centre
    world.dock_visible=lambda *args:False  # mapped tag just behind raster wall
    choices=world.candidates(np.array([1.5,1.5,0.]),world,[],0.,docks=[d])
    assert choices and all(c[5]==(2,) for c in choices)
    import rclpy
    from planning.coverage_node import CoverageNode
    from types import SimpleNamespace
    rclpy.init();node=CoverageNode()
    try:
        node.docks=m;d.valid=True;node.phase='PLAN'
        node.nav=SimpleNamespace(server_is_ready=lambda:True)
        node.drive(choices[0])  # Async path check finished after moving-camera validation.
        assert node.phase=='CAMERA' and node.goal_pending is None
    finally:node.destroy_node();rclpy.shutdown()


def test_frontier_clearance_rejects_prop_but_allows_unknown_boundary():
    p=parameters();m=room();a=np.array(m.data).reshape(30,30)
    a[15,15]=100;a[15,23:]=-1;m.data=a.ravel().tolist();world=WallCoverage(m,p)
    assert not world.frontier_clear([1.75,1.55],.30)
    assert world.frontier_clear([2.25,1.55],.30)
    assert world.frontier_clear([2.45,1.55],.30)  # explore_lite centroid may be unknown.
    assert not world.clear([2.45,1.55],.30)  # Camera viewpoints must be known free.


def test_obstacle_rejection_returns_error_that_explore_lite_blacklists():
    import rclpy
    from rclpy.task import Future
    from nav2_msgs.action import NavigateToPose,ComputePathToPose
    from planning.coverage_node import CoverageNode
    from types import SimpleNamespace
    rclpy.init();node=CoverageNode()
    try:
        request=NavigateToPose.Goal();request.pose.pose.position.x=.05;request.pose.pose.position.y=.05
        handle=SimpleNamespace(request=request,is_cancel_requested=False,is_active=True,abort=lambda:None)
        result=Future();node.frontier=(handle,result);node.world=WallCoverage(room(),node.p)
        node.nav=SimpleNamespace(server_is_ready=lambda:True)
        node.frontier_tick()
        assert result.done() and result.result().error_code==ComputePathToPose.Result.GOAL_OCCUPIED
    finally:node.destroy_node();rclpy.shutdown()
