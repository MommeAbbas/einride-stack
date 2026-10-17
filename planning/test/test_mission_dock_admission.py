"""Occupied dock mission transitions before target-entry logic."""
from types import SimpleNamespace as NS
import numpy as np
from geometry_msgs.msg import Pose
from interfaces.msg import DockLandmark
from planning.mission_node import MissionNode
from planning.dock_occupancy import DockAdmission
from test_dock_occupancy import grid


def fixture():
    node=MissionNode.__new__(MissionNode)
    params=dict(costmap_stale_timeout=1.,dock_occupancy_radius=.5,dock_use_predictions=False,
        dock_standoff_back=.8,dock_standoff_side=.4,robot_radius=.168,obstacle_clearance=.13,goal_tolerance=.09)
    node.p=lambda key:params[key]
    node.static_map=grid(np.zeros((40,40),int));node.live_costmap=grid(np.zeros((40,40),int))
    node.dynamic_obstacles=np.array([[.05,0.]])
    node.source_points={};node.admission=DockAdmission(2.);node.predictions=None;node.request=None;node.status=None;node.revision=1
    node.phase='dock';node.path=[];node.standoff_active=False;node.commands=[]
    node.issue=lambda *args,**kwargs:node.commands.append(args)
    dock=DockLandmark();dock.approach_pose.pose.position.x=-.3;dock.approach_pose.pose.orientation.w=1.
    dock.docking_pose.pose.orientation.w=1.
    return node,dock


def test_occupied_goal_redirects_before_entry_then_waits_and_resumes_approach():
    n,d=fixture();pose=np.array([-1.5,1.,0.])
    assert n.dock_admission(d,pose,0.)
    assert n.commands[-1][0]=='DRIVE' and n.standoff_active
    target=n.admission.target.copy();pose[:2]=target
    assert n.dock_admission(d,pose,.1)
    assert n.commands[-1][0]=='WAIT'
    n.dynamic_obstacles=np.empty((0,2))
    n.live_costmap.header.stamp.sec=1
    assert n.dock_admission(d,pose,1.)
    n.live_costmap.header.stamp.sec=2
    assert n.dock_admission(d,pose,2.9)
    n.live_costmap.header.stamp.sec=3
    assert not n.dock_admission(d,pose,3.)
    assert not n.standoff_active and n.phase=='approach'


def test_stale_costmap_does_not_authorize_entry():
    n,d=fixture();n.dynamic_obstacles=np.empty((0,2))
    assert n.dock_admission(d,np.array([-1.,1.,0.]),2.)
    assert n.commands[-1][0]=='WAIT'


def test_recent_precise_camera_track_can_hold_dock_without_costmap_mark():
    from builtin_interfaces.msg import Time
    n,d=fixture();n.dynamic_obstacles=np.empty((0,2));old=n.p
    extra=dict(dock_use_predictions=True,dock_track_max_age=2.,dock_track_max_sigma=.3,prediction_stale_timeout=1.)
    n.p=lambda key:extra[key] if key in extra else old(key)
    group=NS(probability=1.,points=[NS(x=0.,y=0.,covariance=[.01,0.,0.,.01])])
    robot=NS(track_id=0,ever_confirmed=True,groups=[],leftover=group)
    n.predictions=NS(map_revision=1,header=NS(stamp=Time()),robots=[robot])
    n.camera_tracks=NS(detections=[NS(id='0',header=NS(stamp=Time()))])
    assert n.dock_admission(d,np.array([-1.5,1.,0.]),0.)
    assert n.dock_occupied and n.occupancy_cues==['camera_supported_track']
    assert n.commands[-1][0]=='DRIVE' and n.standoff_active


def test_package_dock_receipt_then_undock_before_new_delivery():
    from interfaces.msg import CoordinatorStatus
    from types import SimpleNamespace as NS
    n,d=fixture();n.arrived=False;n.docked_at=None
    n.publish_trajectory=lambda *a:None
    n.package_docking(d,3,None,0.)
    assert n.commands[-1][1]=='DockRobot:3'
    n.request=NS(request_id='dock');n.status=NS(request_id='dock',terminal=True,status=CoordinatorStatus.SUCCEEDED)
    n.package_docking(d,3,None,1.)
    assert n.arrived and n.docked_at==3 and n.commands[-1][0]=='WAIT'
    n.arrived=False;n.request=None;n.status=None
    n.package_docking(d,4,None,2.,undock=True)
    assert n.commands[-1][1]=='UndockRobot:3'
    n.request=NS(request_id='leave');n.status=NS(request_id='leave',terminal=True,status=CoordinatorStatus.SUCCEEDED)
    n.package_docking(d,4,None,3.,undock=True)
    assert n.docked_at is None


def test_next_delivery_at_same_dock_keeps_receipt_without_restaging():
    n,d=fixture();n.docked_at=3;n.arrived=False
    n.publish_trajectory=lambda *a:None
    n.package_docking(d,3,None,0.)
    assert n.arrived and n.commands[-1][0]=='WAIT'
