import numpy as np
from nav_msgs.msg import OccupancyGrid
from planning.nav2_exploration import ExplorationMap


def arena():
    m=OccupancyGrid();m.info.resolution=.1;m.info.height=40;m.info.width=40;m.info.origin.orientation.w=1.
    raw=np.zeros((40,40),int);raw[:,30:]=-1;raw[0,:]=100;raw[-1,:]=100;raw[:,0]=100
    m.data=raw.flatten().tolist();return ExplorationMap(m,50)


def test_frontier_endpoints_are_free_and_failed_target_is_not_selected_again():
    m=arena();pose=np.array([1.,2.,0.]);target=m.frontier(pose,[],.3,.6,.4,1)
    assert target is not None and m.clear(target,.3)
    other=m.frontier(pose,[target],.3,.6,.4,1)
    assert other is not None and np.linalg.norm(target-other)>=.4


def test_tag_view_uses_observed_tag_position_and_avoids_failed_view():
    m=arena();pose=np.array([1.,2.,0.]);a=m.tag_view(pose,[2.,2.],[],.3,.7,16,.4)
    b=m.tag_view(pose,[2.,2.],[a],.3,.7,16,.4)
    assert a is not None and b is not None and np.linalg.norm(a-b)>=.4
    assert abs(np.linalg.norm(a-[2.,2.])-.7)<1e-8


def test_successful_nav2_frontier_advances_when_endpoint_was_tolerance_adjusted():
    from types import SimpleNamespace as NS
    from planning.mission_node import MissionNode
    from interfaces.msg import CoordinatorRequest,CoordinatorStatus
    node=MissionNode.__new__(MissionNode)
    node.explorer=None;node.exploration_target=np.array([1.,1.]);node.exploration_tag=None
    node.request=NS(request_id='done');node.status=NS(request_id='done',terminal=True,status=CoordinatorStatus.SUCCEEDED)
    node.visited=[];node.p=lambda key:.09;calls=[];node.issue=lambda *a,**k:calls.append(a)
    node.explore(np.array([.85,1.,0.]),10.)
    assert node.exploration_target is None and len(node.visited)==1
    assert calls[-1][0]=='WAIT' and node.request is None
