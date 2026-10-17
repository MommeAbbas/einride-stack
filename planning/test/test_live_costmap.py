from types import SimpleNamespace as N
import numpy as np
from nav_msgs.msg import OccupancyGrid
from planning.planning_node import PlanningNode


def test_live_obstacle_wall_blocks_existing_astar_and_stale_map_is_rejected():
    node=N(p=lambda k:{'occupied_threshold':50,'costmap_stale_timeout':1.}[k])
    msg=OccupancyGrid();msg.header.frame_id='map';msg.header.stamp.sec=10
    msg.info.resolution=.1;msg.info.width=msg.info.height=30;msg.data=[0]*900
    PlanningNode.on_costmap(node,msg)
    assert node.live_grid.plan([.5,1.5],[2.5,1.5],.1)
    raw=np.zeros((30,30),dtype=int);raw[:,15]=100;msg.data=raw.ravel().tolist()
    PlanningNode.on_costmap(node,msg)
    assert not node.live_grid.plan([.5,1.5],[2.5,1.5],.1)
    assert PlanningNode.mission_grid(node,10.5) is node.live_grid
    assert PlanningNode.mission_grid(node,12.) is None
