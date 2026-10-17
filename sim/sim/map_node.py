"""Publish the simulator oracle map only on /sim/map; this is not SLAM."""
from pathlib import Path
import numpy as np
from nav_msgs.msg import OccupancyGrid
from .ros_support import SimNode, latched, run


class MapNode(SimNode):
    def __init__(self):
        super().__init__('sim_map')
        data=np.load(Path(self.p('run_dir'))/'map.npz')
        message=OccupancyGrid(); message.header.frame_id='world'
        message.info.resolution=float(data['resolution'])
        message.info.height,message.info.width=data['occupied'].shape
        message.info.origin.position.x=float(data['origin'][0]); message.info.origin.position.y=float(data['origin'][1])
        message.info.origin.orientation.w=1.
        message.data=(data['occupied'].astype(np.int8)*100).ravel().tolist()
        self.pub=self.create_publisher(OccupancyGrid,'/sim/map',latched())
        self.pub.publish(message)


def main():
    run(MapNode)
