"""Existing frontier/tag-view selection adapted to goal-only Nav2 execution.

No custom path search: clearance filters candidate endpoints, Smac checks reachability.
"""
import math
import numpy as np
from scipy.ndimage import distance_transform_edt


class ExplorationMap:
    def __init__(self, message, threshold):
        self.raw=np.asarray(message.data).reshape(message.info.height,message.info.width)
        self.resolution=message.info.resolution
        self.origin=np.array([message.info.origin.position.x,message.info.origin.position.y])
        q=message.info.origin.orientation;angle=2*math.atan2(q.z,q.w)
        self.rotation=np.array([[math.cos(angle),-math.sin(angle)],[math.sin(angle),math.cos(angle)]])
        self.free=(self.raw>=0)&(self.raw<threshold)
        self.clearance=distance_transform_edt(self.free)*self.resolution

    def point(self,cell):return self.origin+self.rotation @ ((np.array(cell)[::-1]+.5)*self.resolution)

    def clear(self,point,radius):
        col,row=np.floor((self.rotation.T @ (np.asarray(point)-self.origin))/self.resolution).astype(int)
        return 0<=row<self.raw.shape[0] and 0<=col<self.raw.shape[1] and self.clearance[row,col]>radius

    def frontier(self,pose,visited,radius,band,spacing,stride):
        # Same free/unknown band, candidate stride and distance ranking as exploration.py.
        distance=distance_transform_edt(self.raw>=0)*self.resolution
        cells=np.argwhere(self.free&(self.clearance>radius)&(distance<band))
        candidates=[]
        for cell in cells[::stride]:
            point=self.point(cell);d=np.linalg.norm(point-pose[:2])
            if d<spacing or any(np.linalg.norm(point-p)<spacing for p in visited):continue
            candidates.append((d,point))
        return min(candidates,key=lambda item:item[0])[1] if candidates else None

    def tag_view(self,pose,tag,visited,radius,standoff,samples,spacing):
        tag=np.asarray(tag);angle=np.arctan2(pose[1]-tag[1],pose[0]-tag[0])
        angles=angle+np.arange(samples)*2*np.pi/samples
        candidates=tag+standoff*np.c_[np.cos(angles),np.sin(angles)]
        for point in sorted(candidates,key=lambda p:np.linalg.norm(p-pose[:2])):
            if self.clear(point,radius) and not any(np.linalg.norm(point-p)<spacing for p in visited):return point
        return None

    def coverage_view(self,pose,visited,wall_distance,spacing,stride):
        """Sensor-map viewpoints for tags not yet seen; no arena/truth coordinates."""
        cells=np.argwhere(self.free&(self.clearance>=wall_distance))
        candidates=[]
        for cell in cells[::stride]:
            point=self.point(cell)
            if any(np.linalg.norm(point-p)<spacing for p in visited):continue
            candidates.append((np.linalg.norm(point-pose[:2]),point))
        return min(candidates,key=lambda item:item[0])[1] if candidates else None
