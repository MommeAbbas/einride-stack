"""TEMPORARY frontier stand-in; no dock coordinates or truth are supplied to exploration."""
import numpy as np
from scipy.ndimage import distance_transform_edt


def frontier_path(raw,grid,pose,visited,radius,band,minimum_distance,stride):
    unknown=raw<0;free=(raw>=0)&~grid.occupied
    clearance=distance_transform_edt(free)*grid.resolution
    unknown_distance=distance_transform_edt(~unknown)*grid.resolution
    candidates=np.argwhere(free&(clearance>radius)&(unknown_distance<band))
    scored=[]
    for cell in candidates[::stride]:
        point=grid.point(cell);distance=np.linalg.norm(point-pose[:2])
        if distance<minimum_distance or any(np.linalg.norm(point-p)<minimum_distance for p in visited):continue
        scored.append((distance,point))
    for _,point in sorted(scored,key=lambda item:item[0]):
        path=grid.plan(pose[:2],point,radius)
        if path:return path
    return []


def tag_view_path(grid,pose,tag,radius,standoff,samples):
    """Reachable view around an uncertain tag position; no trust in its first normal."""
    tag=np.asarray(tag);toward=np.arctan2(pose[1]-tag[1],pose[0]-tag[0])
    angles=toward+np.arange(samples)*2*np.pi/samples
    candidates=tag+standoff*np.c_[np.cos(angles),np.sin(angles)]
    for point in sorted(candidates,key=lambda p:np.linalg.norm(p-pose[:2])):
        path=grid.plan(pose[:2],point,radius)
        if path:return path
    return []
