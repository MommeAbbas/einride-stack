"""TEMPORARY smooth visibility detour; requires information-gain and feasibility review.
The candidate keeps the straight segment's endpoint tangents. It is accepted only
when every sampled edge clears the inflated map, curvature fits the camera turn-rate
limit at the requested speed, and the moving camera exposes a previously hidden view.
"""
import numpy as np
from scipy.ndimage import distance_transform_edt


def observation_candidate(grid,path,target,radius,offset,hfov,spacing,speed,turn_limit,pose):
    if len(path)<2:return None
    target=np.asarray(target);pose=np.asarray(pose)
    direction=np.arctan2(target[1]-pose[1],target[0]-pose[0])
    error=(direction-pose[2]+np.pi)%(2*np.pi)-np.pi
    if abs(error)<hfov/2 and grid.clear_line(pose[:2],target):return None
    blocked=distance_transform_edt(~grid.occupied)*grid.resolution<=radius
    best=None
    for i in range(1,len(path)):
        a,b=np.asarray(path[i-1]),np.asarray(path[i]);delta=b-a;length=np.linalg.norm(delta)
        if length<=spacing:continue
        normal=np.array([-delta[1],delta[0]])/length
        t=np.linspace(0,1,max(3,int(np.ceil(length/spacing))+1))
        for side in (-1,1):
            # Smooth zero-slope lateral excursion. Numeric constants define the curve,
            # while displacement, sampling, speed and turn constraints are ROS parameters.
            curve=a+t[:,None]*delta+(np.sin(np.pi*t)**2)[:,None]*normal*offset*side
            edges=np.diff(curve,axis=0);headings=np.unwrap(np.arctan2(edges[:,1],edges[:,0]))
            turns=np.abs(np.diff(headings))*speed/np.maximum(np.linalg.norm(edges[1:],axis=1),np.finfo(float).eps)
            if turns.size and np.max(turns)>turn_limit:continue
            if any(not grid.clear_line(x,y,blocked) for x,y in zip(curve,curve[1:])):continue
            bearings=np.arctan2(target[1]-curve[:-1,1],target[0]-curve[:-1,0])
            errors=(bearings-headings+np.pi)%(2*np.pi)-np.pi
            if not any(abs(e)<hfov/2 and grid.clear_line(x,target) for x,e in zip(curve[:-1],errors)):continue
            candidate=path[:i]+curve[1:-1].tolist()+path[i:]
            cost=sum(np.linalg.norm(np.asarray(y)-x) for x,y in zip(candidate,candidate[1:]))
            if best is None or cost<best[0]:best=cost,candidate
    return best[1] if best else None
