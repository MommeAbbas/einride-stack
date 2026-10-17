"""Conservative path timing with turn-in-place transitions and explicit stationary horizon."""
import numpy as np


def timed_path(path,pose,speed,turn_rate,dt,horizon):
    current=np.array(pose,float); points=[]; index=1
    if len(path)>1:
        # The geometric path may be up to one replan period old. Start at the
        # closest segment to the CURRENT robot pose, as the controller does, rather
        # than turning back toward an already passed first waypoint.
        vertices=np.asarray(path,float);segments=np.diff(vertices,axis=0)
        squared=np.sum(segments**2,axis=1)
        fraction=np.clip(np.sum((current[:2]-vertices[:-1])*segments,axis=1)/np.maximum(squared,np.finfo(float).eps),0.,1.)
        projections=vertices[:-1]+fraction[:,None]*segments
        index=int(np.argmin(np.sum((projections-current[:2])**2,axis=1)))+1
    for _ in range(round(horizon/dt)+1):
        v=w=0.
        while index<len(path) and np.linalg.norm(np.asarray(path[index])-current[:2])<=speed*dt:
            index+=1
        if index<len(path):
            delta=np.asarray(path[index])-current[:2]; desired=np.arctan2(delta[1],delta[0])
            error=(desired-current[2]+np.pi)%(2*np.pi)-np.pi
            w=float(np.clip(error/dt,-turn_rate,turn_rate))
            if abs(error)<=turn_rate*dt: v=speed
        points.append([*current,v,w])
        current[2]+=w*dt; current[:2]+=v*dt*np.array([np.cos(current[2]),np.sin(current[2])])
    return np.asarray(points)


def densify(path,spacing):
    if len(path)<2:return path
    result=[path[0]]
    for a,b in zip(path,path[1:]):
        steps=max(1,int(np.ceil(np.linalg.norm(np.asarray(b)-a)/spacing)))
        result.extend(np.linspace(a,b,steps+1)[1:].tolist())
    return result
