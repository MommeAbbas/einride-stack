"""Lidar-only escape geometry; metres, radians and seconds, no ROS or truth."""
import math
import numpy as np


def pivot(side,wheel_speed,track):
    # Left obstacle: left track reverses, right stationary (and conversely).
    return -wheel_speed/2, math.copysign(wheel_speed/track,side)


def clearance(points,half_length,half_width):
    p=np.asarray(points).reshape(-1,2)
    return np.hypot(np.maximum(abs(p[:,0])-half_length,0),np.maximum(abs(p[:,1])-half_width,0))


def front_clearance(points,half_length,half_width,half_angle):
    p=np.asarray(points).reshape(-1,2)
    front=np.abs(np.arctan2(p[:,1],p[:,0]))<=half_angle
    return float(np.min(clearance(p[front],half_length,half_width))) if front.any() else None


def swept_clear(points,v,w,horizon,step,half_length,half_width,margin):
    """Reject endpoint hits anywhere in the swept rectangle, including now.

    Inflate by half a sampling interval's maximum boundary travel, covering the
    gaps between sampled poses. No-return bins are not manufactured free points.
    """
    if not all(math.isfinite(x) for x in (v,w,horizon,step,half_length,half_width,margin)) or min(horizon,step,half_length,half_width)<=0 or margin<0:return False
    p=np.asarray(points).reshape(-1,2)
    if not len(p) or not np.isfinite(p).all():return False
    times=np.linspace(0,horizon,max(1,math.ceil(horizon/step))+1)
    pad=margin+(abs(v)+abs(w)*math.hypot(half_length,half_width))*(times[1]-times[0])/2
    a=w*times;c=np.cos(a);s=np.sin(a)
    x=v*times if abs(w)<1e-9 else v/w*s
    y=np.zeros_like(times) if abs(w)<1e-9 else v/w*(1-c)
    dx=p[:,0,None]-x;dy=p[:,1,None]-y
    return not np.any((np.abs(dx*c+dy*s)<=half_length+pad)&(np.abs(-dx*s+dy*c)<=half_width+pad))
