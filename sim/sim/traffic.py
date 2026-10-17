"""Differential-drive traffic path follower. Ignore never reacts to another robot."""
import math
from .geometry import wrap


def command(pose,path,speed,max_turn,gain,turn_angle,waypoint_tolerance,goal_tolerance):
    if not path:
        return 0.,0.,False
    if math.dist(pose[:2],path[-1]) <= goal_tolerance:
        return 0.,0.,True
    while len(path)>1 and math.dist(pose[:2],path[0])<=waypoint_tolerance:
        path.pop(0)
    target=path[0]
    error=wrap(math.atan2(target[1]-pose[1],target[0]-pose[0])-pose[2])
    omega=max(-max_turn,min(max_turn,gain*error))
    v=0. if abs(error)>turn_angle else min(speed,gain*math.dist(pose[:2],target))*max(0.,math.cos(error))
    return v,omega,False


def mode_command(mode,velocity,pose,ego_pose,stop_distance):
    """Only the explicit stop_near ablation reacts to ego; ignore never does."""
    if mode=='stop_near' and ego_pose is not None and math.dist(pose[:2],ego_pose[:2])<stop_distance:
        return 0.,0.
    return velocity


def yield_command(velocity,pose,others,distance,half_angle):
    """Evaluation-only truth-based traffic mode; never used by production nodes."""
    for other in others:
        dx,dy=other[0]-pose[0],other[1]-pose[1]
        angle=wrap(math.atan2(dy,dx)-pose[2])
        if math.hypot(dx,dy)<distance and abs(angle)<half_angle:
            return 0.,0.
    return velocity
