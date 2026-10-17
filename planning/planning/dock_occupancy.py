"""Sensor-only Saga dock admission. Nav2 retains every path/motion decision.

Kept separate from ROS callbacks so occupancy geometry and clear-time hysteresis
can be tested without simulator truth or a running navigation stack.
"""
import math
import numpy as np
from scipy.ndimage import distance_transform_edt
from .ros_support import seconds


def cells(message):
    return np.asarray(message.data).reshape(message.info.height,message.info.width)


def world_cells(message):
    yy,xx=np.indices((message.info.height,message.info.width))
    q=message.info.origin.orientation;a=2*math.atan2(q.z,q.w)
    x=(xx+.5)*message.info.resolution;y=(yy+.5)*message.info.resolution
    return np.stack((message.info.origin.position.x+math.cos(a)*x-math.sin(a)*y,
                     message.info.origin.position.y+math.sin(a)*x+math.cos(a)*y),axis=-1)


def sample(message,points):
    points=np.asarray(points);q=message.info.origin.orientation;a=2*math.atan2(q.z,q.w)
    d=points-np.array([message.info.origin.position.x,message.info.origin.position.y])
    x=np.floor((math.cos(a)*d[...,0]+math.sin(a)*d[...,1])/message.info.resolution).astype(int)
    y=np.floor((-math.sin(a)*d[...,0]+math.cos(a)*d[...,1])/message.info.resolution).astype(int)
    inside=(x>=0)&(y>=0)&(x<message.info.width)&(y<message.info.height)
    values=np.full(x.shape,-1,dtype=int);values[inside]=cells(message)[y[inside],x[inside]]
    return values


def dynamic_points(live,static,static_margin=0.):
    """Only lethal marks in static FREE space; inflation is not another robot."""
    points=world_cells(live)[cells(live)==100]
    if not len(points):return points
    points=points[sample(static,points)==0]
    if static_margin<=0 or not len(points):return points
    # A one-cell scan/map boundary mismatch is static structure, not a truck.
    distance=distance_transform_edt(cells(static)<50)*static.info.resolution
    q=static.info.origin.orientation;a=2*math.atan2(q.z,q.w)
    d=points-[static.info.origin.position.x,static.info.origin.position.y]
    x=np.floor((math.cos(a)*d[:,0]+math.sin(a)*d[:,1])/static.info.resolution).astype(int)
    y=np.floor((-math.sin(a)*d[:,0]+math.cos(a)*d[:,1])/static.info.resolution).astype(int)
    return points[distance[y,x]>static_margin]


def occupied(points,dock,radius):
    return bool(len(points) and np.any(np.linalg.norm(np.asarray(points)-np.asarray(dock),axis=1)<=radius))


def costmap_cues(points,dock,radius,sources,now,max_age,resolution):
    """Attribute lethal dock cells to fresh measured endpoints for the audit log.

    Combined Nav2 costmap does not publish per-cell provenance. Unmatched retained
    marks are explicitly unattributed; this never changes the occupied decision.
    """
    near=np.asarray(points)[np.linalg.norm(np.asarray(points)-dock,axis=1)<=radius]
    if not len(near):return []
    labels=[]
    for name,(capture,endpoints) in sources.items():
        if not 0<=now-capture<=max_age or not len(endpoints):continue
        # Two quantisations can differ by a cell diagonal; geometry, not tuning.
        if any(np.any(np.linalg.norm(endpoints-p,axis=1)<=resolution*math.sqrt(2)) for p in near):labels.append('costmap_'+name)
    return labels or ['costmap_retained_unattributed']


def confirmed_track_points(predictions,max_age,receipts=None,now=None,max_sigma=None):
    """Camera-specific receipt gates current full-mixture position/covariance."""
    points=[]
    if predictions is None or receipts is None or now is None:return points
    captures={d.id:seconds(d.header.stamp) for d in receipts.detections}
    for robot in predictions.robots:
        capture=captures.get(str(robot.track_id))
        if not robot.ever_confirmed or capture is None or not 0<=now-capture<=max_age:continue
        groups=[g for g in [*robot.groups,robot.leftover] if g.probability>0 and g.points]
        total=sum(g.probability for g in groups)
        if not total:continue
        weights=np.array([g.probability/total for g in groups])
        means=np.array([[g.points[0].x,g.points[0].y] for g in groups]);mean=weights@means
        covariance=sum(w*(np.asarray(g.points[0].covariance).reshape(2,2)+np.outer(x-mean,x-mean)) for w,g,x in zip(weights,groups,means))
        if max_sigma is None or not np.isfinite(covariance).all() or np.linalg.eigvalsh(covariance).max()>=max_sigma**2:continue
        points.append(mean)
    return points


def standoff(approach,heading,back,side,pose,live,radius):
    """Choose either side of the exit corridor, checking the full ego envelope."""
    forward=np.array([math.cos(heading),math.sin(heading)])
    lateral=np.array([-forward[1],forward[0]])
    centre=np.asarray(approach)-back*forward
    candidates=[centre+side*lateral,centre-side*lateral]
    grid_points=world_cells(live);raw=cells(live)
    valid=[]
    for point in candidates:
        # An unknown/off-map centre or occupied envelope is never a waiting site.
        bounds=point+np.array([[-radius,-radius],[-radius,radius],[radius,-radius],[radius,radius]])
        if np.any(sample(live,bounds)<0):continue
        mask=np.linalg.norm(grid_points-point,axis=2)<=radius+live.info.resolution/math.sqrt(2)
        if np.any(mask) and not np.any((raw[mask]<0)|(raw[mask]>=99)):
            valid.append(point)
    return min(valid,key=lambda p:np.linalg.norm(p-np.asarray(pose)[:2])) if valid else None


class DockAdmission:
    def __init__(self,clear_time):self.clear_time=clear_time;self.reset()
    def reset(self):self.blocked=False;self.clear_since=None;self.target=None
    def update(self,now,is_occupied,valid=True):
        if not valid:
            self.clear_since=None
            return False
        if is_occupied:
            self.blocked=True;self.clear_since=None
        elif self.blocked:
            if self.clear_since is None:self.clear_since=now
            if now-self.clear_since>=self.clear_time:self.reset()
        return not self.blocked
