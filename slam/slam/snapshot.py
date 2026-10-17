"""Pure snapshot validation and dock-estimation helpers; no ego localisation algorithms."""
import json
import math
from pathlib import Path
import numpy as np


def load_snapshot(path):
    path=Path(path).resolve();data=json.loads(path.read_text())
    if data.get('format_version')!=1:raise ValueError('Unsupported snapshot format; remap E0')
    target=Path(data['map_yaml'])
    if not target.is_absolute():target=path.parent/target
    if not target.is_file():raise ValueError('Saved Nav2 map YAML missing')
    data['map_yaml']=str(target)
    ids=set(data['landmarks'])
    if not ids<={str(i) for i in range(8)}:raise ValueError('Invalid dock identity')
    if ids!={str(i) for i in range(8)} and not data.get('partial',False):raise ValueError('Snapshot requires eight docks or explicit partial flag')
    if len(data['final_pose'])!=3 or len(data['final_pose_covariance'])!=36:raise ValueError('Final pose/covariance missing')
    if not np.isfinite(data['final_pose']+data['final_pose_covariance']).all():raise ValueError('Nonfinite final pose/covariance')
    return data


def sanitise_ranges(ranges,minimum,maximum):
    a=np.asarray(ranges,dtype=float);valid=np.isfinite(a)&(a>=minimum)&(a<=maximum)
    # Zero lies below the real lidar minimum, so neither backend treats it as a wall/free ray.
    return np.where(valid,a,0.).tolist(),int((~valid).sum())


def fuse_landmark(old,xyz,heading,t,close,p,covariance=None,yaw_variance=None):
    observed=np.asarray(xyz[:2]);cov=np.eye(2)*p['tag_sigma']**2 if covariance is None else np.asarray(covariance)
    yaw_var=p.get('tag_yaw_sigma',.12)**2 if yaw_variance is None else yaw_variance
    if old is None:return dict(xy=observed,heading=heading,z=float(xyz[2]),count=int(close),stamp=t,cov=cov,yaw_var=yaw_var)
    if t-old['stamp']<p['tag_observation_interval'] or not close:return None
    inconsistent=np.linalg.norm(observed-old['xy'])>p['tag_outlier_gate']
    if not landmark_valid(old,p) and (old['count']==0 or inconsistent):
        return dict(xy=observed,heading=heading,z=float(xyz[2]),count=1,stamp=t,cov=cov,yaw_var=yaw_var)
    if inconsistent:return None
    w=p['covariance_intersection_weight'];ia=np.linalg.inv(old['cov']);ib=np.linalg.inv(cov)
    fused=np.linalg.inv(w*ia+(1-w)*ib)
    angle=math.atan2(w*math.sin(old['heading'])+(1-w)*math.sin(heading),w*math.cos(old['heading'])+(1-w)*math.cos(heading))
    return dict(xy=fused@(w*ia@old['xy']+(1-w)*ib@observed),cov=fused,heading=angle,z=float(xyz[2]),count=old['count']+1,stamp=t,yaw_var=1/(w/old.get('yaw_var',yaw_var)+(1-w)/yaw_var))


def landmark_valid(value,p):
    if value['count']<p['tag_min_observations']:return False
    if not p.get('tag_geometry_covariance',False):return True
    return (float(np.linalg.eigvalsh(value['cov'])[-1])<=p['dock_max_position_variance'] and
            value.get('yaw_var',p['tag_yaw_sigma']**2)<=p['dock_max_yaw_variance'])


def tag_covariance(distance,incidence,p):
    scale=max(1.,distance/p['tag_reference_range'])/max(p['tag_min_cosine'],math.cos(incidence))
    return np.eye(2)*(p['tag_sigma']*scale)**2,(p['tag_yaw_sigma']*scale)**2
