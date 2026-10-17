"""Camera/lidar likelihoods; units metres/radians. No ROS or simulator truth."""
import numpy as np


def fuse_cues(cues, gate):
    """Drop inconsistent cues; correlated-error upper bound, never inverse-variance sigma."""
    if not cues:
        return None
    values=np.asarray(cues, float)
    median=np.median(values[:,0])
    keep=np.abs(values[:,0]-median)<=gate
    if not keep.any():
        return None
    values=values[keep]
    weights=1/values[:,1]**2; weights/=weights.sum()
    return float(weights@values[:,0]),float(weights@values[:,1])


def camera_cues(box, intrinsics, shape, depth, p, blocked_bottom=False):
    x,y,w,h=box; fx,fy,cx,cy=intrinsics
    left,right,top,bottom=x-w/2,x+w/2,y-h/2,y+h/2
    full=left>0 and right<shape[1]-1 and top>0 and bottom<shape[0]-1
    cues=[]
    if p['floor_cue'] and full and not blocked_bottom and bottom>cy:
        cues.append((p['camera_height']*fy/(bottom-cy),p['floor_sigma']))
    if p['height_cue'] and full and h>0:
        cues.append((p['robot_height']*fy/h,p['height_sigma']))
    if p['depth_cue'] and depth is not None:
        a=depth[max(0,int(top)):min(shape[0],int(bottom)+1),max(0,int(left)):min(shape[1],int(right)+1)]
        a=a[np.isfinite(a)&(a>p['depth_min'])&(a<p['depth_max'])]
        if len(a)>=p['min_depth_pixels']:
            cues.append((float(np.median(a)),p['depth_sigma']))
    fused=fuse_cues(cues,p['cue_gate'])
    if fused is None:
        return None
    z,sigma=fused
    bearing=-np.arctan2(x-cx,fx)
    distance=z/np.cos(bearing)
    sigma=np.sqrt(sigma**2+p['pose_sigma_xy']**2+(distance*p['pose_sigma_yaw'])**2)
    return float(bearing),float(distance),float(sigma)


def surprise_groups(ranges, expected, increment, p, static_returns=None):
    """NaN gap has NO measured distance; wall distance is an interval endpoint only."""
    r=np.asarray(ranges); e=np.asarray(expected)
    returns=np.isfinite(r)&(r<e-p['lidar_surprise'])&(r<=p['lidar_max_range'])
    if static_returns is not None: returns&=~np.asarray(static_returns)
    gaps=np.isnan(r)&np.isfinite(e)&(e<=p['lidar_max_range'])
    mask=returns|gaps
    for i in p['self_mask_bins']:
        if 0<=i<len(mask): mask[i]=False
    groups=[]; start=None
    for i,active in enumerate(np.r_[mask,False]):
        if active and start is None: start=i
        if not active and start is not None:
            bins=np.arange(start,i); centre=int(round((start+i-1)/2))
            if len(bins)<=p['lidar_max_group_bins']:
                hit=bins[returns[bins]]
                distance=float(np.median(r[hit])) if len(hit) else None
                groups.append((centre,distance,float(np.min(e[bins]))))
            start=None
    return groups


def likelihood(particles, origin, bearing, distance, sigma, max_range, p):
    delta=particles[:,:2]-origin; radial=np.linalg.norm(delta,axis=1)
    angle=np.arctan2(delta[:,1],delta[:,0])-bearing
    side=radial*np.sin(angle)
    residual=side**2
    if distance is not None: residual+=(radial-distance)**2
    else: residual+=np.maximum(radial-max_range,0)**2
    forward=np.cos(angle)>0
    normal=np.exp(-residual/(2*sigma*sigma))*forward
    eps=p['outlier_eps'] if p['heavy_tail'] else 0.
    return (1-eps)*normal+eps*p['uniform_likelihood']
