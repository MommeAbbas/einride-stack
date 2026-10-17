"""Conservative static-map and observed-box occlusion; no unseen-robot absence claim."""
import numpy as np


def visible(points, origin, heading, grid, p, occluders=()):
    delta=points-np.asarray(origin); distance=np.linalg.norm(delta,axis=1)
    angle=(np.arctan2(delta[:,1],delta[:,0])-heading+np.pi)%(2*np.pi)-np.pi
    mask=(np.abs(angle)<p['camera_hfov']/2-p['fov_edge_margin'])&(distance<p['detect_max_range'])
    for bearing,width,near in occluders:
        diff=(angle-bearing+np.pi)%(2*np.pi)-np.pi
        mask&=~((np.abs(diff)<=width/2)&(distance>near))
    count=max(1,int(np.ceil(p['detect_max_range']/grid.resolution)))
    for fraction in np.linspace(0,1,count+1):
        indices=np.flatnonzero(mask)
        if not len(indices):break
        cells=grid.cells(np.asarray(origin)+delta[indices]*fraction)
        ok=(cells[:,0]>=0)&(cells[:,0]<grid.occupied.shape[0])&(cells[:,1]>=0)&(cells[:,1]<grid.occupied.shape[1])
        valid=np.flatnonzero(ok);ok[valid]&=~grid.occupied[cells[valid,0],cells[valid,1]];mask[indices]&=ok
    return mask


def ray_ranges(grid,origin,angles,max_range):
    ranges=np.full(len(angles),max_range);active=np.ones(len(angles),bool)
    directions=np.c_[np.cos(angles),np.sin(angles)]
    for distance in np.arange(grid.resolution,max_range,grid.resolution):
        ids=np.flatnonzero(active)
        if not len(ids):break
        cells=grid.cells(np.asarray(origin)+distance*directions[ids])
        inside=(cells[:,0]>=0)&(cells[:,0]<grid.occupied.shape[0])&(cells[:,1]>=0)&(cells[:,1]<grid.occupied.shape[1])
        blocked=~inside;valid=np.flatnonzero(inside);blocked[valid]|=grid.occupied[cells[valid,0],cells[valid,1]]
        ranges[ids[blocked]]=distance;active[ids[blocked]]=False
    return ranges


def static_return_mask(grid,origin,angles,ranges,clearance,gate):
    """Associate finite endpoints with mapped surfaces despite ray-edge aliasing.

    NaN gaps remain gap evidence. This does not modify the scan or the raw guard.
    """
    ranges=np.asarray(ranges);mask=np.zeros(len(ranges),bool)
    ids=np.flatnonzero(np.isfinite(ranges))
    endpoints=np.asarray(origin)+ranges[ids,None]*np.c_[np.cos(angles[ids]),np.sin(angles[ids])]
    cells=grid.cells(endpoints)
    inside=(cells[:,0]>=0)&(cells[:,0]<clearance.shape[0])&(cells[:,1]>=0)&(cells[:,1]<clearance.shape[1])
    mask[ids[inside]]=clearance[cells[inside,0],cells[inside,1]]<=gate
    return mask


def lidar_clear_mask(points,origin,heading,ranges,angle_min,increment,grid,p):
    """Optional stopped-sweep evidence: finite unmasked rays only, before their return.
    A missing return never establishes a clear sector. Static occlusion still applies.
    """
    delta=points-np.asarray(origin);distance=np.linalg.norm(delta,axis=1)
    angle=(np.arctan2(delta[:,1],delta[:,0])-heading-angle_min)%(2*np.pi)
    bins=np.rint(angle/increment).astype(int)%len(ranges);r=np.asarray(ranges)[bins]
    reliable=np.isfinite(r)&(distance+p['robot_radius']<r)
    reliable&=~np.isin(bins,p['self_mask_bins'])
    visibility=p.copy();visibility.update(camera_hfov=2*np.pi,fov_edge_margin=0.,detect_max_range=p['lidar_max_range'])
    return reliable&visible(points,origin,heading,grid,visibility)
