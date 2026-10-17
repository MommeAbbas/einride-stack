"""Independent dock-steered particle dynamics and cached obstacle-aware routes."""
import heapq
import numpy as np
from scipy.ndimage import distance_transform_edt


def route_fields(grid,docks,radius):
    blocked=distance_transform_edt(~grid.occupied)*grid.resolution<=radius
    fields=[]
    for dock in docks:
        dist=np.full(blocked.shape,np.inf); target=grid.cell(dock)
        if not grid.inside(target) or blocked[target]: fields.append(dist); continue
        dist[target]=0.; queue=[(0.,target)]
        while queue:
            cost,here=heapq.heappop(queue)
            if cost>dist[here]: continue
            for dy,dx in ((1,0),(-1,0),(0,1),(0,-1),(1,1),(1,-1),(-1,1),(-1,-1)):
                nxt=(here[0]+dy,here[1]+dx)
                if not grid.inside(nxt) or blocked[nxt]: continue
                if dx and dy and (blocked[here[0]+dy,here[1]] or blocked[here[0],here[1]+dx]): continue
                value=cost+np.hypot(dy,dx)
                if value<dist[nxt]: dist[nxt]=value; heapq.heappush(queue,(value,nxt))
        fields.append(dist)
    return fields,blocked


def advance(states,dt,rng,p,grid,docks,fields,blocked,allowed,noise=True):
    """Vectorised independent dynamics; sampled swept segments checked on inflated occupancy."""
    out=states.copy()
    if dt<=0:return out
    n=len(out);scale=np.sqrt(dt)
    if noise:
        out[:,3]+=rng.normal(0,p['speed_noise']*scale,n);out[:,4]+=rng.normal(0,p['turn_noise']*scale,n)
        switched=rng.random(n)<1-np.exp(-p['dock_switch_rate']*dt);out[switched,5]=rng.choice(allowed,switched.sum())
    near=np.linalg.norm(out[:,:2]-docks[out[:,5].astype(int)],axis=1)<p['dock_radius'];out[near,3:5]=0.
    leave=near&(rng.random(n)<1-np.exp(-p['dock_leave_rate']*dt)) if noise else np.zeros(n,bool)
    for i in np.flatnonzero(leave):
        out[i,5]=rng.choice([d for d in allowed if d!=int(out[i,5])]);out[i,3]=p['departure_speed']
    cells=grid.cells(out[:,:2]);height,width=blocked.shape
    inside=(cells[:,0]>=0)&(cells[:,0]<height)&(cells[:,1]>=0)&(cells[:,1]<width)
    best=np.full(n,np.inf);targets=out[:,:2].copy();dest=out[:,5].astype(int)
    if p['dock_steering']:
        fields=np.asarray(fields)
        for dy,dx in ((1,0),(-1,0),(0,1),(0,-1),(1,1),(1,-1),(-1,1),(-1,-1)):
            nxt=cells+[dy,dx];valid=inside&(nxt[:,0]>=0)&(nxt[:,0]<height)&(nxt[:,1]>=0)&(nxt[:,1]<width)
            indices=np.flatnonzero(valid);cy,cx=nxt[indices].T
            valid[indices]&=~blocked[cy,cx]
            if dx and dy:
                valid[indices]&=~blocked[cells[indices,0]+dy,cells[indices,1]]&~blocked[cells[indices,0],cells[indices,1]+dx]
            indices=np.flatnonzero(valid);value=fields[dest[indices],nxt[indices,0],nxt[indices,1]]
            improve=value<best[indices];indices=indices[improve];best[indices]=value[improve];targets[indices]=grid.points(nxt[indices])
        delta=targets-out[:,:2];error=(np.arctan2(delta[:,1],delta[:,0])-out[:,2]+np.pi)%(2*np.pi)-np.pi
        desired=np.clip(p['steering_gain']*error,-p['max_turn_rate'],p['max_turn_rate'])
        active=(~near|leave)&np.isfinite(best);out[active,4]+=(desired[active]-out[active,4])*min(1.,p['steering_strength']*dt)
    out[:,3]=np.clip(out[:,3],0,p['max_speed']);out[:,4]=np.clip(out[:,4],-p['max_turn_rate'],p['max_turn_rate'])
    out[:,2]=(out[:,2]+out[:,4]*dt+np.pi)%(2*np.pi)-np.pi
    delta=out[:,3,None]*dt*np.c_[np.cos(out[:,2]),np.sin(out[:,2])];valid=inside.copy()
    count=max(1,int(np.ceil(np.max(np.linalg.norm(delta,axis=1))/(grid.resolution/2))))
    for fraction in np.linspace(0,1,count+1):
        c=grid.cells(out[:,:2]+delta*fraction);ok=(c[:,0]>=0)&(c[:,0]<height)&(c[:,1]>=0)&(c[:,1]<width)
        indices=np.flatnonzero(ok);ok[indices]&=~blocked[c[indices,0],c[indices,1]];valid&=ok
    out[valid,:2]+=delta[valid];out[~valid,3]=0.
    return out
