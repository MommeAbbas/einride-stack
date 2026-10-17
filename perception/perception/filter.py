"""Three independent particle clouds; robust updates and free-space injection."""
import numpy as np
from .association import marginal_associations
from .measurements import likelihood
from .motion import advance,route_fields
from .visibility import visible


def systematic(weights,rng):
    positions=(rng.random()+np.arange(len(weights)))/len(weights)
    return np.searchsorted(np.cumsum(weights),positions,side='left').clip(max=len(weights)-1)


class Belief:
    def __init__(self,grid,docks,ego,p):
        self.grid=grid; self.docks=np.asarray(docks); self.p=p; self.rng=np.random.default_rng(p['seed'])
        self.fields,self.blocked=route_fields(grid,self.docks,p['robot_radius'])
        self.free=np.array([grid.point(c) for c in zip(*np.where(~self.blocked))])
        self.allowed=list(range(8)); self.ego=np.asarray(ego); self.last_support=[None]*3; self.view=None
        cloud=self.fresh(p['particles_per_robot'],initial=True)
        self.clouds=[cloud.copy() for _ in range(3)]
        self.weights=[np.full(len(cloud),1/len(cloud)) for _ in range(3)]
        self.view=None;self.last_reset=[None]*3;self.deleted_tracks=0
        self.last_camera_support=[None]*3

    def fresh(self,n,initial=False):
        free=self.free[np.linalg.norm(self.free-self.ego,axis=1)>self.p['init_exclusion_radius']]
        if self.view is not None:
            origin,heading,occluders=self.view
            free=free[~visible(free,origin,heading,self.grid,self.p,occluders)]
        if len(free)==0: raise ValueError('No valid injection space outside current view')
        out=np.zeros((n,6)); out[:,:2]=free[self.rng.integers(len(free),size=n)]
        out[:,2]=self.rng.uniform(-np.pi,np.pi,n); out[:,5]=self.rng.choice(self.allowed,n)
        if not initial: out[:,3]=self.rng.uniform(0,self.p['max_speed'],n)
        return out

    def reservations(self,held):
        self.allowed=[d for d in range(8) if d not in held]
        if not self.allowed: raise ValueError('No destination candidates')
        for cloud in self.clouds:
            invalid=~np.isin(cloud[:,5],self.allowed)
            cloud[invalid,5]=self.rng.choice(self.allowed,invalid.sum())

    def predict(self,dt):
        self.clouds=[advance(c,dt,self.rng,self.p,self.grid,self.docks,self.fields,self.blocked,self.allowed) for c in self.clouds]

    def repair_map_support(self,ego):
        """PROVISIONAL: replace only support made impossible by a map/TF correction.

        Valid particles remain in odom, unchanged. An invalid particle otherwise
        stays trapped forever because swept-motion checking rejects its start cell.
        Redistribute its existing mass over admissible free space, using the same
        camera and ego exclusions as fresh particles; this is NOT a detection.
        """
        self.ego=np.asarray(ego)
        for i,cloud in enumerate(self.clouds):
            cells=self.grid.cells(cloud[:,:2]);height,width=self.blocked.shape
            valid=(cells[:,0]>=0)&(cells[:,0]<height)&(cells[:,1]>=0)&(cells[:,1]<width)
            ids=np.flatnonzero(valid)
            valid[ids]&=~self.blocked[cells[ids,0],cells[ids,1]]
            if not valid.all():
                cloud[~valid]=self.fresh(int((~valid).sum()))
                # Relocated support has no measurement confirmation.
                self.last_support[i]=None;self.last_camera_support[i]=None

    def expire(self,stamp):
        for i in range(3):
            if self.last_reset[i] is None:self.last_reset[i]=stamp
            last=self.last_support[i] if self.last_support[i] is not None else self.last_reset[i]
            if stamp-last>self.p['unsupported_track_timeout']:
                self.clouds[i]=self.fresh(len(self.clouds[i]),initial=True)
                self.weights[i][:]=1/len(self.weights[i]);self.last_support[i]=None;self.last_camera_support[i]=None
                self.last_reset[i]=stamp;self.deleted_tracks+=1

    def coast_age(self,i,stamp):
        last=self.last_support[i] if self.last_support[i] is not None else self.last_reset[i]
        return max(0.,stamp-last) if last is not None else 0.

    def camera_births(self,measurements,stamp):
        """Initialise unconfirmed slots where the camera actually measured a robot.

        A lidar-only confirmation may still be diffuse. The first camera range
        must initialise it too. Uniform particles can miss a narrow likelihood; reweighting alone
        cannot create missing support. Existing confirmed slots retain gating.
        """
        if not self.p.get('camera_measurement_birth',False):return
        available=[i for i in range(3) if self.last_camera_support[i] is None]
        used=set()
        for origin,bearing,distance,sigma,end in measurements[:3]:
            if distance is None or not available:continue
            point=np.asarray(origin)+distance*np.array([np.cos(bearing),np.sin(bearing)])
            supported=False
            for i,(cloud,weights) in enumerate(zip(self.clouds,self.weights)):
                if self.last_camera_support[i] is None and i not in used:continue
                mean=weights@cloud[:,:2];delta=cloud[:,:2]-mean
                cov=(delta.T*weights)@delta+np.eye(2)*(sigma*sigma+self.p['covariance_epsilon'])
                residual=point-mean
                if residual@np.linalg.solve(cov,residual)<=self.p['association_gate_squared']:supported=True;break
            if supported:continue
            i=available.pop(0);cloud=self.clouds[i]
            # Draw a broad measurement proposal; the existing likelihood update
            # narrows it. Reject samples outside the existing full-body map support.
            proposal=point+self.rng.normal(0,sigma*self.p['camera_birth_sigma_scale'],size=(len(cloud)*self.p['camera_birth_draw_multiplier'],2))
            cells=self.grid.cells(proposal);height,width=self.blocked.shape
            valid=(cells[:,0]>=0)&(cells[:,0]<height)&(cells[:,1]>=0)&(cells[:,1]<width)
            ids=np.flatnonzero(valid);valid[ids]&=~self.blocked[cells[ids,0],cells[ids,1]]
            valid&=np.linalg.norm(proposal-self.ego,axis=1)>self.p['robot_radius']*2
            proposal=proposal[valid]
            if not len(proposal):continue
            cloud[:,:2]=proposal[self.rng.integers(len(proposal),size=len(cloud))]
            self.weights[i][:]=1/len(cloud);used.add(i)

    def update(self,measurements,stamp,view=None,negative=False,negative_masks=None,detection_probability=None):
        # measurement=(origin, absolute bearing, range or None, sigma, interval end)
        self.expire(stamp)
        if view is not None:self.camera_births(measurements,stamp)
        m=min(3,len(measurements)); likelihoods=[]
        for cloud in self.clouds:
            likelihoods.append([likelihood(cloud,*item,self.p) for item in measurements[:m]])
        evidence=np.array([[self.weights[i]@v for v in row] for i,row in enumerate(likelihoods)]).reshape(3,m)
        for i,cloud in enumerate(self.clouds):
            if self.last_support[i] is None:continue
            weights=self.weights[i];mean=weights@cloud[:,:2];delta=cloud[:,:2]-mean
            cov=(delta.T*weights)@delta+np.eye(2)*(self.p['covariance_epsilon']+self.p['coasting_variance_rate']*self.coast_age(i,stamp))
            for j,(origin,bearing,distance,sigma,end) in enumerate(measurements[:m]):
                if distance is None:continue # Bearing interval is not a fabricated point.
                measured=np.asarray(origin)+distance*np.array([np.cos(bearing),np.sin(bearing)])
                residual=measured-mean
                if residual@np.linalg.solve(cov+np.eye(2)*sigma*sigma,residual)>self.p['association_gate_squared']:evidence[i,j]=0.
        association=marginal_associations(evidence,first=all(t is None for t in self.last_support),unmatched=self.p['association_unmatched_evidence'])
        applied=False
        # Lidar updates must not erase the camera exclusion used during injection.
        if view is not None:self.view=view if negative else None
        for i,(cloud,weights) in enumerate(zip(self.clouds,self.weights)):
            assigned=float(association[i].sum()) if m else 0.
            update=np.full(len(cloud),1-assigned)
            for j in range(m): update+=association[i,j]*likelihoods[i][j]/max(evidence[i,j],np.finfo(float).tiny)
            ranged=sum(association[i,j] for j in range(m) if measurements[j][2] is not None)
            if ranged>self.p['support_probability']:
                self.last_support[i]=stamp
                if view is not None:self.last_camera_support[i]=stamp
            if negative and (view is not None or negative_masks is not None):
                mask=negative_masks[i] if negative_masks is not None else visible(cloud[:,:2],view[0],view[1],self.grid,self.p,view[2])
                probability=self.p['p_detect'] if detection_probability is None else detection_probability
                update*=1-(1-assigned)*probability*mask
                applied=True # Actual measurement operation, even when this cloud has no visible support.
            weights*=update; total=weights.sum()
            if not np.isfinite(total) or total<=0: weights[:]=1/len(weights)
            else: weights/=total
            if 1/(weights@weights)<self.p['resample_threshold']*len(weights):
                self.clouds[i]=cloud[systematic(weights,self.rng)].copy()
                count=int(self.p['inject_fraction']*len(weights)) if self.p['injection'] else 0
                if count: self.clouds[i][:count]=self.fresh(count)
                weights[:]=1/len(weights)
        return applied
