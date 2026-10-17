"""Camera visibility over exposed occupied cells; Nav2 supplies path reachability."""
import math
import numpy as np
from scipy.ndimage import convolve,distance_transform_edt


class WallCoverage:
    def __init__(self,message,p,previous=None):
        self.p=p;self.raw=np.array(message.data).reshape(message.info.height,message.info.width)
        self.resolution=message.info.resolution;self.origin=np.array([message.info.origin.position.x,message.info.origin.position.y])
        q=message.info.origin.orientation;angle=2*math.atan2(q.z,q.w)
        self.rotation=np.array([[math.cos(angle),-math.sin(angle)],[math.sin(angle),math.cos(angle)]])
        free=(self.raw>=0)&(self.raw<p['coverage_occupied_threshold'])
        self.clearance=distance_transform_edt(free)*self.resolution
        self.obstacle_clearance=distance_transform_edt(self.raw<p['coverage_occupied_threshold'])*self.resolution
        # Only exposed occupied cells can be observed. Interior solid-wall cells
        # have no visible surface and must not make completion impossible.
        y,x=np.mgrid[-1:2,-1:2]
        nx=convolve(free.astype(float),-x.astype(float),mode='constant',cval=0.)
        ny=convolve(free.astype(float),-y.astype(float),mode='constant',cval=0.)
        boundary=(self.raw>=p['coverage_occupied_threshold'])&(np.hypot(nx,ny)>0)
        self.cells=np.argwhere(boundary);self.points=self.world(self.cells)
        normals=np.c_[nx[boundary],ny[boundary]]
        self.normals=(normals/np.linalg.norm(normals,axis=1)[:,None])@self.rotation.T
        self.seen=np.zeros(len(self.cells),dtype=bool)
        if previous is not None and len(previous.points):
            old_cells=previous.cell(self.points);valid=previous.inside(old_cells)
            old_mask=np.zeros(previous.raw.shape,dtype=bool);old_mask[tuple(previous.cells[previous.seen].T)]=True
            self.seen[valid]=old_mask[tuple(old_cells[valid].T)]

    def world(self,cells):return (np.asarray(cells)[...,::-1]+.5)*self.resolution@self.rotation.T+self.origin
    def cell(self,points):return np.floor(((np.asarray(points)-self.origin)@self.rotation)/self.resolution).astype(int)[...,::-1]
    def inside(self,cells):return np.all((cells>=0)&(cells<self.raw.shape),axis=-1)

    def clear(self,point,clearance):
        c=self.cell(point)
        return bool(self.inside(c) and self.clearance[tuple(c)]>=clearance)

    def frontier_clear(self,point,clearance):
        # Frontier goals border unknown cells; only known obstacles determine
        # clearance. explore_lite centroids may themselves lie in unknown cells;
        # Nav2 and live sensors still validate the route. Camera views use clear().
        c=self.cell(point)
        return bool(self.inside(c) and self.raw[tuple(c)]<self.p['coverage_occupied_threshold'] and self.obstacle_clearance[tuple(c)]>=clearance)

    def ray_clear(self,origin,target):
        distance=np.linalg.norm(target-origin)
        count=max(2,int(math.ceil(distance/(self.resolution*self.p['coverage_ray_step_fraction']))))
        # Ignore endpoint occupied cell; every earlier cell must be known free.
        cells=self.cell(origin+(target-origin)*np.linspace(0,1,count)[:,None]);end=self.cell(target)
        cells=cells[np.any(cells!=end,axis=1)]
        if not self.inside(cells).all():return False
        values=self.raw[tuple(cells.T)]
        return bool(np.all((values>=0)&(values<self.p['coverage_occupied_threshold'])))

    def visible(self,origin,heading,frame=None,unseen_only=False):
        origin=np.asarray(origin);delta=self.points-origin[:2];distance=np.linalg.norm(delta,axis=1)
        direction=delta/np.maximum(distance[:,None],1e-9)
        incidence=np.sum(-direction*self.normals,axis=1)
        bearing=np.arctan2(delta[:,1],delta[:,0])-heading;bearing=np.arctan2(np.sin(bearing),np.cos(bearing))
        mask=(distance<=self.p['coverage_detection_range'])&(distance>0)&(incidence>=math.cos(math.radians(self.p['coverage_max_incidence_deg'])))&(np.abs(bearing)<=math.radians(self.p['coverage_hfov_deg'])/2)
        if unseen_only:mask &= ~self.seen
        if frame is not None:
            xyz=np.c_[self.points,np.full(len(self.points),self.p['coverage_wall_height'])]
            camera=(xyz-np.array(frame['origin']))@np.array(frame['rotation'])
            z=np.maximum(camera[:,2],1e-9);u=frame['fx']*camera[:,0]/z+frame['cx'];v=frame['fy']*camera[:,1]/z+frame['cy']
            mask &= (camera[:,2]>0)&(u>=0)&(u<frame['width'])&(v>=0)&(v<frame['height'])
        ids=np.flatnonzero(mask)
        if not len(ids):return ids
        targets=self.points[ids];steps=max(2,int(math.ceil(float(distance[ids].max())/(self.resolution*self.p['coverage_ray_step_fraction']))))
        cells=self.cell(origin[:2]+(targets-origin[:2])[:,None,:]*np.linspace(0,1,steps)[None,:,None])
        endpoint=np.all(cells==self.cell(targets)[:,None,:],axis=2)
        inside=self.inside(cells);safe=np.clip(cells,[0,0],np.array(self.raw.shape)-1)
        values=self.raw[safe[:,:,0],safe[:,:,1]]
        clear=np.all(endpoint | (inside & (values>=0)&(values<self.p['coverage_occupied_threshold'])),axis=1)
        return ids[clear]

    def observe(self,frame):
        rotation=np.array(frame['rotation']);heading=math.atan2(rotation[1,2],rotation[0,2])
        ids=self.visible(frame['origin'],heading,frame)
        self.seen[ids]=True
        return len(ids)

    def dock_visible(self,point,heading,dock):
        p=dock.tag_pose.pose.position;target=np.array([p.x,p.y]);delta=target-np.asarray(point)
        bearing=math.atan2(delta[1],delta[0])-heading
        q=dock.tag_pose.pose.orientation;a=2*math.atan2(q.z,q.w)
        incidence=float((-delta/max(np.linalg.norm(delta),1e-9))@np.array([math.cos(a),math.sin(a)]))
        return (np.linalg.norm(delta)<=self.p['coverage_detection_range'] and
                abs(math.atan2(math.sin(bearing),math.cos(bearing)))<=math.radians(self.p['coverage_hfov_deg'])/2 and
                incidence>=math.cos(math.radians(self.p['coverage_max_incidence_deg'])) and self.ray_clear(point,target))

    def candidates(self,pose,live,cooldown,now,docks=(),qualified=()):
        step=max(1,round(self.p['coverage_candidate_spacing']/self.resolution))
        cells=np.argwhere(self.clearance>=self.p['coverage_min_clearance']);cells=cells[(cells[:,0]%step==0)&(cells[:,1]%step==0)]
        if len(cells)>self.p['coverage_max_candidates']:
            cells=cells[np.linspace(0,len(cells)-1,self.p['coverage_max_candidates']).astype(int)]
        points=list(self.world(cells));special={}
        for dock in docks:
            p=dock.tag_pose.pose;xy=np.array([p.position.x,p.position.y]);q=p.orientation;heading=2*math.atan2(q.z,q.w)
            point=xy+self.p['coverage_tag_standoff']*np.array([math.cos(heading),math.sin(heading)])
            if self.clear(point,self.p['coverage_min_clearance']):
                special[len(points)]=(math.atan2(xy[1]-point[1],xy[0]-point[0]),dock.dock_id);points.append(point)
        # Qualified dock neighbourhoods no longer contribute finish-condition gain.
        useful=~self.seen.copy()
        for dock in qualified:
            p=dock.tag_pose.pose.position
            useful &= np.linalg.norm(self.points-np.array([p.x,p.y]),axis=1)>self.p['coverage_qualified_radius']
        results=[]
        for j,point in enumerate(points):
            if not live.clear(point,self.p['coverage_min_clearance']):continue
            if any(np.linalg.norm(point-np.array(xy))<self.p['coverage_candidate_spacing']/2 and now<t for xy,t in cooldown):continue
            headings=[special[j][0]] if j in special else np.arange(self.p['coverage_heading_samples'])*2*np.pi/self.p['coverage_heading_samples']
            best=None
            for heading in headings:
                visible=self.visible(point,heading)
                gain=float(useful[visible].sum())
                targets=[dock.dock_id for dock in docks if self.dock_visible(point,heading,dock)]
                if j in special and special[j][1] not in targets:targets.append(special[j][1])
                gain+=len(targets)*self.p['coverage_unqualified_weight']
                if gain<=0:continue
                score=gain-self.p['coverage_travel_weight']*np.linalg.norm(point-pose[:2])
                if best is None or score>best[0]:best=(score,float(point[0]),float(point[1]),float(heading),gain,tuple(sorted(targets)))
            if best is not None:results.append(best)
        return sorted(results,reverse=True)[:self.p['coverage_reachable_checks']]


def validation_hold(p):
    frames=p['coverage_required_sightings']
    return max(frames/p['coverage_camera_rate'],frames/p['coverage_validator_rate'],(frames-1)*p['coverage_sighting_interval']+1/p['coverage_validator_rate'])
