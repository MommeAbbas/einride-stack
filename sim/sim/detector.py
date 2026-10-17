"""Truth-to-box camera emulator: real intrinsics/extrinsics, visibility and seeded errors."""
from itertools import product
import math
import numpy as np
from scipy.spatial.transform import Rotation


def transform(position,quaternion):
    result=np.eye(4)
    result[:3,:3]=Rotation.from_quat(quaternion).as_matrix(); result[:3,3]=position
    return result


def project_box(robot_pose,camera_pose,intrinsics,image_size,size,hfov,vfov,near,far):
    corners=np.array([[x*size[0]/2,y*size[1]/2,z*size[2],1.] for x,y,z in product((-1,1),(-1,1),(0,1))])
    optical=(np.linalg.inv(camera_pose)@robot_pose@corners.T).T[:,:3]
    center=optical.mean(axis=0)
    if center[2]<=near or np.linalg.norm(center)>far or np.any(optical[:,2]<=near):
        return None
    if abs(math.atan2(center[0],center[2]))>hfov/2 or abs(math.atan2(center[1],center[2]))>vfov/2:
        return None
    pixels=(intrinsics@optical.T).T
    pixels=pixels[:,:2]/pixels[:,2,None]
    low=np.maximum(pixels.min(axis=0),[0,0]); high=np.minimum(pixels.max(axis=0),image_size)
    if np.any(high<=low):
        return None
    return np.r_[low,high]


def emulate(robots,camera,intrinsics,image_size,size,params,grid,rng,audit=None):
    boxes=[]
    for robot_id,pose in sorted(robots.items()):
        if robot_id==0:
            continue
        centre=(np.linalg.inv(camera)@pose@np.array([0.,0.,size[2]/2,1.]))[:3]
        in_fov=centre[2]>0 and abs(math.atan2(centre[0],centre[2]))<=params['detector_hfov']/2 and abs(math.atan2(centre[1],centre[2]))<=params['detector_vfov']/2
        in_range=centre[2]>params['detector_min_range'] and np.linalg.norm(centre)<=params['detector_max_range']
        clear=not params['detector_map_occlusion'] or grid.clear_line(camera[:2,3],pose[:2,3])
        record=dict(robot_id=robot_id,in_fov=bool(in_fov),in_range=bool(in_range),unoccluded=bool(clear),eligible=False,box_emitted=False,reason='projection_or_view')
        if audit is not None:audit.append(record)
        box=project_box(pose,camera,intrinsics,image_size,size,params['detector_hfov'],
            params['detector_vfov'],params['detector_min_range'],params['detector_max_range'])
        if box is None:
            continue
        if not clear:
            record['reason']='map_occluded';continue
        record['eligible']=True
        if rng.random()<params['detector_miss_rate']:
            record['reason']='configured_miss';continue
        box=box+rng.normal(0,params['detector_box_noise'],4)
        box[[0,2]]=np.clip(box[[0,2]],0,image_size[0]); box[[1,3]]=np.clip(box[[1,3]],0,image_size[1])
        if box[2]>box[0] and box[3]>box[1]:
            boxes.append(box);record['box_emitted']=True;record['reason']='box'
        else:record['reason']='noise_degenerate'
    return boxes
