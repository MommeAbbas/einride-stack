"""Timestamped pose interpolation for sensor capture and future prediction scoring."""
from collections import deque
import numpy as np
from scipy.spatial.transform import Rotation, Slerp


class PoseHistory:
    def __init__(self,seconds):
        self.seconds=seconds; self.samples=deque()

    def add(self,stamp,poses):
        if self.samples and stamp<=self.samples[-1][0]:
            if stamp==self.samples[-1][0]:
                return
            self.samples.clear()
        self.samples.append((stamp,poses))
        while self.samples and stamp-self.samples[0][0]>self.seconds:
            self.samples.popleft()

    def at(self,stamp,tolerance):
        if not self.samples or stamp<self.samples[0][0] or stamp>self.samples[-1][0]:
            return None
        for (ta,a),(tb,b) in zip(self.samples,list(self.samples)[1:]):
            if ta<=stamp<=tb:
                if tb-ta>tolerance:
                    return None
                weight=(stamp-ta)/(tb-ta)
                poses={}
                for key in a.keys() & b.keys():
                    result=np.eye(4); result[:3,3]=(1-weight)*a[key][:3,3]+weight*b[key][:3,3]
                    rotation=Rotation.from_matrix(np.stack([a[key][:3,:3],b[key][:3,:3]]))
                    result[:3,:3]=Slerp([ta,tb],rotation)(stamp).as_matrix()
                    poses[key]=result
                return poses
        return None
