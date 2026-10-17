"""Raw depth projection for collision guard; unrelated to detector/belief outputs."""
import numpy as np


def project_depth(depth,k,stride,minimum,maximum):
    yy,xx=np.mgrid[0:depth.shape[0]:stride,0:depth.shape[1]:stride];z=depth[::stride,::stride]
    valid=np.isfinite(z)&(z>=minimum)&(z<=maximum)
    return np.c_[(xx[valid]-k[2])*z[valid]/k[0],(yy[valid]-k[5])*z[valid]/k[4],z[valid]].astype('<f4')
