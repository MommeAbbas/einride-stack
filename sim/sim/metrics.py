"""Offline-compatible probabilistic and timing metrics; no ROS dependencies."""
import math
import numpy as np


def interpolate_points(points,stamp):
    """points=(absolute time, XY mean, 2x2 covariance), no extrapolation."""
    for a,b in zip(points,points[1:]):
        if a[0]<=stamp<=b[0] and b[0]>a[0]:
            t=(stamp-a[0])/(b[0]-a[0])
            return (1-t)*a[1]+t*b[1],(1-t)*a[2]+t*b[2]
    return None


def mixture_moments(components,probability_tolerance):
    """Moment-matched ellipse diagnostic; not a claim that a multimodal belief is Gaussian."""
    if not components or not math.isclose(sum(w for w,_,_ in components),1.,rel_tol=0.,abs_tol=probability_tolerance):
        return None
    if any(w<0 or not np.isfinite(m).all() or not np.isfinite(c).all() or
           not np.allclose(c,c.T) or np.linalg.eigvalsh(c).min()<0 for w,m,c in components):
        return None
    mean=sum(w*m for w,m,_ in components)
    covariance=sum(w*(c+np.outer(m-mean,m-mean)) for w,m,c in components)
    return mean,covariance


def ellipse_hits(truth,mean,covariance,epsilon):
    error=np.asarray(truth)-mean
    mahal=float(error@np.linalg.solve(covariance+np.eye(2)*epsilon,error))
    # In TWO dimensions radius-1/radius-2 Gaussian ellipses have these expected masses.
    return int(mahal<=1),int(mahal<=4),1-math.exp(-.5),1-math.exp(-2.)


def look_confirmed(completion,capture,processed,applied,prediction_stamp):
    """Confirmation requires applied evidence; eligible-but-not-applied is insufficient."""
    return bool(processed and applied and capture>completion and prediction_stamp>capture)
