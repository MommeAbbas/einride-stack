"""PROVISIONAL joint trajectory risk: deterministic Gaussian samples with shared temporal noise.
Temporal dependence is a stand-in requiring calibration review; it is not calibrated by construction.
"""
import numpy as np
from scipy.special import ndtri
from scipy.stats import qmc


def interpolate(points,times):
    source=np.array([p[0] for p in points]); values=np.array([p[1:] for p in points])
    if len(source)<2 or not np.isfinite(source).all() or np.any(np.diff(source)<=0) or times[0]<source[0] or times[-1]>source[-1]:
        return None
    return np.stack([np.interp(times,source,values[:,i]) for i in range(values.shape[1])],axis=1)


def trajectory_risk(own,times,robots,radius,margin,samples,epsilon,pose_sigma):
    """Every group including leftover, matched at absolute times. Invalid input fails closed."""
    if not np.isfinite(own).all() or not np.isfinite(times).all(): return 1.
    draws=ndtri(np.clip(qmc.Sobol(2,scramble=False).random_base2(samples),epsilon,1-epsilon))
    risks=[]
    for groups in robots:
        risk=0.; mass=0.
        for probability,points in groups:
            if not np.isfinite(probability) or probability<0 or probability>1+epsilon: return 1.
            mass+=probability
            # A normalised floating-point sum can be 1+one ULP. Apply the
            # existing numerical mass tolerance, not a phantom certain collision.
            probability=min(1.,probability)
            if probability<=0: continue
            values=interpolate(points,times)
            if values is None or not np.isfinite(values).all() or not np.isfinite(probability): return 1.
            covariance=values[:,2:].reshape(-1,2,2)
            if np.any(np.abs(covariance[:,0,1]-covariance[:,1,0])>epsilon): return 1.
            eig,vectors=np.linalg.eigh(covariance)
            if np.any(eig < -epsilon): return 1.
            # Perception already inflates measurements by localisation uncertainty.
            # Own extra sigma is separately configurable, zero avoids double counting default.
            root=vectors*np.sqrt(np.maximum(eig,epsilon)+pose_sigma**2)[:,None,:]
            positions=values[:,:2,None]+np.einsum('tij,nj->tin',root,draws)
            distance=np.linalg.norm(positions-own[:,:,None],axis=1)
            probability_hit=float(np.mean(np.any(distance<2*radius+margin,axis=0)))
            risk+=probability*probability_hit
        if abs(mass-1)>epsilon: return 1.
        risks.append(min(1.,risk))
    # Independent filters are the AGREED model; union bound across robot identities is conservative.
    return min(1.,sum(risks))


def confirmed_look(completion,frame,prediction,processed,applied,revision_matches):
    return revision_matches and processed and applied and completion<frame<prediction
