"""Enumerated one-to-one associations, retaining ambiguous pairings as marginals."""
from itertools import product
import numpy as np


def marginal_associations(evidence, first=False, unmatched=0.):
    evidence=np.asarray(evidence,float)
    n,m=evidence.shape
    if m==0: return np.zeros((n,0))
    m=min(m,n); out=np.zeros((n,m))
    if first:
        used=set()
        for j in range(m):
            candidates=[i for i in range(n) if i not in used and evidence[i,j]>unmatched]
            if candidates:
                i=max(candidates,key=lambda i:evidence[i,j]);out[i,j]=1.;used.add(i)
        return out
    # Retain one-to-one marginals, but allow a gated-out detection to be unmatched.
    assignments=[a for a in product(range(-1,n),repeat=m) if len([i for i in a if i>=0])==len(set(i for i in a if i>=0))]
    weights=np.array([np.prod([unmatched if t<0 else evidence[t,j] for j,t in enumerate(a)]) for a in assignments])
    total=weights.sum()
    if total<=0:return out
    weights/=total
    for a,w in zip(assignments,weights):
        for j,t in enumerate(a):
            if t>=0:out[t,j]+=w
    return out
