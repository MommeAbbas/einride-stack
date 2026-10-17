"""Full timestamped group/leftover moments; temporary spatial route-mode partition."""
import numpy as np
from .motion import advance


def understanding_region(belief,target,now):
    """PROVISIONAL local support centroid, rather than a view of the ego path point.

    A stationary timed trajectory repeats ego's position. A LOOK toward that
    point has undefined bearing and can turn away from the uncertain support.
    Keep the existing path neighbourhood/mass definition, but aim at its belief.
    """
    mass=0.;weighted=np.zeros(2);labels=[]
    for i,(cloud,weights) in enumerate(zip(belief.clouds,belief.weights)):
        support=belief.last_support[i]
        if support is not None and now-support<=belief.p['understanding_age']:continue
        positions=belief.grid.to_map(cloud[:,:2])
        near=np.linalg.norm(positions-target,axis=1)<belief.p['understanding_radius']
        value=float(weights[near].sum())
        if value>0:
            mass+=value;weighted+=weights[near]@positions[near];labels.append(i)
    return (weighted/mass,min(1.,mass),labels) if mass>0 else None


def moments(points,weights,epsilon):
    total=weights.sum()
    w=weights/total if total>0 else np.full(len(weights),1/len(weights))
    mean=w@points; delta=points-mean
    return mean,(delta.T*w)@delta+np.eye(2)*epsilon


def predictions(belief,lag,now=None):
    p=belief.p; outputs=[]
    # Separate RNG: producing output must not alter the filter's posterior or stochastic evolution.
    rng=np.random.default_rng(p['prediction_seed'])
    for i,(cloud,weights) in enumerate(zip(belief.clouds,belief.weights)):
        cloud=advance(cloud,lag,rng,p,belief.grid,belief.docks,belief.fields,belief.blocked,belief.allowed)
        rolls=[cloud]
        for _ in range(round(p['horizon']/p['prediction_dt'])):
            rolls.append(advance(rolls[-1],p['prediction_dt'],rng,p,belief.grid,belief.docks,belief.fields,belief.blocked,belief.allowed))
        rolls=np.stack(rolls)
        keys=np.c_[cloud[:,5].astype(int),np.floor(cloud[:,:2]/p['mode_cell_size']).astype(int)]
        unique,inverse=np.unique(keys,axis=0,return_inverse=True)
        groups=[]; remaining=np.ones(len(cloud),bool)
        ordered=sorted(range(len(unique)),key=lambda k:-weights[inverse==k].sum())
        for k in ordered:
            mask=inverse==k; mass=min(1.,float(weights[mask].sum()))
            if len(groups)>=p['max_groups'] or mass<p['minimum_group_mass']: continue
            stats=[moments(r[mask,:2],weights[mask],p['covariance_epsilon']) for r in rolls]
            # Do not publish a named route whose mean crosses static geometry.
            if any(not belief.grid.clear_line(a[0],b[0],belief.blocked) for a,b in zip(stats,stats[1:])): continue
            groups.append((mass,int(unique[k,0]),f'cell_{unique[k,1]}_{unique[k,2]}',stats)); remaining[mask]=False
        mask=remaining if remaining.any() else np.ones(len(cloud),bool)
        leftover=(min(1.,float(weights[remaining].sum())),-1,'',[moments(r[mask,:2],weights[mask],p['covariance_epsilon']) for r in rolls])
        age=belief.coast_age(i,now) if now is not None else 0.
        for _,_,_,stats in groups+[leftover]:
            for j,(mean,cov) in enumerate(stats):
                stats[j]=(mean,cov+np.eye(2)*p['coasting_variance_rate']*(age+j*p['prediction_dt']))
        outputs.append((groups,leftover))
    return outputs
