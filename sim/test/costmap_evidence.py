#!/usr/bin/env python3
"""Read-only bag comparison: lethal obstacles versus expected inflation costs.

Uses OccupancyGrid's costmap translation (100 lethal, 99 inscribed, <99 inflation).
The historical >= occupied_threshold count is also reported, but must not be
mistaken for live obstacle marking after adding Nav2's InflationLayer.
"""
import argparse, json, math
from pathlib import Path
import numpy as np
from scipy.ndimage import distance_transform_edt
import yaml
from area_check import messages, seconds


def compare_arrays(live, static, threshold):
    known = static >= 0
    extra = (live == 100) & (static != 100)
    return {
        'global_lethal_cells': int(np.sum(live == 100)),
        'static_occupied_cells': int(np.sum(static == 100)),
        'extra_lethal_cells': int(np.sum(extra)),
        'extra_lethal_in_static_free': int(np.sum(extra & known)),
        'extra_lethal_in_static_unknown': int(np.sum(extra & ~known)),
        'extra_ge_threshold_cells': int(np.sum((live >= threshold) & (static < threshold))),
        'unknown_global_cells': int(np.sum(live < 0)),
    }


def aligned_static(live, static):
    # Map cell centres through both OccupancyGrid origin poses, including rotation.
    rows, cols = np.indices((live.info.height, live.info.width))
    q = live.info.origin.orientation
    angle = 2*math.atan2(q.z,q.w)
    x=(cols+.5)*live.info.resolution; y=(rows+.5)*live.info.resolution
    wx=live.info.origin.position.x+math.cos(angle)*x-math.sin(angle)*y
    wy=live.info.origin.position.y+math.sin(angle)*x+math.cos(angle)*y
    q=static.info.origin.orientation; angle=2*math.atan2(q.z,q.w)
    dx=wx-static.info.origin.position.x; dy=wy-static.info.origin.position.y
    sx=np.floor((math.cos(angle)*dx+math.sin(angle)*dy)/static.info.resolution).astype(int)
    sy=np.floor((-math.sin(angle)*dx+math.cos(angle)*dy)/static.info.resolution).astype(int)
    valid=(sx>=0)&(sy>=0)&(sx<static.info.width)&(sy<static.info.height)
    result=np.full(rows.shape,-1,dtype=np.int16)
    data=np.asarray(static.data).reshape(static.info.height,static.info.width)
    result[valid]=data[sy[valid],sx[valid]]
    return result,valid


def check(directory):
    p=Path(directory)
    planning=yaml.safe_load((p/'planning.yaml').read_text())['/**']['ros__parameters']
    threshold=planning['occupied_threshold']
    samples=[]; static=None; max_extra=None
    meta=json.loads((p/'result.json').read_text()) if (p/'result.json').exists() else None
    for topic,m,t in messages(p,['/map','/global_costmap/costmap']):
        if topic=='/map':static=m;continue
        if static is None:continue
        if meta and not meta['start_sim_s']<=t<=meta['start_sim_s']+meta['duration_sim_s']:continue
        raw=np.asarray(m.data).reshape(m.info.height,m.info.width)
        ref,valid=aligned_static(m,static)
        r=compare_arrays(raw,ref,threshold)
        r.update(sim_s=t,costmap_stamp_s=seconds(m.header.stamp),resolution_m=m.info.resolution,
                 cells_outside_static_extent=int(np.sum(~valid)))
        samples.append(r)
        if max_extra is None or r['extra_lethal_cells']>max_extra['extra_lethal_cells']:
            max_extra=dict(r)
            extra=(raw==100)&(ref!=100)
            dist=distance_transform_edt(ref!=100)*m.info.resolution
            max_extra['maximum_extra_distance_from_static_obstacle_m']=float(np.max(dist[extra])) if np.any(extra) else 0.
            max_extra['extra_more_than_one_cell_from_static_obstacle']=int(np.sum(extra & (dist>m.info.resolution*(1+np.finfo(float).eps))))
    out={'samples':len(samples),'historical_occupied_threshold':threshold,
         'definition':'Extra lethal: global value 100 where aligned static cell is not 100. Inflation is reported separately. Off-map cells counted as static unknown.',
         'extra_lethal_min':min((r['extra_lethal_cells'] for r in samples),default=None),
         'extra_lethal_max':max((r['extra_lethal_cells'] for r in samples),default=None),
         'extra_lethal_mean':float(np.mean([r['extra_lethal_cells'] for r in samples])) if samples else None,
         'final':samples[-1] if samples else None,'maximum_extra_snapshot':max_extra}
    (p/'costmap_evidence.json').write_text(json.dumps(out,indent=2)+'\n')
    (p/'costmap_samples.json').write_text(json.dumps(samples)+'\n')
    return out


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('directory');args=parser.parse_args()
    print(json.dumps(check(args.directory),indent=2))
