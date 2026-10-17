import numpy as np
from costmap_evidence import compare_arrays, aligned_static
from nav_msgs.msg import OccupancyGrid


def test_inflation_is_not_phantom_lethal_marking():
    static=np.array([[100,0,0],[0,0,-1]])
    live=np.array([[100,99,70],[0,100,100]])
    r=compare_arrays(live,static,50)
    assert r['extra_lethal_cells']==2
    assert r['extra_lethal_in_static_free']==1
    assert r['extra_lethal_in_static_unknown']==1
    assert r['extra_ge_threshold_cells']==4


def test_static_comparison_aligns_cell_centres_and_extent():
    s=OccupancyGrid();s.info.width=3;s.info.height=1;s.info.resolution=1.;s.info.origin.orientation.w=1.
    s.data=[100,0,-1]
    g=OccupancyGrid();g.info.width=3;g.info.height=1;g.info.resolution=1.;g.info.origin.orientation.w=1.
    g.info.origin.position.x=1.
    ref,valid=aligned_static(g,s)
    assert ref.tolist()==[[0,-1,-1]]
    assert valid.tolist()==[[True,True,False]]
