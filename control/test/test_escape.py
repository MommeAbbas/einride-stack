import math
import numpy as np
import pytest
from control.escape import pivot,clearance,front_clearance,swept_clear


def test_single_track_pivot_inverse_kinematics():
    for side in (1,-1):
        v,w=pivot(side,.25,.38);left=v-w*.38/2;right=v+w*.38/2
        assert (left,right)==pytest.approx((-.25,0) if side==1 else (0,-.25))


def test_sweep_checks_corners_and_reverse_not_only_endpoint():
    points=np.array([[.15,-.06],[2,0],[-2,0]])
    assert not swept_clear(points,0,.8,.5,.02,.12596,.115,.01)
    assert swept_clear(points,-.25,0,.5,.02,.12596,.115,.01)
    assert not swept_clear([[ -.19,0]],-.25,0,.5,.02,.12596,.115,.01)
    assert not swept_clear([[0,0]],-.25,0,.5,.02,.12596,.115,.01)
    assert not swept_clear([],0,0,.5,.02,.12596,.115,.01)


def test_clearance_is_outside_rectangle_and_empty_front_unknown():
    assert clearance([[.27596,0]],.12596,.115)==pytest.approx([.15])
    assert front_clearance([[-1,0]],.12596,.115,math.pi/4) is None


def test_stop_admission_geometry_matches_unchanged_native_hard_stop():
    import ast,yaml
    from pathlib import Path
    root=Path(__file__).resolve().parents[1]/'config'
    stop=yaml.safe_load((root/'hardware_motion.yaml').read_text())['collision_monitor']['ros__parameters']['Stop']
    settings=yaml.safe_load((root/'escape.yaml').read_text())['/**']['ros__parameters']
    points=np.asarray(ast.literal_eval(stop['points']))
    assert np.max(abs(points[:,0]))==settings['escape_stop_half_length']
    assert np.max(abs(points[:,1]))==settings['escape_stop_half_width']
