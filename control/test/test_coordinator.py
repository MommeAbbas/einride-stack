from control.coordinator import validate,turn_command


def test_requests_fail_closed():
    assert validate(9,'map',2,1,'goal',1,1,.3)
    assert validate(1,'map',1,1,'goal',1,1,.3)
    assert validate(1,'odom',2,1,'goal',1,1,.3)
    assert validate(1,'map',2,1,'goal',1,2,.3)
    assert not validate(1,'map',2,1,'goal',1,1,.3)


def test_look_confirms_stopping_before_turn_and_completion():
    p=dict(look_timeout_limit=10.,heading_tolerance=.05,look_turn_rate=.2,look_gain=1.)
    assert turn_command(1,False,False,1,p)==(0.,'STOPPING')
    assert turn_command(1,True,False,1,p)==(.2,'ACTIVE')
    assert turn_command(.01,False,True,2,p)==(0.,'STOPPING')
    assert turn_command(.01,True,True,2,p)==(0.,'SUCCEEDED')
    assert turn_command(1,True,True,11,p)==(0.,'FAILED')


def test_raw_depth_projection_rejects_invalid_and_preserves_close_returns():
    import numpy as np
    from control.depth import project_depth
    k=[100.,0.,1.,0.,100.,1.,0.,0.,1.]
    p=project_depth(np.array([[.1,np.nan],[np.inf,2.]]),k,1,.05,8.)
    assert len(p)==2 and np.isfinite(p).all() and np.isclose(p[0,2],.1)
