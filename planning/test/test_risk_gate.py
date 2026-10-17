from planning.risk_gate import RiskGate


def gate():return RiskGate(8.,1.,4.,3.)


def test_persistent_high_risk_cannot_keep_a_single_wait_beyond_eight_seconds():
    g=gate()
    assert g.step(0,True)[0]=='WAIT'
    assert g.step(7.99,True)[0]=='WAIT'
    assert g.step(8,True)[0]=='DRIVE'
    assert g.step(8.8,True)[0]=='DRIVE'
    assert g.step(9.01,True)[0]=='WAIT'
    assert g.step(17.01,True)[0]=='DRIVE'


def test_look_heading_is_fixed_and_motion_completion_needs_camera_receipt():
    g=gate();assert g.step(0,True,heading=1.)[0]=='LOOK'
    assert g.step(1,True,heading=2.)[2]==1.
    assert g.step(2,True,look_terminal=True,confirmed=False,completion=2)[0]=='LOOK'
    assert g.step(3,True,look_terminal=True,confirmed=True,completion=2)[0]=='DRIVE'


def test_failed_or_unconfirmed_look_falls_back_to_bounded_wait():
    g=gate();g.step(0,True,heading=1.)
    assert g.step(2,True,look_terminal=False)[0]=='WAIT'
    assert g.step(10,True)[0]=='DRIVE'
    g=gate();g.step(0,True,heading=1.)
    assert g.step(5,True,look_terminal=True,confirmed=False,completion=2)[0]=='WAIT'
    assert g.step(13,True)[0]=='DRIVE'
