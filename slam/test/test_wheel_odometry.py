import math
from pathlib import Path
import pytest
import yaml
from slam.wheel_odometry import WheelOdometry, integrate_arc


def parameters():
    return yaml.safe_load((Path(__file__).parents[1]/'config/wheel_odometry.yaml').read_text())['/**']['ros__parameters']


def calibrated():
    model = WheelOdometry(parameters())
    for i in range(45):
        t = i / 20
        model.add_imu(t, .03)
        estimate = model.step(t, 1200., -700.)  # Arbitrary accumulated starting driver offsets.
    assert model.bias == pytest.approx(.03)
    assert estimate.yaw_source == 'gyro'
    assert estimate.x == pytest.approx(0)
    assert estimate.yaw == pytest.approx(0, abs=1e-12)
    return model


def test_exact_arc_straight_reverse_and_quarter_turn():
    assert integrate_arc(0, 0, 0, 1, 0) == (1, 0, 0)
    assert integrate_arc(0, 0, 0, -1, 0) == (-1, 0, 0)
    assert integrate_arc(0, 0, 0, math.pi/2, math.pi/2) == pytest.approx((1, 1, math.pi/2))
    assert integrate_arc(0, 0, 0, 1, 1e-12) == pytest.approx((1, 5e-13, 1e-12))


def test_translation_uses_encoder_delta_not_absolute_position():
    model = calibrated()
    model.add_imu(2.25, .03)
    estimate = model.step(2.25, 1225., -675.)
    assert estimate.x == pytest.approx(.01)
    assert estimate.linear == pytest.approx(.04)
    assert estimate.angular == pytest.approx(0, abs=1e-12)


def test_gyro_yaw_overrides_differential_wheel_slip():
    model = calibrated()
    model.add_imu(2.25, .53)
    estimate = model.step(2.25, 1175., -675.)
    assert estimate.linear == 0
    assert estimate.angular == pytest.approx(.5)
    assert estimate.yaw == pytest.approx(.025)
    assert estimate.yaw_source == 'gyro'


def test_stale_imu_fallback_and_recovery_without_double_slip_multiplier():
    model = calibrated()
    estimate = model.step(2.5, 1100., -600.)
    assert estimate.yaw_source == 'encoder_fallback'
    assert estimate.angular == pytest.approx(.08 / (model.p['effective_track'] * .3))
    before = estimate.yaw
    model.add_imu(2.55, .03)
    estimate = model.step(2.55, 1100., -600.)
    assert estimate.yaw_source == 'gyro'
    assert estimate.yaw == pytest.approx(before)


def test_stationary_bias_restarts_after_in_place_wheel_motion():
    model = WheelOdometry(parameters())
    for i in range(25):
        t = i/20; model.add_imu(t, .03); assert model.step(t, 0, 0) is None
    model.add_imu(1.25, .03); assert model.step(1.25, -25., 25.) is None
    for i in range(26, 64):
        t = i/20; model.add_imu(t, .03); assert model.step(t, -25., 25.) is None
    assert model.bias is None  # Total startup >2 s, but not continuous stillness.
    for i in range(64, 70):
        t=i/20; model.add_imu(t,.03); model.step(t,-25.,25.)
    assert model.bias == pytest.approx(.03)


def test_bias_resets_if_gyro_reports_rotation_with_stationary_wheels():
    model = WheelOdometry(parameters())
    for i in range(60):
        t=i/20; model.add_imu(t, .5); assert model.step(t, 0, 0) is None
    assert model.bias is None


def test_missing_imu_falls_back_only_after_timeout():
    model = WheelOdometry(parameters())
    assert model.step(0, 0, 0) is None
    assert model.step(.1, 0, 0) is None
    estimate = model.step(.3, 50., 50.)
    assert estimate.yaw_source == 'encoder_fallback'
    assert estimate.x == pytest.approx(.02)


def test_bad_samples_and_encoder_reset_do_not_teleport():
    model = calibrated()
    assert not model.add_imu(2.0, 100)
    assert not model.add_imu(2.25, math.nan)
    assert model.step(2.15, 1000, 1000) is None
    assert model.step(2.25, math.nan, 0) is None
    assert model.step(2.25, 0, 0) is None  # Counter reset is outside speed bound.
    assert model.step(3., 0, 0) is None  # Long gap rebases without a jump.
    model.add_imu(3.05, .03)
    estimate=model.step(3.05, 25., 25.)
    assert estimate.x == pytest.approx(.01)


def test_future_imu_does_not_contaminate_encoder_capture():
    model = calibrated()
    model.add_imu(2.3, 100.)
    estimate=model.step(2.25,1200.,-700.)
    assert estimate.yaw == pytest.approx(0,abs=1e-12)


@pytest.mark.parametrize('left_counts,right_counts,expected', [(79,83,.81),(23,21,.22)])
def test_measured_chassis_counts_convert_to_metres(left_counts,right_counts,expected):
    # Each raw integer count is 1 cm; driver divides that count by radius .04.
    model = calibrated()
    for i in range(1,101):
        t=2.2+i/20
        model.add_imu(t,.03)
        left=1200.+round(i*left_counts/100)/.04
        right=-700.+round(i*right_counts/100)/.04
        estimate=model.step(t,left,right)
    assert estimate.x == pytest.approx(expected)
    # .81 m / .22 m are encoder predictions; physical tape measurements were .77 / ~.2.


def test_quantised_velocity_window_does_not_smear_integrated_pose():
    model=calibrated()
    for i in range(1,21):
        t=2.2+i/20
        # One centimetre every five messages: raw velocity alternates .2 and 0.
        position=25.*(i//5)
        model.add_imu(t,.03)
        estimate=model.step(t,1200.+position,-700.+position)
        if i>=5:
            assert estimate.linear == pytest.approx(.04)
        assert estimate.x == pytest.approx(.01*(i//5))
    # No residual pose creep while the velocity window empties after stopping.
    for i in range(21,27):
        t=2.2+i/20;model.add_imu(t,.03)
        estimate=model.step(t,1300.,-600.)
        assert estimate.x == pytest.approx(.04)
    assert estimate.linear == pytest.approx(0,abs=1e-12)


def test_irregular_samples_use_exact_trailing_time_window():
    model=calibrated()
    # Uniform reverse motion sampled at irregular intervals, with large arbitrary offsets.
    for t in (2.27,2.35,2.43,2.51,2.64):
        displacement=-(t-2.2)*.1
        position=displacement/(.04/100)
        model.add_imu(t,.03)
        estimate=model.step(t,1200.+position,-700.+position)
    assert estimate.linear == pytest.approx(-.1)
    assert estimate.x == pytest.approx(-.044)


def test_stationary_bias_refresh_removes_measured_drift_without_pose_reset():
    model=calibrated()
    # Move once to reset stationary detection and establish a nonzero pose.
    model.add_imu(2.25,.03);model.step(2.25,1225.,-675.)
    drift=math.radians(.1)
    old_x=model.x
    for i in range(46,66):
        t=i/20;model.add_imu(t,.03+drift)
        estimate=model.step(t,1225.,-675.)
        if t<2.75:assert model.bias==pytest.approx(.03)
    assert model.bias==pytest.approx(.03+drift)
    assert estimate.angular==pytest.approx(0,abs=1e-12)
    assert model.x==pytest.approx(old_x)
    assert model.yaw>0 # Past small integration is retained, not teleported to zero.


def test_bias_refresh_does_not_learn_turning_motion_or_stale_imu():
    model=calibrated()
    for i in range(45,70):
        t=i/20;model.add_imu(t,.53)
        model.step(t,1200.,-700.) # Encoder quantisation/failure must not hide clear gyro rotation.
    assert model.bias==pytest.approx(.03)
    for i in range(70,90):model.step(i/20,1200.,-700.)
    assert model.bias==pytest.approx(.03)


def test_any_encoder_tick_restarts_stationary_refresh_window():
    model=calibrated()
    for i in range(45,85):
        t=i/20;model.add_imu(t,.035)
        ticks=(i-44)//5
        model.step(t,1200.+25*ticks,-700.+25*ticks)
    assert model.bias==pytest.approx(.03)
