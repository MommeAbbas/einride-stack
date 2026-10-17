import math
from pathlib import Path
import pytest
import yaml
from control.hardware_motion import hardware_command


def command(v,w,forward=.35,reverse=.15,angular=.8):
    return hardware_command(v,w,forward,reverse,angular,.15,.35,.02,.05)


def test_strict_deadband_intervals_preserve_curves_and_zeros():
    assert command(0,0)==(0,0)
    assert command(.01,.01)==(.01,.01)
    assert command(.02,.05)==(.02,.05)
    assert command(.10,.10)==(.15,.10)  # Gentle moving curve is not forced to rotate in place.
    assert command(.2,.1)==(.2,.1)
    assert command(0,.1)==(0,.35)
    assert command(0,-.1)==(0,-.35)
    assert command(.019,.2)==(.019,.35)
    assert command(.02,.2)==(.02,.2)
    assert command(0,.35)==(0,.35)
    assert command(1,2)==(.35,.8)


def test_authorised_caps_and_nonfinite_commands():
    assert command(.1,.5,forward=.1)==(0,0)
    assert command(-.05,.5)==(-.15,.5)
    assert command(-.05,.5,reverse=.10)==(0,0)  # Explicit lower caps still win.
    assert command(-.1,.05,reverse=.2)==(-.15,.05)
    assert command(0,.1,angular=.2)==(0,0)
    assert command(math.nan,0)==(0,0)


def test_hardware_overlay_uses_installed_progress_checker_and_critic():
    root=Path(__file__).parents[1]/'config'
    base=yaml.safe_load((root/'control.yaml').read_text());overlay=yaml.safe_load((root/'hardware_motion.yaml').read_text())
    assert not base['/**']['ros__parameters']['hardware_minimum_commands']
    ctrl=overlay['controller_server']['ros__parameters'];follow=ctrl['FollowPath']
    assert follow['vx_max']==.35 and follow['angular_dist_threshold']==1.05
    assert follow['VelocityDeadbandCritic']['deadband_velocities']==[.15,0.,0.]
    assert 'PredictionCritic' in follow['critics']
    assert ctrl['progress_checker']==dict(plugin='nav2_controller::PoseProgressChecker',required_movement_radius=.10,required_movement_angle=.3,movement_time_allowance=30.)
    assert overlay['/**']['ros__parameters']['reverse_speed_limit']==.25
    assert overlay['collision_monitor']['ros__parameters']['polygons']==['Approach','Stop']
    assert overlay['collision_monitor']['ros__parameters']['Approach']['time_before_collision']==.6
