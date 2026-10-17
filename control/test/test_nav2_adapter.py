"""Safety regressions for the leased topic to installed-Nav2 action boundary."""
from types import SimpleNamespace, MethodType
from copy import deepcopy
from pathlib import Path
import yaml
from rclpy.task import Future
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from builtin_interfaces.msg import Time, Duration
from interfaces.msg import CoordinatorRequest, CoordinatorStatus, TrajectoryPoint, Predictions, PerceptionHealth
from control.nav2_coordinator import Nav2Coordinator


class Client:
    def __init__(self): self.calls = []
    def server_is_ready(self): return True
    def send_goal_async(self, goal, **kw):
        f = Future(); self.calls.append((goal, f)); return f


class Handle:
    accepted = True
    def __init__(self): self.result = Future(); self.cancelled = False
    def get_result_async(self): return self.result
    def cancel_goal_async(self): self.cancelled = True; return Future()


def probe():
    params = yaml.safe_load((Path(__file__).parents[1]/'config/control.yaml').read_text())['/**']['ros__parameters']
    params['require_predictions'] = False
    tr = TransformStamped(); tr.header.stamp.sec = 10; tr.transform.rotation.w = 1.
    odom = Odometry(); odom.header.stamp.sec = 10
    sent = []; statuses = []
    p = SimpleNamespace(mapping_active=False,request=None, revision=1, odom=odom, predictions=None, health=None,
        goal_handle=None, pending=None, generation=0, cancel_pending=[], raw=None, raw_time=-1.,
        terminal=False, result=None, completion=None, action_result=None, progress=0.,
        remaining=0., initial_distance=None, bt_state='', navigate=Client(), spin=Client(),
        now=lambda:10., p=params.__getitem__, get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(to_msg=lambda:Time(sec=10))),
        buffer=SimpleNamespace(lookup_transform=lambda *a:tr), pub=SimpleNamespace(publish=sent.append),
        status_pub=SimpleNamespace(publish=statuses.append), limit_pub=SimpleNamespace(publish=lambda m:None))
    for name in ('on_request','cancel','send','tick','command','current_command','predictions_valid'):
        setattr(p,name,MethodType(getattr(Nav2Coordinator,name),p))
    return p, params, sent, statuses


def drive():
    r = CoordinatorRequest(request_id='one', goal_id='dock', map_revision=1, mode=1, speed_limit=.3)
    r.header.frame_id='map'; r.valid_until.sec=12; r.trajectory.map_revision=1
    r.trajectory.goal_id='dock'; r.trajectory.points=[TrajectoryPoint(x=1., y=2., yaw=.4)]
    return r


def test_wait_cancels_late_goal_and_gates_previous_velocity_before_resuming():
    p, _, sent, _ = probe(); p.on_request(drive()); p.tick()
    assert len(p.navigate.calls)==1
    old = p.navigate.calls[0][1]
    wait=CoordinatorRequest(request_id='wait', mode=0); wait.valid_until.sec=12
    p.on_request(wait)
    handle=Handle(); old.set_result(handle)
    assert handle.cancelled
    cmd=Twist(); cmd.linear.x=.3; p.command(cmd); p.tick()
    assert sent[-1].linear.x==0. and p.raw is None
    p.on_request(drive()); p.tick()
    assert len(p.navigate.calls)==1  # Cancellation not acknowledged yet.
    handle.result.set_result(SimpleNamespace(status=5)); p.tick()
    assert len(p.navigate.calls)==2
    pose=p.navigate.calls[-1][0].pose
    assert pose.header.frame_id=='map' and pose.pose.position.x==1. and pose.pose.position.y==2.


def test_backup_keeps_existing_speed_cap_and_expired_lease_stops():
    p, _, sent, _ = probe(); p.on_request(drive()); p.tick()
    h=Handle(); p.navigate.calls[0][1].set_result(h)
    cmd=Twist(); cmd.linear.x=-.15; p.command(cmd); p.tick()
    assert sent[-1].linear.x==-.1
    p.request.valid_until.sec=9; p.tick()
    assert h.cancelled and sent[-1].linear.x==0.


def test_prediction_validity_uses_approved_stamp_plus_horizon_contract():
    p, params, _, _=probe(); params['require_predictions']=True
    p.predictions=Predictions(map_revision=1,horizon=Duration(sec=3)); p.predictions.header.stamp.sec=10
    p.health=PerceptionHealth(map_revision=1,status=PerceptionHealth.NOMINAL); p.health.header.stamp.sec=10
    assert p.predictions_valid()
    p.predictions.horizon.sec=2
    assert not p.predictions_valid()


def test_repeated_id_cannot_change_target_or_speed_and_lease_only_extends():
    p, _, _, _=probe(); r=drive(); p.on_request(r)
    heartbeat=deepcopy(r); heartbeat.valid_until.sec=11; p.on_request(heartbeat)
    assert p.request.valid_until.sec==12
    mutation=deepcopy(r); mutation.trajectory.points[0].x=3.; p.on_request(mutation)
    assert p.terminal and p.result[0]==CoordinatorStatus.REJECTED


def test_actual_node_initializes_and_holds_without_inputs():
    import rclpy
    rclpy.init()
    node = Nav2Coordinator()
    try:
        node.tick()
        assert node.get_clock().now().nanoseconds > 0  # Unlaunched nodes default to hardware clock.
        assert node.request is None and node.goal_handle is None
        assert node.count_publishers('/cmd_vel') == 0
    finally:
        node.destroy_node(); rclpy.shutdown()


def test_full_mapping_passthrough_expires_and_handoff_requires_mission_lease():
    p, params, sent, _ = probe()
    p.mapping_active = True
    params.update(minimum_linear_command=.15, minimum_angular_command=.35)
    cmd = Twist(); cmd.linear.x=.10; cmd.angular.z=.10
    p.command(cmd); p.tick()
    assert sent[-1].linear.x == .15 and sent[-1].angular.z == .10
    assert not p.navigate.calls  # Mapping owns goals until its completion receipt.
    p.raw_time = 0.; p.tick()
    assert sent[-1].linear.x == 0.
    p.mapping_active = False; p.command(cmd); p.tick()
    assert sent[-1].linear.x == 0.  # Old mapping commands cannot move after handoff.
    p.on_request(drive()); p.tick()
    assert len(p.navigate.calls) == 1


def test_active_commands_forward_on_arrival_without_status_tick():
    p, _, sent, _=probe();p.on_request(drive());p.tick()
    p.navigate.calls[0][1].set_result(Handle())
    for speed in (.2,.25,.3):
        command=Twist();command.linear.x=speed;p.command(command)
        assert sent[-1].linear.x==speed
    p.request.valid_until.sec=9;p.command(command)
    assert sent[-1].linear.x==0.


def test_arrival_forwarding_stops_with_stale_odometry():
    p, _, sent, _=probe();p.on_request(drive());p.tick()
    p.navigate.calls[0][1].set_result(Handle());p.odom.header.stamp.sec=1
    command=Twist();command.linear.x=.3;p.command(command)
    assert sent[-1].linear.x==0.
