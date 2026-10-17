"""Exercise generated ROS CDR types and the agreed content, not just text schemas."""

import math
import pytest
from builtin_interfaces.msg import Duration, Time
from rclpy.serialization import serialize_message, deserialize_message
from interfaces import msg


def roundtrip(message):
    result = deserialize_message(serialize_message(message), type(message))
    assert result == message
    return result


@pytest.mark.parametrize('name', [
    'RobotTruth', 'RobotTruthArray', 'CollisionEvent', 'SimRouteEvent',
    'PredictionPoint', 'PredictedTrajectory', 'RobotPrediction', 'Predictions',
    'CameraObservation', 'PerceptionHealth', 'DockState', 'DockStates',
    'DockLandmark', 'DockMap', 'TrajectoryPoint', 'PlannedTrajectory', 'SagaState',
    'CoordinatorRequest', 'CoordinatorStatus', 'UnderstandingRegion', 'Understanding',
])
def test_generated_types_roundtrip(name):
    roundtrip(getattr(msg, name)())


def test_modes_same_dock_and_leftover_keep_spatial_support():
    predictions = msg.Predictions()
    predictions.header.frame_id = 'map'
    predictions.header.stamp = Time(sec=10)
    predictions.time_step = Duration(sec=1)
    predictions.horizon = Duration(sec=4)
    predictions.map_revision = 7
    robots = []
    for track_id in range(3):
        trajectories = []
        for dock, mode, probability, y in [(2, 'left', .5, 1.), (2, 'right', .3, -1.),
                                          (-1, '', .2, 0.)]:
            points = [msg.PredictionPoint(stamp=Time(sec=10 + t), x=float(t), y=y,
                                         covariance=[.25, .05, .05, .36]) for t in range(5)]
            trajectories.append(msg.PredictedTrajectory(probability=probability,
                                destination_dock=dock, route_mode=mode, points=points))
        robots.append(msg.RobotPrediction(track_id=track_id, ever_confirmed=False,
                      groups=trajectories[:2], leftover=trajectories[2]))
    predictions.robots = robots
    received = roundtrip(predictions)
    assert len(received.robots) == 3
    for robot in received.robots:
        assert math.isclose(sum(g.probability for g in robot.groups) + robot.leftover.probability, 1.)
        assert robot.groups[0].destination_dock == robot.groups[1].destination_dock
        assert robot.groups[0].route_mode != robot.groups[1].route_mode
        assert robot.leftover.points[-1].stamp.sec == 14
        assert robot.groups[0].points[0].stamp == received.header.stamp
        assert not robot.ever_confirmed


def test_unknown_and_null_never_mean_clear_or_dock_a():
    assert msg.PerceptionHealth().status == msg.PerceptionHealth.FAILED
    assert not msg.DockState().valid
    assert not msg.DockState().physically_free
    assert not msg.DockMap().ready
    assert not msg.CameraObservation().processed
    assert not msg.SagaState().available
    assert msg.SagaState().next_dock == -1
    assert msg.CoordinatorRequest().mode == msg.CoordinatorRequest.WAIT
    assert msg.CoordinatorStatus().status == msg.CoordinatorStatus.UNKNOWN
    assert not msg.CoordinatorStatus().stopped
    assert not msg.CoordinatorStatus().final_heading_valid


def test_request_identity_timed_path_and_result_are_preserved():
    request = msg.CoordinatorRequest(request_id='session/request-1', goal_id='route-12/load',
                                    mode=msg.CoordinatorRequest.DRIVE, map_revision=7,
                                    speed_limit=.2, valid_until=Time(sec=15))
    request.header.frame_id = 'map'
    request.header.stamp = Time(sec=10)
    request.trajectory = msg.PlannedTrajectory(goal_id=request.goal_id, path_version=2,
        map_revision=7, points=[msg.TrajectoryPoint(stamp=Time(sec=10), x=1., yaw=.1),
                               msg.TrajectoryPoint(stamp=Time(sec=11), x=1.2, yaw=.1)])
    request.trajectory.header = request.header
    assert roundtrip(request).trajectory.path_version == 2
    request.mode = msg.CoordinatorRequest.LOOK
    request.target_heading = -1.2
    request.look_timeout = Duration(sec=5)
    assert roundtrip(request).target_heading == -1.2
    status = msg.CoordinatorStatus(request_id=request.request_id, goal_id=request.goal_id,
        mode=request.mode, status=msg.CoordinatorStatus.SUCCEEDED, terminal=True, stopped=True,
        completion_stamp=Time(sec=12), final_heading=-1.2, final_heading_valid=True)
    assert roundtrip(status).completion_stamp.sec == 12


def test_empty_processed_view_is_distinct_from_no_camera_data():
    health = msg.PerceptionHealth(status=msg.PerceptionHealth.NOMINAL, has_camera_observation=True)
    health.last_camera_observation = msg.CameraObservation(processed=True,
        negative_evidence_usable=True, detection_count=0, map_revision=7)
    health.last_camera_observation.header.stamp = Time(sec=12, nanosec=1)
    health.last_camera_observation.header.frame_id = 'map'
    assert roundtrip(health).last_camera_observation.header.stamp.nanosec == 1


def test_look_receipt_records_application_and_later_prediction():
    receipt = msg.CameraObservation(processed=True, negative_evidence_usable=True,
                                    negative_evidence_applied=True,
                                    predictions_after_frame=Time(sec=13))
    receipt.header.stamp = Time(sec=12, nanosec=1)
    restored = roundtrip(msg.PerceptionHealth(has_camera_observation=True,
                                              last_camera_observation=receipt))
    assert restored.last_camera_observation.negative_evidence_applied
    assert restored.last_camera_observation.predictions_after_frame.sec == 13
