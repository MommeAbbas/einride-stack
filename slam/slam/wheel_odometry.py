"""Wheel translation + bias-corrected gyro yaw; hardware adapter, no ROS imports."""
from collections import deque
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class Estimate:
    stamp: float
    x: float
    y: float
    yaw: float
    linear: float
    angular: float
    yaw_source: str
    dt: float


def integrate_arc(x, y, yaw, distance, angle):
    """Exact constant-twist SE(2) step, stable at zero yaw and for reverse motion."""
    half = angle / 2
    scale = math.sin(half) / half if half else 1.0
    x += distance * scale * math.cos(yaw + half)
    y += distance * scale * math.sin(yaw + half)
    return x, y, math.atan2(math.sin(yaw + angle), math.cos(yaw + angle))


class WheelOdometry:
    def __init__(self, parameters):
        self.p = parameters
        for key in ('wheel_radius', 'encoder_distance_scale', 'velocity_window', 'effective_track', 'bias_duration', 'imu_stale_timeout',
                    'stationary_bias_duration', 'max_encoder_interval', 'max_wheel_speed', 'stationary_wheel_speed', 'stationary_gyro_limit'):
            if not math.isfinite(self.p[key]) or self.p[key] <= 0:
                raise ValueError(f'{key} must be positive and finite')
        if min(self.p['bias_min_samples'], self.p['stationary_bias_min_samples']) < 2 or self.p['imu_history_size'] < max(self.p['bias_min_samples'], self.p['stationary_bias_min_samples']):
            raise ValueError('Invalid IMU calibration/history sample limits')
        self.imu = deque(maxlen=self.p['imu_history_size'])
        self.previous = None
        self.distance_history = deque()
        self.travel = 0.0
        self.first_stamp = None
        self.bias = None
        self.stationary_start = None
        self.bias_start = None
        self.bias_samples = []
        self.bias_last_stamp = -math.inf
        self.x = self.y = self.yaw = 0.0
        self.reason = 'Awaiting encoders and 2 s stationary gyro calibration'

    def add_imu(self, stamp, yaw_rate):
        if not math.isfinite(stamp) or not math.isfinite(yaw_rate):
            return False
        if self.imu and stamp <= self.imu[-1][0]:
            return False
        self.imu.append((stamp, yaw_rate))
        return True

    def reset_calibration(self):
        self.bias_start = None
        self.bias_samples.clear()
        self.bias_last_stamp = -math.inf

    def window_velocity(self, stamp, distance):
        """Capture-time displacement over the trailing window, interpolating its edge."""
        self.travel += distance
        self.distance_history.append((stamp, self.travel))
        cutoff = stamp - self.p['velocity_window']
        while len(self.distance_history) > 2 and self.distance_history[1][0] <= cutoff:
            self.distance_history.popleft()
        start, position = self.distance_history[0]
        if start < cutoff and len(self.distance_history) > 1:
            end, next_position = self.distance_history[1]
            position += (next_position - position) * (cutoff - start) / (end - start)
            start = cutoff
        return (self.travel - position) / (stamp - start) if stamp > start else 0.0

    def step(self, stamp, left, right):
        if not all(math.isfinite(v) for v in (stamp, left, right)):
            self.reason = 'Nonfinite encoder sample'; return None
        if self.previous is None:
            self.previous = (stamp, left, right)
            self.first_stamp = stamp
            self.distance_history.append((stamp, self.travel))
            return None
        old_stamp, old_left, old_right = self.previous
        dt = stamp - old_stamp
        if dt <= 0:
            self.reason = 'Non-increasing encoder stamp'; return None
        self.previous = (stamp, left, right)
        # Driver divided integer centimetres by wheel radius without converting to metres.
        scale = self.p['wheel_radius'] * self.p['encoder_distance_scale']
        dl = (left - old_left) * scale
        dr = (right - old_right) * scale
        if dt > self.p['max_encoder_interval'] or max(abs(dl), abs(dr)) / dt > self.p['max_wheel_speed']:
            if self.bias is None:
                self.reset_calibration()
            self.stationary_start = None
            self.distance_history.clear()
            self.distance_history.append((stamp, self.travel))
            self.reason = 'Encoder gap/reset: rebased without pose jump'; return None
        distance = (dl + dr) / 2
        linear = self.window_velocity(stamp, distance)
        available = [(t, z) for t, z in self.imu if t <= stamp]
        sample = available[-1] if available else None
        fresh = sample is not None and stamp - sample[0] <= self.p['imu_stale_timeout']
        if self.bias is None:
            stationary = max(abs(dl), abs(dr)) / dt <= self.p['stationary_wheel_speed']
            if fresh and stationary and abs(sample[1]) <= self.p['stationary_gyro_limit']:
                if self.bias_start is None:
                    self.bias_start = stamp
                for t, z in available:
                    if t >= self.bias_start and t > self.bias_last_stamp:
                        self.bias_samples.append(z); self.bias_last_stamp = t
                if stamp - self.bias_start >= self.p['bias_duration'] and len(self.bias_samples) >= self.p['bias_min_samples']:
                    self.bias = sum(self.bias_samples) / len(self.bias_samples)
            else:
                self.reset_calibration()
            if self.bias is None and (fresh or stamp - self.first_stamp <= self.p['imu_stale_timeout']):
                self.reason = 'Keep stationary: gyro bias calibration pending'
                return None  # No TF/localisation readiness until calibration, if IMU is available.
        if self.bias is not None:
            # Exact no-count-change detection: any encoder tick resets this window.
            # Reject clear body rotation even if the coarse encoders did not tick.
            if fresh and dl == 0 and dr == 0 and abs(sample[1] - self.bias) <= self.p['stationary_gyro_limit']:
                if self.stationary_start is None:
                    self.stationary_start = old_stamp
                duration = self.p['stationary_bias_duration']
                recent = [(t, z) for t, z in available if t >= max(self.stationary_start, stamp - duration)]
                if stamp - self.stationary_start > duration and len(recent) >= self.p['stationary_bias_min_samples']:
                    self.bias = sum(z for _, z in recent) / len(recent)
            else:
                self.stationary_start = None
        if fresh and self.bias is not None:
            # Gyro sampled at encoder capture time (latest causal sample, zero-order hold).
            angular = sample[1] - self.bias
            source = 'gyro'
        else:
            angular = (dr - dl) / (self.p['effective_track'] * dt)
            source = 'encoder_fallback'
        self.x, self.y, self.yaw = integrate_arc(self.x, self.y, self.yaw, distance, angular * dt)
        self.reason = source
        return Estimate(stamp, self.x, self.y, self.yaw, linear, angular, source, dt)
