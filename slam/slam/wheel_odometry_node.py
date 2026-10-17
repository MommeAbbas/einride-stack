"""Hardware-only /wheel_encoders + /imu -> /odom and odom -> base_footprint."""
from pathlib import Path
import math
import yaml
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState, Imu
from tf2_ros import TransformBroadcaster
from .wheel_odometry import WheelOdometry


def seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def diagonal(values):
    result = [0.0] * 36
    for i, value in enumerate(values):
        result[7 * i] = float(value)
    return result


class WheelOdometryNode(Node):
    def __init__(self):
        super().__init__('slam_wheel_odometry')
        config = Path(get_package_share_directory('slam')) / 'config/wheel_odometry.yaml'
        params = yaml.safe_load(config.read_text())['/**']['ros__parameters']
        for key, value in params.items():
            if key != 'use_sim_time':
                self.declare_parameter(key, value)
        self.p = {key: self.get_parameter(key).value for key in params}
        self.integrator = WheelOdometry(self.p)
        for key in ('pose_variance', 'pose_variance_rate', 'twist_variance'):
            if len(self.p[key]) != 6 or not all(math.isfinite(v) and v >= 0 for v in self.p[key]):
                raise ValueError(f'{key} must contain six finite nonnegative variances')
        self.pose_variance = list(self.p['pose_variance'])
        self.last_source = None
        self.last_bias = None
        self.publisher = self.create_publisher(Odometry, self.p['odom_topic'], self.p['output_queue'])
        self.broadcaster = TransformBroadcaster(self)
        self.create_subscription(Imu, self.p['imu_topic'], self.on_imu, qos_profile_sensor_data)
        self.create_subscription(JointState, self.p['encoder_topic'], self.on_encoders, qos_profile_sensor_data)
        self.get_logger().info('Wheel odometry: keep robot stationary for 2 s gyro bias calibration; hardware-only TF publisher')

    def on_imu(self, message):
        if message.header.frame_id != self.p['imu_frame'] or message.angular_velocity_covariance[0] < 0:
            self.get_logger().warning('Ignoring IMU: wrong frame or unavailable angular velocity', throttle_duration_sec=self.p['log_period'])
            return
        self.integrator.add_imu(seconds(message.header.stamp), message.angular_velocity.z)

    def on_encoders(self, message):
        try:
            left = message.position[message.name.index(self.p['left_joint'])]
            right = message.position[message.name.index(self.p['right_joint'])]
        except (ValueError, IndexError):
            self.get_logger().warning('Ignoring encoders: required joint positions missing', throttle_duration_sec=self.p['log_period'])
            return
        estimate = self.integrator.step(seconds(message.header.stamp), left, right)
        if estimate is None:
            self.get_logger().warning(self.integrator.reason, throttle_duration_sec=self.p['log_period'])
            return
        if self.integrator.bias is not None and self.last_bias is None:
            self.last_bias = self.integrator.bias
            self.get_logger().info(f'Gyro bias calibrated: {self.last_bias:.8f} rad/s')
        if self.last_source != estimate.yaw_source:
            self.last_source = estimate.yaw_source
            self.get_logger().info(f'Odometry yaw source: {self.last_source}')
        rates = list(self.p['pose_variance_rate'])
        twist_variance = list(self.p['twist_variance'])
        if estimate.yaw_source == 'encoder_fallback':
            rates[5] = self.p['fallback_yaw_variance_rate']
            twist_variance[5] = self.p['fallback_yaw_variance']
        self.pose_variance = [v + rate * estimate.dt for v, rate in zip(self.pose_variance, rates)]
        output = Odometry()
        output.header.stamp = message.header.stamp
        output.header.frame_id = self.p['odom_frame']
        output.child_frame_id = self.p['base_frame']
        output.pose.pose.position.x, output.pose.pose.position.y = estimate.x, estimate.y
        output.pose.pose.orientation.z = math.sin(estimate.yaw / 2)
        output.pose.pose.orientation.w = math.cos(estimate.yaw / 2)
        output.twist.twist.linear.x = estimate.linear
        output.twist.twist.angular.z = estimate.angular
        output.pose.covariance = diagonal(self.pose_variance)
        output.twist.covariance = diagonal(twist_variance)
        transform = TransformStamped(header=output.header, child_frame_id=output.child_frame_id)
        transform.transform.translation.x = estimate.x
        transform.transform.translation.y = estimate.y
        transform.transform.rotation = output.pose.pose.orientation
        self.publisher.publish(output)
        self.broadcaster.sendTransform(transform)


def main():
    rclpy.init()
    node = None
    try:
        node = WheelOdometryNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
