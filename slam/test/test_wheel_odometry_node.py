"""Hardware-topic adapter check with synthetic sensors; no simulator or actuators."""
import time
import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data, ReliabilityPolicy
from builtin_interfaces.msg import Time
from sensor_msgs.msg import JointState, Imu
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage
from slam.wheel_odometry_node import WheelOdometryNode


def test_best_effort_sensors_publish_matching_odom_and_tf():
    rclpy.init(args=['--ros-args', '-p', 'use_sim_time:=false'])
    node = WheelOdometryNode()
    probe = rclpy.create_node('wheel_odometry_test')
    executor = SingleThreadedExecutor(); executor.add_node(node); executor.add_node(probe)
    enc = probe.create_publisher(JointState, '/wheel_encoders', qos_profile_sensor_data)
    imu = probe.create_publisher(Imu, '/imu', qos_profile_sensor_data)
    odom, transforms = [], []
    probe.create_subscription(Odometry, '/odom', odom.append, 10)
    probe.create_subscription(TFMessage, '/tf', lambda m: transforms.extend(m.transforms), 10)
    def until(condition):
        deadline = time.monotonic() + 5
        while not condition() and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=.01)
        assert condition(), 'ROS adapter check timed out'
    def send(i, left, right, gyro=.03):
        stamp = Time(sec=i//20, nanosec=(i%20)*50000000)
        if gyro is not None:
            m=Imu();m.header.stamp=stamp;m.header.frame_id='base_imu_link';m.angular_velocity.z=gyro
            imu.publish(m)
            until(lambda: bool(node.integrator.imu) and node.integrator.imu[-1][0] == pytest.approx(i/20))
        # Deliberately reverse joint order; this is name-based, not index-based.
        m=JointState(name=['right_up_wheel_link_joint','left_up_wheel_link_joint'],position=[right,left])
        m.header.stamp=stamp;enc.publish(m)
        until(lambda: node.integrator.previous is not None and node.integrator.previous[0] == pytest.approx(i/20))
    try:
        until(lambda: enc.get_subscription_count()>0 and imu.get_subscription_count()>0 and node.publisher.get_subscription_count()>0)
        for topic in ('/wheel_encoders','/imu'):
            assert all(info.qos_profile.reliability==ReliabilityPolicy.BEST_EFFORT for info in probe.get_subscriptions_info_by_topic(topic))
        for i in range(45):send(i,1200.,-700.)
        until(lambda: bool(odom) and bool(transforms))
        assert node.integrator.bias == pytest.approx(.03)
        send(45,1225.,-675.,.53)
        until(lambda: any(m.header.stamp.nanosec==250000000 for m in odom) and any(m.header.stamp.nanosec==250000000 for m in transforms))
        m=next(m for m in reversed(odom) if m.header.stamp.nanosec==250000000)
        tf=next(m for m in reversed(transforms) if m.header.stamp.nanosec==250000000)
        assert m.header.frame_id=='odom' and m.child_frame_id=='base_footprint'
        assert tf.header==m.header and tf.child_frame_id==m.child_frame_id
        assert tf.transform.translation.x==m.pose.pose.position.x
        assert tf.transform.rotation==m.pose.pose.orientation
        assert m.twist.twist.linear.x==pytest.approx(.04)
        assert m.twist.twist.angular.z==pytest.approx(.5)
        assert m.pose.covariance[0]>node.p['pose_variance'][0]
        send(51,1200.,-650.,gyro=None)
        until(lambda: any(m.header.stamp.nanosec==550000000 for m in odom))
        m=next(m for m in reversed(odom) if m.header.stamp.nanosec==550000000)
        assert m.twist.covariance[35]==node.p['fallback_yaw_variance']
        assert node.count_publishers('/cmd_vel')==0
    finally:
        executor.shutdown();node.destroy_node();probe.destroy_node();rclpy.shutdown()
