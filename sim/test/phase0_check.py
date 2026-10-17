"""Phase 0 live check. Run with Gazebo already publishing /clock and /scan.

This is test infrastructure, not the Phase 1 simulator or a mission node.
Wall-clock deadlines below bound this smoke test only; they are not robot tuning.
"""

import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.parameter_client import AsyncParameterClient
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from interfaces.msg import CoordinatorRequest, Predictions


def main():
    assert os.environ.get('ROS_AUTOMATIC_DISCOVERY_RANGE') == 'LOCALHOST'
    processes = []
    outputs = []
    rclpy.init()
    node = Node('phase0_check', parameter_overrides=[Parameter('use_sim_time', value=True)])
    logdir = Path(tempfile.mkdtemp(prefix='phase0_check_'))
    received = []
    scans = []
    node.create_subscription(Predictions, '/phase0/predictions', received.append, 1)
    node.create_subscription(LaserScan, '/scan', scans.append, qos_profile_sensor_data)
    pub = node.create_publisher(Predictions, '/phase0/predictions', 1)
    try:
        result = subprocess.run(['ros2', 'launch', 'interfaces', 'interfaces.launch.py'],
                                capture_output=True, text=True, timeout=15)
        assert result.returncode == 0, result.stderr
        for area in ['slam', 'planning', 'control', 'perception', 'sim']:
            output = (logdir / (area + '.log')).open('w')
            outputs.append(output)
            processes.append(subprocess.Popen(
                ['ros2', 'launch', area, area + '.launch.py', 'use_sim_time:=true'],
                stdout=output, stderr=subprocess.STDOUT, start_new_session=True))
        deadline = time.monotonic() + 30
        clients = {area: AsyncParameterClient(node, '/' + area + '_skeleton')
                   for area in ['slam', 'planning', 'control', 'perception', 'sim']}
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.1)
            assert all(proc.poll() is None for proc in processes), str(logdir)
            if all(client.services_are_ready() for client in clients.values()):
                break
        assert all(client.services_are_ready() for client in clients.values()), str(logdir)
        for area, client in clients.items():
            future = client.get_parameters(['use_sim_time'])
            rclpy.spin_until_future_complete(node, future, timeout_sec=5)
            assert future.done() and future.result().values[0].bool_value, area
        # DDS delivery check; only test-prefixed topic, never a production prediction.
        message = Predictions()
        message.header.frame_id = 'map'
        message.map_revision = 42
        deadline = time.monotonic() + 15
        while time.monotonic() < deadline and (not received or not scans):
            pub.publish(message)
            rclpy.spin_once(node, timeout_sec=.1)
        assert received and received[-1].map_revision == 42
        assert len(received[-1].robots) == 3
        assert scans, 'Gazebo /scan missing'
        assert node.get_clock().now().nanoseconds > 0, 'Gazebo /clock missing'
        assert node.count_publishers('/cmd_vel') == 0, 'Unexpected Phase 0 motion publisher'
        assert CoordinatorRequest().mode == CoordinatorRequest.WAIT
        print('PASS: six launch files; five live shells use_sim_time=true; /clock advances; '
              '/scan arrives; custom messages traverse DDS; /cmd_vel has zero publishers.')
        print('Per-launch output: ' + str(logdir))
    finally:
        for proc in processes:
            if proc.poll() is None:
                os.killpg(proc.pid, signal.SIGINT)
        for proc in processes:
            try:
                proc.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(proc.pid, signal.SIGTERM)
                proc.wait(timeout=5)
        for output in outputs:
            output.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
