"""Check installed launch/config and a live ROS shell, with no motion publication."""

import importlib.util
from pathlib import Path

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from planning.skeleton_node import SkeletonNode


def test_installed_launch_and_sim_clock():
    share = Path(get_package_share_directory('planning'))
    params = yaml.safe_load((share / 'config/planning.yaml').read_text())['/**']['ros__parameters']
    assert params['use_sim_time'] is True
    spec = importlib.util.spec_from_file_location('area_launch', share / 'launch/planning.launch.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    assert module.generate_launch_description().entities
    rclpy.init(args=['--ros-args', '-p', 'use_sim_time:=true'])
    node = SkeletonNode()
    try:
        assert node.get_parameter('use_sim_time').value is True
        assert node.get_clock().now().nanoseconds == 0  # No /clock: never use wall time.
        assert node.count_publishers('/cmd_vel') == 0
    finally:
        node.destroy_node()
        rclpy.shutdown()
