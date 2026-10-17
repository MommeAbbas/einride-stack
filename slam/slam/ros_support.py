"""Thin shared ROS parameter/message conversions; algorithms live in other modules."""
from pathlib import Path
import math
import yaml
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, DurabilityPolicy


def defaults():
    path = Path(get_package_share_directory('slam')) / 'config/slam.yaml'
    return yaml.safe_load(path.read_text())['/**']['ros__parameters']


class AreaNode(Node):
    def __init__(self, name, overrides=()):
        # Constructor overrides outrank launch YAML in rclpy. Never force sim time here.
        super().__init__(name, parameter_overrides=list(overrides))
        values=defaults()
        values.update({p.name:p.value for p in overrides})
        for key, value in values.items():
            if key != 'use_sim_time':
                self.declare_parameter(key, value)

    def p(self, key):
        return self.get_parameter(key).value

    def now(self):
        return self.get_clock().now().nanoseconds / 1e9


def seconds(stamp):
    return stamp.sec + stamp.nanosec / 1e9


def yaw(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))


def latched():
    return QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)


def run(cls):
    rclpy.init()
    node = cls()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    except RuntimeError:
        # Jazzy may race SIGINT against take_message's pybind conversion. Only
        # suppress this after ROS context shutdown; live runtime errors still fail.
        if rclpy.ok():raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
