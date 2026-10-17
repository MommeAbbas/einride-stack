"""Minimal package health-check node with no planning outputs."""

import rclpy
from rclpy.node import Node


class SkeletonNode(Node):
    """Keep the package launchable without pretending the area is implemented."""

    def __init__(self):
        super().__init__('planning_skeleton')
        self.get_logger().info('Planning package health-check node; no functional outputs')


def main(args=None):
    rclpy.init(args=args)
    node = SkeletonNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
