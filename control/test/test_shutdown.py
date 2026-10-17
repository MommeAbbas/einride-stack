"""Real rclpy context teardown must not publish or invoke closed actions."""
import rclpy
from control.nav2_coordinator import Nav2Coordinator

def test_coordinator_teardown_after_context_shutdown():
    rclpy.init()
    node=Nav2Coordinator()
    rclpy.shutdown()
    node.cancel=lambda: (_ for _ in ()).throw(AssertionError('closed action context used'))
    node.destroy_node()
