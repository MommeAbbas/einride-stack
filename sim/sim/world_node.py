"""One-shot ROS-parameterised generator, executed before Gazebo starts."""
import rclpy
from ament_index_python.packages import get_package_share_directory
from .ros_support import SimNode, defaults
from .world import generate


def main():
    rclpy.init()
    node=SimNode('sim_world')
    try:
        generate(get_package_share_directory('einride_mini_truck_gazebo'),
                 get_package_share_directory('einride_mini_truck_description'),node.p('run_dir'),
                 {key:node.p(key) for key in defaults()})
        node.get_logger().info('Generated private arena, sensor-free traffic, oracle map and bridges')
    finally:
        node.destroy_node(); rclpy.shutdown()
