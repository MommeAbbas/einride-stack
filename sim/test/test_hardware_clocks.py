import importlib
import pytest
import rclpy
import yaml

@pytest.mark.parametrize('sim',[False,True])
@pytest.mark.parametrize('area',['slam','control','planning','perception'])
def test_launch_clock_reaches_real_node_and_subscription(tmp_path,area,sim):
    config=tmp_path/'clock.yaml';config.write_text(yaml.safe_dump({'/**':{'ros__parameters':{'use_sim_time':sim}}}))
    rclpy.init(args=['--ros-args','--params-file',str(config)])
    node=importlib.import_module(area+'.ros_support').AreaNode('clock_regression_'+area)
    try:
        assert node.get_parameter('use_sim_time').value is sim
        assert any(s.topic_name=='/clock' for s in node.subscriptions)==sim
        assert (node.get_clock().now().nanoseconds==0)==sim
    finally:node.destroy_node();rclpy.shutdown()
