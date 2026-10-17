"""Keep context alive through an interrupted callback, and never mask live faults."""
import os,signal
import pytest
import rclpy
from rclpy.node import Node
from perception.ros_support import run

@pytest.mark.parametrize('sig',[signal.SIGINT,signal.SIGTERM])
def test_signal_defers_context_shutdown_until_callback_returns(sig):
    evidence=[]
    class Probe(Node):
        def __init__(self):
            super().__init__('perception_shutdown_regression')
            self.create_timer(.01,self.callback)
        def callback(self):
            os.kill(os.getpid(),sig)
            evidence.append(self.context.ok())
    run(Probe)
    assert evidence==[True]
    assert not rclpy.ok()


def test_live_runtime_error_is_not_suppressed():
    class Probe(Node):
        def __init__(self):
            super().__init__('perception_live_fault_regression')
            self.create_timer(.01,self.callback)
        def callback(self):raise RuntimeError('live fault')
    with pytest.raises(RuntimeError,match='live fault'):run(Probe)
    assert not rclpy.ok()
