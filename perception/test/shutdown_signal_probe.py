# Test-only scheduling hook: signal at a real image take, not a fabricated exception.
import os,time
from perception import ros_support
from perception.dock_node import DockNode
from perception.perception_node import PerceptionNode
cls=DockNode if os.environ['TARGET']=='dock' else PerceptionNode
original=ros_support.DiagnosticExecutor._take_subscription
start=time.monotonic();armed=True

def take(self,sub):
    global armed
    if armed and time.monotonic()-start>6 and sub.msg_type.__name__=='Image':
        armed=False
        self.audit()
        os.write(int(os.environ['SIGNAL_FD']),b'I')
    return original(self,sub)
ros_support.DiagnosticExecutor._take_subscription=take
ros_support.run(cls)
