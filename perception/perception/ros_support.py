"""Thin shared ROS parameter/message conversions; algorithms live in other modules."""
from pathlib import Path
import math
import json
import sys
import signal
from rclpy.signals import SignalHandlerOptions
from rclpy.executors import SingleThreadedExecutor
import yaml
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, DurabilityPolicy


def defaults():
    path = Path(get_package_share_directory('perception')) / 'config/perception.yaml'
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


class DiagnosticExecutor(SingleThreadedExecutor):
    """Identify the exact failing take and declared/published types; live errors remain fatal."""
    def __init__(self,node):
        super().__init__(context=node.context)
        self.node=node
        self.type_snapshot={}
        period=node.get_parameter('subscription_audit_period').value if node.has_parameter('subscription_audit_period') else defaults()['subscription_audit_period']
        self.audit_timer=node.create_timer(period,self.audit)

    def describe(self,sub):
        cls=sub.msg_type
        expected=f'{cls.__module__.split(".")[0]}/msg/{cls.__name__}'
        publishers=self.node.get_publishers_info_by_topic(sub.topic_name)
        local=[e for e in self.node.get_subscriptions_info_by_topic(sub.topic_name) if e.node_name==self.node.get_name() and e.node_namespace==self.node.get_namespace()]
        return dict(topic=sub.topic_name,declared=expected,module=sys.modules[cls.__module__].__file__,
                    declared_hashes=sorted({str(e.topic_type_hash) for e in local if e.topic_type==expected}),
                    publishers=[dict(node=p.node_name,type=p.topic_type,type_hash=str(p.topic_type_hash)) for p in publishers])

    def audit(self):
        for sub in self.node.subscriptions:
            info=self.describe(sub)
            previous=self.type_snapshot.get(sub.topic_name)
            self.type_snapshot[sub.topic_name]=info
            if info!=previous:
                mismatched=any(p['type']!=info['declared'] for p in info['publishers'])
                message='subscription_type '+json.dumps(info,sort_keys=True)
                (self.node.get_logger().error if mismatched else self.node.get_logger().info)(message)

    def _take_subscription(self,sub):
        try:return super()._take_subscription(sub)
        except RuntimeError as error:
            info=self.type_snapshot.get(sub.topic_name,dict(topic=sub.topic_name,declared=str(sub.msg_type)))
            # stderr still works after the ROS logging context is invalid.
            print('subscription_take_failure '+json.dumps(dict(info,context_ok=self.node.context.ok(),error=str(error))),file=sys.stderr,flush=True)
            raise


def run(cls):
    # Default rclpy SIGINT tears down the context inside Image conversion; Jazzy's
    # pybind then reports a misleading argument-conversion RuntimeError. Defer
    # shutdown until spin_once returns. Signal handlers never call ROS or raise.
    stopped=False
    def request_stop(signum,frame):
        nonlocal stopped
        stopped=True
    previous={sig:signal.getsignal(sig) for sig in (signal.SIGINT,signal.SIGTERM)}
    for sig in previous:signal.signal(sig,request_stop)
    node=None;executor=None
    try:
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
        node=cls();executor=DiagnosticExecutor(node);executor.add_node(node)
        while rclpy.ok() and not stopped:
            executor.spin_once(timeout_sec=0.1)  # [s wall] Maximum idle stop latency.
    except (KeyboardInterrupt,rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if executor is not None:executor.shutdown()
        if node is not None:node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
        for sig,handler in previous.items():signal.signal(sig,handler)
