from types import SimpleNamespace as NS
from rclpy.task import Future
from planning.explore_adapter import ExploreAdapter

class Handle:
    is_active=True
    is_cancel_requested=False
    def succeed(self):self.result='success';self.is_active=False
    def abort(self):self.result='abort';self.is_active=False
    def canceled(self):self.result='cancel';self.is_active=False

def test_cancelled_frontier_finishes_without_success():
    adapter=ExploreAdapter.__new__(ExploreAdapter);handle=Handle();handle.is_cancel_requested=True
    future=Future();adapter.active=(handle,future);adapter.node=NS(request=object())
    adapter.finish(True)
    assert handle.result=='cancel' and future.done() and adapter.active is None and adapter.node.request is None

def test_failed_frontier_is_reported_to_package_for_blacklisting():
    adapter=ExploreAdapter.__new__(ExploreAdapter);handle=Handle();future=Future()
    adapter.active=(handle,future);adapter.node=NS(request=object());adapter.finish(False)
    assert handle.result=='abort' and future.done()
    assert future.result().error_code!=0 # explore_lite interprets zero as preemption

def test_real_action_result_failure_reaches_explore_client():
    """Exercise rclpy async action scheduling, not just the result helper."""
    import time
    import rclpy
    from rclpy.node import Node
    from rclpy.action import ActionClient
    from nav2_msgs.action import NavigateToPose
    from action_msgs.msg import GoalStatus
    from interfaces.msg import CoordinatorStatus
    rclpy.init()
    node=Node('explore_bridge_test');node.request=None;node.status=None
    node.p=lambda key:True;node.now=lambda:0.
    node.issue=lambda *args:setattr(node,'request',NS(request_id='test'))
    adapter=ExploreAdapter(node);client=ActionClient(node,NavigateToPose,'/exploration/navigate_to_pose')
    def until(predicate):
        deadline=time.monotonic()+5
        while not predicate() and time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.02)
        assert predicate()
    try:
        assert client.wait_for_server(timeout_sec=3)
        goal=NavigateToPose.Goal();goal.pose.header.frame_id='map';goal.pose.pose.orientation.w=1.
        accepted=client.send_goal_async(goal);until(accepted.done);handle=accepted.result()
        assert handle.accepted
        until(lambda:adapter.active is not None)
        result=handle.get_result_async();adapter.tick()
        node.status=CoordinatorStatus(request_id='test',terminal=True,status=CoordinatorStatus.FAILED)
        adapter.tick();until(result.done)
        assert result.result().status==GoalStatus.STATUS_ABORTED
        assert result.result().result.error_code!=0
    finally:
        adapter.finish();client.destroy();adapter.server.destroy();node.destroy_node();rclpy.shutdown()
