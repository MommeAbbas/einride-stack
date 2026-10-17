import time
import rclpy
from rclpy.action import ActionClient,ActionServer,CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from control.navigation_relay import NavigationRelay


def test_aborted_active_goal_replans_by_resubmitting_same_target():
    rclpy.init(args=['--ros-args','-p','navigation_retry_period:=0.01','-p','navigation_watch_period:=0.01'])
    relay=NavigationRelay();backend=Node('fake_installed_nav2');probe=Node('relay_check');calls=[]
    async def execute(handle):
        calls.append(handle.request.pose.pose.position.x)
        if len(calls)==1:handle.abort()
        else:handle.succeed()
        return NavigateToPose.Result()
    server=ActionServer(backend,NavigateToPose,'/nav2/navigate_to_pose',execute_callback=execute,callback_group=ReentrantCallbackGroup())
    client=ActionClient(probe,NavigateToPose,'/navigate_to_pose')
    executor=SingleThreadedExecutor()
    for node in (relay,backend,probe):executor.add_node(node)
    def until(predicate):
        deadline=time.monotonic()+5
        while not predicate() and time.monotonic()<deadline:executor.spin_once(timeout_sec=.01)
        assert predicate()
    try:
        until(lambda:client.server_is_ready() and relay.client.server_is_ready())
        pose=PoseStamped();pose.header.frame_id='map';pose.pose.position.x=2.;pose.pose.orientation.w=1.
        accepted=client.send_goal_async(NavigateToPose.Goal(pose=pose));until(accepted.done)
        result=accepted.result().get_result_async();until(result.done)
        assert result.result().status==4 and calls==[2.,2.]
        assert relay.active is None
    finally:
        executor.shutdown();server.destroy()
        for node in (relay,backend,probe):node.destroy_node()
        rclpy.shutdown()


def test_cancelled_goal_is_not_resubmitted():
    from rclpy.task import Future
    rclpy.init(args=['--ros-args','-p','navigation_retry_period:=0.01','-p','navigation_watch_period:=0.01'])
    relay=NavigationRelay();backend=Node('fake_nav2_cancel');probe=Node('relay_cancel_check');calls=[];pending=[]
    async def execute(handle):
        calls.append(handle);future=Future();pending.append((handle,future));return await future
    server=ActionServer(backend,NavigateToPose,'/nav2/navigate_to_pose',execute_callback=execute,
        cancel_callback=lambda h:CancelResponse.ACCEPT,callback_group=ReentrantCallbackGroup())
    def tick():
        for handle,future in pending:
            if handle.is_cancel_requested and not future.done():handle.canceled();future.set_result(NavigateToPose.Result())
    backend.create_timer(.01,tick)
    client=ActionClient(probe,NavigateToPose,'/navigate_to_pose');executor=SingleThreadedExecutor()
    for node in (relay,backend,probe):executor.add_node(node)
    def until(predicate):
        end=time.monotonic()+5
        while not predicate() and time.monotonic()<end:executor.spin_once(timeout_sec=.01)
        assert predicate()
    try:
        until(lambda:client.server_is_ready() and relay.client.server_is_ready())
        future=client.send_goal_async(NavigateToPose.Goal());until(future.done);handle=future.result()
        until(lambda:len(calls)==1)
        cancel=handle.cancel_goal_async();until(cancel.done)
        result=handle.get_result_async();until(result.done)
        assert result.result().status==5
        until(lambda:all(f.done() for h,f in pending))
        for _ in range(20):executor.spin_once(timeout_sec=.01)
        assert len(calls)==1 and relay.active is None
    finally:
        executor.shutdown();server.destroy()
        for node in (relay,backend,probe):node.destroy_node()
        rclpy.shutdown()


def test_escape_pauses_backend_but_retains_target_and_replans():
    from types import SimpleNamespace
    from rclpy.task import Future
    from std_msgs.msg import Bool
    rclpy.init();relay=NavigationRelay()
    result=Future();cancelled=[]
    target=object();relay.active=(target,Future())
    relay.backend=SimpleNamespace(get_result_async=lambda:result,cancel_goal_async=lambda:cancelled.append(True))
    try:
        relay.escape(Bool(data=True))
        assert cancelled and relay.backend is None and relay.active[0] is target and relay.escape_hold
        assert relay.retiring==[result]
        relay.escape(Bool(data=False))
        assert relay.active[0] is target and not relay.escape_hold
    finally:relay.active=None;relay.destroy_node();rclpy.shutdown()
