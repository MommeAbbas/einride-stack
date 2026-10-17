"""Retain the active NavigateToPose target across Nav2 aborts; cancellation wins.

Nav2 remains responsible for planning and control. The adapter retains goals across aborts
and pauses the backend for the separately guarded escape. Hardware profiles only.
"""
from copy import deepcopy
import rclpy
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import Bool
from nav2_msgs.msg import SpeedLimit
from rclpy.action import ActionClient,ActionServer,GoalResponse,CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.task import Future
from .ros_support import AreaNode,run


class NavigationRelay(AreaNode):
    def __init__(self):
        super().__init__('control_navigation_relay')
        self.escape_hold=False
        self.active_pub=self.create_publisher(Bool,'/control/navigation_active',1)
        self.paused_pub=self.create_publisher(Bool,'/control/navigation_paused',1)
        self.create_subscription(Bool,'/control/escape_active',self.escape,1)
        self.speed=self.create_publisher(SpeedLimit,'/speed_limit',10)
        self.active=None;self.backend=None;self.pending=None;self.retiring=[];self.retry_at=0.;self.attempt=0
        self.client=ActionClient(self,NavigateToPose,'/nav2/navigate_to_pose')
        self.server=ActionServer(self,NavigateToPose,'/navigate_to_pose',execute_callback=self.execute,
            goal_callback=lambda g:GoalResponse.ACCEPT,cancel_callback=lambda h:CancelResponse.ACCEPT,
            callback_group=ReentrantCallbackGroup())
        self.create_timer(self.p('navigation_watch_period'),self.tick)

    def escape(self,m):
        if m.data and not self.escape_hold:self.cancel_backend()
        self.escape_hold=m.data

    def cancel_backend(self):
        if self.backend is not None:
            self.retiring.append(self.backend.get_result_async());self.backend.cancel_goal_async();self.backend=None
        if self.pending is not None:self.retiring.append(self.pending);self.pending=None

    def finish(self,status,result=None):
        if self.active is None:return
        handle,future=self.active;self.active=None
        self.cancel_backend()
        self.speed.publish(SpeedLimit(speed_limit=0.,percentage=False))
        if handle.is_active:
            if handle.is_cancel_requested:handle.canceled()
            elif status==GoalStatus.STATUS_SUCCEEDED:handle.succeed()
            else:handle.abort()
        if not future.done():future.set_result(result or NavigateToPose.Result())

    async def execute(self,handle):
        self.finish(GoalStatus.STATUS_ABORTED)  # New target replaces, never retries an obsolete target.
        future=Future();self.active=(handle,future);self.retry_at=self.now();self.attempt=0
        return await future

    def tick(self):
        self.retiring=[f for f in self.retiring if not f.done()]
        self.active_pub.publish(Bool(data=self.active is not None))
        self.paused_pub.publish(Bool(data=self.escape_hold and self.backend is None and self.pending is None and not self.retiring))
        if self.active is None:return
        handle,_=self.active
        if handle.is_cancel_requested:self.finish(GoalStatus.STATUS_CANCELED);return
        if self.escape_hold or self.backend is not None or self.pending is not None or self.retiring or self.now()<self.retry_at:return
        if not self.client.server_is_ready():return
        self.attempt+=1;goal=deepcopy(handle.request);goal.pose.header.stamp=self.get_clock().now().to_msg()
        self.get_logger().info(f'NavigateToPose attempt={self.attempt} target=({goal.pose.pose.position.x:.3f},{goal.pose.pose.position.y:.3f})')
        def feedback(message):
            if self.active and self.active[0] is handle:
                cap=self.p('minimum_linear_command') if message.feedback.distance_remaining<self.p('navigation_slow_distance') else 0.
                self.speed.publish(SpeedLimit(speed_limit=cap,percentage=False))
            if self.active is not None and self.active[0] is handle and handle.is_active:handle.publish_feedback(message.feedback)
        future=self.client.send_goal_async(goal,feedback_callback=feedback);self.pending=future
        def accepted(f):
            try:backend=f.result()
            except Exception as error:
                if self.active and self.active[0] is handle:self.pending=None;self.retry(str(error))
                return
            if self.active is None or self.active[0] is not handle or handle.is_cancel_requested or self.escape_hold:
                if backend.accepted:self.retiring.append(backend.get_result_async());backend.cancel_goal_async()
                if self.pending is f:self.pending=None
                return
            self.pending=None
            if not backend.accepted:self.retry('Nav2 rejected target');return
            self.backend=backend
            def completed(done):
                if self.active is None or self.active[0] is not handle:return
                self.backend=None
                if handle.is_cancel_requested:self.finish(GoalStatus.STATUS_CANCELED);return
                try:result=done.result()
                except Exception as error:self.retry(str(error));return
                if result.status==GoalStatus.STATUS_SUCCEEDED:self.finish(result.status,result.result)
                else:self.retry(f'Nav2 status={result.status}; {result.result.error_msg}')
            backend.get_result_async().add_done_callback(completed)
        future.add_done_callback(accepted)

    def retry(self,reason):
        self.retry_at=self.now()+self.p('navigation_retry_period')
        self.get_logger().warning(f'{reason}; active target retained; resending for fresh Nav2 plan')

    def destroy_node(self):
        if rclpy.ok(context=self.context):self.cancel_backend()
        super().destroy_node()


def main():run(NavigationRelay)
