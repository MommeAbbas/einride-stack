"""Adapt explore_lite NavigateToPose goals to the existing leased coordinator.

Only protocol glue: explore_lite selects/blacklists frontiers; Nav2 plans/drives.
"""
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.task import Future
from nav2_msgs.action import NavigateToPose, FollowPath
from explore_lite_msgs.msg import ExploreStatus
from std_msgs.msg import Bool
from interfaces.msg import CoordinatorStatus
from .ros_support import latched


class ExploreAdapter:
    def __init__(self, node):
        self.node=node;self.active=None;self.done=False;self.finished_at=None
        self.resume=node.create_publisher(Bool,'/explore/resume',10)
        self.subscription=node.create_subscription(ExploreStatus,'/explore/status',self.status,latched())
        self.server=ActionServer(node,NavigateToPose,'/exploration/navigate_to_pose',
            execute_callback=self.execute,goal_callback=self.accept,
            cancel_callback=lambda g:CancelResponse.ACCEPT,callback_group=ReentrantCallbackGroup())

    def accept(self,goal):
        return GoalResponse.ACCEPT if self.node.p('explore') and not self.done and goal.pose.header.frame_id=='map' else GoalResponse.REJECT

    def status(self,msg):
        if msg.status==msg.EXPLORATION_COMPLETE:
            self.done=True;self.finished_at=self.node.now()
            self.node.get_logger().info(f'explore_lite frontier completion at {self.finished_at:.3f} sim s (includes blacklisted frontiers; not proof of complete coverage)')

    async def execute(self,handle):
        if self.active is not None:
            old,future=self.active
            if old.is_active:old.abort()
            if not future.done():future.set_result(NavigateToPose.Result())
        future=Future();self.active=(handle,future)
        self.node.request=None
        return await future

    def finish(self,success=False):
        if self.active is None:return
        handle,future=self.active
        if handle.is_active:
            if handle.is_cancel_requested:handle.canceled()
            elif success:handle.succeed()
            else:handle.abort()
        result=NavigateToPose.Result()
        if not success and not handle.is_cancel_requested:
            result.error_code=FollowPath.Result.UNKNOWN
            result.error_msg='Coordinator reported failed navigation; blacklist this frontier'
        if not future.done():future.set_result(result)
        self.active=None;self.node.request=None

    def tick(self):
        n=self.node
        if self.done:
            self.finish();return False
        n.goal='exploration'
        if self.active is None:
            n.issue('WAIT','Map: awaiting explore_lite frontier');return True
        handle,_=self.active
        if handle.is_cancel_requested:
            self.finish();n.issue('WAIT','Map: frontier cancelled');return True
        if n.request and n.status and n.status.request_id==n.request.request_id and n.status.terminal:
            self.finish(n.status.status==CoordinatorStatus.SUCCEEDED)
            n.issue('WAIT','Map: frontier result');return True
        n.issue('DRIVE','Map: explore_lite '+bytes(handle.goal_id.uuid).hex(),handle.request.pose.pose)
        return True

    def stop(self):
        self.resume.publish(Bool(data=False));self.done=True;self.finish()
