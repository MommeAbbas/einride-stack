"""Dock action state and target selection; installed Nav2 handles all motion."""
from copy import deepcopy
from pathlib import Path
import math
from ament_index_python.packages import get_package_share_directory
from rclpy.action import ActionClient,ActionServer,GoalResponse,CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.task import Future
from rclpy.time import Time
from tf2_ros import Buffer,TransformListener,TransformException
from nav2_msgs.action import NavigateToPose,Spin,DockRobot,UndockRobot
from geometry_msgs.msg import PoseStamped
from builtin_interfaces.msg import Duration
from std_msgs.msg import String
from interfaces.msg import DockMap
from .ros_support import AreaNode,latched,run,yaw

class DockingAdapter(AreaNode):
    def __init__(self):
        super().__init__('control_docking_adapter')
        self.status_pub=self.create_publisher(String,'/control/docking_state',10)
        self.docks=None;self.active=None;self.backend_handle=None;self.pending=None;self.retiring=[];self.phase='IDLE'
        self.buffer=Buffer();self.listener=TransformListener(self.buffer,self)
        self.create_subscription(DockMap,'/slam/docks',lambda m:setattr(self,'docks',m),latched())
        self.nav=ActionClient(self,NavigateToPose,'/navigate_to_pose')
        self.spin=ActionClient(self,Spin,'/spin')
        self.dock=ActionClient(self,DockRobot,'/nav2/dock_robot')
        self.undock=ActionClient(self,UndockRobot,'/nav2/undock_robot')
        self.server=ActionServer(self,DockRobot,'/dock_robot',execute_callback=self.execute_dock,goal_callback=self.allow,
            cancel_callback=lambda h:CancelResponse.ACCEPT,callback_group=ReentrantCallbackGroup())
        self.unserver=ActionServer(self,UndockRobot,'/undock_robot',execute_callback=self.execute_undock,goal_callback=self.allow,
            cancel_callback=lambda h:CancelResponse.ACCEPT,callback_group=ReentrantCallbackGroup())
        self.create_timer(self.p('navigation_watch_period'),self.tick)

    def allow(self,g):return GoalResponse.ACCEPT if self.active is None else GoalResponse.REJECT

    async def execute_dock(self,h):
        future=Future();self.active=(h,future,DockRobot.Result);self.phase='STAGE'
        self.started=self.now()
        try:
            ident=int(h.request.dock_id.removeprefix('dock_')) if h.request.use_dock_id else int(h.request.dock_type.removeprefix('dock_'))
            if not 0<=ident<8 or self.docks is None or not self.docks.ready or not self.docks.docks[ident].valid:raise ValueError('Qualified saved dock map not ready')
            self.target=deepcopy(self.docks.docks[ident]);self.ident=ident
        except (ValueError,IndexError) as e:self.finish(False,str(e))
        return await future

    async def execute_undock(self,h):
        future=Future();self.active=(h,future,UndockRobot.Result);self.phase='UNDOCK';self.started=self.now()
        return await future

    def finish(self,success,reason=''):
        if self.active is None:return
        h,f,kind=self.active;self.active=None;self.phase='IDLE'
        if self.backend_handle is not None:
            self.retiring.append(self.backend_handle.get_result_async());self.backend_handle.cancel_goal_async();self.backend_handle=None
        result=kind();result.success=success;result.error_msg=reason;result.error_code=0 if success else result.FAILED_TO_CONTROL
        if h.is_cancel_requested:h.canceled()
        elif success:h.succeed()
        else:h.abort()
        f.set_result(result)

    def submit(self,client,goal,next_phase):
        if not client.server_is_ready():return
        active_goal=self.active;self.pending=client.send_goal_async(goal)
        def accepted(f):
            self.pending=None;handle=f.result()
            if self.active is not active_goal:
                if handle.accepted:self.retiring.append(handle.get_result_async());handle.cancel_goal_async()
                return
            if not handle.accepted:self.finish(False,'Nav2 rejected '+self.phase);return
            self.backend_handle=handle
            def result(f):
                if self.active is not active_goal:return
                self.backend_handle=None;r=f.result()
                if r.status!=4:self.finish(False,f'{self.phase} action failed: {r.status}; {getattr(r.result, "error_msg", "")}');return
                self.phase=next_phase
                if next_phase=='DONE':self.finish(True)
            handle.get_result_async().add_done_callback(result)
        self.pending.add_done_callback(accepted)

    def tick(self):
        self.retiring=[f for f in self.retiring if not f.done()]
        if self.active is None:return
        h,_,kind=self.active
        self.status_pub.publish(String(data=self.phase))
        if kind is DockRobot.Result:
            feedback=DockRobot.Feedback();feedback.state=feedback.NAV_TO_STAGING_POSE if self.phase=='STAGE' else feedback.INITIAL_PERCEPTION if self.phase=='ALIGN' else feedback.CONTROLLING
            h.publish_feedback(feedback)
        if h.is_cancel_requested:self.finish(False,'Cancelled');return
        if self.now()-self.started>self.p('docking_stage_timeout'):self.finish(False,'Dock sequence timeout');return
        if self.backend_handle or self.pending or self.retiring:return
        if self.phase=='STAGE':
            m=PoseStamped();m.header.frame_id='map';m.header.stamp=self.get_clock().now().to_msg();m.pose=self.target.approach_pose.pose
            self.submit(self.nav,NavigateToPose.Goal(pose=m,behavior_tree=str(Path(get_package_share_directory('control'))/'config/staging.xml')),'ALIGN')
        elif self.phase=='ALIGN':
            try:tr=self.buffer.lookup_transform('map','base_footprint',Time())
            except TransformException:return
            error=yaw(self.target.docking_pose.pose.orientation)-yaw(tr.transform.rotation);error=math.atan2(math.sin(error),math.cos(error))
            self.submit(self.spin,Spin.Goal(target_yaw=error,time_allowance=Duration(sec=int(self.p('docking_spin_timeout')))),'DOCK')
        elif self.phase=='DOCK':
            m=PoseStamped();m.header.frame_id='map';m.header.stamp=self.get_clock().now().to_msg();m.pose=self.target.docking_pose.pose
            self.submit(self.dock,DockRobot.Goal(use_dock_id=False,dock_type=f'dock_{self.ident}',dock_pose=m,navigate_to_staging_pose=False),'DONE')
        elif self.phase=='UNDOCK':self.submit(self.undock,h.request,'DONE')

def main():run(DockingAdapter)
