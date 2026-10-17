import rclpy
from control.docking_adapter import DockingAdapter

def test_real_docking_adapter_initializes_action_endpoints():
    rclpy.init();node=DockingAdapter()
    try:
        assert node.phase=='IDLE' and node.active is None
        from nav2_msgs.action import DockRobot,UndockRobot
        assert node.server.action_type is DockRobot
        assert node.unserver.action_type is UndockRobot
        assert node.count_publishers('/cmd_vel')==0
    finally:node.destroy_node();rclpy.shutdown()


def test_docking_stages_aligns_then_calls_installed_action_and_undocks():
    import time
    from rclpy.node import Node
    from rclpy.action import ActionClient,ActionServer
    from rclpy.executors import SingleThreadedExecutor
    from nav2_msgs.action import NavigateToPose,Spin,DockRobot,UndockRobot
    from interfaces.msg import DockMap
    from geometry_msgs.msg import TransformStamped
    rclpy.init(args=['--ros-args','-p','navigation_watch_period:=0.01'])
    node=DockingAdapter();backend=Node('fake_package_actions');probe=Node('dock_sequence_probe')
    node.docks=DockMap(ready=True);target=node.docks.docks[3];target.valid=True
    target.approach_pose.pose.position.x=1.;target.approach_pose.pose.orientation.w=1.
    target.docking_pose.pose.position.x=1.48;target.docking_pose.pose.orientation.w=1.
    tr=TransformStamped();tr.header.frame_id='map';tr.child_frame_id='base_footprint';tr.transform.rotation.w=1.
    node.buffer.set_transform_static(tr,'test')
    calls=[];servers=[]
    def serve(kind,name):
        def execute(h):
            calls.append((name,h.request));h.succeed();result=kind.Result()
            if hasattr(result,'success'):result.success=True
            return result
        servers.append(ActionServer(backend,kind,name,execute_callback=execute))
    for kind,name in [(NavigateToPose,'/navigate_to_pose'),(Spin,'/spin'),(DockRobot,'/nav2/dock_robot'),(UndockRobot,'/nav2/undock_robot')]:serve(kind,name)
    executor=SingleThreadedExecutor()
    for n in (node,backend,probe):executor.add_node(n)
    def until(f):
        deadline=time.monotonic()+5
        while not f() and time.monotonic()<deadline:executor.spin_once(timeout_sec=.01)
        assert f()
    try:
        for kind,name,goal in [(DockRobot,'/dock_robot',DockRobot.Goal(use_dock_id=True,dock_id='3')),(UndockRobot,'/undock_robot',UndockRobot.Goal(dock_type='dock_3'))]:
            client=ActionClient(probe,kind,name);until(client.server_is_ready)
            ack=client.send_goal_async(goal);until(ack.done);assert ack.result().accepted
            result=ack.result().get_result_async();until(result.done)
            assert result.result().status==4 and result.result().result.success
        assert [c[0] for c in calls]==['/navigate_to_pose','/spin','/nav2/dock_robot','/nav2/undock_robot']
        assert calls[0][1].behavior_tree.endswith('staging.xml')
        assert calls[0][1].pose.pose.position.x==1.
        assert not calls[2][1].navigate_to_staging_pose and calls[2][1].dock_type=='dock_3'
        assert calls[2][1].dock_pose.pose.position.x==1.48
    finally:
        executor.shutdown()
        for server in servers:server.destroy()
        for n in (node,backend,probe):n.destroy_node()
        rclpy.shutdown()
