"""Safety handshakes must fail closed, including a revoked goal during escape."""
import rclpy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from control.escape_node import EscapeNode


def test_revoked_goal_restores_approach_without_escape_motion():
    rclpy.init();node=EscapeNode()
    try:
        node.state='PIVOT';node.started=node.now();node.active=False
        node.tick()
        assert node.state=='RESTORE'
        assert node.command_out.linear.x==node.command_out.angular.z==0.
        assert node.count_publishers('/cmd_vel')==0
    finally:node.destroy_node();rclpy.shutdown()


def test_missing_scan_and_stale_odom_cannot_move_during_escape():
    rclpy.init();node=EscapeNode()
    try:
        node.state='PIVOT';node.started=node.now();node.navigation(Bool(data=True))
        node.odom=Odometry();node.odom.header.stamp=node.get_clock().now().to_msg()
        node.tick();assert node.command_out.linear.x==node.command_out.angular.z==0.
    finally:node.destroy_node();rclpy.shutdown()


def test_navigation_prehold_preserves_stop_exit_room_but_final_docking_unchanged():
    import numpy as np
    rclpy.init();node=EscapeNode()
    class Publisher:
        def publish(self,m):self.last=m
    actual=node.pub;node.pub=Publisher()
    try:
        node.points=lambda:np.array([[.23,0.],[-2.,0.]])
        node.navigation(Bool(data=True))
        forward=Twist();forward.linear.x=.15
        node.normal_command(forward)
        assert node.preheld and node.pub.last.linear.x==0.
        reverse=Twist();reverse.linear.x=-.25
        node.normal_command(reverse)
        assert not node.preheld and node.pub.last.linear.x==-.25
        node.navigation(Bool(data=False)) # Final DockRobot approach has no active NavigateToPose.
        node.normal_command(forward)
        assert node.pub.last.linear.x==.15
    finally:node.pub=actual;node.destroy_node();rclpy.shutdown()


def test_pivot_to_creep_gap_uses_guarded_backup_instead_of_oscillating_nav2():
    import numpy as np
    rclpy.init();node=EscapeNode()
    try:
        node.state='PIVOT';node.started=node.now();node.navigation(Bool(data=True))
        node.odom=Odometry();node.odom.header.stamp=node.get_clock().now().to_msg()
        node.points=lambda:np.array([[.55,0.],[-2.,0.]]) # .424m beyond front: pivot done, creep not admitted.
        node.tick()
        assert node.state=='BACKUP'
        assert node.command_out.linear.x==node.command_out.angular.z==0.
    finally:node.destroy_node();rclpy.shutdown()


def test_accumulated_prestop_survives_goal_gap_zero_commands_and_small_motion():
    rclpy.init();node=EscapeNode()
    try:
        node.odom=Odometry();node.prehold_latched=True
        node.odom.twist.twist.angular.z=.8 # Shim rotation does not translate out of the physical trap.
        # Two 3s holds separated by a 3s cancelled/replaced-goal gap (~6s resend).
        fired=[]
        for i in range(101):
            t=i*.1;active=t<3 or t>=6
            node.odom.pose.pose.position.x=.02 if t>=6 else 0.
            if node.held_motion(t,True,active,False,True):fired.append(t)
        assert fired and 8.<fired[0]<8.3
        # A real >5cm displacement ends this physical intent.
        node.odom.pose.pose.position.x=.051
        assert not node.held_motion(10.1,True,True,False,True)
        assert node.held_seconds==0. and not node.prehold_latched
    finally:node.destroy_node();rclpy.shutdown()


def test_stale_sensors_or_inactive_goal_cannot_trigger_saved_hold_credit():
    rclpy.init();node=EscapeNode()
    try:
        node.odom=Odometry();node.prehold_latched=True;node.held_seconds=6.
        assert not node.held_motion(1.,False,True,False,True)
        assert not node.held_motion(1.1,True,False,False,True)
        assert node.held_motion(1.2,True,True,False,True)
    finally:node.destroy_node();rclpy.shutdown()
