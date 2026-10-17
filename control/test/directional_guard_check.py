#!/usr/bin/env python3
"""Manual integration check of the real Nav2 guard, isolated from Gazebo/production ROS.
Run with ROS_DOMAIN_ID=92. Synthetic sensor/command probes are test fixtures only.
No belief input or collision-monitor parameter is changed. Tests the installed
VelocityPolygon implementation with toward/away commands and unchanged thresholds.
"""
import argparse,json,math,os,signal,subprocess,time
from pathlib import Path
import rclpy
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from ament_index_python.packages import get_package_prefix
from builtin_interfaces.msg import Time
from geometry_msgs.msg import Twist,TransformStamped
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan
from sensor_msgs_py.point_cloud2 import create_cloud_xyz32
from std_msgs.msg import Header
from lifecycle_msgs.srv import ChangeState
from lifecycle_msgs.msg import Transition
from tf2_ros import StaticTransformBroadcaster


def check(output):
    assert os.environ.get('ROS_DOMAIN_ID')=='92','Use isolated ROS_DOMAIN_ID=92'
    output=Path(output);output.mkdir(parents=True,exist_ok=True)
    config=Path(__file__).parents[1]/'config/control.yaml'
    executable=Path(get_package_prefix('nav2_collision_monitor'))/'lib/nav2_collision_monitor/collision_monitor'
    stream=(output/'guard.log').open('w')
    child=subprocess.Popen([str(executable),'--ros-args','--params-file',str(config)],stdout=stream,stderr=subprocess.STDOUT,start_new_session=True)
    rclpy.init();n=rclpy.create_node('control_guard_probe',parameter_overrides=[Parameter('use_sim_time',value=True)])
    clock=n.create_publisher(Clock,'/clock',10);scan=n.create_publisher(LaserScan,'/scan',qos_profile_sensor_data)
    from sensor_msgs.msg import PointCloud2
    depth=n.create_publisher(PointCloud2,'/control/depth_points',qos_profile_sensor_data)
    command=n.create_publisher(Twist,'/control/guard_input',10);received=[]
    n.create_subscription(Twist,'/cmd_vel',lambda m:received.append((m.linear.x,m.angular.z)),10)
    tf=StaticTransformBroadcaster(n);tr=TransformStamped();tr.header.frame_id='odom';tr.child_frame_id='base_footprint';tr.transform.rotation.w=1.;tf.sendTransform(tr)
    sim=1.
    def pump(seconds,lidar=2.,zdepth=2.,sensors=True,vx=.2,wz=0.):
        nonlocal sim
        end=time.monotonic()+seconds
        while time.monotonic()<end:
            sim+=.02;t=Time(sec=int(sim),nanosec=round((sim%1)*1e9));clock.publish(Clock(clock=t))
            if sensors:
                h=Header(stamp=t,frame_id='base_footprint')
                m=LaserScan(header=h,angle_min=0.,angle_max=2*math.pi,angle_increment=2*math.pi/359,range_min=.02,range_max=12.,ranges=[float(lidar)]+[math.nan]*359)
                scan.publish(m);depth.publish(create_cloud_xyz32(h,[[zdepth,0.,.15]]))
            m=Twist();m.linear.x=vx;m.angular.z=wz;command.publish(m)
            rclpy.spin_once(n,timeout_sec=.01);time.sleep(.01)
            if child.poll() is not None:raise RuntimeError('Guard process exited; inspect guard.log')
    try:
        client=n.create_client(ChangeState,'/collision_monitor/change_state')
        end=time.monotonic()+20
        while not client.service_is_ready():
            pump(.1)
            if time.monotonic()>end:raise TimeoutError('Guard lifecycle discovery')
        for transition in (Transition.TRANSITION_CONFIGURE,Transition.TRANSITION_ACTIVATE):
            future=client.call_async(ChangeState.Request(transition=Transition(id=transition)))
            end=time.monotonic()+10
            while not future.done():
                pump(.1)
                if time.monotonic()>end:raise TimeoutError('Guard transition')
            assert future.result().success
        report={}
        for name,lidar,zdepth,sensors,expected in [('clear',2.,2.,True,.2),('lidar_slow',.32,2.,True,.06),('lidar_stop',.20,2.,True,0.),('clear_between_stops',2.,2.,True,.2),('depth_stop',2.,.20,True,0.),('clear_again',2.,2.,True,.2),('stale_sources',2.,2.,False,0.)]:
            received.clear();pump(2.,lidar,zdepth,sensors)
            assert received,(name,'No final commands')
            assert all(abs(x-expected)<1e-6 and z==0. for x,z in received[-5:]),(name,received[-5:])
            report[name]={'samples':len(received),'last_linear_mps':received[-1][0]}
        for name,lidar,zdepth,vx,wz,expected in [
                ('front_reverse_away',.2,2.,-.1,0.,(-.1,0.)),
                ('rear_reverse_toward',2.,-.2,-.1,0.,(0.,0.)),
                ('rear_forward_away',2.,-.2,.2,0.,(.2,0.)),
                ('front_rotation_stop',.2,2.,0.,.2,(0.,0.))]:
            received.clear();pump(2.,lidar,zdepth,True,vx,wz)
            assert received and all(abs(x-expected[0])<1e-6 and abs(z-expected[1])<1e-6 for x,z in received[-5:]),(name,received[-5:])
            report[name]={'samples':len(received),'last_command':received[-1]}
        report['passed']=True;(output/'guard_check.json').write_text(json.dumps(report,indent=2)+'\n');return report
    finally:
        n.destroy_node();rclpy.shutdown();child.send_signal(signal.SIGINT)
        try:child.wait(timeout=5)
        except subprocess.TimeoutExpired:os.killpg(child.pid,signal.SIGTERM);child.wait(timeout=5)
        stream.close()


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('output');a=p.parse_args();print(json.dumps(check(a.output),indent=2))
