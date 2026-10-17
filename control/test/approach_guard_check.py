#!/usr/bin/env python3
"""Real installed approach guard, isolated ROS domain 92; no Gazebo or hardware.

Fixture footprint reproduces the unchanged local-costmap rectangle plus padding.
Contact return at x=0.12596 m represents the model's front wheel outline.
Test fixtures never modify production guard parameters. Collect all results,
including failures, before exiting; no repeated simulation runs or tuning.
"""
import argparse, ast, json, math, os, signal, subprocess, time
from pathlib import Path
import yaml
import rclpy
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from ament_index_python.packages import get_package_prefix
from builtin_interfaces.msg import Time
from geometry_msgs.msg import Twist, TransformStamped, PolygonStamped, Point32
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan, PointCloud2
from sensor_msgs_py.point_cloud2 import create_cloud_xyz32
from std_msgs.msg import Header
from lifecycle_msgs.srv import ChangeState
from lifecycle_msgs.msg import Transition
from nav2_msgs.msg import CollisionMonitorState
from tf2_ros import StaticTransformBroadcaster


def check(output):
    assert os.environ.get('ROS_DOMAIN_ID') == '92', 'Use isolated ROS_DOMAIN_ID=92'
    output = Path(output); output.mkdir(parents=True, exist_ok=True)
    config = Path(__file__).parents[1]/'config/control.yaml'
    params = yaml.safe_load(config.read_text())
    local = params['local_costmap']['local_costmap']['ros__parameters']
    padding = local['footprint_padding']
    footprint = [[x+math.copysign(padding,x), y+math.copysign(padding,y)]
                 for x,y in ast.literal_eval(local['footprint'])]
    executable = Path(get_package_prefix('nav2_collision_monitor'))/'lib/nav2_collision_monitor/collision_monitor'
    assert str(executable).startswith('/opt/ros/jazzy/')
    stream = (output/'guard.log').open('w')
    child = subprocess.Popen([str(executable),'--ros-args','--params-file',str(config)],
                             stdout=stream,stderr=subprocess.STDOUT,start_new_session=True)
    rclpy.init()
    n = rclpy.create_node('approach_guard_probe',parameter_overrides=[Parameter('use_sim_time',value=True)])
    clock = n.create_publisher(Clock,'/clock',10)
    scan = n.create_publisher(LaserScan,'/scan',qos_profile_sensor_data)
    depth = n.create_publisher(PointCloud2,'/control/depth_points',qos_profile_sensor_data)
    poly = n.create_publisher(PolygonStamped,'/local_costmap/published_footprint',10)
    command = n.create_publisher(Twist,'/control/guard_input',10)
    received=[]; states=[]
    n.create_subscription(Twist,'/cmd_vel',lambda m:received.append((m.linear.x,m.angular.z)),10)
    n.create_subscription(CollisionMonitorState,'/control/collision_state',
                          lambda m:states.append({'action_type':m.action_type,'polygon_name':m.polygon_name}),10)
    tf = StaticTransformBroadcaster(n); tr = TransformStamped()
    tr.header.frame_id='odom'; tr.child_frame_id='base_footprint'; tr.transform.rotation.w=1.; tf.sendTransform(tr)
    sim=1.

    def pump(duration, lidar=(2.,0.), point=(2.,0.,.15), sensors=True, vx=.2):
        nonlocal sim
        end=time.monotonic()+duration
        while time.monotonic()<end:
            sim+=.02; ns=round(sim*1e9); stamp=Time(sec=ns//10**9,nanosec=ns%10**9)
            clock.publish(Clock(clock=stamp)); header=Header(stamp=stamp,frame_id='base_footprint')
            p=PolygonStamped(header=header); p.polygon.points=[Point32(x=x,y=y) for x,y in footprint]; poly.publish(p)
            if sensors:
                angle=math.atan2(lidar[1],lidar[0]); distance=math.hypot(*lidar)
                scan.publish(LaserScan(header=header,angle_min=angle,angle_max=angle+2*math.pi,
                    angle_increment=2*math.pi/359,range_min=.02,range_max=12.,ranges=[distance]+[math.nan]*359))
                depth.publish(create_cloud_xyz32(header,[list(point)]))
            m=Twist(); m.linear.x=vx; command.publish(m)
            rclpy.spin_once(n,timeout_sec=.01); time.sleep(.01)
            if child.poll() is not None: raise RuntimeError('Guard exited; inspect guard.log')

    try:
        client=n.create_client(ChangeState,'/collision_monitor/change_state')
        end=time.monotonic()+20
        while not client.service_is_ready():
            pump(.1)
            if time.monotonic()>end: raise TimeoutError('Guard lifecycle discovery')
        for transition in (Transition.TRANSITION_CONFIGURE,Transition.TRANSITION_ACTIVATE):
            future=client.call_async(ChangeState.Request(transition=Transition(id=transition)))
            end=time.monotonic()+10
            while not future.done():
                pump(.1)
                if time.monotonic()>end: raise TimeoutError('Guard transition')
            assert future.result().success
        # Each case permits transient transport settling; assess its final 5 commands.
        cases=[
            ('clear', (2.,0.),(2.,0.,.15),True,.2,'full'),
            ('front_near_toward',(.2,0.),(2.,0.,.15),True,.2,'reduced'),
            ('front_near_away',(.2,0.),(2.,0.,.15),True,-.1,'moving'),
            ('rear_near_toward',(2.,0.),(-.2,0.,.15),True,-.1,'reduced'),
            ('rear_near_away',(2.,0.),(-.2,0.,.15),True,.2,'moving'),
            ('floor_excluded',(2.,0.),(.12596,0.,0.),True,.2,'full'),
            ('front_contact_toward',(.12596,.10),(2.,0.,.15),True,.2,'stop'),
            ('front_contact_away',(.12596,.10),(2.,0.,.15),True,-.1,'moving'),
            ('rear_contact_toward',(2.,0.),(-.12596,.10,.15),True,-.1,'stop'),
            ('rear_contact_away',(2.,0.),(-.12596,.10,.15),True,.2,'moving'),
            ('released',(2.,0.),(2.,0.,.15),True,.2,'full'),
            ('sensor_loss',(2.,0.),(2.,0.,.15),False,.2,'stop')]
        report={'executable':str(executable),'footprint_m':footprint,
                'time_before_collision_s':params['collision_monitor']['ros__parameters']['Approach']['time_before_collision'],
                'cases':{}}
        for name,lidar,point,sensors,vx,expected in cases:
            received.clear(); pump(1.2,lidar,point,sensors,vx)
            def acceptable(cmd):
                x,w=cmd
                if abs(w)>1e-8:return False
                if expected=='stop':return abs(x)<1e-8
                if expected=='full':return abs(x-vx)<1e-8
                if expected=='reduced':return 0<=x/vx<1
                return 0<x/vx<=1.000001
            tail=received[-5:]
            report['cases'][name]={'lidar_point_m':lidar,'depth_point_m':point,
                'input_linear_mps':vx,'expected':expected,'output_tail':tail,
                'last_state':states[-1] if states else None,
                'passed':len(tail)==5 and all(acceptable(c) for c in tail)}
        report['passed']=all(c['passed'] for c in report['cases'].values())
        (output/'guard_check.json').write_text(json.dumps(report,indent=2)+'\n')
        return report
    finally:
        command.publish(Twist()); n.destroy_node(); rclpy.shutdown()
        child.send_signal(signal.SIGINT)
        try:child.wait(timeout=5)
        except subprocess.TimeoutExpired:os.killpg(child.pid,signal.SIGTERM);child.wait(timeout=5)
        stream.close()


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('output');args=parser.parse_args()
    result=check(args.output);print(json.dumps(result,indent=2))
    raise SystemExit(0 if result['passed'] else 1)
