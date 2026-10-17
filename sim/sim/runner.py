"""N seeded headless runs: isolated Saga, Gazebo, ROS bag, CSV and bounded cleanup."""
import argparse
import csv
import json
import math
import os
from pathlib import Path
import secrets
import shlex
import signal
import sys
import uuid
import subprocess
import time
import yaml
import rclpy
from rclpy.parameter import Parameter
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import Bool
from nav2_msgs.srv import ManageLifecycleNodes
from interfaces.msg import RobotTruthArray, SimRouteEvent, DockMap
from .ros_support import SimNode, defaults, latched
from .network import assert_port_available
from .discovery import write_discovery_profile


BAG_TOPICS=['/oak/stereo/image_raw','/perception/camera_tracks','/rosout','/explore/status','/explore/frontiers','/control/depth_points','/local_costmap/published_footprint','/control/stop_polygon','/plan','/behavior_tree_log','/control/nav2_cmd','/navigate_to_pose/_action/status','/spin/_action/status','/oak/rgb/camera_info','/clock','/tf','/tf_static','/odom','/scan','/slam/scan','/initialpose','/amcl_pose','/pose','/cmd_vel','/sim/truth','/sim/map',
            '/sim/collisions','/sim/minimum_separation','/sim/routes','/sim/detections',
            '/sim/forced_collision','/perception/predictions','/perception/health',
            '/perception/dock_states','/perception/understanding','/planning/trajectory',
            '/planning/saga_state','/planning/time_ms','/control/request','/control/status']


class ExperimentRunner(SimNode):
    def __init__(self,scenario,output,workspace,mission_source=None,mission_python=None,gazebo_environment=None):
        params=dict(scenario.get('parameters',{}))
        params.update(scenario.get('features',{}))
        unknown=params.keys()-defaults().keys()
        if unknown:
            raise ValueError('Unsupported scenario parameters/features: '+str(sorted(unknown)))
        super().__init__('sim_experiment', [Parameter(k,value=v) for k,v in params.items()])
        self.scenario=scenario; self.output=Path(output).resolve(); self.workspace=Path(workspace).resolve()
        self.mission_source=Path(mission_source).resolve() if mission_source else None
        self.mission_python=mission_python or sys.executable
        self.gazebo_environment=Path(gazebo_environment).resolve() if gazebo_environment else self.workspace/'env.sh'
        self.directory=None
        self.map_ready=False
        self.create_subscription(DockMap,'/slam/docks',lambda m:setattr(self,'map_ready',m.ready),latched())
        self.children=[]; self.streams=[]; self.truth=None; self.route_seen=False
        self.create_subscription(RobotTruthArray,'/sim/truth',self.on_truth,self.p('qos_depth'))
        self.create_subscription(SimRouteEvent,'/sim/routes',self.on_route,self.p('qos_depth'))
        self.force_pub=self.create_publisher(Bool,'/sim/forced_collision',self.p('qos_depth'))
        # Stop consumers/update threads before removing their map/TF providers.
        self.lifecycle_shutdown=[(name,self.create_client(ManageLifecycleNodes,name+'/manage_nodes'))
            for name in ('/control_lifecycle','/planning_costmap_lifecycle','/slam_lifecycle_manager')]

    def on_truth(self,message):
        self.truth=message

    def on_route(self,message):
        self.route_seen=True

    def spawn(self,command,name,directory,environment):
        stream=(directory/(name+'.log')).open('w'); self.streams.append(stream)
        child=subprocess.Popen(command,stdout=stream,stderr=subprocess.STDOUT,env=environment,
                               cwd=directory,start_new_session=True)
        self.children.append((name,child)); return child

    def spin_until(self,predicate,timeout):
        end=time.monotonic()+timeout
        while not predicate():
            if time.monotonic()>=end:
                raise TimeoutError('Simulation readiness/run watchdog expired; inspect run logs')
            stack_log=self.directory/'stack.log' if self.directory else None
            if stack_log and stack_log.exists():
                content=stack_log.read_text(errors='replace')
                if 'Failed to bring up all requested nodes' in content or 'Traceback (most recent call last)' in content:
                    raise RuntimeError('Required stack startup failed; inspect stack.log')
            failed=[name for name,p in self.children if p.poll() is not None]
            if failed:
                raise RuntimeError('Child exited: '+', '.join(failed))
            rclpy.spin_once(self,timeout_sec=self.p('runner_poll_period'))

    def stop(self):
        # Use installed lifecycle transitions while ROS, map, TF and /clock live.
        if self.children and rclpy.ok():
            for name,client in self.lifecycle_shutdown:
                if not client.service_is_ready():continue
                future=client.call_async(ManageLifecycleNodes.Request(command=ManageLifecycleNodes.Request.SHUTDOWN))
                deadline=time.monotonic()+self.p('shutdown_timeout')
                while not future.done() and time.monotonic()<deadline:
                    rclpy.spin_once(self,timeout_sec=self.p('runner_poll_period'))
                ok=future.done() and future.result() is not None and future.result().success
                self.get_logger().info(f'Lifecycle shutdown {name}: success={ok}')
        # Stop support, flush bags, then stop Gazebo and Saga; each group is owned by this runner.
        for name,child in reversed(self.children):
            if child.poll() is None:
                child.send_signal(signal.SIGINT)
                try:
                    child.wait(timeout=self.p('shutdown_timeout'))
                except subprocess.TimeoutExpired:
                    os.killpg(child.pid,signal.SIGTERM)
                    try:
                        child.wait(timeout=self.p('shutdown_timeout'))
                    except subprocess.TimeoutExpired:
                        os.killpg(child.pid,signal.SIGKILL); child.wait()
            # Ruby/gz may fork a server that outlives the launcher. Reap the whole process group.
            try:
                os.killpg(child.pid,signal.SIGTERM)
            except ProcessLookupError:
                pass
        self.children.clear()
        for stream in self.streams:
            stream.close()
        self.streams.clear()

    def teleport(self,pose):
        from gz.transport13 import Node
        from gz.msgs10.pose_pb2 import Pose
        from gz.msgs10.boolean_pb2 import Boolean
        request=Pose(); request.name='traffic_1'
        request.position.x=pose.position.x; request.position.y=pose.position.y; request.position.z=pose.position.z
        request.orientation.x=pose.orientation.x; request.orientation.y=pose.orientation.y
        request.orientation.z=pose.orientation.z; request.orientation.w=pose.orientation.w
        ok,response=Node().request('/world/arena/set_pose',request,Pose,Boolean,int(self.p('gz_request_timeout')*1000))
        if not ok or not response.data:
            raise RuntimeError('Forced-contact teleport service failed')

    def run_one(self,index):
        directory=self.output/f'run_{index:03d}'
        directory.mkdir(parents=True,exist_ok=False)
        self.directory=directory
        params={key:self.p(key) for key in defaults()}
        params['run_dir']=str(directory)
        params['saga_url']=f'http://127.0.0.1:{self.p("saga_port")}'
        # Repetitions use the SAME seed for repeatability; multi-seed sweeps supply separate scenarios.
        (directory/'params.yaml').write_text(yaml.safe_dump({'/**':{'ros__parameters':params}}))
        (directory/'scenario.yaml').write_text(yaml.safe_dump(self.scenario))
        environment=os.environ.copy(); environment['ROS_AUTOMATIC_DISCOVERY_RANGE']='LOCALHOST'
        profile=directory/'discovery.xml'
        write_discovery_profile(profile,int(environment.get('ROS_DOMAIN_ID','0')),self.p('discovery_participant_capacity'))
        environment['FASTRTPS_DEFAULT_PROFILES_FILE']=str(profile)
        environment['FASTDDS_DEFAULT_PROFILES_FILE']=str(profile)
        environment['PYTHONDONTWRITEBYTECODE']='1'; environment['PYTHONNOUSERSITE']='1'
        environment['PATH']='/usr/bin:/bin:'+environment['PATH']
        environment['GZ_PARTITION']='sim_'+uuid.uuid4().hex
        os.environ['GZ_PARTITION']=environment['GZ_PARTITION']
        for key in ['__NV_PRIME_RENDER_OFFLOAD','__GLX_VENDOR_LIBRARY_NAME','__EGL_VENDOR_LIBRARY_FILENAMES']:
            environment.pop(key,None)
        environment['AUTONOMY_SIM_ADMIN_USER']='admin'; environment['AUTONOMY_SIM_ADMIN_PASSWORD']=secrets.token_urlsafe(24)
        for i in range(1,5):
            environment[f'AUTONOMY_SIM_TOKEN_{i}']=secrets.token_urlsafe(24)
        # Credentials exist in runtime environment/ROS launch parameters only, never durable run config or bags.
        port=self.p('saga_port')
        assert_port_available(port) # Fail before touching an existing local server.
        if self.mission_source is None or not self.mission_source.is_dir():
            raise RuntimeError('Set --mission-source to an external compatible mission-server checkout')
        if not self.gazebo_environment.is_file():
            raise RuntimeError('Set --gazebo-environment to the simulator environment script')
        self.spawn([str(self.mission_python),'-m','sim.saga_server','--source',str(self.mission_source),
                    '--run-dir',str(directory),'--seed',str(self.p('route_seed')),'--port',str(port)],'saga',directory,environment)
        self.spawn(['bash','--noprofile','--norc','-c','source "$1"; shift; exec "$@"','gazebo-terminal',
                    str(self.gazebo_environment),'ros2','launch','sim','world.launch.py',
                    'run_dir:='+str(directory),'config:='+str(directory/'params.yaml'),
                    'traffic_count:='+str(self.p('traffic_count')),'headless:='+str(self.p('headless')).lower()],'world',directory,environment)
        self.spin_until(lambda:(directory/'map.npz').exists(),self.p('startup_timeout'))
        # Bag before support to capture initial organiser transitions. Do not record /parameter_events.
        topics=BAG_TOPICS+['/global_costmap/costmap','/control/guard_input','/control/behavior_cmd','/backup/_action/status','/wait/_action/status','/map','/slam/docks','/slam/pose','/slam/status','/planning/path','/planning/behaviour','/planning/risk','/perception/diagnostics','/perception/dock_observations','/control/collision_state','/visualization/robots','/visualization/predictions']+(['/perception/image/compressed'] if self.p('record_images') else [])+[f'/traffic_{i}/'+topic for i in range(1,self.p('traffic_count')+1) for topic in ('cmd_vel','odom')]
        self.spawn(['ros2','bag','record','--use-sim-time','-o',str(directory/'bag'),*topics],'bag',directory,environment)
        self.spawn(['ros2','launch','sim','sim.launch.py','run_dir:='+str(directory),
                    'config:='+str(directory/'params.yaml'),'traffic_count:='+str(self.p('traffic_count')),
                    'route_seed:='+str(self.p('route_seed'))],'support',directory,environment)
        if self.p('stack_stage')!='phase1':
            self.spawn(['ros2','launch','sim','stack.launch.py','config:='+str(directory/'params.yaml')],'stack',directory,environment)
        self.truth=None; self.route_seen=False; self.map_ready=False
        wall_start=time.monotonic()
        self.spin_until(lambda:self.truth is not None and len(self.truth.robots)==self.p('traffic_count')+1 and self.route_seen,
                        self.p('startup_timeout'))
        expected=0 if self.p('stack_stage')=='phase1' else 1
        self.spin_until(lambda:self.count_publishers('/cmd_vel')==expected,self.p('full_stack_startup_timeout'))
        start=self.now(); wall_start=time.monotonic()
        hz_stream=(directory/'scan_hz.log').open('w'); self.streams.append(hz_stream)
        hz=subprocess.Popen(['ros2','topic','hz','/scan'],stdout=hz_stream,stderr=subprocess.STDOUT,
                            env=environment,start_new_session=True)
        self.children.append(('scan_hz',hz))
        forced=False; restored=False; saved=None; force_time=None
        while self.now()-start<self.p('duration'):
            rclpy.spin_once(self,timeout_sec=self.p('runner_poll_period'))
            if time.monotonic()-wall_start>self.p('run_wall_timeout'):
                raise TimeoutError('Run exceeded wall watchdog before sim duration')
            if hz.poll() is None and time.monotonic()-wall_start>=self.p('scan_sample_seconds'):
                os.killpg(hz.pid,signal.SIGINT); hz.wait(timeout=self.p('shutdown_timeout'))
                self.children=[(n,p) for n,p in self.children if p is not hz]
            failed=[n for n,p in self.children if p.poll() is not None]
            if failed:
                raise RuntimeError('Child exited: '+str(failed))
            if self.p('stop_when_map_ready') and self.p('stack_stage')=='E0' and self.map_ready and (directory/'slam_snapshot.json').is_file():
                # Evaluation-only physical continuity: localisation never reads this truth.
                ego=next(r for r in self.truth.robots if r.robot_id==0).pose
                q=ego.orientation
                angle=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
                (directory/'sim_handover.json').write_text(json.dumps({'world_pose':[ego.position.x,ego.position.y,ego.position.z,0.,0.,angle]},indent=2)+'\n')
                break
            elapsed=self.now()-start
            if not forced and 0<=self.p('force_collision_at')<=elapsed:
                robots={r.robot_id:r for r in self.truth.robots}
                if 1 not in robots:
                    raise ValueError('Forced contact needs traffic_count>=1')
                saved=robots[1].pose; self.force_pub.publish(Bool(data=True))
                # Flush marker before set_pose so collisions are visibly labelled forced.
                rclpy.spin_once(self,timeout_sec=self.p('runner_poll_period'))
                self.teleport(robots[0].pose); forced=True; force_time=self.now()
            if forced and not restored and self.now()-force_time>=self.p('forced_hold_duration'):
                self.teleport(saved); restored=True; self.force_pub.publish(Bool(data=False))
        metadata={'duration_sim_s':self.now()-start,'duration_wall_s':time.monotonic()-wall_start,
                  'real_time_factor':(self.now()-start)/(time.monotonic()-wall_start),
                  'route_seed':self.p('route_seed'),'traffic_count':self.p('traffic_count'),'forced_test':forced,
                  'stack_stage':self.p('stack_stage'),'map_source':self.p('map_source'),'start_sim_s':start,'gz_partition':environment['GZ_PARTITION']}
        self.stop()
        # Verify recorder finalised its index and the collector wrote actual rows.
        if not (directory/'bag/metadata.yaml').exists():
            raise RuntimeError('Bag metadata missing after shutdown')
        rows=list(csv.DictReader((directory/'metrics.csv').open()))
        if not rows:
            raise RuntimeError('No metrics rows')
        metadata['last_metrics']=rows[-1]
        (directory/'result.json').write_text(json.dumps(metadata,indent=2))
        print(f'Run {index}: {metadata["duration_sim_s"]:.1f}s sim, RTF={metadata["real_time_factor"]:.3f}; {directory}',flush=True)
        return metadata


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--scenario',required=True); parser.add_argument('--output-dir',required=True)
    parser.add_argument('--workspace',default=str(Path.cwd()))
    parser.add_argument('--mission-source',default=os.environ.get('AUTONOMY_MISSION_SOURCE',''))
    parser.add_argument('--mission-python',default=sys.executable)
    parser.add_argument('--gazebo-environment',default='')
    args,ros_args=parser.parse_known_args()
    scenario=yaml.safe_load(Path(args.scenario).read_text())
    os.environ.setdefault('FASTDDS_BUILTIN_TRANSPORTS','UDPv4')
    # Keep the ROS context alive through ordered lifecycle cleanup.
    rclpy.init(args=ros_args,signal_handler_options=SignalHandlerOptions.NO)
    node=ExperimentRunner(scenario,args.output_dir,args.workspace,args.mission_source,args.mission_python,
                          args.gazebo_environment or None)
    try:
        results=[]
        for index in range(node.p('repetitions')):
            results.append(node.run_one(index))
        (Path(args.output_dir)/'results.json').write_text(json.dumps(results,indent=2))
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted; preserving partial recording and stopping owned processes')
        raise SystemExit(130)
    finally:
        node.stop(); node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__=='__main__':
    main()
