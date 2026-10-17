"""Hardware map/dock/full profiles; use_sim_time=true is a laptop verification mode."""
from pathlib import Path
import os
import shlex
import tempfile
import yaml
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetEnvironmentVariable, LogInfo, RegisterEventHandler, EmitEvent
from launch.substitutions import LaunchConfiguration
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch_ros.actions import Node
from sim.discovery import write_discovery_profile
from sim.hardware import service_environment, profile_configurations


def start(context):
    value=lambda key:LaunchConfiguration(key).perform(context)
    share=lambda package:Path(get_package_share_directory(package))
    defaults=yaml.safe_load(Path(value('config')).read_text())['/**']['ros__parameters']
    profile=value('profile');profile={'competition':'full'}.get(profile,profile);sim_time=value('use_sim_time').lower()=='true'
    directory=Path(value('run_dir')).expanduser() if value('run_dir') else Path(tempfile.mkdtemp(prefix='autonomy_hardware_'))
    directory.mkdir(parents=True,exist_ok=True)
    environment=service_environment(value('service_config') or defaults['service_config'])
    for package in ('nav2_planner','nav2_smac_planner','nav2_bt_navigator','nav2_controller','nav2_behaviors',
                    'opennav_docking','nav2_collision_monitor','nav2_amcl','nav2_map_server','nav2_costmap_2d','nav2_lifecycle_manager'):
        if get_package_prefix(package)!='/opt/ros/jazzy':raise RuntimeError(f'{package} must resolve to /opt/ros/jazzy')
    dds=directory/'fastdds.xml'
    write_discovery_profile(dds,int(environment['ROS_DOMAIN_ID']),defaults['discovery_capacity'])
    environment.update(ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST',FASTDDS_DEFAULT_PROFILES_FILE=str(dds),FASTRTPS_DEFAULT_PROFILES_FILE=str(dds),
        OMP_NUM_THREADS='1',OPENBLAS_NUM_THREADS='1',MKL_NUM_THREADS='1')
    actions=[SetEnvironmentVariable(k,v) for k,v in environment.items()]
    snapshot=str(Path(value('snapshot')).expanduser()) if value('snapshot') else ''
    if profile in ('full','competition_lite') and not snapshot:
        raise RuntimeError('Competition requires a completed map snapshot; run profile:=map first')
    if profile in ('full','competition_lite'):
        from slam.snapshot import load_snapshot
        if load_snapshot(snapshot).get('partial',False):raise RuntimeError('Competition cannot use a partial mapping snapshot')
    if profile in ('full','competition_lite') and snapshot and not os.environ.get('AUTONOMY_SAGA_TOKEN'):
        raise RuntimeError('Full saved-map missions require AUTONOMY_SAGA_TOKEN')
    save=Path(value('save_snapshot') or defaults['save_snapshot']).expanduser();save.parent.mkdir(parents=True,exist_ok=True)
    if value('tag_search') not in ('coverage','off'):raise ValueError('tag_search must be coverage or off')
    configs,lifecycle=profile_configurations(share,directory,profile,sim_time,snapshot,str(save),value('saga_url'),value('tag_search'))
    common={'use_sim_time':sim_time}
    def node(package,executable,name=None,parameters=(),core=False,perception=False,**kwargs):
        process=Node(package=package,executable=executable,name=name,parameters=[*parameters,common],output='screen',
            respawn=perception,respawn_delay=3.0 if perception else 0.0,**kwargs)
        if core:
            actions.append(RegisterEventHandler(OnProcessExit(target_action=process,
                on_exit=[EmitEvent(event=Shutdown(reason=f'Core process {name or executable} exited'))])))
        actions.append(process)
        return process
    def manager(name,managed):
        node('nav2_lifecycle_manager','lifecycle_manager',name,[dict(lifecycle,node_names=managed)],core=True)
    actions.append(LogInfo(msg=f'Profile={profile}; configs={directory}; UDPv4; camera clock must be configured in the base service'))
    if not sim_time:
        node('slam','slam_wheel_odometry_node',parameters=[value('odometry_config') or str(share('slam')/'config/wheel_odometry.yaml')],core=True)
    managed=[]
    if snapshot:
        node('nav2_map_server','map_server','map_server',[configs['slam_backend']],core=True)
        node('nav2_amcl','amcl','amcl',[configs['slam_backend']],core=True);managed=['map_server','amcl']
    else:
        node('slam_toolbox','async_slam_toolbox_node','slam_toolbox',[configs['slam_backend']],core=True)
        node('nav2_map_server','map_saver_server','map_saver',[configs['slam_backend']],core=True);managed=['slam_toolbox','map_saver']
    node('slam','slam_slam_node',parameters=[configs['slam']])
    manager('slam_lifecycle_manager',managed)
    node('nav2_planner','planner_server','planner_server',[configs['planning']],core=True)
    manager('planning_costmap_lifecycle',['planner_server'])
    for package,executable in [('nav2_controller','controller_server'),('nav2_behaviors','behavior_server'),
                               ('nav2_bt_navigator','bt_navigator'),('nav2_collision_monitor','collision_monitor')]:
        node(package,executable,executable,[configs['control']],core=True,
             remappings=[('cmd_vel','/control/nav2_cmd')] if executable in ('controller_server','behavior_server') else [('navigate_to_pose','/nav2/navigate_to_pose')] if executable=='bt_navigator' else [])
    node('opennav_docking','opennav_docking','docking_server',[configs['docking']],core=True,remappings=[('cmd_vel','/control/nav2_cmd'),('dock_robot','/nav2/dock_robot'),('undock_robot','/nav2/undock_robot')])
    node('control','control_docking_adapter',parameters=[configs['control']],core=True)
    manager('control_lifecycle',['controller_server','behavior_server','collision_monitor','bt_navigator','docking_server'])
    node('control','control_coordinator_node' if profile in ('full','competition_lite') else 'control_direct_command_node',parameters=[configs['control']],remappings=[('/control/guard_input','/control/normal_cmd')])
    node('control','control_escape_node',parameters=[str(share('control')/'config/escape.yaml')],core=True)
    node('control','control_navigation_relay',parameters=[configs['control']],core=True)
    if not snapshot and profile=='map':
        node('planning','planning_coverage_node',parameters=[configs['coverage'],*([value('coverage_config')] if value('coverage_config') else []),{'tag_search':value('tag_search')}])
        node('explore_lite','explore',parameters=[str(share('planning')/'config/explore.yaml')],remappings=[('navigate_to_pose','/exploration/navigate_to_pose')])
    if profile in ('full','competition_lite'):
        node('planning','planning_planning_node',parameters=[configs['planning'],{'saga_token':os.environ.get('AUTONOMY_SAGA_TOKEN','')}])
    if profile=='full':
        node('control','control_depth_node',parameters=[configs['control']])
        for executable in ('perception_perception_node','perception_dock_node'):
            node('perception',executable,parameters=[configs['perception'],{'particles_per_robot':int(value('particles_per_robot'))}],perception=True)
        node('perception','perception_detector_node',parameters=[value('detector_config') or str(share('perception')/'config/detector.yaml'),{'backend':value('detector_backend')}],
             perception=True,prefix=shlex.quote(value('detector_python') or defaults['detector_python']))
    if value('foxglove').lower()=='true':
        node('foxglove_bridge','foxglove_bridge',parameters=[{'port':defaults['foxglove_port'],'address':'0.0.0.0'}])
    return actions


def generate_launch_description():
    config=str(Path(get_package_share_directory('sim'))/'config/hardware.yaml')
    defaults={'config':config,'profile':'map','use_sim_time':'false','run_dir':'','service_config':'','snapshot':'',
        'save_snapshot':'','coverage_config':'','tag_search':'coverage','saga_url':'http://127.0.0.1:8000','detector_backend':'tensorrt','detector_config':'','particles_per_robot':'160',
        'odometry_config':'','detector_python':'','foxglove':'false'}
    return LaunchDescription([*[DeclareLaunchArgument(k,default_value=v) for k,v in defaults.items()],OpaqueFunction(function=start)])
