"""Joint-build area launch. Simulation-only fixtures are explicit and cannot masquerade as SLAM."""
import os
from pathlib import Path
import yaml
from ament_index_python.packages import get_package_share_directory,get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,OpaqueFunction,IncludeLaunchDescription,RegisterEventHandler,EmitEvent,LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit,OnProcessStart
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def start(context):
    for package in ('nav2_bt_navigator','nav2_planner','nav2_smac_planner','nav2_amcl','nav2_map_server','nav2_costmap_2d','nav2_lifecycle_manager','nav2_controller','nav2_behaviors','nav2_collision_monitor'):
        if get_package_prefix(package)!='/opt/ros/jazzy':
            raise RuntimeError(f'{package} must resolve to system /opt/ros/jazzy; remove overlays')
    config=LaunchConfiguration('config').perform(context)
    sim=yaml.safe_load(Path(config).read_text())['/**']['ros__parameters'];stage=sim['stack_stage'];directory=Path(sim['run_dir'])
    if stage not in ('E0','E1','E2','E3','E4'):raise ValueError('Unsupported stage')
    perception=stage in ('E3','E4')
    def process_started(event,context):
        executable=event.cmd[0]
        if '/nav2_' in executable:
            return [LogInfo(msg=f'Nav2 executable started: {executable} (pid {event.pid})')]
        return []
    nodes=[RegisterEventHandler(OnProcessStart(on_start=process_started))]
    if sim['map_source']=='fixture':nodes.append(Node(package='sim',executable='sim_fixture_node',parameters=[config],output='screen'))
    else:
        nodes.append(IncludeLaunchDescription(PythonLaunchDescriptionSource(str(Path(get_package_share_directory('slam'))/'launch/slam.launch.py')),launch_arguments={'use_sim_time':'true','save_snapshot':str(directory/'slam_snapshot.json'),'load_snapshot':sim['map_snapshot'] if sim['map_source']=='snapshot' else ''}.items()))
    planning=yaml.safe_load((Path(get_package_share_directory('planning'))/'config/planning.yaml').read_text());p=planning['/**']['ros__parameters']
    p.update(mission_enabled=stage!='E0',risk_check=sim['planning_risk'],look=sim['planning_look'],observation_curves=sim['planning_observation_curves'],manual_goal=sim['manual_goal'] if stage=='E1' else -1,saga_url=sim['saga_url'])
    # Only E0 maps. A delayed/missing DockMap in saved-map stages must hold,
    # rather than incorrectly starting mapping exploration before readiness.
    p['explore']=stage=='E0'
    if sim['planning_risk_tolerance']>=0:
        if sim['planning_risk_tolerance']>p['risk_tolerance']:raise ValueError('Scenario risk override must be no less cautious than baseline')
        p['risk_tolerance']=sim['planning_risk_tolerance']
    file=directory/'planning.yaml';file.write_text(yaml.safe_dump(planning))
    nodes.append(IncludeLaunchDescription(PythonLaunchDescriptionSource(str(Path(get_package_share_directory('planning'))/'launch/planning.launch.py')),launch_arguments={'config':str(file)}.items()))
    if perception:
        params=yaml.safe_load((Path(get_package_share_directory('perception'))/'config/perception.yaml').read_text());p=params['/**']['ros__parameters']
        mapping={'lidar':'perception_lidar','camera_negative_evidence':'perception_camera_negative','lidar_negative_evidence':'perception_lidar_negative','dock_steering':'perception_dock_steering','injection':'perception_injection','heavy_tail':'perception_heavy_tail','floor_cue':'perception_floor_cue','height_cue':'perception_height_cue','depth_cue':'perception_depth_cue','detector_source':'detector_source'}
        p.update({k:sim[v] for k,v in mapping.items()});file=directory/'perception.yaml';file.write_text(yaml.safe_dump(params))
        nodes.append(Node(package='perception',executable='perception_perception_node',respawn=True,respawn_delay=3.0,parameters=[str(file)],output='screen'))
    if stage!='E0':nodes.append(Node(package='perception',executable='perception_dock_node',respawn=True,respawn_delay=3.0,parameters=[str(Path(get_package_share_directory('perception'))/'config/perception.yaml')],output='screen'))
    launch=Path(get_package_share_directory('control'))/'launch/control.launch.py'
    nodes.append(IncludeLaunchDescription(PythonLaunchDescriptionSource(str(launch)),launch_arguments={'config':str(Path(get_package_share_directory('control'))/'config/control.yaml'),'require_predictions':str(perception and (sim['control_prediction_critic'] or sim['planning_risk'])).lower(),'navigation_tree':'navigate_to_pose_w_replanning_and_recovery.xml' if stage=='E0' else 'navigate_w_replanning_time.xml','prediction_critic':str(perception and sim['control_prediction_critic']).lower()}.items()))
    nodes.append(Node(package='sim',executable='sim_visualization_node',parameters=[config],output='screen'))
    if sim['foxglove']:
        nodes.append(Node(package='foxglove_bridge',executable='foxglove_bridge',parameters=[{'use_sim_time':True,'port':sim['foxglove_port'],'address':'127.0.0.1'}],output='screen'))
    core={'async_slam_toolbox_node','map_server','amcl','map_saver_server','planner_server','controller_server','behavior_server','bt_navigator','collision_monitor','lifecycle_manager','slam_wheel_odometry_node'}
    nodes.append(RegisterEventHandler(OnProcessExit(target_action=lambda action:Path(action.process_details['cmd'][0]).name in core,on_exit=[EmitEvent(event=Shutdown(reason='A core stack process exited'))])))
    return nodes


def generate_launch_description():return LaunchDescription([DeclareLaunchArgument('config'),OpaqueFunction(function=start)])
