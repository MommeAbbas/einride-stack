"""CPU Nav2 MPPI, coordinator and raw-sensor collision monitor, with an explicit clock choice."""
from pathlib import Path
import tempfile
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_nodes(context):
    sim_time=LaunchConfiguration('use_sim_time').perform(context).lower()=='true'
    config=LaunchConfiguration('config').perform(context)
    required=LaunchConfiguration('require_predictions').perform(context).lower()=='true'
    enabled=LaunchConfiguration('prediction_critic').perform(context).lower()=='true'
    params=yaml.safe_load(Path(config).read_text())
    def clock_parameters(value):
        if isinstance(value,dict):
            for key,item in value.items():
                if key=='use_sim_time': value[key]=sim_time
                else: clock_parameters(item)
    clock_parameters(params)
    tree=LaunchConfiguration('navigation_tree').perform(context)
    params['bt_navigator']['ros__parameters']['default_nav_to_pose_bt_xml']=str(Path(get_package_share_directory('nav2_bt_navigator'))/'behavior_trees'/tree)
    critic=params['controller_server']['ros__parameters']['FollowPath']['PredictionCritic']
    critic['require_predictions']=required;critic['enabled']=enabled
    params['/**']['ros__parameters']['require_predictions']=required
    with tempfile.NamedTemporaryFile(mode='w',suffix='.yaml',prefix='control_',delete=False) as f:
        yaml.safe_dump(params,f);config=f.name
    return [Node(package='control',executable='control_depth_node',parameters=[config],output='screen'),Node(package='nav2_controller',executable='controller_server',name='controller_server',output='screen',parameters=[config,{'use_sim_time':sim_time,'FollowPath.PredictionCritic.require_predictions':required,'FollowPath.PredictionCritic.enabled':enabled}],remappings=[('cmd_vel','/control/nav2_cmd')]),
      Node(package='nav2_behaviors',executable='behavior_server',name='behavior_server',output='screen',parameters=[config],remappings=[('cmd_vel','/control/nav2_cmd')]),
      Node(package='nav2_bt_navigator',executable='bt_navigator',name='bt_navigator',output='screen',parameters=[config]),
      Node(package='nav2_collision_monitor',executable='collision_monitor',name='collision_monitor',output='screen',parameters=[config]),
      Node(package='nav2_lifecycle_manager',executable='lifecycle_manager',name='control_lifecycle',output='screen',parameters=[{'use_sim_time':sim_time,'autostart':True,'bond_timeout':20.0,'attempt_respawn_reconnection':True,'node_names':['controller_server','behavior_server','collision_monitor','bt_navigator']}]),
      Node(package='control',executable='control_coordinator_node',output='screen',parameters=[config,{'use_sim_time':sim_time,'require_predictions':required}])]


def generate_launch_description():
    config=str(Path(get_package_share_directory('control'))/'config/control.yaml')
    return LaunchDescription([DeclareLaunchArgument('config',default_value=config),DeclareLaunchArgument('use_sim_time',default_value='true'),DeclareLaunchArgument('require_predictions',default_value='true'),DeclareLaunchArgument('prediction_critic',default_value='false'),DeclareLaunchArgument('navigation_tree',default_value='navigate_w_replanning_time.xml'),OpaqueFunction(function=launch_nodes)])
