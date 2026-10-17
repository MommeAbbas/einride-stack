from pathlib import Path
import os
import tempfile
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def nodes(context):
    sim_time=LaunchConfiguration('use_sim_time').perform(context).lower()=='true'
    data=yaml.safe_load(Path(LaunchConfiguration('config').perform(context)).read_text())
    def clock_parameters(value):
        if isinstance(value,dict):
            for key,item in value.items():
                if key=='use_sim_time': value[key]=sim_time
                else: clock_parameters(item)
    clock_parameters(data)
    with tempfile.NamedTemporaryFile(mode='w',suffix='.yaml',prefix='planning_',delete=False) as f:
        yaml.safe_dump(data,f);config=f.name
    parameters=[config, {'use_sim_time':sim_time}]
    parameters.append({'saga_token':os.environ.get('AUTONOMY_SAGA_TOKEN',os.environ.get('AUTONOMY_SIM_TOKEN_1',''))})
    result=[Node(package='nav2_planner',executable='planner_server',name='planner_server',parameters=parameters[:1]+[{'use_sim_time':sim_time}],output='screen'),
        Node(package='nav2_lifecycle_manager',executable='lifecycle_manager',name='planning_costmap_lifecycle',parameters=[{'use_sim_time':sim_time,'autostart':True,'bond_timeout':20.0,'attempt_respawn_reconnection':True,'node_names':['planner_server']}],output='screen'),
        Node(package='planning',executable='planning_planning_node',output='screen',parameters=parameters)]
    if yaml.safe_load(Path(parameters[0]).read_text())['/**']['ros__parameters'].get('explore',False):
        result.append(Node(package='explore_lite',executable='explore',parameters=[str(Path(get_package_share_directory('planning'))/'config/explore.yaml'), {'use_sim_time':sim_time}],remappings=[('navigate_to_pose','/exploration/navigate_to_pose')],output='screen'))
    return result


def generate_launch_description():
    config=str(Path(get_package_share_directory('planning'))/'config/planning.yaml')
    return LaunchDescription([DeclareLaunchArgument('use_sim_time',default_value='true'),DeclareLaunchArgument('config',default_value=config),OpaqueFunction(function=nodes)])
