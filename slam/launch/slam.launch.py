"""Selected mapping/localisation with unchanged public interfaces."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from slam.snapshot import load_snapshot


def nodes(context):
    share=Path(get_package_share_directory('slam'))
    value=lambda key:LaunchConfiguration(key).perform(context)
    common={'use_sim_time':value('use_sim_time').lower()=='true'}
    snapshot=value('load_snapshot');actions=[]
    if snapshot:
        data=load_snapshot(snapshot);config=str(share/'config/localisation.yaml')
        actions += [Node(package='nav2_map_server',executable='map_server',name='map_server',parameters=[config,common,{'yaml_filename':data['map_yaml']}],output='screen'),Node(package='nav2_amcl',executable='amcl',name='amcl',parameters=[config,common],output='screen')]
        managed=['map_server','amcl']
    else:
        config=str(share/'config/mapping.yaml')
        actions += [Node(package='slam_toolbox',executable='async_slam_toolbox_node',name='slam_toolbox',parameters=[config,common],output='screen'),Node(package='nav2_map_server',executable='map_saver_server',name='map_saver',parameters=[config,common],output='screen')]
        managed=['slam_toolbox','map_saver']
    actions += [Node(package='nav2_lifecycle_manager',executable='lifecycle_manager',name='slam_lifecycle_manager',parameters=[common,{'autostart':True,'bond_timeout':20.0,'attempt_respawn_reconnection':True,'node_names':managed}],output='screen'),Node(package='slam',executable='slam_slam_node',parameters=[value('config'),common,{'save_snapshot':value('save_snapshot'),'load_snapshot':snapshot}],output='screen')]
    return actions


def generate_launch_description():
    config=str(Path(get_package_share_directory('slam'))/'config/slam.yaml')
    return LaunchDescription([DeclareLaunchArgument('use_sim_time',default_value='true'),DeclareLaunchArgument('config',default_value=config),DeclareLaunchArgument('save_snapshot',default_value=''),DeclareLaunchArgument('load_snapshot',default_value=''),OpaqueFunction(function=nodes)])
