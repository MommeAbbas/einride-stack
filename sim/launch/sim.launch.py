"""Simulation support nodes. Start world.launch.py in the separate Gazebo terminal."""
import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share=Path(get_package_share_directory('sim'))
    def start(context):
        count=int(LaunchConfiguration('traffic_count').perform(context))
        if count not in range(4):
            raise ValueError('traffic_count must be 0..3')
        config=LaunchConfiguration('config').perform(context)
        common={'run_dir':LaunchConfiguration('run_dir').perform(context),'traffic_count':count,
                'use_sim_time':True,'route_seed':int(LaunchConfiguration('route_seed').perform(context))}
        nodes=[Node(package='sim',executable='sim_'+name+'_node',parameters=[config,common],output='screen')
               for name in ['truth','map','collision','detector','metrics']]
        secrets={'admin_username':os.environ.get('AUTONOMY_SIM_ADMIN_USER','admin'),
                 'admin_password':os.environ.get('AUTONOMY_SIM_ADMIN_PASSWORD','')}
        secrets.update({f'saga_token_{i}':os.environ.get(f'AUTONOMY_SIM_TOKEN_{i}','') for i in range(1,5)})
        nodes.append(Node(package='sim',executable='sim_organiser_node',parameters=[config,common,secrets],output='screen'))
        for i in range(1,count+1):
            nodes.append(Node(package='sim',executable='sim_traffic_node',name=f'sim_traffic_{i}',
                parameters=[config,common,{'robot_id':i,'saga_token':secrets[f'saga_token_{i+1}']}],output='screen'))
        return nodes
    return LaunchDescription([
        DeclareLaunchArgument('config',default_value=str(share/'config/sim.yaml')),
        DeclareLaunchArgument('run_dir',default_value='/tmp/sim_run'),
        DeclareLaunchArgument('traffic_count',default_value='3'),
        DeclareLaunchArgument('route_seed',default_value='7'),OpaqueFunction(function=start)])
