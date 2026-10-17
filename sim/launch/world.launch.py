"""Gazebo launch; source the external simulator environment first."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share=Path(get_package_share_directory('sim'))
    config=LaunchConfiguration('config'); directory=LaunchConfiguration('run_dir')
    prepare=Node(package='sim',executable='sim_world_node',output='screen',parameters=[config,{
        'run_dir':directory,'traffic_count':ParameterValue(LaunchConfiguration('traffic_count'),value_type=int),
        'use_sim_time':True}])
    def ready(event,context):
        if event.returncode:
            return [EmitEvent(event=Shutdown(reason='World generation failed'))]
        run_dir=directory.perform(context)
        actions=[IncludeLaunchDescription(PythonLaunchDescriptionSource(str(Path(
            get_package_share_directory('einride_mini_truck_bringup'))/'launch/simulation.launch.py')),
            launch_arguments={'world':str(Path(run_dir)/'arena'),'headless':LaunchConfiguration('headless'),
                              'rviz':'false'}.items()),
            Node(package='ros_gz_bridge',executable='parameter_bridge',name='sim_bridge',
                 parameters=[{'use_sim_time':True,'config_file':str(Path(run_dir)/'bridge.yaml')}],output='screen')]
        return actions if int(LaunchConfiguration('traffic_count').perform(context)) else actions[:1]
    return LaunchDescription([
        DeclareLaunchArgument('config',default_value=str(share/'config/sim.yaml')),
        DeclareLaunchArgument('run_dir',default_value='/tmp/sim_run'),
        DeclareLaunchArgument('traffic_count',default_value='3'),
        DeclareLaunchArgument('headless',default_value='true'),
        RegisterEventHandler(OnProcessExit(target_action=prepare,on_exit=ready)),prepare])
