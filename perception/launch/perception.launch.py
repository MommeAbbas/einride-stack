from pathlib import Path
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def nodes(context):
    parameters=[LaunchConfiguration('config').perform(context), {'use_sim_time':LaunchConfiguration('use_sim_time').perform(context).lower()=='true'}]

    return [Node(package='perception',executable=executable,output='screen',parameters=parameters,respawn=True,respawn_delay=3.0) for executable in ('perception_perception_node','perception_dock_node')]


def generate_launch_description():
    config=str(Path(get_package_share_directory('perception'))/'config/perception.yaml')
    return LaunchDescription([DeclareLaunchArgument('use_sim_time',default_value='true'),DeclareLaunchArgument('config',default_value=config),OpaqueFunction(function=nodes)])
