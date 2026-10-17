"""Definition-only package: no publishers or consumers to start."""
from launch import LaunchDescription
from launch.actions import LogInfo


def generate_launch_description():
    return LaunchDescription([LogInfo(msg='PROVISIONAL interfaces definitions installed; no runtime nodes')])
