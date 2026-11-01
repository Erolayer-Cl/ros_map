"""mission_node con su YAML. No despega solo: hay que llamar a /mission/start o /mission/map.

    ros2 launch uav_mission mission.launch.py
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default = os.path.join(get_package_share_directory('uav_mission'), 'config', 'mission.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=default,
                              description='YAML con waypoints, área de mapeo y parámetros de esquive'),
        Node(package='uav_mission', executable='mission_node', name='mission_node',
             output='screen', parameters=[LaunchConfiguration('config')]),
    ])
