"""mapper_node con un archivo de configuración (por defecto, el de Unreal).

    ros2 launch uav_wall_mapping mapper.launch.py
    ros2 launch uav_wall_mapping mapper.launch.py config:=$(ros2 pkg prefix uav_wall_mapping)/share/uav_wall_mapping/config/mapper_mid360.yaml
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default = os.path.join(get_package_share_directory('uav_wall_mapping'), 'config', 'mapper_unreal.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=default,
                              description='YAML del mapper (sensores, franja de alturas, paredes)'),
        Node(package='uav_wall_mapping', executable='mapper_node', name='mapper_node',
             output='screen', parameters=[LaunchConfiguration('config')]),
    ])
