"""Stack completo SIN Unreal ni GPU: PX4 + lidar falsos, mapper y misión.

    ros2 launch uav_sim fake_sim.launch.py escenario:=06_habitacion
    ros2 service call /mission/map std_srvs/srv/Trigger

    # o que arranque sola la misión que propone el escenario:
    ros2 launch uav_sim fake_sim.launch.py escenario:=05_trampa_U auto:=true

Para verlo: foxglove_bridge (ros2 launch foxglove_bridge foxglove_bridge_launch.xml)
y en Foxglove los paneles 3D (/map, /walls/markers, /fake_world/markers,
/map/cloud, /mission/path) y Plot de /mission/status.
"""
import os
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from _common import mapper_node, mission_node, scenario_mission  # noqa: E402


def _setup(context):
    scenario = LaunchConfiguration('escenario').perform(context)
    kind, overrides = scenario_mission(scenario, LaunchConfiguration('mision').perform(context))
    auto = LaunchConfiguration('auto').perform(context).lower() in ('1', 'true', 'si', 'sí')
    pitch = float(LaunchConfiguration('lidar_pitch').perform(context))
    lidar_rpy = [0.0, pitch, 0.0]
    return [
        Node(package='uav_sim', executable='fake_px4_world', name='fake_px4_world', output='screen',
             parameters=[{'escenario': scenario, 'lidar.mount_rpy_deg': lidar_rpy}]),
        mapper_node('mapper_unreal.yaml', {'cloud.mount_rpy_deg': lidar_rpy}),
        mission_node(overrides, kind if auto else ''),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('escenario', default_value='06_habitacion',
                              description='escenario de src/uav_sim/scenarios'),
        DeclareLaunchArgument('mision', default_value='',
                              description="'mapeo' | 'waypoints' (vacío = el del escenario)"),
        DeclareLaunchArgument('auto', default_value='false',
                              description='arrancar la misión sola, sin llamar al servicio'),
        DeclareLaunchArgument('lidar_pitch', default_value='0.0',
                              description='inclinación del lidar en grados (negativo = hacia abajo)'),
        OpaqueFunction(function=_setup),
    ])
