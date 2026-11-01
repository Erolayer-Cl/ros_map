"""mapper + misión contra Unreal (Cosys-AirSim), con los parámetros del escenario.

PX4 SITL, el agente uXRCE-DDS y airsim_node se levantan aparte
(scripts/launch_sim_unreal.sh lo hace todo en tmux). Esto sólo agrega lo propio:

    ros2 launch uav_sim unreal.launch.py escenario:=06_habitacion
    ros2 run uav_sim spawn_scenario --escenario 06_habitacion     # crear las paredes
    ros2 service call /mission/map std_srvs/srv/Trigger

Sin `escenario`, la misión usa el área y los waypoints de mission.yaml (paredes
puestas a mano en el editor de Unreal).
"""
import os
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from _common import mapper_node, mission_node, scenario_mission  # noqa: E402


def _setup(context):
    scenario = LaunchConfiguration('escenario').perform(context)
    kind, overrides = scenario_mission(scenario, LaunchConfiguration('mision').perform(context))
    pitch = float(LaunchConfiguration('lidar_pitch').perform(context))
    return [
        mapper_node('mapper_unreal.yaml', {'cloud.mount_rpy_deg': [0.0, pitch, 0.0]}),
        mission_node(overrides, ''),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('escenario', default_value='',
                              description='escenario creado con spawn_scenario (vacío = mission.yaml)'),
        DeclareLaunchArgument('mision', default_value=''),
        DeclareLaunchArgument('lidar_pitch', default_value='0.0',
                              description='tiene que coincidir con el Pitch del lidar en settings.json'),
        OpaqueFunction(function=_setup),
    ])
