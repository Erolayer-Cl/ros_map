"""Piezas compartidas por los launch de simulación (no es un launch en sí)."""
import os

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

from uav_sim.scenario import load_scenario, mission_overrides


def scenario_mission(scenario_name, mission):
    """(tipo de misión, overrides) para un escenario; mission vacío = el del escenario."""
    if not scenario_name:
        return mission, {}
    scenario = load_scenario(scenario_name)
    kind = mission or scenario.mission.get('tipo', 'mapeo')
    return kind, mission_overrides(scenario, kind)


def mapper_node(config_name='mapper_unreal.yaml', extra=None):
    config = os.path.join(get_package_share_directory('uav_wall_mapping'), 'config', config_name)
    return Node(package='uav_wall_mapping', executable='mapper_node', name='mapper_node',
                output='screen', parameters=[config] + ([extra] if extra else []))


def mission_node(overrides, auto_start=''):
    config = os.path.join(get_package_share_directory('uav_mission'), 'config', 'mission.yaml')
    params = dict(overrides)
    params['auto_start'] = auto_start
    return Node(package='uav_mission', executable='mission_node', name='mission_node',
                output='screen', parameters=[config, params])
