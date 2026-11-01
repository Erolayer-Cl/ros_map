"""Pruebas de punta a punta: misión + mapper + lidar simulado + dron, sin ROS.

Cada escenario de `scenarios/` declara en `esperado:` lo que tiene que salir.
Son las mismas geometrías que `spawn_unreal.py` crea en Unreal.
"""
import os
import sys
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
SRC = os.path.dirname(os.path.dirname(HERE))
for package in ('uav_sim', 'uav_mission', 'uav_wall_mapping'):
    sys.path.insert(0, os.path.join(SRC, package))

from uav_sim.closed_loop import run  # noqa: E402
from uav_sim.scenario import list_scenarios, load_scenario  # noqa: E402

# Todos los escenarios tardan unos minutos; por defecto se corren los más
# representativos. ROS_MAP_TODOS=1 corre todos.
QUICK = ['01_pared_plana', '02_pared_irregular', '05_trampa_U', '06_habitacion',
         '07_pared_inclinada']


class ClosedLoopTest(unittest.TestCase):

    def check(self, name):
        scenario = load_scenario(name)
        mission = scenario.mission.get('tipo', 'mapeo')
        result = run(scenario, mission, max_time=400.0)
        exp = scenario.expected
        msg = f'{name}: {result.final_state}, {result.walls_flat} planas, ' \
              f'{result.walls_irregular} irregulares, {result.obstacles} obstáculos'
        self.assertFalse(result.collided, f'{name}: choque (mínimo {result.min_clearance:.2f} m)')
        self.assertEqual(result.final_state, exp.get('estado_final', 'DONE'), msg)
        self.assertGreaterEqual(result.walls_flat, exp.get('paredes_planas_min', 0), msg)
        self.assertGreaterEqual(result.walls_irregular, exp.get('paredes_irregulares_min', 0), msg)
        self.assertGreaterEqual(result.obstacles, exp.get('obstaculos_min', 0), msg)
        if 'paredes_irregulares_max' in exp:
            self.assertLessEqual(result.walls_irregular, exp['paredes_irregulares_max'], msg)
        if 'cobertura_min' in exp:
            self.assertGreaterEqual(result.coverage, exp['cobertura_min'], msg)
        if 'inclinacion_max_min_deg' in exp:
            tilts = [w.tilt_deg for w in result.mapper.walls if w.tilt_deg == w.tilt_deg]
            self.assertTrue(tilts and max(tilts) >= exp['inclinacion_max_min_deg'], msg)


def _make(name):
    return lambda self: self.check(name)


for _name in (list_scenarios() if os.environ.get('ROS_MAP_TODOS') else QUICK):
    setattr(ClosedLoopTest, f'test_{_name}', _make(_name))


if __name__ == '__main__':
    unittest.main()
