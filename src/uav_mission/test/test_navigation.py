"""Pruebas de la evasión VFH+, el planificador A* y la exploración (sin ROS)."""
import math
import os
import sys
import unittest

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(HERE)), 'uav_wall_mapping'))

from uav_mission.avoidance import AvoidConfig, VfhAvoider  # noqa: E402
from uav_mission.exploration import find_frontiers, lawnmower_waypoints  # noqa: E402
from uav_mission.planner import GridPlanner, path_lookahead  # noqa: E402
from uav_wall_mapping.occupancy import GridSnapshot  # noqa: E402

INF = float('inf')


def free_sectors():
    return np.full(72, INF)


class AvoidanceTest(unittest.TestCase):

    def setUp(self):
        self.avoider = VfhAvoider(AvoidConfig())

    def test_clear_path_goes_straight(self):
        cmd = self.avoider.update(free_sectors(), 0.3, 10.0, 0.0)
        self.assertFalse(cmd.deviated)
        self.assertAlmostEqual(cmd.direction, 0.3)
        self.assertAlmostEqual(cmd.speed_scale, 1.0)

    def test_wall_ahead_deviates(self):
        ranges = free_sectors()
        ranges[66:72] = 2.5        # pared al frente, un poco hacia el oeste
        ranges[0:7] = 2.5
        cmd = self.avoider.update(ranges, 0.0, 10.0, 0.0)
        self.assertTrue(cmd.deviated)
        self.assertGreater(abs(cmd.direction), math.radians(35))

    def test_obstacle_beyond_goal_does_not_block(self):
        ranges = free_sectors()
        ranges[0] = 3.0
        cmd = self.avoider.update(ranges, 0.0, 1.0, 0.0)   # objetivo a 1 m, pared a 3 m
        self.assertFalse(cmd.deviated)

    def test_emergency_moves_away(self):
        ranges = free_sectors()
        ranges[18] = 0.6          # algo pegado al Este
        cmd = self.avoider.update(ranges, 0.0, 10.0, 0.0)
        self.assertTrue(cmd.emergency)
        self.assertAlmostEqual(abs(cmd.direction), math.pi / 2, delta=0.1)   # hacia el Oeste

    def test_surrounded_is_blocked(self):
        cmd = self.avoider.update(np.full(72, 1.5), 0.0, 10.0, 0.0)
        self.assertTrue(cmd.blocked)
        self.assertIsNone(cmd.direction)


def u_trap_snapshot():
    """Mapa 30x30 m a 0.15 m con una U abierta hacia el sur entre el dron y el objetivo."""
    res = 0.15
    size = 200
    values = np.zeros((size, size), dtype=np.int8)
    origin = -15.0

    def cell(n, e):
        return int((n - origin) / res), int((e - origin) / res)

    r0, c0 = cell(8.0, -4.0)
    r1, c1 = cell(8.3, 4.0)
    values[r0:r1, c0:c1] = 100                  # fondo de la U
    r0, c0 = cell(3.0, -4.0)
    values[r0:r1, c0:c0 + 2] = 100              # brazo oeste
    r0, c1 = cell(3.0, 4.0)
    values[r0:r1, c1 - 2:c1] = 100              # brazo este
    return GridSnapshot(values=values, resolution=res, origin_n=origin, origin_e=origin)


class PlannerTest(unittest.TestCase):

    def test_path_leaves_u_trap(self):
        snap = u_trap_snapshot()
        planner = GridPlanner()
        result = planner.plan(snap, (5.0, 0.0), (12.0, 0.0))
        self.assertTrue(result.ok, result.reason)
        path = np.asarray(result.path)
        # El camino tiene que salir por un costado de la U (|este| > 4 m)...
        self.assertGreater(np.max(np.abs(path[:, 1])), 4.0)
        # ...y ningún punto del camino puede caer en una celda ocupada.
        for n, e in path:
            self.assertLess(snap.value_at(n, e), 65)

    def test_unknown_space_is_traversable(self):
        snap = GridSnapshot(values=np.full((100, 100), -1, dtype=np.int8), resolution=0.2,
                            origin_n=-10.0, origin_e=-10.0)
        result = GridPlanner().plan(snap, (0.0, 0.0), (25.0, 3.0))
        self.assertTrue(result.ok)

    def test_lookahead(self):
        point = path_lookahead([(0.0, 0.0), (10.0, 0.0), (10.0, 10.0)], (9.0, 0.5), 3.0)
        self.assertAlmostEqual(point[0], 10.0, delta=0.05)
        self.assertAlmostEqual(point[1], 2.0, delta=0.05)


class ExplorationTest(unittest.TestCase):

    def test_frontier_between_free_and_unknown(self):
        values = np.full((100, 100), -1, dtype=np.int8)
        values[:, :50] = 0                       # mitad oeste libre, este desconocida
        snap = GridSnapshot(values=values, resolution=0.2, origin_n=0.0, origin_e=0.0)
        frontiers = find_frontiers(snap, area=(0.0, 0.0, 20.0, 20.0))
        self.assertTrue(frontiers)
        biggest = max(frontiers, key=lambda f: f.size)
        self.assertAlmostEqual(biggest.target[1], 9.9, delta=0.3)

    def test_no_frontier_when_area_known(self):
        values = np.zeros((100, 100), dtype=np.int8)
        snap = GridSnapshot(values=values, resolution=0.2, origin_n=0.0, origin_e=0.0)
        self.assertEqual(find_frontiers(snap, area=(2.0, 2.0, 18.0, 18.0)), [])

    def test_lawnmower_covers_area(self):
        wps = lawnmower_waypoints((0.0, 0.0, 30.0, 12.0), spacing=4.0)
        easts = sorted({round(p[1], 2) for p in wps})
        self.assertEqual(easts[0], 1.0)
        self.assertEqual(easts[-1], 11.0)
        self.assertTrue(all(abs(b[0] - a[0]) > 1.0 or abs(b[1] - a[1]) <= 4.01
                            for a, b in zip(wps[:-1], wps[1:])))


if __name__ == '__main__':
    unittest.main()
