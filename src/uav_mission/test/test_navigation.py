"""Pruebas de la evasión VFH+, el planificador A* y la exploración (sin ROS)."""
import math
import os
import sys
import unittest

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(HERE)), 'uav_wall_mapping'))

from uav_mission.avoidance import (AvoidConfig, VfhAvoider, braking_distance,  # noqa: E402
                                   max_safe_speed)
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

    def test_map_memory_alone_does_not_trigger_emergency(self):
        """Un punto que sólo está en la memoria del mapa frena/desvía pero no
        hace huir: la emergencia la decide el sensor en vivo."""
        ranges = free_sectors()
        ranges[18] = 0.6
        cmd = self.avoider.update(ranges, 0.0, 10.0, 0.0, live_ranges=free_sectors())
        self.assertFalse(cmd.emergency)
        live = free_sectors()
        live[18] = 0.6
        cmd = self.avoider.update(ranges, 0.0, 10.0, 0.0, live_ranges=live)
        self.assertTrue(cmd.emergency)

    def test_escape_speed_is_absolute(self):
        ranges = free_sectors()
        ranges[18] = 0.6
        cmd = VfhAvoider(AvoidConfig(escape_speed=0.75)).update(ranges, 0.0, 10.0, 0.0)
        self.assertAlmostEqual(cmd.max_speed, 0.75)

    def test_escape_only_towards_seen_sectors(self):
        """Cámara mirando al norte, algo pegado al norte: atrás nunca se vio,
        así que no se retrocede a ciegas: se frena en el lugar."""
        ranges = free_sectors()
        ranges[0] = 0.6
        seen = np.zeros(72, dtype=bool)
        seen[63:72] = True
        seen[0:10] = True
        cmd = self.avoider.update(ranges, 0.0, 10.0, 0.0, seen=seen)
        self.assertTrue(cmd.emergency)
        self.assertIsNone(cmd.direction)
        self.assertEqual(cmd.max_speed, 0.0)
        # Con lidar 360° (todo visto) sí se aleja hacia el sur.
        cmd = VfhAvoider(AvoidConfig()).update(ranges, 0.0, 10.0, 0.0)
        self.assertAlmostEqual(abs(cmd.direction), math.pi, delta=0.1)

    def test_escape_avoids_wall_behind(self):
        ranges = free_sectors()
        ranges[0] = 0.6               # algo al norte
        ranges[33:40] = 1.0           # y una pared al sur
        cmd = self.avoider.update(ranges, 0.0, 10.0, 0.0)
        self.assertTrue(cmd.emergency)
        self.assertIsNotNone(cmd.direction)
        self.assertGreater(abs(math.degrees(cmd.direction)), 30)
        self.assertLess(abs(math.degrees(cmd.direction)), 150)

    def test_deviation_only_towards_seen(self):
        ranges = free_sectors()
        ranges[66:72] = 2.5
        ranges[0:7] = 2.5
        seen = np.zeros(72, dtype=bool)
        seen[60:72] = True
        seen[0:12] = True             # la cámara ve ±60°, la pared tapa ±35°
        cmd = self.avoider.update(ranges, 0.0, 10.0, 0.0, seen=seen)
        self.assertTrue(cmd.deviated)
        self.assertLessEqual(abs(math.degrees(cmd.direction)), 60)

    def test_surrounded_is_blocked(self):
        cmd = self.avoider.update(np.full(72, 1.5), 0.0, 10.0, 0.0)
        self.assertTrue(cmd.blocked)
        self.assertIsNone(cmd.direction)


class BrakingTest(unittest.TestCase):
    """Frenado físico: v_max(d) = −a·t + √((a·t)² + 2·a·d)."""

    def test_max_speed_inverts_braking_distance(self):
        for v in (0.5, 1.5, 3.0, 8.0):
            for a, t in ((1.0, 0.4), (3.0, 0.1), (2.0, 0.9)):
                d = braking_distance(v, a, t)
                self.assertAlmostEqual(max_safe_speed(d, a, t), v, places=6)
        self.assertEqual(max_safe_speed(-0.5, 1.0, 0.4), 0.0)

    def test_px4_cp_example(self):
        # Ejemplo de la doc de investigación (PX4 CP sin jerk): 2 m libres,
        # a = 3 m/s², sin retardo -> sqrt(2·3·2) = 3.46 m/s.
        self.assertAlmostEqual(max_safe_speed(2.0, 3.0, 0.0), math.sqrt(12.0), places=6)

    def test_speed_limit_falls_with_distance_and_latency(self):
        avoider = VfhAvoider(AvoidConfig(max_decel=1.0, latency=0.4))
        ranges = free_sectors()
        ranges[0] = 4.0                      # pared al frente a 4 m
        near = avoider.update(ranges, 0.0, 20.0, 0.0)
        ranges[0] = 8.0
        far = avoider.update(ranges, 0.0, 20.0, 0.0)
        old = avoider.update(ranges, 0.0, 20.0, 0.0, data_age=0.6)
        self.assertLess(near.max_speed, far.max_speed)
        self.assertLess(old.max_speed, far.max_speed)
        # Con lo libre exactamente igual a la distancia a mantener, no se avanza.
        ranges[0] = avoider.keep_distance()
        self.assertAlmostEqual(avoider.limit_for(ranges, 0.0)[1], 0.0, places=6)

    def test_side_wall_does_not_brake(self):
        avoider = VfhAvoider(AvoidConfig())
        ranges = free_sectors()
        ranges[15:22] = 1.6                  # pared paralela 1.6 m al este, yendo al norte
        cmd = avoider.update(ranges, 0.0, 20.0, 0.0)
        self.assertFalse(cmd.deviated)
        self.assertEqual(cmd.max_speed, INF)

    def test_lookahead_grows_with_speed(self):
        avoider = VfhAvoider(AvoidConfig(lookahead=4.0, max_decel=1.0, latency=0.4))
        ranges = free_sectors()
        ranges[0] = 7.0                      # obstáculo a 7 m: a 1 m/s no importa
        slow = avoider.update(ranges, 0.0, 30.0, 0.0, speed=1.0)
        fast = avoider.update(ranges, 0.0, 30.0, 0.0, speed=4.0)
        self.assertFalse(slow.deviated)
        self.assertTrue(fast.deviated)
        self.assertGreater(fast.block_range, 7.0)

    def test_sensor_range_caps_speed(self):
        avoider = VfhAvoider(AvoidConfig(max_decel=1.0, latency=0.4))
        cmd = avoider.update(free_sectors(), 0.0, 30.0, 0.0, sensor_range=6.0)
        self.assertAlmostEqual(cmd.max_speed, max_safe_speed(6.0 - avoider.keep_distance(),
                                                             1.0, 0.4), places=6)

    def test_obstacle_beside_corridor_does_not_stop(self):
        # Esquina a 1 m, 75° del rumbo: el VFH deja pasar (queda a 0.97 m del
        # eje, fuera del corredor de 0.95 m) y el frenado no tiene que trabar.
        avoider = VfhAvoider(AvoidConfig())
        ranges = free_sectors()
        ranges[15] = 1.0
        cmd = avoider.update(ranges, 0.0, 20.0, 0.0)
        self.assertFalse(cmd.deviated)
        self.assertEqual(cmd.max_speed, INF)


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

    def test_two_meter_door_is_passable(self):
        """Pared con una puerta de 2 m: el A* (inflado 0.65 m medido en la
        grilla fina) tiene que cruzarla y no rodear."""
        res = 0.15
        values = np.zeros((200, 200), dtype=np.int8)
        origin = -15.0
        row = int((8.0 - origin) / res)
        values[row:row + 2, :] = 100
        c0, c1 = int((0.0 - origin) / res), int((2.0 - origin) / res)
        values[row:row + 2, c0:c1] = 0
        snap = GridSnapshot(values=values, resolution=res, origin_n=origin, origin_e=origin)
        result = GridPlanner(inflation_radius=0.65).plan(snap, (2.0, 1.0), (14.0, 1.0))
        self.assertTrue(result.ok, result.reason)
        path = np.asarray(result.path)
        self.assertLess(np.max(np.abs(path[:, 1] - 1.0)), 1.0)

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
