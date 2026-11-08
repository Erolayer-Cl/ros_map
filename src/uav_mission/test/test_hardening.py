"""Entradas basura para la máquina de estados: nunca un setpoint no finito a
PX4, nunca una excepción, y NaN es "no sé", no "libre"."""
import math
import os
import sys
import unittest

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(HERE)), 'uav_wall_mapping'))

from uav_mission.avoidance import VfhAvoider  # noqa: E402
from uav_mission.mission_core import DEFAULTS, MissionCore, State, validate_params  # noqa: E402

NAN = float('nan')


def fly(feed, steps=500, dt=0.05):
    """Arma, despega y vuela waypoints con un PX4 de juguete; `feed(core, i,
    t, pos)` entrega las entradas. Devuelve (core, setpoints no finitos)."""
    core = MissionCore()
    pos = [0.0, 0.0, 0.0]
    armed = offboard = False
    bad = []
    t = 0.0
    for i in range(steps):
        feed(core, i, t, pos)
        core.update_status(armed, offboard)
        if i == 10:
            ok, msg = core.start_waypoints(t)
            assert ok, msg
        core.planning_request(t)
        core.step(t)
        for c in core.pop_commands():
            armed |= c == 'arm'
            offboard |= c == 'mode_offboard'
        sp = core.setpoint
        if core.stream_offboard and sp is not None:
            if not all(math.isfinite(v) for v in sp):
                bad.append((i, sp))
            else:
                for k in range(3):
                    pos[k] += max(-0.1, min(0.1, sp[k] - pos[k]))
        if core.velocity_setpoint is not None and not all(
                math.isfinite(v) for v in core.velocity_setpoint):
            bad.append((i, core.velocity_setpoint))
        t += dt
    return core, bad


def normal(core, i, t, pos, ranges=None):
    core.update_vehicle(tuple(pos), (0.0, 0.0, 0.0), 0.0, stamp=t)
    core.update_sectors(np.full(72, 10.0) if ranges is None else ranges, t, range_max=20.0)


class MissionInputsTest(unittest.TestCase):

    def test_nan_heading_and_velocity_never_reach_px4(self):
        def feed(core, i, t, pos):
            core.update_vehicle(tuple(pos), (NAN, math.inf, 0.0), NAN, stamp=t)
            core.update_sectors(np.full(72, 10.0), t)
        core, bad = fly(feed)
        self.assertEqual(bad, [])
        self.assertNotEqual(core.state, State.IDLE)

    def test_nan_position_is_pose_loss(self):
        def feed(core, i, t, pos):
            if 200 <= i < 400:
                core.update_vehicle((NAN, 0.0, -2.0), (0, 0, 0), 0.0, stamp=t)
                core.update_sectors(np.full(72, 10.0), t)
            else:
                normal(core, i, t, pos)
        core, bad = fly(feed, steps=400)
        self.assertEqual(bad, [])
        self.assertFalse(core.position_valid)
        # 10 s sin posición: termina aterrizando (nav.pose_loss_land_after).
        self.assertIn(core.state, (State.LAND, State.HOLD, State.NAVIGATE, State.AVOID))
        self.assertTrue(all(math.isfinite(v) for v in core.position))

    def test_empty_or_tiny_sector_arrays_are_ignored(self):
        def feed(core, i, t, pos):
            core.update_vehicle(tuple(pos), (0, 0, 0), 0.0, stamp=t)
            core.update_sectors(np.full(72, 10.0), t)
            core.update_sectors([] if i % 2 else [1.0] * 3, t)
        core, bad = fly(feed)
        self.assertEqual(bad, [])
        self.assertEqual(core.avoider.n_bins, 72)

    def test_nan_sectors_count_as_unseen_not_free(self):
        core = MissionCore()
        ranges = np.full(72, 10.0)
        ranges[:10] = NAN
        core.update_sectors(ranges, 1.0)
        self.assertTrue(np.all(np.isinf(core._sectors[:10])))
        seen = core._seen_mask(1.0)
        self.assertFalse(np.any(seen[:10]))
        self.assertTrue(np.all(seen[10:]))

    def test_non_finite_sector_stamp_is_ignored(self):
        core = MissionCore()
        core.update_sectors(np.full(72, 5.0), 1.0)
        core.update_sectors(np.full(72, 0.5), NAN)
        self.assertEqual(core._sectors_stamp, 1.0)
        self.assertEqual(float(core._sectors[0]), 5.0)

    def test_garbage_map_is_ignored(self):
        core = MissionCore()
        for snap in (None, 'basura', object()):
            core.update_map(snap)
        self.assertIsNone(core.snapshot)

    def test_rejected_inputs_are_logged_throttled(self):
        lines = []
        core = MissionCore(log=lambda level, msg: lines.append(msg))
        for _ in range(250):
            core.update_sectors([], 1.0)
        self.assertEqual(len([m for m in lines if 'sectores ignorados' in m]), 3)   # 1, 100, 200

    def test_avoider_refuses_degenerate_bins(self):
        with self.assertRaises(ValueError):
            VfhAvoider(n_bins=0)

    def test_non_finite_setpoint_is_held_not_sent(self):
        def feed(core, i, t, pos):
            normal(core, i, t, pos)
            if i == 250:
                core._carrot = [NAN, 0.0, -2.0]        # simula un bug interno
        core, bad = fly(feed, steps=300)
        self.assertEqual(bad, [])
        self.assertEqual(core.state, State.HOLD)
        self.assertTrue(all(math.isfinite(v) for v in core.setpoint))


class ConfigValidationTest(unittest.TestCase):

    def check(self, **overrides):
        params = dict(DEFAULTS)
        params.update(overrides)
        return validate_params(lambda n: params[n])

    def test_defaults_are_valid(self):
        self.assertEqual(self.check(), ([], []))

    def test_known_traps_are_errors(self):
        for bad in ({'avoid.emergency_distance': 0.9},        # >= radio + margen
                    {'control_rate': 0.0}, {'control_rate': 5.0},
                    {'safety.min_altitude': 5.0, 'safety.max_altitude': 3.0},
                    {'waypoints': [1.0, 2.0]},
                    {'map_mission.area': [10.0, 0.0, 0.0, 5.0]},
                    {'safety.geofence': [0.0, 0.0, 1.0]},
                    {'nav.yaw_mode': 'loco'}, {'cruise_speed': float('nan')},
                    {'avoid.stop_distance': 0.5}):
            errors, _ = self.check(**bad)
            self.assertTrue(errors, f'{bad} debería ser un error')

    def test_dangerous_but_legal_are_warnings(self):
        errors, warnings = self.check(**{'avoid.enabled': False})
        self.assertEqual(errors, [])
        self.assertTrue(any('SIN evasión' in w for w in warnings))
        # Alturas fuera de los límites: se recortan en vuelo (no impiden despegar).
        errors, warnings = self.check(**{'waypoints': [1.0, 2.0, 50.0], 'takeoff_altitude': 30.0})
        self.assertEqual(errors, [])
        self.assertEqual(len([w for w in warnings if 'se recorta' in w]), 2)

    def test_invalid_config_refuses_to_take_off(self):
        params = dict(DEFAULTS)
        params['avoid.emergency_distance'] = 2.0
        core = MissionCore(param=lambda n: params[n])
        core.update_vehicle((0.0, 0.0, 0.0), (0, 0, 0), 0.0, stamp=0.0)
        core.update_sectors(np.full(72, 10.0), 0.0)
        ok, msg = core.start_waypoints(0.0)
        self.assertFalse(ok)
        self.assertIn('configuración inválida', msg)
        self.assertEqual(core.state, State.IDLE)


if __name__ == '__main__':
    unittest.main()
