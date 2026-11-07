"""Pruebas de la máquina de estados (sin ROS): armado, servicios y seguridad."""
import os
import sys
import unittest

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(HERE)), 'uav_wall_mapping'))

from uav_mission.mission_core import DEFAULTS, MissionCore, State  # noqa: E402


class FakeVehicle:
    """Lo mínimo para que la misión arme y despegue."""

    def __init__(self, core):
        self.core = core
        self.armed = False
        self.offboard = False
        self.pos = np.zeros(3)
        self.sectors_kwargs = {}
        self.pose_valid = True
        self.sectors_on = True

    def tick(self, t):
        if self.core.stream_offboard and self.core.setpoint and self.armed:
            target = np.array(self.core.setpoint[:3])
            step = np.clip(target - self.pos, -0.1, 0.1)
            self.pos = self.pos + step
        self.core.update_vehicle(self.pos, (0.0, 0.0, 0.0), 0.0, valid=self.pose_valid, stamp=t)
        self.core.update_status(self.armed, self.offboard)
        if self.sectors_on:
            self.core.update_sectors(np.full(72, float('inf')), t, **self.sectors_kwargs)
        self.core.step(t)
        for cmd in self.core.pop_commands():
            if cmd == 'arm':
                self.armed = True
            elif cmd == 'mode_offboard':
                self.offboard = True
            elif cmd == 'land':
                self.offboard = False


def make_core(**overrides):
    params = dict(DEFAULTS)
    params.update(overrides)
    return MissionCore(param=lambda name: params[name])


class MissionCoreTest(unittest.TestCase):

    def test_refuses_without_position(self):
        core = make_core()
        ok, msg = core.start_waypoints(0.0)
        self.assertFalse(ok)
        self.assertIn('posición', msg)

    def test_refuses_without_obstacle_data(self):
        core = make_core()
        core.update_vehicle((0, 0, 0), (0, 0, 0), 0.0)
        ok, msg = core.start_waypoints(0.0)
        self.assertFalse(ok)
        self.assertIn('obstáculos', msg)

    def test_arm_and_takeoff(self):
        core = make_core()
        vehicle = FakeVehicle(core)
        vehicle.tick(0.0)
        ok, _ = core.start_waypoints(0.0)
        self.assertTrue(ok)
        t = 0.0
        while t < 10.0 and core.state in (State.ARMING, State.TAKEOFF):
            t += 0.05
            vehicle.tick(t)
        self.assertTrue(vehicle.armed and vehicle.offboard)
        self.assertEqual(core.state, State.NAVIGATE)
        self.assertAlmostEqual(-vehicle.pos[2], DEFAULTS['takeoff_altitude'], delta=0.35)

    def test_pilot_takeover_is_respected(self):
        core = make_core()
        vehicle = FakeVehicle(core)
        vehicle.tick(0.0)
        core.start_waypoints(0.0)
        t = 0.0
        while t < 10.0 and core.state != State.NAVIGATE:
            t += 0.05
            vehicle.tick(t)
        vehicle.offboard = False          # el piloto pasó a Position desde la radio
        vehicle.tick(t + 0.05)
        self.assertEqual(core.state, State.PREEMPTED)
        self.assertFalse(core.stream_offboard)

    def test_pause_and_resume(self):
        core = make_core()
        vehicle = FakeVehicle(core)
        vehicle.tick(0.0)
        core.start_waypoints(0.0)
        t = 0.0
        while t < 10.0 and core.state != State.NAVIGATE:
            t += 0.05
            vehicle.tick(t)
        ok, _ = core.pause(t)
        self.assertTrue(ok)
        self.assertEqual(core.state, State.HOLD)
        ok, _ = core.resume(t)
        self.assertTrue(ok)
        self.assertEqual(core.state, State.NAVIGATE)

    def test_turns_before_moving_out_of_view(self):
        """scaleToFOV: con una cámara mirando al norte y el objetivo al sur, no
        se avanza: primero se gira hasta tener el avance a la vista."""
        core = make_core(**{'waypoints': [-15.0, 0.0, 3.0]})
        vehicle = FakeVehicle(core)
        seen_age = np.full(72, float('inf'))
        seen_age[63:72] = 0.0
        seen_age[0:10] = 0.0          # sólo se ve ±45° alrededor del norte
        vehicle.sectors_kwargs = {'seen_age': seen_age}
        vehicle.tick(0.0)
        core.start_waypoints(0.0)
        t = 0.0
        while t < 10.0 and core.state in (State.ARMING, State.TAKEOFF):
            t += 0.05
            vehicle.tick(t)
        north0 = vehicle.pos[0]
        for _ in range(20):
            t += 0.05
            vehicle.tick(t)
        self.assertAlmostEqual(vehicle.pos[0], north0, delta=0.05)    # no se movió
        self.assertGreater(abs(core._yaw_sp), 0.5)                     # pero gira

    def fly_to_navigate(self, core, vehicle, t=0.0):
        vehicle.tick(t)
        core.start_waypoints(t)
        while t < 10.0 and core.state != State.NAVIGATE:
            t += 0.05
            vehicle.tick(t)
        self.assertEqual(core.state, State.NAVIGATE)
        return t

    def test_pose_loss_brakes_then_lands(self):
        core = make_core()
        vehicle = FakeVehicle(core)
        t = self.fly_to_navigate(core, vehicle)
        vehicle.pose_valid = False            # xy_valid = false en vuelo
        landed_at = None
        for _ in range(80):
            t += 0.05
            vehicle.tick(t)
            if core.state == State.LAND and landed_at is None:
                landed_at = t
        self.assertIsNotNone(landed_at)
        self.assertIsNone(core.velocity_setpoint)

    def test_sensor_loss_lands_after_timeout(self):
        core = make_core(**{'avoid.sensor_loss_land_after': 3.0})
        vehicle = FakeVehicle(core)
        t = self.fly_to_navigate(core, vehicle)
        vehicle.sectors_on = False
        for _ in range(40):                   # 2 s: frena pero no aterriza
            t += 0.05
            vehicle.tick(t)
        self.assertNotEqual(core.state, State.LAND)
        for _ in range(40):
            t += 0.05
            vehicle.tick(t)
        self.assertEqual(core.state, State.LAND)

    def test_failed_return_lands(self):
        core = make_core()
        vehicle = FakeVehicle(core)
        t = self.fly_to_navigate(core, vehicle)
        core._go_home(t, 'prueba')
        core._on_goal_failed(t, 'rodeado de obstáculos')
        self.assertEqual(core.state, State.LAND)
        self.assertIn('land', core.pop_commands())

    def test_altitude_is_clamped(self):
        core = make_core(**{'waypoints': [5.0, 0.0, 50.0], 'safety.max_altitude': 10.0})
        vehicle = FakeVehicle(core)
        t = self.fly_to_navigate(core, vehicle)
        self.assertAlmostEqual(core.home[2] - core._goal[2], 10.0)

    def test_geofence_clamps_reference(self):
        core = make_core(**{'safety.geofence': [-2.0, -2.0, 2.0, 2.0],
                            'waypoints': [15.0, 0.0, 3.0]})
        vehicle = FakeVehicle(core)
        t = self.fly_to_navigate(core, vehicle)
        for _ in range(200):
            t += 0.05
            vehicle.tick(t)
        self.assertLessEqual(core._carrot[0], core.home[0] + 2.0 + 1e-6)

    def test_bad_area_is_rejected(self):
        core = make_core(**{'map_mission.area': [0.0, 0.0, 0.0, 5.0]})
        core.update_vehicle((0, 0, 0), (0, 0, 0), 0.0)
        core.update_sectors(np.full(72, float('inf')), 0.0)
        ok, _ = core.start_mapping(0.0)
        self.assertFalse(ok)


if __name__ == '__main__':
    unittest.main()
