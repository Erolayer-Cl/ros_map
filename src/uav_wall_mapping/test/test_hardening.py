"""Entradas basura que pueden llegar en un vuelo real: nada de esto puede tirar
una excepción ni contaminar el mapa (ver docs/hardening.md)."""
import math
import os
import sys
import unittest

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from uav_wall_mapping.cloud import pointcloud2_to_xyz  # noqa: E402
from uav_wall_mapping.geometry import SensorMount, quat_from_yaw  # noqa: E402
from uav_wall_mapping.guard import CallbackGuard, GuardedNodeMixin  # noqa: E402
from uav_wall_mapping.mapper_core import MapperConfig, WallMapper  # noqa: E402
from uav_wall_mapping.pose_buffer import PoseBuffer  # noqa: E402
from uav_wall_mapping.sectors import to_px4_obstacle_distance  # noqa: E402

NAN = float('nan')
XYZ = [('x', 0, 7), ('y', 4, 7), ('z', 8, 7)]
POS = (0.0, 0.0, -2.0)
Q0 = quat_from_yaw(0.0)


def wall_cloud(n=1500, dist=4.0, seed=0):
    rng = np.random.default_rng(seed)
    return np.column_stack((np.full(n, dist), rng.uniform(-3, 3, n), rng.uniform(-1, 1, n)))


class CloudParsingTest(unittest.TestCase):

    def test_truncated_cloud_uses_whole_points_only(self):
        data = np.ones((10, 3), np.float32).tobytes()
        out = pointcloud2_to_xyz(XYZ, data, 12, 100, 1)       # dice 100, hay 10
        self.assertEqual(out.shape, (10, 3))
        self.assertEqual(pointcloud2_to_xyz(XYZ, b'\x00' * 7, 12, 3, 1).shape, (0, 3))

    def test_structural_errors_are_value_errors(self):
        data = np.zeros((10, 3), np.float32).tobytes()
        with self.assertRaises(ValueError):
            pointcloud2_to_xyz(XYZ, data, 8, 10, 1)           # z no entra en point_step
        with self.assertRaises(ValueError):
            pointcloud2_to_xyz([('x', 0, 99), ('y', 4, 7), ('z', 8, 7)], data, 12, 10, 1)
        with self.assertRaises(ValueError):
            pointcloud2_to_xyz([('x', -4, 7), ('y', 4, 7), ('z', 8, 7)], data, 12, 10, 1)


class PoseBufferTest(unittest.TestCase):

    def test_nan_samples_never_enter_the_history(self):
        pb = PoseBuffer()
        pb.add_position(1.0, (0.0, 0.0, 0.0))
        pb.add_position(1.5, (NAN, 0.0, 0.0))
        pb.add_position(NAN, (1.0, 1.0, 1.0))
        pb.add_position(2.0, (2.0, 0.0, 0.0))
        pb.add_attitude(1.0, Q0)
        pb.add_attitude(1.7, [NAN, 0, 0, 1])
        pb.add_attitude(2.0, Q0)
        pos, q = pb.lookup(1.5)
        np.testing.assert_allclose(pos, [1.0, 0.0, 0.0])
        self.assertTrue(np.all(np.isfinite(q)))
        self.assertIsNone(pb.lookup(NAN))


class Px4EncodingTest(unittest.TestCase):

    def test_nan_is_unknown_never_free(self):
        out = to_px4_obstacle_distance([NAN, math.inf, -1.0] + [3.0] * 69, 20.0)
        self.assertEqual(out[0], 65535)                       # sin dato
        self.assertEqual(out[1], 2001)                        # libre: max + 1
        self.assertEqual(out[2], 20)                          # min_distance
        self.assertTrue(all(0 <= v <= 65535 for v in out))


class MapperInputsTest(unittest.TestCase):

    def setUp(self):
        self.m = WallMapper(MapperConfig())
        self.m.update_trajectory(POS)
        self.mount = SensorMount(axes='flu')

    def _occupied(self):
        return int(np.count_nonzero(self.m.grid.occupied_mask()))

    def test_bad_pose_or_stamp_drops_the_whole_cloud(self):
        for pos, q, stamp in (((NAN, 0.0, -2.0), Q0, 1.0), (POS, [0, 0, 0, 0], 1.0),
                              (POS, [NAN, 0, 0, 1], 1.0), (POS, Q0, NAN),
                              ((math.inf, 0.0, -2.0), Q0, 1.0)):
            self.assertEqual(self.m.integrate_sensor(wall_cloud(), self.mount, pos, q, stamp), 0)
        self.assertEqual(self.m.rejected_clouds, 5)
        self.assertEqual(self._occupied(), 0)

    def test_nan_points_are_dropped_and_the_rest_used(self):
        pts = wall_cloud()
        pts[::5] = NAN
        pts[1::7, 2] = math.inf
        with np.errstate(all='raise'):                         # sin warnings de NaN
            self.m.integrate_sensor(pts, self.mount, POS, Q0, 1.0)
        self.assertGreater(self._occupied(), 10)
        info = self.m.sector_info(1.0, POS)
        self.assertFalse(np.any(np.isnan(info.ranges)))
        self.assertAlmostEqual(float(np.min(info.ranges)), 4.0, delta=0.3)

    def test_far_outlier_does_not_blow_up_the_grid(self):
        pts = np.vstack([wall_cloud(), [[1e9, 1e9, 0.0]]])
        self.m.integrate_sensor(pts, self.mount, POS, Q0, 1.0)
        self.assertLessEqual(max(self.m.grid.shape), self.m.grid.max_cells)

    def test_sectors_without_pose_are_not_fresh(self):
        self.m.integrate_sensor(wall_cloud(), self.mount, POS, Q0, 1.0)
        info = self.m.sector_info(1.0, (NAN, 0.0, -2.0))
        self.assertFalse(info.fresh)
        self.assertFalse(np.any(np.isnan(info.ranges)))

    def test_nan_trajectory_is_ignored(self):
        before = len(self.m.trajectory)
        self.m.update_trajectory((NAN, 0.0, -2.0))
        self.assertEqual(len(self.m.trajectory), before)


class MapperConfigTest(unittest.TestCase):

    def test_defaults_are_valid(self):
        self.assertEqual(MapperConfig().problems(), [])

    def test_invalid_config_fails_at_startup(self):
        for bad in ({'resolution': 0.0}, {'sensor_fov_deg': 0.0}, {'min_range': 50.0},
                    {'min_height': 9.0}, {'sector_bins': 0}, {'miss_logodds': 0.4},
                    {'self_filter_box': (0.5, -0.5, -0.5, 0.5, -0.3, 0.3)},
                    {'sector_timeout': float('nan')}):
            with self.assertRaises(ValueError, msg=str(bad)):
                WallMapper(MapperConfig(**bad))


class GuardTest(unittest.TestCase):

    def test_exception_is_counted_logged_and_swallowed(self):
        lines = []
        t = [0.0]
        guard = CallbackGuard('prueba', lines.append, period=5.0, clock=lambda: t[0])

        def boom(msg):
            raise ZeroDivisionError('x')
        wrapped = guard.wrap(boom)
        for i in range(50):
            t[0] = i * 0.1
            self.assertIsNone(wrapped(object()))
        self.assertEqual(guard.errors, 50)
        self.assertIn('Traceback', lines[0])                  # la 1.ª con traza
        self.assertLessEqual(len(lines), 2)                   # después, limitado a 1 cada 5 s

    def test_service_answers_failure_instead_of_hanging(self):
        class Res:
            success = True
            message = ''
        guard = CallbackGuard('srv', lambda _: None)

        def broken(req, res):
            raise RuntimeError('roto')
        res = guard.wrap(broken, service=True)(object(), Res())
        self.assertFalse(res.success)
        self.assertIn('roto', res.message)

    def test_mixin_wraps_everything_registered(self):
        class FakeNode:
            def __init__(self):
                self.registered = []

            def get_logger(self):
                class L:
                    def error(self, _):
                        pass
                return L()

            def create_subscription(self, msg_type, topic, cb, qos):
                self.registered.append(cb)

            def create_timer(self, period, cb):
                self.registered.append(cb)

            def create_service(self, srv, name, cb):
                self.registered.append(cb)

        class Node(GuardedNodeMixin, FakeNode):
            pass

        def bad(*_):
            raise ValueError('mal')
        node = Node()
        node.create_subscription(None, '/t', bad, 10)
        node.create_timer(0.1, bad)
        node.create_service(None, '/s', bad)
        for cb in node.registered:
            cb(object(), object()) if cb is node.registered[2] else cb(object())
        self.assertEqual(sorted(node.callback_errors().values()), [1, 1, 1])


if __name__ == '__main__':
    unittest.main()
