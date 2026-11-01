"""Pruebas de la grilla, los sectores, las transformaciones y la exportación."""
import math
import os
import sys
import tempfile
import unittest

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from uav_wall_mapping.cloud import (laserscan_to_xyz, pointcloud2_to_xyz,  # noqa: E402
                                    range_to_xyz, xyz_to_pointcloud2_data)
from uav_wall_mapping.export import save_map_bundle  # noqa: E402
from uav_wall_mapping.geometry import (SensorMount, quat_from_yaw, quat_slerp,  # noqa: E402
                                       yaw_from_quat)
from uav_wall_mapping.mapper_core import MapperConfig, WallMapper  # noqa: E402
from uav_wall_mapping.occupancy import OccupancyGrid2D  # noqa: E402
from uav_wall_mapping.pose_buffer import PoseBuffer  # noqa: E402
from uav_wall_mapping.sectors import (compute_sectors, free_ray_ends,  # noqa: E402
                                      to_px4_obstacle_distance)


class GeometryTest(unittest.TestCase):

    def test_flu_point_ahead_goes_north_when_facing_north(self):
        mount = SensorMount(axes='flu')
        world, origin = mount.to_world([[5.0, 0.0, 0.0]], (1.0, 2.0, -3.0), quat_from_yaw(0.0))
        np.testing.assert_allclose(world[0], [6.0, 2.0, -3.0], atol=1e-9)

    def test_flu_left_is_west_when_facing_north(self):
        mount = SensorMount(axes='flu')
        world, _ = mount.to_world([[0.0, 2.0, 0.0]], (0.0, 0.0, 0.0), quat_from_yaw(0.0))
        np.testing.assert_allclose(world[0], [0.0, -2.0, 0.0], atol=1e-9)

    def test_yaw_east_rotates_forward_to_east(self):
        mount = SensorMount(axes='frd')
        world, _ = mount.to_world([[3.0, 0.0, 0.0]], (0.0, 0.0, 0.0), quat_from_yaw(math.pi / 2))
        np.testing.assert_allclose(world[0], [0.0, 3.0, 0.0], atol=1e-9)

    def test_optical_axes(self):
        mount = SensorMount(axes='optical')   # z adelante, x derecha, y abajo
        world, _ = mount.to_world([[1.0, 0.5, 4.0]], (0.0, 0.0, 0.0), quat_from_yaw(0.0))
        np.testing.assert_allclose(world[0], [4.0, 1.0, 0.5], atol=1e-9)

    def test_mount_offset_rotates_with_vehicle(self):
        mount = SensorMount(position_frd=(0.5, 0.0, 0.0), axes='frd')
        _, origin = mount.to_world(np.zeros((0, 3)), (0.0, 0.0, 0.0), quat_from_yaw(math.pi / 2))
        np.testing.assert_allclose(origin, [0.0, 0.5, 0.0], atol=1e-9)


class CloudTest(unittest.TestCase):

    def test_pointcloud2_roundtrip(self):
        pts = np.array([[1.0, 2.0, 3.0], [-1.5, 0.25, 9.0]])
        fields, data, step = xyz_to_pointcloud2_data(pts, rgb=np.array([[255, 0, 0], [0, 0, 255]]))
        out = pointcloud2_to_xyz([(f[0], f[1], f[2]) for f in fields], data, step, len(pts), 1)
        np.testing.assert_allclose(out, pts, atol=1e-6)

    def test_nan_points_are_dropped(self):
        pts = np.array([[1.0, 2.0, 3.0], [np.nan, 0.0, 0.0]])
        fields, data, step = xyz_to_pointcloud2_data(pts)
        out = pointcloud2_to_xyz([(f[0], f[1], f[2]) for f in fields], data, step, 2, 1)
        self.assertEqual(out.shape, (1, 3))

    def test_laserscan_skips_max_range(self):
        out = laserscan_to_xyz([1.0, float('inf'), 10.0], 0.0, math.pi / 2, 0.1, 10.0)
        np.testing.assert_allclose(out, [[1.0, 0.0, 0.0]], atol=1e-9)

    def test_range_rejects_garbage(self):
        self.assertEqual(range_to_xyz(-0.1, 0.2, 25.0).shape, (0, 3))
        self.assertEqual(range_to_xyz(25.0, 0.2, 25.0).shape, (0, 3))
        self.assertEqual(range_to_xyz(3.0, 0.2, 25.0).shape, (1, 3))


class GridTest(unittest.TestCase):

    def test_hits_become_occupied_and_rays_free(self):
        grid = OccupancyGrid2D(resolution=0.2, initial_size=10.0)
        wall = np.column_stack((np.full(50, 4.0), np.linspace(-2, 2, 50)))
        for _ in range(4):
            grid.update((0.0, 0.0), wall, free_ray_ends((0.0, 0.0), wall))
        snap = grid.snapshot()
        self.assertGreaterEqual(snap.value_at(4.05, 0.0), 65)
        self.assertLessEqual(snap.value_at(2.0, 0.0), 30)
        self.assertEqual(snap.value_at(-4.0, 3.0), -1)

    def test_grid_grows(self):
        grid = OccupancyGrid2D(resolution=0.2, initial_size=4.0)
        far = np.array([[30.0, -25.0]])
        grid.update((0.0, 0.0), far, far)
        self.assertGreaterEqual(grid.snapshot().value_at(30.05, -24.95), 50)

    def test_empty_sector_does_not_carve_through_neighbor_wall(self):
        """Un sector sin impacto entre dos con pared no se talla a través de ella."""
        hits = np.array([[3.0, -0.05], [3.0, 0.05]])
        ends = free_ray_ends((0.0, 0.0), hits, n_bins=360, no_hit_range=8.0)
        reach = np.hypot(ends[:, 0], ends[:, 1])
        ahead = np.abs(np.arctan2(ends[:, 1], ends[:, 0])) < math.radians(4)
        self.assertTrue(np.all(reach[ahead] <= 3.0))


class SectorsTest(unittest.TestCase):

    def test_nearest_per_sector(self):
        ranges, counts = compute_sectors([[5.0, 0.0], [2.0, 0.0], [0.0, 3.0]], n_bins=72)
        self.assertAlmostEqual(ranges[0], 2.0)
        self.assertAlmostEqual(ranges[18], 3.0)     # 90° = Este
        self.assertTrue(math.isinf(ranges[36]))
        self.assertEqual(counts[0], 2)

    def test_px4_encoding(self):
        distances = to_px4_obstacle_distance([1.5, float('inf')], max_distance_m=20.0)
        self.assertEqual(distances, [150, 2001])


class PoseBufferTest(unittest.TestCase):

    def test_slerp_halfway(self):
        q = quat_slerp(quat_from_yaw(0.0), quat_from_yaw(1.0), 0.5)
        self.assertAlmostEqual(yaw_from_quat(q), 0.5, places=6)

    def test_interpolates_position_and_attitude(self):
        buf = PoseBuffer(horizon=2.0)
        for k in range(11):
            t = 10.0 + 0.1 * k
            buf.add_position(t, (k * 0.3, 0.0, -2.0))       # 3 m/s al norte
            buf.add_attitude(t, quat_from_yaw(0.1 * k))
        pos, q = buf.lookup(10.25)
        self.assertAlmostEqual(pos[0], 0.75, places=6)
        self.assertAlmostEqual(yaw_from_quat(q), 0.25, places=6)

    def test_outside_history_is_rejected(self):
        buf = PoseBuffer(horizon=1.0, max_extrapolation=0.05)
        for k in range(30):
            buf.add_position(k * 0.1, (0.0, 0.0, 0.0))
            buf.add_attitude(k * 0.1, quat_from_yaw(0.0))
        self.assertIsNone(buf.lookup(0.5))           # más viejo que el historial
        self.assertIsNone(buf.lookup(3.5))           # mucho más nuevo que la última pose
        self.assertIsNotNone(buf.lookup(2.92))       # apenas más nuevo: la última


class SectorTimingTest(unittest.TestCase):

    def wall(self, north):
        e = np.linspace(-3, 3, 120)
        return np.column_stack((np.full_like(e, north), e, np.full_like(e, -2.0)))

    def test_sectors_follow_current_pose(self):
        """Una nube vieja se reutiliza contra la pose ACTUAL: si el dron avanzó
        1 m hacia la pared, la pared está 1 m más cerca."""
        mapper = WallMapper(MapperConfig(sectors_from_map=False))
        mapper.set_home(0.0)
        mapper.integrate(self.wall(6.0), (0.0, 0.0, -2.0), (0.0, 0.0, -2.0), stamp=5.0)
        info = mapper.sector_info(5.2, (1.0, 0.0, -2.0))
        self.assertTrue(info.fresh)
        self.assertEqual(info.stamp, 5.0)
        self.assertAlmostEqual(float(info.ranges[0]), 5.0, delta=0.05)
        self.assertFalse(mapper.sector_info(5.6, (1.0, 0.0, -2.0)).fresh)

    def test_empty_cloud_is_not_fresh_and_does_not_carve(self):
        mapper = WallMapper(MapperConfig())
        mapper.set_home(0.0)
        mapper.integrate(np.zeros((0, 3)), (0.0, 0.0, -2.0), (0.0, 0.0, -2.0), stamp=1.0)
        self.assertFalse(mapper.has_fresh_source(1.0))
        self.assertEqual(mapper.starved_clouds, 1)
        self.assertEqual(int(np.count_nonzero(mapper.grid.known)), 0)

    def test_integrate_sensor_uses_given_pose(self):
        mapper = WallMapper(MapperConfig(sectors_from_map=False))
        mapper.set_home(0.0)
        mount = SensorMount(axes='frd')
        e = np.linspace(-3, 3, 120)
        pts = np.column_stack((np.full_like(e, 4.0), e, np.zeros_like(e)))   # 4 m adelante
        mapper.integrate_sensor(pts, mount, (2.0, 0.0, -2.0), quat_from_yaw(0.0), 3.0)
        # Pared en N = 6 (pose de la captura); el dron ahora está en N = 3.
        info = mapper.sector_info(3.1, (3.0, 0.0, -2.0))
        self.assertAlmostEqual(float(info.ranges[0]), 3.0, delta=0.05)


class MapperExportTest(unittest.TestCase):

    def test_mapper_and_bundle(self):
        mapper = WallMapper(MapperConfig())
        mapper.set_home(0.0)
        e = np.linspace(-5, 5, 200)
        for height in np.linspace(0.5, 3.5, 7):
            wall = np.column_stack((np.full_like(e, 8.0), e, np.full_like(e, -height)))
            mapper.integrate(wall, (0.0, 0.0, -2.0), (0.0, 0.0, -2.0), stamp=0.0)
            mapper.integrate(wall, (0.0, 0.0, -2.0), (0.0, 0.0, -2.0), stamp=0.1)
        mapper.update_trajectory((0.0, 0.0, -2.0))
        walls = mapper.extract_walls()
        self.assertEqual(len(walls), 1)
        self.assertAlmostEqual(walls[0].height_max, 3.5, delta=0.4)
        sectors = mapper.sectors(0.1, (0.0, 0.0, -2.0))
        self.assertAlmostEqual(float(sectors[0]), 8.0, delta=0.3)
        with tempfile.TemporaryDirectory() as tmp:
            files = save_map_bundle(tmp, 'prueba', mapper)
            for f in files:
                self.assertTrue(os.path.getsize(f) > 0, f)
            with open(os.path.join(tmp, 'prueba.png'), 'rb') as fh:
                self.assertEqual(fh.read(8), b'\x89PNG\r\n\x1a\n')


if __name__ == '__main__':
    unittest.main()
