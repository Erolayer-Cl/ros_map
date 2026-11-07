"""Pruebas de la grilla, los sectores, las transformaciones y la exportación."""
import math
import os
import sys
import tempfile
import unittest

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from uav_wall_mapping.cloud import (isolated_mask, laserscan_to_xyz,  # noqa: E402
                                    outside_box, pointcloud2_to_xyz, range_to_xyz,
                                    xyz_to_pointcloud2_data)
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

    def test_no_return_rays_do_not_erase_seen_obstacles(self):
        """Un cable visto de vez en cuando no se borra con los rayos sin eco."""
        grid = OccupancyGrid2D(resolution=0.15, initial_size=20.0)
        cable = np.array([[5.0, 0.0]])
        hit_ends, weak = free_ray_ends((0.0, 0.0), np.zeros((0, 2)), n_bins=360,
                                       no_hit_range=8.0, split=True)
        grid.update((0.0, 0.0), cable, cable, stamp=0.0)      # una detección
        for k in range(1, 20):                                # 1.9 s de escaneos sin eco
            grid.update((0.0, 0.0), np.zeros((0, 2)), hit_ends, stamp=0.1 * k,
                        weak_ends_ne=weak)
        snap = grid.snapshot()
        self.assertGreaterEqual(snap.value_at(5.05, 0.05), 65)
        self.assertLessEqual(snap.value_at(3.0, 0.0), 30)    # lo desconocido sí se libera
        # Si no se re-confirma en `weak_protect_sec`, se olvida (lo que se fue).
        for k in range(20, 60):
            grid.update((0.0, 0.0), np.zeros((0, 2)), hit_ends, stamp=0.1 * k,
                        weak_ends_ne=weak)
        self.assertLess(grid.snapshot().value_at(5.05, 0.05), 65)

    def test_traversed_space_becomes_known_without_erasing_walls(self):
        grid = OccupancyGrid2D(resolution=0.15, initial_size=10.0)
        wall = np.array([[0.45, 0.0]])
        for k in range(3):
            grid.update((0.0, 0.0), wall, wall, stamp=float(k))
        grid.mark_traversed((0.0, 0.0), 0.8)
        snap = grid.snapshot()
        self.assertGreaterEqual(snap.value_at(0.5, 0.05), 65)     # la pared sigue
        self.assertEqual(snap.value_at(-0.5, 0.3), 50)             # lo recorrido: conocido, neutro
        self.assertEqual(snap.value_at(-2.0, 0.0), -1)             # lo lejano sigue desconocido

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


class FilterTest(unittest.TestCase):

    def test_kth_nearest_ignores_single_spurious_point(self):
        wall = [[5.0, -0.05], [5.0, 0.0], [5.0, 0.05]]
        ranges, _ = compute_sectors(wall + [[1.0, 0.0]], n_bins=72, min_points=2)
        self.assertAlmostEqual(ranges[0], 5.0, places=6)        # el polvo a 1 m no cuenta
        ranges, _ = compute_sectors([[1.0, 0.0], [1.02, 0.01]] + wall, n_bins=72, min_points=2)
        self.assertAlmostEqual(ranges[0], 1.02, places=2)       # dos puntos juntos sí

    def test_isolated_points_are_noise(self):
        rng = np.random.default_rng(1)
        e = np.linspace(-2, 2, 80)
        z = np.linspace(-1, 1, 10)
        ee, zz = np.meshgrid(e, z)
        wall = np.column_stack((np.full(ee.size, 4.0), ee.ravel(), zz.ravel()))
        dust = rng.uniform(-3, 3, size=(5, 3)) * np.array([0.3, 1.0, 1.0]) + np.array([1.5, 0, 0])
        far = np.array([[20.0, 0.0, 0.0]])                      # suelto pero lejos: se queda
        pts = np.vstack((wall, dust, far))
        noisy = isolated_mask(pts, voxel=0.25, min_points=2, max_range=8.0)
        self.assertFalse(np.any(noisy[:len(wall)]))
        self.assertTrue(np.all(noisy[len(wall):len(wall) + 5]))
        self.assertFalse(noisy[-1])

    def test_repeated_isolated_point_is_kept(self):
        """Un punto suelto que se repite en el mismo lugar (un cable fino) pasa
        el filtro desde la segunda nube; uno al azar (polvo), no."""
        mapper = WallMapper(MapperConfig(sectors_from_map=False))
        mapper.set_home(0.0)
        mount = SensorMount(axes='frd')
        e = np.linspace(-2, 2, 80)
        wall = np.column_stack((np.full_like(e, 9.0), e, np.zeros_like(e)))
        cable = np.array([[4.0, 0.0, 0.0]])
        before = []
        for k in range(3):
            n0 = mapper.outliers
            mapper.integrate_sensor(np.vstack((wall, cable)), mount, (0.0, 0.0, -2.0),
                                    quat_from_yaw(0.0), 0.1 * k)
            before.append(mapper.outliers - n0)
        self.assertEqual(before, [1, 0, 0])

    def test_body_box(self):
        pts = np.array([[0.3, 0.3, 0.0], [0.6, 0.0, 0.0], [0.0, 0.0, 0.5]])
        keep = outside_box(pts, (-0.5, 0.5, -0.5, 0.5, -0.35, 0.4))
        self.assertEqual(keep.tolist(), [False, True, True])

    def test_self_hits_never_reach_sectors(self):
        """Puntos en las hélices (a 0.45 m del centro, dentro de min_range no)
        no llegan a los sectores: la caja del cuerpo los saca."""
        mapper = WallMapper(MapperConfig(min_range=0.3, sectors_from_map=False))
        mapper.set_home(0.0)
        mount = SensorMount(position_frd=(0.05, 0.0, -0.12), axes='frd')
        props = np.array([[0.32, 0.32, 0.1], [-0.32, 0.32, 0.1], [0.32, -0.32, 0.1],
                          [-0.32, -0.32, 0.1]]) - mount.position_frd
        e = np.linspace(-2, 2, 60)
        wall = np.column_stack((np.full_like(e, 6.0), e, np.zeros_like(e)))
        for k in range(3):
            mapper.integrate_sensor(np.vstack((props, props * 1.01, wall)), mount,
                                    (0.0, 0.0, -2.0), quat_from_yaw(0.0), 0.1 * k)
        info = mapper.sector_info(0.25, (0.0, 0.0, -2.0))
        self.assertGreater(float(np.min(info.ranges)), 5.0)
        self.assertGreater(mapper.self_hits, 0)


class SeenSectorsTest(unittest.TestCase):

    def test_camera_marks_only_its_fov(self):
        mapper = WallMapper(MapperConfig(sensor_fov_deg=87.0))
        mapper.set_home(0.0)
        e = np.linspace(-1, 1, 60)
        wall = np.column_stack((np.full_like(e, 4.0), e, np.full_like(e, -2.0)))
        mapper.integrate(wall, (0.0, 0.0, -2.0), (0.0, 0.0, -2.0), stamp=2.0, sensor_heading=0.0)
        info = mapper.sector_info(2.1, (0.0, 0.0, -2.0))
        self.assertEqual(float(info.seen_age[0]), 0.0)        # adelante: visto
        self.assertTrue(math.isinf(info.seen_age[36]))       # atrás: nunca
        self.assertTrue(math.isinf(info.seen_age[18]))       # 90° al costado: fuera de 87°

    def test_px4_unknown_is_uint16_max(self):
        distances = to_px4_obstacle_distance([1.5, float('inf'), float('inf')], 20.0,
                                             unknown_mask=[False, False, True])
        self.assertEqual(distances, [150, 2001, 65535])


class ForgetTest(unittest.TestCase):

    def box_points(self, north, east):
        n, e, z = np.meshgrid(np.linspace(north - 0.3, north + 0.3, 6),
                              np.linspace(east - 0.3, east + 0.3, 6), np.linspace(-1.5, -2.5, 4))
        return np.column_stack((n.ravel(), e.ravel(), z.ravel()))

    def test_transient_obstacle_is_forgotten_but_wall_is_not(self):
        cfg = MapperConfig(forget_after=5.0, forget_max_hits=15, no_hit_free_range=0.0)
        mapper = WallMapper(cfg)
        mapper.set_home(0.0)
        e = np.linspace(-3, 3, 100)
        wall = np.vstack([np.column_stack((np.full_like(e, 8.0), e, np.full_like(e, -h)))
                          for h in (1.5, 2.0, 2.5)])
        t = 0.0
        for k in range(5):                     # caja vista 5 nubes en (4, -5)
            mapper.integrate(np.vstack((wall, self.box_points(4.0, -5.0))), (0, 0, -2.0),
                             (0, 0, -2.0), stamp=t)
            t += 0.1
        self.assertGreaterEqual(mapper.snapshot().value_at(4.0, -5.0), 65)
        while t < 7.0:                         # se fue; la pared se sigue viendo
            mapper.integrate(wall, (0, 0, -2.0), (0, 0, -2.0), stamp=t)
            t += 0.1
        snap = mapper.snapshot()
        self.assertLess(snap.value_at(4.0, -5.0), 0)          # desconocido otra vez
        self.assertGreaterEqual(snap.value_at(8.05, 0.0), 65)  # la pared sigue
        self.assertGreater(mapper.forgotten, 0)


class HeightBandTest(unittest.TestCase):

    def test_ceiling_is_not_a_2d_obstacle(self):
        """El techo (4 m, dron a 2 m) queda en el mapa 3D pero no en la grilla 2D."""
        mapper = WallMapper(MapperConfig(max_height_above_drone=1.0))
        mapper.set_home(0.0)
        n, e = np.meshgrid(np.linspace(-6, 6, 60), np.linspace(-6, 6, 60))
        ceiling = np.column_stack((n.ravel(), e.ravel(), np.full(n.size, -4.0)))
        e1 = np.linspace(-3, 3, 100)
        wall = np.vstack([np.column_stack((np.full_like(e1, 5.0), e1, np.full_like(e1, -h)))
                          for h in (1.0, 2.0, 3.0)])
        for k in range(3):
            mapper.integrate(np.vstack((ceiling, wall)), (0.0, 0.0, -2.0), (0.0, 0.0, -2.0),
                             stamp=0.1 * k)
        snap = mapper.snapshot()
        self.assertGreaterEqual(snap.value_at(5.05, 0.0), 65)        # la pared sí
        self.assertLess(snap.value_at(-3.0, 3.0), 65)                # el techo no
        occupied = mapper.grid.occupied_points()
        self.assertTrue(np.all(np.abs(occupied[:, 0] - 5.0) < 0.5))
        self.assertGreater(len(mapper.voxels), 1000)                 # en 3D está todo

    def test_relative_band_can_be_disabled(self):
        mapper = WallMapper(MapperConfig(max_height_above_drone=0.0, max_height=8.0))
        mapper.set_home(0.0)
        pts = np.column_stack((np.full(40, 3.0), np.linspace(-1, 1, 40), np.full(40, -5.0)))
        mapper.integrate(pts, (0.0, 0.0, -2.0), (0.0, 0.0, -2.0), stamp=0.0)
        self.assertGreaterEqual(mapper.snapshot().value_at(3.05, 0.0), 50)


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
        state = mapper.capture_state()
        mapper.integrate(wall, (0.0, 0.0, -2.0), (0.0, 0.0, -2.0), stamp=0.2)  # sigue mapeando
        self.assertEqual(len(mapper.extract_walls(state)), 1)
        with tempfile.TemporaryDirectory() as tmp:
            save_map_bundle(tmp, 'copia', state)
            self.assertTrue(os.path.getsize(os.path.join(tmp, 'copia.png')) > 0)
            files = save_map_bundle(tmp, 'prueba', mapper)
            for f in files:
                self.assertTrue(os.path.getsize(f) > 0, f)
            with open(os.path.join(tmp, 'prueba.png'), 'rb') as fh:
                self.assertEqual(fh.read(8), b'\x89PNG\r\n\x1a\n')


if __name__ == '__main__':
    unittest.main()
