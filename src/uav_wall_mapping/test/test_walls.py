"""Pruebas del extractor de paredes con geometrías sintéticas (sin ROS).

    python3 -m unittest discover -s src/uav_wall_mapping/test -t src/uav_wall_mapping
"""
import math
import os
import sys
import unittest

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from uav_wall_mapping.walls import FLAT, IRREGULAR, OBSTACLE, WallExtractor, fit_plane  # noqa: E402

RES = 0.15


def raster(points, viewer=(0.0, 0.0)):
    """Celdas ocupadas (centros) + dirección desde la que se vieron."""
    cells = np.unique(np.floor(np.asarray(points) / RES).astype(int), axis=0)
    centers = (cells + 0.5) * RES
    rel = centers - np.asarray(viewer)
    return centers, rel / np.hypot(*rel.T)[:, None]


def segment(a, b, step=0.05):
    a, b = np.asarray(a, float), np.asarray(b, float)
    n = int(np.linalg.norm(b - a) / step) + 1
    return a + np.linspace(0.0, 1.0, n)[:, None] * (b - a)


def kinds(walls):
    return sorted(w.kind for w in walls)


class WallExtractorTest(unittest.TestCase):

    def setUp(self):
        self.ex = WallExtractor(resolution=RES)

    def test_straight_wall_is_flat(self):
        rng = np.random.default_rng(0)
        pts = segment((10, -6), (10, 6)) + rng.normal(0, 0.03, (241, 2))
        walls = self.ex.extract(*raster(pts))
        self.assertEqual(kinds(walls), [FLAT])
        self.assertAlmostEqual(walls[0].length, 12.0, delta=0.4)

    def test_diagonal_wall_is_flat(self):
        walls = self.ex.extract(*raster(segment((3, 3), (10, 9))))
        self.assertEqual(kinds(walls), [FLAT])

    def test_corner_gives_two_flat_walls(self):
        pts = np.vstack((segment((10, -6), (10, 6)), segment((10, 6), (2, 6))))
        walls = self.ex.extract(*raster(pts))
        self.assertEqual(kinds(walls), [FLAT, FLAT])

    def test_closed_room_gives_four_walls(self):
        pts = np.vstack((segment((-5, -5), (-5, 5)), segment((-5, 5), (5, 5)),
                         segment((5, 5), (5, -5)), segment((5, -5), (-5, -5))))
        walls = self.ex.extract(*raster(pts))
        self.assertEqual(kinds(walls), [FLAT] * 4)

    def test_zigzag_wall_is_irregular(self):
        pts = np.vstack([segment((10 + 0.5 * (i % 2), -6 + i), (10 + 0.5 * ((i + 1) % 2), -5 + i))
                         for i in range(12)])
        walls = self.ex.extract(*raster(pts))
        self.assertEqual(kinds(walls), [IRREGULAR])

    def test_curved_wall_is_irregular(self):
        arc = np.array([[8 * math.cos(t), 8 * math.sin(t)] for t in np.linspace(-0.8, 0.8, 400)])
        walls = self.ex.extract(*raster(arc))
        self.assertEqual(kinds(walls), [IRREGULAR])

    def test_wall_with_protrusions_is_irregular(self):
        base = [segment((10, -6), (10, 6))]
        bumps = [segment((9.6, -5 + 2 * i), (9.6, -4.4 + 2 * i)) for i in range(6)]
        sides = [segment((10, -5 + 2 * i), (9.6, -5 + 2 * i)) for i in range(6)]
        walls = self.ex.extract(*raster(np.vstack(base + bumps + sides)))
        self.assertEqual(kinds(walls), [IRREGULAR])

    def test_thick_wall_seen_from_both_sides_is_flat(self):
        """Las dos caras de una pared de 30 cm, cada una vista desde su lado."""
        front, views_f = raster(segment((10, -6), (10, 6)), viewer=(0, 0))
        back, views_b = raster(segment((10.3, -6), (10.3, 6)), viewer=(20, 0))
        walls = self.ex.extract(np.vstack((front, back)), np.vstack((views_f, views_b)))
        self.assertEqual(kinds(walls), [FLAT])

    def test_pillar_is_obstacle(self):
        circle = np.array([[5 + 0.4 * math.cos(t), 5 + 0.4 * math.sin(t)]
                           for t in np.linspace(0, 2 * math.pi, 100)])
        walls = self.ex.extract(*raster(circle))
        self.assertEqual(kinds(walls), [OBSTACLE])

    def test_door_gap_is_kept(self):
        """Una puerta de 1 m no se tiene que "cerrar" uniendo las dos paredes."""
        pts = np.vstack((segment((0, -6), (0, -0.5)), segment((0, 0.5), (0, 6))))
        walls = self.ex.extract(*raster(pts))
        self.assertEqual(kinds(walls), [FLAT, FLAT])

    def test_plane_tilt(self):
        rng = np.random.default_rng(1)
        h = rng.uniform(0.3, 4.0, 500)
        e = rng.uniform(-5, 5, 500)
        n = 10.0 + h * math.tan(math.radians(20))      # pared inclinada 20°
        plane = fit_plane(np.column_stack((n, e, h)))
        self.assertAlmostEqual(plane.tilt_deg, 20.0, delta=1.0)


if __name__ == '__main__':
    unittest.main()
