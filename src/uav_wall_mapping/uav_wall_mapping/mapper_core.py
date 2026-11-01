#!/usr/bin/env python3
"""Núcleo del mapeo: nubes en el mundo -> grilla 2D, vóxeles 3D, sectores y paredes.

Python puro (sin rclpy). `mapper_node.py` es sólo el adaptador ROS: convierte
mensajes, llama a esto y publica. Las pruebas de lazo cerrado (`uav_sim`) usan
esta misma clase sin ROS.
"""

import math
import time
from dataclasses import dataclass, field

import numpy as np

from uav_wall_mapping.occupancy import OccupancyGrid2D, VoxelMap3D
from uav_wall_mapping.sectors import INF, compute_sectors, free_ray_ends, merge_sectors
from uav_wall_mapping.walls import WallExtractor, WallTracker, annotate_3d


@dataclass
class MapperConfig:
    # --- grilla 2D ---
    resolution: float = 0.15          # m por celda
    initial_size: float = 40.0        # m (crece sola)
    max_size: float = 400.0           # m: tope de crecimiento (RAM de la Jetson)
    min_height: float = 0.3           # m sobre el despegue: lo de abajo es piso
    max_height: float = 8.0           # m: lo de arriba no se dibuja (techos, cables)
    no_hit_free_range: float = 6.0    # m que se dan por libres en sectores sin impacto
    free_ray_bins: int = 360          # sectores del tallado de espacio libre: NO más finos
    #                                   que la resolución horizontal del sensor
    free_ray_neighbors: int = 3       # sectores vecinos que limitan el tallado de un sector vacío
    sensor_fov_deg: float = 360.0     # campo horizontal del sensor (87 para una RealSense)
    min_range: float = 0.5            # m: descarta impactos contra el propio dron
    max_range: float = 40.0           # m
    hit_logodds: float = 0.85
    miss_logodds: float = -0.4
    # --- mapa 3D ---
    voxel_resolution: float = 0.2
    voxel_min_height: float = 0.15    # m: no guardar el piso en la vista 3D
    voxel_min_hits: int = 2
    # --- sectores para la evasión ---
    sector_bins: int = 72
    sector_range: float = 15.0
    sector_band_below: float = 0.7    # m por debajo del dron que cuentan como choque
    sector_band_above: float = 0.7
    sector_min_points: int = 1
    sector_timeout: float = 0.6       # s: una fuente más vieja que esto no cuenta
    sectors_from_map: bool = True     # sumar lo que ya está en el mapa (memoria)
    # --- paredes ---
    walls: dict = field(default_factory=dict)   # kwargs para WallExtractor
    trajectory_step: float = 0.3      # m entre puntos guardados de la trayectoria


class WallMapper:

    def __init__(self, config=None):
        self.cfg = config or MapperConfig()
        c = self.cfg
        self.grid = OccupancyGrid2D(resolution=c.resolution, initial_size=c.initial_size,
                                    max_size=c.max_size, hit_logodds=c.hit_logodds,
                                    miss_logodds=c.miss_logodds)
        self.voxels = VoxelMap3D(resolution=c.voxel_resolution)
        wall_kwargs = dict(resolution=c.resolution)
        wall_kwargs.update(c.walls or {})
        self.extractor = WallExtractor(**wall_kwargs)
        self.tracker = WallTracker()
        self.home_down = None
        self.walls = []
        self.trajectory = []
        self.clouds_integrated = 0
        self.points_integrated = 0
        self._sources = {}            # nombre -> (stamp, ranges)
        self.last_extract_sec = 0.0

    # ------------------------------------------------------------------
    def set_home(self, down):
        """Altura del piso en el despegue (coordenada D de NED). Todas las
        alturas (franja del mapa, alturas de paredes) se miden desde acá."""
        self.home_down = float(down)

    def reset(self):
        self.grid.clear()
        self.voxels.clear()
        self.tracker.reset()
        self.walls = []
        self.trajectory = []
        self.clouds_integrated = 0
        self.points_integrated = 0
        self._sources.clear()

    def update_trajectory(self, position_ned):
        if self.home_down is None:
            self.set_home(position_ned[2])
        p = (float(position_ned[0]), float(position_ned[1]), float(position_ned[2]))
        if (not self.trajectory
                or math.dist(p, self.trajectory[-1]) >= self.cfg.trajectory_step):
            self.trajectory.append(p)

    # ------------------------------------------------------------------
    def integrate(self, points_world, sensor_origin, drone_position, stamp,
                  source='lidar', sensor_heading=0.0):
        """Integra una nube ya transformada al mundo NED.

        `sensor_origin`: posición NED del sensor (para el tallado de espacio
        libre). `drone_position`: NED del centro del dron (para los sectores).
        `sensor_heading`: rumbo del eje del sensor, sólo importa si
        `sensor_fov_deg` < 360. Devuelve cuántos puntos se usaron.
        """
        c = self.cfg
        if self.home_down is None:
            self.set_home(drone_position[2])
        pts = np.asarray(points_world, dtype=float).reshape(-1, 3)
        origin = np.asarray(sensor_origin, dtype=float).reshape(3)
        if pts.shape[0]:
            rng = np.linalg.norm(pts - origin, axis=1)
            pts = pts[(rng >= c.min_range) & (rng <= c.max_range)]
        heights = self.home_down - pts[:, 2]

        in_band = (heights >= c.min_height) & (heights <= c.max_height)
        hits_ne = pts[in_band, :2]
        ends = free_ray_ends(origin[:2], hits_ne, n_bins=c.free_ray_bins,
                             no_hit_range=c.no_hit_free_range,
                             fov_center=sensor_heading, fov=math.radians(c.sensor_fov_deg),
                             max_range=c.max_range, neighbor_bins=c.free_ray_neighbors)
        self.grid.update(origin[:2], hits_ne, ends)
        self.voxels.insert(pts[heights >= c.voxel_min_height], sensor_origin=origin)

        drone = np.asarray(drone_position, dtype=float).reshape(3)
        drone_h = self.home_down - drone[2]
        near = ((heights >= drone_h - c.sector_band_below)
                & (heights <= drone_h + c.sector_band_above)
                & (heights >= c.min_height))
        ranges, _ = compute_sectors(pts[near, :2] - drone[:2], n_bins=c.sector_bins,
                                    max_range=c.sector_range, min_points=c.sector_min_points)
        self._sources[source] = (float(stamp), ranges)
        self.clouds_integrated += 1
        self.points_integrated += int(pts.shape[0])
        return int(pts.shape[0])

    def map_sectors(self, drone_position):
        """Sectores calculados con las celdas ocupadas del mapa cerca del dron.

        Le da memoria a la evasión: con una cámara de 87° de campo, una pared que
        quedó al costado sigue existiendo aunque el sensor ya no la vea.
        """
        c = self.cfg
        g = self.grid
        rows, cols = g.to_cells(np.array([[drone_position[0], drone_position[1]]]))
        span = int(math.ceil(c.sector_range / g.resolution))
        r0, r1 = max(0, rows[0] - span), min(g.shape[0], rows[0] + span + 1)
        c0, c1 = max(0, cols[0] - span), min(g.shape[1], cols[0] + span + 1)
        if r0 >= r1 or c0 >= c1:
            return np.full(c.sector_bins, INF)
        window = g.known[r0:r1, c0:c1] & (g.log_odds[r0:r1, c0:c1] >= g.occ_l)
        rr, cc = np.nonzero(window)
        if rr.size == 0:
            return np.full(c.sector_bins, INF)
        centers = g.cell_centers(rr + r0, cc + c0)
        ranges, _ = compute_sectors(centers - np.asarray(drone_position[:2], dtype=float),
                                    n_bins=c.sector_bins, max_range=c.sector_range)
        return ranges

    def sectors(self, now, drone_position):
        """Sectores combinados (mínimo) de las fuentes frescas + el mapa."""
        c = self.cfg
        fresh = [r for stamp, r in self._sources.values() if now - stamp <= c.sector_timeout]
        if c.sectors_from_map:
            fresh.append(self.map_sectors(drone_position))
        merged = merge_sectors(fresh)
        return merged if merged is not None else np.full(c.sector_bins, INF)

    def has_fresh_source(self, now):
        return any(now - stamp <= self.cfg.sector_timeout for stamp, _ in self._sources.values())

    # ------------------------------------------------------------------
    def extract_walls(self):
        t0 = time.monotonic()
        voxels, voxel_views = self.voxels.points(self.cfg.voxel_min_hits, with_views=True)
        cells, views = self.grid.occupied_points(with_views=True)
        walls = self.extractor.extract(cells, views, voxels, voxel_views,
                                       home_down=self.home_down or 0.0,
                                       min_height=self.cfg.min_height)
        annotate_3d(walls, voxels, home_down=self.home_down or 0.0)
        self.walls = self.tracker.assign(walls)
        self.last_extract_sec = time.monotonic() - t0
        return self.walls

    def snapshot(self, stamp=0.0):
        return self.grid.snapshot(stamp)

    def known_fraction(self, area):
        """Fracción conocida (libre u ocupada) de un rectángulo [n0, e0, n1, e1]."""
        g = self.grid
        n0, e0, n1, e1 = area
        r0, c0 = g.to_cells(np.array([[n0, e0]]))
        r1, c1 = g.to_cells(np.array([[n1, e1]]))
        r0, r1 = sorted((int(r0[0]), int(r1[0])))
        c0, c1 = sorted((int(c0[0]), int(c1[0])))
        total = max(1, (r1 - r0 + 1) * (c1 - c0 + 1))
        rs0, rs1 = max(0, r0), min(g.shape[0], r1 + 1)
        cs0, cs1 = max(0, c0), min(g.shape[1], c1 + 1)
        if rs0 >= rs1 or cs0 >= cs1:
            return 0.0
        return float(np.count_nonzero(g.known[rs0:rs1, cs0:cs1])) / total
