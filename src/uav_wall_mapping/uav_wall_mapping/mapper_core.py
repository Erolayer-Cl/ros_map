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

from uav_wall_mapping.geometry import body_to_world, yaw_from_quat
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
    sector_timeout: float = 0.4       # s desde la CAPTURA: una fuente más vieja no cuenta
    #                                   (PX4 Collision Prevention descarta a los 0.5 s)
    sector_store_margin: float = 0.5  # m extra de franja que se guarda por si el dron sube/baja
    min_cloud_points: int = 20        # una nube con menos puntos útiles es un sensor ciego:
    #                                   no talla libre "donde no chocó" ni cuenta como dato fresco
    sectors_from_map: bool = True     # sumar lo que ya está en el mapa (memoria)
    # --- paredes ---
    walls: dict = field(default_factory=dict)   # kwargs para WallExtractor
    trajectory_step: float = 0.3      # m entre puntos guardados de la trayectoria


@dataclass
class SectorInfo:
    """Lo que necesita la evasión, calculado contra la pose ACTUAL del dron."""
    ranges: np.ndarray        # mínimo entre las fuentes en vivo y la memoria del mapa
    live: np.ndarray          # sólo las fuentes en vivo (sin memoria)
    stamp: float              # instante de captura del dato en vivo más nuevo (-inf: ninguno)
    fresh: bool               # hay al menos una fuente en vivo dentro de `sector_timeout`


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
        # nombre -> (stamp de captura, puntos NE en el mundo, alturas, min_points).
        # Se guardan los PUNTOS (en NED) y no las distancias relativas al dron:
        # así los sectores se recalculan con la pose actual en cada publicación
        # y no arrastran el corrimiento de una nube vieja.
        self._sources = {}
        self.starved_clouds = 0        # nubes (casi) vacías: sensor ciego
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
    def integrate_sensor(self, points_sensor, mount, position_ned, q, stamp, source='lidar',
                         min_points=None, min_cloud_points=None):
        """Camino completo desde el marco del sensor: lo usan `mapper_node` y
        `closed_loop`, así los dos hacen exactamente lo mismo.

        `position_ned` y `q` tienen que ser la pose del dron en `stamp` (el
        instante de captura), no la última recibida: ver `pose_buffer.py`.
        """
        body = mount.to_body(points_sensor)
        world = body_to_world(body, position_ned, q)
        origin = mount.origin_world(position_ned, q)
        heading = yaw_from_quat(q) + mount.heading_offset()
        return self.integrate(world, origin, position_ned, stamp, source=source,
                              sensor_heading=heading, min_points=min_points,
                              min_cloud_points=min_cloud_points)

    def integrate(self, points_world, sensor_origin, drone_position, stamp,
                  source='lidar', sensor_heading=0.0, min_points=None, min_cloud_points=None):
        """Integra una nube ya transformada al mundo NED.

        `sensor_origin`: posición NED del sensor (para el tallado de espacio
        libre). `drone_position`: NED del centro del dron EN EL INSTANTE DE LA
        NUBE. `stamp`: instante de captura (no de llegada). `sensor_heading`:
        rumbo del eje del sensor, sólo importa si `sensor_fov_deg` < 360.
        `min_points`: puntos por sector para esta fuente (1 para un
        rangefinder, que da uno solo). `min_cloud_points`: con menos puntos
        útiles que esto la nube se trata como de un sensor ciego (tapado,
        saturado, cable suelto que sigue publicando nubes vacías): no se talla
        espacio libre donde "no chocó" y la fuente no se refresca, así la
        evasión se entera de que no tiene datos y frena (0 para un
        rangefinder, que sin eco no devuelve nada). Devuelve cuántos puntos se
        usaron.
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
        needed = c.min_cloud_points if min_cloud_points is None else int(min_cloud_points)
        starved = pts.shape[0] < needed
        if starved:
            self.starved_clouds += 1

        in_band = (heights >= c.min_height) & (heights <= c.max_height)
        hits_ne = pts[in_band, :2]
        ends = free_ray_ends(origin[:2], hits_ne, n_bins=c.free_ray_bins,
                             no_hit_range=0.0 if starved else c.no_hit_free_range,
                             fov_center=sensor_heading, fov=math.radians(c.sensor_fov_deg),
                             max_range=c.max_range, neighbor_bins=c.free_ray_neighbors)
        self.grid.update(origin[:2], hits_ne, ends)
        self.voxels.insert(pts[heights >= c.voxel_min_height], sensor_origin=origin)

        drone = np.asarray(drone_position, dtype=float).reshape(3)
        drone_h = self.home_down - drone[2]
        keep = ((heights >= max(c.min_height, drone_h - c.sector_band_below - c.sector_store_margin))
                & (heights <= drone_h + c.sector_band_above + c.sector_store_margin))
        rel = pts[:, :2] - drone[:2]
        keep &= np.hypot(rel[:, 0], rel[:, 1]) <= c.sector_range + 2.0
        previous = self._sources.get(source)
        if not starved and (previous is None or float(stamp) >= previous[0]):
            self._sources[source] = (float(stamp), pts[keep, :2].copy(), heights[keep].copy(),
                                     int(min_points if min_points is not None
                                         else c.sector_min_points))
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

    def live_sectors(self, now, drone_position):
        """`(ranges, stamp_más_nuevo)` de las fuentes en vivo, relativos a la
        posición actual del dron, con la franja de alturas actual."""
        c = self.cfg
        drone = np.asarray(drone_position, dtype=float).reshape(3)
        drone_h = (self.home_down if self.home_down is not None else drone[2]) - drone[2]
        lo = max(c.min_height, drone_h - c.sector_band_below)
        hi = drone_h + c.sector_band_above
        lists, newest = [], -math.inf
        for stamp, ne, heights, min_points in self._sources.values():
            if now - stamp > c.sector_timeout:
                continue
            band = (heights >= lo) & (heights <= hi)
            ranges, _ = compute_sectors(ne[band] - drone[:2], n_bins=c.sector_bins,
                                        max_range=c.sector_range, min_points=min_points)
            lists.append(ranges)
            newest = max(newest, stamp)
        merged = merge_sectors(lists)
        return (merged if merged is not None else np.full(c.sector_bins, INF)), newest

    def sector_info(self, now, drone_position):
        """Sectores para la evasión: en vivo, memoria del mapa y su edad."""
        c = self.cfg
        live, newest = self.live_sectors(now, drone_position)
        ranges = live
        if c.sectors_from_map:
            ranges = np.minimum(live, self.map_sectors(drone_position))
        return SectorInfo(ranges=ranges, live=live, stamp=newest,
                          fresh=math.isfinite(newest))

    def sectors(self, now, drone_position):
        """Sectores combinados (mínimo) de las fuentes frescas + el mapa."""
        return self.sector_info(now, drone_position).ranges

    def has_fresh_source(self, now):
        return any(now - stamp <= self.cfg.sector_timeout for stamp, *_ in self._sources.values())

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
