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

from uav_wall_mapping.cloud import isolated_mask, outside_box, voxel_downsample
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
    max_height: float = 8.0           # m sobre el despegue: tope absoluto de la grilla 2D
    max_height_above_drone: float = 1.0  # m sobre el DRON: tope de la grilla 2D (techos,
    #                                   aleros, copas: el dron pasa por debajo). <= 0: sólo
    #                                   max_height. El mapa 3D guarda todo igual.
    no_hit_free_range: float = 6.0    # m que se dan por libres en sectores sin impacto
    free_ray_bins: int = 360          # sectores del tallado de espacio libre: NO más finos
    #                                   que la resolución horizontal del sensor
    free_ray_neighbors: int = 3       # sectores vecinos que limitan el tallado de un sector vacío
    sensor_fov_deg: float = 360.0     # campo horizontal del sensor (87 para una RealSense)
    min_range: float = 0.5            # m: descarta impactos contra el propio dron
    # Caja del propio dron en el marco del cuerpo FRD [xmin, xmax, ymin, ymax,
    # zmin, zmax] (m): lo de adentro son hélices, patas, mástil... (z+ = abajo).
    # Medir el dron y sumar ~5 cm. None = sin filtro (sólo min_range).
    self_filter_box: tuple = (-0.5, 0.5, -0.5, 0.5, -0.35, 0.4)
    # Ruido suelto (polvo, lluvia, insectos): puntos con menos de N en su
    # vecindad de 3x3x3 vóxeles, hasta cierto alcance del sensor.
    outlier_voxel: float = 0.25
    outlier_min_points: int = 2       # 1 = sin filtro
    outlier_max_range: float = 8.0
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
    sector_min_points: int = 2        # distancia del sector = la del k-ésimo punto (ruido suelto)
    sector_timeout: float = 0.4       # s desde la CAPTURA: una fuente más vieja no cuenta
    #                                   (PX4 Collision Prevention descarta a los 0.5 s)
    sector_store_margin: float = 0.5  # m extra de franja que se guarda por si el dron sube/baja
    min_cloud_points: int = 20        # una nube con menos puntos útiles es un sensor ciego:
    #                                   no talla libre "donde no chocó" ni cuenta como dato fresco
    sectors_from_map: bool = True     # sumar lo que ya está en el mapa (memoria)
    # --- olvido de lo efímero (obstáculos que se movieron) ---
    forget_after: float = 0.0         # s sin verse (ArduPilot OA_DB_EXPIRE = 10 s); 0 = nunca
    forget_max_hits: int = 10         # sólo celdas/vóxeles vistos en <= N nubes (~1 s a 10 Hz)
    forget_period: float = 1.0        # s entre pasadas
    forget_radius: float = 10.0       # m alrededor del dron (donde el sensor lo seguiría viendo)
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
    seen_age: np.ndarray = None   # s entre la última vez que un sensor en vivo miró cada
    #                               sector y `stamp` (inf = nunca): un sector libre que nadie
    #                               mira (fuera del campo de una cámara) es DESCONOCIDO


class _FrozenVoxels:
    """Lo único que `save_map_bundle` le pide al mapa 3D, ya copiado."""

    def __init__(self, points):
        self._points = points

    def points(self, min_hits=2, with_views=False):
        return self._points


@dataclass
class MapState:
    """Copia del estado del mapa para procesarla FUERA del lock de mapper_node.

    Copiarla es barato (arreglos numpy); extraer paredes o escribir PNG/SVG/PLY
    puede tardar más de un segundo en la Jetson, y con el lock tomado los
    sectores de la evasión llegaban tarde (03_brechas.md, A5). Tiene la misma
    interfaz que `WallMapper` para `save_map_bundle`.
    """
    grid_snapshot: object
    cells: np.ndarray
    views: np.ndarray
    voxel_points: np.ndarray
    voxel_views: np.ndarray
    home_down: float
    trajectory: list
    clouds_integrated: int
    points_integrated: int
    cfg: object
    walls: list = field(default_factory=list)

    def snapshot(self, stamp=0.0):
        return self.grid_snapshot

    @property
    def voxels(self):
        return _FrozenVoxels(self.voxel_points)


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
        # Última vez (stamp de captura) que un sensor en vivo tuvo cada sector
        # (rumbo NED) dentro de su campo horizontal.
        self._seen = np.full(self.cfg.sector_bins, -math.inf)
        self.starved_clouds = 0        # nubes (casi) vacías: sensor ciego
        self.self_hits = 0             # puntos dentro de la caja del dron (acumulado)
        self.forgotten = 0             # celdas olvidadas por efímeras (acumulado)
        self._last_forget = -math.inf
        self.outliers = 0              # puntos aislados descartados (acumulado)
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
        self._last_forget = -math.inf
        self._seen[:] = -math.inf

    def update_trajectory(self, position_ned):
        if self.home_down is None:
            self.set_home(position_ned[2])
        p = (float(position_ned[0]), float(position_ned[1]), float(position_ned[2]))
        if (not self.trajectory
                or math.dist(p, self.trajectory[-1]) >= self.cfg.trajectory_step):
            self.trajectory.append(p)

    # ------------------------------------------------------------------
    def integrate_sensor(self, points_sensor, mount, position_ned, q, stamp, source='lidar',
                         min_points=None, min_cloud_points=None, downsample=0.0,
                         filter_outliers=True, fov_deg=None):
        """Camino completo desde el marco del sensor: lo usan `mapper_node` y
        `closed_loop`, así los dos hacen exactamente lo mismo.

        1. ruido suelto (antes de submuestrear, que si no lo esconde),
        2. submuestreo por vóxel,
        3. al marco del cuerpo y caja del propio dron,
        4. al mundo con la pose de la captura.

        `position_ned` y `q` tienen que ser la pose del dron en `stamp` (el
        instante de captura), no la última recibida: ver `pose_buffer.py`.
        """
        c = self.cfg
        pts = np.asarray(points_sensor, dtype=float).reshape(-1, 3)
        if filter_outliers and c.outlier_min_points > 1 and pts.shape[0]:
            noisy = isolated_mask(pts, voxel=c.outlier_voxel, min_points=c.outlier_min_points,
                                  max_range=c.outlier_max_range)
            self.outliers += int(np.count_nonzero(noisy))
            pts = pts[~noisy]
        if downsample and downsample > 0.0:
            pts = voxel_downsample(pts, downsample)
        body = mount.to_body(pts)
        if c.self_filter_box is not None and body.shape[0]:
            keep = outside_box(body, c.self_filter_box)
            self.self_hits += int(np.count_nonzero(~keep))
            body = body[keep]
        world = body_to_world(body, position_ned, q)
        origin = mount.origin_world(position_ned, q)
        heading = yaw_from_quat(q) + mount.heading_offset()
        return self.integrate(world, origin, position_ned, stamp, source=source,
                              sensor_heading=heading, min_points=min_points,
                              min_cloud_points=min_cloud_points, fov_deg=fov_deg)

    def integrate(self, points_world, sensor_origin, drone_position, stamp,
                  source='lidar', sensor_heading=0.0, min_points=None, min_cloud_points=None,
                  fov_deg=None):
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
        rangefinder, que sin eco no devuelve nada). `fov_deg`: campo horizontal
        de ESTE sensor (por defecto `sensor_fov_deg`): los sectores dentro de
        él quedan marcados como "vistos" en `stamp`. Devuelve cuántos puntos se
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
        # Sólo para sensores de campo amplio: un lidar de 360° siempre ve algo
        # (el piso, una pared); una cámara mirando a un espacio abierto devuelve
        # una nube vacía y está sana (lo que no ve queda "no visto", ver seen_age).
        if min_cloud_points is not None:
            needed = int(min_cloud_points)
        else:
            needed = c.min_cloud_points if c.sensor_fov_deg >= 180.0 else 0
        starved = pts.shape[0] < needed
        if starved:
            self.starved_clouds += 1

        # Franja de la grilla 2D: del piso (min_height) hasta un poco por arriba
        # del DRON en el instante de la nube. Lo que está más arriba (el techo
        # de una habitación, un alero) no es obstáculo para volar por debajo y
        # sin esto tapaba toda la grilla (03_brechas.md, C4). Va igual al mapa 3D.
        drone = np.asarray(drone_position, dtype=float).reshape(3)
        drone_h = self.home_down - drone[2]
        top = c.max_height
        if c.max_height_above_drone > 0.0:
            top = min(top, drone_h + c.max_height_above_drone)
        in_band = (heights >= c.min_height) & (heights <= top)
        hits_ne = pts[in_band, :2]
        fov = math.radians(c.sensor_fov_deg if fov_deg is None else float(fov_deg))
        ends = free_ray_ends(origin[:2], hits_ne, n_bins=c.free_ray_bins,
                             no_hit_range=0.0 if starved else c.no_hit_free_range,
                             fov_center=sensor_heading, fov=fov,
                             max_range=c.max_range, neighbor_bins=c.free_ray_neighbors)
        self.grid.update(origin[:2], hits_ne, ends, stamp=float(stamp))
        self.voxels.insert(pts[heights >= c.voxel_min_height], sensor_origin=origin,
                           stamp=float(stamp))
        if c.forget_after > 0.0 and float(stamp) - self._last_forget >= c.forget_period:
            self._last_forget = float(stamp)
            self.forgotten += self.grid.forget_transient(float(stamp), c.forget_after,
                                                         c.forget_max_hits, drone[:2],
                                                         c.forget_radius)
            self.voxels.forget_transient(float(stamp), c.forget_after, c.forget_max_hits,
                                         drone[:2], c.forget_radius)

        keep = ((heights >= max(c.min_height, drone_h - c.sector_band_below - c.sector_store_margin))
                & (heights <= drone_h + c.sector_band_above + c.sector_store_margin))
        rel = pts[:, :2] - drone[:2]
        keep &= np.hypot(rel[:, 0], rel[:, 1]) <= c.sector_range + 2.0
        if not starved:
            self._mark_seen(float(stamp), sensor_heading, fov)
        previous = self._sources.get(source)
        if not starved and (previous is None or float(stamp) >= previous[0]):
            self._sources[source] = (float(stamp), pts[keep, :2].copy(), heights[keep].copy(),
                                     int(min_points if min_points is not None
                                         else c.sector_min_points))
        self.clouds_integrated += 1
        self.points_integrated += int(pts.shape[0])
        return int(pts.shape[0])

    def _mark_seen(self, stamp, heading, fov):
        if fov >= 2.0 * math.pi - 1e-6:
            np.maximum(self._seen, stamp, out=self._seen)
            return
        n = self.cfg.sector_bins
        centers = np.arange(n) * (2.0 * math.pi / n)
        diff = np.abs(np.arctan2(np.sin(centers - heading), np.cos(centers - heading)))
        inside = diff <= fov / 2.0
        self._seen[inside] = np.maximum(self._seen[inside], stamp)

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
        """Sectores para la evasión: en vivo y con la memoria del mapa.

        La memoria (celdas ocupadas de la grilla) sirve para no olvidar una
        pared que salió del campo de visión, pero NO tiene que disparar una
        huida de emergencia: un punto espurio queda en la grilla un par de
        nubes después de que el sensor deja de verlo. Por eso se publica
        también `live` (sólo fuentes en vivo) y la misión decide la emergencia
        con eso.
        """
        c = self.cfg
        live, newest = self.live_sectors(now, drone_position)
        ranges = live
        if c.sectors_from_map:
            ranges = np.minimum(live, self.map_sectors(drone_position))
        if math.isfinite(newest):
            seen_age = np.where(np.isfinite(self._seen), np.maximum(newest - self._seen, 0.0), INF)
        else:
            seen_age = np.full(c.sector_bins, INF)
        return SectorInfo(ranges=ranges, live=live, stamp=newest,
                          fresh=math.isfinite(newest), seen_age=seen_age)

    def sectors(self, now, drone_position):
        """Sectores combinados (mínimo) de las fuentes frescas + el mapa."""
        return self.sector_info(now, drone_position).ranges

    def has_fresh_source(self, now):
        return any(now - stamp <= self.cfg.sector_timeout for stamp, *_ in self._sources.values())

    # ------------------------------------------------------------------
    def capture_state(self):
        """Copia rápida para procesar afuera del lock (ver `MapState`)."""
        voxels, voxel_views = self.voxels.points(self.cfg.voxel_min_hits, with_views=True)
        cells, views = self.grid.occupied_points(with_views=True)
        return MapState(grid_snapshot=self.grid.snapshot(), cells=cells, views=views,
                        voxel_points=voxels, voxel_views=voxel_views,
                        home_down=self.home_down or 0.0, trajectory=list(self.trajectory),
                        clouds_integrated=self.clouds_integrated,
                        points_integrated=self.points_integrated, cfg=self.cfg,
                        walls=list(self.walls))

    def extract_walls(self, state=None):
        """Paredes del mapa. Con `state` (de `capture_state()`) trabaja sobre la
        copia y no toca la grilla: se puede llamar sin el lock del nodo
        (pero no en paralelo consigo misma: el `WallTracker` es uno solo)."""
        t0 = time.monotonic()
        if state is None:
            state = self.capture_state()
        walls = self.extractor.extract(state.cells, state.views, state.voxel_points,
                                       state.voxel_views, home_down=state.home_down,
                                       min_height=self.cfg.min_height)
        annotate_3d(walls, state.voxel_points, home_down=state.home_down)
        self.walls = self.tracker.assign(walls)
        state.walls = self.walls
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
