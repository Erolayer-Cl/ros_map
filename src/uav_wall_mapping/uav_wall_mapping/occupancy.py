#!/usr/bin/env python3
"""Grilla de ocupación 2D (log-odds) y mapa 3D de vóxeles. Python puro + numpy.

La grilla 2D es el "plano" que se dibuja: cada celda guarda la evidencia
acumulada de que hay algo sólido en la franja de alturas configurada
(`map.min_height`..`map.max_height`). Proyectar toda esa franja a 2D es lo que
hace que una pared irregular (con salientes, inclinada, con huecos a distintas
alturas) quede dibujada con su huella completa y no sólo con el corte a la
altura de vuelo.

Indexado: `log_odds[fila, columna]` con fila = norte y columna = este. Así el
array aplanado en orden C es exactamente el `data` de un `nav_msgs/OccupancyGrid`
en ENU (filas = y = norte, columnas = x = este), sin transponer nada.
"""

import math
from dataclasses import dataclass

import numpy as np


def logit(p):
    return math.log(p / (1.0 - p))


@dataclass
class GridSnapshot:
    """Foto inmutable de la grilla, en el formato de `nav_msgs/OccupancyGrid`.

    `values`: int8 (filas=norte, columnas=este); -1 desconocido, 0..100 ocupado.
    Es lo que consume la misión (planificador y exploración): la arma tanto el
    mapper (directo, en las pruebas) como el nodo de misión (desde el tópico /map).
    """

    values: np.ndarray
    resolution: float
    origin_n: float
    origin_e: float
    stamp: float = 0.0

    @property
    def shape(self):
        return self.values.shape

    def world_to_cell(self, north, east):
        return (int(math.floor((north - self.origin_n) / self.resolution)),
                int(math.floor((east - self.origin_e) / self.resolution)))

    def cell_to_world(self, row, col):
        return (self.origin_n + (row + 0.5) * self.resolution,
                self.origin_e + (col + 0.5) * self.resolution)

    def in_bounds(self, row, col):
        return 0 <= row < self.values.shape[0] and 0 <= col < self.values.shape[1]

    def value_at(self, north, east):
        row, col = self.world_to_cell(north, east)
        if not self.in_bounds(row, col):
            return -1
        return int(self.values[row, col])


class OccupancyGrid2D:
    """Grilla log-odds que crece sola a medida que el dron explora."""

    def __init__(self, resolution=0.15, initial_size=40.0, max_size=400.0,
                 hit_logodds=0.85, miss_logodds=-0.4, min_logodds=-2.0,
                 max_logodds=3.5, occupied_threshold=0.65, free_threshold=0.35,
                 grow_step=10.0, occupied_miss_scale=0.3):
        self.resolution = float(resolution)
        self.max_cells = int(math.ceil(max_size / self.resolution))
        self.grow_cells = max(1, int(math.ceil(grow_step / self.resolution)))
        self.hit = float(hit_logodds)
        self.miss = float(miss_logodds)
        self.min_l = float(min_logodds)
        self.max_l = float(max_logodds)
        self.occ_l = logit(occupied_threshold)
        self.free_l = logit(free_threshold)
        # Una celda ya confirmada como ocupada se vacía más despacio. En una
        # grilla 2D que proyecta una franja de alturas, un rayo que pasa POR
        # ENCIMA de un objeto bajo (una caja de 1.5 m con el dron a 2.5 m) o
        # rasante a una pared lejana atraviesa celdas que sí están ocupadas. Sin
        # esto, los objetos bajos y las paredes vistas de costado se borran.
        self.occupied_miss_scale = float(occupied_miss_scale)

        cells = max(2, int(math.ceil(initial_size / self.resolution)))
        cells = min(cells, self.max_cells)
        half = cells * self.resolution / 2.0
        # El origen se alinea a la resolución para que la celda de un punto no
        # dependa de cuándo creció la grilla.
        self.origin_n = math.floor(-half / self.resolution) * self.resolution
        self.origin_e = self.origin_n
        self.log_odds = np.zeros((cells, cells), dtype=np.float32)
        self.known = np.zeros((cells, cells), dtype=bool)
        # Suma de las direcciones (horizontales, unitarias) desde las que el
        # sensor vio cada celda ocupada. Le dice al extractor de paredes de qué
        # lado de una pared está cada celda (ver walls.py).
        self.view = np.zeros((cells, cells, 2), dtype=np.float32)
        self.dropped_points = 0

    # ------------------------------------------------------------------
    # Índices
    # ------------------------------------------------------------------
    @property
    def shape(self):
        return self.log_odds.shape

    def to_cells(self, points_ne):
        pts = np.asarray(points_ne, dtype=float).reshape(-1, 2)
        rows = np.floor((pts[:, 0] - self.origin_n) / self.resolution).astype(np.int64)
        cols = np.floor((pts[:, 1] - self.origin_e) / self.resolution).astype(np.int64)
        return rows, cols

    def cell_centers(self, rows, cols):
        return np.column_stack((self.origin_n + (np.asarray(rows) + 0.5) * self.resolution,
                                self.origin_e + (np.asarray(cols) + 0.5) * self.resolution))

    def ensure_contains(self, points_ne, margin=2.0):
        """Agranda la grilla para que entren estos puntos (con margen).

        Crece de a `grow_step` metros para no realocar en cada nube, y nunca
        más allá de `max_size`: lo que quede afuera se descarta (y se cuenta en
        `dropped_points`) en vez de llenar la RAM de la Jetson.
        """
        pts = np.asarray(points_ne, dtype=float).reshape(-1, 2)
        if pts.shape[0] == 0:
            return
        rows, cols = self.to_cells(pts)
        m = int(math.ceil(margin / self.resolution))
        need_top = max(0, -(int(rows.min()) - m))
        need_left = max(0, -(int(cols.min()) - m))
        need_bottom = max(0, int(rows.max()) + m + 1 - self.shape[0])
        need_right = max(0, int(cols.max()) + m + 1 - self.shape[1])
        if not (need_top or need_left or need_bottom or need_right):
            return

        def chunk(n):
            return 0 if n == 0 else int(math.ceil(n / self.grow_cells)) * self.grow_cells

        top, left, bottom, right = chunk(need_top), chunk(need_left), chunk(need_bottom), chunk(need_right)
        # Respetar el tamaño máximo recortando el crecimiento.
        while self.shape[0] + top + bottom > self.max_cells and (top or bottom):
            if bottom >= top and bottom > 0:
                bottom = max(0, bottom - self.grow_cells)
            else:
                top = max(0, top - self.grow_cells)
        while self.shape[1] + left + right > self.max_cells and (left or right):
            if right >= left and right > 0:
                right = max(0, right - self.grow_cells)
            else:
                left = max(0, left - self.grow_cells)
        if not (top or left or bottom or right):
            return
        self.log_odds = np.pad(self.log_odds, ((top, bottom), (left, right)))
        self.known = np.pad(self.known, ((top, bottom), (left, right)))
        self.view = np.pad(self.view, ((top, bottom), (left, right), (0, 0)))
        self.origin_n -= top * self.resolution
        self.origin_e -= left * self.resolution

    # ------------------------------------------------------------------
    # Actualización
    # ------------------------------------------------------------------
    def update(self, sensor_ne, hits_ne, ray_ends_ne):
        """Integra una observación.

        `hits_ne`: puntos sólidos (N, 2) -> sus celdas suben hacia "ocupado".
        `ray_ends_ne`: extremos (M, 2) de los rayos de espacio libre; todas las
        celdas entre el sensor y cada extremo (sin incluirlo) bajan hacia
        "libre". Cada celda se actualiza una sola vez por observación aunque
        la crucen muchos rayos: si no, una celda cerca del dron se vaciaría
        cien veces más rápido que una lejana.
        """
        sensor = np.asarray(sensor_ne, dtype=float).reshape(2)
        hits = np.asarray(hits_ne, dtype=float).reshape(-1, 2)
        ends = np.asarray(ray_ends_ne, dtype=float).reshape(-1, 2)
        if hits.shape[0] == 0 and ends.shape[0] == 0:
            return
        self.ensure_contains(np.vstack((hits, ends, sensor[None, :])))

        hit_all = self._flat_in_bounds(hits, count_dropped=True)
        hit_flat = np.unique(hit_all)
        if hit_all.size:
            rel = hits - sensor
            norm = np.hypot(rel[:, 0], rel[:, 1])
            units = rel / np.maximum(norm, 1e-6)[:, None]
            rows, cols = self.to_cells(hits)
            ok = (rows >= 0) & (rows < self.shape[0]) & (cols >= 0) & (cols < self.shape[1])
            view = self.view.reshape(-1, 2)
            np.add.at(view, hit_all, units[ok].astype(np.float32))

        if ends.shape[0]:
            free_flat = self._ray_cells(sensor, ends)
            if hit_flat.size:
                free_flat = np.setdiff1d(free_flat, hit_flat, assume_unique=True)
            if free_flat.size:
                lo = self.log_odds.reshape(-1)
                current = lo[free_flat]
                miss = np.where(current >= self.occ_l, self.miss * self.occupied_miss_scale, self.miss)
                lo[free_flat] = np.clip(current + miss, self.min_l, self.max_l)
                self.known.reshape(-1)[free_flat] = True

        if hit_flat.size:
            lo = self.log_odds.reshape(-1)
            lo[hit_flat] = np.clip(lo[hit_flat] + self.hit, self.min_l, self.max_l)
            self.known.reshape(-1)[hit_flat] = True

    def _flat_in_bounds(self, points_ne, count_dropped=False):
        if points_ne.shape[0] == 0:
            return np.zeros(0, dtype=np.int64)
        rows, cols = self.to_cells(points_ne)
        ok = (rows >= 0) & (rows < self.shape[0]) & (cols >= 0) & (cols < self.shape[1])
        if count_dropped:
            self.dropped_points += int(np.count_nonzero(~ok))
        return rows[ok] * self.shape[1] + cols[ok]

    def _ray_cells(self, sensor, ends):
        """Celdas recorridas por los rayos sensor->extremo, sin la celda final."""
        delta = ends - sensor
        lengths = np.hypot(delta[:, 0], delta[:, 1])
        keep = lengths > self.resolution
        if not np.any(keep):
            return np.zeros(0, dtype=np.int64)
        delta, lengths = delta[keep], lengths[keep]
        units = delta / lengths[:, None]
        step = self.resolution * 0.5
        max_steps = int(math.ceil(lengths.max() / step))
        t = np.arange(max_steps) * step
        # Se frena una celda antes del impacto: esa celda es la pared.
        mask = t[None, :] < (lengths[:, None] - self.resolution)
        north = sensor[0] + units[:, 0:1] * t[None, :]
        east = sensor[1] + units[:, 1:2] * t[None, :]
        pts = np.column_stack((north[mask], east[mask]))
        return np.unique(self._flat_in_bounds(pts))

    # ------------------------------------------------------------------
    # Lectura
    # ------------------------------------------------------------------
    def probabilities(self):
        return 1.0 - 1.0 / (1.0 + np.exp(self.log_odds))

    def occupied_mask(self):
        return self.known & (self.log_odds >= self.occ_l)

    def free_mask(self):
        return self.known & (self.log_odds <= self.free_l)

    def unknown_mask(self):
        return ~self.known

    def occupancy_values(self):
        values = np.rint(self.probabilities() * 100.0).astype(np.int8)
        values[~self.known] = -1
        return values

    def snapshot(self, stamp=0.0):
        return GridSnapshot(values=self.occupancy_values(), resolution=self.resolution,
                            origin_n=self.origin_n, origin_e=self.origin_e, stamp=stamp)

    def occupied_points(self, with_views=False):
        """Centros de las celdas ocupadas (y, opcional, su dirección de
        observación media normalizada)."""
        rows, cols = np.nonzero(self.occupied_mask())
        centers = self.cell_centers(rows, cols)
        if not with_views:
            return centers
        view = self.view[rows, cols].astype(float)
        norm = np.hypot(view[:, 0], view[:, 1])
        # Unitario; queda en cero si la celda se vio igual desde lados opuestos
        # (pared de una celda de espesor): esas no aportan lado.
        units = np.where(norm[:, None] > 1e-3, view / np.maximum(norm, 1e-6)[:, None], 0.0)
        return centers, units

    def clear(self):
        self.log_odds[:] = 0.0
        self.known[:] = False
        self.view[:] = 0.0
        self.dropped_points = 0


class VoxelMap3D:
    """Mapa 3D disperso: por vóxel, cantidad de impactos y su posición media.

    Guardar la media (y no sólo el centro del vóxel) elimina el ruido de
    cuantización: un plano a 45° visto en vóxeles de 20 cm tendría un rms de
    ~8 cm sólo por la escalera de los centros, y no pasaría por plano.

    Sirve para la vista 3D y para medir altura e inclinación de cada pared. No
    borra espacio libre (no hay ray casting 3D): es un mapa de lo que se vio,
    pensado para escenas estáticas. Para filtrar ruido se piden `min_hits`.
    """

    _OFFSET = 1 << 20   # índices de -1M..1M por eje entran en 21 bits

    def __init__(self, resolution=0.2, max_voxels=2_000_000):
        self.resolution = float(resolution)
        self.max_voxels = int(max_voxels)
        self.table = {}          # clave -> [impactos, Σn, Σe, Σd, Σvista_n, Σvista_e]
        self.full_warned = False

    def _keys(self, points):
        idx = np.floor(np.asarray(points, dtype=float).reshape(-1, 3) / self.resolution).astype(np.int64)
        idx += self._OFFSET
        return (idx[:, 0] << 42) | (idx[:, 1] << 21) | idx[:, 2]

    def insert(self, points_ned, sensor_origin=None):
        """Suma impactos. Con `sensor_origin` también acumula desde qué lado
        (dirección horizontal sensor->punto) se vio cada vóxel."""
        pts = np.asarray(points_ned, dtype=float).reshape(-1, 3)
        if len(pts) == 0:
            return
        keys, inverse, counts = np.unique(self._keys(pts), return_inverse=True, return_counts=True)
        inverse = inverse.reshape(-1)
        data = np.zeros((len(keys), 5))
        np.add.at(data[:, 0:3], inverse, pts)
        if sensor_origin is not None:
            rel = pts[:, :2] - np.asarray(sensor_origin, dtype=float).reshape(-1)[:2]
            units = rel / np.maximum(np.hypot(rel[:, 0], rel[:, 1]), 1e-6)[:, None]
            np.add.at(data[:, 3:5], inverse, units)
        table = self.table
        for key, count, row in zip(keys.tolist(), counts.tolist(), data.tolist()):
            entry = table.get(key)
            if entry is not None:
                entry[0] += count
                for i in range(5):
                    entry[i + 1] += row[i]
            elif len(table) < self.max_voxels:
                table[key] = [count] + row
            else:
                self.full_warned = True

    def points(self, min_hits=2, with_views=False):
        """Posición media de cada vóxel con al menos `min_hits` impactos
        (y, opcional, su dirección de observación media normalizada)."""
        if not self.table:
            empty = np.zeros((0, 3))
            return (empty, np.zeros((0, 2))) if with_views else empty
        data = np.array(list(self.table.values()), dtype=float)
        data = data[data[:, 0] >= min_hits]
        means = data[:, 1:4] / data[:, 0:1]
        if not with_views:
            return means
        view = data[:, 4:6]
        norm = np.hypot(view[:, 0], view[:, 1])
        units = np.where(norm[:, None] > 1e-3, view / np.maximum(norm, 1e-6)[:, None], 0.0)
        return means, units

    def __len__(self):
        return len(self.table)

    def clear(self):
        self.table.clear()
        self.full_warned = False
