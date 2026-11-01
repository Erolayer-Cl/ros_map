#!/usr/bin/env python3
"""Estrategias de la misión de mapeo: exploración por fronteras y barrido.

Python puro + numpy. Las dos producen "el próximo objetivo"; la navegación (A*
+ evasión) es la misma que para cualquier waypoint.

- **frontier**: una frontera es una celda libre pegada a una desconocida. Ir a
  las fronteras es la forma estándar de explorar un lugar que no se conoce:
  cada visita revela lo que había detrás, y cuando no quedan fronteras dentro
  del área, el área está mapeada. Se adapta sola a la forma del lugar
  (habitaciones, pasillos, paredes irregulares).
- **lawnmower**: barrido en zigzag de un rectángulo, con carriles separados
  `lane_spacing`. Predecible y fácil de comparar entre corridas; útil en
  exteriores abiertos o cuando se quiere repetir exactamente el mismo vuelo.
"""

import math
from collections import deque
from dataclasses import dataclass

import numpy as np


@dataclass
class Frontier:
    target: tuple          # (n, e) celda de frontera más cercana al centroide
    centroid: tuple
    size: int              # celdas
    length: float          # metros aproximados de frontera


def area_mask(shape, origin_n, origin_e, resolution, area):
    """Máscara de las celdas cuyo centro cae dentro de [n0, e0, n1, e1]."""
    n0, e0, n1, e1 = area
    north = origin_n + (np.arange(shape[0]) + 0.5) * resolution
    east = origin_e + (np.arange(shape[1]) + 0.5) * resolution
    return ((north >= min(n0, n1)) & (north <= max(n0, n1)))[:, None] & \
           ((east >= min(e0, e1)) & (east <= max(e0, e1)))[None, :]


def _near_occupied(values, radius_cells, occupied_threshold=65):
    """Celdas a menos de `radius_cells` de una ocupada (dilatación cuadrada)."""
    occ = values >= occupied_threshold
    out = occ.copy()
    for _ in range(max(0, radius_cells)):
        grown = out.copy()
        grown[1:, :] |= out[:-1, :]
        grown[:-1, :] |= out[1:, :]
        grown[:, 1:] |= out[:, :-1]
        grown[:, :-1] |= out[:, 1:]
        out = grown
    return out


def find_frontiers(snapshot, area=None, free_threshold=35, min_size=5, clearance=0.0):
    """Fronteras (agrupadas) de la grilla, opcionalmente limitadas a un área.

    Fuera de la grilla se considera desconocido: la grilla crece a medida que el
    dron ve, así que su borde es, literalmente, una frontera.

    `clearance`: el objetivo de cada frontera se elige entre sus celdas que
    quedan a más de esto de un obstáculo. Una frontera pegada a una pared (lo
    desconocido de atrás de una caja, la rendija de una pared) no se puede
    alcanzar sin chocar: si no tiene ninguna celda despejada, se descarta.
    """
    values = np.pad(snapshot.values, 1, constant_values=-1)
    on = snapshot.origin_n - snapshot.resolution
    oe = snapshot.origin_e - snapshot.resolution
    free = (values >= 0) & (values <= free_threshold)
    unknown = values < 0
    if area is not None:
        inside = area_mask(values.shape, on, oe, snapshot.resolution, area)
        free &= inside
        unknown &= inside   # mirar afuera del área no cuenta como explorar
    near_unknown = np.zeros_like(unknown)
    near_unknown[1:, :] |= unknown[:-1, :]
    near_unknown[:-1, :] |= unknown[1:, :]
    near_unknown[:, 1:] |= unknown[:, :-1]
    near_unknown[:, :-1] |= unknown[:, 1:]
    frontier = free & near_unknown

    cells = set(zip(*np.nonzero(frontier)))
    clusters = []
    while cells:
        seed = cells.pop()
        queue = deque([seed])
        group = [seed]
        while queue:
            r, c = queue.popleft()
            for dr in (-1, 0, 1):
                for dc in (-1, 0, 1):
                    other = (r + dr, c + dc)
                    if other in cells:
                        cells.remove(other)
                        queue.append(other)
                        group.append(other)
        if len(group) >= min_size:
            clusters.append(group)

    out = []
    res = snapshot.resolution
    blocked = (_near_occupied(values, int(np.ceil(clearance / res))) if clearance > 0
               else np.zeros_like(free))
    for group in clusters:
        g = np.asarray(group, dtype=float)
        pts = np.column_stack((on + (g[:, 0] + 0.5) * res, oe + (g[:, 1] + 0.5) * res))
        centroid = pts.mean(axis=0)
        ok = ~blocked[tuple(np.asarray(group).T)]
        if not np.any(ok):
            continue
        candidates = pts[ok]
        target = candidates[int(np.argmin(np.hypot(*(candidates - centroid).T)))]
        out.append(Frontier(target=(float(target[0]), float(target[1])),
                            centroid=(float(centroid[0]), float(centroid[1])),
                            size=len(group), length=len(group) * res))
    return out


def select_frontier(frontiers, position_ne, heading, blacklist=(), blacklist_radius=1.5,
                    distance_weight=1.0, size_weight=0.6, turn_weight=1.5, min_distance=0.8):
    """La frontera más conveniente: cerca, grande y sin tener que pegar la vuelta.

    `blacklist`: objetivos que ya fallaron (inalcanzables). Se descartan las
    fronteras a menos de `blacklist_radius` de cualquiera de ellos.
    """
    best, best_cost = None, math.inf
    for f in frontiers:
        if any(math.hypot(f.target[0] - b[0], f.target[1] - b[1]) < blacklist_radius
               for b in blacklist):
            continue
        dist = math.hypot(f.target[0] - position_ne[0], f.target[1] - position_ne[1])
        if dist < min_distance:
            continue   # ya estamos encima: si sigue siendo frontera es que no se ve desde acá
        bearing = math.atan2(f.target[1] - position_ne[1], f.target[0] - position_ne[0])
        turn = abs(math.atan2(math.sin(bearing - heading), math.cos(bearing - heading)))
        cost = distance_weight * dist + turn_weight * turn - size_weight * math.sqrt(f.length)
        if cost < best_cost:
            best, best_cost = f, cost
    return best


def lawnmower_waypoints(area, spacing, start_ne=(0.0, 0.0), margin=1.0):
    """Zigzag sobre el rectángulo [n0, e0, n1, e1], con carriles a lo largo del
    lado más largo y arrancando por la esquina más cercana al dron."""
    n0, e0, n1, e1 = area
    n0, n1 = sorted((n0, n1))
    e0, e1 = sorted((e0, e1))
    n0, n1, e0, e1 = n0 + margin, n1 - margin, e0 + margin, e1 - margin
    if n1 <= n0 or e1 <= e0:
        return [((n0 + n1) / 2.0, (e0 + e1) / 2.0)]
    spacing = max(float(spacing), 0.5)
    along_north = (n1 - n0) >= (e1 - e0)
    if along_north:
        lanes = np.arange(e0, e1 + 1e-6, spacing)
        if lanes[-1] < e1 - 1e-6:
            lanes = np.append(lanes, e1)
        if abs(start_ne[1] - e1) < abs(start_ne[1] - e0):
            lanes = lanes[::-1]
        first_low = abs(start_ne[0] - n0) <= abs(start_ne[0] - n1)
        wps = []
        for i, e in enumerate(lanes):
            low_first = first_low if i % 2 == 0 else not first_low
            ends = [(n0, e), (n1, e)] if low_first else [(n1, e), (n0, e)]
            wps.extend(ends)
    else:
        lanes = np.arange(n0, n1 + 1e-6, spacing)
        if lanes[-1] < n1 - 1e-6:
            lanes = np.append(lanes, n1)
        if abs(start_ne[0] - n1) < abs(start_ne[0] - n0):
            lanes = lanes[::-1]
        first_low = abs(start_ne[1] - e0) <= abs(start_ne[1] - e1)
        wps = []
        for i, n in enumerate(lanes):
            low_first = first_low if i % 2 == 0 else not first_low
            ends = [(n, e0), (n, e1)] if low_first else [(n, e1), (n, e0)]
            wps.extend(ends)
    return [(float(a), float(b)) for a, b in wps]


def coverage(snapshot, area):
    """Fracción del área con celdas conocidas (libres u ocupadas)."""
    mask = area_mask(snapshot.values.shape, snapshot.origin_n, snapshot.origin_e,
                     snapshot.resolution, area)
    n0, e0, n1, e1 = area
    total = max(1.0, abs(n1 - n0) * abs(e1 - e0) / snapshot.resolution ** 2)
    known = np.count_nonzero(mask & (snapshot.values >= 0))
    return min(1.0, known / total)
