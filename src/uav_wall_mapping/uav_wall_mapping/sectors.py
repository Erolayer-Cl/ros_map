#!/usr/bin/env python3
"""Histograma polar de obstáculos alrededor del dron. Python puro + numpy.

Es la vista que usa la evasión reactiva: para cada sector de rumbo (por
defecto 72 de 5°, el mismo formato que `ObstacleDistance` de PX4) la distancia
horizontal al obstáculo más cercano. A diferencia de los 5 haces fijos del
repo de referencia, acá entran **todos** los puntos de la nube que caen en la
franja de altura del dron: una pared irregular con un saliente entre dos haces
ya no pasa desapercibida.

Rumbos NED: el sector 0 está centrado en el Norte y crecen hacia el Este.
"""

import math

import numpy as np

INF = float('inf')


def sector_index(bearings, n_bins):
    increment = 2.0 * math.pi / n_bins
    return np.mod(np.rint(np.asarray(bearings) / increment).astype(np.int64), n_bins)


def compute_sectors(points_rel_ne, n_bins=72, max_range=20.0, min_points=1):
    """Distancia por sector para puntos relativos al dron (N, 2).

    Devuelve `(ranges, counts)`. Con `min_points` = k, la distancia de un
    sector es la del k-ésimo punto más cercano (no la del primero): un retorno
    suelto de polvo o un insecto delante de una pared no "acerca" la pared,
    pero cualquier superficie real (que da varios puntos a casi la misma
    distancia) cuenta igual. `ranges[i] = inf` si el sector tiene menos de k
    puntos.
    """
    pts = np.asarray(points_rel_ne, dtype=float).reshape(-1, 2)
    ranges = np.full(n_bins, INF)
    counts = np.zeros(n_bins, dtype=np.int64)
    if pts.shape[0] == 0:
        return ranges, counts
    dist = np.hypot(pts[:, 0], pts[:, 1])
    keep = dist <= max_range
    if not np.any(keep):
        return ranges, counts
    pts, dist = pts[keep], dist[keep]
    idx = sector_index(np.arctan2(pts[:, 1], pts[:, 0]), n_bins)
    counts = np.bincount(idx, minlength=n_bins)
    k = max(1, int(min_points))
    if k == 1:
        np.minimum.at(ranges, idx, dist)
        return ranges, counts
    order = np.lexsort((dist, idx))            # por sector y, adentro, de menor a mayor
    idx_sorted, dist_sorted = idx[order], dist[order]
    starts = np.searchsorted(idx_sorted, np.arange(n_bins))
    ok = counts >= k
    ranges[ok] = dist_sorted[starts[ok] + k - 1]
    return ranges, counts


def merge_sectors(sector_list):
    """Mínimo sector a sector entre varias fuentes (lidar + mapa + rangefinders)."""
    if not sector_list:
        return None
    ranges = np.minimum.reduce([np.asarray(s, dtype=float) for s in sector_list])
    return ranges


def free_ray_ends(sensor_ne, hits_ne, n_bins=360, no_hit_range=0.0,
                  fov_center=0.0, fov=2.0 * math.pi, max_range=40.0,
                  neighbor_bins=3, margin=0.3, grazing_margin=0.15, max_shorten=2.0,
                  split=False):
    """Extremos de los rayos de espacio libre para la grilla 2D.

    Para cada sector fino (vistos desde el sensor): si hay impactos, el rayo
    llega hasta el más cercano; si no los hay y el sector está dentro del campo
    de visión, se asume libre hasta `no_hit_range` (0 = no suponer nada). Un
    lidar no devuelve puntos donde no chocó, así que sin esto el espacio
    abierto nunca se marcaría como explorado.

    Cuidado con esa suposición: un sector puede quedar sin impacto sólo porque
    cayó entre dos haces del lidar (o porque el haz pasó por encima de la
    franja de alturas), con una pared ahí mismo. Por eso el rayo de un sector
    vacío nunca llega más lejos que el impacto más cercano de los
    `neighbor_bins` sectores vecinos (menos `margin`): si al lado hay una
    pared a 3 m, el vacío no se talla a 6 m a través de ella.

    Además, con `split=True` se devuelven aparte los rayos con impacto y los
    "sin retorno": la ausencia de eco es evidencia débil (un cable de 3 cm que
    un lidar de patrón no repetitivo ve en uno de cada cuatro escaneos, una
    superficie oscura, un vidrio) y no tiene que borrar lo que sí se vio.

    Y con los rayos que sí chocan: uno casi rasante a una pared viaja pegado a
    su cara antes de impactar más adelante, y "vaciaría" las celdas de la cara
    que otros rayos marcaron como ocupadas. Se estima la incidencia con los
    impactos de los sectores vecinos (la tangente de la superficie) y el rayo
    libre se acorta `grazing_margin / sin(incidencia)` (hasta `max_shorten`).
    """
    sensor = np.asarray(sensor_ne, dtype=float).reshape(2)
    hits = np.asarray(hits_ne, dtype=float).reshape(-1, 2)
    increment = 2.0 * math.pi / n_bins
    nearest = np.full(n_bins, INF)
    nearest_pt = np.zeros((n_bins, 2))
    if hits.shape[0]:
        rel = hits - sensor
        dist = np.hypot(rel[:, 0], rel[:, 1])
        keep = (dist > 1e-3) & (dist <= max_range)
        rel, dist, hits = rel[keep], dist[keep], hits[keep]
        if hits.shape[0]:
            idx = sector_index(np.arctan2(rel[:, 1], rel[:, 0]), n_bins)
            order = np.lexsort((dist, idx))       # por sector, y adentro de menor a mayor
            idx_sorted = idx[order]
            first = np.ones(len(order), dtype=bool)
            first[1:] = idx_sorted[1:] != idx_sorted[:-1]
            winners = order[first]
            nearest[idx[winners]] = dist[winners]
            nearest_pt[idx[winners]] = hits[winners]

    hit_bins = np.nonzero(np.isfinite(nearest))[0]
    hit_ends = nearest_pt[hit_bins]
    if hit_bins.size and grazing_margin > 0.0:
        prev_pt = nearest_pt[(hit_bins - 1) % n_bins]
        next_pt = nearest_pt[(hit_bins + 1) % n_bins]
        prev_ok = np.isfinite(nearest[(hit_bins - 1) % n_bins])
        next_ok = np.isfinite(nearest[(hit_bins + 1) % n_bins])
        a = np.where(prev_ok[:, None], prev_pt, hit_ends)
        b = np.where(next_ok[:, None], next_pt, hit_ends)
        tangent = b - a
        t_norm = np.hypot(tangent[:, 0], tangent[:, 1])
        ray = hit_ends - sensor
        r_norm = np.maximum(np.hypot(ray[:, 0], ray[:, 1]), 1e-6)
        sin_inc = np.abs(ray[:, 0] * tangent[:, 1] - ray[:, 1] * tangent[:, 0]) / (
            r_norm * np.maximum(t_norm, 1e-6))
        have_tangent = (prev_ok | next_ok) & (t_norm > 1e-3)
        shorten = np.where(have_tangent,
                           np.minimum(max_shorten, grazing_margin / np.maximum(sin_inc, 0.05)),
                           grazing_margin)
        shorten = np.minimum(shorten, r_norm)
        hit_ends = hit_ends - ray / r_norm[:, None] * shorten[:, None]
    ends = [hit_ends]
    if no_hit_range > 0.0:
        empty = np.nonzero(~np.isfinite(nearest))[0]
        # Alcance de cada sector vacío: lo que permita el vecindario.
        neighbor_min = nearest.copy()
        for k in range(1, max(0, int(neighbor_bins)) + 1):
            neighbor_min = np.minimum(neighbor_min, np.roll(nearest, k))
            neighbor_min = np.minimum(neighbor_min, np.roll(nearest, -k))
        reach = np.minimum(min(no_hit_range, max_range), neighbor_min[empty] - margin)
        centers = empty * increment
        keep = reach > 0.0
        if fov < 2.0 * math.pi - 1e-6:
            diff = np.abs(np.arctan2(np.sin(centers - fov_center), np.cos(centers - fov_center)))
            keep &= diff <= fov / 2.0
        centers, reach = centers[keep], reach[keep]
        ends.append(np.column_stack((sensor[0] + reach * np.cos(centers),
                                     sensor[1] + reach * np.sin(centers))))
    if split:
        # (rayos con impacto, rayos "sin retorno"): la grilla trata los segundos
        # como evidencia débil (ver OccupancyGrid2D.update, `weak_ends_ne`).
        weak = ends[1] if len(ends) > 1 else np.zeros((0, 2))
        return ends[0], weak
    return np.vstack(ends) if ends else np.zeros((0, 2))


def to_px4_obstacle_distance(ranges, max_distance_m, min_distance_m=0.2, unknown_mask=None):
    """Sectores -> arreglo `distances` de `px4_msgs/ObstacleDistance` (cm, uint16).

    Codificación de PX4: `max_distance + 1` = libre, UINT16_MAX = sin dato.
    Requiere 72 sectores de 5° (lo que espera Collision Prevention).
    """
    ranges = np.asarray(ranges, dtype=float)
    max_cm = int(round(max_distance_m * 100.0))
    out = np.full(ranges.shape, max_cm + 1, dtype=np.int64)
    finite = np.isfinite(ranges)
    out[finite] = np.clip(np.rint(ranges[finite] * 100.0), int(min_distance_m * 100.0), max_cm)
    if unknown_mask is not None:
        out[np.asarray(unknown_mask, dtype=bool)] = 65535
    return out.astype(np.uint16).tolist()
