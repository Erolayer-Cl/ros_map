#!/usr/bin/env python3
"""Conversión PointCloud2 <-> numpy y adaptadores de LaserScan / Range.

Sin rclpy ni sensor_msgs_py a propósito: así se prueba con bytes armados a
mano y no hay que pelearse con versiones de `sensor_msgs_py` entre Humble y
Jazzy. Los nodos le pasan los campos crudos del mensaje.
"""

import math

import numpy as np

# sensor_msgs/PointField: constantes de tipo -> dtype de numpy
POINTFIELD_DTYPES = {
    1: np.int8,
    2: np.uint8,
    3: np.int16,
    4: np.uint16,
    5: np.int32,
    6: np.uint32,
    7: np.float32,
    8: np.float64,
}
FLOAT32 = 7


def pointcloud2_to_xyz(fields, data, point_step, width, height, is_bigendian=False):
    """Extrae los campos x/y/z de una PointCloud2 como un array (N, 3) float64.

    `fields` es una lista de objetos con `.name`, `.offset`, `.datatype`
    (sirven tal cual los `PointField` del mensaje) o tuplas
    `(name, offset, datatype)`. Descarta puntos NaN/inf.
    """
    lookup = {}
    for field in fields:
        if isinstance(field, tuple):
            name, offset, datatype = field[0], field[1], field[2]
        else:
            name, offset, datatype = field.name, field.offset, field.datatype
        lookup[name] = (int(offset), int(datatype))
    if not all(axis in lookup for axis in ('x', 'y', 'z')):
        raise ValueError('la nube no tiene campos x/y/z')

    count = int(width) * int(height)
    if count == 0 or point_step == 0:
        return np.zeros((0, 3))

    names, formats, offsets = [], [], []
    for axis in ('x', 'y', 'z'):
        offset, datatype = lookup[axis]
        dtype = np.dtype(POINTFIELD_DTYPES[datatype])
        if is_bigendian:
            dtype = dtype.newbyteorder('>')
        names.append(axis)
        formats.append(dtype)
        offsets.append(offset)
    structured = np.dtype({'names': names, 'formats': formats,
                           'offsets': offsets, 'itemsize': int(point_step)})
    raw = np.frombuffer(bytes(data), dtype=structured, count=count)
    xyz = np.column_stack((raw['x'], raw['y'], raw['z'])).astype(np.float64)
    return xyz[np.all(np.isfinite(xyz), axis=1)]


def xyz_to_pointcloud2_data(points, rgb=None):
    """Arma los campos y bytes de una PointCloud2 (x, y, z float32 [+ rgb]).

    Devuelve `(fields, data, point_step)` donde `fields` es una lista de
    tuplas `(name, offset, datatype, count)`; el nodo arma los `PointField`.
    `rgb`, si viene, es (N, 3) uint8 y se empaqueta como float32 al estilo PCL,
    que es lo que entienden RViz y Foxglove.
    """
    pts = np.asarray(points, dtype=np.float32).reshape(-1, 3)
    if rgb is None:
        fields = [('x', 0, FLOAT32, 1), ('y', 4, FLOAT32, 1), ('z', 8, FLOAT32, 1)]
        return fields, pts.tobytes(), 12
    colors = np.asarray(rgb, dtype=np.uint32).reshape(-1, 3)
    packed = (colors[:, 0] << 16) | (colors[:, 1] << 8) | colors[:, 2]
    buffer = np.zeros((pts.shape[0], 4), dtype=np.float32)
    buffer[:, :3] = pts
    buffer[:, 3] = packed.astype(np.uint32).view(np.float32)
    fields = [('x', 0, FLOAT32, 1), ('y', 4, FLOAT32, 1), ('z', 8, FLOAT32, 1),
              ('rgb', 12, FLOAT32, 1)]
    return fields, buffer.tobytes(), 16


def laserscan_to_xyz(ranges, angle_min, angle_increment, range_min, range_max):
    """LaserScan (estándar ROS: ángulos antihorarios en el plano XY, FLU) -> (N, 3).

    Lecturas fuera de [range_min, range_max) se descartan: un lidar 2D marca
    "no hay nada" con inf o con range_max, y ninguna de las dos es un punto.
    """
    r = np.asarray(ranges, dtype=np.float64)
    if r.size == 0:
        return np.zeros((0, 3))
    angles = angle_min + angle_increment * np.arange(r.size)
    valid = np.isfinite(r) & (r >= range_min) & (r < range_max * 0.999)
    r, angles = r[valid], angles[valid]
    return np.column_stack((r * np.cos(angles), r * np.sin(angles), np.zeros_like(r)))


def range_to_xyz(distance, min_range, max_range):
    """Un `sensor_msgs/Range` (haz sobre +X del sensor) -> punto (1, 3) o vacío.

    Como en el repo de referencia: Cosys-AirSim devuelve exactamente
    `max_range` cuando el rayo no choca, y a veces lecturas basura <= 0 justo
    después del spawn. Ninguna de las dos es un obstáculo.
    """
    if (not math.isfinite(distance) or distance <= max(min_range, 0.0)
            or distance >= max_range * 0.98):
        return np.zeros((0, 3))
    return np.array([[float(distance), 0.0, 0.0]])


def outside_box(points, box):
    """Máscara de los puntos (N, 3) que quedan FUERA de una caja alineada
    `[xmin, xmax, ymin, ymax, zmin, zmax]` (en el marco en que estén los
    puntos). Se usa en el marco del cuerpo para tirar los impactos contra el
    propio dron: hélices, patas, tren, mástil del GPS, carga útil.
    """
    pts = np.asarray(points, dtype=float).reshape(-1, 3)
    if box is None or len(box) != 6 or pts.shape[0] == 0:
        return np.ones(pts.shape[0], dtype=bool)
    lo = np.array([box[0], box[2], box[4]], dtype=float)
    hi = np.array([box[1], box[3], box[5]], dtype=float)
    inside = np.all((pts >= lo) & (pts <= hi), axis=1)
    return ~inside


# Desplazamientos de la vecindad 3x3x3 de un vóxel.
_NEIGHBORS = np.array([(i, j, k) for i in (-1, 0, 1) for j in (-1, 0, 1) for k in (-1, 0, 1)],
                      dtype=np.int64)
_KEY_OFFSET = 1 << 20


def _voxel_keys(idx):
    idx = idx + _KEY_OFFSET
    return (idx[:, 0] << 42) | (idx[:, 1] << 21) | idx[:, 2]


def isolated_mask(points, voxel=0.25, min_points=2, max_range=8.0, origin=(0.0, 0.0, 0.0)):
    """Máscara de puntos AISLADOS (ruido): con menos de `min_points` puntos
    (contándose) en su vecindad de 3x3x3 vóxeles de `voxel` m.

    Polvo, lluvia, insectos y reflejos dan retornos sueltos; una superficie
    real da varios puntos juntos en la misma nube. Es el criterio de
    PX4-Avoidance (`min_num_points_per_cell`) y del planificador de CMU (≥ 2
    puntos), hecho con vóxeles para que sea lineal en numpy. Sólo se aplica a
    menos de `max_range` del sensor: más lejos los puntos de una superficie ya
    quedan separados por la resolución angular del lidar (y lejos no disparan
    la evasión). Ojo: la nube de polvo densa SÍ pasa (Phillips et al. 2017: se
    ve como una pared en el borde delantero); esto sólo saca lo suelto.
    """
    pts = np.asarray(points, dtype=float).reshape(-1, 3)
    n = pts.shape[0]
    if n == 0 or min_points <= 1 or voxel <= 0.0:
        return np.zeros(n, dtype=bool)
    near = np.linalg.norm(pts - np.asarray(origin, dtype=float).reshape(3), axis=1) <= max_range
    if not np.any(near):
        return np.zeros(n, dtype=bool)
    idx = np.floor(pts / voxel).astype(np.int64)
    keys, inverse, counts = np.unique(_voxel_keys(idx), return_inverse=True, return_counts=True)
    inverse = inverse.reshape(-1)
    # Para cada vóxel ocupado, la suma de puntos de sus 27 vecinos.
    first = np.zeros(len(keys), dtype=np.int64)
    first[inverse] = np.arange(n)
    vox_idx = idx[first]
    total = np.zeros(len(keys), dtype=np.int64)
    for offset in _NEIGHBORS:
        nk = _voxel_keys(vox_idx + offset)
        pos = np.searchsorted(keys, nk)
        pos = np.minimum(pos, len(keys) - 1)
        hit = keys[pos] == nk
        total += np.where(hit, counts[pos], 0)
    return near & (total[inverse] < min_points)


def voxel_downsample(points, size):
    """Un punto por vóxel de `size` metros (el primero). Baja la carga de CPU de
    la Jetson: un Mid-360 da 200 mil puntos/s y para el mapa alcanza con ~10%."""
    pts = np.asarray(points, dtype=float).reshape(-1, 3)
    if size <= 0.0 or len(pts) == 0:
        return pts
    keys = np.floor(pts / size).astype(np.int64)
    _, first = np.unique(keys, axis=0, return_index=True)
    return pts[np.sort(first)]
