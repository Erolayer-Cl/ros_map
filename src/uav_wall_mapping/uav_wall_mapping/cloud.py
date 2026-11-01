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


def voxel_downsample(points, size):
    """Un punto por vóxel de `size` metros (el primero). Baja la carga de CPU de
    la Jetson: un Mid-360 da 200 mil puntos/s y para el mapa alcanza con ~10%."""
    pts = np.asarray(points, dtype=float).reshape(-1, 3)
    if size <= 0.0 or len(pts) == 0:
        return pts
    keys = np.floor(pts / size).astype(np.int64)
    _, first = np.unique(keys, axis=0, return_index=True)
    return pts[np.sort(first)]
