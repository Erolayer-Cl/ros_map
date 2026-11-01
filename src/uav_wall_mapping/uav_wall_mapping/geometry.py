#!/usr/bin/env python3
"""Geometría compartida: marcos NED/FRD, cuaterniones y convenciones de ejes.

Python puro + numpy, sin rclpy, para poder probarlo en un intérprete pelado.

Convenciones (las mismas de PX4 y del repo de referencia UAV_ros2-beta):

- **Mundo**: NED local de PX4 (x = Norte, y = Este, z = Abajo). El origen es
  donde arrancó el EKF, o sea el punto de despegue.
- **Cuerpo**: FRD (x = adelante, y = derecha, z = abajo).
- **Rumbos**: radianes, 0 = Norte, positivo hacia el Este (horario visto desde
  arriba), igual que `TrajectorySetpoint.yaw`.
- **Cuaterniones**: `[w, x, y, z]`, rotación cuerpo FRD -> mundo NED, igual que
  `VehicleAttitude.q` / `VehicleOdometry.q`.

ROS (RViz/Foxglove) usa ENU/FLU. La conversión se hace sólo en los bordes, al
publicar cosas para visualizar; nada de la lógica interna usa ENU.
"""

import math

import numpy as np

# Matrices que llevan un vector expresado en los ejes "nativos" de un sensor a
# ejes FRD del propio sensor. Cada driver publica en su convención:
#   - flu:     estándar ROS (REP-103). Cosys-AirSim (airsim_node) publica la
#              nube del lidar así: invierte y/z de su NED/FRD interno.
#   - frd:     convención PX4 / AirSim crudo.
#   - optical: cámaras de profundidad (RealSense, ZED, Orbbec): z adelante,
#              x derecha, y abajo.
AXES_TO_FRD = {
    'frd': np.eye(3),
    'flu': np.diag([1.0, -1.0, -1.0]),
    'optical': np.array([[0.0, 0.0, 1.0],
                         [1.0, 0.0, 0.0],
                         [0.0, 1.0, 0.0]]),
}


def wrap_pi(angle):
    """Normaliza un ángulo al rango [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def angle_diff(a, b):
    """Diferencia angular absoluta entre dos rumbos, en [0, pi]."""
    return abs(wrap_pi(a - b))


def bearing_between(from_ne, to_ne):
    """Rumbo NED que va de un punto a otro (n, e)."""
    return math.atan2(to_ne[1] - from_ne[1], to_ne[0] - from_ne[0])


def unit_from_bearing(bearing):
    """Vector unitario (norte, este) para un rumbo NED."""
    return (math.cos(bearing), math.sin(bearing))


def distance_2d(a_ne, b_ne):
    return math.hypot(b_ne[0] - a_ne[0], b_ne[1] - a_ne[1])


def quat_to_rot(q):
    """Matriz de rotación 3x3 a partir de un cuaternión [w, x, y, z].

    Con un cuaternión de PX4 (FRD -> NED), `R @ v_frd` da el vector en NED.
    Normaliza la entrada: un cuaternión levemente desnormalizado (pasa con
    float32) metería un factor de escala en todas las nubes.
    """
    w, x, y, z = (float(v) for v in q)
    norm = math.sqrt(w * w + x * x + y * y + z * z)
    if norm < 1e-9 or not math.isfinite(norm):
        return np.eye(3)
    w, x, y, z = w / norm, x / norm, y / norm, z / norm
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])


def quat_from_yaw(yaw):
    """Cuaternión [w, x, y, z] de una rotación pura de yaw (dron nivelado)."""
    return [math.cos(yaw / 2.0), 0.0, 0.0, math.sin(yaw / 2.0)]


def yaw_from_quat(q):
    """Yaw (rumbo NED) de un cuaternión FRD->NED."""
    w, x, y, z = (float(v) for v in q)
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def rot_from_rpy(roll, pitch, yaw):
    """Rotación ZYX (yaw, luego pitch, luego roll), ángulos en radianes.

    Es la convención de montaje de AirSim (`Roll`/`Pitch`/`Yaw` de cada sensor
    en settings.json) y la de PX4: lleva ejes del sensor a ejes del cuerpo.
    """
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])


class SensorMount:
    """Dónde y cómo va montado un sensor en el dron.

    `position_frd`: offset del sensor respecto del centro del dron, en metros,
    ejes FRD (igual que los `X`/`Y`/`Z` de settings.json de AirSim).
    `rpy_deg`: orientación del sensor respecto del cuerpo (grados).
    `axes`: convención de ejes en la que el driver publica los puntos
    ('flu', 'frd' u 'optical').
    """

    def __init__(self, position_frd=(0.0, 0.0, 0.0), rpy_deg=(0.0, 0.0, 0.0), axes='flu'):
        if axes not in AXES_TO_FRD:
            raise ValueError(f'convención de ejes desconocida: {axes!r} '
                             f'(se espera una de {sorted(AXES_TO_FRD)})')
        self.position_frd = np.asarray(position_frd, dtype=float).reshape(3)
        roll, pitch, yaw = (math.radians(v) for v in rpy_deg)
        # sensor nativo -> sensor FRD -> cuerpo FRD
        self.rotation = rot_from_rpy(roll, pitch, yaw) @ AXES_TO_FRD[axes]
        self.axes = axes

    def to_world(self, points_sensor, position_ned, q_body_to_ned):
        """Lleva puntos del sensor (N, 3) al mundo NED.

        Devuelve `(puntos_mundo, origen_del_sensor_en_mundo)`.
        """
        rot_body = quat_to_rot(q_body_to_ned)
        pts = np.asarray(points_sensor, dtype=float).reshape(-1, 3)
        body = pts @ self.rotation.T + self.position_frd
        world = body @ rot_body.T + np.asarray(position_ned, dtype=float)
        origin = rot_body @ self.position_frd + np.asarray(position_ned, dtype=float)
        return world, origin


def ned_to_enu(points_ned):
    """(N, 3) NED -> ENU, para publicar cosas que se van a ver en Foxglove/RViz."""
    pts = np.asarray(points_ned, dtype=float).reshape(-1, 3)
    return np.column_stack((pts[:, 1], pts[:, 0], -pts[:, 2]))


def ned_yaw_to_enu_quat(yaw_ned):
    """Cuaternión ENU (x, y, z, w) para un rumbo NED. Sirve para flechas y TF."""
    yaw_enu = math.pi / 2.0 - yaw_ned
    return (0.0, 0.0, math.sin(yaw_enu / 2.0), math.cos(yaw_enu / 2.0))


def px4_quat_to_enu_flu(q):
    """Cuaternión de PX4 (FRD->NED, [w,x,y,z]) a ROS (FLU->ENU, (x,y,z,w)).

    q_enu_flu = q_ned_to_enu * q_frd_ned * q_flu_to_frd. Se resuelve con
    matrices para no depender de tf_transformations.
    """
    r_frd_ned = quat_to_rot(q)
    ned_to_enu_m = np.array([[0.0, 1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, -1.0]])
    flu_to_frd = np.diag([1.0, -1.0, -1.0])
    r = ned_to_enu_m @ r_frd_ned @ flu_to_frd
    return rot_to_quat_xyzw(r)


def rot_to_quat_xyzw(r):
    """Matriz de rotación -> cuaternión (x, y, z, w)."""
    trace = r[0, 0] + r[1, 1] + r[2, 2]
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * s
        x = (r[2, 1] - r[1, 2]) / s
        y = (r[0, 2] - r[2, 0]) / s
        z = (r[1, 0] - r[0, 1]) / s
    elif r[0, 0] > r[1, 1] and r[0, 0] > r[2, 2]:
        s = math.sqrt(1.0 + r[0, 0] - r[1, 1] - r[2, 2]) * 2.0
        w = (r[2, 1] - r[1, 2]) / s
        x = 0.25 * s
        y = (r[0, 1] + r[1, 0]) / s
        z = (r[0, 2] + r[2, 0]) / s
    elif r[1, 1] > r[2, 2]:
        s = math.sqrt(1.0 + r[1, 1] - r[0, 0] - r[2, 2]) * 2.0
        w = (r[0, 2] - r[2, 0]) / s
        x = (r[0, 1] + r[1, 0]) / s
        y = 0.25 * s
        z = (r[1, 2] + r[2, 1]) / s
    else:
        s = math.sqrt(1.0 + r[2, 2] - r[0, 0] - r[1, 1]) * 2.0
        w = (r[1, 0] - r[0, 1]) / s
        x = (r[0, 2] + r[2, 0]) / s
        y = (r[1, 2] + r[2, 1]) / s
        z = 0.25 * s
    return (float(x), float(y), float(z), float(w))
