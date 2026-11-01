#!/usr/bin/env python3
"""Mundo simulado: ray casting del lidar, modelo del dron y chequeo de choques.

Python puro + numpy. No intenta ser un simulador físico: alcanza con que el
lidar vea la misma geometría que vería en Unreal y que el dron siga el setpoint
con velocidad y aceleración limitadas, para probar el mapeo y el esquive de
punta a punta en segundos.

El lidar devuelve los puntos **en el marco del sensor, ejes FLU**, igual que
`airsim_node` de Cosys-AirSim (que invierte y/z de su FRD interno). Así la
cadena de transformaciones del mapper se ejercita completa: si hubiera un
error de signo en `SensorMount`, el mapa saldría espejado y las pruebas lo
detectarían.
"""

import math
from dataclasses import dataclass

import numpy as np

INF = float('inf')


def quat_from_euler(roll, pitch, yaw):
    """[w, x, y, z], convención ZYX de PX4 (FRD -> NED)."""
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return [cr * cp * cy + sr * sp * sy,
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy]


def rot_from_quat(q):
    w, x, y, z = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])


class World:

    def __init__(self, scenario):
        self.scenario = scenario
        self.boxes = []
        for box in scenario.boxes:
            rot = box.rotation()
            self.boxes.append((box.center.astype(float), rot, box.size.astype(float) / 2.0))
        self.cylinders = list(scenario.cylinders)
        self.ground = scenario.ground

    def raycast(self, origin, directions, max_range):
        """Distancia al primer impacto de cada rayo (inf si no choca)."""
        o = np.asarray(origin, dtype=float).reshape(3)
        d = np.asarray(directions, dtype=float).reshape(-1, 3)
        best = np.full(d.shape[0], INF)
        with np.errstate(divide='ignore', invalid='ignore'):
            for center, rot, half in self.boxes:
                ol = rot.T @ (o - center)
                dl = d @ rot          # = (rot.T @ d.T).T
                inv = np.where(np.abs(dl) < 1e-12, 1e12, 1.0 / np.where(dl == 0, 1e-12, dl))
                t1 = (-half - ol) * inv
                t2 = (half - ol) * inv
                tmin = np.max(np.minimum(t1, t2), axis=1)
                tmax = np.min(np.maximum(t1, t2), axis=1)
                hit = (tmax >= np.maximum(tmin, 0.0)) & (tmax > 0)
                t = np.where(tmin > 0, tmin, 0.0)
                best = np.where(hit & (t < best), t, best)
            for cyl in self.cylinders:
                ox, oy = o[0] - cyl.center_ne[0], o[1] - cyl.center_ne[1]
                a = d[:, 0] ** 2 + d[:, 1] ** 2
                b = 2.0 * (ox * d[:, 0] + oy * d[:, 1])
                c = ox * ox + oy * oy - cyl.radius ** 2
                disc = b * b - 4 * a * c
                ok = (disc >= 0) & (a > 1e-12)
                sq = np.sqrt(np.where(ok, disc, 0.0))
                t_near = (-b - sq) / np.where(a > 1e-12, 2 * a, 1.0)
                t_far = (-b + sq) / np.where(a > 1e-12, 2 * a, 1.0)
                t = np.where(t_near > 0, t_near, t_far)
                z = o[2] + t * d[:, 2]
                side = ok & (t > 0) & (z >= cyl.top_d) & (z <= cyl.bottom_d)
                best = np.where(side & (t < best), t, best)
                # Tapa superior (rayos desde arriba).
                tz = (cyl.top_d - o[2]) / np.where(np.abs(d[:, 2]) < 1e-12, 1e-12, d[:, 2])
                px = ox + tz * d[:, 0]
                py = oy + tz * d[:, 1]
                cap = (tz > 0) & (px * px + py * py <= cyl.radius ** 2)
                best = np.where(cap & (tz < best), tz, best)
            if self.ground:
                tg = (0.0 - o[2]) / np.where(d[:, 2] > 1e-12, d[:, 2], np.nan)
                ground = np.isfinite(tg) & (tg > 0)
                best = np.where(ground & (tg < best), tg, best)
        best[best > max_range] = INF
        return best

    def clearance(self, position):
        """Distancia del punto a la superficie sólida más cercana (piso aparte)."""
        p = np.asarray(position, dtype=float)
        best = INF
        for center, rot, half in self.boxes:
            local = rot.T @ (p - center)
            delta = np.maximum(np.abs(local) - half, 0.0)
            best = min(best, float(np.linalg.norm(delta)))
        for cyl in self.cylinders:
            radial = max(0.0, math.hypot(p[0] - cyl.center_ne[0], p[1] - cyl.center_ne[1]) - cyl.radius)
            vertical = max(0.0, cyl.top_d - p[2], p[2] - cyl.bottom_d)
            best = min(best, math.hypot(radial, vertical))
        return best


@dataclass
class LidarModel:
    """Lidar giratorio genérico. Por defecto imita un Livox Mid-360:
    360° horizontal, -7°..+52° vertical, montado arriba del dron."""

    channels: int = 16
    vertical_min_deg: float = -7.0
    vertical_max_deg: float = 52.0
    horizontal_samples: int = 360
    horizontal_fov_deg: float = 360.0
    max_range: float = 40.0
    mount_frd: tuple = (0.0, 0.0, -0.15)
    mount_rpy_deg: tuple = (0.0, 0.0, 0.0)    # como Roll/Pitch/Yaw en settings.json
    noise_std: float = 0.01
    cloud_latency: float = 0.0                # s entre la captura y la entrega de la nube

    def mount_rotation(self):
        roll, pitch, yaw = (math.radians(v) for v in self.mount_rpy_deg)
        return rot_from_quat(quat_from_euler(roll, pitch, yaw))

    def directions_frd(self):
        if self.channels == 1:
            elev = np.array([math.radians(self.vertical_max_deg)])
        else:
            elev = np.radians(np.linspace(self.vertical_min_deg, self.vertical_max_deg, self.channels))
        half = math.radians(self.horizontal_fov_deg) / 2.0
        if self.horizontal_fov_deg >= 359.9:
            azim = np.linspace(-math.pi, math.pi, self.horizontal_samples, endpoint=False)
        else:
            azim = np.linspace(-half, half, self.horizontal_samples)
        ev, az = np.meshgrid(elev, azim, indexing='ij')
        ev, az = ev.reshape(-1), az.reshape(-1)
        # FRD: x adelante, y derecha, z abajo. Elevación positiva = hacia arriba.
        return np.column_stack((np.cos(ev) * np.cos(az), np.cos(ev) * np.sin(az), -np.sin(ev)))

    def scan(self, world, position_ned, q, rng=None):
        """Devuelve los impactos en el marco del sensor, ejes FLU (como AirSim)."""
        dirs_sensor = self.directions_frd()
        rot = rot_from_quat(q)
        origin = np.asarray(position_ned, dtype=float) + rot @ np.asarray(self.mount_frd, dtype=float)
        dirs_world = dirs_sensor @ (rot @ self.mount_rotation()).T
        dist = world.raycast(origin, dirs_world, self.max_range)
        hit = np.isfinite(dist)
        d = dist[hit]
        if self.noise_std > 0 and rng is not None:
            d = d + rng.normal(0.0, self.noise_std, d.shape)
        pts_frd = dirs_sensor[hit] * d[:, None]
        return pts_frd * np.array([1.0, -1.0, -1.0])


class SimVehicle:
    """Multirrotor de juguete: sigue posición + velocidad feedforward como el
    controlador de PX4, con límites de velocidad, aceleración y giro."""

    def __init__(self, position=(0.0, 0.0, 0.0), yaw=0.0, max_speed=4.0, max_accel=3.0,
                 kp=1.0, max_yaw_rate=1.2, land_speed=0.7):
        self.position = np.array(position, dtype=float)
        self.velocity = np.zeros(3)
        self.yaw = float(yaw)
        self.max_speed = max_speed
        self.max_accel = max_accel
        self.kp = kp
        self.max_yaw_rate = max_yaw_rate
        self.land_speed = land_speed
        self.armed = False
        self.offboard = False
        self.landing = False

    @property
    def q(self):
        return quat_from_euler(0.0, 0.0, self.yaw)

    def step(self, dt, setpoint=None, velocity_ff=None):
        if self.landing:
            self.velocity = np.array([0.0, 0.0, self.land_speed])
            self.position = self.position + self.velocity * dt
            if self.position[2] >= 0.0:
                self.position[2] = 0.0
                self.velocity[:] = 0.0
                self.armed = False
                self.landing = False
            return
        if not self.armed or setpoint is None:
            self.velocity[:] = 0.0
            return
        target = np.array(setpoint[:3], dtype=float)
        cmd = self.kp * (target - self.position)
        if velocity_ff is not None:
            cmd[:2] += np.asarray(velocity_ff, dtype=float)
        horiz = math.hypot(cmd[0], cmd[1])
        if horiz > self.max_speed:
            cmd[:2] *= self.max_speed / horiz
        cmd[2] = max(-2.0, min(2.0, cmd[2]))
        dv = cmd - self.velocity
        dv_norm = float(np.linalg.norm(dv))
        if dv_norm > self.max_accel * dt:
            dv *= self.max_accel * dt / dv_norm
        self.velocity = self.velocity + dv
        self.position = self.position + self.velocity * dt
        if self.position[2] > 0.0:          # no atravesar el piso
            self.position[2] = 0.0
            self.velocity[2] = min(self.velocity[2], 0.0)
        yaw_sp = setpoint[3]
        if yaw_sp is not None and math.isfinite(yaw_sp):
            delta = math.atan2(math.sin(yaw_sp - self.yaw), math.cos(yaw_sp - self.yaw))
            limit = self.max_yaw_rate * dt
            self.yaw += max(-limit, min(limit, delta))
            self.yaw = math.atan2(math.sin(self.yaw), math.cos(self.yaw))
