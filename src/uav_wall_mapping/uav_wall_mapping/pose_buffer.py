#!/usr/bin/env python3
"""Historial de poses para transformar cada nube con la pose de SU captura.

Python puro + numpy (sin rclpy). Lo usan `mapper_node` (con los tiempos de
PX4) y `uav_sim.closed_loop` (con el reloj simulado).

Por qué hace falta: una nube del Mid-360 llega 50-150 ms después de capturada
y en ese tiempo el dron se movió y, sobre todo, cabeceó. Transformarla con la
última pose recibida corre el mapa y, con el sensor inclinado, mete el piso o
el techo en la franja de alturas: paredes fantasma en todo el mapa (medido en
docs/investigacion/03_brechas.md, C3: 50 ms de actitud vieja = 864 celdas
falsas). Es lo mismo que hacen PX4-Avoidance (`transform_buffer.cpp`, lineal +
slerp), voxblox y KR (TF en el stamp de la nube).

Posición y actitud llegan por tópicos distintos y a ritmos distintos
(`vehicle_local_position` y `vehicle_attitude`), así que se guardan por
separado y se interpolan cada una: lineal la posición, slerp la actitud.
"""

import bisect
import math
import threading

import numpy as np

from uav_wall_mapping.geometry import quat_slerp


class _Series:
    """Muestras (t, valor) ordenadas por tiempo, recortadas a `horizon` s."""

    def __init__(self, horizon):
        self.horizon = float(horizon)
        self.times = []
        self.values = []

    def add(self, t, value):
        t = float(t)
        if self.times and t < self.times[-1]:
            # Fuera de orden (pasa con BEST_EFFORT): insertar donde va.
            i = bisect.bisect_left(self.times, t)
            self.times.insert(i, t)
            self.values.insert(i, value)
        else:
            self.times.append(t)
            self.values.append(value)
        cutoff = self.times[-1] - self.horizon
        drop = bisect.bisect_left(self.times, cutoff)
        if drop > 0 and drop < len(self.times):
            del self.times[:drop]
            del self.values[:drop]

    def bracket(self, t):
        """`(i0, i1, tau)` para interpolar en t, o None si t cae afuera."""
        if not self.times:
            return None
        i = bisect.bisect_left(self.times, t)
        if i == 0:
            return (0, 0, 0.0) if abs(self.times[0] - t) < 1e-9 else None
        if i >= len(self.times):
            return None
        t0, t1 = self.times[i - 1], self.times[i]
        tau = 0.0 if t1 - t0 < 1e-9 else (t - t0) / (t1 - t0)
        return i - 1, i, tau


class PoseBuffer:
    """Posición NED y actitud (cuaternión FRD->NED [w, x, y, z]) con su instante.

    `lookup(t)` devuelve la pose interpolada en `t` o None si `t` cae fuera
    del historial (más vieja que `horizon`, o más nueva que la última muestra
    por más de `max_extrapolation`: en ese caso se usa la última, sin
    extrapolar, porque la pose de PX4 llega a 50-250 Hz y unos ms de diferencia
    no importan).
    """

    def __init__(self, horizon=2.0, max_extrapolation=0.1):
        self.max_extrapolation = float(max_extrapolation)
        self._pos = _Series(horizon)
        self._att = _Series(horizon)
        self._lock = threading.Lock()

    def add_position(self, t, position_ned):
        """Las muestras no finitas (PX4 manda NaN en x/y cuando el estimador no
        tiene posición) se descartan: una sola NaN en el historial contamina
        toda interpolación que la toque."""
        p = np.asarray(position_ned, dtype=float).reshape(3)
        if not math.isfinite(float(t)) or not np.all(np.isfinite(p)):
            return
        with self._lock:
            self._pos.add(t, p.copy())

    def add_attitude(self, t, q):
        q = np.asarray(q, dtype=float).reshape(4)
        norm = float(np.linalg.norm(q))
        if norm < 1e-9 or not math.isfinite(norm) or not math.isfinite(float(t)):
            return
        with self._lock:
            self._att.add(t, q / norm)

    def has_attitude(self):
        with self._lock:
            return bool(self._att.times)

    def newest_time(self):
        with self._lock:
            if not self._pos.times:
                return None
            return self._pos.times[-1]

    def span(self):
        """(más vieja, más nueva) del historial de posición, o None."""
        with self._lock:
            if not self._pos.times:
                return None
            return self._pos.times[0], self._pos.times[-1]

    def clear(self):
        with self._lock:
            self._pos = _Series(self._pos.horizon)
            self._att = _Series(self._att.horizon)

    def _sample(self, series, t, interpolate):
        if not series.times:
            return None
        newest = series.times[-1]
        if t >= newest:
            if t - newest <= self.max_extrapolation:
                return series.values[-1]
            return None
        found = series.bracket(t)
        if found is None:
            return None
        i0, i1, tau = found
        return interpolate(series.values[i0], series.values[i1], tau)

    def lookup(self, t):
        """`(posición, q)` en el instante t, o None si no se puede."""
        t = float(t)
        if not math.isfinite(t):
            return None
        with self._lock:
            pos = self._sample(self._pos, t, lambda a, b, tau: a + (b - a) * tau)
            q = self._sample(self._att, t, quat_slerp)
        if pos is None or q is None:
            return None
        return np.asarray(pos, dtype=float), np.asarray(q, dtype=float)

    def latest(self):
        """La última pose completa `(t, posición, q)`, o None."""
        with self._lock:
            if not self._pos.times or not self._att.times:
                return None
            return self._pos.times[-1], self._pos.values[-1].copy(), self._att.values[-1].copy()
