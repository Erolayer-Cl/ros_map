#!/usr/bin/env python3
"""Evasión reactiva tipo VFH+ sobre el histograma polar de obstáculos.

Python puro (sin rclpy). Entrada: `ranges[i]` = distancia horizontal al
obstáculo más cercano en el sector i (72 sectores de 5°, rumbos NED, sector 0
= Norte), como lo publica `mapper_node` en `/obstacles/sectors`.

Por qué VFH+ y no el esquive "bug" de 5 haces del repo de referencia: con una
nube densa cada sector tiene su distancia, así que no hace falta adivinar hacia
dónde termina la pared. Cada obstáculo se **agranda** según el radio del dron
más un margen (un obstáculo cercano tapa más sectores que uno lejano), y se
elige el sector libre que menos se aparta del objetivo. Eso funciona igual con
una pared plana que con una irregular: lo que importa es qué sectores quedan
libres, no la forma.

El esquive local no resuelve trampas en U ni laberintos: eso lo hace el
planificador global A* sobre el mapa (`planner.py`). Acá sólo se decide la
dirección de los próximos metros.
"""

import math
from dataclasses import dataclass

import numpy as np

INF = float('inf')


def wrap_pi(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def angle_diff(a, b):
    return abs(wrap_pi(a - b))


@dataclass
class AvoidConfig:
    robot_radius: float = 0.45      # m: radio del dron contando hélices
    safety_margin: float = 0.5      # m extra alrededor de cada obstáculo
    lookahead: float = 4.0          # m: más lejos que esto no bloquea
    slow_distance: float = 3.5      # m: desde acá se empieza a frenar
    stop_distance: float = 1.3      # m: con esto al frente no se avanza
    emergency_distance: float = 0.9  # m: más cerca que esto, alejarse
    goal_weight: float = 5.0
    heading_weight: float = 1.0
    previous_weight: float = 2.0
    min_speed_scale: float = 0.25   # al bordear no frenar del todo


@dataclass
class AvoidCommand:
    direction: float = None    # rumbo NED a seguir; None = no hay salida
    speed_scale: float = 1.0
    deviated: bool = False     # True si no se va derecho al objetivo
    blocked: bool = False      # ningún sector libre
    emergency: bool = False    # algo demasiado cerca: alejarse
    nearest: float = INF       # distancia mínima en cualquier dirección
    nearest_bearing: float = 0.0
    reason: str = ''


class VfhAvoider:

    def __init__(self, config=None, n_bins=72):
        self.cfg = config or AvoidConfig()
        self.n_bins = n_bins
        self.increment = 2.0 * math.pi / n_bins
        self.centers = np.arange(n_bins) * self.increment
        self._previous = None

    def reset(self):
        self._previous = None

    def _bin(self, bearing):
        return int(round(wrap_pi(bearing) / self.increment)) % self.n_bins

    def blocked_mask(self, ranges, block_range):
        """Sectores tapados después de agrandar cada obstáculo.

        Un obstáculo a distancia d ocupa ±asin((radio + margen) / d) alrededor
        de su sector: a 1 m tapa casi 120°, a 8 m apenas 14°.
        """
        c = self.cfg
        inflate = c.robot_radius + c.safety_margin
        blocked = np.zeros(self.n_bins, dtype=bool)
        close = np.nonzero(np.asarray(ranges) < block_range)[0]
        for i in close:
            d = max(float(ranges[i]), 1e-3)
            half = math.asin(min(1.0, inflate / d)) + self.increment / 2.0
            diff = np.abs(np.arctan2(np.sin(self.centers - self.centers[i]),
                                     np.cos(self.centers - self.centers[i])))
            blocked |= diff <= half
        return blocked

    def clearance(self, ranges, bearing, half_width=math.radians(25)):
        """Distancia libre en un cono alrededor de un rumbo."""
        diff = np.abs(np.arctan2(np.sin(self.centers - bearing), np.cos(self.centers - bearing)))
        window = np.asarray(ranges)[diff <= half_width]
        return float(window.min()) if window.size else INF

    def update(self, ranges, goal_bearing, goal_distance, heading):
        c = self.cfg
        ranges = np.asarray(ranges, dtype=float)
        nearest_idx = int(np.argmin(ranges))
        nearest = float(ranges[nearest_idx])
        cmd = AvoidCommand(nearest=nearest, nearest_bearing=float(self.centers[nearest_idx]))

        if nearest < c.emergency_distance:
            # Demasiado cerca: lo único sensato es alejarse del punto más cercano.
            cmd.emergency = True
            cmd.direction = wrap_pi(cmd.nearest_bearing + math.pi)
            cmd.speed_scale = c.min_speed_scale
            cmd.deviated = True
            cmd.reason = (f'obstáculo a {nearest:.1f} m '
                          f'({math.degrees(cmd.nearest_bearing):.0f}°): alejándose')
            self._previous = cmd.direction
            return cmd

        # Un obstáculo más allá del objetivo no debería impedir llegar a él
        # (típico: una frontera de exploración pegada a una pared).
        inflate = c.robot_radius + c.safety_margin
        block_range = min(c.lookahead, goal_distance + inflate)
        blocked = self.blocked_mask(ranges, block_range)

        if not blocked[self._bin(goal_bearing)]:
            cmd.direction = goal_bearing
            cmd.deviated = False
        else:
            free = np.nonzero(~blocked)[0]
            if free.size == 0:
                cmd.blocked = True
                cmd.reason = 'rodeado: ningún sector libre'
                self._previous = None
                return cmd
            candidates = self.centers[free]
            cost = c.goal_weight * np.abs(np.arctan2(np.sin(candidates - goal_bearing),
                                                     np.cos(candidates - goal_bearing)))
            cost += c.heading_weight * np.abs(np.arctan2(np.sin(candidates - heading),
                                                         np.cos(candidates - heading)))
            if self._previous is not None:
                # Inercia: sin esto, ante una pared simétrica el dron elegiría
                # izquierda y derecha en ciclos alternos y no iría a ningún lado.
                cost += c.previous_weight * np.abs(np.arctan2(np.sin(candidates - self._previous),
                                                              np.cos(candidates - self._previous)))
            cmd.direction = float(wrap_pi(candidates[int(np.argmin(cost))]))
            cmd.deviated = True
            side = 'derecha' if wrap_pi(cmd.direction - goal_bearing) > 0 else 'izquierda'
            cmd.reason = (f'bordeando por la {side} '
                          f'({math.degrees(angle_diff(cmd.direction, goal_bearing)):.0f}° del objetivo)')

        ahead = self.clearance(ranges, cmd.direction)
        span = max(c.slow_distance - c.stop_distance, 1e-3)
        scale = min(1.0, max(0.0, (ahead - c.stop_distance) / span))
        if cmd.deviated:
            scale = max(scale, c.min_speed_scale)
        cmd.speed_scale = scale
        self._previous = cmd.direction
        return cmd
