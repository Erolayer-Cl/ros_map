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

**Frenado físico** (como la Collision Prevention de PX4 y `AC_Avoid` de
ArduPilot, ver docs/evasion-entornos-reales.md): la velocidad no baja con una
rampa fija entre dos distancias, sino con lo que hace falta para frenar a
tiempo: con `d` libres en la dirección de avance,
`v_max = −a·t + √((a·t)² + 2·a·(d − d_keep))`, donde `a` es la desaceleración
supuesta (`max_decel`), `t` la latencia configurada más la edad del dato y
`d_keep` la distancia a la que tiene que quedar quieto. Y el alcance con el que
un obstáculo "tapa" sectores crece con la distancia de frenado: a 5 m/s se mira
mucho más lejos que a 1 m/s.
"""

import math
from dataclasses import dataclass

import numpy as np

INF = float('inf')


def wrap_pi(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def angle_diff(a, b):
    return abs(wrap_pi(a - b))


def braking_distance(speed, decel, delay):
    """Distancia que recorre el dron hasta quedar quieto: `v·t + v²/(2a)`.

    `delay` (s) es todo lo que pasa antes de que empiece a frenar de verdad:
    edad del dato del sensor + procesamiento + seguimiento del controlador
    (lo que PX4 llama `CP_DELAY`).
    """
    speed = max(0.0, float(speed))
    return speed * max(0.0, delay) + speed * speed / (2.0 * max(decel, 1e-3))


def max_safe_speed(distance, decel, delay):
    """Velocidad máxima con la que se puede frenar en `distance` metros.

    Es la inversa de `braking_distance`: v·t + v²/(2a) = d  =>
    v = −a·t + √((a·t)² + 2·a·d). Misma fórmula que la Collision Prevention de
    PX4 (`computeMaxSpeedFromDistance`, sin el término de jerk) y que
    `AC_Avoid::get_max_speed` de ArduPilot con t = 0.
    """
    if not math.isfinite(distance):
        return INF
    if distance <= 0.0:
        return 0.0
    a = max(decel, 1e-3)
    at = a * max(0.0, delay)
    return -at + math.sqrt(at * at + 2.0 * a * distance)


@dataclass
class AvoidConfig:
    robot_radius: float = 0.45      # m: radio del dron contando hélices
    safety_margin: float = 0.5      # m extra alrededor de cada obstáculo
    lookahead: float = 4.0          # m: mínimo; se agranda con la distancia de frenado
    stop_distance: float = 1.3      # m (desde el centro): con esto al frente no se avanza
    emergency_distance: float = 0.9  # m: más cerca que esto, alejarse
    max_decel: float = 1.0          # m/s² que se suponen para frenar (ArduPilot usa ≤ 1)
    latency: float = 0.4            # s sensor→frenado sin contar la edad del dato (PX4 CP_DELAY)
    goal_weight: float = 5.0
    heading_weight: float = 1.0
    previous_weight: float = 2.0
    corridor_mode: bool = False     # paso angosto: ir derecho si el corredor está libre
    escape_speed: float = 0.75      # m/s al alejarse en una emergencia (ArduPilot AVOID_BACKUP_SPD)


@dataclass
class AvoidCommand:
    direction: float = None    # rumbo NED a seguir; None = no hay salida
    speed_scale: float = 1.0
    max_speed: float = INF     # m/s: tope físico para poder frenar a tiempo
    ahead: float = INF         # m libres en el corredor de la dirección elegida
    block_range: float = 0.0   # m hasta donde un obstáculo tapa sectores
    deviated: bool = False     # True si no se va derecho al objetivo
    blocked: bool = False      # ningún sector libre
    unseen: bool = False       # bloqueado sólo porque lo libre no se está viendo: girar
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

    def _escape(self, cmd, ranges, live, idx, seen):
        """Huida de emergencia hacia el sector visto más opuesto al obstáculo
        que se aleje de verdad (el cono de ±25° alrededor tiene que estar más
        libre que la distancia actual). Sin ninguno: frenar en el lugar, que
        retroceder a ciegas (cámara de 87°) es peor que quedarse quieto."""
        c = self.cfg
        cmd.emergency = True
        cmd.deviated = True
        cmd.nearest_bearing = float(self.centers[idx])
        cmd.nearest = float(live[idx])
        away = wrap_pi(cmd.nearest_bearing + math.pi)
        need = max(cmd.nearest + 0.3, c.emergency_distance + 0.4)
        best, best_cost = None, INF
        for i in np.nonzero(seen)[0]:
            b = float(self.centers[i])
            # Nunca con componente hacia el obstáculo (>= 90° de él), y con el
            # cono de esa dirección más libre que lo que se tiene encima.
            if angle_diff(b, cmd.nearest_bearing) < math.pi / 2.0 or \
                    self.clearance(ranges, b) <= need:
                continue
            cost = angle_diff(b, away)
            if cost < best_cost:
                best, best_cost = b, cost
        if best is None:
            cmd.direction = None
            cmd.blocked = True
            cmd.max_speed = 0.0
            cmd.reason = (f'obstáculo a {cmd.nearest:.1f} m '
                          f'({math.degrees(cmd.nearest_bearing):.0f}°) y ninguna salida vista: '
                          'frenando en el lugar')
            self._previous = None
            return cmd
        cmd.direction = wrap_pi(best)
        cmd.max_speed = c.escape_speed
        cmd.reason = (f'obstáculo a {cmd.nearest:.1f} m '
                      f'({math.degrees(cmd.nearest_bearing):.0f}°): alejándose')
        self._previous = cmd.direction
        return cmd

    def keep_distance(self):
        """Distancia (desde el centro del dron) a la que tiene que quedar frenado."""
        c = self.cfg
        return max(c.stop_distance, c.robot_radius + c.safety_margin)

    def free_travel(self, ranges, bearing):
        """Metros que se puede avanzar por `bearing` sin quedar a menos de
        `keep_distance()` de ningún obstáculo del corredor de avance.

        Cada sector se proyecta sobre la dirección de avance: un obstáculo a
        distancia d y ángulo δ queda a `a = d·cos δ` por delante y a
        `l = d·|sin δ|` del eje. Sólo cuentan los del corredor que barre el dron
        (`l` < radio + margen, el mismo ancho con el que el VFH agranda los
        obstáculos: lo que el VFH deja pasar por el costado no frena), y para
        cada uno el dron puede avanzar `a − √(K² − l²)` antes de quedar a K de
        él. Así una pared paralela al costado no frena, y una que se cruza en
        el camino sí.
        """
        c = self.cfg
        r = np.asarray(ranges, dtype=float)
        finite = np.isfinite(r)
        if not np.any(finite):
            return INF
        r = r[finite]
        delta = np.arctan2(np.sin(self.centers[finite] - bearing),
                           np.cos(self.centers[finite] - bearing))
        along = r * np.cos(delta)
        lateral = r * np.abs(np.sin(delta))
        hit = (along > 0.0) & (lateral < c.robot_radius + c.safety_margin)
        if not np.any(hit):
            return INF
        keep = self.keep_distance()
        travel = along[hit] - np.sqrt(np.maximum(keep * keep - lateral[hit] ** 2, 0.0))
        return float(travel.min())

    def limit_for(self, ranges, bearing, data_age=0.0, sensor_range=INF):
        """`(avance_libre, v_max)` para ir por `bearing` con sectores de esta edad.

        Si en el corredor no hay nada, lo libre llega hasta el alcance del
        sensor (`sensor_range`): más allá no se sabe, y a esa velocidad hay que
        poder frenar antes de lo que aparezca en el borde.
        """
        c = self.cfg
        travel = min(self.free_travel(ranges, bearing), sensor_range - self.keep_distance())
        delay = c.latency + max(0.0, float(data_age))
        return travel, max_safe_speed(travel, c.max_decel, delay)

    def update(self, ranges, goal_bearing, goal_distance, heading, speed=0.0, data_age=0.0,
               sensor_range=INF, live_ranges=None, seen=None):
        """Decide la dirección y el tope de velocidad del próximo ciclo.

        `speed`: velocidad horizontal actual o pedida, la mayor (m/s): define
        hasta dónde hay que mirar. `data_age`: edad de los sectores (s): se suma
        a `latency` porque el dron siguió moviéndose desde que se midieron.
        `sensor_range`: alcance de los sectores; un corredor libre se toma como
        libre sólo hasta ahí (más allá no se sabe). `live_ranges`: sectores
        sólo de los sensores en vivo; si vienen, la emergencia se decide con
        ellos (la memoria del mapa frena y desvía, pero no hace huir).
        `seen`: máscara de sectores que un sensor miró hace poco; los desvíos
        y la huida sólo van hacia ésos (un sector sin datos no es libre).
        """
        c = self.cfg
        ranges = np.asarray(ranges, dtype=float)
        delay = c.latency + max(0.0, float(data_age))
        nearest_idx = int(np.argmin(ranges))
        nearest = float(ranges[nearest_idx])
        cmd = AvoidCommand(nearest=nearest, nearest_bearing=float(self.centers[nearest_idx]))

        seen = (np.ones(self.n_bins, dtype=bool) if seen is None
                else np.asarray(seen, dtype=bool).reshape(self.n_bins))
        live = ranges if live_ranges is None else np.asarray(live_ranges, dtype=float)
        live_idx = int(np.argmin(live))
        if float(live[live_idx]) < c.emergency_distance:
            # Demasiado cerca (y lo confirma el sensor en vivo): alejarse, pero
            # sólo hacia un sector visto y más libre que lo que se tiene encima.
            return self._escape(cmd, ranges, live, live_idx, seen)

        # Hasta dónde mirar: nunca menos que lo que se tarda en frenar a la
        # velocidad actual (con la latencia), más la distancia a mantener.
        # Un obstáculo más allá del objetivo no debería impedir llegar a él
        # (típico: una frontera de exploración pegada a una pared).
        inflate = c.robot_radius + c.safety_margin
        look = max(c.lookahead,
                   braking_distance(speed, c.max_decel, delay) + self.keep_distance())
        block_range = min(look, goal_distance + inflate)
        cmd.block_range = block_range
        blocked = self.blocked_mask(ranges, block_range)

        # En un paso angosto (lo marca el A*: el camino pasa por un hueco más
        # angosto que radio + margen normal) se confía en la prueba de corredor:
        # desde lejos, el agrandado angular de los dos bordes de una puerta tapa
        # el sector central por cuantización (sectores de 5°) aunque el dron
        # entre de sobra por el medio.
        goal_free = not blocked[self._bin(goal_bearing)]
        if not goal_free and c.corridor_mode:
            goal_free = self.free_travel(ranges, goal_bearing) > 0.5
        if goal_free:
            cmd.direction = goal_bearing
            cmd.deviated = False
        else:
            # Desvío: sólo hacia sectores libres Y vistos hace poco. Con una
            # cámara, "sin puntos" fuera del campo no quiere decir libre.
            free = np.nonzero(~blocked & seen)[0]
            if free.size == 0:
                cmd.blocked = True
                cmd.max_speed = 0.0
                cmd.unseen = bool(np.any(~blocked))
                cmd.reason = ('sin salida vista: girando hasta ver un sector libre' if cmd.unseen
                              else 'rodeado: ningún sector libre')
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

        # Frenado físico: lo libre en el corredor de avance (o el alcance del
        # sensor, si no hay nada) tiene que alcanzar para frenar a tiempo.
        cmd.ahead, cmd.max_speed = self.limit_for(ranges, cmd.direction, data_age, sensor_range)
        cmd.speed_scale = 1.0
        self._previous = cmd.direction
        return cmd
