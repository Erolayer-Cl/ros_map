#!/usr/bin/env python3
"""Máquina de estados de la misión, sin ROS.

`mission_node.py` es un adaptador fino: le pasa a esta clase la posición de
PX4, los sectores de obstáculos y el mapa, llama a `step()` a 20 Hz y publica
lo que sale (setpoint, comandos, estado). Separarlo así permite correr la
misión entera en un intérprete pelado contra un mundo simulado
(`uav_sim/closed_loop.py`) y probar el esquive y el mapeo sin Unreal ni ROS.

Dos misiones:

- **Waypoints** (`/mission/start`): vuela la lista `waypoints` esquivando.
- **Mapeo** (`/mission/map`): explora el área `map_mission.area` (fronteras o
  barrido), dibuja las paredes en el mapa, guarda los archivos y vuelve.

Coordenadas: NED de PX4. `waypoints` y `map_mission.area` son **relativos al
punto de despegue** (norte, este) y las alturas son positivas hacia arriba.
"""

import math
from enum import IntEnum

import numpy as np

from uav_mission.avoidance import AvoidConfig, VfhAvoider, angle_diff, wrap_pi
from uav_mission.exploration import (coverage, find_frontiers, lawnmower_waypoints,
                                     select_frontier)
from uav_mission.planner import GridPlanner, path_lookahead


class State(IntEnum):
    # Los valores coinciden con las constantes STATE_* de uav_map_msgs/MissionStatus.
    IDLE = 0
    ARMING = 1
    TAKEOFF = 2
    NAVIGATE = 3
    AVOID = 4
    HOLD = 5
    RETURN = 6
    LAND = 7
    DONE = 8
    ABORTED = 9
    PREEMPTED = 10
    FAILED = 11
    SCAN = 12


class MissionType(IntEnum):
    NONE = 0
    WAYPOINTS = 1
    MAPPING = 2


# Estados en los que este nodo maneja el dron y tiene que sostener el flujo de
# setpoints: si se corta, PX4 sale de OFFBOARD por failsafe (COM_OF_LOSS_T).
FLYING_STATES = {State.ARMING, State.TAKEOFF, State.NAVIGATE, State.AVOID,
                 State.HOLD, State.RETURN, State.SCAN}
MOVING_STATES = {State.NAVIGATE, State.AVOID, State.RETURN}

DEFAULTS = {
    # --- misión de waypoints ---
    'waypoints': [15.0, 0.0, 3.0],      # norte, este, altura (relativos al despegue)
    'takeoff_altitude': 3.0,
    'land_at_end': True,
    # --- vuelo ---
    'cruise_speed': 1.5,                # m/s
    'climb_speed': 1.0,
    'acceptance_radius': 0.8,
    'max_lead': 1.5,                    # m que la zanahoria puede adelantarse al dron
    'nav.lead_time': 1.0,               # s: además, la zanahoria no se adelanta más que v·esto
    'arm_timeout': 15.0,
    'control_rate': 20.0,
    'nav.accel_ramp_sec': 1.5,
    'nav.yaw_mode': 'travel',           # travel | goal | fixed
    'nav.max_yaw_rate': 0.8,            # rad/s
    'nav.use_planner': True,
    'nav.replan_period': 1.0,
    'nav.path_lookahead': 2.5,
    'nav.stuck_timeout': 25.0,          # s sin acercarse al objetivo = inalcanzable
    'nav.stuck_progress': 1.0,          # m que hay que ganar en ese tiempo
    'nav.velocity_feedforward': True,
    # --- planificador ---
    'plan.resolution': 0.3,
    'plan.inflation_radius': 0.65,      # radio del dron + 0.2 m (duro); el resto es costo suave
    'plan.cost_radius': 2.0,
    'plan.unknown_penalty': 0.5,
    'plan.max_expansions': 60000,
    'plan.window_margin': 12.0,
    'plan.max_failures': 3,
    # --- evasión ---
    'avoid.enabled': True,
    'avoid.robot_radius': 0.45,
    'avoid.safety_margin': 0.35,
    'avoid.lookahead': 4.0,             # m mínimos; crece con la distancia de frenado
    'avoid.stop_distance': 1.3,         # m desde el centro: frenado del todo
    'avoid.emergency_distance': 0.7,    # < radio + margen: si no, bordear dispara la huida
    'avoid.escape_speed': 0.75,         # m/s al alejarse (ArduPilot AVOID_BACKUP_SPD)
    'avoid.seen_timeout': 1.0,          # s: un sector no mirado en este tiempo es desconocido
    'avoid.fov_check_deg': 20.0,        # ± grados alrededor del avance que tienen que estar vistos
    'avoid.max_decel': 1.0,             # m/s² supuestos para frenar (ArduPilot: ≤ 1)
    'avoid.latency': 0.4,               # s sensor→frenado, sin la edad del dato (PX4 CP_DELAY)
    'avoid.trigger_hold': 0.2,          # s que el desvío tiene que persistir
    'avoid.emergency_hold': 0.15,
    'avoid.blocked_timeout': 10.0,      # s rodeado antes de dar el objetivo por perdido
    'avoid.sensor_timeout': 0.5,        # s de edad (desde la captura) para usar los sectores
    'avoid.hold_on_sensor_loss': True,
    'avoid.require_sensors': True,      # no despegar sin datos de obstáculos
    'avoid.on_failure': 'return',       # return | hold | skip
    # --- misión de mapeo ---
    'map_mission.area': [-5.0, -15.0, 25.0, 15.0],   # n0, e0, n1, e1 relativos al despegue
    'map_mission.altitude': 2.5,
    'map_mission.strategy': 'frontier',              # frontier | lawnmower
    'map_mission.lane_spacing': 6.0,
    'map_mission.max_duration': 600.0,
    'map_mission.min_frontier_length': 0.9,          # m
    'map_mission.coverage_goal': 0.98,
    'map_mission.spin_at_start': True,
    'map_mission.spin_at_goals': False,
    'map_mission.reset_map': True,
    'map_mission.save_on_finish': True,
    'map_mission.return_home': True,
    'map_mission.goal_timeout': 60.0,
    'map_mission.max_goal_failures': 12,
    'scan.yaw_rate': 0.6,
    # --- failsafes (docs/evasion-entornos-reales.md) ---
    'nav.pose_timeout': 0.5,            # s sin posición válida de PX4: frenar (hold)
    'nav.pose_loss_land_after': 2.0,    # s sin posición: aterrizar en el lugar (0 = nunca)
    'avoid.sensor_loss_land_after': 10.0,  # s sin datos de obstáculos en vuelo: aterrizar (0 = nunca)
    'avoid.on_return_failure': 'land',  # land | hold: si no se puede volver al despegue
    'nav.max_tracking_error': 0.75,     # m (KR usa 0.75): más lejos de la referencia = frenar (0 = off)
    'nav.tracking_brake_sec': 1.0,      # s que se frena al dispararse
    'safety.geofence': [],              # [n0, e0, n1, e1] relativos al despegue; [] = automática
    'safety.geofence_margin': 3.0,      # m alrededor de map_mission.area (geocerca automática)
    'safety.min_altitude': 0.5,         # m sobre el despegue: la referencia nunca baja de esto
    'safety.max_altitude': 20.0,        # m sobre el despegue: ni sube de esto
}


def distance_2d(a, b):
    return math.hypot(b[0] - a[0], b[1] - a[1])


def bearing_between(a, b):
    return math.atan2(b[1] - a[1], b[0] - a[0])


class MissionCore:

    def __init__(self, param=None, log=None):
        self.param = param or (lambda name: DEFAULTS[name])
        self.log = log or (lambda level, msg: None)

        self.state = State.IDLE
        self.mission = MissionType.NONE
        self.message = 'esperando /mission/start o /mission/map'

        # --- entradas ---
        self.position = None
        self.velocity = (0.0, 0.0, 0.0)
        self.heading = 0.0
        self.position_valid = False
        self.armed = False
        self.offboard = False
        self._sectors = None
        self._sectors_stamp = -math.inf
        self._sectors_ever = False
        self._sectors_range = math.inf
        self._sectors_live = None
        self._sectors_seen_age = None
        self._fov_scale = 1.0
        self.snapshot = None
        self.walls_summary = (0, 0, 0)

        # --- salidas ---
        self.setpoint = None          # (n, e, d, yaw)
        self.velocity_setpoint = None  # (vn, ve) feedforward o None
        self.stream_offboard = False
        self._commands = []

        # --- estado interno ---
        self.home = None
        self._carrot = None
        self._yaw_sp = 0.0
        self._now = 0.0
        self._last_step = None
        self._dt = 0.05
        self._arm_ticks = 0
        self._arm_attempts = 0
        self._last_arm_request = -math.inf
        self._land_requested = False
        self._takeoff_xy = None
        self._hold_anchor = None
        self._paused_from = None
        self._ramp_start = None
        self._scan_swept = 0.0
        self._scan_return = None
        self._distance_traveled = 0.0
        self._last_position = None

        self._waypoints = []
        self._wp_index = 0
        self._goal = None
        self._goal_kind = ''
        self._goal_id = 0
        self._goal_started = 0.0
        self._progress_best = math.inf
        self._progress_time = 0.0
        self._path = None
        self._path_goal_id = -1
        self._plan_pending = False
        self._last_plan_time = -math.inf
        self._plan_failures = 0
        self._plan_reason = ''
        self._goal_reachable = None

        self._avoid_dir_prev = None
        self._deviate_since = None
        self._freeze_anchor = None     # dónde se empezó a frenar (se sostiene ahí)
        self._emergency_since = None
        self._blocked_since = None
        self._avoid_active = False
        self._avoid_reason = ''
        self._sensor_loss_warned = False
        self._nearest = math.inf
        self._nearest_bearing = 0.0
        self._speed_cap = math.inf
        self._ahead = math.inf

        # --- failsafes ---
        self._pose_time = None          # instante de la última posición válida (None = "ahora")
        self._pose_lost_since = None
        self._sensor_lost_since = None
        self._tracking_brake_until = -math.inf
        self._fence = None              # (n0, e0, n1, e1) absolutos, o None
        self._fence_warned = False

        self._map_area = None
        self._map_start = 0.0
        self._lanes = []
        self._blacklist = []
        self._goal_failures = 0
        self.goals_reached = 0
        self.coverage = 0.0
        self.frontiers_open = 0
        self._last_frontier_check = -math.inf

        self.avoider = VfhAvoider(self._avoid_config())

    # ------------------------------------------------------------------
    # Parámetros
    # ------------------------------------------------------------------
    def p(self, name):
        return self.param(name)

    def _avoid_config(self):
        return AvoidConfig(
            robot_radius=self.p('avoid.robot_radius'),
            safety_margin=self.p('avoid.safety_margin'),
            lookahead=self.p('avoid.lookahead'),
            stop_distance=self.p('avoid.stop_distance'),
            emergency_distance=self.p('avoid.emergency_distance'),
            escape_speed=self.p('avoid.escape_speed'),
            max_decel=self.p('avoid.max_decel'),
            latency=self.p('avoid.latency'))

    def make_planner(self):
        """El nodo y el simulador construyen el planificador con esto, para que
        corra fuera del lazo de control (es lo único lento)."""
        return GridPlanner(resolution=self.p('plan.resolution'),
                           inflation_radius=self.p('plan.inflation_radius'),
                           cost_radius=self.p('plan.cost_radius'),
                           unknown_penalty=self.p('plan.unknown_penalty'),
                           max_expansions=int(self.p('plan.max_expansions')),
                           window_margin=self.p('plan.window_margin'))

    def _load_waypoints(self):
        flat = [float(v) for v in (self.p('waypoints') or [])]
        if len(flat) % 3 != 0 or not flat:
            self.log('error', f'waypoints tiene {len(flat)} valores: tiene que ser múltiplo '
                              'de 3 (norte, este, altura)')
            return []
        wps = []
        for i in range(0, len(flat), 3):
            if flat[i + 2] <= 0.0:
                self.log('warn', f'waypoint {i // 3} con altura {flat[i + 2]} m: se espera '
                                 'altura positiva sobre el despegue')
            wps.append((flat[i], flat[i + 1], flat[i + 2]))
        return wps

    # ------------------------------------------------------------------
    # Entradas
    # ------------------------------------------------------------------
    def update_vehicle(self, position, velocity, heading, valid=True, stamp=None):
        """Posición de PX4. `stamp`: cuándo llegó (para el timeout de pose); sin
        stamp se la toma como recién llegada."""
        self.position_valid = bool(valid)
        if not valid:
            return
        self._pose_time = None if stamp is None else float(stamp)
        position = (float(position[0]), float(position[1]), float(position[2]))
        if self.armed and self._last_position is not None:
            self._distance_traveled += math.dist(position, self._last_position)
        self._last_position = position
        self.position = position
        self.velocity = tuple(float(v) for v in velocity)
        self.heading = float(heading)

    def update_status(self, armed, offboard):
        was_offboard = self.offboard
        self.armed = bool(armed)
        self.offboard = bool(offboard)
        # Si perdemos OFFBOARD sin pedirlo, el piloto tomó el control desde QGC
        # o la radio. Pelear mandando setpoints sería lo peor: nos apartamos.
        if (was_offboard and not self.offboard and self.state in FLYING_STATES
                and self.state != State.ARMING and not self._land_requested):
            self._enter(State.PREEMPTED, 'el dron salió de OFFBOARD desde afuera: control cedido')

    def update_sectors(self, ranges, stamp, range_max=math.inf, live=None, seen_age=None):
        """Sectores de obstáculos. `stamp`: instante de CAPTURA del dato más
        nuevo que los formó (no el de llegada): con eso se sabe su edad.
        `live`: los mismos sectores sólo con sensores en vivo (sin la memoria
        del mapa); la emergencia se decide con ellos. `seen_age`: por sector,
        s entre la última vez que un sensor lo miró y `stamp` (inf = nunca)."""
        self._sectors = np.asarray(ranges, dtype=float)
        self._sectors_live = (np.asarray(live, dtype=float)
                              if live is not None and len(live) == len(self._sectors) else None)
        self._sectors_seen_age = (np.asarray(seen_age, dtype=float)
                                  if seen_age is not None and len(seen_age) == len(self._sectors)
                                  else None)
        self._sectors_stamp = float(stamp)
        self._sectors_ever = True
        self._sectors_range = float(range_max) if range_max and range_max > 0 else math.inf
        if len(self._sectors) != self.avoider.n_bins:
            self.avoider = VfhAvoider(self._avoid_config(), n_bins=len(self._sectors))

    def update_map(self, snapshot):
        self.snapshot = snapshot

    def update_walls_summary(self, flat, irregular, obstacles):
        self.walls_summary = (int(flat), int(irregular), int(obstacles))

    def pop_commands(self):
        out, self._commands = self._commands, []
        return out

    def _sectors_fresh(self, now):
        return self._sectors is not None and now - self._sectors_stamp <= self.p('avoid.sensor_timeout')

    def _sectors_age(self, now):
        return max(0.0, now - self._sectors_stamp) if self._sectors is not None else math.inf

    def _seen_mask(self, now):
        """Sectores que un sensor miró hace menos de `avoid.seen_timeout` s
        (None = no se sabe: se asumen todos vistos, como antes)."""
        if self._sectors_seen_age is None:
            return None
        age = self._sectors_seen_age + self._sectors_age(now)
        return age <= float(self.p('avoid.seen_timeout'))

    def _fov_factor(self, seen, direction):
        """Como `scaleToFOV` de PX4-Avoidance: la fracción de los sectores
        alrededor del avance que se están viendo. 0 = avanzar a ciegas: no se
        avanza, se gira hasta tenerlo en el campo del sensor."""
        if seen is None:
            return 1.0
        n = len(seen)
        centers = np.arange(n) * (2.0 * math.pi / n)
        diff = np.abs(np.arctan2(np.sin(centers - direction), np.cos(centers - direction)))
        window = diff <= math.radians(float(self.p('avoid.fov_check_deg')))
        if not np.any(window):
            return 1.0
        return float(np.count_nonzero(seen[window])) / float(np.count_nonzero(window))

    # ------------------------------------------------------------------
    # Servicios
    # ------------------------------------------------------------------
    def _can_start(self, now):
        if self.state in FLYING_STATES and self.state != State.HOLD:
            return False, f'la misión ya está corriendo ({self.state.name})'
        if not self.position_valid or self.position is None:
            return False, 'todavía no hay posición local válida de PX4'
        if (self.p('avoid.enabled') and self.p('avoid.require_sensors')
                and not self._sectors_fresh(now)):
            return False, ('no llegan datos de obstáculos (/obstacles/sectors): ¿está '
                           'corriendo mapper_node y publicando el sensor?')
        return True, ''

    def start_waypoints(self, now):
        ok, why = self._can_start(now)
        if not ok:
            return False, why
        self._waypoints = self._load_waypoints()
        if not self._waypoints:
            return False, 'no hay waypoints cargados'
        self._begin(MissionType.WAYPOINTS, now)
        return True, f'misión de waypoints iniciada ({len(self._waypoints)} puntos)'

    def start_mapping(self, now):
        ok, why = self._can_start(now)
        if not ok:
            return False, why
        area = [float(v) for v in (self.p('map_mission.area') or [])]
        if len(area) != 4 or area[0] == area[2] or area[1] == area[3]:
            return False, 'map_mission.area tiene que ser [norte_min, este_min, norte_max, este_max]'
        if self.p('map_mission.reset_map'):
            self._commands.append('map_reset')
        self._begin(MissionType.MAPPING, now)
        n0, e0, n1, e1 = area
        return True, (f'misión de mapeo iniciada: área de {abs(n1 - n0):.0f} x {abs(e1 - e0):.0f} m '
                      f'a {self.p("map_mission.altitude"):.1f} m, estrategia '
                      f'{self.p("map_mission.strategy")}')

    def pause(self, now):
        if self.state not in (State.NAVIGATE, State.AVOID, State.TAKEOFF, State.RETURN, State.SCAN):
            return False, f'no se puede pausar en {self.state.name}'
        self._paused_from = self.state
        self._enter(State.HOLD, 'pausada: manteniendo posición')
        return True, 'misión pausada'

    def resume(self, now):
        if self.state != State.HOLD or self._paused_from is None:
            return False, f'no está pausada (estado {self.state.name})'
        target = self._paused_from
        self._paused_from = None
        self._ramp_start = now
        self._path = None
        self._reset_progress(now)
        self._enter(target, 'misión reanudada')
        return True, 'misión reanudada'

    def cancel(self, now):
        if self.state not in FLYING_STATES or self.state == State.ARMING:
            return False, f'nada que cancelar (estado {self.state.name})'
        self._go_home(now, 'misión cancelada: volviendo al despegue')
        return True, 'cancelada, volviendo al punto de despegue'

    def abort(self, now):
        if not self.armed:
            return False, 'el dron no está armado'
        self._request_land('misión abortada: aterrizaje inmediato', State.ABORTED)
        return True, 'abortada, aterrizando en el lugar'

    # ------------------------------------------------------------------
    # Transiciones
    # ------------------------------------------------------------------
    def _enter(self, state, message):
        if state != self.state:
            self.log('info', f'{self.state.name} -> {state.name}: {message}')
            if state == State.HOLD and self.position is not None:
                self._hold_anchor = (self.position[0], self.position[1])
        self.state = state
        self.message = message

    def _begin(self, mission, now):
        self.mission = mission
        self._wp_index = 0
        self._goal = None
        self._path = None
        self._blacklist = []
        self._goal_failures = 0
        self.goals_reached = 0
        self._land_requested = False
        self._paused_from = None
        self.avoider.reset()
        airborne = self.armed and self.offboard and self.state == State.HOLD
        if not airborne:
            self.home = self.position
            self._distance_traveled = 0.0
        self._map_start = now
        if mission == MissionType.MAPPING:
            n0, e0, n1, e1 = [float(v) for v in self.p('map_mission.area')]
            hn, he = self.home[0], self.home[1]
            self._map_area = (hn + n0, he + e0, hn + n1, he + e1)
            self._lanes = lawnmower_waypoints(self._map_area, self.p('map_mission.lane_spacing'),
                                              start_ne=self.position[:2])
        self._carrot = list(self.position)
        self._yaw_sp = self.heading
        self._takeoff_xy = (self.position[0], self.position[1])
        if airborne:
            self._compute_fence()
            self._enter(State.TAKEOFF, f'ya en el aire: ajustando altura a {self._mission_altitude():.1f} m')
        else:
            self._arm_ticks = 0
            self._arm_attempts = 0
            self._last_arm_request = -math.inf
            self._enter(State.ARMING, 'armando y entrando en OFFBOARD')

    def _mission_altitude(self):
        if self.mission == MissionType.MAPPING:
            return float(self.p('map_mission.altitude'))
        return float(self.p('takeoff_altitude'))

    def _altitude_down(self, altitude):
        return self.home[2] - altitude

    def _set_goal(self, north, east, altitude, kind, now):
        low, high = float(self.p('safety.min_altitude')), float(self.p('safety.max_altitude'))
        if not low <= altitude <= high:
            self.log('warn', f'{kind}: altura {altitude:.1f} m fuera de [{low:.1f}, {high:.1f}] m '
                             '(safety.min_altitude / max_altitude): se recorta')
            altitude = min(max(altitude, low), high)
        self._goal = (float(north), float(east), self._altitude_down(altitude))
        self._goal_kind = kind
        self._goal_id += 1
        self._goal_started = now
        self._path = None
        self._goal_reachable = None
        self._plan_failures = 0
        self._blocked_since = None
        self._reset_progress(now)
        self._ramp_start = now
        state = State.RETURN if kind == 'home' else State.NAVIGATE
        self._enter(state, f'rumbo a {kind} ({north:.1f}, {east:.1f})')

    def _reset_progress(self, now):
        self._progress_best = math.inf
        self._progress_time = now

    def _go_home(self, now, message):
        self.log('info', message)
        altitude = self.home[2] - self._carrot[2] if self._carrot else self._mission_altitude()
        self._set_goal(self.home[0], self.home[1], max(altitude, 1.0), 'home', now)
        self.message = message

    def _request_land(self, message, land_state=State.LAND):
        self._land_requested = True
        self._enter(land_state, message)
        self._commands.append('land')

    # ------------------------------------------------------------------
    # Lazo principal
    # ------------------------------------------------------------------
    def step(self, now):
        if self._last_step is None:
            self._dt = 1.0 / float(self.p('control_rate'))
        else:
            self._dt = min(max(now - self._last_step, 0.0), 0.25)
        self._last_step = now
        self._now = now
        self.velocity_setpoint = None

        if self.state in FLYING_STATES:
            if self.position is None:
                self.stream_offboard = False
                return
            self.stream_offboard = True
            self._step_flight(now)
            self.setpoint = (self._carrot[0], self._carrot[1], self._carrot[2], wrap_pi(self._yaw_sp))
        elif self.state in (State.LAND, State.ABORTED):
            self._step_landing()
        else:
            self.stream_offboard = False

    def _step_landing(self):
        # PX4 puede tardar unos ciclos en aceptar AUTO_LAND; mientras siga en
        # OFFBOARD se insiste. Una vez fuera, dejar de publicar setpoints.
        if self.offboard:
            self.stream_offboard = True
            if self._carrot is not None:
                self.setpoint = (self._carrot[0], self._carrot[1], self._carrot[2], wrap_pi(self._yaw_sp))
            self._commands.append('land')
            return
        self.stream_offboard = False
        if not self.armed and self.state == State.LAND:
            self._enter(State.DONE, 'misión completada, dron desarmado')

    def _step_flight(self, now):
        if self.state == State.ARMING:
            self._step_arming(now)
            return
        if self._pose_failsafe(now) or self._sensor_failsafe(now):
            return
        if self.state == State.HOLD:
            self._hold()
            return
        if self.state == State.TAKEOFF:
            self._step_takeoff(now)
            return
        if self.state == State.SCAN:
            self._step_scan(now)
            return
        self._step_navigation(now)

    # ------------------------------------------------------------------
    # Failsafes
    # ------------------------------------------------------------------
    def _pose_failsafe(self, now):
        """Sin posición válida de PX4 (tópico cortado o xy_valid = False en
        vuelo): no se puede navegar ni esquivar con una pose congelada. Se deja
        fija la referencia (PX4 frena) y, si dura, se aterriza en el lugar.
        Devuelve True si tomó el control de este ciclo."""
        age = 0.0 if self._pose_time is None else now - self._pose_time
        lost = (not self.position_valid) or age > float(self.p('nav.pose_timeout'))
        if not lost:
            if self._pose_lost_since is not None:
                self.log('info', f'posición de PX4 recuperada tras {now - self._pose_lost_since:.1f} s')
                self._pose_lost_since = None
                self._ramp_start = now
            return False
        if self._pose_lost_since is None:
            self._pose_lost_since = now
            self.log('warn', 'sin posición válida de PX4 (vehicle_local_position cortada o '
                             'xy_valid = false): frenando con la referencia fija')
        self.velocity_setpoint = None
        land_after = float(self.p('nav.pose_loss_land_after'))
        if land_after > 0.0 and now - self._pose_lost_since > land_after:
            self._request_land(f'sin posición válida hace {now - self._pose_lost_since:.1f} s: '
                               'aterrizando en el lugar')
            return True
        self.message = f'sin posición válida hace {now - self._pose_lost_since:.1f} s: frenando'
        return True

    def _sensor_failsafe(self, now):
        """Sin datos de obstáculos en vuelo por mucho tiempo: aterrizar en el
        lugar (como PX4-Avoidance, que tras perder la nube pasa a hover y
        después a terminación; la Collision Prevention de PX4 pasa a Hold a los
        5 s). El freno inmediato lo hace la navegación (hold_on_sensor_loss)."""
        if (not self.p('avoid.enabled') or not self._sectors_ever
                or self._sectors_fresh(now)):
            self._sensor_lost_since = None
            return False
        if self._sensor_lost_since is None:
            self._sensor_lost_since = now
        land_after = float(self.p('avoid.sensor_loss_land_after'))
        if land_after > 0.0 and now - self._sensor_lost_since > land_after:
            self._request_land(f'sin datos de obstáculos hace {now - self._sensor_lost_since:.0f} s: '
                               'aterrizando en el lugar')
            return True
        return False

    def _compute_fence(self):
        """Geocerca blanda (en este nodo; la de PX4 es aparte, GF_*): la
        referencia nunca sale de ella. Explícita en `safety.geofence` (relativa
        al despegue) o, en el mapeo, el área + `safety.geofence_margin`."""
        fence = [float(v) for v in (self.p('safety.geofence') or [])]
        margin = float(self.p('safety.geofence_margin'))
        if len(fence) == 4:
            n0, e0, n1, e1 = fence
            hn, he = self.home[0], self.home[1]
            self._fence = (hn + min(n0, n1), he + min(e0, e1), hn + max(n0, n1), he + max(e0, e1))
        elif self.mission == MissionType.MAPPING and self._map_area is not None:
            n0, e0, n1, e1 = self._map_area
            self._fence = (min(n0, n1) - margin, min(e0, e1) - margin,
                           max(n0, n1) + margin, max(e0, e1) + margin)
        else:
            self._fence = None
        self._fence_warned = False

    def _apply_fence(self):
        if self._fence is None:
            return
        n0, e0, n1, e1 = self._fence
        n = min(max(self._carrot[0], n0), n1)
        e = min(max(self._carrot[1], e0), e1)
        if (n, e) != (self._carrot[0], self._carrot[1]):
            if not self._fence_warned:
                self._fence_warned = True
                self.log('warn', f'geocerca: la referencia ({self._carrot[0]:.1f}, '
                                 f'{self._carrot[1]:.1f}) salía del área permitida; se recorta')
            self._carrot[0], self._carrot[1] = n, e
        elif self._fence_warned and n0 + 0.5 < n < n1 - 0.5 and e0 + 0.5 < e < e1 - 0.5:
            self._fence_warned = False

    def _clamp_altitude_down(self, target_d):
        """Altura (D de NED) dentro de [min_altitude, max_altitude] sobre el despegue."""
        if self.home is None:
            return target_d
        low = self.home[2] - float(self.p('safety.min_altitude'))
        high = self.home[2] - float(self.p('safety.max_altitude'))
        return min(max(target_d, high), low)

    def _tracking_error(self):
        """Cuánto se pasó el dron por DELANTE de la referencia (en la dirección
        de avance: viene más rápido de lo pedido, p. ej. al frenar o con viento
        de cola) y la diferencia de altura. El desvío lateral no cuenta: frenar
        no lo corrige (lo corrige el control de posición) y con viento
        constante frenar cada tanto lo dejaba derivar contra la pared (con
        0.3 m + climb_speed de tolerancia: subiendo, PX4 va ~1 s atrás)."""
        pos, carrot = self.position, self._carrot
        dn, de = pos[0] - carrot[0], pos[1] - carrot[1]
        if self._avoid_dir_prev is None:
            horizontal = 0.0
        else:
            un, ue = math.cos(self._avoid_dir_prev), math.sin(self._avoid_dir_prev)
            horizontal = max(0.0, dn * un + de * ue)
        vertical = max(0.0, abs(pos[2] - carrot[2]) - 0.3 - float(self.p('climb_speed')))
        return math.hypot(horizontal, vertical)

    def _tracking_failsafe(self, now):
        """Error de seguimiento excesivo (ráfaga, controlador saturado): frenar
        un rato en vez de seguir arrastrando la referencia (KR aborta con
        0.75 m). Devuelve True mientras hay que frenar; la navegación igual
        corre la evasión antes, para que una huida de emergencia pueda actuar."""
        limit = float(self.p('nav.max_tracking_error'))
        if now < self._tracking_brake_until:
            return True
        if limit <= 0.0:
            return False
        error = self._tracking_error()
        if error > limit:
            self.log('warn', f'error de seguimiento {error:.2f} m > {limit:.2f} m: frenando '
                             f'{self.p("nav.tracking_brake_sec"):.1f} s')
            self._tracking_brake_until = now + float(self.p('nav.tracking_brake_sec'))
            self._ramp_start = self._tracking_brake_until
            return True
        return False

    def _step_arming(self, now):
        # PX4 rechaza OFFBOARD si no viene recibiendo setpoints: primero se
        # manda el flujo un segundo y recién ahí se pide el modo y el armado.
        self._carrot = list(self.position)
        self._arm_ticks += 1
        if self._arm_ticks * self._dt < 1.0:
            return
        if self.offboard and self.armed:
            self.home = self.position
            self._carrot = list(self.position)
            self._takeoff_xy = (self.position[0], self.position[1])
            if self.mission == MissionType.MAPPING:
                n0, e0, n1, e1 = [float(v) for v in self.p('map_mission.area')]
                self._map_area = (self.home[0] + n0, self.home[1] + e0,
                                  self.home[0] + n1, self.home[1] + e1)
                self._map_start = now
            self._compute_fence()
            self._enter(State.TAKEOFF, f'despegando a {self._mission_altitude():.1f} m')
            return
        if now - self._last_arm_request >= 1.0:
            self._last_arm_request = now
            self._arm_attempts += 1
            if not self.offboard:
                self._commands.append('mode_offboard')
            if not self.armed:
                self._commands.append('arm')
        if self._arm_attempts > self.p('arm_timeout'):
            self._enter(State.FAILED, 'no se pudo armar ni entrar en OFFBOARD: revisar los '
                                      'preflight checks en QGroundControl')

    def _step_takeoff(self, now):
        target_d = self._altitude_down(self._mission_altitude())
        self._move_vertical(target_d)
        self._carrot[0], self._carrot[1] = self._takeoff_xy
        self._clamp_lead()
        if abs(self.position[2] - target_d) <= 0.3:
            self._ramp_start = now
            self._mission_flow_start(now)

    def _mission_flow_start(self, now):
        if self.mission == MissionType.MAPPING and self.p('map_mission.spin_at_start'):
            self._start_scan(now)
            return
        self._next_goal(now)

    def _start_scan(self, now):
        self._scan_swept = 0.0
        self._hold_anchor = (self.position[0], self.position[1])
        self._enter(State.SCAN, 'girando 360° para ver todo alrededor')

    def _step_scan(self, now):
        self._hold()
        self._move_vertical(self._altitude_down(self._mission_altitude()))
        step = float(self.p('scan.yaw_rate')) * self._dt
        self._yaw_sp = wrap_pi(self._yaw_sp + step)
        self._scan_swept += step
        if self._scan_swept >= 2.0 * math.pi:
            self._next_goal(now)
        else:
            self.message = f'barriendo {math.degrees(self._scan_swept):.0f}° de 360°'

    def _hold(self):
        if self._hold_anchor is None:
            self._hold_anchor = (self.position[0], self.position[1])
        self._carrot[0], self._carrot[1] = self._hold_anchor
        self._clamp_lead()

    # ------------------------------------------------------------------
    # Elección del próximo objetivo
    # ------------------------------------------------------------------
    def _next_goal(self, now):
        if self.mission == MissionType.WAYPOINTS:
            if self._wp_index >= len(self._waypoints):
                if self.p('land_at_end'):
                    self._go_home(now, 'waypoints completados: volviendo')
                else:
                    self._paused_from = None
                    self._enter(State.HOLD, 'waypoints completados: manteniendo posición')
                return
            n, e, alt = self._waypoints[self._wp_index]
            self._set_goal(self.home[0] + n, self.home[1] + e, alt,
                           f'waypoint {self._wp_index}', now)
            return

        if self.mission == MissionType.MAPPING:
            if now - self._map_start > self.p('map_mission.max_duration'):
                self._finish_mapping(now, 'tiempo máximo de mapeo alcanzado')
                return
            if self._goal_failures >= self.p('map_mission.max_goal_failures'):
                self._finish_mapping(now, 'demasiados objetivos inalcanzables')
                return
            if self.p('map_mission.strategy') == 'lawnmower':
                if not self._lanes:
                    self._finish_mapping(now, 'barrido completo')
                    return
                n, e = self._lanes.pop(0)
                self._set_goal(n, e, self._mission_altitude(), 'carril de barrido', now)
                return
            frontier = self._pick_frontier()
            if frontier is None:
                why = ('sin fronteras abiertas: el área quedó mapeada' if self.snapshot is not None
                       else 'no llega el mapa (/map): ¿está corriendo mapper_node?')
                self._finish_mapping(now, why)
                return
            self._set_goal(frontier.target[0], frontier.target[1], self._mission_altitude(),
                           'frontera', now)
            return

        self._enter(State.HOLD, 'sin misión: manteniendo posición')

    def _pick_frontier(self):
        if self.snapshot is None:
            return None
        min_cells = max(2, int(self.p('map_mission.min_frontier_length') / self.snapshot.resolution))
        frontiers = find_frontiers(self.snapshot, self._map_area, min_size=min_cells,
                                   clearance=float(self.p('plan.inflation_radius')))
        self.frontiers_open = len(frontiers)
        self.coverage = coverage(self.snapshot, self._map_area)
        return select_frontier(frontiers, self.position[:2], self.heading, self._blacklist)

    def _frontier_still_open(self):
        """¿Queda algo desconocido cerca del objetivo? Con un lidar de 40 m la
        frontera muchas veces se "descubre" de lejos y no hace falta llegar."""
        snap = self.snapshot
        if snap is None or self._goal is None:
            return True
        radius = 1.0
        r0, c0 = snap.world_to_cell(self._goal[0] - radius, self._goal[1] - radius)
        r1, c1 = snap.world_to_cell(self._goal[0] + radius, self._goal[1] + radius)
        rows, cols = snap.values.shape
        if r1 < 0 or c1 < 0 or r0 >= rows or c0 >= cols:
            return True
        r0, c0 = max(0, r0), max(0, c0)
        r1, c1 = min(rows - 1, r1), min(cols - 1, c1)
        window = snap.values[r0:r1 + 1, c0:c1 + 1]
        expected = (2 * int(radius / snap.resolution) + 1) ** 2
        if window.size < expected * 0.8:
            return True   # el objetivo está en el borde de la grilla: sigue abierto
        return bool(np.any(window < 0))

    def _finish_mapping(self, now, reason):
        self.log('info', f'mapeo terminado: {reason}')
        if self.p('map_mission.save_on_finish'):
            self._commands.append('map_save')
        if self.p('map_mission.return_home'):
            self._go_home(now, f'mapeo terminado ({reason}): volviendo')
        else:
            self._paused_from = None
            self._enter(State.HOLD, f'mapeo terminado ({reason}): manteniendo posición')

    def _on_goal_reached(self, now):
        self.goals_reached += 1
        if self.state == State.RETURN:
            self._request_land('sobre el punto de despegue: aterrizando')
            return
        if self.mission == MissionType.WAYPOINTS:
            self._wp_index += 1
            self._next_goal(now)
            return
        if self.p('map_mission.spin_at_goals'):
            self._start_scan(now)
        else:
            self._next_goal(now)

    def _on_goal_failed(self, now, reason):
        self.log('warn', f'objetivo {self._goal_kind} abandonado: {reason}')
        if self.state == State.RETURN:
            if self.p('avoid.on_return_failure') == 'land':
                # Antes quedaba en HOLD hasta que PX4 cortara por batería.
                self._request_land(f'no se pudo volver al despegue ({reason}): aterrizando '
                                   'en el lugar')
                return
            self._paused_from = None
            self._enter(State.HOLD, f'no se pudo volver al despegue ({reason}): en espera')
            return
        if self.mission == MissionType.MAPPING:
            self._goal_failures += 1
            self._blacklist.append((self._goal[0], self._goal[1]))
            self._next_goal(now)
            return
        policy = self.p('avoid.on_failure')
        if policy == 'skip':
            self._wp_index += 1
            self._next_goal(now)
        elif policy == 'return':
            self._go_home(now, f'waypoint inalcanzable ({reason}): volviendo')
        else:
            self._paused_from = None
            self._enter(State.HOLD, f'waypoint inalcanzable ({reason}): en espera')

    # ------------------------------------------------------------------
    # Navegación
    # ------------------------------------------------------------------
    def planning_request(self, now):
        """Datos para que el planificador corra afuera (otro hilo / el simulador)."""
        if (not self.p('nav.use_planner') or self.state not in MOVING_STATES
                or self._goal is None or self.snapshot is None or self.position is None
                or self._plan_pending):
            return None
        if self._path is not None and self._path_goal_id == self._goal_id and \
                now - self._last_plan_time < self.p('nav.replan_period'):
            return None
        # Tras un fallo también se espera replan_period: antes se reintentaba en
        # cada ciclo y plan.max_failures se agotaba en 0.15 s, con un mapa que
        # todavía no había cambiado (QA, puertas).
        if self._path is None and self._plan_failures > 0 and \
                now - self._last_plan_time < self.p('nav.replan_period'):
            return None
        self._plan_pending = True
        return {'id': self._goal_id, 'snapshot': self.snapshot,
                'start': (self.position[0], self.position[1]),
                'goal': (self._goal[0], self._goal[1])}

    def planning_result(self, request_id, result, now):
        self._plan_pending = False
        self._last_plan_time = now
        if request_id != self._goal_id:
            return
        if result.ok:
            self._path = result.path
            self._path_goal_id = request_id
            self._plan_failures = 0
            self._plan_reason = result.reason
            # El objetivo caía dentro de la zona inflada de un obstáculo: A* lo
            # corrió a la celda libre más cercana. Llegar ahí cuenta como llegar.
            self._goal_reachable = (tuple(result.path[-1]) if result.goal_adjusted else None)
        else:
            self._plan_failures += 1
            self._plan_reason = result.reason
            self._path = None
            self.log('warn', f'A* sin camino a {self._goal_kind}: {result.reason} '
                             f'({self._plan_failures}/{self.p("plan.max_failures")})')
            if self._plan_failures >= self.p('plan.max_failures') and self.state in MOVING_STATES:
                self._on_goal_failed(now, f'planificador: {result.reason}')

    def _step_navigation(self, now):
        goal = self._goal
        pos = self.position
        self._move_vertical(goal[2])
        dist = distance_2d(pos, goal)

        reach = self._goal_reachable
        if abs(pos[2] - goal[2]) <= 0.6 and (
                dist <= self.p('acceptance_radius')
                or (reach is not None and distance_2d(pos, reach) <= self.p('acceptance_radius'))):
            self._on_goal_reached(now)
            return

        # Progreso: un objetivo al que no nos acercamos en un buen rato no es alcanzable.
        if dist < self._progress_best - self.p('nav.stuck_progress'):
            self._progress_best = dist
            self._progress_time = now
        elif now - self._progress_time > self.p('nav.stuck_timeout'):
            self._on_goal_failed(now, f'sin progreso en {self.p("nav.stuck_timeout"):.0f} s')
            return
        if self.mission == MissionType.MAPPING and self.state != State.RETURN:
            if now - self._goal_started > self.p('map_mission.goal_timeout'):
                self._on_goal_failed(now, 'tiempo por objetivo agotado')
                return
            if now - self._last_frontier_check >= 1.0:
                self._last_frontier_check = now
                if self._map_area is not None and self.snapshot is not None:
                    self.coverage = coverage(self.snapshot, self._map_area)
                if (self.p('map_mission.strategy') == 'frontier'
                        and not self._frontier_still_open()):
                    self.goals_reached += 1
                    self._next_goal(now)
                    return

        tracking_brake = self._tracking_failsafe(now)

        target = (goal[0], goal[1])
        if self._path is not None and self._path_goal_id == self._goal_id:
            look = path_lookahead(self._path, pos[:2], self.p('nav.path_lookahead'))
            if look is not None:
                target = look
        desired = bearing_between(pos, target)
        direction, scale = desired, 1.0
        self._avoid_active = False
        self._avoid_reason = ''
        cruise = self._ramped_speed(now)
        cap = math.inf
        self._fov_scale = 1.0

        if self.p('avoid.enabled'):
            if self._sectors_fresh(now):
                self._sensor_loss_warned = False
                age = self._sectors_age(now)
                # Para decidir hasta dónde mirar: la velocidad real o la pedida,
                # la mayor (con viento de cola o zanahoria adelantada, la real
                # supera a la pedida).
                speed_ref = max(math.hypot(self.velocity[0], self.velocity[1]), cruise)
                seen = self._seen_mask(now)
                cmd = self.avoider.update(self._sectors, desired, dist, self.heading,
                                          speed=speed_ref, data_age=age,
                                          sensor_range=self._sectors_range,
                                          live_ranges=self._sectors_live, seen=seen)
                self._nearest, self._nearest_bearing = cmd.nearest, cmd.nearest_bearing
                direction, scale, hold = self._apply_avoidance(cmd, desired, now)
                if hold:
                    self._freeze()
                    if cmd.unseen:
                        # Con un sensor de campo limitado lo libre puede estar
                        # fuera de la vista: girar en el lugar hacia el objetivo
                        # (y más allá) hasta verlo, en vez de esperar quieto.
                        self._update_yaw(wrap_pi(self._yaw_sp + math.pi / 2.0), goal, 0.0,
                                         force=True)
                    return
                if cmd.emergency:
                    # Alejarse a velocidad fija (no una fracción del crucero: con
                    # una ráfaga empujando hacia la pared, 0.25·crucero no alcanza).
                    cap = cmd.max_speed
                    scale = cmd.max_speed / max(cruise, 1e-3)
                else:
                    # Tope físico en la dirección que de verdad se va a seguir
                    # (durante el debounce puede no ser la que propuso el VFH).
                    self._ahead, cap = self.avoider.limit_for(
                        self._sectors, direction, age, self._sectors_range)
                    self._fov_scale = self._fov_factor(seen, direction)
                    if self._fov_scale < 1.0:
                        cap = min(cap, cruise * self._fov_scale)
                        if self._fov_scale <= 0.0:
                            self._avoid_active = True
                            self._avoid_reason = 'girando para mirar hacia donde va antes de avanzar'
            elif self._sectors_ever and self.p('avoid.hold_on_sensor_loss'):
                if not self._sensor_loss_warned:
                    self.log('warn', 'se cortaron los datos de obstáculos: frenando en el lugar')
                    self._sensor_loss_warned = True
                self._avoid_active = True
                self._avoid_reason = 'sin datos de obstáculos'
                self.message = self._avoid_reason
                self._freeze()
                return

        if tracking_brake and not (self._emergency_since is not None and self._avoid_active):
            # Frenar (la huida de emergencia, si hay, tiene prioridad).
            self._freeze()
            self._avoid_active = True
            self._avoid_reason = 'error de seguimiento: frenando'
            self.message = self._avoid_reason
            return
        speed = min(cruise * scale, cap)
        self._speed_cap = cap
        if (self._avoid_dir_prev is not None
                and angle_diff(direction, self._avoid_dir_prev) > math.radians(25)):
            # Cambio brusco de dirección: la zanahoria vieja quedó apuntando
            # hacia donde ya no hay que ir. Se re-ancla en el dron.
            self._carrot[0], self._carrot[1] = pos[0], pos[1]
        self._avoid_dir_prev = direction
        stop_at = target if (direction == desired and target == (goal[0], goal[1])) else None
        self._advance_carrot(direction, speed, stop_at)
        if self.p('nav.velocity_feedforward'):
            self.velocity_setpoint = (math.cos(direction) * speed, math.sin(direction) * speed)
        self._update_yaw(direction, goal, speed, force=self._fov_scale < 1.0)

        if self._avoid_active and self.state == State.NAVIGATE:
            self._enter(State.AVOID, self._avoid_reason)
        elif not self._avoid_active and self.state == State.AVOID:
            self._enter(State.NAVIGATE, f'rumbo a {self._goal_kind}')
        elif self._avoid_active:
            self.message = self._avoid_reason

    def _apply_avoidance(self, cmd, desired, now):
        """Debounce de las decisiones de la evasión. Devuelve (dirección, escala, frenar).

        Como en el repo de referencia, ninguna decisión se toma con una sola
        lectura: el simulador (y un lidar real con polvo) puede dar un dato
        espurio que se despeja solo en décimas de segundo.
        """
        if cmd.emergency:
            if self._emergency_since is None:
                self._emergency_since = now
            if now - self._emergency_since >= self.p('avoid.emergency_hold'):
                self._avoid_active = True
                self._avoid_reason = cmd.reason
                if cmd.direction is None:
                    # Ninguna salida vista: frenar en el lugar (no retroceder a ciegas).
                    return desired, 0.0, True
                return cmd.direction, cmd.speed_scale, False
            return desired, 0.0, True
        self._emergency_since = None

        if cmd.blocked:
            if self._blocked_since is None:
                self._blocked_since = now
                self._path = None    # forzar replanificación
                self._last_plan_time = -math.inf
            self._avoid_active = True
            self._avoid_reason = cmd.reason
            if now - self._blocked_since > self.p('avoid.blocked_timeout'):
                self._blocked_since = None
                self._on_goal_failed(now, 'rodeado de obstáculos')
            return desired, 0.0, True
        self._blocked_since = None

        if cmd.deviated:
            if self._deviate_since is None:
                self._deviate_since = now
            if now - self._deviate_since >= self.p('avoid.trigger_hold'):
                self._avoid_active = True
                self._avoid_reason = cmd.reason
                return cmd.direction, cmd.speed_scale, False
            # Dentro de la ventana de confirmación: no se avanza a ciegas hacia
            # el obstáculo, pero tampoco se desvía todavía.
            return desired, min(cmd.speed_scale, 0.3), False
        self._deviate_since = None
        return cmd.direction, cmd.speed_scale, False

    def _ramped_speed(self, now):
        """`cruise_speed`, en rampa si recién se arranca desde quieto.

        Un escalón de velocidad obliga a PX4 a cabecear fuerte, y eso apunta los
        sensores fijos al cuerpo hacia el piso: el dron "ve" una pared que no
        existe (lección del repo de referencia).
        """
        cruise = float(self.p('cruise_speed'))
        ramp = float(self.p('nav.accel_ramp_sec'))
        if self._ramp_start is None or ramp <= 0.0:
            return cruise
        elapsed = now - self._ramp_start
        if elapsed >= ramp:
            self._ramp_start = None
            return cruise
        low = min(0.3, cruise)
        return low + (cruise - low) * elapsed / ramp

    def _update_yaw(self, direction, goal, speed, force=False):
        """`force`: mirar hacia el avance aunque esté quieto (el sensor de campo
        limitado no ve hacia donde hay que ir: primero girar)."""
        mode = self.p('nav.yaw_mode')
        if mode == 'fixed' and not force:
            return
        if mode == 'goal' and not force:
            desired = bearing_between(self.position, goal)
        else:
            if speed < 0.2 and not force:
                return
            desired = direction
        max_step = float(self.p('nav.max_yaw_rate')) * self._dt
        delta = wrap_pi(desired - self._yaw_sp)
        self._yaw_sp = wrap_pi(self._yaw_sp + max(-max_step, min(max_step, delta)))

    # ------------------------------------------------------------------
    # Zanahoria
    # ------------------------------------------------------------------
    def _advance_carrot(self, direction, speed, stop_at=None):
        self._freeze_anchor = None
        step = speed * self._dt
        if stop_at is not None:
            remaining = distance_2d(self._carrot, stop_at)
            if remaining <= step:
                self._carrot[0], self._carrot[1] = stop_at
                self._limit_along_lead(direction, speed)
                self._clamp_lead()
                return
        self._carrot[0] += math.cos(direction) * step
        self._carrot[1] += math.sin(direction) * step
        self._limit_along_lead(direction, speed)
        self._clamp_lead()
        self._apply_fence()

    def _limit_along_lead(self, direction, speed):
        """La zanahoria no se adelanta más que `v·lead_time` EN LA DIRECCIÓN DE
        AVANCE: el control de posición de PX4 suma ~MPC_XY_P·(distancia a la
        zanahoria) a la velocidad, así que una zanahoria lejos con la velocidad
        ya recortada por el frenado seguiría empujando hacia el obstáculo. La
        componente lateral no se recorta (hasta `max_lead`): es la que le da al
        control autoridad para corregir una deriva por viento."""
        lead = speed * float(self.p('nav.lead_time'))
        un, ue = math.cos(direction), math.sin(direction)
        along = (self._carrot[0] - self.position[0]) * un + (self._carrot[1] - self.position[1]) * ue
        if along > lead:
            self._carrot[0] -= (along - lead) * un
            self._carrot[1] -= (along - lead) * ue

    def _freeze(self):
        """Frenar y sostener la posición donde se empezó a frenar (no la que el
        dron tenga en cada ciclo: con viento, seguir al dron dejaría el control
        sin error de posición y la deriva lo llevaría contra la pared)."""
        if self._freeze_anchor is None:
            self._freeze_anchor = (self.position[0], self.position[1])
        self._carrot[0], self._carrot[1] = self._freeze_anchor
        self._clamp_lead()
        self._avoid_dir_prev = None

    def _move_vertical(self, target_d):
        target_d = self._clamp_altitude_down(target_d)
        delta = target_d - self._carrot[2]
        step = float(self.p('climb_speed')) * self._dt
        self._carrot[2] += math.copysign(min(abs(delta), step), delta)

    def _clamp_lead(self, max_lead=None):
        if max_lead is None:
            max_lead = float(self.p('max_lead'))
        dn = self._carrot[0] - self.position[0]
        de = self._carrot[1] - self.position[1]
        lead = math.hypot(dn, de)
        if lead > max_lead:
            scale = max_lead / lead
            self._carrot[0] = self.position[0] + dn * scale
            self._carrot[1] = self.position[1] + de * scale

    # ------------------------------------------------------------------
    # Estado para publicar
    # ------------------------------------------------------------------
    def status(self):
        pos = self.position
        out = {
            'state': int(self.state),
            'state_name': self.state.name,
            'mission_type': int(self.mission),
            'message': self.message,
            'waypoint_index': self._wp_index,
            'waypoint_count': len(self._waypoints),
            'goal_kind': self._goal_kind,
            'distance_to_goal': float('nan'),
            'distance_to_home': float('nan'),
            'distance_traveled': self._distance_traveled,
            'tracking_error': float('nan'),
            'altitude': float('nan'),
            'ground_speed': math.hypot(self.velocity[0], self.velocity[1]),
            'avoiding': self._avoid_active,
            'avoid_reason': self._avoid_reason,
            'obstacle_distance': self._nearest if math.isfinite(self._nearest) else float('nan'),
            'obstacle_bearing': self._nearest_bearing,
            'path_points': len(self._path) if self._path else 0,
            'plan_status': self._plan_reason,
            'coverage': self.coverage,
            'frontiers': self.frontiers_open,
            'goals_reached': self.goals_reached,
            'walls_flat': self.walls_summary[0],
            'walls_irregular': self.walls_summary[1],
            'obstacles': self.walls_summary[2],
        }
        if pos is not None and self._goal is not None:
            out['distance_to_goal'] = distance_2d(pos, self._goal)
        if pos is not None and self.home is not None:
            out['distance_to_home'] = math.dist(pos, self.home)
            out['altitude'] = self.home[2] - pos[2]
        if pos is not None and self._carrot is not None:
            out['tracking_error'] = math.dist(pos, self._carrot)
        return out
