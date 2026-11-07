#!/usr/bin/env python3
"""Kit del harness de QA: perturbaciones de entorno real sobre `closed_loop.run`.

No edita el repo. Importa `uav_sim.closed_loop` desde la copia indicada con
`--repo` y reemplaza, sólo en el namespace de ese módulo y sólo dentro del
proceso, las clases que `run()` instancia:

- `World`       -> HarnessWorld: agrega obstáculos dinámicos (cajas que se mueven).
- `SimVehicle`  -> HarnessVehicle: inercia/velocidad distinta, viento, reloj y
                   registro de la trayectoria y del despeje en cada paso.
- `WallMapper`  -> HarnessMapper: corte del sensor (el driver deja de publicar)
                   y rumbo del sensor para cámaras de FOV acotado.
- `MapperConfig`-> fábrica con los parámetros del mapper que pida el caso.
- `MissionCore` -> HarnessCore: sólo recibe sectores si el mapper tiene una
                   fuente fresca, como hace `mapper_node._publish_sectors`.

El lidar se pasa por el argumento `lidar=` de `run()` (HarnessLidar, subclase de
`LidarModel`): ruido, puntos perdidos, polvo, latencia, sensor ciego y fase
aleatoria de los haces.

Así `run()` se ejecuta tal cual está en el repo: si la otra copia lo mejora,
el harness prueba la versión mejorada.
"""

import inspect
import math
import os
import sys
import time
from collections import deque

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
SCEN_DIR = os.path.join(HERE, 'escenarios')
# Por defecto, el repo que contiene este arnés (qa/harness -> raíz).
DEFAULT_REPO = os.path.dirname(os.path.dirname(HERE))

MOVING = {'NAVIGATE', 'AVOID', 'RETURN'}
FLIGHT = {'TAKEOFF', 'NAVIGATE', 'AVOID', 'HOLD', 'RETURN', 'SCAN'}
END = {'DONE', 'FAILED', 'PREEMPTED', 'ABORTED'}


# ----------------------------------------------------------------------
# Repo
# ----------------------------------------------------------------------
def setup_repo(repo):
    """Pone los paquetes del repo al frente de sys.path e importa closed_loop."""
    repo = os.path.abspath(repo or DEFAULT_REPO)
    sys.dont_write_bytecode = True     # no dejar __pycache__ dentro del repo probado
    src = os.path.join(repo, 'src')
    if not os.path.isdir(src):
        raise SystemExit(f'--repo {repo}: no existe {src}')
    for pkg in ('uav_sim', 'uav_mission', 'uav_wall_mapping'):
        path = os.path.join(src, pkg)
        while path in sys.path:
            sys.path.remove(path)
        sys.path.insert(0, path)
    import uav_sim.closed_loop as cl  # noqa: E402
    where = os.path.abspath(cl.__file__)
    if not where.startswith(src):
        raise SystemExit(f'closed_loop importado desde {where}, no desde {src}')
    return cl


def resolve_scenario(name):
    """Nombre de escenario del repo, ruta, o 'qa:<nombre>' (escenarios del harness)."""
    if name.startswith('qa:'):
        return os.path.join(SCEN_DIR, name[3:] + '.yaml')
    return name


# ----------------------------------------------------------------------
# Contexto compartido entre las envolturas
# ----------------------------------------------------------------------
class Window:
    """Ventana de tiempo fija o disparada por un evento.

    spec: {'t': 12.0, 'duracion': 2.0}
          {'cuando_despeje_menor': 3.0, 'duracion': 2.0}
          {'cuando_n_mayor': 12.0, 'duracion': 2.0}
    """

    def __init__(self, spec):
        self.spec = dict(spec)
        self.duration = float(spec.get('duracion', 2.0))
        self.t0 = float(spec['t']) if 't' in spec else None

    def update(self, ctx):
        if self.t0 is not None or not ctx.airborne:
            return
        s = self.spec
        if 'cuando_despeje_menor' in s and ctx.last_clear < float(s['cuando_despeje_menor']):
            self.t0 = ctx.t
        elif 'cuando_n_mayor' in s and ctx.pos is not None and ctx.pos[0] > float(s['cuando_n_mayor']):
            self.t0 = ctx.t
        if self.t0 is not None:
            ctx.event(f'ventana {s} activada en t={self.t0:.2f}')

    def active(self, t):
        return self.t0 is not None and self.t0 <= t < self.t0 + self.duration

    def describe(self):
        if self.t0 is None:
            return 'nunca se activó'
        return f'{self.t0:.1f}-{self.t0 + self.duration:.1f} s'


class Ctx:

    def __init__(self, spec, dt=0.05):
        self.spec = spec
        self.dt = dt
        self.t = 0.0
        self.vehicle = None
        self.core = None
        self.mapper = None
        self.world = None
        self.lidar = None
        self.pos = None
        self.airborne = False
        self.last_clear = math.inf
        self.min_clear = math.inf
        self.min_clear_at = None
        self.first_collision = None
        self.samples = []          # (t, n, e, d, yaw, estado, despeje)
        self.probes = []           # (t, celdas ocupadas a <= 6 m del dron)
        self.events = []
        self.counters = {'escaneos': 0, 'escaneos_cortados': 0, 'escaneos_ciegos': 0,
                         'sectores_descartados': 0, 'puntos_polvo': 0, 'puntos_perdidos': 0}
        self.cut = [Window(w) for w in spec.get('corte_sensor', [])]
        self.blind = [Window(w) for w in spec.get('sensor_ciego', [])]
        self.collision_radius = float(spec.get('radio_choque', 0.3))

    def event(self, msg):
        self.events.append((round(self.t, 2), msg))

    def state_name(self):
        if self.core is None:
            return '?'
        return getattr(self.core.state, 'name', str(self.core.state))


# ----------------------------------------------------------------------
# Construcción de las subclases (después de importar el repo)
# ----------------------------------------------------------------------
def build_classes(cl, ctx):
    originals = getattr(cl, '_qa_originals', None)
    if originals is None:
        originals = {k: getattr(cl, k, None) for k in
                     ('World', 'SimVehicle', 'WallMapper', 'MapperConfig', 'MissionCore', 'LidarModel')}
        cl._qa_originals = originals
    World = originals['World']
    SimVehicle = originals['SimVehicle']
    WallMapper = originals['WallMapper']
    MapperConfig = originals['MapperConfig']
    MissionCore = originals['MissionCore']
    LidarModel = originals['LidarModel']
    spec = ctx.spec

    # ---------------- mundo con obstáculos dinámicos ----------------
    class HarnessWorld(World):

        def __init__(self, scenario, *args, **kwargs):
            super().__init__(scenario, *args, **kwargs)
            ctx.world = self
            self._static_boxes = list(self.boxes)
            self._dyn = [DynamicBox(d) for d in spec.get('dinamicos', [])]

        def _sync(self):
            if not self._dyn:
                return
            for d in self._dyn:
                d.trigger(ctx)
            self.boxes = self._static_boxes + [b for b in (d.box(ctx.t) for d in self._dyn) if b]

        def raycast(self, origin, directions, max_range):
            self._sync()
            return super().raycast(origin, directions, max_range)

        def clearance(self, position):
            self._sync()
            return super().clearance(position)

        def static_clearance(self, position):
            saved = self.boxes
            self.boxes = self._static_boxes
            try:
                return World.clearance(self, position)
            finally:
                self.boxes = saved

    # ---------------- vehículo: inercia, viento, registro ----------------
    vkw = dict(spec.get('vehiculo', {}))

    class HarnessVehicle(SimVehicle):

        def __init__(self, *args, **kwargs):
            merged = dict(kwargs)
            merged.update(vkw)
            super().__init__(*args, **merged)
            ctx.vehicle = self
            self._n = 0
            self._wind = Wind(spec.get('viento'), seed=int(spec.get('semilla', 0)) + 101)

        def step(self, dt, setpoint=None, velocity_ff=None):
            ctx.dt = dt
            ctx.t = self._n * dt
            self._n += 1
            super().step(dt, setpoint, velocity_ff)
            altitude = -float(self.position[2])
            ctx.airborne = bool(self.armed and altitude > 0.3)
            if ctx.airborne and not self.landing:
                w = self._wind.effective(ctx.t, dt)
                if w is not None:
                    self.position[0] += w[0] * dt
                    self.position[1] += w[1] * dt
            ctx.pos = self.position.copy()
            clear = math.inf
            if ctx.airborne and ctx.world is not None:
                clear = float(ctx.world.clearance(self.position))
                ctx.last_clear = clear
                if clear < ctx.min_clear:
                    ctx.min_clear = clear
                    ctx.min_clear_at = (round(ctx.t, 2), round(float(self.position[0]), 2),
                                        round(float(self.position[1]), 2), ctx.state_name())
                if clear < ctx.collision_radius and ctx.first_collision is None:
                    ctx.first_collision = ctx.min_clear_at
                    ctx.event(f'CHOQUE: despeje {clear:.2f} m en ({self.position[0]:.2f}, '
                              f'{self.position[1]:.2f}) estado {ctx.state_name()}')
            for w in ctx.cut + ctx.blind:
                w.update(ctx)
            if self._n % 5 == 0 and ctx.mapper is not None:
                ctx.probes.append((round(ctx.t, 2), occupied_near(ctx.mapper, self.position, 6.0)))
            ctx.samples.append((round(ctx.t, 3), float(self.position[0]), float(self.position[1]),
                                float(self.position[2]), float(self.yaw), ctx.state_name(), clear))

    # ---------------- mapper: corte del sensor + rumbo del sensor ----------------
    mcfg = dict(spec.get('mapper', {}))
    inject_heading = bool(spec.get('rumbo_sensor', False))

    def config_factory(*args, **kwargs):
        merged = dict(kwargs)
        merged.update(mcfg)
        return MapperConfig(*args, **merged)

    class HarnessMapper(WallMapper):

        def __init__(self, *args, **kwargs):
            super().__init__(*args, **kwargs)
            ctx.mapper = self

        def integrate(self, points_world, sensor_origin, drone_position, stamp, *args, **kwargs):
            ctx.counters['escaneos'] += 1
            if any(w.active(ctx.t) for w in ctx.cut):
                ctx.counters['escaneos_cortados'] += 1
                return 0
            if inject_heading and 'sensor_heading' not in kwargs and len(args) < 2:
                yaw = ctx.vehicle.yaw if ctx.vehicle is not None else 0.0
                mount_yaw = math.radians(float(ctx.lidar.mount_rpy_deg[2])) if ctx.lidar else 0.0
                kwargs['sensor_heading'] = yaw + mount_yaw
            return super().integrate(points_world, sensor_origin, drone_position, stamp, *args, **kwargs)

    # ---------------- misión: sectores sólo con fuente fresca ----------------
    gate = bool(spec.get('sectores_como_ros', True))

    class HarnessCore(MissionCore):

        def __init__(self, *args, **kwargs):
            super().__init__(*args, **kwargs)
            ctx.core = self

        def update_sectors(self, ranges, stamp, *args, **kwargs):
            m = ctx.mapper
            if gate and m is not None and hasattr(m, 'has_fresh_source') \
                    and not m.has_fresh_source(stamp):
                # mapper_node no publica /obstacles/sectors sin fuente fresca.
                ctx.counters['sectores_descartados'] += 1
                return None
            return super().update_sectors(ranges, stamp, *args, **kwargs)

    # ---------------- lidar ----------------
    lkw = dict(spec.get('lidar', {}))

    class HarnessLidar(LidarModel):

        def __init__(self, **kwargs):
            super().__init__(**kwargs)
            self._rng = np.random.default_rng(int(spec.get('semilla', 0)) + 7)
            self._queue = deque()
            self._phase = 0.0
            self._elev_off = 0.0
            p = spec.get('percepcion', {})
            self.drop_frac = float(p.get('perdidos', 0.0))
            self.dust = p.get('polvo')
            self.latency = float(p.get('latencia', 0.0))
            self.az_jitter = bool(p.get('fase_azimut_aleatoria', False))
            self.el_jitter = bool(p.get('fase_elevacion_aleatoria', False))
            ctx.lidar = self

        def directions_frd(self):
            dirs = super().directions_frd()
            if self._phase == 0.0 and self._elev_off == 0.0:
                return dirs
            # Re-generar con fase: rotar en azimut y desplazar la elevación.
            az = np.arctan2(dirs[:, 1], dirs[:, 0]) + self._phase
            el = np.arcsin(np.clip(-dirs[:, 2], -1, 1)) + self._elev_off
            return np.column_stack((np.cos(el) * np.cos(az), np.cos(el) * np.sin(az), -np.sin(el)))

        def scan(self, world, position_ned, q, rng=None):
            if self.az_jitter:
                step = math.radians(self.horizontal_fov_deg) / max(1, self.horizontal_samples)
                self._phase = float(self._rng.uniform(0.0, step))
            if self.el_jitter and self.channels > 1:
                step = math.radians(self.vertical_max_deg - self.vertical_min_deg) / (self.channels - 1)
                self._elev_off = float(self._rng.uniform(-step / 2, step / 2))
            if any(w.active(ctx.t) for w in ctx.blind):
                ctx.counters['escaneos_ciegos'] += 1
                pts = np.zeros((0, 3))
            else:
                pts = super().scan(world, position_ned, q, rng)
                if self.drop_frac > 0 and len(pts):
                    keep = self._rng.random(len(pts)) >= self.drop_frac
                    ctx.counters['puntos_perdidos'] += int((~keep).sum())
                    pts = pts[keep]
                if self.dust:
                    n = int(self.dust.get('puntos', 5))
                    lo, hi = self.dust.get('distancia', [0.5, 2.0])
                    v = self._rng.normal(size=(n, 3))
                    v /= np.maximum(np.linalg.norm(v, axis=1, keepdims=True), 1e-9)
                    dust = v * self._rng.uniform(lo, hi, size=(n, 1))
                    pts = np.vstack((pts, dust))
                    ctx.counters['puntos_polvo'] += n
            if self.latency <= 0:
                return pts
            # La nube capturada en t llega en t + latencia; run() la transforma
            # con la pose ACTUAL (la del momento de llegada).
            self._queue.append((ctx.t, pts))
            out = None
            while self._queue and self._queue[0][0] <= ctx.t - self.latency + 1e-6:
                out = self._queue.popleft()[1]
            if out is None:
                return np.zeros((0, 3))
            return out

    lidar = HarnessLidar(**lkw)
    return {'World': HarnessWorld, 'SimVehicle': HarnessVehicle, 'WallMapper': HarnessMapper,
            'MapperConfig': config_factory, 'MissionCore': HarnessCore}, lidar


def occupied_near(mapper, position, radius):
    """Celdas ocupadas de la grilla 2D a menos de `radius` del dron (memoria local)."""
    try:
        g = mapper.grid
        rows, cols = g.to_cells(np.array([[position[0], position[1]]]))
        span = int(math.ceil(radius / g.resolution))
        r0, r1 = max(0, rows[0] - span), min(g.shape[0], rows[0] + span + 1)
        c0, c1 = max(0, cols[0] - span), min(g.shape[1], cols[0] + span + 1)
        if r0 >= r1 or c0 >= c1:
            return 0
        occ = g.known[r0:r1, c0:c1] & (g.log_odds[r0:r1, c0:c1] >= g.occ_l)
        rr, cc = np.nonzero(occ)
        centers = g.cell_centers(rr + r0, cc + c0)
        d = np.hypot(centers[:, 0] - position[0], centers[:, 1] - position[1])
        return int(np.count_nonzero(d <= radius))
    except Exception:  # noqa: BLE001
        return -1


class DynamicBox:
    """Caja que se mueve por una poligonal [[t_rel, n, e], ...] (lineal entre puntos).

    'inicio': {'t': 10} o {'cuando_n_mayor': 3.0}: cuándo arranca el reloj
    relativo. Antes de arrancar está en el primer punto. Después del último,
    queda quieta ahí (ponerlo lejos para que "se vaya").
    """

    def __init__(self, spec):
        self.spec = spec
        self.size = np.array(spec.get('tamano', [1.0, 1.0, 4.0]), dtype=float)
        self.base = float(spec.get('base', 0.0))
        self.path = np.array(spec['trayectoria'], dtype=float)
        self.start = spec.get('inicio', {'t': 0.0})
        self.t0 = float(self.start['t']) if 't' in self.start else None
        yaw = math.radians(float(spec.get('yaw', 0.0)))
        cy, sy = math.cos(yaw), math.sin(yaw)
        self.rot = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]])

    def trigger(self, ctx):
        if self.t0 is None and ctx.airborne and ctx.pos is not None:
            if 'cuando_n_mayor' in self.start and ctx.pos[0] > float(self.start['cuando_n_mayor']):
                self.t0 = ctx.t
                ctx.event(f'caja dinámica arranca en t={ctx.t:.2f}')

    def position(self, t):
        if self.t0 is None:
            return self.path[0, 1:]
        tr = t - self.t0
        p = self.path
        if tr <= p[0, 0]:
            return p[0, 1:]
        for i in range(len(p) - 1):
            if tr <= p[i + 1, 0]:
                a = (tr - p[i, 0]) / max(p[i + 1, 0] - p[i, 0], 1e-9)
                return p[i, 1:] + a * (p[i + 1, 1:] - p[i, 1:])
        return p[-1, 1:]

    def box(self, t):
        ne = self.position(t)
        center = np.array([ne[0], ne[1], -(self.base + self.size[2] / 2.0)])
        return (center, self.rot, self.size / 2.0)


class Wind:
    """Viento como velocidad de deriva sumada a la posición del dron.

    spec: {'constante': [vn, ve], 'rafagas': {'amplitud': 2.0, 'duracion': [1, 3],
           'intervalo': [3, 8]}, 'compensacion_tau': 2.0}
    `compensacion_tau`: el controlador estima el viento con un filtro de primer
    orden (como el integrador de velocidad de PX4) y lo resta; None = sin
    compensación (sólo el P de posición del SimVehicle).
    """

    def __init__(self, spec, seed=0):
        self.spec = spec or None
        self.est = np.zeros(2)
        self.gusts = []
        if not self.spec:
            return
        self.const = np.array(spec.get('constante', [0.0, 0.0]), dtype=float)
        self.tau = spec.get('compensacion_tau')
        g = spec.get('rafagas')
        if g:
            rng = np.random.default_rng(seed)
            t = float(g.get('desde', 6.0))
            while t < 400.0:
                dur = rng.uniform(*g.get('duracion', [1.0, 3.0]))
                ang = rng.uniform(-math.pi, math.pi)
                amp = float(g.get('amplitud', 2.0)) * rng.uniform(0.6, 1.0)
                self.gusts.append((t, dur, amp * np.array([math.cos(ang), math.sin(ang)])))
                t += dur + rng.uniform(*g.get('intervalo', [3.0, 8.0]))

    def raw(self, t):
        w = self.const.copy()
        for t0, dur, vec in self.gusts:
            if t0 <= t < t0 + dur:
                w += vec * 0.5 * (1 - math.cos(2 * math.pi * (t - t0) / dur))
        return w

    def effective(self, t, dt):
        if not self.spec:
            return None
        w = self.raw(t)
        if self.tau:
            self.est += (w - self.est) * min(1.0, dt / float(self.tau))
            return w - self.est
        return w


# ----------------------------------------------------------------------
# Corrida de un caso
# ----------------------------------------------------------------------
def run_case(spec, repo=DEFAULT_REPO, max_time=None, verbose=False):
    cl = setup_repo(repo)
    ctx = Ctx(spec)
    classes, lidar = build_classes(cl, ctx)
    applied = []
    for name, cls in classes.items():
        if getattr(cl, name, None) is not None:
            setattr(cl, name, cls)
            applied.append(name)
    scenario = cl.load_scenario(resolve_scenario(spec['escenario']))
    mission = spec.get('mision') or scenario.mission.get('tipo', 'mapeo')
    kwargs = dict(mission=mission, max_time=float(max_time or spec.get('max_time', 300.0)),
                  lidar=lidar, overrides=dict(spec.get('overrides', {})),
                  collision_radius=ctx.collision_radius, seed=int(spec.get('semilla', 0)),
                  verbose=verbose)
    params = inspect.signature(cl.run).parameters
    kwargs = {k: v for k, v in kwargs.items() if k in params}
    t0 = time.monotonic()
    result = cl.run(scenario, **kwargs)
    wall = time.monotonic() - t0
    out = metrics(cl, ctx, spec, scenario, mission, result, kwargs['max_time'], wall)
    out['patches'] = applied
    out['repo'] = os.path.abspath(repo)
    return out, result, ctx


def metrics(cl, ctx, spec, scenario, mission, result, max_time, wall):
    states = [(t, n) for t, n, *_ in getattr(result, 'states', [])]
    final = getattr(result, 'final_state', ctx.state_name())
    sim_time = float(getattr(result, 'sim_time', ctx.t))
    logs = list(getattr(result, 'log', []))
    abandoned = [m for lvl, m in logs if 'abandonado' in m]
    timed_out = final not in END

    # --- waypoints alcanzados (por trayectoria) ---
    core = getattr(result, 'core', ctx.core)
    params = cl.mission_params(scenario, mission, spec.get('overrides', {}))
    acc = float(params.get('acceptance_radius', 0.8))
    wp_total = wp_hit = 0
    wp_detail = []
    S = np.array([(s[1], s[2]) for s in ctx.samples]) if ctx.samples else np.zeros((0, 2))
    if mission == 'waypoints':
        flat = [float(v) for v in params.get('waypoints', [])]
        home = core.home if core is not None and core.home is not None else (0.0, 0.0, 0.0)
        for i in range(0, len(flat), 3):
            wp = np.array([home[0] + flat[i], home[1] + flat[i + 1]])
            d = float(np.min(np.hypot(*(S - wp).T))) if len(S) else math.inf
            ok = d <= acc + 0.25
            wp_total += 1
            wp_hit += int(ok)
            wp_detail.append(round(d, 2))

    # --- trabado: ventanas de 20 s en vuelo con desplazamiento < 1 m ---
    stall_win = float(spec.get('ventana_trabado', 20.0))
    longest = 0.0
    stall_at = None
    if ctx.samples:
        arr = np.array([(s[0], s[1], s[2]) for s in ctx.samples])
        st = [s[5] for s in ctx.samples]
        flying = np.array([x in MOVING or x == 'HOLD' for x in st])
        n = len(arr)
        step = max(1, int(round(1.0 / ctx.dt)))
        for i in range(0, n, step):
            if not flying[i]:
                continue
            d = np.hypot(arr[i:, 1] - arr[i, 1], arr[i:, 2] - arr[i, 2])
            stop = (d >= 1.0) | ~flying[i:]
            j = i + (int(np.argmax(stop)) - 1 if stop.any() else len(d) - 1)
            dur = arr[max(j, i), 0] - arr[i, 0]
            if dur > longest:
                longest = dur
                stall_at = (round(float(arr[i, 0]), 1), round(float(arr[i, 1]), 1),
                            round(float(arr[i, 2]), 1), st[i])
    stuck = timed_out or longest >= stall_win

    # --- tiempo en cada estado ---
    tin = {}
    for k, (t, name) in enumerate(states):
        t_end = states[k + 1][0] if k + 1 < len(states) else sim_time
        tin[name] = round(tin.get(name, 0.0) + (t_end - t), 1)
    avoid_entries = sum(1 for _, n in states if n == 'AVOID')

    min_clear = float(getattr(result, 'min_clearance', ctx.min_clear))
    collided = bool(getattr(result, 'collided', ctx.first_collision is not None))
    out = {
        'caso': spec['id'], 'grupo': spec.get('grupo', ''), 'titulo': spec.get('titulo', ''),
        'escenario': scenario.name, 'mision': mission,
        'estado_final': final, 'tiempo_sim': round(sim_time, 2), 'tiempo_real': round(wall, 1),
        'max_time': max_time, 'agotado': timed_out,
        'choque': collided, 'despeje_min': round(min_clear, 3) if math.isfinite(min_clear) else None,
        'despeje_min_en': ctx.min_clear_at, 'primer_choque': ctx.first_collision,
        'trabado': stuck, 'mayor_quieto_s': round(longest, 1), 'quieto_en': stall_at,
        'waypoints': f'{wp_hit}/{wp_total}' if wp_total else '-',
        'dist_min_a_wp': wp_detail,
        'objetivos_abandonados': abandoned,
        'entradas_avoid': avoid_entries, 'tiempo_por_estado': tin,
        'recorrido_m': round(float(getattr(result, 'distance_traveled', 0.0)), 1),
        'cobertura': (round(float(result.coverage), 3)
                      if math.isfinite(getattr(result, 'coverage', float('nan'))) else None),
        'paredes': [getattr(result, 'walls_flat', None), getattr(result, 'walls_irregular', None),
                    getattr(result, 'obstacles', None)],
        'estados': states,
        'eventos': ctx.events,
        'contadores': ctx.counters,
        'ventanas': {'corte_sensor': [w.describe() for w in ctx.cut],
                     'sensor_ciego': [w.describe() for w in ctx.blind]},
        'memoria_6m_en_ventanas': [_probe_window(ctx, w) for w in ctx.cut + ctx.blind],
        'log_warn': [m for lvl, m in logs if lvl in ('warn', 'error')][:15],
    }
    # --- calidad del mapa: celdas ocupadas lejos de cualquier superficie real ---
    try:
        out.update(map_accuracy(ctx, result))
    except Exception as exc:  # noqa: BLE001 - la métrica no debe tumbar el caso
        out['mapa_error'] = repr(exc)
    exp_s = scenario.expected or {}
    walls_ok = True
    for key, idx, cmp in (('paredes_planas_min', 0, 'min'), ('paredes_irregulares_min', 1, 'min'),
                          ('obstaculos_min', 2, 'min'), ('paredes_irregulares_max', 1, 'max')):
        if key in exp_s and out['paredes'][idx] is not None:
            v = out['paredes'][idx]
            walls_ok &= v >= exp_s[key] if cmp == 'min' else v <= exp_s[key]
    out['paredes_como_escenario'] = walls_ok
    # --- chequeos propios del caso ---
    extra = {}
    for check in spec.get('chequeos', []):
        fn = CHECKS[check['tipo']]
        extra.update(fn(ctx, result, check))
    out['extra'] = extra
    out['veredicto'], out['motivos'] = verdict(spec, out)
    return out


def _probe_window(ctx, w):
    """Celdas ocupadas a <= 6 m del dron justo antes y justo al final de la ventana."""
    if w.t0 is None or not ctx.probes:
        return None
    before = [c for t, c in ctx.probes if t <= w.t0]
    during = [c for t, c in ctx.probes if w.t0 <= t <= w.t0 + w.duration]
    return {'antes': before[-1] if before else None,
            'minimo_en_ventana': min(during) if during else None,
            'al_final': during[-1] if during else None}


def _clearance_2d(world, pts_ne, heights):
    """Distancia horizontal mínima (sobre varias alturas) a la geometría real.

    Vectorizado sobre la estructura interna de World (boxes/cylinders); si la
    otra copia del repo la cambió, cae al World.clearance punto a punto.
    """
    pts_ne = np.asarray(pts_ne, dtype=float).reshape(-1, 2)
    best = np.full(len(pts_ne), np.inf)
    try:
        for h in heights:
            p = np.column_stack((pts_ne, np.full(len(pts_ne), -h)))
            for center, rot, half in world.boxes:
                local = (p - center) @ rot
                delta = np.maximum(np.abs(local) - half, 0.0)
                best = np.minimum(best, np.linalg.norm(delta, axis=1))
            for cyl in world.cylinders:
                radial = np.maximum(0.0, np.hypot(p[:, 0] - cyl.center_ne[0], p[:, 1] - cyl.center_ne[1])
                                    - cyl.radius)
                vertical = np.maximum(0.0, np.maximum(cyl.top_d - p[:, 2], p[:, 2] - cyl.bottom_d))
                best = np.minimum(best, np.hypot(radial, vertical))
    except (AttributeError, TypeError, ValueError):
        best = np.array([min(world.clearance(np.array([n, e, -h])) for h in heights) for n, e in pts_ne])
    return best


def map_accuracy(ctx, result, tol=0.35):
    mapper = getattr(result, 'mapper', None) or ctx.mapper
    if mapper is None or ctx.world is None:
        return {}
    pts = mapper.grid.occupied_points()
    if len(pts) == 0:
        return {'mapa_celdas_ocupadas': 0, 'mapa_mal_ubicado_pct': 0.0}
    cfg = mapper.cfg
    heights = np.arange(max(cfg.min_height, 0.3), min(cfg.max_height, 6.0) + 1e-6, 0.25)
    d = _clearance_2d(ctx.world, pts, heights)
    bad = d > tol
    return {'mapa_celdas_ocupadas': int(len(pts)),
            'mapa_mal_ubicado_pct': round(100.0 * float(bad.mean()), 1),
            'mapa_mal_ubicado_m2': round(float(bad.sum()) * mapper.grid.resolution ** 2, 2),
            'mapa_error_max_m': round(float(d.max()), 2)}


# ----------------------------------------------------------------------
# Chequeos específicos
# ----------------------------------------------------------------------
def check_ghost_cells(ctx, result, check):
    """Celdas ocupadas que quedaron en el mapa en una zona donde AL FINAL no hay nada."""
    g = result.mapper.grid
    occ = g.occupied_mask()
    rows, cols = np.nonzero(occ)
    centers = g.cell_centers(rows, cols)
    n0, e0, n1, e1 = check['zona']
    inside = (centers[:, 0] >= n0) & (centers[:, 0] <= n1) & (centers[:, 1] >= e0) & (centers[:, 1] <= e1)
    world = ctx.world
    ghosts = 0
    for c in centers[inside]:
        p = np.array([c[0], c[1], -3.0])
        if world.clearance(p) > 0.5:
            ghosts += 1
    return {'celdas_fantasma_final': ghosts,
            'm2_fantasma_final': round(ghosts * g.resolution ** 2, 2)}


def check_pass_time(ctx, result, check):
    """Cuándo cruzó el dron la línea n = valor por primera vez (y vuelta)."""
    n_line = float(check['n'])
    cross = []
    prev = None
    for s in ctx.samples:
        side = s[1] > n_line
        if prev is not None and side != prev:
            cross.append(round(s[0], 1))
        prev = side
    return {f'cruces_n{n_line:g}': cross}


CHECKS = {'fantasmas': check_ghost_cells, 'cruce': check_pass_time}


def verdict(spec, out):
    exp = spec.get('esperado', {})
    reasons = []
    bad = False
    if out['choque']:
        bad = True
        reasons.append(f"choque (despeje mín {out['despeje_min']} m)")
    if out['agotado']:
        bad = True
        reasons.append(f"no terminó en {out['max_time']:.0f} s ({out['estado_final']})")
    elif out['estado_final'] != exp.get('estado_final', 'DONE'):
        bad = True
        reasons.append(f"estado final {out['estado_final']}")
    if out['mision'] == 'waypoints' and exp.get('waypoints', True) and out['waypoints'] != '-':
        hit, total = (int(x) for x in out['waypoints'].split('/'))
        if hit < total:
            bad = True
            reasons.append(f"waypoint no alcanzado ({out['waypoints']}; volvió/abandonó)")
    if out['mayor_quieto_s'] >= float(spec.get('ventana_trabado', 20.0)):
        if not bad:
            reasons.append(f"trabado {out['mayor_quieto_s']:.0f} s")
        else:
            reasons.append(f"quieto {out['mayor_quieto_s']:.0f} s")
        bad = True
    if 'cobertura_min' in exp and (out['cobertura'] or 0) < exp['cobertura_min']:
        bad = True
        reasons.append(f"cobertura {out['cobertura']} < {exp['cobertura_min']}")
    if 'fantasmas_max_m2' in exp:
        m2 = out['extra'].get('m2_fantasma_final', 0)
        if m2 > exp['fantasmas_max_m2']:
            bad = True
            reasons.append(f'el mapa no olvidó la caja: {m2} m² ocupados donde ya no hay nada')
    warn = []
    if not bad and out['despeje_min'] is not None and out['despeje_min'] < float(exp.get('despeje_aviso', 0.6)):
        warn.append(f"despeje justo ({out['despeje_min']} m)")
    t_max = float(exp.get('tiempo_max', 120.0))
    if not out['agotado'] and out['tiempo_sim'] > t_max:
        warn.append(f"lento: {out['tiempo_sim']:.0f} s simulados (> {t_max:.0f} s; los controles tardan 35-60 s)")
    pct = out.get('mapa_mal_ubicado_pct')
    if pct is not None and pct > float(exp.get('mapa_mal_ubicado_max_pct', 10.0)):
        warn.append(f"mapa: {pct}% de celdas ocupadas a >0.35 m de una superficie real "
                    f"({out.get('mapa_mal_ubicado_m2')} m²)")
    if not out.get('paredes_como_escenario', True):
        warn.append(f"paredes {out['paredes']} no cumplen el 'esperado' del escenario")
    if bad:
        return 'FALLA', reasons + [f'(aviso) {w}' for w in warn]
    if warn:
        return 'AVISO', warn
    return 'PASA', ['ok']
