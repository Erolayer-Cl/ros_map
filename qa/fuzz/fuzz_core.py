#!/usr/bin/env python3
"""Fuzzer adversarial de la lógica pura: entradas basura que pueden llegar de
un driver o de PX4 en un vuelo real. Cada caso reporta EXC (excepción),
BAD (salida no finita / estado inválido) u OK."""
import math
import os
import sys
import time
import traceback

import numpy as np

REPO = sys.argv[1] if len(sys.argv) > 1 else os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
for p in ('src/uav_wall_mapping', 'src/uav_mission', 'src/uav_sim'):
    sys.path.insert(0, f'{REPO}/{p}')

from uav_wall_mapping.mapper_core import WallMapper, MapperConfig  # noqa: E402
from uav_wall_mapping.geometry import SensorMount, quat_from_yaw  # noqa: E402
from uav_wall_mapping.cloud import pointcloud2_to_xyz, laserscan_to_xyz, range_to_xyz  # noqa
from uav_wall_mapping.pose_buffer import PoseBuffer  # noqa: E402
from uav_wall_mapping import sectors as S  # noqa: E402
from uav_mission.mission_core import MissionCore  # noqa: E402

results = []
np.random.seed(0)


def case(name):
    def deco(fn):
        t0 = time.monotonic()
        try:
            r = fn()
            status = 'OK' if r in (None, True) else f'BAD: {r}'
        except Exception as e:  # noqa: BLE001
            status = f'EXC: {type(e).__name__}: {e}'
            tb = traceback.format_exc().strip().splitlines()
            status += ' @ ' + ' | '.join(l.strip() for l in tb[-4:-1])
        dt = time.monotonic() - t0
        if dt > 3.0:
            status += f' (LENTO {dt:.1f}s)'
        results.append((name, status))
        return fn
    return deco


def wall_cloud(n=2000, dist=4.0):
    y = np.random.uniform(-3, 3, n)
    z = np.random.uniform(-1, 1, n)
    return np.column_stack((np.full(n, dist), y, z))


def fresh_mapper():
    m = WallMapper(MapperConfig())
    m.update_trajectory((0.0, 0.0, -2.0))
    return m


MOUNT = SensorMount(axes='flu')
Q0 = quat_from_yaw(0.0)
POS = (0.0, 0.0, -2.0)


def sane_mapper(m, now=1.0):
    info = m.sector_info(now, POS)
    r = np.asarray(info.ranges, dtype=float)
    if np.any(np.isnan(r)):
        return 'sectores con NaN'
    m.snapshot(now)
    m.extract_walls()
    return None


# ---------------- mapper -----------------
@case('mapper: puntos NaN/inf mezclados')
def _():
    m = fresh_mapper()
    pts = wall_cloud()
    pts[::7] = np.nan
    pts[::11, 1] = np.inf
    m.integrate_sensor(pts, MOUNT, POS, Q0, 1.0)
    return sane_mapper(m)


@case('mapper: nube vacía y (0,3)')
def _():
    m = fresh_mapper()
    m.integrate_sensor(np.zeros((0, 3)), MOUNT, POS, Q0, 1.0)
    m.integrate_sensor([], MOUNT, POS, Q0, 1.1)
    return sane_mapper(m, 1.2)


@case('mapper: punto a 1e9 m (grilla no explota)')
def _():
    m = fresh_mapper()
    pts = np.vstack([wall_cloud(), [[1e9, 1e9, 0.0]], [[-1e7, 3e8, 5.0]]])
    m.integrate_sensor(pts, MOUNT, POS, Q0, 1.0)
    cells = m.grid.shape[0] * m.grid.shape[1]
    if cells > 5e7:
        return f'grilla enorme {m.grid.shape}'
    return sane_mapper(m)


@case('mapper: pose NaN')
def _():
    m = fresh_mapper()
    m.integrate_sensor(wall_cloud(), MOUNT, (math.nan, 0.0, -2.0), Q0, 1.0)
    return sane_mapper(m)


@case('mapper: cuaternión cero / NaN')
def _():
    m = fresh_mapper()
    m.integrate_sensor(wall_cloud(), MOUNT, POS, [0, 0, 0, 0], 1.0)
    m.integrate_sensor(wall_cloud(), MOUNT, POS, [math.nan, 0, 0, 1], 1.1)
    return sane_mapper(m, 1.2)


@case('mapper: stamp NaN / hacia atrás')
def _():
    m = fresh_mapper()
    m.integrate_sensor(wall_cloud(), MOUNT, POS, Q0, 5.0)
    m.integrate_sensor(wall_cloud(), MOUNT, POS, Q0, 1.0)
    m.integrate_sensor(wall_cloud(), MOUNT, POS, Q0, math.nan)
    return sane_mapper(m, 5.1)


@case('mapper: trayectoria NaN')
def _():
    m = fresh_mapper()
    m.update_trajectory((math.nan, 0.0, -2.0))
    m.update_trajectory((1.0, math.inf, -2.0))
    m.integrate_sensor(wall_cloud(), MOUNT, POS, Q0, 1.0)
    return sane_mapper(m)


@case('mapper: todos los puntos en el origen del sensor')
def _():
    m = fresh_mapper()
    m.integrate_sensor(np.zeros((500, 3)), MOUNT, POS, Q0, 1.0)
    return sane_mapper(m)


@case('mapper: 300k puntos (tiempo)')
def _():
    m = fresh_mapper()
    pts = np.random.uniform(-20, 20, (300000, 3))
    t0 = time.monotonic()
    m.integrate_sensor(pts, MOUNT, POS, Q0, 1.0, downsample=0.1)
    dt = time.monotonic() - t0
    if dt > 2.0:
        return f'{dt:.2f}s por nube'
    return sane_mapper(m)


@case('mapper: sector_info con pose NaN')
def _():
    m = fresh_mapper()
    m.integrate_sensor(wall_cloud(), MOUNT, POS, Q0, 1.0)
    info = m.sector_info(1.0, (math.nan, 0, -2))
    r = np.asarray(info.ranges, dtype=float)
    if np.any(np.isnan(r)):
        return 'NaN en sectores'


# ---------------- cloud -----------------
class F:
    def __init__(self, name, offset, datatype):
        self.name, self.offset, self.datatype = name, offset, datatype


XYZ = [F('x', 0, 7), F('y', 4, 7), F('z', 8, 7)]


@case('cloud: data truncada (width mayor que los bytes)')
def _():
    data = np.zeros((10, 3), np.float32).tobytes()
    out = pointcloud2_to_xyz(XYZ, data, 12, 100, 1)
    if out.shape[0] > 10:
        return 'leyó más de lo que hay'


@case('cloud: point_step menor que los campos')
def _():
    data = np.zeros((10, 3), np.float32).tobytes()
    try:
        pointcloud2_to_xyz(XYZ, data, 8, 10, 1)
    except ValueError:
        return None
    return 'sin ValueError'


@case('cloud: datatype desconocido')
def _():
    data = np.zeros((10, 3), np.float32).tobytes()
    try:
        pointcloud2_to_xyz([F('x', 0, 99), F('y', 4, 7), F('z', 8, 7)], data, 12, 10, 1)
    except ValueError:
        return None
    return 'sin ValueError'


@case('cloud: data no múltiplo de point_step')
def _():
    data = np.zeros(37, np.uint8).tobytes()
    pointcloud2_to_xyz(XYZ, data, 12, 3, 1)


@case('cloud: laserscan con range_max NaN / incremento 0')
def _():
    laserscan_to_xyz([1.0, 2.0, math.nan], 0.0, 0.0, 0.1, math.nan)
    out2 = laserscan_to_xyz([1.0, -1.0, math.inf], -1.0, 0.01, 0.1, 30.0)
    if not np.all(np.isfinite(out2)):
        return 'no finito'


@case('cloud: range NaN')
def _():
    range_to_xyz(math.nan, 0.1, 10.0)
    range_to_xyz(5.0, math.nan, math.nan)


# ---------------- pose buffer -----------------
@case('pose: muestras fuera de orden y NaN')
def _():
    pb = PoseBuffer()
    pb.add_position(1.0, (0, 0, 0))
    pb.add_position(0.5, (1, 1, 1))
    pb.add_position(math.nan, (1, 1, 1))
    pb.add_position(1.5, (math.nan, 0, 0))
    pb.add_attitude(1.0, Q0)
    pb.add_attitude(1.5, Q0)
    r = pb.lookup(1.2)
    if r is not None and not np.all(np.isfinite(r[0])):
        return f'pose no finita {r[0]}'
    pb.lookup(math.nan)


@case('pose: muchas muestras iguales en t')
def _():
    pb = PoseBuffer()
    for _ in range(1000):
        pb.add_position(1.0, (0, 0, 0))
        pb.add_attitude(1.0, Q0)
    pb.lookup(1.0)


# ---------------- sectores -----------------
@case('sectors: compute con NaN')
def _():
    r, c = S.compute_sectors(np.array([[math.nan, 1.0], [1.0, 1.0]]), 72)
    if np.any(np.isnan(r)):
        return 'NaN en rangos'


@case('sectors: to_px4 con NaN/negativos')
def _():
    out = S.to_px4_obstacle_distance([math.nan, -1.0, math.inf] + [1.0] * 69, 20.0)
    if any(v < 0 or v > 65535 for v in out):
        return 'fuera de uint16'
    if out[0] != 65535:
        return f'NaN se codificó como {out[0]} (no UINT16_MAX)'


# ---------------- misión -----------------
def run_core(feed, steps=400, dt=0.05, start='map'):
    core = MissionCore()
    t = 0.0
    pos = [0.0, 0.0, 0.0]
    armed = off = False
    for i in range(steps):
        feed(core, i, t, pos)
        core.update_status(armed, off)
        if i == 10:
            (core.start_mapping if start == 'map' else core.start_waypoints)(t)
        core.planning_request(t)
        core.step(t)
        for c in core.pop_commands():
            if c == 'arm':
                armed = True
            elif c == 'mode_offboard':
                off = True
        sp = core.setpoint
        if core.stream_offboard and sp is not None:
            if not all(math.isfinite(v) for v in sp):
                return f'setpoint no finito {sp} en estado {core.state.name}'
            # integrar a lo bruto hacia el setpoint
            for k in range(3):
                pos[k] += max(-0.1, min(0.1, sp[k] - pos[k]))
        vs = core.velocity_setpoint
        if vs is not None and not all(math.isfinite(v) for v in vs):
            return f'velocidad no finita {vs}'
        t += dt
    core.status()
    return core


def basic_feed(core, i, t, pos, ranges=None):
    core.update_vehicle(tuple(pos), (0, 0, 0), 0.0, stamp=t)
    r = np.full(72, 10.0) if ranges is None else ranges
    core.update_sectors(r, t, range_max=20.0)


@case('misión: posición NaN a mitad de vuelo')
def _():
    def feed(core, i, t, pos):
        if 150 <= i < 160:
            core.update_vehicle((math.nan, 0.0, -2.0), (0, 0, 0), 0.0, stamp=t)
            core.update_sectors(np.full(72, 10.0), t)
        else:
            basic_feed(core, i, t, pos)
    r = run_core(feed)
    if isinstance(r, str):
        return r


@case('misión: velocidad/heading NaN')
def _():
    def feed(core, i, t, pos):
        core.update_vehicle(tuple(pos), (math.nan, math.inf, 0), math.nan, stamp=t)
        core.update_sectors(np.full(72, 10.0), t)
    r = run_core(feed)
    if isinstance(r, str):
        return r


@case('misión: sectores NaN')
def _():
    def feed(core, i, t, pos):
        r = np.full(72, 10.0)
        r[::3] = np.nan
        basic_feed(core, i, t, pos, r)
    r = run_core(feed)
    if isinstance(r, str):
        return r


@case('misión: sectores largo 0 / largo raro')
def _():
    def feed(core, i, t, pos):
        core.update_vehicle(tuple(pos), (0, 0, 0), 0.0, stamp=t)
        core.update_sectors(np.full(0 if i % 2 else 7, 10.0), t)
    r = run_core(feed)
    if isinstance(r, str):
        return r


@case('misión: reloj hacia atrás')
def _():
    core = MissionCore()
    t = 10.0
    for i in range(300):
        core.update_vehicle((0, 0, -2), (0, 0, 0), 0.0, stamp=t)
        core.update_sectors(np.full(72, 10.0), t)
        if i == 5:
            core.start_waypoints(t)
        core.step(t)
        core.pop_commands()
        t += -0.5 if i == 100 else 0.05
        sp = core.setpoint
        if sp is not None and not all(math.isfinite(v) for v in sp):
            return 'setpoint no finito'


@case('misión: stamp de sectores NaN / futuro')
def _():
    def feed(core, i, t, pos):
        core.update_vehicle(tuple(pos), (0, 0, 0), 0.0, stamp=t)
        core.update_sectors(np.full(72, 10.0), math.nan if i % 3 == 0 else t + 100)
    r = run_core(feed)
    if isinstance(r, str):
        return r


@case('misión: snapshot basura en update_map')
def _():
    def feed(core, i, t, pos):
        basic_feed(core, i, t, pos)
        core.update_map(None if i % 2 else 'basura')
    r = run_core(feed)
    if isinstance(r, str):
        return r


@case('misión: planning_result con resultado None/basura')
def _():
    core = MissionCore()
    core.planning_result(12345, None, 0.0)
    core.planning_result(None, object(), 0.0)


@case('misión: comandos repetidos pause/resume/cancel/abort en IDLE')
def _():
    core = MissionCore()
    for f in (core.pause, core.resume, core.cancel, core.abort, core.pause, core.resume):
        f(0.0)
    core.step(0.0)


for name, status in results:
    print(f'{"OK " if status == "OK" else "!! "} {name}: {status}')
ok = sum(s == 'OK' for _, s in results)
print(f'\n{ok}/{len(results)} OK')
sys.exit(0 if ok == len(results) else 1)
