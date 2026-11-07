#!/usr/bin/env python3
"""Simulación offline de lazo cerrado: misión + mapper + dron + lidar, sin ROS.

Corre exactamente las mismas clases que los nodos (`MissionCore`, `WallMapper`,
`SensorMount`, `GridPlanner`) contra un escenario YAML. Sirve para:

- probar un escenario en segundos en cualquier PC (ni ROS ni Unreal),
- las pruebas automáticas (`test/test_closed_loop.py`),
- generar el mapa esperado de cada caso de prueba y compararlo con el de Unreal.

    python3 -m uav_sim.closed_loop --escenario 06_habitacion --mision mapeo --salida /tmp/mapas
    ros2 run uav_sim sim_offline --escenario 02_pared_irregular --mision waypoints
"""

import argparse
import math
import os
import sys
import time
from collections import deque
from dataclasses import dataclass, field

import numpy as np

from uav_mission.mission_core import DEFAULTS, MissionCore, State
from uav_sim.scenario import list_scenarios, load_scenario, mission_overrides
from uav_sim.world import LidarModel, SimVehicle, World
from uav_wall_mapping.export import save_map_bundle
from uav_wall_mapping.geometry import SensorMount
from uav_wall_mapping.mapper_core import MapperConfig, WallMapper
from uav_wall_mapping.pose_buffer import PoseBuffer
from uav_wall_mapping.walls import FLAT, IRREGULAR, OBSTACLE

END_STATES = {State.DONE, State.FAILED, State.PREEMPTED, State.ABORTED}


@dataclass
class SimResult:
    scenario: str
    mission: str
    final_state: str
    sim_time: float
    wall_time: float
    min_clearance: float
    collided: bool
    distance_traveled: float
    walls_flat: int
    walls_irregular: int
    obstacles: int
    coverage: float
    states: list = field(default_factory=list)
    files: list = field(default_factory=list)
    mapper: object = None
    core: object = None
    trajectory: list = field(default_factory=list)
    log: list = field(default_factory=list)


def mission_params(scenario, mission, overrides=None):
    """Parámetros de la misión: defaults + lo que diga el escenario + overrides."""
    params = dict(DEFAULTS)
    params.update(mission_overrides(scenario, mission))
    params.update(overrides or {})
    return params


def run(scenario, mission='mapeo', max_time=400.0, dt=0.05, lidar=None, overrides=None,
        out_dir=None, collision_radius=0.3, seed=0, verbose=False):
    if isinstance(scenario, str):
        scenario = load_scenario(scenario)
    params = mission_params(scenario, mission, overrides)
    world = World(scenario)
    lidar = lidar or LidarModel()
    rng = np.random.default_rng(seed)
    mount = SensorMount(position_frd=lidar.mount_frd, rpy_deg=lidar.mount_rpy_deg, axes='flu')
    mapper = WallMapper(MapperConfig(sensor_fov_deg=float(lidar.horizontal_fov_deg)))
    # Como mapper_node: historial de poses y cada nube transformada con la pose
    # de su captura. `cloud_latency` demora la entrega de la nube; un modelo de
    # lidar que ya devuelve la nube demorada desde `scan()` declara ese retardo
    # en el atributo `latency` (así lo hace el arnés de QA).
    poses = PoseBuffer(horizon=3.0)
    in_flight = deque()
    logs = []

    def log(level, msg):
        logs.append((level, msg))
        if verbose:
            print(f'[{level}] {msg}')

    core = MissionCore(param=lambda name: params[name], log=log)
    planner = core.make_planner()
    vehicle = SimVehicle(position=(0.0, 0.0, 0.0), yaw=float(scenario.mission.get('yaw_inicial', 0.0)))

    t = 0.0
    next_scan = next_map = next_walls = 0.0
    started = False
    min_clear = math.inf
    collided = False
    states = []
    files = []
    wall_start = time.monotonic()

    while t < max_time:
        # --- "PX4" ---
        setpoint = core.setpoint if core.stream_offboard else None
        vehicle.step(dt, setpoint, core.velocity_setpoint)
        core.update_vehicle(vehicle.position, vehicle.velocity, vehicle.yaw)
        core.update_status(vehicle.armed, vehicle.offboard)
        mapper.update_trajectory(vehicle.position)
        poses.add_position(t, vehicle.position)
        poses.add_attitude(t, vehicle.q)

        # --- sensores + mapper ---
        if t >= next_scan:
            next_scan += 0.1
            pts_sensor = lidar.scan(world, vehicle.position, vehicle.q, rng)
            captured = t - float(getattr(lidar, 'latency', 0.0) or 0.0)
            in_flight.append((captured + float(getattr(lidar, 'cloud_latency', 0.0)),
                              captured, pts_sensor))
        delivered = False
        while in_flight and in_flight[0][0] <= t + 1e-9:
            _, stamp, pts_sensor = in_flight.popleft()
            pose = poses.lookup(stamp)
            if pose is None:
                pose, stamp = (vehicle.position, vehicle.q), t
            mapper.integrate_sensor(pts_sensor, mount, pose[0], pose[1], stamp)
            delivered = True
        if delivered:
            # Como mapper_node: sectores contra la pose ACTUAL, con el stamp de
            # la captura, y sólo si hay una fuente en vivo fresca.
            info = mapper.sector_info(t, vehicle.position)
            if info.fresh:
                core.update_sectors(info.ranges, info.stamp, range_max=mapper.cfg.sector_range,
                                    live=info.live)
        if t >= next_map:
            next_map += 0.5
            core.update_map(mapper.snapshot(t))
        if t >= next_walls:
            next_walls += 2.0
            walls = mapper.extract_walls()
            core.update_walls_summary(sum(w.kind == FLAT for w in walls),
                                      sum(w.kind == IRREGULAR for w in walls),
                                      sum(w.kind == OBSTACLE for w in walls))

        # --- misión ---
        if not started and t >= 0.5:
            ok, msg = (core.start_mapping(t) if mission == 'mapeo' else core.start_waypoints(t))
            log('info', f'inicio: {msg}')
            if not ok:
                break
            started = True
        request = core.planning_request(t)
        if request is not None:
            result = planner.plan(request['snapshot'], request['start'], request['goal'])
            core.planning_result(request['id'], result, t)
        core.step(t)
        for command in core.pop_commands():
            if command == 'mode_offboard':
                vehicle.offboard = True
            elif command == 'arm':
                vehicle.armed = True
            elif command == 'land':
                vehicle.landing = True
                vehicle.offboard = False
            elif command == 'map_reset':
                mapper.reset()
            elif command == 'map_save' and out_dir:
                mapper.extract_walls()
                files = save_map_bundle(out_dir, f'{scenario.name}_{mission}', mapper,
                                        {'escenario': scenario.name, 'mision': mission})

        if not states or states[-1][1] != core.state.name:
            states.append((round(t, 2), core.state.name, core.message))
        if vehicle.armed and -vehicle.position[2] > 0.3:
            clear = world.clearance(vehicle.position)
            min_clear = min(min_clear, clear)
            if clear < collision_radius:
                collided = True
        if started and core.state in END_STATES:
            break
        t += dt

    walls = mapper.extract_walls()
    if out_dir and not files:
        files = save_map_bundle(out_dir, f'{scenario.name}_{mission}', mapper,
                                {'escenario': scenario.name, 'mision': mission})
    area = core._map_area
    return SimResult(
        scenario=scenario.name, mission=mission, final_state=core.state.name,
        sim_time=round(t, 2), wall_time=round(time.monotonic() - wall_start, 2),
        min_clearance=min_clear, collided=collided,
        distance_traveled=core._distance_traveled,
        walls_flat=sum(w.kind == FLAT for w in walls),
        walls_irregular=sum(w.kind == IRREGULAR for w in walls),
        obstacles=sum(w.kind == OBSTACLE for w in walls),
        coverage=mapper.known_fraction(area) if area else float('nan'),
        states=states, files=files, mapper=mapper, core=core,
        trajectory=list(mapper.trajectory), log=logs)


def summarize(result):
    lines = [
        f'Escenario: {result.scenario}  misión: {result.mission}',
        f'  estado final: {result.final_state} a los {result.sim_time:.1f} s simulados '
        f'({result.wall_time:.1f} s reales)',
        f'  recorrido: {result.distance_traveled:.1f} m, distancia mínima a obstáculos: '
        f'{result.min_clearance:.2f} m{"  ¡CHOQUE!" if result.collided else ""}',
        f'  paredes: {result.walls_flat} planas, {result.walls_irregular} irregulares, '
        f'{result.obstacles} obstáculos aislados',
    ]
    if math.isfinite(result.coverage):
        lines.append(f'  cobertura del área: {100 * result.coverage:.0f}%')
    lines.append('  estados:')
    for t, name, msg in result.states:
        lines.append(f'    {t:7.1f} s  {name:10s} {msg}')
    for wall in result.mapper.walls:
        extra = ''
        if math.isfinite(wall.height_max):
            extra += f' alto {wall.height_min:.1f}-{wall.height_max:.1f} m'
        if math.isfinite(wall.tilt_deg):
            extra += f' incl {wall.tilt_deg:.0f}°'
        a, b = wall.vertices[0], wall.vertices[-1]
        lines.append(f'  P{wall.id:<3d} {wall.kind_name:10s} {wall.length:5.1f} m  '
                     f'({a[0]:.1f},{a[1]:.1f})->({b[0]:.1f},{b[1]:.1f}) rms {wall.rms:.2f}{extra}')
    if result.files:
        lines.append('  archivos:')
        lines.extend(f'    {f}' for f in result.files)
    return '\n'.join(lines)


def main(argv=None):
    parser = argparse.ArgumentParser(description='Simulación offline (sin ROS ni Unreal) de un escenario.')
    parser.add_argument('--escenario', default='01_pared_plana',
                        help=f'nombre o ruta .yaml. Disponibles: {", ".join(list_scenarios())}')
    parser.add_argument('--mision', choices=('mapeo', 'waypoints'), default=None,
                        help='por defecto, la que indique el escenario')
    parser.add_argument('--salida', default=None, help='carpeta donde guardar el mapa')
    parser.add_argument('--tiempo', type=float, default=600.0, help='tiempo simulado máximo (s)')
    parser.add_argument('--todos', action='store_true', help='correr todos los escenarios')
    parser.add_argument('-v', '--verbose', action='store_true')
    args = parser.parse_args(argv)

    names = list_scenarios() if args.todos else [args.escenario]
    failed = False
    for name in names:
        scenario = load_scenario(name)
        mission = args.mision or scenario.mission.get('tipo', 'mapeo')
        out = args.salida or os.path.join(os.getcwd(), 'mapas_offline')
        result = run(scenario, mission, max_time=args.tiempo, out_dir=out, verbose=args.verbose)
        print(summarize(result))
        print()
        failed |= result.collided or result.final_state != 'DONE'
    return 1 if failed else 0


if __name__ == '__main__':
    sys.exit(main())
