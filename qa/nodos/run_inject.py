"""Los 3 nodos sobre el ROS falso + inyección de mensajes basura en vuelo.

Pasa si: ningún nodo se cae, la misión termina (DONE), no hay choque, y nunca
sale un TrajectorySetpoint no finito hacia PX4."""
import math
import os
import random
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
SRC = os.environ.get('UAV_SRC', os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))), 'src'))
for p in ('uav_wall_mapping', 'uav_mission', 'uav_sim'):
    sys.path.insert(0, os.path.join(SRC, p))

import fakeros as F  # noqa: E402
F.install()

from uav_sim.scenario import load_scenario, mission_overrides  # noqa: E402

scenario_name = sys.argv[1] if len(sys.argv) > 1 else '06_habitacion'
seconds = float(sys.argv[2]) if len(sys.argv) > 2 else 300.0
rate = float(sys.argv[3]) if len(sys.argv) > 3 else 2.0     # basura por segundo
scenario = load_scenario(scenario_name)
kind = scenario.mission.get('tipo', 'mapeo')
out_dir = tempfile.mkdtemp(prefix='mapas_')
F.OVERRIDES['fake_px4_world'] = {'escenario': scenario_name}
F.OVERRIDES['mapper_node'] = {'output_dir': out_dir, 'px4_obstacle_distance.enabled': True}
F.OVERRIDES['mission_node'] = dict(mission_overrides(scenario, kind))

from uav_sim.fake_px4_world import FakePX4World  # noqa: E402
from uav_wall_mapping.mapper_node import MapperNode  # noqa: E402
from uav_mission.mission_node import MissionNode  # noqa: E402

world = FakePX4World()
mapper = MapperNode()
mission = MissionNode()

bad_setpoints = []


def watch_setpoint(m):
    vals = list(m.position) + [m.yaw]
    if not all(math.isfinite(float(v)) for v in vals):
        bad_setpoints.append((F.CLOCK.t, vals))


F.BUS.setdefault('/fmu/in/trajectory_setpoint', []).append(watch_setpoint)
statuses = []
F.BUS.setdefault('/mission/status', []).append(
    lambda m: statuses.append((F.CLOCK.t, m.state_name, m.message)))

LIDAR = '/airsim_node/PX4/lidar/points/lidar'
POS = '/fmu/out/vehicle_local_position_v1'
ATT = '/fmu/out/vehicle_attitude'
rng = random.Random(1)
NAN = float('nan')


def deliver(topic, msg):
    for cb in list(F.BUS.get(topic, [])):
        cb(msg)


def garbage():
    kind = rng.choice(['pos_nan', 'att_nan', 'att_zero', 'cloud_step', 'cloud_trunc',
                       'cloud_type', 'map_len', 'sect_nan', 'sect_empty', 'sect_inc0',
                       'pos_heading_nan'])
    PX4 = F.PX4
    if kind in ('pos_nan', 'pos_heading_nan'):
        m = PX4['VehicleLocalPosition'](xy_valid=True, z_valid=True)
        if kind == 'pos_nan':
            m.x = NAN
        else:
            m.heading = NAN
            m.x, m.y, m.z = (float(v) for v in world.vehicle.position)
        deliver(POS, m)
    elif kind == 'att_nan':
        deliver(ATT, PX4['VehicleAttitude'](q=[NAN, 0, 0, 1]))
    elif kind == 'att_zero':
        deliver(ATT, PX4['VehicleAttitude'](q=[0, 0, 0, 0]))
    elif kind.startswith('cloud'):
        fields = [F.PointField(name=n, offset=o, datatype=7, count=1)
                  for n, o in (('x', 0), ('y', 4), ('z', 8))]
        m = F.PointCloud2(width=100, height=1, point_step=12, fields=fields,
                          data=bytes(1200))
        if kind == 'cloud_step':
            m.point_step = 8
        elif kind == 'cloud_trunc':
            m.data = bytes(37)
        else:
            m.fields = [F.PointField(name='x', offset=0, datatype=99, count=1)] + fields[1:]
        m.header.stamp.sec = int(F.CLOCK.t)
        deliver(LIDAR, m)
    elif kind == 'map_len':
        m = F.OccupancyGrid()
        m.info.width, m.info.height, m.info.resolution = 10, 10, 0.1
        m.data = [0] * 37
        deliver('/map', m)
    else:
        m = F.MAP['ObstacleSectors']()
        m.header.stamp.sec = int(F.CLOCK.t)
        m.header.stamp.nanosec = int((F.CLOCK.t % 1) * 1e9)
        m.increment = 2 * math.pi / 72
        if kind == 'sect_nan':
            m.ranges = [NAN] * 72
        elif kind == 'sect_empty':
            m.ranges = []
        else:
            m.ranges = [5.0] * 72
            m.angle_offset, m.increment = 0.3, 0.0
        deliver('/obstacles/sectors', m)
    return kind


F.spin_for(2.0)
res = F.call('/mission/map' if kind == 'mapeo' else '/mission/start')
print('servicio:', res.success, res.message)
injected = {}
last = None
t0 = F.CLOCK.t
crash = None
try:
    while F.CLOCK.t - t0 < seconds:
        F.spin_for(1.0 / rate)
        k = garbage()
        injected[k] = injected.get(k, 0) + 1
        state = statuses[-1][1] if statuses else None
        if state != last:
            print(f'{F.CLOCK.t - t0:7.1f} s  {state:10s} {statuses[-1][2] if statuses else ""}')
            last = state
        if state in ('DONE', 'FAILED', 'PREEMPTED'):
            break
except Exception:  # noqa: BLE001
    import traceback
    crash = traceback.format_exc()

print('inyectado:', injected)
print('errores atrapados por nodo:',
      {type(n).__name__: {k: v for k, v in (n.callback_errors() if hasattr(n, "callback_errors") else {}).items() if v}
       for n in (world, mapper, mission)})
print('setpoints no finitos:', bad_setpoints[:3])
print('distancia mínima:', round(world.min_clearance, 2), 'choque:', world.collision_warned)
final = statuses[-1][1] if statuses else None
ok = crash is None and final == 'DONE' and not bad_setpoints and not world.collision_warned
print('CAIDA:\n' + crash if crash else '', 'RESULTADO:', 'PASA' if ok else 'FALLA', final)
sys.exit(0 if ok else 1)
