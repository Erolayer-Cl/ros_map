"""Corre fake_px4_world + mapper_node + mission_node juntos sobre el ROS de mentira."""
import os
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

statuses = []
F.BUS.setdefault('/mission/status', []).append(lambda m: statuses.append((F.CLOCK.t, m.state_name, m.message)))
F.spin_for(2.0)
res = F.call('/mission/map' if kind == 'mapeo' else '/mission/start')
print('servicio:', res.success, res.message)
last = None
t0 = F.CLOCK.t
while F.CLOCK.t - t0 < seconds:
    F.spin_for(1.0)
    state = statuses[-1][1] if statuses else None
    if state != last:
        print(f'{F.CLOCK.t - t0:7.1f} s  {state:10s} {statuses[-1][2] if statuses else ""}')
        last = state
    if state in ('DONE', 'FAILED', 'PREEMPTED'):
        break
print('distancia mínima a obstáculos:', round(world.min_clearance, 2), 'choque:', world.collision_warned)
walls_pub = [n for n in F.NODES if n is mapper][0]
print('paredes:', [(w.kind_name, round(w.length, 1)) for w in mapper.mapper.walls])
print('archivos guardados:', sorted(os.listdir(out_dir)))
errors = [l for n in F.NODES for l in n.get_logger().lines if l.startswith('[ERROR]')]
print('errores logueados:', errors[:5])
