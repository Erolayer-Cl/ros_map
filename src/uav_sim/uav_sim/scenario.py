#!/usr/bin/env python3
"""Escenarios de prueba: un mismo YAML para el simulador offline y para Unreal.

Python puro (PyYAML + numpy). Lo usan:

- `world.py` / `closed_loop.py`: ray casting del lidar contra estas primitivas.
- `fake_px4_world.py` (nodo ROS): el mismo mundo, publicado como si fuera AirSim.
- `spawn_unreal.py`: crea exactamente estas mismas cajas y cilindros en Unreal
  con la API de Cosys-AirSim, para comparar el mapa del simulador offline con
  el de Unreal sobre la misma geometría.

Coordenadas: norte/este en metros **relativos al punto de despegue** (donde
está el PlayerStart / donde arranca el EKF de PX4), alturas positivas hacia
arriba sobre el piso. Ángulos en grados, `yaw` positivo hacia el Este (horario
visto desde arriba), como todo lo demás en NED.

Tipos de objeto:

```yaml
- tipo: caja             # pared plana, columna, cubo
  centro: [12.0, 0.0]    # norte, este
  tamano: [0.3, 16.0, 5.0]   # largo (eje local x), ancho (eje local y), alto
  yaw: 0                 # rotación sobre la vertical
  inclinacion: 0         # rotación sobre el eje local x (pared inclinada)
  base: 0.0              # altura de la base sobre el piso
- tipo: cilindro
  centro: [5.0, 5.0]
  radio: 0.4
  alto: 5.0
- tipo: pared            # pared recta entre dos puntos (azúcar sobre "caja")
  desde: [10, -8]
  hasta: [10, 8]
  alto: 5
  espesor: 0.3
  inclinacion: 0
- tipo: pared_irregular  # cadena de bloques con salientes al azar (semilla fija)
  desde: [10, -8]
  hasta: [10, 8]
  alto: 5
  espesor: 0.4
  rugosidad: 0.6         # m de amplitud de las salientes
  tramo: 1.0             # largo de cada bloque
  semilla: 7
- tipo: pared_dentada    # zig-zag regular
  desde: [...]
  hasta: [...]
  amplitud: 0.6
  periodo: 2.0
- tipo: pared_curva      # arco de bloques
  centro: [0, 0]
  radio: 8
  desde_deg: -40         # rumbo NED de los extremos del arco
  hasta_deg: 40
```

Un techo es una `caja` delgada con `base` = altura del techo.

Bloque opcional `simulacion:` (sólo `closed_loop`/`fake_px4_world`; Unreal
no lo usa): condiciones de entorno real para probar la evasión.

```yaml
simulacion:
  latencia_nube: 0.15        # s entre la captura y la entrega de la nube
  cabeceo: true              # actitud por la aceleración (el lidar ve el piso al frenar)
  ruido: 0.03                # desvío del rango (m)
  polvo: {puntos: 3, distancia: [0.6, 5.0]}   # retornos sueltos por nube
  auto_impactos: [[0.32, 0.32, 0.05], ...]    # puntos del propio dron (FRD, m)
  viento: {velocidad: [0.0, 1.0], rafagas: 0.8, periodo: 6.0}   # deriva (m/s)
  lidar: {mount_rpy_deg: [0, -20, 0]}        # campos de LidarModel
  mapper: {min_range: 0.4}                   # campos de MapperConfig
  dinamicos:                                 # cajas que aparecen o se mueven
    - tamano: [0.6, 4.0, 5.0]
      aparece: {cuando_norte_mayor: 4.0}     # o {t: 12.0} (s desde el arranque)
      trayectoria: [[0, 10.0, 0.0], [5, 10.0, 3.0]]   # [t relativo, norte, este]
```
"""

import math
import os
from dataclasses import dataclass

import numpy as np

try:
    import yaml
except ImportError:  # pragma: no cover - yaml viene con ROS; en Windows: pip install pyyaml
    yaml = None


@dataclass
class Box:
    """Paralelepípedo orientado. `center` NED (n, e, d) del centro geométrico."""

    name: str
    center: np.ndarray
    size: np.ndarray        # (largo x, ancho y, alto z) en ejes locales
    yaw: float              # rad
    tilt: float             # rad, giro sobre el eje local x
    kind: str = 'caja'

    def rotation(self):
        """Local -> NED: primero inclinación (roll local), después yaw."""
        cy, sy = math.cos(self.yaw), math.sin(self.yaw)
        cr, sr = math.cos(self.tilt), math.sin(self.tilt)
        r_yaw = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]])
        r_roll = np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]])
        return r_yaw @ r_roll


@dataclass
class Cylinder:
    name: str
    center_ne: np.ndarray
    radius: float
    bottom_d: float         # D de la base (más positivo = más abajo)
    top_d: float
    kind: str = 'cilindro'


@dataclass
class Scenario:
    name: str
    description: str
    boxes: list
    cylinders: list
    mission: dict
    expected: dict
    ground: bool = True
    simulation: dict = None    # bloque `simulacion:` (condiciones de entorno real)


def _box_from_wall(name, start, end, height, thickness, base=0.0, tilt_deg=0.0, kind='pared'):
    start = np.asarray(start, dtype=float)
    end = np.asarray(end, dtype=float)
    mid = (start + end) / 2.0
    length = float(np.hypot(*(end - start)))
    yaw = math.atan2(end[1] - start[1], end[0] - start[0])
    tilt = math.radians(tilt_deg)
    # Con inclinación, el centro se corre para que la base siga apoyada en el
    # piso sobre la línea indicada (la pared "cae" hacia un costado).
    # La rotación lleva el "arriba" local hacia +normal cuando tilt > 0, así
    # que el centro queda corrido hacia ese lado la mitad de la proyección.
    lateral = math.sin(tilt) * height / 2.0
    normal = np.array([-math.sin(yaw), math.cos(yaw)])
    center_ne = mid + normal * lateral
    center_d = -(base + math.cos(tilt) * height / 2.0)
    return Box(name=name, center=np.array([center_ne[0], center_ne[1], center_d]),
               size=np.array([length, thickness, height]), yaw=yaw, tilt=tilt, kind=kind)


def expand_objects(objects):
    """Lista de objetos del YAML -> (cajas, cilindros) primitivos."""
    boxes, cylinders = [], []
    for index, obj in enumerate(objects or []):
        kind = obj.get('tipo', 'caja')
        name = obj.get('nombre', f'{kind}_{index}')
        if kind == 'caja':
            size = [float(v) for v in obj['tamano']]
            base = float(obj.get('base', 0.0))
            n, e = (float(v) for v in obj['centro'])
            boxes.append(Box(name=name, center=np.array([n, e, -(base + size[2] / 2.0)]),
                             size=np.array(size), yaw=math.radians(float(obj.get('yaw', 0.0))),
                             tilt=math.radians(float(obj.get('inclinacion', 0.0)))))
        elif kind == 'cilindro':
            n, e = (float(v) for v in obj['centro'])
            base = float(obj.get('base', 0.0))
            height = float(obj['alto'])
            cylinders.append(Cylinder(name=name, center_ne=np.array([n, e]),
                                      radius=float(obj['radio']), bottom_d=-base,
                                      top_d=-(base + height)))
        elif kind == 'pared':
            boxes.append(_box_from_wall(name, obj['desde'], obj['hasta'], float(obj['alto']),
                                        float(obj.get('espesor', 0.3)), float(obj.get('base', 0.0)),
                                        float(obj.get('inclinacion', 0.0))))
        elif kind == 'pared_irregular':
            boxes.extend(_irregular_wall(name, obj))
        elif kind == 'pared_dentada':
            boxes.extend(_zigzag_wall(name, obj))
        elif kind == 'pared_curva':
            boxes.extend(_curved_wall(name, obj))
        else:
            raise ValueError(f'tipo de objeto desconocido en el escenario: {kind!r}')
    return boxes, cylinders


def _irregular_wall(name, obj):
    """Bloques consecutivos con desplazamiento, giro y altura al azar.

    Imita una pared de escombros / roca / tablas: la cara que ve el lidar no es
    una recta. La semilla fija hace que Unreal y el simulador offline tengan
    exactamente la misma pared.
    """
    rng = np.random.default_rng(int(obj.get('semilla', 0)))
    start = np.asarray(obj['desde'], dtype=float)
    end = np.asarray(obj['hasta'], dtype=float)
    height = float(obj['alto'])
    thickness = float(obj.get('espesor', 0.4))
    rough = float(obj.get('rugosidad', 0.5))
    piece = float(obj.get('tramo', 1.0))
    axis = end - start
    length = float(np.hypot(*axis))
    unit = axis / max(length, 1e-9)
    normal = np.array([-unit[1], unit[0]])
    yaw = math.atan2(unit[1], unit[0])
    count = max(1, int(math.ceil(length / piece)))
    boxes = []
    for k in range(count):
        t = (k + 0.5) * length / count
        offset = rng.uniform(-rough, rough)
        jitter_yaw = math.radians(rng.uniform(-20.0, 20.0))
        h = height * rng.uniform(0.75, 1.0)
        center = start + unit * t + normal * offset
        boxes.append(Box(name=f'{name}_{k}', center=np.array([center[0], center[1], -h / 2.0]),
                         size=np.array([length / count * 1.15, thickness + abs(offset) * 0.5, h]),
                         yaw=yaw + jitter_yaw, tilt=0.0, kind='pared_irregular'))
    return boxes


def _zigzag_wall(name, obj):
    start = np.asarray(obj['desde'], dtype=float)
    end = np.asarray(obj['hasta'], dtype=float)
    amp = float(obj.get('amplitud', 0.6))
    period = float(obj.get('periodo', 2.0))
    height = float(obj['alto'])
    thickness = float(obj.get('espesor', 0.25))
    axis = end - start
    length = float(np.hypot(*axis))
    unit = axis / max(length, 1e-9)
    normal = np.array([-unit[1], unit[0]])
    half = period / 2.0
    count = max(1, int(round(length / half)))
    boxes = []
    for k in range(count):
        a = start + unit * (k * half) + normal * (amp if k % 2 else -amp)
        b = start + unit * ((k + 1) * half) + normal * (-amp if k % 2 else amp)
        box = _box_from_wall(f'{name}_{k}', a, b, height, thickness, kind='pared_dentada')
        box.size[0] += thickness   # solapar en los vértices para que no queden rendijas
        boxes.append(box)
    return boxes


def _curved_wall(name, obj):
    center = np.asarray(obj['centro'], dtype=float)
    radius = float(obj['radio'])
    a0 = math.radians(float(obj['desde_deg']))
    a1 = math.radians(float(obj['hasta_deg']))
    height = float(obj['alto'])
    thickness = float(obj.get('espesor', 0.3))
    segment = float(obj.get('tramo', 0.8))
    count = max(2, int(math.ceil(abs(a1 - a0) * radius / segment)))
    angles = np.linspace(a0, a1, count + 1)
    boxes = []
    for k in range(count):
        p = center + radius * np.array([math.cos(angles[k]), math.sin(angles[k])])
        q = center + radius * np.array([math.cos(angles[k + 1]), math.sin(angles[k + 1])])
        box = _box_from_wall(f'{name}_{k}', p, q, height, thickness, kind='pared_curva')
        box.size[0] += thickness * 0.5
        boxes.append(box)
    return boxes


def scenarios_dir():
    """Carpeta de escenarios: la instalada por colcon o la del código fuente."""
    try:
        from ament_index_python.packages import get_package_share_directory
        path = os.path.join(get_package_share_directory('uav_sim'), 'scenarios')
        if os.path.isdir(path):
            return path
    except Exception:  # noqa: BLE001 - sin ROS, o paquete sin instalar
        pass
    return os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'scenarios')


def list_scenarios():
    folder = scenarios_dir()
    return sorted(f[:-5] for f in os.listdir(folder) if f.endswith('.yaml'))


def load_scenario(name_or_path):
    """Carga un escenario por nombre (`01_pared_plana`) o por ruta a un .yaml."""
    if yaml is None:
        raise RuntimeError('falta PyYAML (sudo apt install python3-yaml / pip install pyyaml)')
    path = name_or_path
    if not os.path.isfile(path):
        candidate = os.path.join(scenarios_dir(), name_or_path)
        path = candidate if candidate.endswith('.yaml') else candidate + '.yaml'
    with open(path, 'r', encoding='utf-8') as fh:
        data = yaml.safe_load(fh) or {}
    boxes, cylinders = expand_objects(data.get('objetos', []))
    return Scenario(name=data.get('nombre', os.path.splitext(os.path.basename(path))[0]),
                    description=data.get('descripcion', ''), boxes=boxes, cylinders=cylinders,
                    mission=data.get('mision', {}) or {}, expected=data.get('esperado', {}) or {},
                    ground=bool(data.get('piso', True)),
                    simulation=data.get('simulacion', {}) or {})


def mission_overrides(scenario, mission):
    """Parámetros de mission_node que fija el escenario (área, waypoints, altura).

    Lo usan la simulación offline y los launch de ROS, para que la misión
    coincida con la geometría que se creó en Unreal / en el mundo falso.
    """
    m = scenario.mission or {}
    out = {}
    if mission == 'waypoints' and 'waypoints' in m:
        altitude = float(m.get('altura', 3.0))
        flat = []
        for wp in m['waypoints']:
            wp = [float(v) for v in wp]
            flat.extend(wp if len(wp) == 3 else wp[:2] + [altitude])
        out['waypoints'] = flat
        out['takeoff_altitude'] = altitude
    if mission == 'mapeo':
        if 'area' in m:
            out['map_mission.area'] = [float(v) for v in m['area']]
        if 'altura' in m:
            out['map_mission.altitude'] = float(m['altura'])
        if 'estrategia' in m:
            out['map_mission.strategy'] = str(m['estrategia'])
    for key, value in (m.get('parametros') or {}).items():
        out[key] = value
    return out
