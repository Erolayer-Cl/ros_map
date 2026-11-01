#!/usr/bin/env python3
"""Crea en Unreal (Cosys-AirSim) las paredes de un escenario de prueba.

Lee el mismo YAML que usa el simulador offline y pone en el mundo, con la API
`simSpawnObject` de Cosys-AirSim, una caja escalada por cada pared/columna y un
cilindro por cada cilindro. Así se compara el mapa que dibuja el dron en Unreal
con el que dibuja en la simulación offline, sobre la misma geometría.

Uso (Unreal en Play; desde WSL2 o desde Windows con `pip install cosysairsim pyyaml`):

    ros2 run uav_sim spawn_scenario --escenario 06_habitacion
    ros2 run uav_sim spawn_scenario --escenario 06_habitacion --limpiar   # borra y vuelve a crear
    ros2 run uav_sim spawn_scenario --solo-limpiar                          # borra lo creado
    ros2 run uav_sim spawn_scenario --escenario 02_pared_irregular --mostrar
        # sin conectarse: lista cada pieza con Location/Rotation/Scale en unidades
        # de Unreal, para ponerlas a mano en el editor si se prefiere

Coordenadas: el escenario es NED relativo al punto de despegue. AirSim pone el
origen de su NED "global" en el PlayerStart, que es donde aparece el dron, así
que con la configuración por defecto coinciden. Si el vehículo tiene X/Y/Z
propios en settings.json, pasarlos con `--offset N E D`.

Cada objeto se "entierra" `--enterrar` metros bajo el piso: si el piso real no
está exactamente donde se calculó, la pared igual queda apoyada y sin rendija.
"""

import argparse
import math
import sys

from uav_sim.scenario import list_scenarios, load_scenario

CUBE_CANDIDATES = ['Cube', '1M_Cube', '1M_Cube_Chamfer', 'Shape_Cube', 'SM_Cube', 'TemplateCube_Rounded']
CYLINDER_CANDIDATES = ['Cylinder', 'Shape_Cylinder', 'SM_Cylinder', '1M_Cylinder']


def _import_airsim():
    try:
        import cosysairsim as airsim
        return airsim
    except ImportError:
        pass
    try:
        import airsim
        return airsim
    except ImportError:
        sys.exit('falta el cliente de Cosys-AirSim: pip install cosysairsim  (o '
                 'pip install -e dependencias/Cosys-AirSim/PythonClient)')


def _pick_asset(assets, wanted, candidates, kind):
    if wanted:
        if wanted not in assets:
            print(f'AVISO: el asset {wanted!r} no figura en simListAssets(); se intenta igual')
        return wanted
    for name in candidates:
        if name in assets:
            return name
    lowered = {a.lower(): a for a in assets}
    for name in candidates:
        for key, original in lowered.items():
            if name.lower() in key:
                return original
    sys.exit(f'no encontré un asset de {kind} en el proyecto. Assets disponibles (primeros 40): '
             f'{assets[:40]}. Pasalo con --asset-{kind}. En un proyecto en blanco, agregá el '
             'contenido de inicio (Starter Content) o usá el mapa Blocks de Cosys-AirSim.')


def _quaternion(roll, pitch, yaw):
    """[w, x, y, z] de una rotación ZYX (la convención de AirSim y PX4)."""
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (cy * cr * cp + sy * sr * sp, cy * sr * cp - sy * cr * sp,
            cy * cr * sp + sy * sr * cp, sy * cr * cp - cy * sr * sp)


def primitives(scenario, offset=(0.0, 0.0, 0.0), ground_d=0.0, bury=0.5):
    """(nombre, tipo, centro NED mundo, tamaño xyz m, yaw rad, inclinación rad)."""
    out = []
    for box in scenario.boxes:
        size = [float(v) for v in box.size]
        center = [float(v) for v in box.center]
        # Alargar hacia abajo para enterrar la base (en ejes locales, z = alto).
        size[2] += bury
        center[2] += bury / 2.0 * math.cos(box.tilt)
        lateral = -math.sin(box.tilt) * bury / 2.0
        center[0] += -math.sin(box.yaw) * lateral
        center[1] += math.cos(box.yaw) * lateral
        world = (center[0] + offset[0], center[1] + offset[1], center[2] + ground_d + offset[2])
        out.append((box.name, 'caja', world, size, box.yaw, box.tilt))
    for cyl in scenario.cylinders:
        height = cyl.bottom_d - cyl.top_d + bury
        center_d = (cyl.top_d + cyl.bottom_d + bury) / 2.0
        world = (float(cyl.center_ne[0]) + offset[0], float(cyl.center_ne[1]) + offset[1],
                 center_d + ground_d + offset[2])
        out.append((cyl.name, 'cilindro', world, [2 * cyl.radius, 2 * cyl.radius, height], 0.0, 0.0))
    return out


def show(items):
    """Tabla para poner las piezas a mano en el editor de Unreal.

    Unreal: centímetros, X = norte, Y = este, Z = ARRIBA, relativos al
    PlayerStart. Escala para el Cube de /Engine/BasicShapes (100 cm de lado,
    pivote en el centro).
    """
    print(f'{"nombre":28s} {"tipo":9s} {"Location X,Y,Z (cm)":>28s}  {"Rotation R,P,Y (°)":>20s}  '
          f'{"Scale X,Y,Z":>18s}')
    for name, kind, (n, e, d), size, yaw, tilt in items:
        loc = f'{n * 100:8.0f},{e * 100:8.0f},{-d * 100:8.0f}'
        rot = f'{math.degrees(tilt):5.1f},{0.0:5.1f},{math.degrees(yaw):6.1f}'
        scale = f'{size[0]:5.2f},{size[1]:5.2f},{size[2]:5.2f}'
        print(f'{name:28s} {kind:9s} {loc:>28s}  {rot:>20s}  {scale:>18s}')


def main(argv=None):
    parser = argparse.ArgumentParser(description='Crea en Unreal las paredes de un escenario.')
    parser.add_argument('--escenario', help=f'disponibles: {", ".join(list_scenarios())}')
    parser.add_argument('--host', default='localhost')
    parser.add_argument('--puerto', type=int, default=41451)
    parser.add_argument('--prefijo', default='rosmap_')
    parser.add_argument('--limpiar', action='store_true', help='borrar antes lo creado con el prefijo')
    parser.add_argument('--solo-limpiar', action='store_true')
    parser.add_argument('--mostrar', action='store_true', help='sólo listar las piezas, sin conectarse')
    parser.add_argument('--asset-caja', default='')
    parser.add_argument('--asset-cilindro', default='')
    parser.add_argument('--lado-caja', type=float, default=1.0, help='m de lado del asset de caja')
    parser.add_argument('--lado-cilindro', type=float, default=1.0, help='m de diámetro y alto del asset')
    parser.add_argument('--pivote', choices=('centro', 'base'), default='centro',
                        help='dónde tiene el pivote el asset (Engine/BasicShapes: centro)')
    parser.add_argument('--offset', type=float, nargs=3, default=(0.0, 0.0, 0.0),
                        metavar=('N', 'E', 'D'), help='despegue respecto del PlayerStart (m, NED)')
    parser.add_argument('--suelo-d', type=float, default=None,
                        help='D del piso en el NED de AirSim (por defecto se estima con el dron)')
    parser.add_argument('--alto-dron', type=float, default=0.12,
                        help='altura del centro del dron sobre el piso cuando está apoyado')
    parser.add_argument('--enterrar', type=float, default=0.5)
    args = parser.parse_args(argv)

    if args.mostrar:
        if not args.escenario:
            parser.error('--mostrar necesita --escenario')
        scenario = load_scenario(args.escenario)
        print(f'# {scenario.name}: {scenario.description}')
        show(primitives(scenario, args.offset, 0.0, args.enterrar))
        return 0

    airsim = _import_airsim()
    client = airsim.VehicleClient(ip=args.host, port=args.puerto)
    client.confirmConnection()

    if args.limpiar or args.solo_limpiar:
        existing = client.simListSceneObjects(f'{args.prefijo}.*')
        for name in existing:
            client.simDestroyObject(name)
        print(f'borrados {len(existing)} objetos con prefijo {args.prefijo!r}')
        if args.solo_limpiar:
            return 0
    if not args.escenario:
        parser.error('falta --escenario')

    scenario = load_scenario(args.escenario)
    if args.suelo_d is None:
        pose = client.simGetVehiclePose()
        ground_d = float(pose.position.z_val) + args.alto_dron
        print(f'piso estimado con la pose del dron: D = {ground_d:.2f} m (usar --suelo-d para fijarlo)')
    else:
        ground_d = args.suelo_d

    assets = client.simListAssets()
    cube = _pick_asset(assets, args.asset_caja, CUBE_CANDIDATES, 'caja')
    cylinder = _pick_asset(assets, args.asset_cilindro, CYLINDER_CANDIDATES, 'cilindro') \
        if scenario.cylinders else None
    print(f'assets: caja={cube!r} cilindro={cylinder!r}')

    items = primitives(scenario, args.offset, ground_d, args.enterrar)
    created = 0
    for name, kind, (n, e, d), size, yaw, tilt in items:
        base = args.lado_caja if kind == 'caja' else args.lado_cilindro
        if args.pivote == 'base':
            d += size[2] / 2.0        # el asset crece hacia arriba desde su pivote
        # Cuaternión ZYX (yaw, luego roll = inclinación) armado a mano: Cosys-AirSim
        # trae euler_to_quaternion(roll, pitch, yaw) y el AirSim original
        # to_quaternion(pitch, roll, yaw); así no depende de cuál esté instalado.
        qw, qx, qy, qz = _quaternion(tilt, 0.0, yaw)
        pose = airsim.Pose(airsim.Vector3r(n, e, d), airsim.Quaternionr(qx, qy, qz, qw))
        scale = airsim.Vector3r(size[0] / base, size[1] / base, size[2] / base)
        asset = cube if kind == 'caja' else cylinder
        real_name = client.simSpawnObject(f'{args.prefijo}{name}', asset, pose, scale, False, False)
        created += 1
        print(f'  {real_name}: {kind} en N={n:.1f} E={e:.1f} D={d:.1f}, '
              f'{size[0]:.2f}x{size[1]:.2f}x{size[2]:.2f} m')
    print(f'listo: {created} objetos del escenario {scenario.name}. Misión sugerida: '
          f'{scenario.mission.get("tipo", "mapeo")}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
