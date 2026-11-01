# CLAUDE.md

Guía para trabajar con código de este repositorio.

## Qué es

Workspace ROS 2 (Humble; también compila en Jazzy) para un dron con PX4 v1.17
por uXRCE-DDS (sin MAVROS): detecta paredes planas e irregulares con un lidar 3D
o cámara de profundidad, las esquiva (VFH+ + A* sobre el mapa) y tiene una
misión de mapeo (`/mission/map`) que explora un área y guarda el plano con las
paredes clasificadas. Tomado como referencia: `Laboratorio-VNT/UAV_ros2-beta`.

Comentarios, docstrings, logs y docs en español; identificadores en inglés.

## Probar

```sh
./run_tests.sh rapido     # unitarias, segundos, sin ROS (python3 + numpy + pyyaml)
./run_tests.sh            # + 5 escenarios de lazo cerrado
TODOS=1 ./run_tests.sh    # los 9 escenarios (~3 min)
PYTHONPATH=src/uav_wall_mapping:src/uav_mission:src/uav_sim python3 -m uav_sim.closed_loop --escenario 06_habitacion -v
```

Con ROS: `colcon build --symlink-install`, `./launch_fake_sim.sh <escenario>`.
Después de cambiar un `.msg`: `rm -rf build/uav_map_msgs install/uav_map_msgs`.

## Arquitectura (ver docs/arquitectura.md)

- **Toda la lógica es Python puro** (sin rclpy): `mapper_core`, `occupancy`,
  `walls`, `sectors`, `export`, `geometry`, `cloud` (uav_wall_mapping) y
  `mission_core`, `avoidance`, `planner`, `exploration` (uav_mission). Los nodos
  (`mapper_node`, `mission_node`, `fake_px4_world`) son adaptadores. Mantenerlo
  así: es lo que permite `closed_loop.py` y las pruebas sin ROS.
- `mission_node` es el **único** que publica en `/fmu/in/trajectory_setpoint`,
  `offboard_control_mode` y `vehicle_command`. El A* corre en otro hilo; el lazo
  de control (20 Hz) nunca debe bloquearse ni dejar escapar excepciones.
- NED/FRD adentro; ENU sólo en lo publicado para visualizar. `waypoints` y
  `map_mission.area` son relativos al despegue.
- PX4 v1.17: `/fmu/out/vehicle_local_position_v1` y `/fmu/out/vehicle_status_v1`
  (MESSAGE_VERSION = 1 → sufijo `_v1`). QoS BEST_EFFORT para `/fmu/out/*`.
- Los escenarios YAML de `src/uav_sim/scenarios/` son la única fuente de
  geometría para offline, `fake_px4_world` y `spawn_unreal.py`. Cada uno tiene un
  bloque `esperado:` que verifica `test_closed_loop.py`.

## Trampas conocidas (no deshacer)

- Tallado de espacio libre: los sectores vacíos se limitan por los vecinos y los
  rayos rasantes se acortan; si no, las paredes se borran (ver `sectors.py`).
- Celdas ocupadas se vacían más lento (`occupied_miss_scale`): objetos bajos.
- La planitud se mide **por cara** usando el lado de observación guardado en
  grilla y vóxeles; sin eso una pared gruesa vista de ambos lados sale
  irregular y una dentada sale plana.
- `free_ray_bins` no puede ser más fino que la resolución horizontal del sensor.
- Debounce en todas las decisiones de evasión (lección de la referencia).
- `cloud.mount_*` tiene que coincidir con el `settings.json` de AirSim o con el
  montaje real: un error de signo espeja el mapa.
