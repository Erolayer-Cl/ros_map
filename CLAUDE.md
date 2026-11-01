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

## Trabajo en curso: evasión para entornos reales

Pedido: buscar código de ejemplo de evasión que se use en vuelos reales, ver
qué consideran, y mejorar este stack con agentes asíncronos (investigación,
creación, análisis, QA).

Estado:
- Base: commit `be2db04` en `claude/ros2-px4-wall-detection-mapping-taoqhe`.
  El **push a GitHub falla con 403** (falta la GitHub App de Claude en
  `Erolayer-Cl/ros_map`); arreglarlo en https://claude.ai/connect-github y
  reintentar `git push -u origin claude/ros2-px4-wall-detection-mapping-taoqhe`.
- Ola 1 (en curso): investigación de planificadores reales (EGO-Planner,
  Fast-Planner/FUEL, FASTER, PX4-Avoidance, CMU, voxblox/nvblox,
  kr_autonomous_flight, aerostack2), investigación de autopilotos (PX4 v1.17
  CollisionPrevention, ArduPilot AC_Avoid/BendyRuler/Dijkstra/AP_Proximity) y
  lista de problemas del entorno real, análisis de brechas de este código, y QA
  adversarial (postes/cables finos, puerta angosta, ruido, polvo, latencia,
  inercia, viento, obstáculo dinámico, pérdida de sensor, cámara 87°).
- Ola 2 (pendiente): creación en una copia aislada (worktree) con las mejoras
  priorizadas + `docs/evasion-entornos-reales.md` con los ejemplos y lo que
  consideran; luego QA final con el arnés de la ola 1; integrar, `TODOS=1
  ./run_tests.sh`, commit y push.

Reportes de la sesión (scratchpad, efímero: copiar lo útil a `docs/` antes de
cerrar): `investigacion/01_codigo_planificadores.md`,
`investigacion/02_autopiloto_y_entorno_real.md`, `analisis/03_brechas.md`,
`qa/04_qa_inicial.md`, arnés en `qa/harness/run_all.py --repo <ruta>`.

Candidatos de mejora ya identificados (confirmar con los reportes):
distancia de frenado dependiente de la velocidad (v²/2a + v·t_latencia),
transformar cada nube con la pose de su timestamp (buffer de poses), filtro de
puntos espurios y de auto-impactos, olvido de obstáculos dinámicos en el mapa
y en los sectores, banda vertical según la actitud, retroceso sólo hacia
sectores vistos, altura mínima y geocerca.
