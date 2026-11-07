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
./run_tests.sh            # + 6 escenarios de lazo cerrado
TODOS=1 ./run_tests.sh    # los 14 escenarios (~3 min)
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
- Evasión en entornos reales (docs/evasion-entornos-reales.md):
  - Cada nube se transforma con la pose de su `header.stamp` (`pose_buffer.py`)
    y los sectores se recalculan contra la pose ACTUAL desde puntos guardados en
    NED. No volver a "última pose" ni a distancias relativas guardadas.
  - `ObstacleSectors.header.stamp` es la CAPTURA, no la publicación; la edad
    entra en el frenado. QoS de `/obstacles/sectors` BEST_EFFORT/KEEP_LAST 1 en
    los dos nodos (si uno queda RELIABLE no se conectan).
  - El corredor del frenado (`free_travel`) tiene el mismo ancho que el
    agrandado del VFH; con uno más ancho el dron se traba al bordear.
  - `avoid.emergency_distance` < `robot_radius + safety_margin`: si no, bordear
    dispara la huida. La emergencia se decide con `live_ranges` (no la memoria).
  - La zanahoria se recorta SÓLO en la dirección de avance (`_limit_along_lead`)
    y `_freeze()` sostiene donde empezó a frenar: recortar lo lateral o
    re-anclar en el dron cada ciclo deja al control sin autoridad contra el
    viento (choque en QA 6a).
  - El error de seguimiento cuenta sólo el pasarse por delante (y la altura):
    frenar por desvío lateral con viento lo empujaba a la pared.
  - Nube vacía = sensor ciego sólo si el campo es >= 180°: una cámara mirando a
    un espacio abierto está sana (si no, la misión queda en HOLD).
  - La grilla 2D va del piso a 1 m sobre el dron y el piso sube d·tan(2°): sin
    eso el techo tapa la grilla y el piso lejano con 1-2° de error de actitud se
    vuelve pared.
  - El A* mide la distancia en la grilla fina; en la gruesa las puertas < 2.4 m
    no pasaban.
  - Sectores no vistos van a PX4 como UINT16_MAX, nunca como libres.

## Trabajo en curso: evasión para entornos reales

Pedido: buscar código de ejemplo de evasión que se use en vuelos reales, ver
qué consideran, y mejorar este stack.

Estado:
- Base: commit `be2db04` en `claude/ros2-px4-wall-detection-mapping-taoqhe`.
  El **push a GitHub falla con 403** (falta la GitHub App de Claude en
  `Erolayer-Cl/ros_map`); arreglarlo en https://claude.ai/connect-github.
- Ola 1 (hecha): investigación, análisis de brechas y QA adversarial. Los
  reportes están en `docs/investigacion/` (01 planificadores, 02 autopilotos y
  entorno real, 03 brechas); el arnés de QA quedó en el scratchpad de la sesión
  (`qa/harness/run_all.py --repo <ruta>`), no en el repo.
- Ola 2 (hecha en la rama del worktree `worktree-agent-a8e8f2f339dc0c66f`):
  frenado físico, tiempos (historial de poses + edad del dato), franja relativa
  al dron, filtros de auto-impactos y ruido, sectores vistos/scaleToFOV,
  failsafes, mapper sin bloqueo, simulación de entorno real (5 escenarios
  nuevos), puertas de 2 m, y `docs/evasion-entornos-reales.md`.
- Pendiente: integrar la rama, QA final, probar con ROS 2 real / Unreal / vuelo.
  Lo no cubierto (vidrio, cables, deskew por punto → FAST-LIO2, evasión
  vertical, batería, predicción de dinámicos, mapeo con cámara: livelock entre
  fronteras en QA 9c) está en docs/evasion-entornos-reales.md §4.
