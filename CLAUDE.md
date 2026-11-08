# CLAUDE.md

Guía para trabajar con código de este repositorio.

## Qué es

Workspace ROS 2 (Humble; también compila en Jazzy) para un dron con PX4 v1.17
por uXRCE-DDS (sin MAVROS): detecta paredes planas e irregulares con un lidar 3D
o cámara de profundidad, las esquiva (VFH+ + A* sobre el mapa) y tiene una
misión de mapeo (`/mission/map`) que explora un área y guarda el plano con las
paredes clasificadas. Tomado como referencia: `Laboratorio-VNT/UAV_ros2-beta`.

Comentarios, docstrings, logs y docs en español; identificadores en inglés.

## Hardware del dron real (lo que se sabe, 2026-11)

Fuentes: el usuario + sus repos `UAVinformaticaignacio/UAV_wiki` (vault de
Obsidian), `UAVinformaticaignacio/Jetson_pruebas` y la organización
`Laboratorio-VNT/UAV_ros2-beta` (ramas `main`, `chore/team-setup`,
`feat/yolo-y-mision`). Ninguno trae una lista de componentes con modelos.

- **Cámaras: una mirando al frente y una mirando abajo** (dicho por el usuario).
  Modelos sin definir (el usuario todavía no eligió la frontal). Recomendado:
  frontal **RealSense D455** (o Orbbec Gemini 336L), ambas con nube en marco
  óptico → `mapper_realsense.yaml` sirve tal cual salvo tópico y montaje;
  abajo, para posición sin GPS, un sensor de flujo óptico con telémetro
  (Holybro H-Flow / ARK Flow, DroneCAN a la 6C) antes que una cámara.
- **No hay lidar.** Ningún repo lo nombra como componente: la wiki sólo dice
  "sensores (GPS, cámara, lidar)" de forma genérica, y la rama de la org esquiva
  con 5 sensores `Distance` (rangefinders 1D) de AirSim + cámara frontal para YOLO.
- Computadora de compañía: **Jetson Orin NX** con **JetPack 6 (Ubuntu 22.04) +
  ROS 2 Humble nativo** por apt (Python 3.10). El repo `Jetson_pruebas` es de
  una Jetson Nano anterior (JetPack 4.6, Docker `dustynv`): no aplica.
- Controladora: **Pixhawk 6C con PX4 v1.17** (dicho por el usuario).
  uXRCE-DDS por **TELEM2** (`UXRCE_DDS_CFG` 102, `SER_TEL2_BAUD` 921600) a un
  UART de la Jetson o adaptador USB-serie. La 6C **no tiene Ethernet** (eso es
  la 6X): sólo serie. `px4_msgs` en `release/1.17` ya coincide.

Consecuencias para este stack:
- El mapeo y la evasión necesitan **puntos 3D**. Una cámara RGB sola no sirve:
  la frontal tiene que ser **de profundidad** (RealSense D435i/D455, OAK-D,
  Orbbec) → usar `mapper_realsense.yaml` (FOV ~87°, `no_hit_free_range` 3 m).
  Con una webcam común sólo queda YOLO, no paredes.
- Con cámara frontal el dron ve ~87°: hay que girar para ver los costados (la
  exploración ya lo hace) y no hay nada atrás. Considerar 2-4 rangefinders
  laterales/traseros a `/fmu/in/obstacle_distance` o como sectores extra.
- La cámara de abajo no entra al mapper (la franja de altura descarta el piso):
  sirve para flujo óptico (posición sin GPS en interiores, con un telémetro
  hacia abajo) y aterrizaje de precisión/ArUco. En interiores sin GPS **la
  posición es requisito** (docs/sensores.md §4).
- Orin NX: sobra para este stack (Python + numpy) con la cámara de profundidad
  a 10-15 Hz, y deja margen para YOLO (TensorRT) sobre la cámara frontal o
  VIO/flujo óptico con la de abajo. Modo de energía máximo (`sudo nvpmodel -m 0`,
  `sudo jetson_clocks`) para que el mapper no tenga picos de latencia.
- UART con la Pixhawk: el del header de 40 pines (`ls /dev/ttyTHS*`); quitar la
  consola serie de ese puerto si la tiene (`systemctl disable nvgetty`) y
  agregar el usuario al grupo `dialout`.

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
  - Los rayos "sin eco" (sectores vacíos tallados hasta `no_hit_free_range`)
    son evidencia débil: no vacían celdas con impactos en los últimos
    `weak_protect_sec` (2 s). Sin esto un cable fino visto de vez en cuando se
    borraba entre detecciones y el dron chocaba (QA 1d); protegerlas para
    siempre deja fantasmas de lo que se mueve (QA 7a/7c/7d).
  - No agregar histéresis al debounce del desvío: se probó para el cable y
    produjo un choque con ráfagas (QA 6e).
  - `no_hit_free_range` del lidar se queda en 6 m: con 15 m se borraba el rastro
    de lo que se movió (QA 7b) pero aparecieron 2 choques (1f, 5f).
  - El espacio por el que pasó el dron se da por conocido (`traversed_radius`,
    sólo celdas desconocidas, valor 50 % para no crear fronteras): con una cámara
    lo pegado al dron nunca se ve y la exploración volvía a esas "fronteras".
  - Fronteras: libre = conocido y < 50 % (con cámara los bordes se ven una vez y
    quedan en ~40 %). Una frontera elegida 3 veces en el mismo lugar se descarta.
  - Paso angosto: el A* marca `narrow` sólo si el camino pasa entre obstáculos
    A AMBOS LADOS a menos de radio + margen del VFH (marcarlo junto a una sola
    pared bajaba los márgenes en la trampa en U). En ese modo el VFH usa margen
    chico y la prueba de corredor, con histéresis de `plan.narrow_hold`.

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
- Ola 2 (hecha e integrada en la rama): frenado físico, tiempos (historial de
  poses + edad del dato), franja relativa al dron, filtros de auto-impactos y
  ruido, sectores vistos/scaleToFOV, failsafes, mapper sin bloqueo, simulación
  de entorno real (5 escenarios nuevos), puertas de 2 m, rayos sin eco como
  evidencia débil, y `docs/evasion-entornos-reales.md`.
- Revisión con pruebas y arreglos: `TODOS=1 ./run_tests.sh` OK (84 unitarias
  + 14 escenarios). Arnés de QA ahora en el repo (`qa/`, ver `qa/README.md`):
  61/65 sin falla y 0 choques (original: 39/65 y 5 choques). Arreglado en la
  última ronda: puerta de 1.5 m (modo de paso angosto), mapeo con cámara que
  quedaba en un ciclo o a medias (espacio recorrido, umbral de frontera,
  límite de repeticiones, `no_hit_free_range` 3 m en la config de cámara).
  Límites conocidos: puertas de 1.2 m (2a/2f), rastro de una caja que se fue
  lejos del camino (7b), cámara configurada como lidar (9d, lenta).
- Pendiente: probar con ROS 2 real / Unreal / vuelo.
  Lo no cubierto (vidrio, cables, deskew por punto → FAST-LIO2, evasión
  vertical, batería, predicción de dinámicos, mapeo con cámara: livelock entre
  fronteras en QA 9c) está en docs/evasion-entornos-reales.md §4.
