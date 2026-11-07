# Evasión de obstáculos en entornos reales

Qué hacen los planificadores y autopilotos que vuelan de verdad para que la
evasión funcione fuera de la simulación, qué adoptamos acá (con qué parámetro)
y qué **no** cubrimos. Los reportes completos, con archivo y línea de cada
cita, están en [docs/investigacion/](investigacion/):

- [01_codigo_planificadores.md](investigacion/01_codigo_planificadores.md):
  EGO-Planner, Fast-Planner/FUEL, FASTER/MADER/PANTHER, PX4-Avoidance, CMU,
  voxblox/nvblox, KR autonomous flight, aerostack2.
- [02_autopiloto_y_entorno_real.md](investigacion/02_autopiloto_y_entorno_real.md):
  PX4 v1.17 Collision Prevention, ArduPilot AC_Avoid / AP_Proximity /
  BendyRuler / Dijkstra, ejemplos ROS 2, lista de verificación de entorno real.
- [03_brechas.md](investigacion/03_brechas.md): las 18 brechas de este
  código (con archivo:línea) y los experimentos que las reproducen.

El resumen: el stack funcionaba en un mundo sin latencia, con la pose exacta
en el instante de la nube, sin cabeceo, sin ruido, sin techo y sin el propio
dron en la nube. Fuera de ese mundo chocaba o abortaba sola la misión. Cada
apartado de abajo dice qué lo resolvía en los proyectos reales y cómo quedó
acá.

---

## 1. Lo que hacen los proyectos que vuelan

| Consideración | Proyecto · archivo | Qué hace | Valor típico |
|---|---|---|---|
| Distancia de frenado | PX4 v1.17 `src/lib/collision_prevention/CollisionPrevention.cpp` (`_getVelocityCompensationAcceleration`), `TrajMath.hpp` (`computeMaxSpeedFromDistance`) | Por sector: `v_max` para frenar en `d − CP_DIST − v·(CP_DELAY + edad del dato)` con aceleración y jerk máximos | `CP_DELAY` 0.4 s, `MPC_ACC_HOR` 3 m/s², `MPC_JERK_MAX` 8 m/s³ |
| Velocidad segura | ArduPilot `libraries/AC_Avoidance/AC_Avoid.cpp` (`get_max_speed`, `AC_AVOID_ACCEL_CMSS_MAX`) | `v_max = √(2·a·d)` con `a ≤ 1 m/s²` para la evasión | 1 m/s² |
| Edad del dato / timeout | PX4 CP (`RANGE_STREAM_TIMEOUT_US`, `TIMEOUT_HOLD_US`); ArduPilot `AP_Proximity_Boundary_3D` | Cada sector caduca a los 0.5 s (PX4) / 1 s (ArduPilot); sin datos: no moverse, Hold a los 5 s | 0.5 s / 5 s |
| Sin datos = ? | PX4 CP (`CP_GO_NO_DATA = 0`) vs ArduPilot (frontera a 100 m) | PX4: sector sin datos bloqueado. ArduPilot: libre | — |
| Pose en el instante de la nube | PX4-Avoidance `avoidance/src/transform_buffer.cpp`; voxblox `transformer.cc`; KR `tf_listener.cpp` | Buffer de TF con interpolación lineal + slerp al stamp de la nube; si no hay TF todavía, esperar | buffer 5 s; espera 0.4-0.5 s |
| Timestamp de captura a PX4 | PX4 `Tools/msg/templates/ucdr/msg.h.em` | `timestamp = 0` ⇒ PX4 pone "ahora" y oculta la latencia; con `UXRCE_DDS_SYNCT=1` se traduce la base de tiempo | `UXRCE_DDS_SYNCT=1` |
| Ruido suelto | PX4-Avoidance `planner_functions.cpp` (`min_num_points_per_cell`); CMU `localPlanner.cpp` (`pointPerPathThre`) | Mínimo de puntos por celda/camino | 1-2 puntos |
| Auto-impactos | ArduPilot `AP_Proximity_Backend.cpp` (`PRXn_IGN_ANGx`); `robot_body_filter` (ROS) | Zonas ignoradas / caja del cuerpo | dron + 0.1-0.2 m |
| Sólo volar hacia lo que se ve | PX4-Avoidance `local_planner` (`scaleToFOV`); CMU (`sensorMaxYaw 40°`) | Velocidad × fracción en el FOV (0 fuera); caminos fuera del FOV bloqueados | — |
| Huida | ArduPilot `AC_Avoid.cpp` (`AVOID_BACKUP_SPD`, `AVOID_BACKUP_DZ`) | Retroceso limitado, combinado por cuadrante | 0.75 m/s, banda 0.1 m |
| Sensor perdido | PX4-Avoidance `avoidance_node.cpp`; ego-planner-swarm `grid_map.cpp`; KR `local_global_replan_server.cpp` | Hover a 0.5 s, terminación a 15 s; parada de emergencia a 1 s; stopping policy si el mapa tiene > 2 s | 0.5-2 s |
| Error de seguimiento | KR `trajectory_tracker_upgraded.cpp` (`max_pos_error`) | Abortar si `‖odom − referencia‖` supera el umbral | 0.75 m (0.5-1.0) |
| Obstáculos que se van | ArduPilot `AP_OADatabase.cpp` (`OA_DB_EXPIRE`); PX4-Avoidance (`max_point_age_s`, sólo fuera del FOV); nvblox (decaimiento) | Olvidar lo no visto en un tiempo | 10 s (ArduPilot), 20 s (PX4-Av.) |
| Planificador colgado | ArduPilot `AP_OAPathPlanner.cpp` (`OA_TIMEOUT_MS`) | Resultado de > 3 s = error ⇒ frenar en el punto de frenado | 1 Hz, 3 s |
| Techo / franja vertical | PX4-Avoidance `compressHistogramElevation` (`vertical_cap 1 m`); nvblox `esdf_slice_*`; aerostack2 `max_height 1.0` | La vista 2D para evadir es una franja alrededor de la altura del dron | ±1 m |
| Geocerca | ArduPilot `AC_Avoid` (cerca como obstáculo), PX4 `GF_*` (acción de failsafe) | Cerca blanda en la evasión + failsafe del autopiloto | área + margen |

Lo que **no** encontró ningún proyecto: tratamiento específico de cables
finos o vidrio (ver §4).

**Dónde actúa la Collision Prevention de PX4.** En v1.17 la única llamada a
`modifySetpoint()` está en `StickAccelerationXY` (modos Position y Position
Slow). **En Offboard no modifica el `trajectory_setpoint`**: toda la evasión en
vuelo autónomo es responsabilidad de este stack. `/fmu/in/obstacle_distance`
sólo protege cuando el piloto toma el control (ver
[jetson-px4-1.17.md](jetson-px4-1.17.md) §5).

---

## 2. Qué adoptamos y con qué parámetro

Defaults en `src/uav_mission/config/mission.yaml` (misión) y
`src/uav_wall_mapping/config/mapper_*.yaml` (mapper); entre paréntesis la
brecha de [03_brechas.md](investigacion/03_brechas.md).

### 2.1 Frenado físico (C1) — `avoidance.py`

Con `d` metros que el dron puede avanzar en el corredor de su dirección de
avance antes de quedar a `stop_distance` de algo,

```
v_max(d) = −a·t + √((a·t)² + 2·a·d)        t = avoid.latency + edad de los sectores
alcance del VFH = max(avoid.lookahead, v·t + v²/2a + stop_distance),  v = max(medida, pedida)
```

El corredor tiene el mismo ancho con el que el VFH agranda los obstáculos
(`robot_radius + safety_margin`): una pared paralela al costado no frena, una
que se cruza sí. Sin nada en el corredor, lo libre llega hasta el alcance del
sensor (más allá no se sabe). La zanahoria no se adelanta más que
`v · nav.lead_time` en la dirección de avance (el P de posición de PX4 seguiría
empujando hacia el obstáculo), pero sí de costado (autoridad contra el viento).

| Parámetro | Default | Fuente |
|---|---|---|
| `avoid.max_decel` | 1.0 m/s² | ArduPilot (≤ 1 m/s² para la velocidad segura); PX4 `MPC_ACC_HOR` = 3 |
| `avoid.latency` | 0.4 s | PX4 `CP_DELAY` |
| `avoid.lookahead` | 4.0 m (mínimo) | — |
| `avoid.stop_distance` | 1.3 m | — |
| `nav.lead_time` | 1.0 s | — |

Ejemplo: a 1.5 m/s con 0.5 s de latencia total, empieza a frenar con ~3.2 m
libres; a 3 m/s, con ~7 m (y el VFH mira 7 m). Escenario `13_obstaculo_subito`:
pared que aparece ~7 m delante a 3 m/s con la nube 0.15 s demorada.

### 2.2 Tiempos (C2, C3) — `pose_buffer.py`, `mapper_core.py`, nodos

- Historial de 2 s de posición y actitud de PX4 con su `timestamp_sample`;
  cada nube se transforma con la pose **interpolada en su `header.stamp`**
  (lineal + slerp, como PX4-Avoidance). Si el stamp está desfasado más de
  `time.max_clock_skew` (1 s) del reloj del nodo, o cae fuera del historial, se
  avisa y se usa la hora de llegada con la última pose.
- Los sectores se recalculan en cada publicación contra la pose **actual** a
  partir de los puntos guardados en NED (no de distancias relativas a la pose
  vieja).
- `ObstacleSectors.header.stamp` = captura del dato en vivo más nuevo; la
  misión suma esa edad a la latencia del frenado. QoS BEST_EFFORT, KEEP_LAST 1.
- Una nube (casi) vacía de un sensor de campo amplio (< `sectors.min_cloud_points`
  = 20 puntos) es un sensor ciego: no talla libre ni cuenta como dato fresco.

| Parámetro | Default |
|---|---|
| `sectors.sensor_timeout` (mapper) | 0.4 s desde la captura (antes 0.6 s desde el procesamiento) |
| `avoid.sensor_timeout` (misión) | 0.5 s (antes 1.0 s desde la llegada) |
| `time.use_px4_timestamps`, `time.max_clock_skew`, `time.pose_history`, `time.max_extrapolation` | true, 1.0 s, 2.0 s, 0.1 s |

### 2.3 Franja de alturas (C4) — `mapper_core.py`

La grilla 2D (A*, fronteras, memoria de los sectores) va del piso
(`map.min_height`) a `altura del dron + map.max_height_above_drone` (1.0 m):
el techo de una habitación ya no tapa la grilla (escenario
`12_habitacion_con_techo`). El mapa 3D guarda todo. Además el piso "sube" con
la distancia (`map.floor_slope_deg` = 2°): con 1-2° de error de actitud un
retorno del piso a 20 m quedaba 0.3-0.7 m arriba y se volvía pared.

### 2.4 Auto-impactos y ruido (C5, A1) — `cloud.py`, `sectors.py`

| Parámetro | Default | Qué hace |
|---|---|---|
| `self_filter.box` | [-0.5, 0.5, -0.5, 0.5, -0.35, 0.4] m (cuerpo FRD) | Saca hélices, patas, mástil. Medir el dron y sumar ~5 cm. |
| `outliers.min_points`, `outliers.voxel`, `outliers.max_range` | 2, 0.25 m, 8 m | Puntos con menos de N en su vecindad 3x3x3 de vóxeles: ruido. |
| `sectors.min_points` | 2 | Distancia del sector = la del k-ésimo punto más cercano (un retorno suelto no acerca la pared). Los rangefinders usan 1. |

La **huida de emergencia sólo la dispara el sensor en vivo**
(`ObstacleSectors.live_ranges`), no la memoria del mapa; va a
`avoid.escape_speed` = 0.75 m/s (ArduPilot `AVOID_BACKUP_SPD`).

### 2.5 Sectores vistos y campo de visión (A2)

- `ObstacleSectors.seen_age`: por sector, cuánto hace que un sensor en vivo lo
  tuvo en su campo. Desvíos y huida **sólo hacia sectores vistos** hace menos de
  `avoid.seen_timeout` (1.0 s) y libres; la huida además a ≥ 90° del obstáculo.
  Sin salida vista: frenar y girar en el lugar (antes, con cámara, retrocedía a
  ciegas).
- `scaleToFOV`: velocidad × fracción de sectores vistos en ±`avoid.fov_check_deg`
  (20°) del avance; con 0 no avanza y gira hacia el avance.
- `/fmu/in/obstacle_distance`: no vistos = `UINT16_MAX` (desconocido; con
  `CP_GO_NO_DATA = 0` PX4 no deja moverse hacia ahí) y `timestamp` = captura.

### 2.6 Failsafes (A4) — `mission_core.py`

| Parámetro | Default | Qué hace |
|---|---|---|
| `nav.pose_timeout` / `nav.pose_loss_land_after` | 0.5 s / 2 s | Sin posición válida: referencia fija (PX4 frena); si dura, LAND. |
| `avoid.hold_on_sensor_loss` / `avoid.sensor_loss_land_after` | true / 10 s | Sin sectores frescos: frenar; si dura, LAND (PX4-Avoidance; PX4 CP pasa a Hold a 5 s). |
| `avoid.on_return_failure` | land | RETURN imposible: aterrizar en el lugar (antes HOLD infinito). |
| `nav.max_tracking_error` / `nav.tracking_brake_sec` | 0.75 m / 1 s | El dron se pasó por delante de la referencia (o de altura): frenar (KR). El desvío lateral no cuenta: frenar no lo corrige y con viento lo empujaba a la pared. |
| `safety.geofence` / `safety.geofence_margin` | [] / 3 m | Cerca blanda: la referencia no sale del área de mapeo + margen (o de la cerca explícita). |
| `safety.min_altitude` / `safety.max_altitude` | 0.5 m / 20 m | Recorte de la referencia y de los objetivos. |

### 2.7 Otros

- **Puertas**: el A* mide la distancia a los obstáculos en la grilla fina e
  infla `plan.inflation_radius` = 0.65 m (radio + 0.2; el costo suave hasta
  2 m prefiere el centro); VFH `avoid.safety_margin` 0.35 m,
  `avoid.emergency_distance` 0.7 m. **Ancho mínimo de paso práctico ≈ 1.8-2.0 m**
  (antes ~2.4 m); con 1.6 m el VFH oscila delante de la puerta porque desde
  cerca el hueco queda tapado por el agrandado de los marcos. Tras un fallo del
  A* se espera `nav.replan_period` antes de reintentar (antes se agotaban los
  3 intentos en 0.15 s). Escenario `11_puerta_angosta` (2.0 m).
- **Extracción de paredes y guardado fuera del lock** de `mapper_node`
  (`WallMapper.capture_state()`): los sectores no esperan (A5).
- **Olvido de lo efímero** (`map.forget_after`, desactivado por defecto; 10 s
  recomendado con gente o vehículos): celdas y vóxeles vistos en pocas nubes
  que no se volvieron a ver.

### 2.8 Simulación de entorno real — `uav_sim`

El bloque `simulacion:` de un escenario agrega latencia de la nube (con stamp
de captura), actitud que cabecea con la aceleración (el lidar ve el piso al
frenar), ruido, polvo, auto-impactos, viento con ráfagas y obstáculos que
aparecen o se mueven; `fake_px4_world` usa lo mismo. Escenarios nuevos:
`10_poste_fino`, `11_puerta_angosta`, `12_habitacion_con_techo`,
`13_obstaculo_subito`, `14_entorno_real` (todo junto). `test_closed_loop.py`
cuenta como choque una distancia menor que `avoid.robot_radius` (antes 0.3 m,
menos que el radio del dron).

---

## 3. Configuración para el primer vuelo real

`mapper_mid360.yaml` ya trae los valores de dron real. En PX4: `UXRCE_DDS_SYNCT=1`,
`MPC_TILTMAX_AIR` 25-30°, `MPC_XY_VEL_MAX` ≈ `cruise_speed + 0.5`,
`COM_OF_LOSS_T` 0.5-1.0 s, `GF_*` con el área + 10 m. Empezar con
`cruise_speed` ≤ 1.5 m/s con lidar y ≤ 1.0 m/s con cámara, medir la latencia
real (`ros2 topic delay /livox/lidar`, edad de `/obstacles/sectors`) y recién
ahí subir `cruise_speed`.

---

## 4. Lo que NO cubrimos

- **Vidrio, espejos, superficies negras**: el lidar los ve sólo de frente o no
  los ve. Lo único que ayuda es `occupied_miss_scale` (una celda ocupada vista
  de frente no la vacía un rayo que la atraviesa). Ningún proyecto revisado lo
  resuelve en código; ver TOPGN (detección por intensidad) en el reporte 02.
- **Cables y ramas finas**: un cable de 1 cm a 5 m necesita < 0.11° entre
  rayos; el Mid-360 no llega. El escenario `10_poste_fino` es de 10 cm. Volar
  despacio cerca de vegetación y cables.
- **Deskew completo**: el Mid-360 integra 100 ms por nube; se usa la pose del
  `header.stamp`, no la de cada punto. Girando a 0.8 rad/s eso son ~4.6° de
  distorsión por cuadro. **Recomendado: FAST-LIO2** (`/cloud_registered`, ya
  deskewada y registrada, y su odometría como pose externa para EKF2) cuando
  la pose de PX4 no alcance (interiores sin GPS).
- **Evasión vertical**: no hay chequeo arriba/abajo al cambiar de altura,
  despegar o aterrizar (A3); LAND es el de PX4.
- **Polvo denso / niebla**: una nube de polvo se ve como pared en el borde
  delantero (Phillips et al. 2017); el filtro sólo saca retornos sueltos.
- **Obstáculos dinámicos rápidos**: no hay predicción de movimiento; una caja
  que se detiene un rato y se va queda en el mapa hasta que un rayo la vacía
  (con `map.forget_after` sólo si se vio pocas veces).
- **Batería**: no se estima el consumo para volver (A4 d).
- **Deriva del EKF / multipath**: el mapa queda en el marco del EKF; si salta,
  el mapa queda corrido.
