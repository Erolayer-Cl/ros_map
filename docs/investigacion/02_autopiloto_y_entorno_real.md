# 02 — Cómo resuelven los autopilotos (PX4, ArduPilot) y los ejemplos ROS 2 los problemas del entorno real

Fecha: 2026-11-01. Contexto: workspace `/home/user/ros_map` (ROS 2 Humble, PX4 v1.17 por uXRCE-DDS, Jetson,
VFH+ sobre 72 sectores, A* sobre grilla, publicación opcional de `/fmu/in/obstacle_distance`).

Convenciones de este documento:
- **[código]**: leído directamente del fuente (ruta y línea aproximada indicadas).
- **[doc]**: leído de la documentación fuente (repositorio de docs).
- **[análisis]**: deducción propia a partir del código; no es una afirmación del proyecto.
- **[NO VERIFICADO]**: no pude confirmarlo leyendo una fuente primaria en esta sesión.

Copias locales de lo leído (sólo lectura, nada se ejecutó):
`/tmp/claude-0/-home-user-ros-map/ed946d96-4b5a-536d-8bc7-aabb24974ec4/scratchpad/investigacion/repos/`
(`px4/`, `px4-sparse/` = PX4 tag v1.17.0; `ardupilot-sparse/` = ArduPilot master, commit
`363ab1b5efdb67a0d0ade62984685b6d44d0f40d` del 2026-10-31; `ardupilot_wiki/`; `ros2ex/`; `px4avoid/`).

---

## 1. PX4 v1.17 — Collision Prevention

Archivos: `src/lib/collision_prevention/CollisionPrevention.cpp` (589 líneas), `CollisionPrevention.hpp`,
`collisionprevention_params.c`, `ObstacleMath.cpp`, `src/lib/mathlib/math/TrajMath.hpp`,
`docs/en/computer_vision/collision_prevention.md` (todo en el tag `v1.17.0`).

### 1.1 Dónde actúa (lo más importante para este workspace)

- [código] La única llamada a `modifySetpoint()` está en
  `src/modules/flight_mode_manager/tasks/Utility/StickAccelerationXY.cpp:117-118`:
  ```cpp
  if (_collision_prevention.is_active()) {
      _collision_prevention.modifySetpoint(_acceleration_setpoint, _velocity_setpoint);
  }
  ```
  `StickAccelerationXY` lo usan `FlightTaskManualAcceleration` (modo Position con
  `MPC_POS_MODE` = acceleration based), `FlightTaskManualAccelerationSlow` (Position Slow) y
  `FlightTaskAuto` sólo para el "nudging" con sticks durante el aterrizaje (`FlightTaskAuto.cpp:287`).
  `FlightTaskManualPosition.hpp:71` declara un `CollisionPrevention` pero el `.cpp` no lo invoca.
- [análisis] **En modo Offboard la Collision Prevention de PX4 NO modifica `trajectory_setpoint`.**
  Publicar `/fmu/in/obstacle_distance` (está en `src/modules/uxrce_dds_client/dds_topics.yaml:146`)
  sólo protege cuando el vehículo está en Position/Position Slow (p. ej., el piloto toma el control,
  o el failsafe de pérdida de offboard con `COM_OBL_RC_ACT=0` = "Position mode", ver §3.1).
  La evasión en Offboard es 100 % responsabilidad de la Jetson.
- [doc] `collision_prevention.md:6`: "It can be enabled ... in Position mode (with MPC_POS_MODE set to
  `Acceleration based`)". `is_active()` = `CP_DIST > 0` (`CollisionPrevention.cpp:72-82`).

### 1.2 Parámetros (`collisionprevention_params.c`)

| Parámetro | Default | Significado (del fuente) |
|---|---|---|
| `CP_DIST` | -1 m (desactivado) | distancia mínima a mantener; [doc] "is the distance to the sensors, not the outside of your vehicle or propellers. Be sure to leave a safe margin!" |
| `CP_DELAY` | 0.4 s (0–1) | "Average delay of the range measurement data plus the tracking delay of the position controller" |
| `CP_GUIDE_ANG` | 30° (0–90) | ángulo a cada lado del comando que el algoritmo puede desviar |
| `CP_GO_NO_DATA` | 0 | permitir moverse hacia sectores sin datos |

### 1.3 Mapa de obstáculos, envejecimiento y timeouts

- [código] `BIN_COUNT = 72`, `BIN_SIZE = 5°` (`.hpp:83-85`), mapa en `MAV_FRAME_BODY_FRD`.
- [código] `RANGE_STREAM_TIMEOUT_US = 500 ms`, `TIMEOUT_HOLD_US = 5 s` (`.hpp:189-190`).
- [código] Al recibir datos (`_updateObstacleMap`, `.cpp:109-151`): se descarta el mensaje si su
  `timestamp` tiene más de 500 ms o si `increment <= 0`; los `distance_sensor` con orientación
  hacia abajo/arriba se ignoran.
- [código] **Envejecimiento por sector** (`_updateObstacleData`, `.cpp:159-163`):
  ```cpp
  if (getTime() - _data_timestamps[i] > RANGE_STREAM_TIMEOUT_US) {
      _obstacle_map_body_frame.distances[i] = UINT16_MAX;   // sector vuelve a "desconocido"
  }
  ```
  Cada sector caduca a los 0.5 s si nadie lo refresca.
- [código] **Sin datos** (`.cpp:210-224`): si no queda ningún sector válido, `setpoint_accel.setZero()`
  (no se mueve en XY), avisa "No obstacle data, not moving..." cada 1 s y, si pasan más de 5 s sin datos
  (y CP lleva activo > 5 s), publica `VEHICLE_CMD_DO_SET_MODE` → AUTO_LOITER (Hold) (`_publishVehicleCmdDoLoiter`, `.cpp:574-589`).
  [doc] líneas 86-94: "If the autopilot does not receive range data from any sensor for longer than 0.5s ...
  After 5 seconds ... switch into HOLD mode."
- [código] **Dirección sin cobertura** (`_checkSetpointDirectionFeasability`, `.cpp:355-370`): si el sector del
  comando no tiene dato y `CP_GO_NO_DATA=0` (o tiene `_data_fov` = alguna vez lo cubrió un sensor), la parte
  "piloto" de la aceleración queda en cero. [doc] "if you have no rear-sensor data, you will not be able to fly backwards".
  Si un sensor de varios muere, su zona pasa a "sin cobertura" ([doc] líneas 93-99).

### 1.4 Fusión de varios sensores y orientación

- [código] `_enterData()` (`.cpp:323-353`) decide si un dato nuevo sobrescribe el sector:
  1. lectura en rango, y el sector tiene dato válido de un sensor de **igual o menor alcance**, o el sector estaba fuera de rango;
  2. lectura fuera de rango sólo pisa si el sector también estaba fuera de rango y este sensor tiene **mayor** alcance,
     o si viene del **mismo** sensor (mismo `max_distance`).
  No hay promedio ni filtro: último dato "aceptable" gana. [análisis] no hay rechazo de outliers en PX4: el filtrado
  de ruido (polvo, insectos) debe hacerlo la Jetson antes de publicar.
- [código] `distance_sensor`: orientación por `orientation` (YAW_0…YAW_315) o `ROTATION_CUSTOM` con cuaternión `q`
  (`ObstacleMath.cpp:38-84`); se ocupan todos los bins que tocan `±h_fov/2` (`.cpp:401-420`); la distancia se
  **proyecta al plano horizontal** con la actitud del vehículo (`project_distance_on_horizontal_plane`,
  `ObstacleMath.cpp:8-18`); se descartan lecturas `<= min_distance`; `-1` con `signal_quality==0` = "fuera de rango válido".
- [código] `obstacle_distance` externo (`_addObstacleSensorData`, `.cpp:228-321`): acepta `frame` =
  `MAV_FRAME_GLOBAL`/`MAV_FRAME_LOCAL_NED` (alineado al norte; se rota por el yaw del vehículo) o
  `MAV_FRAME_BODY_FRD` (=12, alineado al frente); cualquier otro frame se rechaza con un log crítico.
  Hay superposición angular entre los bins del mensaje y los internos (permite `increment` distinto de 5°).
  Comentario en `.cpp:227`: "TODO this gives false output if the offset is not a multiple of the resolution".
  Valores `UINT16_MAX` se ignoran (desconocido). Definición del mensaje (`msg/ObstacleDistance.msg`):
  `0` = obstáculo pegado al sensor, `max_distance+1` = libre, `UINT16_MAX` = desconocido, unidades cm,
  `angle_offset` positivo en sentido horario.
- [código] **Marcas de tiempo por uXRCE-DDS** (`Tools/msg/templates/ucdr/msg.h.em:166-169`):
  ```
  if (topic.timestamp == 0) topic.timestamp = hrt_absolute_time();
  else topic.timestamp = math::min(topic.timestamp - time_offset, hrt_absolute_time());
  ```
  [análisis] Si la Jetson publica `timestamp = 0`, PX4 lo toma como "recién llegado" y **oculta la latencia**
  de la percepción. Conviene poner el instante de captura del sensor (reloj del agente, µs, como hace
  `px4_ros_com`: `get_clock()->now().nanoseconds()/1000`) con `UXRCE_DDS_SYNCT=1`; así el término
  `data_age` de §1.5 descuenta la latencia real.

### 1.5 Cómo limita la velocidad (fórmulas)

PX4 v1.17 trabaja sobre la **aceleración** que pide el stick, en dos partes:

**(a) Escalado de la aceleración del piloto** (`_constrainAccelerationSetpoint` + `_getScale`, `.cpp:487-522`):
```
d_keep        = max(min_distance_del_sensor, CP_DIST)                     (.cpp:186)
scale_distance = max(d_keep, MPC_VEL_MANUAL / MPC_XY_P)                    (.cpp:516)
s(d) = (d - d_keep)                   si d <= d_keep  (negativo: empuja hacia atrás, lineal)
s(d) = min(((d - d_keep)/scale_distance)^2, 1)   si d > d_keep (cuadrático)
```
La aceleración se descompone en componente normal (hacia el obstáculo más cercano) y tangencial;
cada una se escala con `s` de la distancia en su dirección; sólo si el comando apunta hacia el obstáculo
(`dot > 0`). Esto produce el "deslizamiento" a lo largo de paredes.

**(b) Compensación por la velocidad actual** (`_getVelocityCompensationAcceleration`, `.cpp:524-572`), por cada bin con dato:
```
v_par          = max(0, v_setpoint · dir_bin)
delay_distance = v_par * CP_DELAY  (+ v_par * data_age  si d < max_range)
stop_distance  = d - d_keep - delay_distance
si stop_distance >= 0:
    v_max = computeMaxSpeedFromDistance(MPC_JERK_MAX, MPC_ACC_HOR, stop_distance, 0)
    a_comp = MPC_XY_VEL_P_ACC * min(v_max - v_par, 0)
si no:
    a_comp = -MPC_XY_VEL_P_ACC * v_par
```
Se toma el bin con `a_comp` más negativa y se suma en esa dirección (`.cpp:208`).
`computeMaxSpeedFromDistance` (`TrajMath.hpp:61-71`) resuelve v² + (4a²/j)·v − 2·a·d = 0, es decir
**v_max = −2a²/j + sqrt(4a⁴/j² + 2·a·d)** (frenado a aceleración constante con un retardo 2a/j de rampa de jerk).
Equivale a la regla clásica **d_frenado = v·t_retardo + v²/(2a)** con un término extra de jerk.

Defaults v1.17 que entran en las fórmulas [código, `src/modules/mc_pos_control/*_params.c`]:
`MPC_ACC_HOR = 3 m/s²`, `MPC_JERK_MAX = 8 m/s³`, `MPC_XY_P = 0.95`, `MPC_XY_VEL_P_ACC = 1.8`,
`MPC_VEL_MANUAL = 10 m/s`, `MPC_POS_MODE = 4` (acceleration based, ya es el default).

Ejemplos numéricos con esos defaults (calculados con las fórmulas de arriba):
- `v_max(stop_distance)`: 0.5 m → 0.59 m/s; 1 m → 1.08 m/s; 2 m → 1.88 m/s; 5 m → 3.67 m/s.
- Obstáculo a 3 m, `CP_DIST = 1`, `v_par = 1.5 m/s`, `CP_DELAY = 0.4 s`, dato de 0.1 s:
  `stop_distance = 3 − 1 − 1.5·(0.4+0.1) = 1.25 m` → `v_max = 1.29 m/s` → `a_comp = 1.8·(1.29 − 1.5) = −0.37 m/s²`.
- `scale_distance = max(CP_DIST, 10/0.95) = 10.5 m`: la aceleración del piloto empieza a escalarse ~10 m antes de `CP_DIST`.

[doc] `collision_prevention.md:101-115`: sensor delay ≈ 0 para sensores en el FC, "may be as high as 0.2s" para visión
externa; tracking delay "typically between 0.1 and 0.5 seconds"; si la velocidad oscila al acercarse, `CP_DELAY` está alto.
[doc] línea 170-172: prueba inicial a 4 m/s con `OBSTACLE_DISTANCE` a 10 Hz.

### 1.6 Guía (`CP_GUIDE_ANG`) — `_adaptSetpointDirection`, `.cpp:424-467`
```
guidance_bins = floor(CP_GUIDE_ANG / 5)
para i en [i0 - guidance_bins, i0 + guidance_bins]:
    mean_dist = promedio de 3 bins (i-1, i, i+1); bins desconocidos cuentan como CP_DIST
    cost = CP_DIST*50*|i - i0|  -  mean_dist  -  dist[i]          (todo en cm)
elegir el i de menor costo, sólo entre bins con dato (nunca hacia lo desconocido)
```
[doc] "From testing, 30 degrees is a good balance"; demasiado chico → se "pega"; demasiado grande → se "desliza" sin pedirlo.

### 1.7 Qué implica para la Jetson [análisis]
- Llenar los 72 bins con `increment = 5.0`, `frame = 12` (BODY_FRD) y `angle_offset` = 0 (o múltiplo de 5°).
- `UINT16_MAX` donde no hay dato, `max_distance + 1` donde está libre; no usar `0` salvo obstáculo pegado
  (ArduPilot descarta el 0, PX4 lo interpreta como contacto).
- Publicar ≥ 10 Hz y con `timestamp` de captura: cada sector se borra a los 0.5 s.
- Filtrar ruido y auto-impactos ANTES de publicar: PX4 no filtra.

---

## 2. ArduPilot (master, 2026-10-31)

### 2.1 Simple Avoidance — `libraries/AC_Avoidance/AC_Avoid.cpp`

Parámetros (`AC_Avoid.cpp:43-123`, prefijo `AVOID_`):

| Parámetro | Default | Notas del fuente |
|---|---|---|
| `AVOID_ENABLE` | 3 (fence + proximity) | bitmask 0:fence,1:proximity,2:beacon (`AC_Avoid.h:15-19`) |
| `AVOID_MARGIN` | 2.0 m (1–10) | distancia a mantener en modos con GPS |
| `AVOID_BEHAVE` | slide/stop | 0 Slide, 1 Stop |
| `AVOID_BACKUP_SPD` | 0.75 m/s (0–2) | velocidad máx. de retroceso horizontal; 0 desactiva |
| `AVOID_BACKUP_DZ` | 0.10 m | banda muerta entre "frenar" y "retroceder" (evita ir-venir) |
| `AVOID_BACKZ_SPD` | 0.75 m/s | retroceso vertical |
| `AVOID_ACCEL_MAX` | 3.0 m/s² (0–9) | limita el cambio de velocidad que introduce la evasión (suavidad) |
| `AVOID_ALT_MIN` | 0 m | altura mínima (con telémetro hacia abajo) para activar evasión por proximidad |
| `AVOID_DIST_MAX` | 5 m | sólo AltHold (sin GPS): distancia a la que empieza a inclinar |
| `AVOID_ANGLE_MAX` | 10° | sólo AltHold |

Constantes (`AC_Avoid.h:12-26`): `AC_AVOID_ACCEL_CMSS_MAX = 100` cm/s² → **para calcular la velocidad
segura frente a cercas y proximidad se usa como máximo 1 m/s²** (`AC_Avoid.cpp:233`:
`accel_limited_cmss = MIN(accel_cmss, AC_AVOID_ACCEL_CMSS_MAX)`), `AC_AVOID_ACTIVE_LIMIT_TIMEOUT_MS = 500`,
`AC_AVOID_ACCEL_TIMEOUT_MS = 200`.

Fórmulas:
- [código] `get_max_speed` (`AC_Avoid.cpp:763-771`): con `kP = 0` → **v_max = sqrt(2·a·d)**; con `kP > 0` →
  `sqrt_controller(d, kP, a, dt)` (lineal cerca, raíz lejos).
- [código] `get_stopping_distance` (`AC_Avoid.cpp:1492-1513`, copiada de AC_PosControl):
  `kP<=0 → v²/(2a)`; `v < a/kP → v/kP`; si no → **a/(2kP²) + v²/(2a)**.
- [código] `limit_velocity_NEU` (`:603-659`): el margen se aplica en la dirección del movimiento
  (`margin_vector = v̂ · margin`); se proyecta la velocidad deseada sobre la dirección al obstáculo y se recorta
  la componente que excede `v_max(d - margin)`; la componente tangencial queda (→ "slide").
- [código] Modo Stop (`:1265-1271`, `:1325-1350`): se proyecta el punto de frenado
  `v̂ · (0.02 m + margin + stopping_distance)` y, si el segmento intersecta la frontera, se frena;
  dentro del margen → velocidad cero.
- [código] Retroceso (`:1290-1303`, `calc_backup_velocity_2D` `:661-683`): si `dist < margin` y la invasión
  supera `AVOID_BACKUP_DZ`, `v_back = get_max_speed(kP, 0.4·accel, invasión)`, recortada a `AVOID_BACKUP_SPD`;
  las velocidades de varios obstáculos se combinan por cuadrante (máximo por cuadrante, `:718-742`) para no sumar
  dos veces el mismo empuje. El piloto puede alejarse más rápido que el retroceso calculado (`:264-279`).
- [código] Suavizado (`limit_accel_NEU_cm`, `:329-356`): `|Δv/dt| <= AVOID_ACCEL_MAX`; se resetea si la evasión no
  actuó en 200 ms.

Dónde actúa (Copter):
- [código] Loiter: `libraries/AC_WPNav/AC_Loiter.cpp:386`. AltHold/FlowHold: `adjust_roll_pitch_rad`
  (`ArduCopter/mode_althold.cpp:80`, `mode_flowhold.cpp:347`).
- [código] **Guided con velocidad/aceleración**: `ArduCopter/mode_guided.cpp:857-860`
  (`copter.avoid.adjust_velocity_NED_m(...)`). Es decir, en ArduPilot el equivalente a Offboard por velocidad
  **sí** pasa por la evasión simple, a diferencia de PX4. [doc] `common-simple-object-avoidance.rst:26-30`
  sólo menciona AltHold/Loiter y deriva AUTO/GUIDED a BendyRuler/Dijkstra (la doc va por detrás del código).
- [código] `GUID_TIMEOUT` = 3 s (0.1–5) (`ArduCopter/Parameters.cpp:861-867`): en Guided por velocidad,
  si no llegan comandos en ese tiempo, la velocidad objetivo pasa a cero (`mode_guided.cpp:803-810`).
- [código] Altura mínima: `Copter::low_alt_avoidance()` (`ArduCopter/avoidance.cpp:4-19`) desactiva la evasión por
  proximidad bajo `AVOID_ALT_MIN` si hay telémetro hacia abajo válido; sin telémetro la deja activa.

### 2.2 AP_Proximity — filtros, timeouts, zonas ignoradas, fusión

Estructura 3D (`AP_Proximity_Boundary_3D.h:22-30`): **8 sectores de 45° × 5 capas de 30° de pitch**;
`PROXIMITY_BOUNDARY_DIST_MIN = 0.6 m`; `PROXIMITY_BOUNDARY_DIST_DEFAULT = 100 m` ("if we have no data for a
sector, boundary is placed 100m out"); `PROXIMITY_FILT_RESET_TIME = 1000 ms`; `PROXIMITY_FACE_RESET_MS = 1000 ms`.

- [código] **Sin datos = libre** (a diferencia de PX4, donde sin datos = bloqueado).
- [código] **Filtro** (`set_filtered_distance`, `AP_Proximity_Boundary_3D.cpp:99-118`): pasa-bajos de 1.er orden por
  cara con corte `PRX_FILT` (default **0.25 Hz**, rango 0–20, `AP_Proximity.cpp:63-69`); si la cara no se actualizó en
  1 s, el filtro se resetea al valor nuevo (no arrastra datos viejos). [análisis] con 0.25 Hz, τ = 1/(2π·0.25) ≈ 0.64 s:
  un obstáculo que aparece de golpe tarda ~0.6 s en "acercarse" en el filtro; es un compromiso entre rechazar espurios
  y retardo, que se suma a `CP_DELAY`-equivalente.
- [código] **Frontera conservadora** (`update_boundary`, `:123-173`): cada punto de frontera se ubica en el borde entre
  dos sectores, a la **menor** distancia de los dos vecinos; si un vecino no tiene dato se arma una "copa".
- [código] **Caducidad por cara** (`check_face_timeout`, `:213-234`): cada 750 ms se invalidan caras no actualizadas en
  más de 1 s.
- [código] **Fusión de sensores** (`set_face_attributes`, `:60-86`): una instancia distinta no puede pisar una distancia
  **más corta** de otra si esta se actualizó hace menos de 1 s → gana la más cercana (conservador). `reset_face` de
  otra instancia se ignora si el dato tiene < 200 ms (`:187-211`).
- [código] **Rechazo de lecturas** (`ignore_reading`, `AP_Proximity_Backend.cpp:55-85`): fuera de `PRXn_MIN`/`PRXn_MAX`,
  dentro de una zona ignorada (`PRXn_IGN_ANGx` + `PRXn_IGN_WIDx`, x = 1..4 en `AP_Proximity_Params.cpp:32-94`; la wiki
  dice "up to 6" — discrepancia), o "cerca del suelo".
- [código] **Suelo** (`check_obstacle_near_ground`, `AP_Proximity_Utils.cpp:55-88`): con `PRX_IGN_GND=1`, armado y
  telémetro hacia abajo fresco (< 500 ms), un punto rotado a NED cuya `z` está a menos de `PRX_ALT_MIN` (default 1 m)
  del suelo se ignora. [doc] advierte que "might also ignore valid obstacles when the vehicle is flying very close to the ground".
- [código] **MAVLink** (`AP_Proximity_MAV.cpp`): `PROXIMITY_MAV_TIMEOUT_MS = 500` → estado NoData; en
  `OBSTACLE_DISTANCE` se descartan `0`, `65535`, `< min_distance`, `> max_distance` (`:171-176`); se usa `increment_f`
  si no es 0; `PRXn_YAW_CORR` y `PRXn_ORIENT` (montaje invertido) corrigen la orientación (`:146-150`); para cada cara
  se guarda la **mínima** distancia de los bins que caen en ella.

### 2.3 Path planning: AP_OAPathPlanner, BendyRuler, Dijkstra, OADatabase

`AP_OAPathPlanner.cpp`:
- [código] `OA_TYPE`: 0 off, 1 BendyRuler, 2 Dijkstra, 3 Dijkstra+BendyRuler. `OA_MARGIN_MAX` default **5 m** (0.1–100):
  "BendyRuler only accepts a candidate path whose clearance exceeds this, and Dijkstra uses it as the margin held from
  fences" (`:49-56`). [doc] wiki: "2m is a typical value".
- [código] Corre en un **hilo aparte a 1 Hz** (`OA_UPDATE_MS = 1000`, `:34`); resultados de más de **3 s**
  (`OA_TIMEOUT_MS`, `:35`) → `OA_ERROR`; se verifica que el resultado corresponda al destino pedido (`:226-245`).
- [código] Reacción en `AC_WPNav_OA.cpp:181-201`: `OA_PROCESSING` (salvo opción fast-waypoints) y `OA_ERROR`
  → **el vehículo se detiene en su punto de frenado** ("OA temporarily failing — stop vehicle at current position").

`AP_OABendyRuler.cpp`:
- [código] Defaults (`:28-38`): `OA_BR_LOOKAHEAD` **15 m** (wiki: "5m is typical"), `OA_BR_CONT_RATIO` 1.5,
  `OA_BR_CONT_ANGLE` 75°, prueba cada **5°** (`OA_BENDYRULER_BEARING_INC_XY`), paso 2 de al menos 2 m, 2 m más allá
  del destino, por debajo de 0.2 m/s usa el yaw en vez del rumbo sobre el suelo.
- [código] Algoritmo (`search_xy_path`, `:139-243`): alterna izquierda/derecha hasta ±170° desde el rumbo al destino;
  para cada rumbo calcula el margen mínimo del segmento (`calc_avoidance_margin`, distancia segmento-obstáculo
  menos radio); si supera `OA_MARGIN_MAX`, hace un segundo paso desde ese punto hacia el destino en 0° y ±45°;
  entre rumbos válidos prefiere el más cercano al rumbo actual. Si ninguno pasa, elige el de mayor margen y
  acorta el lookahead (×0.9, mínimo 0.5×); si pasa, lo alarga (×1.1).
- [código] Histéresis (`resist_bearing_change`, `:381-417`): si el nuevo rumbo cambia más de `CONT_ANGLE` respecto
  del anterior y su margen no es al menos `CONT_RATIO` × el margen del rumbo anterior, **mantiene el rumbo anterior**
  (anti-oscilación, equivalente al "debounce" del VFH+).
- [código] Variante vertical (`OA_BR_TYPE=2`): prueba adelante, arriba, abajo y atrás.

`AP_OADatabase.cpp` (los obstáculos de proximidad se guardan en coordenadas de mundo):
- [código] `OA_DB_SIZE` 100, `OA_DB_EXPIRE` **10 s** ("Zero means never expires which is useful for a sent-once static
  environment but terrible for dynamic ones"), `OA_DB_QUEUE_SIZE` 80, `OA_DB_BEAM_WIDTH` 5°,
  `OA_DB_RADIUS_MIN` 0.01 m, `OA_DB_DIST_MAX` 0 (sin límite), `OA_DB_ALT_MIN` 0 (`:28-108`).
- [código] Radio del obstáculo = `distancia · tan(beam_width)` (`:132`, `:156`) → los puntos lejanos se inflan más
  (compensa la incertidumbre angular).
- [código] `OA_DB_ALT_MIN`: rechaza puntos si el vehículo está a menos de esa altura en un radio de 3 m del home
  (suelo visto al despegar, `:166-181`).

`AP_OADijkstra.cpp`: [código] sólo trabaja con **cercas** (polígonos de inclusión/exclusión y círculos de exclusión),
infladas por el margen (`create_inclusion_polygon_with_margin(_polyfence_margin*100)`, ~`:96-125`), arma un grafo de
visibilidad y busca el camino más corto; los errores se informan cada 5 s.

---

## 3. Ejemplos ROS 2 (+PX4) y seguridad

### 3.1 Lado PX4: cómo detecta la pérdida del companion
- [doc] `docs/en/flight_modes/offboard.md:8,15-16,26-28` (v1.17): "continuous 2Hz proof of life"; el stream debe
  existir "for at least a second" antes de armar/entrar en offboard; con ROS 2 el latido es `OffboardControlMode`
  y el setpoint va por otro tópico (`TrajectorySetpoint`), que "would only need ... once" para mantener posición.
- [código] `src/modules/commander/HealthAndArmingChecks/checks/offboardCheck.cpp:38-66`: la señal offboard se
  considera viva si `offboard_control_mode.timestamp` tiene menos de `COM_OF_LOSS_T` (default **1.0 s**,
  `commander_params.c:340`) y además, si se pide posición/velocidad, la estimación local debe ser válida.
- [código] `COM_OBL_RC_ACT` (default **0 = Position mode**; 1 Altitude, 2 Stabilized, 3 Return, 4 Land, 5 Hold,
  6 Terminate, 7 Disarm) (`commander_params.c:352-369`). `COM_OBC_LOSS_T` 5 s: aviso de pérdida de computadora
  de a bordo (`:371-380`).
- [análisis] El latido es `offboard_control_mode`, **no** la salud de la percepción: si el nodo sigue publicando el
  latido pero el lidar se colgó, PX4 no se entera. El nodo debe dejar de mandar setpoints de avance (o pasar a Hold)
  cuando su propia entrada está vieja.
- [análisis] Sinergia: con `CP_DIST > 0` y `COM_OBL_RC_ACT = 0`, al caerse offboard PX4 pasa a Position, donde la CP
  sí actúa con `/fmu/in/obstacle_distance` (si la Jetson sigue viva); si la Jetson murió, la CP bloquea XY y a los 5 s
  pasa a Hold (§1.3). Requiere RC/joystick para Position; sin RC entra el failsafe de RC. [NO VERIFICADO en vuelo/SITL].

### 3.2 `px4_ros_com` — `src/examples/offboard/offboard_control.cpp` (rama main)
- [código] Timer de 100 ms (10 Hz) que publica `OffboardControlMode` + `TrajectorySetpoint`; tras 10 setpoints manda
  `DO_SET_MODE` offboard y `ARM` (`:65-86`); `timestamp = get_clock()->now().nanoseconds()/1000` (`:140`, `:154`, `:175`).
- [análisis] No lee `vehicle_status`, no tiene timeouts de entrada, no maneja failsafes: es un ejemplo mínimo,
  **no** una referencia de seguridad.

### 3.3 `px4-ros2-interface-lib` (Auterion; modos externos ROS 2 en PX4 ≥ 1.15)
- [código PX4] `HealthAndArmingChecks/checks/externalChecks.hpp:69-75`: PX4 pide el estado a cada modo externo cada
  **300 ms**, espera respuesta **50 ms**; tras **> 3** pedidos sin respuesta (~1 s; 10 al registrarse) el modo se marca
  *unresponsive* y PX4 aplica failsafe/sale del modo (`externalChecks.cpp:240-290`).
- [código lib] `ModeBase::checkArmingAndRunConditions(HealthAndArmingCheckReporter&)` "is called regularly, also while the
  mode is active" (`px4_ros2_cpp/include/px4_ros2/components/mode.hpp:134-137`); `reporter.armingCheckFailureExt(...)`
  pone `can_arm_and_run = false` (`health_and_arming_checks.hpp:29-40`). Watchdog de 4 s en la lib: si el FMU deja de
  preguntar, el nodo se apaga (`health_and_arming_checks.cpp:67`, `:91-107`).
- [análisis] Es el patrón más limpio para atar "lidar fresco / TF ok / CPU ok" al permiso de volar: el chequeo de
  salud viaja dentro del protocolo que PX4 vigila, en vez de depender sólo del latido offboard.

### 3.4 PX4-Avoidance (ROS 1, archivado, pero con lógica de seguridad reutilizable)
- [código] `avoidance/src/avoidance_node.cpp:10-12`, `:62-83`: `timeout_critical = 0.5 s` sin nube → **hover** y estado
  `MAV_STATE_CRITICAL`; `timeout_startup = 5 s` de gracia; `timeout_termination = 15 s` → `MAV_STATE_FLIGHT_TERMINATION`;
  publica el estado como componente 196 (MAV_COMPONENT_ID_AVOIDANCE).
- [código] `local_planner/src/nodes/planner_functions.cpp:15-62`: descarta NaN, recorta por rango mín/máx del sensor,
  guarda memoria de puntos con edad máxima (`max_point_age_s`) y exige `min_num_points_per_cell` para aceptar una celda
  del histograma (rechazo de puntos sueltos).

### 3.5 Aerostack2 — `as2_core/src/aerial_platform.cpp` (rama main)
- [código] Timer de comandos a `cmd_freq` (default 100 Hz) y de info a `info_freq` (10 Hz) (`:48-49`, `:146-152`);
  `sendCommand()` no manda nada si no está conectado/armado/en offboard (`:364-385`); eventos `KILL_SWITCH` y
  `EMERGENCY_HOVER` (`:340-362`) llevan a estado EMERGENCY y `ownStopPlatform()`.
- [análisis, verificado parcialmente] En la plataforma no vi un timeout de referencias: reenvía la última referencia
  mientras `has_new_references_` siga en true. Si el control de alto nivel muere, la plataforma podría seguir con el
  último comando; no revisé si `as2_motion_controller` u otros nodos agregan su propio timeout. **[NO VERIFICADO]**.

---

## 4. LISTA DE VERIFICACIÓN — entorno real

Formato: **problema → mitigación típica → valor/parámetro típico → fuente**.

1. **Latencia sensor→actuación y distancia de frenado**
   → velocidad máxima función de la distancia libre: `d_frenado = v·t + v²/(2a)`; incluir edad del dato.
   → PX4: `CP_DELAY` 0.4 s (0.1–0.5 tracking + ≤0.2 visión externa); ArduPilot usa ≤ 1 m/s² para la evasión.
   Ej.: v = 2 m/s, t = 0.4 s, a = 3 m/s² → 0.8 + 0.67 = **1.47 m** + margen; con a = 1 m/s² → **2.8 m**.
   → [código] `CollisionPrevention.cpp:544-563`, `TrajMath.hpp:61-71`, `AC_Avoid.cpp:233,763,1492`; [doc] `collision_prevention.md:101-115`.
2. **Edad de los datos / sensor colgado**
   → caducar cada sector y frenar sin datos.
   → PX4 0.5 s por sector, Hold a los 5 s; ArduPilot cara 1 s, MAVLink 500 ms; PX4-Avoidance 0.5 s → hover.
   → [código] `CollisionPrevention.hpp:189-190`, `.cpp:161,210-224`; `AP_Proximity_Boundary_3D.h:30`; `AP_Proximity_MAV.cpp:27`; `avoidance_node.cpp:11`.
3. **Ruido y retornos espurios (polvo, lluvia, insectos)**
   → pasa-bajos por sector, mínimo de puntos por celda, rango mín/máx; filtrar en la Jetson (PX4 no filtra).
   → `PRX_FILT` 0.25 Hz; `min_num_points_per_cell`; `PRXn_MIN/MAX`.
   → [código] `AP_Proximity.cpp:63-69`, `AP_Proximity_Boundary_3D.cpp:99-118`; `planner_functions.cpp:62`. Papers: ver §5.
4. **Auto-impactos (hélices, patas, tren, cables propios)**
   → zonas angulares ignoradas / caja de recorte del cuerpo; distancia mínima del sensor.
   → `PRXn_IGN_ANGx/WIDx` (4 zonas), `PRXn_MIN`; PX4: `min_distance` del mensaje fija `d_keep >= min_distance`.
   → [código] `AP_Proximity_Params.cpp:32-110`, `AP_Proximity_Backend.cpp:55-85`; `CollisionPrevention.cpp:186,396`.
5. **Suelo visto como obstáculo (despegue/aterrizaje, sensor inclinado)**
   → ignorar puntos cerca del suelo con altura de telémetro; desactivar evasión bajo cierta altura; proyectar al plano horizontal.
   → `PRX_IGN_GND=1`, `PRX_ALT_MIN` 1 m, `AVOID_ALT_MIN`, `OA_DB_ALT_MIN`; PX4 ignora sensores hacia abajo/arriba y proyecta con la actitud.
   → [código] `AP_Proximity_Utils.cpp:55-88`, `ArduCopter/avoidance.cpp:4-19`, `AP_OADatabase.cpp:166-181`; `ObstacleMath.cpp:8-18`.
6. **Orientación/montaje del sensor**
   → declarar orientación y corrección de yaw; un error de signo espeja el mapa.
   → PX4 `orientation`/`q`, `frame`, `angle_offset`; ArduPilot `PRXn_ORIENT`, `PRXn_YAW_CORR`.
   → [código] `ObstacleMath.cpp:38-84`; `AP_Proximity_Backend.cpp:44-48`; `AP_Proximity_MAV.cpp:146-150`.
7. **Varios sensores con distinto alcance/FOV**
   → gana la distancia más corta reciente; sectores sin cobertura bloqueados (PX4) o libres (ArduPilot).
   → [código] `CollisionPrevention.cpp:323-353`; `AP_Proximity_Boundary_3D.cpp:60-73`; [doc] `collision_prevention.md:93-99`.
8. **Oscilación / indecisión ante el obstáculo**
   → histéresis de rumbo, banda muerta de retroceso, límite de aceleración.
   → `OA_BR_CONT_RATIO` 1.5, `OA_BR_CONT_ANGLE` 75°, `AVOID_BACKUP_DZ` 0.1 m, `AVOID_ACCEL_MAX` 3 m/s², `CP_GUIDE_ANG` 30°.
   → [código] `AP_OABendyRuler.cpp:381-417`; `AC_Avoid.cpp:99-105,329-356`; [doc] `collision_prevention.md:117-125`.
9. **Obstáculo que se acerca solo (persona) o vehículo dentro del margen**
   → retroceso activo limitado; caducidad corta de obstáculos en la base de datos.
   → `AVOID_BACKUP_SPD` 0.75 m/s; `OA_DB_EXPIRE` 10 s (0 = nunca, "terrible for dynamic ones").
   → [código] `AC_Avoid.cpp:75-81,1290-1303`; `AP_OADatabase.cpp:53-60`.
10. **Planificador lento o colgado**
    → planificar en hilo aparte, invalidar resultados viejos, frenar si no hay resultado.
    → OA 1 Hz, timeout 3 s → parar en el punto de frenado.
    → [código] `AP_OAPathPlanner.cpp:34-35,226-251`; `AC_WPNav_OA.cpp:181-201`.
11. **Pérdida de comunicación companion↔FC**
    → latido + acción configurable; timeout de comandos de velocidad.
    → PX4 `COM_OF_LOSS_T` 1 s, `COM_OBL_RC_ACT` (0 Position, 5 Hold, 4 Land...); ArduPilot `GUID_TIMEOUT` 3 s; modos externos ROS 2: 300 ms/50 ms/~1 s.
    → [código] `commander_params.c:340,369`, `offboardCheck.cpp:46-65`; `ArduCopter/Parameters.cpp:861-867`; `externalChecks.hpp:69-75`.
12. **Percepción muerta pero latido vivo**
    → atar el latido/salud a la frescura de las entradas; reportar falla al FC.
    → `checkArmingAndRunConditions()`; PX4-Avoidance publica `MAV_STATE_CRITICAL`.
    → [código] `mode.hpp:134-137`; `avoidance_node.cpp:62-83`.
13. **Latencia oculta en la marca de tiempo**
    → poner timestamp de captura; activar sincronización de tiempo.
    → `UXRCE_DDS_SYNCT=1`; `timestamp=0` ⇒ PX4 usa "ahora".
    → [código] `Tools/msg/templates/ucdr/msg.h.em:166-169`, `uxrce_dds_client/module.yaml:88-96`.
14. **Ruido en la frontera de sectores / sectores sin dato**
    → frontera = mínimo de sectores vecinos; distancia mínima de frontera 0.6 m.
    → [código] `AP_Proximity_Boundary_3D.cpp:123-173`, `.h:27-28`.
15. **Incertidumbre angular a distancia (haz ancho)**
    → inflar obstáculos proporcionalmente a la distancia.
    → radio = d·tan(`OA_DB_BEAM_WIDTH` = 5°).
    → [código] `AP_OADatabase.cpp:132,156`.
16. **Geocercas**
    → cercas como obstáculos virtuales en la evasión simple y en Dijkstra con margen; en PX4 la cerca es una
      acción de failsafe, no un limitador de velocidad.
    → ArduPilot: `AVOID_ENABLE` bit 0, `OA_MARGIN_MAX` como margen de cerca. PX4 v1.17: `GF_ACTION` default 2 (Hold;
      4 = Terminate), `GF_MAX_HOR_DIST`/`GF_MAX_VER_DIST` (0 = off), `GF_SOURCE`, `GF_PREDICT` (experimental, "may cause flyaways").
      [análisis] conviene además una cerca blanda en la Jetson (el área de `map_mission`) más chica que la de PX4.
    → [código] `AC_Avoid.h:15-19`, `AC_Avoid.cpp:160-220`, `AP_OADijkstra.cpp:46-125`; PX4 `src/modules/navigator/geofence_params.c:62-119`.

Ítems que dependen de bibliografía externa (distorsión por movimiento, vidrio, cables, sol/IR, deriva/multipath,
efecto suelo/pared, ráfagas, obstáculos dinámicos, carga de CPU): ver §5.

---

## 5. Problemas del entorno real con fuentes externas (continuación de la lista)

Nivel de verificación: **[fuente: resumen]** = confirmé título/autores/lugar y la afirmación en el resumen o en una
página del autor, pero NO leí el texto completo; **[análisis]** = deducción propia.

17. **Distorsión por movimiento de lidar giratorio (deskew)**
    → corregir cada punto con la pose en su instante (IMU integrada) antes de usar la nube; requiere timestamp por punto.
    → IMU ≥ 200 Hz recomendada por LIO-SAM; [análisis] a 10 Hz de giro y 2 m/s el barrido dura 100 ms → hasta ~0.2 m de
      desfase entre principio y fin del barrido, y a 90°/s de yaw, hasta 9° de error angular (casi 2 sectores de 5°).
    → [fuente: resumen] Shan et al., *LIO-SAM*, IROS 2020, arXiv:2007.00258 (https://arxiv.org/abs/2007.00258);
      repo https://github.com/TixiaoShan/LIO-SAM (`deskewPoint()` usa el tiempo relativo de cada punto).
18. **Lluvia, nieve, niebla, polvo**
    → filtros de outliers con radio que crece con la distancia (DROR), filtros estadísticos (DSOR), redes de de-noising;
      mínimo de puntos por celda; ignorar retornos muy cercanos.
    → [fuente: resumen] Charron et al. 2018 (DROR) citado por Kurup & Bos, *DSOR*, arXiv:2109.07078
      (https://arxiv.org/pdf/2109.07078) — **no leí el paper original de DROR**; Heinzler et al., *CNN-based Lidar Point
      Cloud De-Noising in Adverse Weather*, arXiv:1912.03874 (https://arxiv.org/pdf/1912.03874).
    → [fuente: resumen] Phillips, Guenther & McAree, *When the Dust Settles: The Four Behaviors of LiDAR in the Presence of
      Fine Airborne Particulates*, J. Field Robotics 34(5):985-1009, 2017: los retornos de polvo aparecen en el **borde
      delantero de la nube** (no como ruido aleatorio) — es decir, un pasa-bajos no alcanza: la nube de polvo se ve como pared.
    → Insectos: **[NO VERIFICADO]** no encontré una fuente específica; [análisis] se comportan como outliers aislados de
      1–2 puntos cercanos: los cubre el criterio de puntos mínimos por celda + persistencia temporal (≥ 2 barridos).
19. **Auto-impactos (hélices, patas, cables propios)**
    → filtro de cuerpo con URDF/caja ("contains" + "shadow test"), además de zonas angulares ignoradas.
    → [fuente: página del paquete] `robot_body_filter` (https://index.ros.org/p/robot_body_filter/), binarios ROS 1
      (Melodic/Noetic); para ROS 2 hay ports de `robot_self_filter` (no evalué su calidad). ArduPilot `PRXn_IGN_ANGx` (§2.2).
    → [análisis] las hélices giran: en una nube aparecen como anillo intermitente; con `free_ray_bins` finos pueden
      bloquear sectores enteros. Recortar una caja del tamaño del dron + 0.1–0.2 m.
20. **Vidrio, espejos y superficies especulares**
    → el lidar ve el vidrio sólo cerca de incidencia normal (retorno de alta intensidad) y en otros ángulos atraviesa o
      refleja; detección por intensidad + geometría, o fusión con ultrasonido/radar; memoria en el mapa de lo visto de frente.
    → [fuente: resumen] Weerakoon et al., *TOPGN: Real-time Transparent Obstacle Detection using Lidar Point Cloud
      Intensity*, arXiv:2408.05608 (https://arxiv.org/abs/2408.05608) (≈50 Hz en CPU móvil; vidrio, acrílico, PVC);
      Zhao, Yang & Schwertfeger, *Mapping with Reflection*, arXiv:1909.12483 (https://arxiv.org/pdf/1909.12483v1.pdf).
    → [análisis] para este workspace: no "vaciar" una celda ocupada por un único rayo que la atraviesa (ya está
      `occupied_miss_scale` en CLAUDE.md); eso es justo lo que protege paredes de vidrio vistas una vez de frente.
21. **Cables y ramas finas**
    → aumentar resolución angular/multi-eco, detección dedicada por visión, margen extra y velocidad baja cerca de
      vegetación; no confiar en "libre" cuando el objeto es más fino que la separación entre rayos.
    → [análisis] un cable de diámetro w a distancia d sólo se garantiza si la separación angular entre rayos < w/d:
      w = 1 cm a 5 m exige < 0.11° (la mayoría de los lidar 3D de drones están en 0.1°–0.4° horizontales y peor en vertical).
    → [fuente: resumen] Madaan, Maturana & Scherer, *Wire Detection using Synthetic Data and Dilated Convolutional Networks
      for UAVs*, IROS 2017, pp. 3487-3494 (https://www.ri.cmu.edu/publications/wire-detection-using-synthetic-data-and-dilated-convolutional-networks-for-unmanned-aerial-vehicles):
      > 3 Hz en Jetson TX2 a 480×640, AP 0.73 en el dataset USF.
22. **Cámaras de profundidad bajo sol (IR)**
    → luz estructurada (Kinect v1, Xtion) falla al sol; estéreo activo (RealSense D400) sigue funcionando con textura
      natural pero con más ruido y alcance dependiente de escena; validar `depth` válido por sector y degradar velocidad.
    → [fuente: resumen] *Kinect Range Sensing: Structured-Light versus Time-of-Flight Kinect*,
      arXiv:1505.05459 (https://arxiv.org/pdf/1505.05459) (autores: Sarbolandi, Lefloch & Kolb según mi memoria —
      **[NO VERIFICADO]**, arxiv.org no resuelve desde este entorno): la versión de luz estructurada no entrega datos con luz
      de fondo > ~1 µW en su ensayo. Keselman et al., *Intel RealSense Stereoscopic Depth Cameras*, arXiv:1705.05548
      (https://arxiv.org/pdf/1705.05548): el proyector y el sol aportan IR; el estéreo aprovecha la luz solar.
      Product brief D400: "Max range 10m+, varies depending on ... scene and light conditions". Afirmación de ruido
      amplificado por sol fuerte: preprint 2025 no revisado por pares (https://www.preprints.org/manuscript/202511.0498) — débil.
23. **Deriva de la estimación de estado y multipath GNSS cerca de paredes**
    → usar las compuertas del EKF y failsafes por precisión; preferir odometría lidar/visual cerca de estructuras;
      planificar lejos de zonas de mala geometría satelital; el mapa (grilla) queda en el marco del EKF: si el EKF salta, el
      mapa queda corrido → limpiar/recentrar o usar el sensor en marco cuerpo para la evasión reactiva.
    → PX4 v1.17 [código]: `EKF2_GPS_CHECK` = 2047 (todas las verificaciones; las de deriva sólo en tierra quieto),
      `EKF2_REQ_EPH` 3 m, `EKF2_REQ_SACC` 0.5 m/s, `EKF2_REQ_NSATS` 6, `EKF2_REQ_HDRIFT` 0.1 m/s, `EKF2_GPS_P_GATE` 5 σ,
      `EKF2_GPS_DELAY` 110 ms (`src/modules/ekf2/params_gnss.yaml`); `EKF2_EV_DELAY` 0 ms por defecto (odometría externa:
      **hay que medirlo**); `COM_POS_FS_EPH` 5 m y `COM_VEL_FS_EVH` 1 m/s disparan failsafe por precisión
      (`commander_params.c:538,553`).
    → [fuente: resumen] PolyU (autores no verificados), *A New Path Planning Algorithm Using a GNSS Localization Error Map for UAVs in
      an Urban Area* (https://research.polyu.edu.hk/en/publications/a-new-path-planning-algorithm-using-a-gnss-localization-error-map/):
      multipath/NLOS ≈ 50 m de error en zona urbana, dependiente del sitio. No encontré una fuente primaria para "pared única
      a pocos metros" en interiores/exteriores: **[NO VERIFICADO]** el orden de magnitud en ese caso.
24. **Efecto suelo, efecto pared/techo y ráfagas**
    → margen mayor cerca de superficies; no planear rutas pegadas a paredes ni pasar por debajo de techos a poca
      distancia; limitar velocidad en exteriores con viento; usar la estimación de viento para reducir velocidad.
    → [fuente: resumen] Sanchez-Cuevas, Heredia & Ollero, *Characterization of the Aerodynamic Ground Effect and Its
      Influence in Multirotor Control*, Int. J. Aerospace Eng. 2017, art. 1823056 (acceso abierto:
      https://idus.us.es/handle/11441/94910): el efecto suelo en multirrotores difiere del helicóptero y hay efecto
      "parcial" (sólo algunos rotores). Mismos autores, *Experimental Approach to the Aerodynamic Effects Produced in
      Multirotors Flying Close to Obstacles*, ROBOT 2017, DOI 10.1007/978-3-319-70833-1_60 (preprint:
      https://zenodo.org/record/2621464/files/2017-ROBOT-AerodynamicEffectsModelling.pdf): paredes, techos y combinaciones.
      **No leí los números** (cuántos diámetros de rotor); [NO VERIFICADO] la regla usual "efecto suelo < ~1–2 diámetros".
    → PX4 publica `/fmu/out/wind` (en `dds_topics.yaml`, rate 1 Hz) — [análisis] puede usarse para bajar la velocidad máxima.
25. **Obstáculos dinámicos (personas, vehículos)**
    → caducidad corta de obstáculos, retroceso activo, velocidad relativa en el cálculo de frenado, latencia baja.
    → [fuente: resumen] Falanga, Kleber & Scaramuzza, *Dynamic Obstacle Avoidance for Quadrotors with Event Cameras*,
      Science Robotics 5(40), eaaz9712, 2020: latencia total 3.5 ms con cámaras de eventos frente a "tens of milliseconds"
      de los sistemas habituales; esquiva a velocidades relativas de hasta 10 m/s.
    → [análisis] con mapas que "vacían lento" (`occupied_miss_scale`) una persona que pasó deja una estela ocupada: es el
      compromiso opuesto al del vidrio; la evasión reactiva debe mirar la nube actual (sectores) y no sólo la grilla.
    → [código] ArduPilot `OA_DB_EXPIRE` 10 s, `AVOID_BACKUP_SPD` 0.75 m/s (§2).
26. **Carga de CPU y latencia del software en la Jetson**
    → hilos separados para planificación (ArduPilot OA a 1 Hz en hilo propio; A* en otro hilo en este workspace), lazo de
      control que nunca bloquea, QoS BEST_EFFORT/KEEP_LAST 1 para sensores, medir y publicar la latencia; reportar la
      salud al FC.
    → [fuente: resumen] Casini, Blaß, Lütkebohle & Brandenburg, *Response-Time Analysis of ROS 2 Processing Chains Under
      Reservation-Based Scheduling*, ECRTS 2019 (https://people.mpi-sws.org/~bbb/papers/pdf/ecrts19-rev1.pdf): el executor
      de ROS 2 no prioriza callbacks como un planificador de tiempo real (análisis sobre ROS 2 Crystal; el executor cambió desde entonces).
    → PX4: `COM_CPU_MAX` 95 % es para el CPU del FMU, no de la Jetson (`commander_params.c:762`); la Jetson puede informar
      su carga por `/fmu/in/onboard_computer_status` (está en `dds_topics.yaml`) — [NO VERIFICADO] qué hace PX4 con ese dato.
    → [NO VERIFICADO] valores de jitter en Jetson; no encontré una fuente primaria en esta sesión.
27. **Pérdida de enlace con la estación terrena / RC**
    → [código] PX4 `COM_DL_LOSS_T` 10 s (GCS), `COM_RC_LOSS_T` 0.5 s ("the vehicle will use the last supplied setpoint
      until the timeout triggers") (`commander_params.c:98,139`).

---

## 6. Resumen de diferencias PX4 vs ArduPilot que importan para la Jetson [análisis sobre código citado]

| Tema | PX4 v1.17 CP | ArduPilot |
|---|---|---|
| Modos | Position / Position Slow (stick) | Loiter, AltHold, **Guided velocidad**; OA en Auto/Guided/RTL |
| Sin datos en un sector | bloquea (`CP_GO_NO_DATA=0`) | libre (frontera a 100 m) |
| Caducidad | 0.5 s por sector; Hold a 5 s sin datos | 1 s por cara; MAVLink 0.5 s → NoData |
| Filtro | ninguno | LPF 0.25 Hz por cara |
| Fusión | sensor de menor alcance gana en rango | distancia más corta reciente gana |
| Frenado | `v_max` con jerk + `CP_DELAY` + edad del dato | `sqrt_controller` / `sqrt(2ad)` con a ≤ 1 m/s² |
| Retroceso | aceleración negativa lineal dentro de `CP_DIST` | `AVOID_BACKUP_SPD` 0.75 m/s con banda muerta 0.1 m |
| Resolución | 72 × 5° (2D) | 8 × 45° × 5 capas (3D) |
| `distances = 0` | obstáculo pegado | descartado |

---

## 7. Mitigaciones priorizadas para `/home/user/ros_map` [análisis; no se modificó el workspace]

1. **No contar con la CP de PX4 en Offboard**: limitar en la Jetson `v_max = −2a²/j + sqrt(4a⁴/j² + 2·a·d_eff)`
   (o la versión simple `sqrt(2·a·d_eff)`) con `d_eff = d_sector − d_keep − v·(t_latencia + edad_dato)`, por sector,
   proyectando la velocidad comandada sobre cada sector. Usar a ≈ 1–3 m/s² y t ≈ 0.4 s como punto de partida.
2. **Edad de datos**: si la nube/sectores tienen > 0.5 s, comandar velocidad cero (hover) y avisar; > 5 s, pedir Hold.
   Atar `offboard_control_mode` (latido) a la salud de la percepción, o migrar a un modo externo de
   `px4-ros2-interface-lib` y reportar fallas en `checkArmingAndRunConditions()`.
3. **`/fmu/in/obstacle_distance` como red de seguridad**: `frame=12`, `increment=5`, `UINT16_MAX` desconocido,
   `max_distance+1` libre, nunca `0`, `timestamp` = captura del sensor, ≥ 10 Hz, `CP_DIST > 0` (medido desde el sensor,
   con margen por hélices), `COM_OBL_RC_ACT=0` si hay RC, para que el piloto o el failsafe queden protegidos.
4. **Filtrado antes de decidir**: recorte del cuerpo (caja dron + 0.1–0.2 m o zonas angulares), rango mín/máx,
   puntos mínimos por sector, persistencia ≥ 2 barridos, deskew si el lidar gira y el dron rota rápido.
5. **Anti-oscilación**: histéresis de rumbo tipo BendyRuler (`ratio 1.5`, `ángulo 75°`), banda muerta de retroceso
   (~0.1 m), límite de cambio de velocidad (~3 m/s²); ya existe debounce en el workspace (CLAUDE.md).
6. **Retroceso** limitado (~0.75 m/s) si algo entra dentro del margen, en vez de sólo frenar.
7. **Planificador**: si el A* no devuelve en ~3 s o devuelve para otro destino, frenar en el punto de frenado.
8. **Cercas**: cerca blanda en la Jetson + `GF_ACTION=2` (Hold) / `GF_MAX_HOR_DIST` / `GF_MAX_VER_DIST` en PX4.
9. **Estimación**: medir `EKF2_EV_DELAY` si se usa odometría externa; vigilar `COM_POS_FS_EPH`; la evasión reactiva en
   marco cuerpo no depende del EKF, la grilla sí.
