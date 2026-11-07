# 01 — Código de planificadores/evasión usados en vuelos reales: qué hacen para funcionar fuera de la simulación

Fecha: 2026-11-01. Todo lo citado se leyó en clones `--depth 1` dentro de
`/tmp/claude-0/-home-user-ros-map/ed946d96-4b5a-536d-8bc7-aabb24974ec4/scratchpad/investigacion/repos/`
(abajo, `repos/` = ese directorio). Las rutas son relativas a `repos/`. No se ejecutó código.
Cuando algo **no se encontró en el código** se dice explícitamente ("NO ENCONTRADO").

Índice: 1 ego-planner/ego-swarm · 2 Fast-Planner · 3 FUEL · 4 FASTER/MADER/PANTHER · 5 PX4-Avoidance
(+ PX4 Collision Prevention del firmware) · 6 CMU (aéreo y terrain_analysis) · 7 voxblox + mav_voxblox_planning ·
8 nvblox · 9 KR autonomous flight · 10 aerostack2 · 11 tabla consolidada · 12 implicancias para ros_map.
No clonado/no leído: el mapper de FASTER (`global_mapper_ros`, está en GitLab), papers en PDF.

---

## 1. ZJU-FAST-Lab/ego-planner (y ego-planner-swarm)

Repos: https://github.com/ZJU-FAST-Lab/ego-planner , https://github.com/ZJU-FAST-Lab/ego-planner-swarm
Volado con D435/D435i + VINS en hardware propio (README: "Improved ROS-RealSense Driver", pruebas con D435/D435i).

Archivos clave:
- `ego-planner/src/planner/plan_env/src/grid_map.cpp` (mapa log-odds + ray casting + inflado)
- `ego-planner/src/planner/plan_manage/src/ego_replan_fsm.cpp` (FSM, chequeo de colisión, emergencia)
- `ego-planner/src/planner/plan_manage/src/planner_manager.cpp` (`EmergencyStop`)
- `ego-planner/src/planner/plan_manage/launch/advanced_param.xml` (parámetros)
- `ego-planner/src/planner/plan_manage/src/traj_server.cpp` (yaw)
- `ego-planner-swarm/src/planner/plan_env/src/grid_map.cpp` (timeout de depth/odom)
- `ego-planner-swarm/src/planner/drone_detect/` (borrar otros drones de la imagen de profundidad)

### 1.1 Mapa
- **Ray casting con conteo hit/miss por voxel por frame** (no por rayo): cada voxel tocado en un frame
  recibe UNA actualización; gana "hit" si `hits >= misses`. Evita que un voxel atravesado por muchos rayos
  y golpeado por pocos se borre por mayoría de rayos rasantes:
  ```cpp
  // grid_map.cpp, raycastProcess()
  double log_odds_update =
      md_.count_hit_[idx_ctns] >= md_.count_hit_and_miss_[idx_ctns] - md_.count_hit_[idx_ctns] ? mp_.prob_hit_log_ : mp_.prob_miss_log_;
  ```
- **Rango máximo de integración**: puntos más lejos que `max_ray_length` (4.5 m) se recortan a esa
  distancia y se marcan como **libres** (miss), no ocupados:
  ```cpp
  if (length > mp_.max_ray_length_) {
    pt_w = (pt_w - md_.camera_pos_) / length * mp_.max_ray_length_ + md_.camera_pos_;
    vox_idx = setCacheOccupancy(pt_w, 0);
  ```
- **Pixeles de profundidad = 0 (inválidos) y > maxdist se tratan como "libre hasta max_ray_length"**,
  y los < `depth_filter_mindist` (0.2 m) se descartan:
  ```cpp
  // grid_map.cpp, projectDepthImage()
  if (*row_ptr == 0) { depth = mp_.max_ray_length_ + 0.1; }
  else if (depth < mp_.depth_filter_mindist_) { continue; }
  else if (depth > mp_.depth_filter_maxdist_) { depth = mp_.max_ray_length_ + 0.1; }
  ```
  Ojo: esto borra obstáculos cuando la cámara devuelve 0 (vidrio, negro mate, saturación). Es una elección
  optimista; ver §6 abajo.
- **Decaimiento temporal**: NO hay decaimiento por tiempo. Lo que sale de la ventana local
  (`local_update_range` 5.5×5.5×4.5 m + `local_map_margin`) se resetea a "desconocido":
  ```cpp
  // clearAndInflateLocalMap()
  md_.occupancy_buffer_[idx] = mp_.clamp_min_log_ - mp_.unknown_flag_;
  ```
  Y los voxels fuera de la ventana local que reciben actualización se fuerzan a `clamp_min_log_` antes de sumar.
- **Clamping**: `p_min=0.12`, `p_max=0.90`, `p_hit=0.65`, `p_miss=0.35`, `p_occ=0.80` (advanced_param.xml).
  Con esos valores un voxel saturado ocupado necesita ~5 misses para pasar a libre (histeresis acotada).
- **Inflado**: cubo de `ceil(obstacles_inflation/resolution)` celdas; `obstacles_inflation=0.099`, `resolution=0.1`
  → 1 celda. El margen real lo da el optimizador (`optimization/dist0 = 0.5` m).
- **Techo virtual**: `virtual_ceil_height=2.5` se marca como ocupado en el mapa inflado.
- **Desconocido = libre** para el planificador: `getInflateOccupancy()` sólo devuelve el buffer inflado de
  ocupados; no hay rama para desconocido. (En la ruta de nube de puntos `cloudCallback` ni siquiera hay ray
  casting: se resetea la ventana local y se inflan los puntos del frame actual → mapa sin memoria.)

### 1.2 Ruido de sensor
- `skip_pixel=2` (submuestreo), `depth_filter_margin=1` px de borde descartado, `mindist=0.2`, `maxdist=5.0`.
- Existe un **chequeo de consistencia por reproyección contra el frame anterior** (`depth_filter_tolerance=0.15`)
  pero está **desactivado en el código** (`if (false)` en `projectDepthImage`).
- Filtro de auto-impactos (propio dron): NO ENCONTRADO (lo resuelve `mindist=0.2`).
- ego-swarm: `drone_detect` borra de la imagen de profundidad los píxeles que caen sobre otros drones del
  enjambre (`drone_detect/config/default.yaml`: `drone_width 0.5`, `drone_height 0.2`, `max_pose_error 0.4`).
  Es el equivalente a un filtro de "objetos conocidos".

### 1.3 Tiempos
- Sincronización pose↔profundidad con `message_filters::ApproximateTime` (cola 100). Pose de cámara =
  `odom * cam2body` fijo en código (`md_.cam2body_` matriz hardcodeada).
- ego-swarm suscribe la extrínseca cámara-IMU **estimada online por VINS** (`/vins_estimator/extrinsic`) y usa
  `tcpNoDelay()` en odometría.
- ego-swarm agrega **timeout de profundidad/odometría** → parada de emergencia:
  ```cpp
  // ego-planner-swarm grid_map.cpp
  if ( md_.flag_use_depth_fusion && (ros::Time::now() - md_.last_occ_update_time_).toSec() > mp_.odom_depth_timeout_ )
  { ROS_ERROR("odom or depth lost! ..."); md_.flag_depth_odom_timeout_ = true; }
  // ego_replan_fsm.cpp
  if (map->getOdomDepthTimeout()) { ROS_ERROR("Depth Lost! EMERGENCY_STOP"); changeFSMExecState(EMERGENCY_STOP, "SAFETY"); }
  ```
  `odom_depth_timeout` default 1.0 s.
- Compensación de latencia/deskew: NO ENCONTRADO (no hay deskew; la cámara es global shutter).

### 1.4 Dinámica
- `max_vel=2.0`, `max_acc=3.0` (run_in_sim.launch), `max_jerk=4`, `feasibility_tolerance=0.05`,
  `bspline/limit_ratio=1.1` (re-asignación de tiempo si se excede 10 %).
- `planning_horizon=7.5` con comentario: "always set to 1.5 times greater than sensing horizon".
- **Replanificación**: desde el **estado de referencia de la trayectoria actual** (no desde la odometría) para
  continuidad: `start_pt_ = info->position_traj_.evaluateDeBoorT(t_cur)` (+vel, +acc).
- Replanifica por distancia recorrida: `thresh_replan=1.5 m` desde el inicio, no replanifica a < `thresh_no_replan=2.0 m` del objetivo.
- **Chequeo de colisión periódico de la trayectoria** a 20 Hz (`safety_timer_ 0.05 s`), muestreando cada
  10 ms, **sólo los primeros 2/3** de la trayectoria (lo lejano todavía no es confiable).
- **Yaw** mira 1.0 s adelante sobre la trayectoria (`traj_server/time_forward=1.0`), con tasa limitada
  `YAW_DOT_MAX_PER_SEC = PI` (traj_server.cpp) → la cámara apunta hacia donde se va.

### 1.5 Fallbacks
- Si el chequeo encuentra colisión: intenta replanificar **inmediatamente** (3 intentos con inicializaciones
  distintas en `planFromCurrentTraj`); si falla y la colisión está a < `emergency_time` (1.0 s) → `EMERGENCY_STOP`;
  si está más lejos → `REPLAN_TRAJ`:
  ```cpp
  if (t - t_cur < emergency_time_) // 0.8s of emergency time
  { ROS_WARN("Suddenly discovered obstacles. emergency stop! ..."); changeFSMExecState(EMERGENCY_STOP, "SAFETY"); }
  ```
- `EmergencyStop` = B-spline con todos los puntos de control en la posición actual (hover en el lugar);
  sale de emergencia cuando `odom_vel_.norm() < 0.1`.
- Margen por error de seguimiento: NO ENCONTRADO explícito (implícito en `dist0=0.5`).

### 1.6 Dinámicos / finos / vidrio
- Dinámicos: sólo otros drones del enjambre (trayectorias compartidas, `swarm_clearance`, chequeo en
  `planner_manager.cpp::checkCollision` cada 0.03 s). Obstáculos dinámicos no cooperativos: NO.
- Finos y vidrio: NO ENCONTRADO. El tratamiento de depth=0 como libre es desfavorable para vidrio.

---

## 2. HKUST-Aerial-Robotics/Fast-Planner

Repo: https://github.com/HKUST-Aerial-Robotics/Fast-Planner (origen del `grid_map` de ego-planner)

Archivos: `Fast-Planner/fast_planner/plan_env/src/sdf_map.cpp`, `.../plan_manage/src/kino_replan_fsm.cpp`,
`.../plan_manage/src/planner_manager.cpp`, `.../plan_manage/launch/kino_algorithm.xml`.

- **Mapa**: mismo esquema log-odds + ray casting + **ESDF** (`esdf_slice_height`). Parámetros reales en
  `kino_algorithm.xml`: `resolution 0.1`, `obstacles_inflation 0.099`, `min_ray_length 0.5`, `max_ray_length 4.5`,
  `depth_filter_maxdist 5.0`, `mindist 0.2`, `depth_filter_margin 2`, `skip_pixel 2`, `p_hit 0.65`, `p_miss 0.35`,
  `p_min 0.12`, `p_max 0.90`, `p_occ 0.80`, `virtual_ceil_height 2.5`.
- **Chequeo de colisión con distancia (ESDF), sólo hasta 6 m adelante**, umbral 0.1 m:
  ```cpp
  // planner_manager.cpp, checkTrajCollision()
  while (radius < 6.0 && t_now + fut_t < local_data_.duration_) {
    fut_pt = local_data_.position_traj_.evaluateDeBoor(tm + t_now + fut_t);
    double dist = edt_environment_->evaluateCoarseEDT(fut_pt, -1.0);
    if (dist < 0.1) { distance = radius; return false; }
  ```
- **Meta dentro de obstáculo**: si la ESDF en la meta es ≤ 0.3 m busca una meta alternativa en anillos
  (r de 0.5 a 2.5 m, θ cada 30°, z ±0.3) maximizando distancia (`kino_replan_fsm.cpp::checkCollisionCallback`).
  Muy útil para metas que resultan pegadas a una pared real.
- Dinámica: `max_jerk 4`, `clearance_threshold 0.2`, `search/margin 0.2`, `local_segment_length 6.0`,
  `optimization/dist0 0.4`. `manager/dynamic_environment` existe (predicción de objetos en `obj_predictor.cpp`)
  pero el launch lo deja en 0.
- Replan desde el estado de referencia de la trayectoria (`REPLAN_TRAJ`: `start_pt_ = position_traj_.evaluateDeBoorT(t_cur)`).
- Fallback: si falla el replan vuelve a `GEN_NEW_TRAJ` (no hay estado de emergencia explícito en Fast-Planner).
- README §5: con imágenes de menor resolución (256×144) poner `skip_pixel=1`; `k_depth_scaling_factor=1000`
  depende del dispositivo.

---

## 3. HKUST-Aerial-Robotics/FUEL (exploración, volado en hardware)

Repo: https://github.com/HKUST-Aerial-Robotics/FUEL
Archivos: `FUEL/fuel_planner/plan_env/src/map_ros.cpp`, `.../plan_env/src/sdf_map.cpp`,
`.../exploration_manager/src/fast_exploration_fsm.cpp`, `.../exploration_manager/launch/algorithm.xml`,
`.../path_searching/src/kinodynamic_astar.cpp`.

- **Desconocido = ocupado** (modo pesimista) para exploración real: `sdf_map/optimistic=false`,
  `search/optimistic=false` en `algorithm.xml`. En la ESDF:
  ```cpp
  // sdf_map.cpp, updateESDF3d() rama !optimistic_
  return (md_->occupancy_buffer_inflate_[adr] == 1 ||
          md_->occupancy_buffer_[adr] < mp_->clamp_min_log_ - 1e-3) ? 0 : max;
  ```
  y en el A* kinodinámico: `if (!optimistic_ && ...getOccupancy(pos) == SDFMap::UNKNOWN)` → descarta.
- Inflado subido a `0.199` m (comentado el 0.099 de Fast-Planner) → el equipo agrandó el margen para exploración.
- Proyección de profundidad: igual que ego (`*row_ptr == 0 || depth > maxdist` → `depth = maxdist` libre;
  `< mindist` → descarta). Sincronización depth↔pose con `message_filters` (`SyncPolicyImagePose`, cola 100);
  ignora el frame si la cámara está fuera del mapa.
- **Compensación de latencia de planificación**: replanifica desde el estado de la trayectoria **0.2 s en el
  futuro** (`fsm/replan_time 0.200`), y sólo publica la nueva cuando su `start_time` ya llegó:
  ```cpp
  // fast_exploration_fsm.cpp, PLAN_TRAJ
  double t_r = (ros::Time::now() - info->start_time_).toSec() + fp_->replan_time_;
  fd_->start_pt_ = info->position_traj_.evaluateDeBoorT(t_r);
  ...
  // PUB_TRAJ
  double dt = (ros::Time::now() - fd_->newest_traj_.start_time).toSec();
  if (dt > 0) { bspline_pub_.publish(fd_->newest_traj_); ...
  ```
  Si el plan falla → `static_state_ = true` (siguiente plan desde la odometría = hover).
- Replan periódico: `thresh_replan3 = 1.5 s`, al cubrirse la frontera (`thresh_replan2 0.5`) o al final (`thresh_replan1 0.5`).
- `safetyCallback` a 20 Hz → `checkTrajCollision` → replan.
- Yaw: `exploration/yd = 60°/s`, `ydd = 90°/s²` (límites de yaw para que la cámara llegue a mirar).
- `max_vel 2.0`, `max_acc 2.0` (exploration.launch).

---

## 4. mit-acl/faster (+ mader, panther)

Repos: https://github.com/mit-acl/faster , https://github.com/mit-acl/mader , https://github.com/mit-acl/panther
FASTER voló en hardware (gif `faster/imgs/uav_hw.gif`); el mapper (`global_mapper_ros`) está en GitLab, NO se clonó.

### 4.1 FASTER — `faster/faster/src/faster.cpp`, `faster/faster/param/faster.yaml`
- **Idea central para entorno real: siempre hay un plan de escape en espacio libre CONOCIDO**. Se resuelven
  dos trayectorias: `whole` (puede atravesar desconocido, rápida) y `safe` (desde un punto R de la whole,
  restringida a politopos que excluyen desconocido+ocupado). Sólo se comprometen el tramo A→R de la whole + la
  safe, que termina en reposo:
  ```cpp
  jps_manager_.cvxEllipsoidDecomp(JPS_whole, OCCUPIED_SPACE, l_constraints_whole_, poly_whole_out);
  ...
  jps_manager_.cvxEllipsoidDecomp(JPS_safe, UNKOWN_AND_OCCUPIED_SPACE, l_constraints_safe_, poly_safe_out);
  ...
  if (appendToPlan(k_end_whole, sg_whole_.X_temp_, k_safe, sg_safe_.X_temp_) != true)
  ```
  Si cualquiera de los dos QP falla → `return` y se sigue ejecutando el plan anterior (que ya termina en libre).
- **Latencia**: el nuevo plan arranca en el estado A, `deltaT_` muestras adelante del plan comprometido
  (`deltaT_ = 10` con `dc = 0.01 s` → 0.1 s): `k_end_whole = std::max((int)plan_.size() - deltaT_, 0); A = plan_[...]`.
- Parámetros (comentarios del yaml, "Hardware" explícito): `drone_radius 0.42`, `inflation_jps 0.47`,
  `Ra 4.0` (radio de planificación), `w_max 4.0 # ~4.0 for Hardware`, `v_max 5.0 #1.4`, `a_max 5.0 #1.4`,
  `j_max 8.0 #35`, `goal_radius 0.3`, `z_max 2`, `z_ground 0.0`. (Los `#1.4` sugieren valores usados antes en hardware.)
- Antes de avanzar gira a mirar la meta (`DroneStatus::YAWING` → `TRAVELING` cuando `|diff| < 0.04`).

### 4.2 MADER — `mader/mader/src/mader.cpp`, `mader/mader/param/mader.yaml`
- **Latencia adaptativa**: el estado de inicio del replan se toma `deltaT = factor_alpha × (muestras que tardó
  el replan anterior)`, saturado entre los límites de tiempo de cómputo:
  ```cpp
  deltaT_ = std::max(par_.factor_alpha * states_last_replan, 1.0);   // factor_alpha: 1.5
  deltaT_ = std::min(1.0 * deltaT_, 2.0 / par_.dc);
  mu::saturate(deltaT_, par_.lower_bound_runtime_snlopt / par_.dc, par_.upper_bound_runtime_snlopt / par_.dc); // 0.05..0.35 s
  ```
- **Check–Recheck** para dinámicos (otros agentes): tras optimizar verifica que no llegaron trayectorias nuevas
  que colisionen; si llegaron durante el chequeo, descarta el plan (`safetyCheckAfterOpt`, `have_received_trajectories_while_checking_`).
- Margen de seguimiento como parámetro: `alpha: 0.0 # [m] Error in position`, `beta` (desvío entre muestras),
  `gamma 0.1 s`. `w_max` con comentario `0.4 for Hardware`. `a_max.z` comentario "if > 9.81 the drone may flip".
- `Ra 4.0` con comentario "Should be < fov_depth" (`fov_depth 4.5`).

### 4.3 PANTHER — `panther/panther/src/tracker_predictor.cpp`, `panther/panther/param/{panther,predictor}.yaml`
- Obstáculos dinámicos NO cooperativos desde nube de puntos: TF a `world` **en el stamp de la nube** con
  espera de 20 ms, VoxelGrid 0.1, clustering euclídeo, asociación húngara, predicción por ajuste polinómico
  en ventana deslizante:
  ```cpp
  transform_stamped = tf_buffer_.lookupTransform("world", header_pcloud.frame_id, header_pcloud.stamp, ros::Duration(0.02));
  ...
  // pcl::ApproximateVoxelGrid<pcl::PointXYZ> sor; //this one crashes randomly on the NUC and Jetson
  pcl::VoxelGrid<pcl::PointXYZ> sor;
  ```
  `predictor.yaml`: `cluster_tolerance 0.3`, `min_cluster_size 15`, `max_cluster_size 600`,
  `min_dim_cluster_size 0.1`, `leaf_size_filter 0.1`, `meters_to_create_new_track 2.0`, `max_frames_skipped 0`.
- Persistencia de obstáculos: `max_seconds_keeping_traj: 1.0` (borra trayectorias de obstáculos no actualizadas en 1 s).
- Márgenes: `drone_radius 0.1` + `drone_radius_extra_safety 0.1`; `norminv_prob` (bandas de incertidumbre de la predicción).
- **Percepción activa**: costo `c_fov 40` para mantener el obstáculo en el FOV (`fov_x_deg 70`, `fov_y_deg 70`,
  `fov_depth 5.0`), `angle_deg_focus_front 90`. `ydot_max` comentario "0.4 for Hardware".

---

## 5. PX4/PX4-Avoidance (deprecado, pero el más parecido a tu arquitectura: histograma polar + PX4)

Repo: https://github.com/PX4/PX4-Avoidance
Archivos: `PX4-Avoidance/local_planner/src/nodes/planner_functions.cpp`, `.../local_planner.cpp`,
`.../local_planner_nodelet.cpp`, `.../star_planner.cpp`, `.../waypoint_generator.cpp`,
`.../cfg/LocalPlannerNode.cfg`, `PX4-Avoidance/avoidance/src/{avoidance_node,transform_buffer,common}.cpp`,
`PX4-Avoidance/avoidance/include/avoidance/histogram.h`, `PX4-Avoidance/safe_landing_planner/cfg/*.cfg`.

### 5.1 Mapa = nube "con memoria" + histograma polar
- Histograma de `ALPHA_RES = 6°` (60 azimut × 30 elevación; `histogram.h`). Tu proyecto usa 72 sectores (5°).
- **Memoria de puntos con edad y borrado por FOV** (clave): los puntos viejos se mantienen sólo si (a) su celda
  no fue poblada por la nube nueva, (b) son más jóvenes que `max_point_age_s` (20 s) y (c) están **fuera del FOV
  actual**. Si la celda está dentro del FOV y la cámara no ve nada ahí, el punto viejo se olvida (= espacio libre
  observado):
  ```cpp
  // planner_functions.cpp, processPointcloud()
  if (histogram_points_counter(p_ind.y(), p_ind.x()) < min_num_points_per_cell && xyzi.intensity < max_age &&
      !pointInsideFOV(fov, p_pol_fcu)) {
    final_cloud.points.push_back(toXYZI(toEigen(xyzi), xyzi.intensity + elapsed_s));
  ```
  (la edad se guarda en `intensity`).
- **Ruido**: celda (a 2° = `ALPHA_RES/3`) necesita `min_num_points_per_cell` puntos para contar (default 1);
  descarte por rango `min_sensor_range 0.2` / `max_sensor_range 15`; NaN descartados.
- **FOV medido de la nube real** (no del datasheet): `removeNaNAndGetMaxima` + `updateFOVFromMaxima`
  (`local_planner_nodelet.cpp`).
- **Distancia por bin = promedio** de los puntos del bin (`generateNewHistogram`), y para el FCU se comprime a 2D
  tomando el mínimo dentro de ±1 m de altura y ±10° de elevación (`compressHistogramElevation`, `vertical_cap = 1.0f`).
- **Costo de obstáculo suave (no binario) + suavizado angular**:
  ```cpp
  const float d = cost_params.obstacle_cost_param - obstacle_distance;          // obstacle_cost_param = 8.5 m
  const float distance_cost = obstacle_distance > 0 ? 5000.0f * (1 + d / sqrt(1 + d * d)) : 0.0f;
  ...
  unsigned int smooth_radius = ceil(smoothing_margin_degrees / ALPHA_RES);      // 40° → kernel cónico
  smoothPolarMatrix(distance_matrix, smooth_radius);
  ```
  y un costo de velocidad que penaliza cambiar de dirección (`velocity_cost_param 6000`) → anti-oscilación.
- **Star planner**: árbol A* de 40 nodos, 8 hijos, nodos a 2 m (`tree_node_distance_`), re-construyendo el
  histograma desde cada nodo con la misma nube (`star_planner.cpp`). Nodos a < 0.2 m de otro se descartan.

### 5.2 Tiempos
- **Buffer propio de TF con interpolación** (lineal en traslación, slerp en rotación) al **stamp de la nube**,
  ventana de 5 s; si el TF del instante todavía no llegó, espera (no extrapola):
  ```cpp
  // transform_buffer.cpp
  if (iterator->second.back().stamp_ < time) { ... "tf has not yet arrived"; return false; }
  ...
  tf::Vector3 translation = tf_earlier.getOrigin() * (1.f - tau) + tf_later.getOrigin() * tau;
  tf::Quaternion rotation = tf_earlier.getRotation().slerp(tf_later.getRotation(), tau);
  ```
- Throttle de nubes: se ignora una nube si llegó < 0.3 s después de la anterior de esa cámara
  (`pointCloudCallback`: `timeSinceLast() < ros::Duration(0.3)`), lazo de planificación `spin_dt_ = 0.1 s`.
  Espera a tener nube transformada de **todas** las cámaras antes de planificar.

### 5.3 Fallbacks (estados MAVLink al FCU)
```cpp
// avoidance_node.cpp, checkFailsafe()
if (since_last_cloud > timeout_termination && since_start > timeout_termination) {
  setSystemStatus(MAV_STATE::MAV_STATE_FLIGHT_TERMINATION);
} else if (since_last_cloud > timeout_critical && since_start > timeout_startup) {
  if (position_received_) { hover = true; setSystemStatus(MAV_STATE::MAV_STATE_CRITICAL); }
```
Defaults (`LocalPlannerNode.cfg`): `timeout_startup 5 s`, `timeout_critical 0.5 s` (→ hover + CRITICAL;
PX4 reacciona con su failsafe), `timeout_termination 15 s`. Sólo envía waypoints si el estado es ACTIVE.
- `WaypointGenerator` (máquina de estados: LOITER/TRY_PATH/ALTITUDE_CHANGE/...): en hover congela la posición
  (`hover_position_ = position_`).
- **Velocidad escalada por FOV**: si el waypoint queda fuera del FOV la velocidad va a 0, en el borde baja
  linealmente; además pasa-bajos de velocidad con τ = 0.9 s y velocidad ≤ distancia a la meta:
  ```cpp
  speed_ *= scaleToFOV(fov_fcu_frame_, p_pol_fcu);   // 0 fuera del FOV
  speed_ = alpha * speed_ + (1.f - alpha) * last_speed;
  ```
- **Suavizado del setpoint** con un sistema de 2º orden (`smoothing_speed_xy 10`, `smoothing_speed_z 3`).
- Lee los parámetros del PX4 (`MPC_ACC_HOR`, `MPC_JERK_MAX`, `MPC_XY_CRUISE`, …) por MAVLink
  (`px4ParamsCallback`) para que el planificador use los mismos límites que el controlador.
- Publica el histograma comprimido como `OBSTACLE_DISTANCE` (LaserScan) al FCU si `CP_DIST > 0` → la
  **Collision Prevention de PX4 actúa como segunda capa** (ver §5.5).

### 5.4 Safe landing planner (`safe_landing_planner/cfg/*.cfg`)
Grilla 10 m de celdas 0.25 m; celda aterrizable si `n_points ≥ 100`, `std_dev < 0.2`, vecinos con
`|Δmedia| < 0.1`; suavizado temporal exponencial `alpha 0.9` y decisión con histéresis `can_land_thr 0.4`,
`beta 0.9`; decide a `loiter_height 8 m`. `realsense_params.sh`: desactiva/reactiva autoexposición de la D435
"as it does not work at startup".

### 5.5 Lecciones del README (hardware)
- "Be careful when attaching the camera with a USB3 cable. USB3 might interfere with GPS and other signals.
  If possible, always use USB2 cables."
- Cámara soportada D435 con firmware 5.9.13.0; montaje cámara↔FCU se pasa como `tf_x..tf_roll`
  (`CAMERA_CONFIGS`).
- `rs_depthcloud.launch`: `depth_fps 15`, sin alinear a color, emisor por defecto.

### 5.6 Complemento: PX4 Collision Prevention (firmware, relevante para PX4 v1.17)
Archivos leídos (ya presentes en `repos/px4/`, copiados del firmware): `CollisionPrevention.cpp/.hpp`,
`TrajMath.hpp`, `collisionprevention_params.c`, `collision_prevention.md`.
- **Distancia de frenado con jerk + aceleración + retardo + edad del dato** por bin:
  ```cpp
  float delay_distance = curr_vel_parallel * _param_cp_delay.get();         // CP_DELAY default 0.4 s
  const hrt_abstime data_age = now - _data_timestamps[i];
  if (distance < max_range) { delay_distance += curr_vel_parallel * (data_age * 1e-6f); }
  const float stop_distance = distance - _min_dist_to_keep - delay_distance;
  ... computeMaxSpeedFromDistance(_param_mpc_jerk_max.get(), _param_mpc_acc_hor.get(), stop_distance, 0.f);
  ```
  ```cpp
  // TrajMath.hpp
  float b =  4.0f * sqr(accel) / jerk;
  float c = - 2.0f * accel * braking_distance - sqr(final_speed);
  float max_speed = 0.5f * (-b + sqrtf(sqr(b) - 4.0f * c));
  ```
- `RANGE_STREAM_TIMEOUT_US = 500 ms` (bin sin datos recientes = sin datos), `TIMEOUT_HOLD_US = 5 s`;
  `CP_GO_NO_DATA = 0` → no se permite moverse hacia direcciones sin cobertura.
- Doc: sólo en **Position mode** con `MPC_POS_MODE = Acceleration based` (no actúa en Offboard);
  "For external vision-based systems the sensor delay may be as high as 0.2s"; retardo de seguimiento típico
  0.1–0.5 s; pruebas iniciales a 4 m/s con `OBSTACLE_DISTANCE` a 10 Hz.

---

## 6. CMU — aerial_navigation_development_environment y autonomous_exploration_development_environment

Repos: https://github.com/caochao39/aerial_navigation_development_environment ,
https://github.com/HongbiaoZ/autonomous_exploration_development_environment (rama noetic).
Ambos son "development environments" cuyo `local_planner`/`terrain_analysis` es el mismo que corre en sus
robots reales (el de suelo en el SubT; el aéreo con cámara de profundidad + tracking camera).

### 6.1 Aéreo — `aerial_navigation_development_environment/src/local_planner/src/localPlanner.cpp`
- Planificador por **biblioteca de caminos precalculados** (4375 caminos en 25 grupos) que se invalidan con la
  nube; el camino se escala (`pathScale 2.0` → `minPathScale 1.0` en pasos de 0.5) si no hay libre; si nada
  está libre publica un camino de 1 punto en el origen (= parar/hover):
  ```cpp
  if (selectedGroupID < 0) { pathScale -= pathScaleStep; } else { break; }
  ...
  if (!pathPublished) { path.poses.resize(1); path.poses[0].pose.position.x = 0; ...
  ```
- **Ruido**: VoxelGrid `scanVoxelSize 0.2`; un camino se bloquea sólo con ≥ `pointPerPathThre = 2` puntos.
- **Sólo vuela hacia donde ve**: caminos cuyo extremo cae fuera del FOV del sensor (`sensorMaxYaw 40°`,
  `sensorMaxPitch 25°`) o por encima de `maxElev` se marcan como bloqueados:
  ```cpp
  if (yawDiff > sensorMaxYaw || pitchDiff > sensorMaxPitch || elev > maxElev) {
    clearPathList[i] += pointPerPathThre; continue; }
  ```
- **Memoria lateral corta** (la cámara no ve los costados): conserva puntos en un cilindro de
  `keepHoriDis 2.0 m` × `keepVertDis 1.0 m` alrededor del vehículo (`keepSurrCloud true`), re-filtrados con voxel.
- **Sincronización con latencia configurable**: cola circular de odometría y elección de la pose más cercana a
  `t - depthCloudDelay` (ojo: usa el tiempo de la última odom, la línea con el stamp de la nube está comentada):
  ```cpp
  //laserTime = laserCloud2->header.stamp.toSec() - depthCloudDelay;
  laserTime = odomTime[odomPointerLast] - depthCloudDelay;
  ```
  Offsets de montaje de cámara (`depthCamPitchOffset`, `depthCamX/Y/ZOffset`) y de la tracking camera
  (`trackingCamBackward`, `trackingCamScale`) como parámetros.
- Path follower (mismo launch, outdoor): `maxSpeed 4.0`, `minSpeed 1.0`, `stopDis 1.0`, `slowDis 4.0`,
  `maxRollPitch 30°`, `lookAheadScale 0.2`, `minLookAheadDis 0.4`, `maxRange 16`.

### 6.2 Suelo — `autonomous_exploration_development_environment/src/terrain_analysis/src/terrainAnalysis.cpp`
- **Decaimiento temporal con zona sin decaimiento**: un punto se conserva si es más nuevo que `decayTime`
  (2 s) **o** está a menos de `noDecayDis` (4 m) del vehículo; además borrado explícito (`clearingDis 8 m`) a pedido:
  ```cpp
  (laserCloudTime - systemInitTime - point.intensity < decayTime || dis < noDecayDis) &&
  !(dis < clearingDis && clearingCloud)
  ```
- Re-filtrado de cada vóxel de terreno cada `voxelPointUpdateThre 100` puntos o `voxelTimeUpdateThre 2 s`.
- **Obstáculos dinámicos**: `clearDyObs true` — una celda planar se considera "dinámica" (y se descarta) si los
  puntos acumulados caen dentro del FOV vertical actual (`minDyObsVFOV -16°..maxDyObsVFOV 16°`) pero el scan
  actual ya no tiene puntos ahí (contador `planarVoxelDyObs` que el scan nuevo resetea a 0).
  Es "borrado por visibilidad" análogo al de PX4-Avoidance.
- `noDataObstacle false` (no tratar "sin datos" como obstáculo), `minBlockPointNum 10`, `vehicleHeight 1.5`.

---

## 7. ethz-asl/voxblox + mav_voxblox_planning

Repos: https://github.com/ethz-asl/voxblox , https://github.com/ethz-asl/mav_voxblox_planning
(voxblox se usó en MAVs reales del ASL; `mav_local_planner` es el planificador local ESDF de ese grupo).

### 7.1 voxblox (TSDF → ESDF)
Archivos: `voxblox/voxblox/include/voxblox/integrator/tsdf_integrator.h`, `voxblox/voxblox/src/integrator/tsdf_integrator.cc`,
`voxblox/voxblox/include/voxblox/integrator/esdf_integrator.h`, `voxblox/voxblox/src/integrator/esdf_integrator.cc`,
`voxblox/voxblox_ros/src/tsdf_server.cc`, `voxblox/voxblox_ros/src/transformer.cc`.
- **Rango**: `min_ray_length_m 0.1`, `max_ray_length_m 5.0` (defaults). Puntos más lejos que el máximo se
  convierten en **rayos de borrado** (sólo espacio libre, sin superficie) si `allow_clear`:
  ```cpp
  // tsdf_integrator.h, isPointValid()
  if (ray_distance < config_.min_ray_length_m) { return false; }
  else if (ray_distance > config_.max_ray_length_m) {
    if (config_.allow_clear || freespace_point) { *is_clearing = true; return true; }
  ```
  Además hay un tópico `freespace_pointcloud` para integrar sólo espacio libre.
- **Anti-grazing** (rayos rasantes, la misma trampa que `sectors.py` de ros_map): un rayo no actualiza vóxeles
  que en este mismo frame son extremo (superficie) de otro rayo:
  ```cpp
  // tsdf_integrator.cc, MergedTsdfIntegrator::integrateVoxel()
  if (enable_anti_grazing) {
    if ((clearing_ray || global_voxel_idx != kv.first) &&
        voxel_map.find(global_voxel_idx) != voxel_map.end()) { continue; }
  ```
  (default `enable_anti_grazing = false`; parámetro ROS `anti_grazing`).
- **Peso por distancia** (ruido de estéreo crece con z²): `weight = 1/z²` (`getVoxelWeight`) y caída de peso
  detrás de la superficie (`use_weight_dropoff`). `max_weight 10000` → en voxblox no hay decaimiento temporal:
  NO ENCONTRADO (lo dinámico se borra sólo por re-observación; ver nvblox para decaimiento).
- **Tiempos**: la nube se encola y se integra **sólo cuando existe TF en su stamp** (no usa el último TF):
  ```cpp
  // transformer.cc
  // Previous behavior was just to use the latest transform if the time is in
  // the future. Now we will just wait.
  if (!tf_listener_.canTransform(to_frame, from_frame_modified, time_to_lookup)) { return false; }
  ```
  Cola máx. 10 nubes, si no se resuelve el TF descarta con error ("unable to look up transform timestamps or the
  processing is taking too long", `tsdf_server.cc`). Modo alternativo con cola propia de poses e interpolación
  con tolerancia `timestamp_tolerance_sec` (1 ms). `min_time_between_msgs_sec` para throttle.
- **ESDF y desconocido alrededor del robot** ("clear sphere"): el desconocido dentro de `clear_sphere_radius`
  (1.5 m) se "alucina" libre (para poder arrancar) y el desconocido dentro de `occupied_sphere_radius` (5 m) se
  marca ocupado:
  ```cpp
  // esdf_integrator.cc, addNewRobotPosition()
  if (!esdf_voxel.observed || esdf_voxel.hallucinated) { esdf_voxel.distance = config_.default_distance_m; ...
  ...
  // Second set all remaining unknown to occupied.
  if (!esdf_voxel.observed) { esdf_voxel.distance = -config_.default_distance_m; ...
  ```
  `esdf_max_distance_m 2.0`, `default_distance_m 2.0`, `min_distance_m 0.2`.

### 7.2 mav_local_planner (`mav_voxblox_planning/mav_local_planner/src/mav_local_planner.cpp`)
- Launch de referencia (`mav_local_planner/launch/firefly_mapping_planning.launch`): `voxel_size 0.20`,
  `robot_radius 0.6`, `clear_radius 1.0`, `occupied_sphere_radius 4.0`, `max_ray_length_m 10.0`,
  `min_time_between_msgs_sec 0.10`, `update_esdf_every_n_sec 0.25`, `replan_dt 0.25`, `command_publishing_dt 0.25`,
  `replan_lookahead_sec 1.0`, `v_max 2.0`, `a_max 2.0`, `planning_horizon_m 20.0`.
- **Replan desde 1 s adelante en el camino ya comandado** (`replan_lookahead_sec`): corta el plan en
  `path_index_ + replan_lookahead_sec/sampling_dt` y re-planifica desde ahí.
- **Chequeo de colisión** del resto del plan con ESDF y margen: `getMapDistance(p) < robot_radius - 0.1` → colisión.
  Si el replan falla pero el plan actual sigue libre → continúa ("Continuing existing solution"); si falla y el
  actual colisiona → `abort()` (borra la trayectoria = queda en el último setpoint) + `dealWithFailure()`, que
  intenta una meta intermedia (`goal_selector_`) y tras `max_failures_ = 5` abandona el waypoint.

---

## 8. nvidia-isaac/nvblox

Repo: https://github.com/nvidia-isaac/nvblox (usado en Isaac ROS sobre Jetson con cámaras estéreo reales).
Archivos: `nvblox/nvblox/include/nvblox/integrators/projective_integrator_params.h`,
`.../tsdf_decay_integrator_params.h`, `.../occupancy_decay_integrator_params.h`, `.../freespace_integrator_params.h`,
`.../esdf_integrator_params.h`, `.../internal/decay_integrator_base.h`, `nvblox/nvblox/include/nvblox/mapper/mapper.h`,
`nvblox/nvblox/src/mapper/mapper.cpp`.
- **Rango**: `projective_integrator_max_integration_distance_m = 7.0` (cámara), `10.0` (lidar);
  `truncation_distance_vox = 4`.
- **Profundidad inválida (≤0) → decaimiento en vez de "libre"** (lo opuesto a ego-planner):
  ```
  "projective_tsdf_integrator_invalid_depth_decay_factor", -1.0,
  "Whenever a voxel projects into an invalid (<=0) depth image pixel, we decay the voxel with this factor
   instead of integrating. This allow us to rapidly prune outliers stemming from dynamic objects and/or
   invalid sensor data. A negative value ... disables the effect"
  ```
  y preprocesado opcional que **dilata las regiones inválidas** de la imagen (`do_depth_preprocessing`,
  `depth_preprocessing_num_dilations 4`) para no integrar los bordes ruidosos alrededor de huecos.
- **Peso máximo bajo para escenas dinámicas**: `projective_integrator_max_weight = 5` ("Setting this number higher
  will lead to higher-quality reconstructions but worse performance in dynamic scenes"); peso `inverse_square`.
- **Decaimiento temporal explícito**: `tsdf_decay_factor 0.95` por llamada, umbral `1e-3`; ocupación decae hacia
  0.5 (`free_region_decay_probability 0.55`, `occupied_region_decay_probability 0.4`, `occupancy_decay_to_free false`
  → lo decaído vuelve a DESCONOCIDO, no a libre). **Excluye de la decadencia lo que se ve en la última vista**
  (`decayTsdfExcludeLastView()`) o un radio alrededor del robot (`DecayBlockExclusionOptions::exclusion_radius_m`).
- **Dinámicos**: (a) chequeo de discrepancia (`projective_dynamic_tsdf_integrator_discrepancy_threshold_m`, desactivado
  por defecto, aplicado sólo a vóxeles con peso ≥ 2): si la medición difiere más que el umbral, se invalida el vóxel;
  (b) **capa de "freespace" de alta confianza** con histéresis temporal: `max_tsdf_distance_for_occupancy_m 0.15`,
  `max_unobserved_to_keep_consecutive_occupancy_ms 200`, `min_duration_since_occupied_for_freespace_ms 1000`,
  `min_consecutive_occupancy_duration_for_reset_ms 2000`, `check_neighborhood true`.
- ESDF 2D: `esdf_slice_min_height 0`, `esdf_slice_max_height 1`, `esdf_slice_height 1`, `esdf_integrator_max_distance_m 2`.

---

## 9. KumarRobotics/kr_autonomous_flight

Repo: https://github.com/KumarRobotics/kr_autonomous_flight (vuelos reales en bosques con lidar Ouster;
carpeta `autonomy_real/`).
Archivos: `kr_autonomous_flight/autonomy_core/map_plan/map_plan_launch/config/mapper.yaml`,
`.../map_plan/mapper/src/voxel_mapper.cpp`, `.../map_plan/mapper/src/tf_listener.cpp`,
`.../state_machine/state_machine_core/src/local_global_replan_server.cpp`,
`.../state_machine/action_trackers/src/{stopping_policy,trajectory_tracker_upgraded}.cpp`,
`.../control/control_launch/config/tracker_params_mp.yaml`, `autonomy_real/real_experiment_launch/config/tracker_params.yaml`.
- **Mapa por contador con decaimiento** ("ray trace" sólo convierte DESCONOCIDO→libre; no borra ocupados):
  ```cpp
  // voxel_mapper.cpp, addCloud()
  if (val_decay_ > 0) { double max_decay_range = max_range * 2.0; decayLocalCloud(pos, max_decay_range); }
  ...
  if (map_[idx] == val_unknown_) { map_[idx] = val_free_; ...
  // Add val_add to the voxel whenever a point lies in it. The voxel will be
  // occupied after N*T > (val_occ - val_even) / val_add
  ```
  `mapper.yaml`, con comentario clave: `decay_times_to_empty: 30 # ... the noisier your state estimator is, the
  smaller this value should be (when using ground-truth pose, set this as 0)`. Global `resolution 0.5`,
  `max_raycast_range 30`, local `resolution 0.25`, `max_raycast_range 13`, `robot_r 0.50`, `robot_h 0.25`
  (inflado anisótropo: z menos que xy), `num_point_cloud_skip 3`.
- **TF en el stamp de la nube** con espera de 0.4 s (`tf_listener.cpp`: `lookupTransform(target, source, time, ros::Duration(0.4))`).
- **Replan por "épocas"**: el planificador local corre cada `1/local_replan_rate` (2 Hz) y planifica desde el
  estado de la trayectoria en la **próxima época** (latencia = 1 época). Timeout de planificación local 1.0 s.
- **Fallbacks**:
  - Mapa local no actualizado en > 2 s (tasa mínima 0.5 Hz) → `AbortReplan()` + **stopping policy**:
    "Probably due to LIDAR packets loss or computation, check (1) LIDAR connection, and (2) computation headroom!"
  - Fallos/timeout del planificador local → reintenta; al agotar `max_local_plan_trials` → `AbortReplan()`.
  - Stopping policy = frenado de **jerk constante** (`acc_xy_des 6.0`, `jerk_xy_des 3.0`, `acc_z_des 2.0`,
    `jerk_z_des 1.0`); avisa si la aceleración inicial supera la del frenado ("Either plan less aggressive
    trajectories or increase stopping policy max acceleration!").
  - **Error de seguimiento**: si `|odom - traj| > max_pos_error` (0.75 m, recomendado 0.5–1.0) y el dron va
    adelantado → aborta la misión ("you need to slow down your execution or tune gains!").
- Límites: planificador `max_v 3.0`, `max_a 4.0`, `max_j 2.0` (comentarios: 3D recomendado `max_a 2`, `max_j 2`),
  `max_v_z 1.5`. Seguidor real (`autonomy_real/.../tracker_params.yaml`): `path_tracker v_max 2.5`, `a_max 0.2`,
  `robot_r 0.2`, `yaw_speed 0.35`; `stopping_policy default_a_des 1.0`, `default_j_des 2.0`.

---

## 10. aerostack2 (ROS 2 — mismo ecosistema que ros_map)

Repo: https://github.com/aerostack2/aerostack2
Archivos: `aerostack2/as2_map_server/plugins/scan2occ_grid/src/scan2occ_grid.cpp`,
`aerostack2/as2_map_server/plugins/*/config/plugin_default.yaml`,
`aerostack2/as2_behaviors/as2_behaviors_path_planning/plugins/a_star/src/{a_star,a_star_searcher}.cpp`,
`aerostack2/as2_behaviors/as2_behaviors_path_planning/config/behavior_default.yaml`, `aerostack2/as2_core/src/aerial_platform.cpp`.
- Mapa 2D: depth/pointcloud → LaserScan (`scan_height 1 px`, `range_min 0.45`, `range_max 10`; para nube
  `min_height 0`, `max_height 1.0`) → grilla 0.1 m 300×300. `inf`/`NaN` → `range_max` (libre hasta el máximo),
  y > `scan_range_max` se recorta (libre).
- Fusión **aditiva asimétrica con "candado"** para obstáculos:
  ```cpp
  aux.setTo(-miss_confidence_, aux == 0);  // free with weight -> 10
  aux.setTo(hit_confidence_, aux == 100);  // occupied with weight -> 40
  ...
  // Keeping obstacles
  aux.setTo(100, cv::Mat(occ_grid_data) > 80);
  ```
  (una celda > 80 queda ocupada para siempre: sin decaimiento) + cierre morfológico sólo para limpiar el libre
  ("obstacles not filtered").
- TF del punto en su stamp con espera de 0.5 s (`lookupTransform(..., point.header.stamp, Duration 0.5)`).
- A*: dilata obstáculos `ceil(safety_distance/res)` (`safety_distance 1.0`), **desconocido = ocupado**
  (`unknown_as_free = false`), y **fuerza libre un cuadrado alrededor del dron** antes de dilatar ("Supposing that
  drone current cells are free") para no quedar "encerrado" por ruido o inflado. Si no hay camino → rechaza la meta.
- Plataforma: comandos a `cmd_freq 100 Hz`; en estado `EMERGENCY` envía `ownStopPlatform()` continuamente.

---

## 11. Tabla consolidada: consideración de entorno real → cómo la resuelve cada proyecto → valor típico

| Consideración real | Cómo la resuelven (proyecto / archivo) | Valor típico |
|---|---|---|
| Borrar espacio libre (ray casting) | log-odds con 1 actualización por voxel por frame, hit gana si hits ≥ misses (ego `grid_map.cpp`); TSDF con rayos de borrado (voxblox); KR sólo DESCONOCIDO→libre y lo ocupado se va por decaimiento; PX4-Avoidance olvida puntos viejos sólo si caen dentro del FOV actual | p_hit 0.65 / p_miss 0.35 / clamp 0.12–0.90 / p_occ 0.80 |
| Rayos rasantes que borran paredes | anti-grazing: no actualizar vóxeles que son superficie de otro rayo del mismo frame (voxblox `integrateVoxel`); regla hits≥misses por frame (ego) | default off en voxblox |
| Rango máx. de integración | más allá: rayo sólo de libre recortado (ego, voxblox, FUEL); descartado (KR, PX4-Avoidance) | 4.5 m (ego/Fast/FUEL, D435), 5 m voxblox, 7 m nvblox cámara, 13–30 m KR lidar |
| Rango mínimo / auto-impactos | descartar < min (todos); drone_detect borra otros drones (ego-swarm) | 0.1–0.5 m (0.2 ego/PX4; 0.45 aerostack2) |
| Profundidad inválida (0/NaN) | = libre hasta max (ego, FUEL, aerostack2); = decaer el voxel (nvblox, opcional) + dilatar huecos | nvblox: 4 dilataciones |
| Decaimiento temporal | ninguno + ventana local que resetea a desconocido (ego/Fast); edad máx. fuera del FOV (PX4-Av.); decayTime salvo cerca del robot (CMU); N decaimientos a vacío (KR); factor por llamada excluyendo la vista actual (nvblox) | PX4 20 s; CMU 2 s y sin decaimiento < 4 m; KR 30 scans; nvblox 0.95 |
| Persistencia lateral (FOV estrecho) | memoria cilíndrica alrededor del vehículo (CMU `keepSurrCloud`); memoria fuera de FOV (PX4-Av.) | CMU 2 m horiz × 1 m vert |
| Desconocido | libre (ego, Fast); ocupado (FUEL `optimistic=false`, aerostack2 A*); trayectoria de escape sólo en conocido-libre (FASTER); libre ≤1.5 m / ocupado ≤5 m alrededor del robot (voxblox) | — |
| Ruido / outliers | VoxelGrid + mín. puntos por camino (CMU, 2 pts); mín. puntos por celda (PX4-Av., 1); clustering con tamaño mín. (PANTHER 15 pts); submuestreo `skip_pixel` + margen de borde (ego); peso 1/z² (voxblox, nvblox) | voxel 0.1–0.2 m |
| Inflado / radio | cubo/esfera de inflado + margen del optimizador (ego 0.1 + dist0 0.5); FUEL 0.2; FASTER 0.47; KR robot_r 0.5 (z 0.25); aerostack2 safety 1.0; mav_local_planner robot_radius 0.6 | 0.2–0.6 m + margen |
| Sincronización pose↔nube | ApproximateTime depth+odom (ego, FUEL); TF en el stamp con espera (voxblox, KR 0.4 s, PANTHER 20 ms, aerostack2 0.5 s); buffer propio con interpolación slerp (PX4-Av.); cola de odom con delay fijo (CMU `depthCloudDelay`) | — |
| Latencia de planificación | planificar desde el estado de referencia futuro: FUEL +0.2 s; FASTER 10×10 ms; MADER 1.5× último tiempo de cómputo (0.05–0.35 s); mav_local_planner +1.0 s; KR próxima época (0.5 s) | 0.1–1 s |
| Continuidad del replan | arrancar del estado de la trayectoria (pos, vel, acc), no de la odometría (ego, Fast, FUEL, FASTER, MADER) | — |
| Límites dinámicos | v/a/j explícitos; leer los del FCU (PX4-Av. lee MPC_*) | v 2–3 m/s, a 2–4 m/s², j 2–8 m/s³ en hardware |
| Distancia de frenado | PX4 CP: v_max(dist − margen − v·(CP_DELAY + edad dato)) con jerk/acc; KR stopping policy jerk constante | CP_DELAY 0.4 s (0.1–0.5 típico, sensor visual hasta 0.2 s) |
| Chequeo de colisión de la trayectoria | 20 Hz sobre los primeros 2/3 (ego); hasta 6 m con ESDF < 0.1 m (Fast); ESDF < r − 0.1 (mav_local_planner) | 20 Hz |
| Frecuencia de replanificación | por distancia recorrida (ego 1.5 m), periódica (FUEL 1.5 s, KR 2 Hz, mav 4 Hz, PX4-Av. 10 Hz, FASTER/MADER continua) | 2–10 Hz |
| Sensor/odom cortado | parada de emergencia (ego-swarm 1 s); hover + MAV_STATE_CRITICAL (PX4-Av. 0.5 s), terminación a 15 s; stopping policy si mapa > 2 s sin actualizar (KR); "no movement" sin datos 0.5 s (PX4 CP) | 0.5–2 s |
| Planificador falla | seguir plan actual si sigue libre (mav, FASTER, MADER); emergencia si la colisión está a < 1 s (ego); hover desde odom (FUEL); N reintentos y abortar (KR 3, mav 5) | — |
| Error de seguimiento | abortar si ‖odom − ref‖ > umbral (KR 0.75 m); margen α en el planificador (MADER `alpha`); salir de emergencia con ‖v‖ < 0.1 (ego) | 0.5–1.0 m |
| Sólo volar hacia lo visible | velocidad × factor FOV (0 fuera) (PX4-Av. `scaleToFOV`); caminos fuera de FOV bloqueados (CMU `sensorMaxYaw 40°`); yaw mira 1 s adelante (ego); costo FOV (PANTHER); `Ra < fov_depth` (MADER) | — |
| Meta en/junto a obstáculo | buscar meta alternativa en anillos maximizando ESDF (Fast-Planner); meta intermedia (mav `goal_selector_`) | ESDF ≤ 0.3 m dispara |
| Arranque dentro de obstáculo/ruido | despejar esfera alrededor del robot (voxblox `clear_sphere` 1–1.5 m); cuadrado libre alrededor del dron (aerostack2 A*) | 1–1.5 m |
| Obstáculos dinámicos | clustering + Hungarian + polinomio (PANTHER); borrar celdas vistas vacías dentro del FOV (CMU `clearDyObs`); discrepancia + capa freespace con histéresis temporal (nvblox); trayectorias compartidas (ego-swarm, MADER) | PANTHER borra tracks a 1 s; nvblox libre tras 1 s sin ocupación |
| Obstáculos finos (cables, ramas) | NO ENCONTRADO tratamiento específico en ningún repo; sólo indirecto: min_num_points=1 (PX4-Av.), resolución fina, peso por distancia | — |
| Vidrio / superficies sin retorno | NO ENCONTRADO tratamiento específico; nvblox (invalid depth decay) y ego (0 → libre) muestran las dos políticas opuestas | — |
| Hardware de cámara | D435 firmware 5.9.13.0, cable USB2 por interferencia con GPS (PX4-Av. README); driver con emisor alternado por frame (ego README); reactivar autoexposición al inicio (PX4 safe landing) | depth 15 fps (PX4-Av.) |

---

## 12. Implicancias para ros_map (interpretación, no verificado contra su código)

Sólo como guía de lectura para las mejoras, a partir de la descripción del proyecto (grilla log-odds 2D,
vóxeles 3D, histograma polar de 72 sectores, zanahoria + feedforward, VFH+ con debounce, A* sobre grilla inflada):

1. El histograma polar se parece al de PX4-Avoidance: vale copiar (a) memoria con edad **sólo fuera del FOV**,
   (b) costo de obstáculo suave + suavizado angular (kernel de 40°) y costo por cambio de dirección, (c) velocidad
   escalada por FOV y pasa-bajos de velocidad.
2. Poner un **timeout de nube/odom** con acción definida (hover a ~0.5 s, aterrizaje/abortar más tarde), como
   PX4-Avoidance / ego-swarm / KR.
3. Limitar la velocidad de la zanahoria con la fórmula de frenado de PX4 CP (`computeMaxSpeedFromDistance`) usando
   la distancia del sector, el retardo total (sensor + edad del dato + seguimiento) y `MPC_ACC_HOR`/`MPC_JERK_MAX`.
4. Integrar la nube con la pose **en su stamp** (buffer de poses con interpolación, como `transform_buffer.cpp`),
   no con la última pose.
5. Decidir explícitamente la política para profundidad inválida (vidrio): no tratarla como "libre hasta max".
6. Abortar/frenar si el error de seguimiento supera ~0.5–1 m (KR) y planificar desde la referencia, no desde la odometría.


---

## 13. Lecciones documentadas fuera del código (búsqueda web, nivel de confianza menor)

- **PX4-Avoidance archivado y la interfaz de evasión retirada del firmware**: en el PX4 Sync del 15-ene-2025 se
  dijo que el lado ROS se archivó y que la interfaz del lado PX4 no se mantenía; una propuesta posterior la
  elimina ("Remove collision avoidance interface on PX4 side"). Consistente con eso, en la copia de
  `repos/px4/FlightTaskAuto.cpp/.hpp` y `dds_topics.yaml` (versión exacta no verificada) **no aparece**
  `ObstacleAvoidance`, `COM_OBS_AVOID` ni `vehicle_trajectory_waypoint`. Para PX4 v1.17 por uXRCE-DDS la evasión
  tiene que vivir en el companion (offboard), y la única capa de firmware disponible es Collision Prevention
  (sólo Position mode). Fuentes: https://discuss.px4.io/t/px4-sync-q-a-jan-15-2025/43274 ,
  https://discuss.px4.io/t/is-px4-avoidance-dead/30278
- **PX4-Avoidance en hardware**: errores de TF entre el frame de la cámara y `local_origin`; el mantenedor sugiere
  verificar primero que PX4 tenga una estimación de posición razonable (hilo de ~2018).
  Fuente: https://discuss.px4.io/t/px4-avoidance-on-hardware/8037
- **EGO-Planner(-v2) en issues** (no abiertos por mí; resumen del buscador): yaw que gira bruscamente cuando el
  obstáculo sale del FOV y movimiento demasiado agresivo; partes del mapa que se pierden por el filtro de
  profundidad; un mantenedor reprodujo en simulación un caso en que los puntos de la colisión no activaban la
  bandera de peligro en `checkCollisionCallback`. Fuentes: https://github.com/ZJU-FAST-Lab/ego-planner/issues/47 ,
  https://github.com/ZJU-FAST-Lab/EGO-Planner-v2/issues/12
- **Fast-Drone-250 (ZJU)**: issues abiertos de "no puedo volar el sistema real con el planificador" y "frecuencia de
  odometría demasiado baja" → la tasa de odometría es un requisito real. Fuente:
  https://issues.ecosyste.ms/hosts/GitHub/repositories/zju-fast-lab%2Ffast-drone-250/issues
