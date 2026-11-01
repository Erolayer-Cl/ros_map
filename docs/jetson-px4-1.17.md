# Despliegue en el dron real: Jetson + ROS 2 Humble + PX4 v1.17

Cómo pasar de la simulación al dron: Jetson Orin como computadora de a bordo
(ROS 2 Humble), Pixhawk con PX4 v1.17 conectado por uXRCE-DDS (sin MAVROS),
lidar Livox Mid-360. Al final, una lista de verificación para el primer vuelo.

> Volar autónomo cerca de paredes es riesgoso. Las primeras pruebas, siempre
> con piloto de seguridad con la radio en la mano, en un lugar despejado, y
> empezando por la simulación (`run_tests.sh`, `launch_fake_sim.sh`, Unreal).

---

## 1. Arquitectura a bordo

```
 Livox Mid-360 ──Ethernet──┐
                           │        Jetson Orin (Ubuntu 22.04, ROS 2 Humble)
                           ├──► livox_ros_driver2 ──/livox/lidar──► mapper_node ──/map, /walls──►
                           │                                            │   /obstacles/sectors
 Pixhawk (PX4 v1.17)       │                                            ▼
   TELEM2 ◄──UART 921600──► MicroXRCEAgent ◄──/fmu/*──────────────► mission_node
   (uxrce_dds_client)                       (vehicle_local_position_v1, setpoints, comandos)
```

- Sólo `mission_node` publica en `/fmu/in/trajectory_setpoint`,
  `/fmu/in/offboard_control_mode` y `/fmu/in/vehicle_command`.
- `mapper_node` puede publicar además `/fmu/in/obstacle_distance` para la
  Collision Prevention de PX4 (§5).

## 2. Jetson

1. **JetPack 6.x** (Ubuntu 22.04). ROS 2 Humble se instala por apt como en
   cualquier Ubuntu 22.04 (`ros-humble-ros-base` alcanza; `ros-dev-tools` para
   compilar). Con JetPack 5 (Ubuntu 20.04) Humble sólo corre desde fuente o en
   Docker.
2. Modo de potencia alto y relojes fijos para que el lazo de control no tenga
   saltos: `sudo nvpmodel -m 0 && sudo jetson_clocks` (el modo exacto depende
   del módulo; ver `nvpmodel -q`).
3. Dependencias de este repo: `sudo apt install python3-numpy python3-yaml
   ros-humble-tf2-ros ros-humble-nav-msgs ros-humble-visualization-msgs
   ros-humble-std-srvs`.
4. Compilar:
   ```bash
   git clone --recursive https://github.com/Erolayer-Cl/ros_map.git
   cd ros_map && source /opt/ros/humble/setup.bash
   colcon build --symlink-install
   source install/setup.bash
   ./run_tests.sh rapido    # las pruebas unitarias también corren en la Jetson
   ```

### 2.1 px4_msgs tiene que coincidir con el firmware

`src/px4_msgs` es un submódulo en la rama `release/1.17`. Si el Pixhawk tiene
otro firmware (otra versión o una compilación propia), sincronizar los mensajes
como explica el README del repo de referencia (copiar `msg/*.msg`,
`msg/versioned/*.msg` y `srv/*.srv` del checkout de PX4). Síntoma de
desincronización: el tópico aparece en `ros2 topic list` pero no llega nada.

**Sufijos de versión en v1.17:** `VehicleStatus` y `VehicleLocalPosition`
tienen `MESSAGE_VERSION = 1`, y el cliente uXRCE-DDS publica en
`<tópico>_v<versión>`: hay que suscribirse a `/fmu/out/vehicle_status_v1` y
`/fmu/out/vehicle_local_position_v1`. `VehicleAttitude`, `VehicleOdometry`,
`TrajectorySetpoint` y `VehicleCommand` son versión 0 (sin sufijo). Esto ya
está en los YAML (`px4.*_topic`); si se cambia de firmware, verificar con
`ros2 topic list | grep fmu`.

## 3. Pixhawk ↔ Jetson (uXRCE-DDS)

Cableado: TELEM2 del Pixhawk (TX, RX, GND; **no** conectar 5 V entre placas)
al UART del header de 40 pines de la Jetson (`/dev/ttyTHS0` o `/dev/ttyTHS1`
según el carrier: `ls /dev/ttyTHS*`), o con un adaptador USB-serie
(`/dev/ttyUSB0`). En Pixhawk 6X/6C con Ethernet se puede usar UDP.

Parámetros de PX4 v1.17 (QGroundControl → Parameters), y reiniciar:

| Parámetro | Valor | Para qué |
|---|---|---|
| `UXRCE_DDS_CFG` | `TELEM2` (102) — o `Ethernet` (1000) | Puerto del cliente uXRCE-DDS. |
| `SER_TEL2_BAUD` | `921600 8N1` | Velocidad del UART (no aplica con Ethernet). |
| `UXRCE_DDS_DOM_ID` | `0` | Tiene que coincidir con `ROS_DOMAIN_ID` de la Jetson. |
| `UXRCE_DDS_SYNCT` | `1` (por defecto) | Sincronización de tiempo con el agente. |
| `UXRCE_DDS_AG_IP` / `UXRCE_DDS_PRT` | IP de la Jetson / `8888` | Sólo con Ethernet. |

Agente en la Jetson (compilado desde `Micro-XRCE-DDS-Agent` v2.4.3):

```bash
sudo usermod -aG dialout $USER          # una vez, y volver a entrar
MicroXRCEAgent serial --dev /dev/ttyTHS1 -b 921600
# con Ethernet: MicroXRCEAgent udp4 -p 8888
```

Si el UART elegido tiene una consola serie activa (servicio `nvgetty` u otro
`getty`), deshabilitarla o el agente recibe basura. Para que arranque solo,
un servicio systemd con `Restart=always`.

Verificación: `ros2 topic hz /fmu/out/vehicle_local_position_v1` (≈ 50-100 Hz).

## 4. Livox Mid-360

1. Red: el Mid-360 viene con IP fija `192.168.1.1xx` (los últimos dígitos del
   número de serie). Poner la interfaz Ethernet de la Jetson en
   `192.168.1.50/24` y ajustar `MID360_config.json` del driver (IP de host y
   del lidar).
2. Driver: compilar `livox_ros_driver2` para Humble (`./build.sh humble`
   dentro del workspace del driver, con Livox-SDK2 instalado).
3. Lanzar con `xfer_format: 0` (PointCloud2 estándar):
   `ros2 launch livox_ros_driver2 msg_MID360_launch.py`.
4. Montaje: medir el offset del centro óptico respecto del centro de masa y
   ponerlo en `config/mapper_mid360.yaml` (`cloud.mount_xyz`, FRD, metros).
   Recomendado inclinarlo ~20° hacia abajo para ver obstáculos bajos
   (`cloud.mount_rpy_deg: [0.0, -20.0, 0.0]`), ver `docs/sensores.md`.
5. Lanzar el mapper con esa configuración:
   ```bash
   ros2 launch uav_wall_mapping mapper.launch.py \
     config:=$(ros2 pkg prefix uav_wall_mapping)/share/uav_wall_mapping/config/mapper_mid360.yaml
   ros2 launch uav_mission mission.launch.py
   ```

Prueba en banco (dron sin hélices, en la mano): caminar con el dron frente a
una pared y mirar `/walls/markers` en Foxglove. Si la pared aparece del lado
contrario o girada, el montaje (`cloud.mount_rpy_deg`, `cloud.axes`) está mal.

## 5. Collision Prevention de PX4 como red de seguridad

Con `px4_obstacle_distance.enabled: true` (ya activo en `mapper_mid360.yaml`),
el mapper publica los 72 sectores en `/fmu/in/obstacle_distance` (marco
`MAV_FRAME_LOCAL_NED`, centímetros, `max_distance + 1` = libre). PX4 sólo usa
esto en **modo Position** (no en Offboard): protege cuando el piloto toma el
control con la radio. Parámetros (PX4 v1.17):

| Parámetro | Valor sugerido | Nota |
|---|---|---|
| `CP_DIST` | `1.5` | Distancia mínima permitida. Negativo = desactivado. Se mide desde el sensor, no desde las hélices: dejar margen. |
| `CP_DELAY` | `0.4` (defecto) | Retardo de sensor + seguimiento de velocidad. |
| `CP_GUIDE_ANG` | `30` (defecto) | Cuánto puede desviar el comando hacia un lado más libre. |
| `CP_GO_NO_DATA` | `0` | Con 0, no deja moverse hacia sectores sin datos. |
| `MPC_POS_MODE` | `Acceleration based` | Requisito de la Collision Prevention. |

**Ojo:** si la Collision Prevention está activa y `mapper_node` deja de
publicar más de 0.5 s, PX4 avisa *"No range data received, no movement
allowed"* y **no deja mover el dron en horizontal en modo Position**. El piloto
tiene que pasar a Altitude o Stabilized. Practicarlo en simulación antes.

## 6. Parámetros de PX4 para el vuelo offboard

| Parámetro | Valor | Nota |
|---|---|---|
| `COM_OF_LOSS_T` | `1.0` (defecto) | Segundos sin setpoints antes del failsafe de offboard. `mission_node` publica a 20 Hz desde un hilo propio; el A* corre en otro. |
| `COM_OBL_RC_ACT` | `5` (Hold) o `0` (Position) | Qué hace PX4 si se pierde offboard con radio presente. |
| `COM_RCL_EXCEPT` | `4` sólo en simulación | En el dron real dejar `0`: perder la radio tiene que disparar el failsafe también en offboard. |
| `NAV_RCL_ACT`, `NAV_DLL_ACT` | los de siempre (no 0 en real) | En `settings.json` de simulación se ponen en 0. |
| `MPC_XY_VEL_MAX` | ≥ `cruise_speed` | El nodo manda la velocidad con la zanahoria + feedforward. |

Si el piloto saca el dron de OFFBOARD (cambia de modo en la radio o en QGC),
`mission_node` pasa a `PREEMPTED` y **deja de mandar setpoints**: no pelea por
el control.

## 7. Rendimiento en la Jetson

- `cloud.downsample: 0.1` y `cloud.max_rate: 10` limitan la nube que procesa el
  mapper (el Mid-360 da 200 000 puntos/s; con 10-20 mil por nube alcanza).
- La extracción de paredes corre cada `publish.walls_period` s en otro hilo;
  si tarda más de 1 s, el nodo avisa: subir el período o `map.resolution`.
- `map.max_size` (400 m) acota la RAM de la grilla; el mapa 3D se limita a
  2 millones de vóxeles.
- Mirar la carga con `tegrastats` durante un vuelo de prueba.

## 8. Lista para el primer vuelo

1. [ ] `./run_tests.sh` pasa en la Jetson.
2. [ ] `ros2 topic hz /fmu/out/vehicle_local_position_v1` y `/livox/lidar` publican.
3. [ ] Prueba en banco (sin hélices): `/walls/markers` dibuja la pared donde está.
4. [ ] `ros2 topic echo /obstacles/sectors --field ranges` cambia al acercar una mano.
5. [ ] En tierra, desarmado: `/mission/start` responde y el dron arma (sin hélices) — luego `/mission/abort`.
6. [ ] Piloto de seguridad listo para pasar a Position (el nodo cede el control).
7. [ ] Primer vuelo: `waypoints` a un punto despejado, `cruise_speed: 0.8`, altura 2 m.
8. [ ] Recién después, una pared real con `/mission/start`, y por último `/mission/map`.
9. [ ] Ctrl-C en `mission_node` con el dron armado manda LAND: probarlo en simulación primero.
