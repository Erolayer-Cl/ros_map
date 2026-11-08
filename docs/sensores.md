# Sensores para detectar y mapear paredes con un dron

Qué sensor conviene para que el dron **vea paredes planas e irregulares, las
esquive y las dibuje en un mapa**, montado en un dron con PX4 v1.17 y una
Jetson con ROS 2 Humble. Al final, la recomendación concreta y cómo se conecta
cada opción a este repo.

> Los precios son aproximados (USD, 2025-2026) y cambian mucho según el
> distribuidor; las especificaciones vienen de las hojas de datos de cada
> fabricante (fuentes al final). Verificá siempre la versión exacta del modelo
> antes de comprar: varios tienen variantes con nombres casi iguales.

---

## 1. Qué necesita el problema

| Requisito | Por qué | Qué descarta |
|---|---|---|
| **Cobertura angular densa** | Una pared irregular (salientes, escombros, zig-zag) tiene detalles que caen *entre* haces separados. El repo de referencia usaba 5 sensores de distancia a 0°, ±30° y ±90°: un saliente a 15° no lo ve nadie. | Anillos de rangefinders 1D como sensor principal. |
| **Campo vertical** | Las paredes inclinadas, los aleros y los objetos bajos están por encima o por debajo del plano del dron. Para medir inclinación hace falta ver la pared a varias alturas. | Lidars 2D (un solo plano). |
| **Alcance ≥ 10 m** | Para mapear hace falta ver lejos (menos vuelo) y para esquivar a 1.5-2 m/s hay que ver con 4-5 m de anticipación. | Cámaras de profundidad como sensor *único* de mapeo (útiles a < 5 m). |
| **Poco peso y consumo** | Cada 100 g resta minutos de vuelo. | Lidars automotrices de 1 kg. |
| **Driver ROS 2 Humble en ARM64 (Jetson)** | Si no hay driver, no hay datos. | Sensores sin SDK para Linux ARM. |

Conclusión: el sensor principal ideal es un **lidar 3D de 360°** liviano. Una
cámara de profundidad es un buen complemento (o la opción barata), y un lidar
1D hacia abajo + flujo óptico resuelven la altura y la posición en interiores.

---

## 2. Comparativa

### 2.1 Lidar 3D (sensor principal recomendado)

| Modelo | Campo de visión | Alcance | Puntos/s | Peso | Precio aprox. | Notas |
|---|---|---|---|---|---|---|
| **Livox Mid-360** | 360° × 59° (−7° a +52°) | 40 m @10 % reflect., 70 m @80 % | 200 000 | ~265 g | 750–1 000 | El estándar de facto en drones de investigación (FAST-LIO, etc.). Zona ciega 0.1 m. Ethernet, PTP. Driver `livox_ros_driver2` (Humble). **Ojo: no ve por debajo de −7°**. |
| Livox Mid-360S | 360° × 59° | hasta 100 m (corte) | 200 000 | ~265 g | ~2 100 | Variante más nueva, con otro rango: es otro producto, validar aparte. |
| **Unitree L2 (4D LiDAR)** | 360° × 96° | ~30 m @90 % | 64 000 efectivos | ~230 g | 420–480 | La opción económica. El campo vertical amplio **sí ve por debajo del dron**. Menos densidad y alcance que el Mid-360. SDK ROS 2. |
| Ouster OS0 / OS1 | 360° × 90° (OS0) / 45° (OS1) | 35–100+ m según modelo | 0.6–2.6 M | 370–450 g | varios miles | Calidad profesional, muy denso. Caro y más pesado. |

### 2.2 Cámaras de profundidad (complemento o presupuesto bajo)

| Modelo | Campo (prof.) | Rango útil | Notas |
|---|---|---|---|
| Intel/RealSense D435i | ~87° × 58° | 0.3–3 m ideal, hasta ~10 m con error creciente | 72 g, USB 3, IMU. Wrapper ROS 2 `realsense2_camera`. RealSense se separó de Intel en 2025 y en 2026 se anunció su compra por Cognex: el driver sigue activo, pero conviene seguir la disponibilidad. |
| RealSense D455 | ~87° × 58° | 0.6–6 m ideal | Mayor base estéreo: mejor a distancia que la D435i. |
| Stereolabs ZED X / ZED X Mini | ~110° × 70° | hasta 15–20 m (profundidad neural) | GMSL2, obturador global, IP67, **pensada para Jetson** (necesita placa de captura). Excelente para exteriores. |
| Orbbec Gemini 335/336 | ~90° × 65° | 0.1–20 m (útil < 6 m) | Alternativa a RealSense con driver ROS 2. |

Limitaciones comunes: campo visual de ~90° (sólo ven adelante), el error de
profundidad crece con el cuadrado de la distancia, y fallan con paredes lisas
sin textura (estéreo pasivo) o con sol directo (luz IR activa). Con una cámara
el dron **tiene que mirar hacia donde va** (`nav.yaw_mode: travel`) y conviene
el giro de 360° al iniciar el mapeo.

### 2.3 Lidar 2D de barrido

| Modelo | Campo | Alcance | Peso | Notas |
|---|---|---|---|---|
| LightWare SF45/B | 320° | ~50 m | ~59 g | **Soportado nativamente por la Collision Prevention de PX4** (UART, sin computadora). Un solo plano. |
| Slamtec RPLIDAR S2/S3/C1 | 360° | 12–40 m | 100–200 g | Baratos, pensados para robots terrestres; un solo plano. |

Un lidar 2D ve la pared sólo a la altura del dron: no mide inclinación ni ve
objetos más bajos. Sirve como red de seguridad o con presupuesto mínimo.

### 2.4 Rangefinders 1D

Benewake TF-Luna (8 m, ~5 g), TFmini-S (12 m), TF02-Pro (40 m), LightWare
LW20/SF11. **Uso recomendado: uno mirando hacia abajo** para la altura (PX4 lo
fusiona en el EKF), no como detector de paredes. Un anillo de varios (como los
5 `Distance` de AirSim del repo de referencia) es lo más barato, pero deja
huecos entre haces: no detecta bien paredes irregulares.

### 2.5 Otros

- **Radar mmWave** (TI IWR6843, Ainstein): ve a través de polvo, humo y niebla,
  pero con resolución muy baja. Complemento para ambientes sucios.
- **Flujo óptico** (ARK Flow, Holybro H-Flow, PMW3901 + rangefinder): no detecta
  paredes, pero le da a PX4 posición en interiores sin GPS. Sin posición local
  válida no hay misión offboard (ver §4).

---

## 3. Recomendación

**Opción recomendada (mejor relación costo/resultado): Livox Mid-360 arriba
del dron + rangefinder hacia abajo.**

- 360° horizontales: el dron ve paredes en cualquier dirección, también
  mientras gira o vuela de costado, y la evasión no depende del rumbo.
- 200 000 puntos/s con 59° verticales: las paredes irregulares, inclinadas y las
  columnas aparecen enteras; el extractor de paredes mide planitud e
  inclinación con el plano 3D.
- Driver ROS 2 Humble en Jetson, y el mismo sensor sirve para LIO
  (FAST-LIO2) si se vuela en interiores sin GPS.
- **Montarlo inclinado ~20° hacia abajo** (`cloud.mount_rpy_deg: [0.0, -20.0, 0.0]`)
  si se vuela bajo en interiores: el Mid-360 no ve por debajo de −7°, y en la
  simulación (escenario 06) la caja de 1.5 m sólo se detectaba desde lejos con
  el lidar horizontal; inclinado, el mapeo terminó ~20 % más rápido.

**Opción económica: Unitree L2.** Su campo vertical de 96° sí cubre por debajo
del dron. Menos alcance y densidad, pero alcanza para habitaciones y
estructuras medianas.

**Opción liviana / bajo presupuesto: cámara de profundidad al frente** (D435i o
ZED X Mini) con `config/mapper_realsense.yaml`. Funciona, pero hay que volar
más despacio (≤ 1 m/s) y el mapeo tarda más.

**Siempre:** dejar la Collision Prevention de PX4 como red de seguridad
(`px4_obstacle_distance.enabled: true`, ver `docs/jetson-px4-1.17.md`) para
cuando el piloto toma el control en modo Position.

### Computadora

Jetson Orin con **JetPack 6.x (Ubuntu 22.04 → ROS 2 Humble nativo)**. Una
Orin Nano 8 GB alcanza para este stack (Python + numpy) más el driver del
Mid-360; una Orin NX 16 GB deja margen para FAST-LIO2 o detección con YOLO.
JetPack 5 trae Ubuntu 20.04: ahí Humble sólo corre compilado desde fuente o en
Docker.

### Configuración del dron del laboratorio (sin lidar)

El dron previsto lleva **una cámara al frente y una hacia abajo**, sin lidar,
con una Jetson Nano (JetPack 4.6, Humble en Docker `dustynv`). Qué hace falta
para que este stack funcione así:

| Pieza | Rol | Qué tiene que ser |
|---|---|---|
| Cámara frontal | Paredes, evasión, mapa | **De profundidad** (D435i/D455, OAK-D, Orbbec Gemini). Config: `mapper_realsense.yaml`. Una RGB común sólo sirve para YOLO. |
| Cámara inferior | Posición y aterrizaje | Flujo óptico + telémetro hacia abajo (p. ej. ARK Flow, o PMW3901 + TFmini) para volar sin GPS; o ArUco para aterrizaje de precisión. No entra al mapper. |
| Opcional | Costados y atrás | 2-4 rangefinders 1D: la cámara frontal ve ~87° y nada detrás. |

Límites con cámara frontal: volar ≤ 1 m/s, el mapeo tarda más (tiene que girar
para ver los costados) y la Jetson Nano va justa: `cloud.max_rate: 5.0`,
`cloud.downsample: 0.1`. Con una Orin Nano esos límites desaparecen.

---

## 4. Posición del dron: requisito previo

El mapper ubica cada nube con la pose de PX4 (`vehicle_local_position_v1` +
`vehicle_attitude`). Si esa pose deriva, las paredes se dibujan dobles o
corridas. Según el lugar:

| Lugar | Fuente de posición | Cómo |
|---|---|---|
| Exterior abierto | GPS (mejor RTK) | Normal de PX4. |
| Interior, piso con textura | Flujo óptico + rangefinder abajo | Parámetros `EKF2_OF_*`, `EKF2_RNG_*`. Deriva lentamente. |
| Interior grande / sin textura | LIO con el mismo Mid-360 (FAST-LIO2) | Publicar la odometría en `/fmu/in/vehicle_visual_odometry` (`EKF2_EV_CTRL`). |
| Unreal (Cosys-AirSim) | GPS simulado | Ya viene en `settings.json`. |

---

## 5. Cómo se conecta cada sensor a este repo

`mapper_node` acepta cualquier combinación de entradas (`config/*.yaml`):

| Sensor | Tipo de mensaje | Parámetros | Archivo de ejemplo |
|---|---|---|---|
| Lidar 3D (Mid-360, L2, Ouster) o AirSim | `PointCloud2` | `cloud.topics`, `cloud.axes: ['flu']`, `cloud.mount_*` | `mapper_mid360.yaml`, `mapper_unreal.yaml` |
| Cámara de profundidad | `PointCloud2` | `cloud.axes: ['optical']`, `map.sensor_fov_deg`, `map.no_hit_free_range: 0.0` | `mapper_realsense.yaml` |
| Lidar 2D | `LaserScan` | `scan.topics`, `scan.mount_*` | — |
| Rangefinders 1D / `Distance` de AirSim | `Range` | `range.topics`, `range.bearings_deg` | — |

`cloud.mount_xyz` es la posición del sensor respecto del centro del dron en
FRD (adelante, derecha, abajo; metros) y `cloud.mount_rpy_deg` su orientación,
igual que `X/Y/Z` y `Roll/Pitch/Yaw` en el `settings.json` de AirSim. Un error
de signo acá dibuja el mapa espejado: las pruebas de `test_mapping.py` cubren
las convenciones `flu`, `frd` y `optical`.

---

## Fuentes

- Livox Mid-360, especificaciones: https://www.livoxtech.com/mid-360/specs
- Livox Mid-360S: https://www.livoxtech.com/mid-360s
- Unitree 4D LiDAR L2: https://unitree.com/L2/ y manual de usuario https://oss-global-cdn.unitree.com/static/Unitree%204D%20LiDAR%20L2%20User%20Manual.pdf
- Unitree L2 en RobotShop: https://www.robotshop.com/products/unitree-4d-lidar-l2
- PX4 v1.17, Collision Prevention (SF45, CP_DIST, CP_GO_NO_DATA): https://docs.px4.io/v1.17/en/computer_vision/collision_prevention
- PX4 v1.17, notas de versión: https://docs.px4.io/v1.17/en/releases/1.17
- RealSense, separación de Intel: https://www.therobotreport.com/intel-spins-out-realsense-as-standalone-company/
- Cosys-AirSim, lidar: https://cosys-lab.github.io/Cosys-AirSim/lidar/ y wrapper ROS 2: https://cosys-lab.github.io/Cosys-AirSim/ros_cplusplus
