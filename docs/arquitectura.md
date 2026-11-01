# Arquitectura y algoritmos

Cómo está armado el stack y por qué. Para instalar y probar, ver el `README.md`.

## 1. Paquetes

| Paquete | Tipo | Qué tiene |
|---|---|---|
| `px4_msgs` | submódulo (rama `release/1.17`) | Mensajes de PX4 v1.17. |
| `uav_map_msgs` | ament_cmake | `MissionStatus`, `Wall`, `WallArray`, `ObstacleSectors`. |
| `uav_wall_mapping` | ament_python | `mapper_node` + módulos puros: grilla, vóxeles, sectores, paredes, exportación. |
| `uav_mission` | ament_python | `mission_node` + módulos puros: máquina de estados, VFH+, A*, exploración. |
| `uav_sim` | ament_python | Escenarios YAML, mundo simulado, `fake_px4_world`, `sim_offline`, `spawn_scenario`. |

Regla heredada del repo de referencia: **toda la lógica es Python puro** (sin
`rclpy`) y los nodos son adaptadores finos. Por eso `run_tests.sh` prueba la
misión completa, el mapeo y el esquive sin ROS ni Unreal, y `closed_loop.py`
corre las mismas clases que los nodos contra un lidar simulado.

```
                     ┌──────────────────── mapper_node ────────────────────┐
 PointCloud2 ───────►│ SensorMount → mundo NED → WallMapper                 │──► /map (OccupancyGrid, ENU)
 LaserScan ─────────►│   ├─ OccupancyGrid2D (log-odds, franja de alturas)   │──► /map/cloud (vóxeles)
 Range ─────────────►│   ├─ VoxelMap3D (media + lado de observación)        │──► /walls, /walls/markers
 vehicle_local_pos ─►│   ├─ sectores polares (72 × 5°)                       │──► /obstacles/sectors
 vehicle_attitude ──►│   └─ WallExtractor (planas / irregulares / obstáculos)│──► /fmu/in/obstacle_distance (opc.)
                     └──────────────────────────────────────────────────────┘
                                             │ /map, /obstacles/sectors, /walls
                     ┌──────────────────── mission_node ───────────────────┐
 /mission/start ────►│ MissionCore (20 Hz)                                  │──► /fmu/in/trajectory_setpoint
 /mission/map ──────►│   ├─ exploración: fronteras | barrido                 │──► /fmu/in/offboard_control_mode
 pause/resume/... ──►│   ├─ GridPlanner A* (otro hilo, 5 Hz)                 │──► /fmu/in/vehicle_command
 vehicle_status_v1 ─►│   ├─ VfhAvoider (VFH+ con debounce)                   │──► /mission/status, /mission/path
                     │   └─ zanahoria + feedforward de velocidad             │──► /map/save, /map/reset (clientes)
                     └──────────────────────────────────────────────────────┘
```

## 2. Marcos de referencia

Todo lo interno es **NED de PX4** (x = Norte, y = Este, z = Abajo; rumbos con
0 = Norte, positivo hacia el Este) y cuerpo **FRD**. Lo publicado para
visualizar (`/map`, `/walls/markers`, `/map/cloud`, `/mission/path`, TF
`map → base_link`) está en **ENU** (x = Este, y = Norte). `Wall.points` y
`ObstacleSectors` quedan en NED. `waypoints` y `map_mission.area` son relativos
al punto de despegue.

Convenciones de los sensores (`cloud.axes`): `flu` (ROS estándar, Livox,
Cosys-AirSim), `frd` (PX4/AirSim crudo), `optical` (cámaras de profundidad).

## 3. Mapeo

**Grilla 2D (log-odds).** Cada nube aporta: impactos (`+0.85`) para los puntos
dentro de la franja de alturas `map.min_height..map.max_height`, y espacio
libre (`−0.4`) en las celdas recorridas por los rayos. Se proyecta toda la
franja (no sólo la altura de vuelo): así una pared inclinada o con salientes
deja su huella completa, y el planificador nunca acerca el dron a la parte que
sobresale. Detalles que importan (cada uno salió de una falla real en la
simulación):

- **Rayos sin impacto.** Un lidar no devuelve puntos donde no chocó; para que el
  espacio abierto cuente como explorado se tallan como libres hasta
  `no_hit_free_range`. Pero un sector puede quedar vacío sólo por caer entre
  dos haces, con una pared ahí: su alcance se limita al impacto más cercano de
  los sectores vecinos.
- **Rayos rasantes.** Un rayo casi paralelo a una pared viaja pegado a su cara
  antes de impactar más adelante y "vaciaba" la cara. El tallado se acorta
  `0.15 / sin(incidencia)`, con la incidencia estimada por los impactos vecinos.
- **Celdas confirmadas.** Una celda ya ocupada se vacía 3 veces más despacio:
  en una grilla 2D, un rayo que pasa por encima de un objeto bajo atraviesa sus
  celdas aunque el objeto exista.
- **Lado de observación.** Cada celda y cada vóxel acumulan la dirección desde
  la que se los vio. Sirve para separar las dos caras de una pared (abajo).

**Mapa 3D.** Vóxeles de 0.2 m que guardan cantidad de impactos, posición media
(sin ruido de cuantización) y lado de observación. No se borra espacio libre en
3D: es para escenas estáticas.

**Sectores.** Para la evasión: distancia horizontal mínima por sector de 5°
con los puntos en la franja `altura del dron ± 0.7 m`, combinada con las
celdas ocupadas del mapa cercanas (memoria: una pared que salió del campo de
una cámara sigue existiendo). Si el sensor deja de publicar, no se publican
sectores y la misión frena.

## 4. Paredes: planas, irregulares, obstáculos

1. Celdas ocupadas → nodos de 0.3 m → componentes conexas (radio 0.6 m: un
   hueco chico no parte la pared, una puerta sí).
2. Esqueleto de cada componente: camino más largo del grafo (doble Dijkstra),
   más ramas para lo que no cubre (la otra mitad de una habitación, una T).
3. Ramer-Douglas-Peucker (0.3 m) → aristas; cada celda se asigna a la arista
   más cercana.
4. **Prueba de planitud** de cada arista, ignorando 0.4 m en cada punta
   (esquinas):
   - con datos 3D, se ajusta un **plano** a los vóxeles del tramo (por eso una
     pared inclinada 20° es plana y se reporta su inclinación);
   - dispersión **por cara** ≤ 8 cm: las caras se separan por el lado desde el
     que se vieron (las dos caras de una pared gruesa) — los dientes de una
     pared irregular se ven del mismo lado y no se pueden "explicar" como dos
     caras;
   - **flecha** de la parábola ajustada ≤ 6 cm: separa una curva de una recta
     con ruido (el coeficiente cuadrático se estima en el centro y se escala
     al largo completo);
   - espesor ≤ 0.8 m, largo ≥ 1 m.
5. Aristas planas consecutivas alineadas se funden; las no planas forman una
   pared irregular (polilínea). Tramos planos cortos pegados a uno irregular son
   facetas de la irregular.
6. Global: se unen planas colineales con huecos < 0.6 m, se reagrupa todo lo no
   plano con radio 1 m (los bloques de una pared de escombros vuelven a ser una),
   lo compacto (< 3 m) que no es una sola plana es **obstáculo**.
7. Alturas e inclinación con el mapa 3D; ids estables entre extracciones.

Validado con formas sintéticas (`test_walls.py`) y en lazo cerrado con los 9
escenarios (`test_closed_loop.py`).

## 5. Navegación

**Zanahoria.** Como en la referencia, el setpoint no es el objetivo lejano sino
un punto que avanza a `cruise_speed` y nunca se aleja más de `max_lead` del
dron; además se manda la velocidad como *feedforward*. Si la evasión cambia la
dirección más de 25°, la zanahoria se re-ancla en el dron (si no, la vieja lo
arrastraría hacia la pared). Rampa de velocidad al arrancar (lección de la
referencia: un escalón de velocidad cabecea el dron y los sensores ven el piso).

**Planificador global A\*** sobre el mapa, a 0.3 m, en una ventana alrededor de
inicio y objetivo, con obstáculos inflados 0.9 m y costo extra hasta 2 m (va
por el medio de los pasillos). Lo desconocido se puede cruzar con un costo
extra (exploración optimista). Se replanifica cada segundo en otro hilo. Es lo
que resuelve la trampa en U.

**Evasión reactiva VFH+** sobre los 72 sectores: cada obstáculo se agranda
`asin((radio + margen) / d)` (uno cercano tapa más sectores que uno lejano), se
elige el sector libre más cercano al rumbo deseado (con inercia, para no
alternar izquierda/derecha), y la velocidad baja con la distancia libre al
frente. Si algo queda a menos de 0.9 m, se aleja. Debounce de 0.2 s en cada
decisión (lección de la referencia: el simulador puede dar lecturas espurias).

**Fallas.** Sin progreso en `nav.stuck_timeout` s, A* sin camino 3 veces, o
rodeado más de `avoid.blocked_timeout` s → el objetivo se abandona: en el
mapeo se pasa a otra frontera; en waypoints, `avoid.on_failure`
(`return`/`hold`/`skip`).

## 6. Misión de mapeo (`/mission/map`)

```
IDLE → ARMING → TAKEOFF → SCAN (360°) → NAVIGATE/AVOID a la frontera más conveniente ─┐
                                   ▲                                                   │
                                   └──── llegó, o la frontera se "vio" desde lejos ────┘
        sin fronteras / tiempo máximo → /map/save → RETURN → LAND → DONE
```

- **Fronteras:** celdas libres pegadas a desconocidas dentro del área. Se
  descartan las que no tienen ninguna celda a más de `plan.inflation_radius` de
  un obstáculo (lo desconocido detrás de una caja no se puede alcanzar). Costo:
  distancia + giro − tamaño. Si mientras vuela la frontera deja de existir
  (el lidar de 40 m la vio de lejos), se pasa a la siguiente sin llegar.
- **Barrido (`lawnmower`):** zig-zag sobre el área, carriles cada `lane_spacing`.
- Al terminar llama a `/map/save` de mapper_node: PGM+YAML (nav2), PNG, SVG con
  cotas, JSON de paredes y PLY 3D.

## 7. Seguridad

- Sólo `mission_node` manda a PX4; si el piloto sale de OFFBOARD, pasa a
  `PREEMPTED` y deja de publicar.
- El lazo de control nunca deja escapar una excepción (pasa a HOLD y sigue
  publicando); el A* corre en otro hilo.
- Sin datos de obstáculos: no despega (`avoid.require_sensors`), y en vuelo
  frena (`avoid.hold_on_sensor_loss`).
- Ctrl-C con el dron armado manda LAND.
- Collision Prevention de PX4 como red para el piloto (ver `jetson-px4-1.17.md`).

## 8. Lo que no hace (todavía)

- Mapa 3D sin espacio libre: objetos que se mueven quedan en el mapa.
- Una sola altura de vuelo por misión de mapeo (el lidar 3D cubre el resto).
- Paredes de vidrio: invisibles para el lidar (también en la realidad).
- Depende de la posición de PX4: con GPS malo o en interiores sin flujo óptico
  ni LIO, el mapa se deforma (ver `sensores.md` §4).
