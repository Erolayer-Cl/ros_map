# Hardening: entradas basura y configuración inválida

Revisión del código con pruebas adversariales (2026-11). La pregunta fue
siempre la misma: **¿qué pasa en vuelo si llega algo que no esperábamos?** Un
driver con un bug, una nube cortada por la red, PX4 con el estimador sin
posición, un YAML mal escrito.

## 1. Qué se encontró

Un fuzzer de la lógica pura (`qa/fuzz/fuzz_core.py`, 29 casos) y una prueba que
corre los tres nodos sobre un ROS 2 falso e inyecta 4 mensajes basura por
segundo durante una misión completa (`qa/nodos/run_inject.py`).

| # | Entrada | Antes | Ahora |
|---|---|---|---|
| 1 | `/obstacles/sectors` con `increment = 0` | **`mission_node` se cae** (división por cero en el callback) al 2.º mensaje basura | se ignora el giro, se usa el mensaje |
| 2 | Cualquier excepción en un callback de suscripción, timer o servicio | sale por `spin()` y **mata el nodo** | se registra y se sigue (`guard.py`) |
| 3 | `heading` NaN de PX4 | **setpoint con yaw NaN hacia PX4** | se usa el último rumbo bueno |
| 4 | x/y/z NaN con `xy_valid` en true | posición NaN en la misión y en el historial de poses | es pérdida de posición (failsafe) |
| 5 | Una muestra NaN en `PoseBuffer` | contaminaba toda interpolación que la tocara | se descarta |
| 6 | `/obstacles/sectors` vacío o con 3 sectores | `ZeroDivisionError` en el VFH | se ignora (aviso limitado) |
| 7 | Sector NaN | se trataba como **libre** (NaN falla toda comparación) | "no visto": el dron gira para verlo antes de avanzar |
| 8 | Sector NaN hacia `/fmu/in/obstacle_distance` | se codificaba como libre (`max + 1`) | `UINT16_MAX` (sin dato) |
| 9 | PointCloud2 con `point_step` chico, datatype desconocido, datos truncados | `ValueError` de numpy / `KeyError` (el `KeyError` no se atrapaba) | error claro y nube descartada; truncada: se usan los puntos enteros |
| 10 | `/map` con `data` de otro tamaño | excepción en el callback | se ignora con aviso |
| 11 | Pose, cuaternión o stamp no finitos en el mapper | `RuntimeWarning` y celdas en índices basura | la nube se descarta entera (`rejected_clouds`) |
| 12 | Setpoint no finito por un bug interno | salía a PX4 | **última barrera**: se sostiene el último bueno y se pasa a HOLD |

Con el código anterior la prueba de inyección termina con `mission_node` caído
antes de despegar. Con el actual, en `06_habitacion`, `05_trampa_U` y
`14_entorno_real`: misión DONE, sin choque, ningún setpoint no finito, con
~150-190 mensajes basura por vuelo.

## 2. Configuración inválida: se detecta en tierra

`mission_core.validate_params()` revisa la configuración al arrancar. Con
errores, `/mission/start` y `/mission/map` responden `success = false` con la
lista y el dron **no despega**. Errores:

- `avoid.emergency_distance >= robot_radius + safety_margin` (bordear dispararía
  la huida; ver CLAUDE.md, trampas). El arnés de QA tenía un caso así
  (`MARGENES_REDUCIDOS`): se corrigió.
- `control_rate < 10 Hz`, velocidades/tiempos <= 0 o no finitos.
- `avoid.stop_distance <= avoid.emergency_distance`.
- `safety.min_altitude >= safety.max_altitude`, área de mapeo o geocerca mal
  formadas, `waypoints` que no son múltiplo de 3.
- Valores fuera de lista en `nav.yaw_mode`, `map_mission.strategy`,
  `avoid.on_failure`, `avoid.on_return_failure`.

Avisos (no impiden volar): alturas fuera de `[min, max]` (se recortan en
vuelo), `avoid.enabled = false`, `avoid.require_sensors = false`,
`plan.inflation_radius < robot_radius`.

`MapperConfig.problems()` hace lo mismo con el mapper: `WallMapper` y
`mapper_node` no arrancan con resolución <= 0, FOV fuera de (0, 360],
`min_range >= max_range`, log-odds con el signo cambiado, caja del dron mal
formada, tasas <= 0, etc. La misión, sin sectores, no despega
(`avoid.require_sensors`).

## 3. Reglas que quedan (no deshacer)

- **NaN es "no sé", nunca "libre".** `inf` sí es libre (sector visto sin eco).
- Todo nodo hereda `GuardedNodeMixin` antes que `Node`: los callbacks nuevos
  quedan protegidos solos. No poner `try/except` que trague errores en la
  lógica pura: ahí se valida la entrada; el guard es para lo imprevisto.
- Ningún setpoint sale sin pasar por `MissionCore._publishable_setpoint()`.
- Las entradas descartadas se avisan la 1.ª vez y después cada 100.

## 4. Cómo correrlo

```sh
./run_tests.sh robustez                      # fuzzer + inyección en vuelo (~10 s)
python3 qa/nodos/run_inject.py 05_trampa_U 300 8   # escenario, s, basura por segundo
python3 qa/nodos/run_nodes.py 06_habitacion        # los 3 nodos sin basura
```

`qa/nodos/fakeros.py` es un ROS 2 en memoria (rclpy, mensajes, ejecutor) que lee
los `.msg` de `uav_map_msgs` del repo. No reemplaza probar con ROS real: no
modela QoS, hilos del `MultiThreadedExecutor` ni la serialización.

## 5. No cubierto todavía

- Concurrencia real (`MultiThreadedExecutor`): el ROS falso es de un hilo.
- Mensajes de PX4 con campos de otra versión (el `px4_msgs` tiene que coincidir
  con el firmware: ver docs/jetson-px4-1.17.md).
- Reloj del sistema que salta (NTP/chrony en la Jetson al conectarse a la red
  en pleno vuelo): conviene fijar la hora antes de despegar.
