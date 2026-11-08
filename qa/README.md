# Arnés de QA adversarial

65 casos que estresan la evasión con condiciones de entorno real que los
escenarios de `src/uav_sim/scenarios/` no cubren: postes y cables finos (con
patrón de lidar no repetitivo), puertas angostas, ruido, polvo, puntos
perdidos, latencia de la nube, dron con más inercia, viento y ráfagas,
obstáculos que se mueven, cortes y "ceguera" del sensor, cámara de
profundidad de 87°, y combinaciones ("día real"). Corre sin ROS, sobre
`uav_sim.closed_loop`, extendiendo el mundo simulado sin tocar el código del
repo.

```sh
python3 qa/harness/run_all.py                    # los 65 casos (~5 min con varios núcleos)
python3 qa/harness/run_all.py --casos 2,7b,9c    # grupo, id o prefijo
python3 qa/harness/run_all.py --listar
python3 qa/harness/run_case.py --caso 1d_cable_fase -v --traza /tmp/1d.csv
python3 qa/harness/run_all.py --repo /otra/copia/del/repo   # comparar versiones
```

Veredictos: PASA, AVISO (terminó sin choque pero con despeje < 0.6 m, lento o
con el mapa impreciso) y FALLA (choque al radio real del dron, objetivo no
alcanzado, trabado o fantasmas en el mapa). Resultados en `qa/resultados/`.

Estado al último commit: 61/65 sin falla, 0 choques. Fallan, como límites
conocidos: puertas de 1.2 m (2a, 2f; 15 cm por lado con un dron de 0.9 m), el
rastro de una caja que se detuvo y se fue lejos del camino (7b) y la cámara
configurada como lidar (9d, sin campo de visión: tarda y se queda quieta 28 s).

## Robustez (`qa/fuzz`, `qa/nodos`)

- `qa/fuzz/fuzz_core.py`: 29 entradas basura contra la lógica pura (NaN, nubes
  malformadas, reloj hacia atrás, sectores vacíos...). Sale con 1 si alguna falla.
- `qa/nodos/fakeros.py`: ROS 2 en memoria para correr `mapper_node`,
  `mission_node` y `fake_px4_world` sin ROS instalado. `run_nodes.py` vuela una
  misión; `run_inject.py <escenario> <s> <basura/s>` además inyecta mensajes
  rotos en vuelo y pasa si termina DONE sin choque ni setpoints no finitos.
- `./run_tests.sh robustez` corre los dos. Ver docs/hardening.md.
