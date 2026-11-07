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
