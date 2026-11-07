#!/usr/bin/env python3
"""Casos de QA de entorno real. Cada caso es un dict que entiende simkit.run_case.

Claves:
  escenario      nombre del repo ('01_pared_plana'), ruta .yaml o 'qa:<nombre>'
                 (escenarios propios en harness/escenarios/)
  mision         'waypoints' | 'mapeo' (por defecto la del escenario)
  lidar          kwargs de LidarModel (noise_std, horizontal_fov_deg, max_range...)
  percepcion     perdidos (fracción), polvo {puntos, distancia}, latencia (s),
                 fase_azimut_aleatoria, fase_elevacion_aleatoria
  vehiculo       kwargs de SimVehicle (max_accel, max_speed, kp...)
  viento         ver simkit.Wind
  dinamicos      cajas móviles, ver simkit.DynamicBox
  corte_sensor   ventanas en que el driver deja de publicar (no llega nube)
  sensor_ciego   ventanas en que llega la nube VACÍA (sensor tapado / niebla)
  mapper         kwargs de MapperConfig
  rumbo_sensor   True: pasar sensor_heading = yaw (como mapper_node)
  overrides      parámetros de la misión (DEFAULTS de mission_core)
  esperado       estado_final, waypoints (bool), cobertura_min, fantasmas_max_m2
  chequeos       [{'tipo': 'fantasmas', 'zona': [n0,e0,n1,e1]}, {'tipo': 'cruce', 'n': 10}]
"""

CAMARA = dict(channels=24, vertical_min_deg=-29.0, vertical_max_deg=29.0,
              horizontal_samples=120, horizontal_fov_deg=87.0, max_range=6.0,
              mount_frd=(0.1, 0.0, -0.05), noise_std=0.01)
# Lo que fija src/uav_wall_mapping/config/mapper_realsense.yaml (mapper_node).
CAMARA_MAPPER = dict(resolution=0.1, sensor_fov_deg=87.0, no_hit_free_range=3.0, min_range=0.3,
                     max_range=6.0, sector_range=6.0, sectors_from_map=True)
# Montaje de producción de mapper_mid360.yaml: 5 cm adelante, 12 cm arriba,
# inclinado 20° hacia abajo; min_range 0.4.
MID360 = dict(mount_frd=(0.05, 0.0, -0.12), mount_rpy_deg=(0.0, -20.0, 0.0))
MID360_MAPPER = dict(min_range=0.4)
INERCIA = dict(max_accel=1.5)
RAPIDO = {'cruise_speed': 3.0}
MARGENES_REDUCIDOS = {'plan.inflation_radius': 0.55, 'avoid.safety_margin': 0.1,
                      'avoid.emergency_distance': 0.6, 'avoid.stop_distance': 0.9}
# "Día real": Mid-360 de producción, ruido 3 cm, 20 % perdidos, polvo leve,
# 0.1 s de latencia y ráfagas de 2 m/s con compensación.
DIA_REAL = dict(lidar=dict(MID360, noise_std=0.03), mapper=dict(MID360_MAPPER),
                percepcion={'perdidos': 0.2, 'polvo': {'puntos': 1, 'distancia': [0.5, 3.0]},
                            'latencia': 0.1, 'fase_azimut_aleatoria': True,
                            'fase_elevacion_aleatoria': True},
                viento={'rafagas': {'amplitud': 2.0, 'duracion': [1.0, 3.0], 'intervalo': [2.0, 6.0]},
                        'compensacion_tau': 2.0})

CASES = [
    # ---------------- 0: control (sin perturbaciones) ----------------
    dict(id='0a_base_pared', grupo='0 control', titulo='01_pared_plana sin perturbar',
         escenario='01_pared_plana'),
    dict(id='0b_base_pasillo', grupo='0 control', titulo='04_pasillo sin perturbar',
         escenario='04_pasillo'),
    dict(id='0c_base_trampaU', grupo='0 control', titulo='05_trampa_U sin perturbar',
         escenario='05_trampa_U'),
    dict(id='0d_base_habitacion', grupo='0 control', titulo='06_habitacion (mapeo) sin perturbar',
         escenario='06_habitacion', esperado={'cobertura_min': 0.85}),

    # ---------------- 1: obstáculos finos ----------------
    dict(id='1a_poste_fino', grupo='1 finos', titulo='poste r=0.05 m en el camino',
         escenario='qa:qa_poste_fino'),
    dict(id='1b_poste_fino_fase', grupo='1 finos',
         titulo='poste r=0.05 m, fase de azimut aleatoria (lidar giratorio real)',
         escenario='qa:qa_poste_fino', percepcion={'fase_azimut_aleatoria': True}),
    dict(id='1c_cable', grupo='1 finos', titulo='cable 3 cm a la altura de vuelo entre 2 postes',
         escenario='qa:qa_cable'),
    dict(id='1d_cable_fase', grupo='1 finos',
         titulo='cable 3 cm, fase de azimut y elevación aleatorias (no repetitivo, tipo Livox)',
         escenario='qa:qa_cable',
         percepcion={'fase_azimut_aleatoria': True, 'fase_elevacion_aleatoria': True}),
    dict(id='1e_poste_mid360', grupo='1 finos',
         titulo='poste r=0.05 m con el montaje de producción (Mid-360 inclinado -20°)',
         escenario='qa:qa_poste_fino', lidar=dict(MID360), mapper=dict(MID360_MAPPER),
         percepcion={'fase_azimut_aleatoria': True, 'fase_elevacion_aleatoria': True}),
    dict(id='1f_cable_mid360', grupo='1 finos',
         titulo='cable 3 cm con el montaje de producción (Mid-360 inclinado -20°)',
         escenario='qa:qa_cable', lidar=dict(MID360), mapper=dict(MID360_MAPPER),
         percepcion={'fase_azimut_aleatoria': True, 'fase_elevacion_aleatoria': True}),

    # ---------------- 2: puerta angosta ----------------
    dict(id='2a_puerta_120', grupo='2 puerta', titulo='puerta de 1.2 m en pared de 60 m',
         escenario='qa:qa_puerta_120', chequeos=[{'tipo': 'cruce', 'n': 10.0}]),
    dict(id='2b_puerta_150', grupo='2 puerta', titulo='puerta de 1.5 m en pared de 60 m',
         escenario='qa:qa_puerta_150', chequeos=[{'tipo': 'cruce', 'n': 10.0}]),
    dict(id='2c_puerta_150_margenes', grupo='2 puerta',
         titulo='puerta de 1.5 m con márgenes reducidos por parámetro',
         escenario='qa:qa_puerta_150', chequeos=[{'tipo': 'cruce', 'n': 10.0}],
         overrides=dict(MARGENES_REDUCIDOS)),
    dict(id='2d_puerta_200', grupo='2 puerta', titulo='puerta de 2.0 m (más ancha que el dron x2)',
         escenario='qa:qa_puerta_200', chequeos=[{'tipo': 'cruce', 'n': 10.0}]),
    dict(id='2e_puerta_240', grupo='2 puerta', titulo='control: puerta de 2.4 m (umbral con defaults)',
         escenario='qa:qa_puerta_240', chequeos=[{'tipo': 'cruce', 'n': 10.0}]),
    dict(id='2f_puerta_120_margenes', grupo='2 puerta',
         titulo='puerta de 1.2 m con márgenes reducidos por parámetro',
         escenario='qa:qa_puerta_120', chequeos=[{'tipo': 'cruce', 'n': 10.0}],
         overrides=dict(MARGENES_REDUCIDOS)),

    # ---------------- 3: ruido / puntos perdidos / polvo ----------------
    dict(id='3a_ruido005_pared', grupo='3 ruido', titulo='noise_std 0.05 en 01_pared_plana',
         escenario='01_pared_plana', lidar={'noise_std': 0.05}),
    dict(id='3b_ruido005_pasillo', grupo='3 ruido', titulo='noise_std 0.05 en 04_pasillo',
         escenario='04_pasillo', lidar={'noise_std': 0.05}),
    dict(id='3c_perdidos25_pasillo', grupo='3 ruido', titulo='25 % de puntos perdidos en 04_pasillo',
         escenario='04_pasillo', percepcion={'perdidos': 0.25}),
    dict(id='3d_perdidos30_trampaU', grupo='3 ruido', titulo='30 % de puntos perdidos en 05_trampa_U',
         escenario='05_trampa_U', percepcion={'perdidos': 0.30}),
    dict(id='3e_polvo_leve_pared', grupo='3 ruido',
         titulo='polvo: 2 puntos espurios/escaneo a 0.5-3 m (01_pared_plana)',
         escenario='01_pared_plana', percepcion={'polvo': {'puntos': 2, 'distancia': [0.5, 3.0]}}),
    dict(id='3f_polvo_fuerte_pared', grupo='3 ruido',
         titulo='polvo: 10 puntos espurios/escaneo a 0.5-2 m (01_pared_plana)',
         escenario='01_pared_plana', percepcion={'polvo': {'puntos': 10, 'distancia': [0.5, 2.0]}}),
    dict(id='3h_polvo_medio_pared', grupo='3 ruido',
         titulo='polvo: 5 puntos espurios/escaneo a 0.5-2.5 m (01_pared_plana)',
         escenario='01_pared_plana', percepcion={'polvo': {'puntos': 5, 'distancia': [0.5, 2.5]}}),
    dict(id='3i_polvo_medio_pasillo', grupo='3 ruido',
         titulo='polvo: 5 puntos espurios/escaneo a 0.5-2.5 m (04_pasillo)',
         escenario='04_pasillo', percepcion={'polvo': {'puntos': 5, 'distancia': [0.5, 2.5]}}),
    dict(id='3g_combinado_habitacion', grupo='3 ruido',
         titulo='mapeo 06_habitacion con ruido 0.05 + 25 % perdidos + polvo 3 pts',
         escenario='06_habitacion', lidar={'noise_std': 0.05},
         percepcion={'perdidos': 0.25, 'polvo': {'puntos': 3, 'distancia': [0.5, 2.5]}},
         esperado={'cobertura_min': 0.85}),

    # ---------------- 4: latencia ----------------
    dict(id='4a_lat01_pasillo', grupo='4 latencia', titulo='nube con 0.1 s de retraso (04_pasillo)',
         escenario='04_pasillo', percepcion={'latencia': 0.1}),
    dict(id='4b_lat02_pasillo', grupo='4 latencia', titulo='nube con 0.2 s de retraso (04_pasillo)',
         escenario='04_pasillo', percepcion={'latencia': 0.2}),
    dict(id='4c_lat03_pasillo', grupo='4 latencia', titulo='nube con 0.3 s de retraso (04_pasillo)',
         escenario='04_pasillo', percepcion={'latencia': 0.3}),
    dict(id='4d_lat03_trampaU', grupo='4 latencia', titulo='nube con 0.3 s de retraso (05_trampa_U)',
         escenario='05_trampa_U', percepcion={'latencia': 0.3}),
    dict(id='4e_lat03_habitacion', grupo='4 latencia',
         titulo='nube con 0.3 s de retraso, mapeo 06_habitacion (giro inicial)',
         escenario='06_habitacion', percepcion={'latencia': 0.3}, esperado={'cobertura_min': 0.85}),
    dict(id='4f_lat03_rapido_pared', grupo='4 latencia',
         titulo='0.3 s de retraso + cruise 3 m/s (01_pared_plana)',
         escenario='01_pared_plana', percepcion={'latencia': 0.3}, overrides=dict(RAPIDO)),

    # ---------------- 5: dinámica ----------------
    dict(id='5a_inercia_pared', grupo='5 dinámica', titulo='max_accel 1.5 + cruise 3.0 (01_pared_plana)',
         escenario='01_pared_plana', vehiculo=dict(INERCIA), overrides=dict(RAPIDO)),
    dict(id='5b_inercia_trampaU', grupo='5 dinámica', titulo='max_accel 1.5 + cruise 3.0 (05_trampa_U)',
         escenario='05_trampa_U', vehiculo=dict(INERCIA), overrides=dict(RAPIDO)),
    dict(id='5c_inercia_pasillo', grupo='5 dinámica', titulo='max_accel 1.5 + cruise 3.0 (04_pasillo)',
         escenario='04_pasillo', vehiculo=dict(INERCIA), overrides=dict(RAPIDO)),
    dict(id='5d_inercia_poste', grupo='5 dinámica', titulo='max_accel 1.5 + cruise 3.0 (poste fino)',
         escenario='qa:qa_poste_fino', vehiculo=dict(INERCIA), overrides=dict(RAPIDO)),
    dict(id='5e_inercia_puerta240', grupo='5 dinámica',
         titulo='max_accel 1.5 + cruise 3.0 (pared con puerta de 2.4 m)',
         escenario='qa:qa_puerta_240', vehiculo=dict(INERCIA), overrides=dict(RAPIDO)),

    dict(id='5f_inercia_oculto', grupo='5 dinámica',
         titulo='max_accel 1.5 + cruise 3.0, columna escondida detrás de la punta de una pared',
         escenario='qa:qa_obstaculo_oculto', vehiculo=dict(INERCIA), overrides=dict(RAPIDO)),

    # ---------------- 6: viento ----------------
    dict(id='6a_viento_cte_pasillo', grupo='6 viento',
         titulo='deriva constante 1 m/s al este, sin compensación (04_pasillo)',
         escenario='04_pasillo', viento={'constante': [0.0, 1.0]}),
    dict(id='6b_viento_cte_comp_pasillo', grupo='6 viento',
         titulo='deriva constante 1.5 m/s al este, con compensación tau 2 s (04_pasillo)',
         escenario='04_pasillo', viento={'constante': [0.0, 1.5], 'compensacion_tau': 2.0}),
    dict(id='6c_rafagas_pasillo', grupo='6 viento',
         titulo='ráfagas de hasta 2.5 m/s, compensación tau 2 s (04_pasillo)',
         escenario='04_pasillo',
         viento={'rafagas': {'amplitud': 2.5, 'duracion': [1.0, 3.0], 'intervalo': [2.0, 6.0]},
                 'compensacion_tau': 2.0}),
    dict(id='6d_rafagas_trampaU', grupo='6 viento',
         titulo='ráfagas de hasta 2 m/s sin compensación (05_trampa_U)',
         escenario='05_trampa_U',
         viento={'rafagas': {'amplitud': 2.0, 'duracion': [1.0, 3.0], 'intervalo': [2.0, 6.0]}}),

    dict(id='6e_rafagas4_comp_habitacion', grupo='6 viento',
         titulo='ráfagas de hasta 4 m/s con compensación tau 2 s, mapeo 06_habitacion',
         escenario='06_habitacion', esperado={'cobertura_min': 0.85},
         viento={'rafagas': {'amplitud': 4.0, 'duracion': [1.0, 3.0], 'intervalo': [2.0, 6.0]},
                 'compensacion_tau': 2.0}),
    dict(id='6f_rafagas4_comp_pasillo', grupo='6 viento',
         titulo='ráfagas de hasta 4 m/s con compensación tau 2 s (04_pasillo, semilla 3)',
         escenario='04_pasillo', semilla=3,
         viento={'rafagas': {'amplitud': 4.0, 'duracion': [1.0, 3.0], 'intervalo': [2.0, 6.0]},
                 'compensacion_tau': 2.0}),

    # ---------------- 7: obstáculo dinámico ----------------
    dict(id='7a_caja_cruza', grupo='7 dinámico',
         titulo='caja 1x1x4 m cruza el camino a 1 m/s y se va (campo abierto)',
         escenario='qa:qa_campo_abierto',
         dinamicos=[{'tamano': [1.0, 1.0, 4.0], 'inicio': {'cuando_n_mayor': 1.0},
                     'trayectoria': [[0, 10.0, -6.0], [12, 10.0, 6.0], [40, 10.0, 34.0]]}],
         chequeos=[{'tipo': 'fantasmas', 'zona': [8.5, -7.0, 11.5, 12.0]}, {'tipo': 'cruce', 'n': 10.0}],
         esperado={'fantasmas_max_m2': 0.5}),
    dict(id='7b_caja_se_detiene', grupo='7 dinámico',
         titulo='caja cruza, se detiene 8 s en el camino y se va (campo abierto)',
         escenario='qa:qa_campo_abierto',
         dinamicos=[{'tamano': [1.0, 1.0, 4.0], 'inicio': {'cuando_n_mayor': 1.0},
                     'trayectoria': [[0, 10.0, -6.0], [6, 10.0, 0.0], [14, 10.0, 0.0],
                                     [20, 10.0, 6.0], [48, 10.0, 34.0]]}],
         chequeos=[{'tipo': 'fantasmas', 'zona': [8.5, -7.0, 11.5, 12.0]}, {'tipo': 'cruce', 'n': 10.0}],
         esperado={'fantasmas_max_m2': 0.5}),
    dict(id='7c_caja_de_frente', grupo='7 dinámico',
         titulo='caja avanza de frente hacia el dron a 1.5 m/s y después se aparta',
         escenario='qa:qa_campo_abierto',
         dinamicos=[{'tamano': [1.0, 1.0, 4.0], 'inicio': {'cuando_n_mayor': 1.0},
                     'trayectoria': [[0, 22.0, 0.3], [12, 4.0, 0.3], [32, 4.0, -26.0]]}],
         chequeos=[{'tipo': 'fantasmas', 'zona': [3.0, -3.0, 23.0, 3.0]}],
         esperado={'fantasmas_max_m2': 0.5}),

    dict(id='7d_persona_pasillo', grupo='7 dinámico',
         titulo='persona (0.5x0.5x1.8 m) entra al pasillo, se queda 5 s en el medio y se va',
         escenario='04_pasillo',
         dinamicos=[{'tamano': [0.5, 0.5, 1.8], 'inicio': {'cuando_n_mayor': 2.0},
                     'trayectoria': [[0, 14.0, -1.8], [2, 14.0, 0.0], [7, 14.0, 0.0],
                                     [13, 14.0, 6.0], [40, 14.0, 30.0]]}],
         chequeos=[{'tipo': 'fantasmas', 'zona': [13.0, -1.5, 15.0, 1.5]}, {'tipo': 'cruce', 'n': 14.0}],
         esperado={'fantasmas_max_m2': 0.5}),

    # ---------------- 8: pérdida del sensor ----------------
    dict(id='8a_corte2s_pared', grupo='8 sensor',
         titulo='el lidar deja de publicar 2 s al quedar a 4 m de la pared (01)',
         escenario='01_pared_plana', corte_sensor=[{'cuando_despeje_menor': 4.0, 'duracion': 2.0}]),
    dict(id='8b_corte2s_pared_cerca', grupo='8 sensor',
         titulo='el lidar deja de publicar 2 s al quedar a 2.5 m de la pared (01)',
         escenario='01_pared_plana', corte_sensor=[{'cuando_despeje_menor': 2.5, 'duracion': 2.0}]),
    dict(id='8c_corte2s_rapido', grupo='8 sensor',
         titulo='corte de 2 s a 5 m de la pared con cruise 3 m/s (01)',
         escenario='01_pared_plana', overrides=dict(RAPIDO),
         corte_sensor=[{'cuando_despeje_menor': 5.0, 'duracion': 2.0}]),
    dict(id='8d_corte2s_pasillo', grupo='8 sensor',
         titulo='el lidar deja de publicar 2 s en medio del pasillo (n>12)',
         escenario='04_pasillo', corte_sensor=[{'cuando_n_mayor': 12.0, 'duracion': 2.0}]),
    dict(id='8e_ciego2s_pared', grupo='8 sensor',
         titulo='el lidar publica nubes VACÍAS 2 s a 3 m de la pared (01)',
         escenario='01_pared_plana', sensor_ciego=[{'cuando_despeje_menor': 3.0, 'duracion': 2.0}]),
    dict(id='8f_ciego2s_pasillo', grupo='8 sensor',
         titulo='el lidar publica nubes VACÍAS 2 s en medio del pasillo (n>12)',
         escenario='04_pasillo', sensor_ciego=[{'cuando_n_mayor': 12.0, 'duracion': 2.0}]),
    dict(id='8g_ciego5s_pared', grupo='8 sensor',
         titulo='el lidar publica nubes VACÍAS 5 s a 3 m de la pared (01)',
         escenario='01_pared_plana', sensor_ciego=[{'cuando_despeje_menor': 3.0, 'duracion': 5.0}]),
    dict(id='8h_ciego5s_pasillo', grupo='8 sensor',
         titulo='el lidar publica nubes VACÍAS 5 s en medio del pasillo (n>12)',
         escenario='04_pasillo', sensor_ciego=[{'cuando_n_mayor': 12.0, 'duracion': 5.0}]),

    dict(id='8i_ciego5s_pared_6m', grupo='8 sensor',
         titulo='nubes VACÍAS 5 s desde que la pared está a 6 m, rumbo a ella (01)',
         escenario='01_pared_plana', sensor_ciego=[{'cuando_despeje_menor': 6.0, 'duracion': 5.0}]),
    dict(id='8j_ciego8s_trampaU', grupo='8 sensor',
         titulo='nubes VACÍAS 8 s a 2 m de la U (05_trampa_U)',
         escenario='05_trampa_U', sensor_ciego=[{'cuando_despeje_menor': 2.0, 'duracion': 8.0}]),
    dict(id='8k_corte5s_pared_6m', grupo='8 sensor',
         titulo='control de 8i: el lidar deja de PUBLICAR 5 s desde 6 m de la pared (01)',
         escenario='01_pared_plana', corte_sensor=[{'cuando_despeje_menor': 6.0, 'duracion': 5.0}]),

    # ---------------- 9: cámara de profundidad ----------------
    dict(id='9a_camara_trampaU', grupo='9 cámara',
         titulo='cámara 87°/6 m en 05_trampa_U con config de mapper_realsense.yaml + rumbo',
         escenario='05_trampa_U', lidar=dict(CAMARA), mapper=dict(CAMARA_MAPPER), rumbo_sensor=True),
    dict(id='9b_camara_trampaU_cl', grupo='9 cámara',
         titulo='cámara 87°/6 m en 05_trampa_U (closed_loop tal cual: tallado 360°, sin rumbo)',
         escenario='05_trampa_U', lidar=dict(CAMARA)),
    dict(id='9c_camara_habitacion', grupo='9 cámara',
         titulo='cámara 87°/6 m, mapeo 06_habitacion con config de mapper_realsense.yaml + rumbo',
         escenario='06_habitacion', lidar=dict(CAMARA), mapper=dict(CAMARA_MAPPER), rumbo_sensor=True,
         esperado={'cobertura_min': 0.85}),
    dict(id='9d_camara_habitacion_cl', grupo='9 cámara',
         titulo='cámara 87°/6 m, mapeo 06_habitacion (closed_loop tal cual)',
         escenario='06_habitacion', lidar=dict(CAMARA), esperado={'cobertura_min': 0.85}),

    # ---------------- 10: combinado "día real" ----------------
    dict(DIA_REAL, id='10a_dia_real_trampaU', grupo='10 combinado',
         titulo='Mid-360 de producción + ruido/perdidos/polvo/latencia 0.1 s/ráfagas (05_trampa_U)',
         escenario='05_trampa_U'),
    dict(DIA_REAL, id='10b_dia_real_pasillo', grupo='10 combinado',
         titulo='Mid-360 de producción + ruido/perdidos/polvo/latencia 0.1 s/ráfagas (04_pasillo)',
         escenario='04_pasillo'),
    dict(DIA_REAL, id='10c_dia_real_habitacion', grupo='10 combinado',
         titulo='Mid-360 de producción + ruido/perdidos/polvo/latencia/ráfagas, mapeo 06_habitacion',
         escenario='06_habitacion', esperado={'cobertura_min': 0.85}),
]

BY_ID = {c['id']: c for c in CASES}


def select(patterns):
    """'todos' | lista separada por comas de ids, prefijos ('1', '4c') o grupos."""
    if not patterns or patterns in ('todos', 'all'):
        return list(CASES)
    out = []
    for pat in [p.strip() for p in patterns.split(',') if p.strip()]:
        if pat.isdigit():   # '1' = grupo 1 (no '10a...')
            hits = [c for c in CASES if c['id'].split('_')[0].rstrip('abcdefghijklmnopqrstuvwxyz') == pat]
        else:
            hits = [c for c in CASES if c['id'] == pat or c['id'].startswith(pat)
                    or c.get('grupo', '').startswith(pat)]
        if not hits:
            raise SystemExit(f'caso desconocido: {pat!r}. Usá --listar.')
        out.extend(c for c in hits if c not in out)
    return out
