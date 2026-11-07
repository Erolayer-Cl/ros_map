#!/usr/bin/env python3
"""Corre UN caso de QA y muestra el resultado (para reproducir un fallo).

    python3 run_case.py --repo /home/user/ros_map --caso 2b_puerta_150 -v
    python3 run_case.py --caso 7a_caja_cruza --json /tmp/7a.json --traza /tmp/7a.csv
"""

import argparse
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import cases  # noqa: E402
import simkit  # noqa: E402


def fmt(out):
    lines = [
        f"[{out['veredicto']}] {out['caso']}: {out['titulo']}",
        f"  escenario {out['escenario']} ({out['mision']}), repo {out.get('repo')}",
        f"  estado final {out['estado_final']} a {out['tiempo_sim']} s sim ({out['tiempo_real']} s reales)",
        f"  choque {out['choque']}  despeje mín {out['despeje_min']} m en {out['despeje_min_en']}",
        f"  trabado {out['trabado']} (mayor tramo quieto {out['mayor_quieto_s']} s en {out['quieto_en']})",
        f"  waypoints {out['waypoints']} (dist. mín. {out['dist_min_a_wp']})  cobertura {out['cobertura']}",
        f"  paredes [planas, irregulares, obstáculos] = {out['paredes']}  recorrido {out['recorrido_m']} m",
        f"  motivos: {'; '.join(out['motivos'])}",
        f"  estados ({len(out['estados'])} cambios, {out['entradas_avoid']} entradas a AVOID): "
        + ' '.join(f'{t:g}:{n}' for t, n in out['estados'][:16])
        + (f" ... {' '.join(f'{t:g}:{n}' for t, n in out['estados'][-4:])}" if len(out['estados']) > 20 else
           ' '.join(f'{t:g}:{n}' for t, n in out['estados'][16:])),
        f"  tiempo por estado: {out['tiempo_por_estado']}",
        f"  mapa: {out.get('mapa_celdas_ocupadas')} celdas ocupadas, {out.get('mapa_mal_ubicado_pct')}% "
        f"a >0.35 m de una superficie real (error máx {out.get('mapa_error_max_m')} m)",
    ]
    if out['objetivos_abandonados']:
        lines.append(f"  abandonados: {out['objetivos_abandonados']}")
    if out['eventos']:
        lines.append(f"  eventos: {out['eventos'][:8]}")
    if out['extra']:
        lines.append(f"  extra: {out['extra']}")
    c = {k: v for k, v in out['contadores'].items() if v}
    lines.append(f"  contadores: {c}  ventanas: {out['ventanas']}")
    if any(out.get('memoria_6m_en_ventanas') or []):
        lines.append(f"  memoria del mapa a <=6 m (celdas ocupadas) en cada ventana: {out['memoria_6m_en_ventanas']}")
    if out['log_warn']:
        lines.append('  warnings del core:')
        lines.extend(f'    {m}' for m in out['log_warn'][:8])
    return '\n'.join(lines)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--repo', default=simkit.DEFAULT_REPO)
    ap.add_argument('--caso', required=True)
    ap.add_argument('--max-time', type=float, default=None)
    ap.add_argument('--json', default=None, help='guardar el resultado en este archivo')
    ap.add_argument('--traza', default=None, help='CSV con t,n,e,d,yaw,estado,despeje')
    ap.add_argument('-v', '--verbose', action='store_true', help='log de la misión en vivo')
    args = ap.parse_args(argv)
    if args.caso not in cases.BY_ID:
        sys.exit(f'caso desconocido {args.caso!r}; disponibles: {", ".join(cases.BY_ID)}')
    out, _, ctx = simkit.run_case(cases.BY_ID[args.caso], repo=args.repo,
                                  max_time=args.max_time, verbose=args.verbose)
    print(fmt(out))
    if args.json:
        with open(args.json, 'w', encoding='utf-8') as fh:
            json.dump(out, fh, ensure_ascii=False, indent=1, default=str)
    if args.traza:
        with open(args.traza, 'w', encoding='utf-8') as fh:
            fh.write('t,n,e,d,yaw,estado,despeje\n')
            for s in ctx.samples:
                fh.write(','.join(str(v) for v in s) + '\n')
    return 0 if out['veredicto'] != 'FALLA' else 1


if __name__ == '__main__':
    sys.exit(main())
