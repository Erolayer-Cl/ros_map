#!/usr/bin/env python3
"""Corre los casos de QA en paralelo (un subproceso por caso) y resume.

    python3 run_all.py --repo /home/user/ros_map                 # todos
    python3 run_all.py --repo /ruta/otra_copia --casos 1,2b,7a   # ids, prefijos o grupos
    python3 run_all.py --listar

Escribe en --salida (por defecto harness/../resultados/<nombre del repo>/):
  <caso>.json, <caso>.log   resultado completo y salida de cada corrida
  resumen.md / resumen.json tabla caso -> veredicto (se reescribe al terminar cada caso)
Con --reporte RUTA agrega además una fila por caso a ese .md a medida que terminan.
Sale con código 1 si algún caso FALLA.
"""

import argparse
import json
import os
import subprocess
import sys
import threading
import time
from concurrent.futures import ThreadPoolExecutor, as_completed

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import cases  # noqa: E402
import simkit  # noqa: E402

HEADER = ('| caso | veredicto | estado final | t sim (s) | choque | despeje mín (m) | trabado '
          '(mayor quieto s) | waypoints / cobertura | motivos | reproducir |\n'
          '|---|---|---|---|---|---|---|---|---|---|\n')


def row(out, repo):
    wp = out['waypoints'] if out['waypoints'] != '-' else f"cob {out['cobertura']}"
    cmd = f"`python3 {os.path.join(HERE, 'run_case.py')} --repo {repo} --caso {out['caso']}`"
    return (f"| {out['caso']} | **{out['veredicto']}** | {out['estado_final']} | {out['tiempo_sim']} | "
            f"{'SÍ' if out['choque'] else 'no'} | {out['despeje_min']} | "
            f"{'SÍ' if out['trabado'] else 'no'} ({out['mayor_quieto_s']}) | {wp} | "
            f"{'; '.join(out['motivos'])} | {cmd} |\n")


def run_one(case, repo, outdir, max_time, timeout):
    js = os.path.join(outdir, f"{case['id']}.json")
    log = os.path.join(outdir, f"{case['id']}.log")
    cmd = [sys.executable, os.path.join(HERE, 'run_case.py'), '--repo', repo, '--caso', case['id'],
           '--json', js, '--traza', os.path.join(outdir, f"{case['id']}.csv")]
    if max_time:
        cmd += ['--max-time', str(max_time)]
    t0 = time.monotonic()
    try:
        proc = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)
        text = proc.stdout + proc.stderr
    except subprocess.TimeoutExpired as exc:
        text = f'TIMEOUT real {timeout} s\n{exc.stdout or ""}{exc.stderr or ""}'
        proc = None
    with open(log, 'w', encoding='utf-8') as fh:
        fh.write(text)
    if proc is not None and os.path.isfile(js) and os.path.getmtime(js) >= t0 - 1:
        with open(js, encoding='utf-8') as fh:
            return json.load(fh), text
    return {'caso': case['id'], 'titulo': case.get('titulo', ''), 'veredicto': 'ERROR',
            'estado_final': '-', 'tiempo_sim': '-', 'choque': False, 'despeje_min': '-',
            'trabado': False, 'mayor_quieto_s': '-', 'waypoints': '-', 'cobertura': '-',
            'motivos': [text.strip().splitlines()[-1] if text.strip() else 'sin salida']}, text


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--repo', default=simkit.DEFAULT_REPO)
    ap.add_argument('--casos', default='todos', help="'todos' o lista: 1,2b,4c_lat03_pasillo,'7 dinámico'")
    ap.add_argument('--jobs', type=int, default=os.cpu_count() or 2)
    ap.add_argument('--max-time', type=float, default=None, help='pisa el max_time de cada caso')
    ap.add_argument('--timeout', type=float, default=900.0, help='s reales por caso')
    ap.add_argument('--salida', default=None)
    ap.add_argument('--reporte', default=None, help='.md al que agregar filas a medida que terminan')
    ap.add_argument('--listar', action='store_true')
    args = ap.parse_args(argv)

    selected = cases.select(args.casos)
    if args.listar:
        for c in selected:
            print(f"{c['id']:28s} {c.get('grupo', ''):12s} {c.get('titulo', '')}")
        return 0
    repo = os.path.abspath(args.repo)
    simkit.setup_repo(repo)   # valida la ruta antes de lanzar nada
    outdir = args.salida or os.path.join(os.path.dirname(HERE), 'resultados',
                                         os.path.basename(repo.rstrip('/')))
    os.makedirs(outdir, exist_ok=True)
    print(f'{len(selected)} casos, {args.jobs} en paralelo, repo {repo}, salida {outdir}', flush=True)

    lock = threading.Lock()
    results = {}
    order = [c['id'] for c in selected]

    def write_summary():
        done = [results[i] for i in order if i in results]
        with open(os.path.join(outdir, 'resumen.json'), 'w', encoding='utf-8') as fh:
            json.dump(done, fh, ensure_ascii=False, indent=1, default=str)
        with open(os.path.join(outdir, 'resumen.md'), 'w', encoding='utf-8') as fh:
            fh.write(f'# Resultados QA ({repo})\n\n{len(done)}/{len(order)} casos\n\n' + HEADER)
            fh.writelines(row(r, repo) for r in done)

    if args.reporte and not os.path.isfile(args.reporte):
        with open(args.reporte, 'w', encoding='utf-8') as fh:
            fh.write('# Resultados QA\n\n' + HEADER)

    with ThreadPoolExecutor(max_workers=max(1, args.jobs)) as pool:
        futs = {pool.submit(run_one, c, repo, outdir, args.max_time, args.timeout): c for c in selected}
        for fut in as_completed(futs):
            out, _ = fut.result()
            with lock:
                results[out['caso']] = out
                write_summary()
                if args.reporte:
                    with open(args.reporte, 'a', encoding='utf-8') as fh:
                        fh.write(row(out, repo))
            print(f"[{out['veredicto']:6s}] {out['caso']:28s} {out.get('estado_final')} "
                  f"t={out.get('tiempo_sim')} despeje={out.get('despeje_min')} "
                  f"-> {'; '.join(out.get('motivos', []))}", flush=True)

    failed = [r for r in results.values() if r['veredicto'] in ('FALLA', 'ERROR')]
    print(f'\n{len(results) - len(failed)}/{len(results)} sin falla. Resumen: '
          f"{os.path.join(outdir, 'resumen.md')}")
    return 1 if failed else 0


if __name__ == '__main__':
    sys.exit(main())
