#!/usr/bin/env python3
"""Guardado del mapa en disco: el "plano" que queda al terminar la misión.

Python puro (zlib/struct/json de la biblioteca estándar + numpy): no hace falta
PIL ni OpenCV en la Jetson para sacar una imagen.

Archivos que genera `save_map_bundle` (todos con el mismo prefijo):

- `<base>.pgm` + `<base>.yaml`: formato de `nav2_map_server`, se puede volver a
  cargar con `ros2 run nav2_map_server map_server` o abrir con cualquier visor.
- `<base>.png`: vista rápida con las paredes coloreadas por tipo y la trayectoria.
- `<base>.svg`: plano vectorial con cotas (largo de cada pared), escala y norte.
- `<base>_walls.json`: las paredes con todos sus datos (para post-procesar).
- `<base>_cloud.ply`: nube 3D de vóxeles (CloudCompare, MeshLab, Blender).

Convención de los archivos: ENU (x = Este, y = Norte), que es lo que esperan
nav2, RViz y los visores 3D. Los JSON también traen NED para no adivinar.
"""

import json
import math
import os
import struct
import zlib

import numpy as np

from uav_wall_mapping.walls import FLAT, IRREGULAR, KIND_NAMES, OBSTACLE

WALL_COLORS = {
    FLAT: (30, 100, 220),        # azul
    IRREGULAR: (240, 130, 0),    # naranja
    OBSTACLE: (200, 0, 200),     # magenta
}
TRAJECTORY_COLOR = (0, 160, 0)
HOME_COLOR = (220, 0, 0)


# ----------------------------------------------------------------------
# Escritores de bajo nivel
# ----------------------------------------------------------------------
def write_png(path, rgb):
    """PNG RGB 8 bits sin dependencias (zlib + struct)."""
    rgb = np.ascontiguousarray(rgb, dtype=np.uint8)
    height, width = rgb.shape[:2]
    raw = b''.join(b'\x00' + rgb[row].tobytes() for row in range(height))

    def chunk(tag, payload):
        body = tag + payload
        return struct.pack('>I', len(payload)) + body + struct.pack('>I', zlib.crc32(body) & 0xFFFFFFFF)

    header = struct.pack('>IIBBBBB', width, height, 8, 2, 0, 0, 0)
    with open(path, 'wb') as fh:
        fh.write(b'\x89PNG\r\n\x1a\n')
        fh.write(chunk(b'IHDR', header))
        fh.write(chunk(b'IDAT', zlib.compress(raw, 6)))
        fh.write(chunk(b'IEND', b''))


def occupancy_to_gray(values):
    """Valores de OccupancyGrid -> gris al estilo map_server (libre blanco,
    ocupado negro, desconocido gris), con la fila 0 = norte máximo."""
    gray = np.full(values.shape, 205, dtype=np.uint8)
    known = values >= 0
    gray[known] = (254 - (values[known].astype(np.int32) * 254) // 100).astype(np.uint8)
    return gray[::-1]


def write_pgm_yaml(base, snapshot, occupied_thresh=0.65, free_thresh=0.25):
    values = snapshot.values
    gray = np.full(values.shape, 205, dtype=np.uint8)
    gray[(values >= 0) & (values <= free_thresh * 100)] = 254
    gray[values >= occupied_thresh * 100] = 0
    mid = (values > free_thresh * 100) & (values < occupied_thresh * 100)
    gray[mid] = 205
    gray = gray[::-1]
    pgm = base + '.pgm'
    with open(pgm, 'wb') as fh:
        fh.write(f'P5\n{gray.shape[1]} {gray.shape[0]}\n255\n'.encode('ascii'))
        fh.write(gray.tobytes())
    yaml_path = base + '.yaml'
    with open(yaml_path, 'w', encoding='utf-8') as fh:
        fh.write(f'image: {os.path.basename(pgm)}\n')
        fh.write('mode: trinary\n')
        fh.write(f'resolution: {snapshot.resolution:.4f}\n')
        # origin en ENU: x = este, y = norte (esquina inferior izquierda)
        fh.write(f'origin: [{snapshot.origin_e:.4f}, {snapshot.origin_n:.4f}, 0.0]\n')
        fh.write('negate: 0\n')
        fh.write(f'occupied_thresh: {occupied_thresh}\n')
        fh.write(f'free_thresh: {free_thresh}\n')
    return pgm, yaml_path


def write_ply(path, points_ned, home_down=0.0):
    """Nube ASCII en ENU (x=este, y=norte, z=altura sobre el despegue)."""
    pts = np.asarray(points_ned, dtype=float).reshape(-1, 3)
    with open(path, 'w', encoding='ascii') as fh:
        fh.write('ply\nformat ascii 1.0\n')
        fh.write(f'element vertex {len(pts)}\n')
        fh.write('property float x\nproperty float y\nproperty float z\nend_header\n')
        for n, e, d in pts:
            fh.write(f'{e:.3f} {n:.3f} {home_down - d:.3f}\n')
    return path


# ----------------------------------------------------------------------
# Imagen raster
# ----------------------------------------------------------------------
def _draw_line(img, r0, c0, r1, c1, color, width=1):
    steps = int(max(abs(r1 - r0), abs(c1 - c0))) + 1
    rr = np.rint(np.linspace(r0, r1, steps)).astype(int)
    cc = np.rint(np.linspace(c0, c1, steps)).astype(int)
    half = width // 2
    for dr in range(-half, half + 1):
        for dc in range(-half, half + 1):
            r, c = rr + dr, cc + dc
            ok = (r >= 0) & (r < img.shape[0]) & (c >= 0) & (c < img.shape[1])
            img[r[ok], c[ok]] = color


def render_png(path, snapshot, walls, trajectory_ned=(), max_pixels=1600):
    gray = occupancy_to_gray(snapshot.values)
    scale = max(1, min(4, max_pixels // max(gray.shape)))
    img = np.repeat(np.repeat(np.stack([gray] * 3, axis=-1), scale, axis=0), scale, axis=1)
    rows_total = snapshot.values.shape[0]

    def to_px(north, east):
        row = (rows_total - (north - snapshot.origin_n) / snapshot.resolution) * scale
        col = ((east - snapshot.origin_e) / snapshot.resolution) * scale
        return row, col

    traj = list(trajectory_ned)
    for a, b in zip(traj[:-1], traj[1:]):
        _draw_line(img, *to_px(a[0], a[1]), *to_px(b[0], b[1]), TRAJECTORY_COLOR, 1)
    for wall in walls:
        verts = wall.vertices
        if wall.kind == OBSTACLE and len(verts) >= 3:
            verts = np.vstack((verts, verts[:1]))
        for a, b in zip(verts[:-1], verts[1:]):
            _draw_line(img, *to_px(a[0], a[1]), *to_px(b[0], b[1]),
                       WALL_COLORS[wall.kind], max(2, scale + 1))
    if traj:
        r, c = to_px(traj[0][0], traj[0][1])
        arm = 4 * scale
        _draw_line(img, r, c - arm, r, c + arm, HOME_COLOR, 2)
        _draw_line(img, r - arm, c, r + arm, c, HOME_COLOR, 2)
    write_png(path, img)
    return path


# ----------------------------------------------------------------------
# SVG
# ----------------------------------------------------------------------
def render_svg(path, snapshot, walls, trajectory_ned=(), title='Mapa de paredes'):
    """Plano vectorial. Coordenadas en metros: x = este, y = -norte."""
    values = snapshot.values
    known_rows, known_cols = np.nonzero(values >= 0)
    if len(known_rows):
        r0, r1 = known_rows.min(), known_rows.max() + 1
        c0, c1 = known_cols.min(), known_cols.max() + 1
    else:
        r0, r1, c0, c1 = 0, values.shape[0], 0, values.shape[1]
    res = snapshot.resolution
    n_min = snapshot.origin_n + r0 * res
    n_max = snapshot.origin_n + r1 * res
    e_min = snapshot.origin_e + c0 * res
    e_max = snapshot.origin_e + c1 * res
    margin = 2.0
    vb_x, vb_y = e_min - margin, -n_max - margin
    vb_w, vb_h = (e_max - e_min) + 2 * margin, (n_max - n_min) + 2 * margin + 3.0
    px_per_m = max(8.0, min(40.0, 1200.0 / max(vb_w, vb_h)))

    out = []
    out.append(f'<svg xmlns="http://www.w3.org/2000/svg" width="{vb_w * px_per_m:.0f}" '
               f'height="{vb_h * px_per_m:.0f}" viewBox="{vb_x:.2f} {vb_y:.2f} {vb_w:.2f} {vb_h:.2f}" '
               f'font-family="sans-serif">')
    out.append(f'<rect x="{vb_x:.2f}" y="{vb_y:.2f}" width="{vb_w:.2f}" height="{vb_h:.2f}" fill="#d9d9d9"/>')
    # Espacio libre conocido: blanco, por corridas de celdas en cada fila.
    out.append('<g fill="#ffffff" stroke="none">')
    out.extend(_runs_to_rects(snapshot, (values >= 0) & (values < 65)))
    out.append('</g><g fill="#333333" stroke="none">')
    out.extend(_runs_to_rects(snapshot, values >= 65))
    out.append('</g>')

    # Grilla cada 5 m.
    out.append('<g stroke="#bbbbbb" stroke-width="0.03">')
    for e in np.arange(math.floor(e_min / 5) * 5, e_max + 5, 5):
        out.append(f'<line x1="{e:.2f}" y1="{-n_max:.2f}" x2="{e:.2f}" y2="{-n_min:.2f}"/>')
    for n in np.arange(math.floor(n_min / 5) * 5, n_max + 5, 5):
        out.append(f'<line x1="{e_min:.2f}" y1="{-n:.2f}" x2="{e_max:.2f}" y2="{-n:.2f}"/>')
    out.append('</g>')

    traj = list(trajectory_ned)
    if len(traj) >= 2:
        pts = ' '.join(f'{p[1]:.2f},{-p[0]:.2f}' for p in traj)
        out.append(f'<polyline points="{pts}" fill="none" stroke="rgb{TRAJECTORY_COLOR}" '
                   'stroke-width="0.08" stroke-dasharray="0.3,0.2"/>')
    if traj:
        out.append(f'<circle cx="{traj[0][1]:.2f}" cy="{-traj[0][0]:.2f}" r="0.35" '
                   f'fill="rgb{HOME_COLOR}"/>')

    for wall in walls:
        color = f'rgb{WALL_COLORS[wall.kind]}'
        verts = wall.vertices
        if wall.kind == OBSTACLE and len(verts) >= 3:
            pts = ' '.join(f'{v[1]:.2f},{-v[0]:.2f}' for v in verts)
            out.append(f'<polygon points="{pts}" fill="{color}" fill-opacity="0.35" '
                       f'stroke="{color}" stroke-width="0.12"/>')
        else:
            pts = ' '.join(f'{v[1]:.2f},{-v[0]:.2f}' for v in verts)
            out.append(f'<polyline points="{pts}" fill="none" stroke="{color}" '
                       'stroke-width="0.18" stroke-linecap="round" stroke-linejoin="round"/>')
        label_at = wall.vertices.mean(axis=0)
        text = f'P{wall.id} {KIND_NAMES[wall.kind]} {wall.length:.1f}m'
        if math.isfinite(wall.tilt_deg) and wall.tilt_deg >= 5.0:
            text += f' incl {wall.tilt_deg:.0f}°'
        out.append(f'<text x="{label_at[1] + 0.3:.2f}" y="{-label_at[0] - 0.3:.2f}" '
                   f'font-size="0.6" fill="{color}">{text}</text>')

    # Escala, norte y leyenda.
    sx, sy = e_min, -n_min + 1.5
    out.append(f'<line x1="{sx:.2f}" y1="{sy:.2f}" x2="{sx + 5:.2f}" y2="{sy:.2f}" '
               'stroke="black" stroke-width="0.12"/>')
    out.append(f'<text x="{sx:.2f}" y="{sy + 0.9:.2f}" font-size="0.7">5 m</text>')
    nx, ny = e_max - 0.5, -n_max + 0.2
    out.append(f'<path d="M {nx:.2f} {ny - 1.2:.2f} L {nx - 0.4:.2f} {ny:.2f} '
               f'L {nx + 0.4:.2f} {ny:.2f} Z" fill="black"/>')
    out.append(f'<text x="{nx - 0.25:.2f}" y="{ny + 0.8:.2f}" font-size="0.7">N</text>')
    lx = sx + 7
    for i, kind in enumerate((FLAT, IRREGULAR, OBSTACLE)):
        x = lx + i * 6
        out.append(f'<line x1="{x:.2f}" y1="{sy:.2f}" x2="{x + 1:.2f}" y2="{sy:.2f}" '
                   f'stroke="rgb{WALL_COLORS[kind]}" stroke-width="0.2"/>')
        out.append(f'<text x="{x + 1.3:.2f}" y="{sy + 0.25:.2f}" font-size="0.6">'
                   f'{KIND_NAMES[kind]}</text>')
    out.append(f'<text x="{e_min:.2f}" y="{-n_max - 0.8:.2f}" font-size="0.9">{title}</text>')
    out.append('</svg>')
    with open(path, 'w', encoding='utf-8') as fh:
        fh.write('\n'.join(out))
    return path


def _runs_to_rects(snapshot, mask):
    rects = []
    res = snapshot.resolution
    for row in np.nonzero(mask.any(axis=1))[0]:
        line = mask[row].astype(np.int8)
        edges = np.diff(np.concatenate(([0], line, [0])))
        starts = np.nonzero(edges == 1)[0]
        stops = np.nonzero(edges == -1)[0]
        north_top = snapshot.origin_n + (row + 1) * res
        for s, t in zip(starts, stops):
            east = snapshot.origin_e + s * res
            rects.append(f'<rect x="{east:.2f}" y="{-north_top:.2f}" '
                         f'width="{(t - s) * res:.3f}" height="{res:.3f}"/>')
    return rects


# ----------------------------------------------------------------------
# JSON + paquete completo
# ----------------------------------------------------------------------
def walls_to_dict(walls, home_down=0.0):
    out = []
    for w in walls:
        out.append({
            'id': int(w.id),
            'tipo': KIND_NAMES[w.kind],
            'vertices_ned': [[round(float(v[0]), 3), round(float(v[1]), 3)] for v in w.vertices],
            'vertices_enu': [[round(float(v[1]), 3), round(float(v[0]), 3)] for v in w.vertices],
            'largo_m': round(float(w.length), 3),
            'espesor_m': round(float(w.thickness), 3),
            'rms_m': round(float(w.rms), 4),
            'desvio_max_m': round(float(w.max_deviation), 3),
            'flecha_m': None if not math.isfinite(w.bow) else round(float(w.bow), 3),
            'altura_min_m': None if not math.isfinite(w.height_min) else round(w.height_min, 2),
            'altura_max_m': None if not math.isfinite(w.height_max) else round(w.height_max, 2),
            'inclinacion_deg': None if not math.isfinite(w.tilt_deg) else round(w.tilt_deg, 1),
            'celdas': int(w.cells),
        })
    return out


def save_map_bundle(directory, basename, mapper, extra_meta=None):
    """Guarda todo lo que sabe el mapper. Devuelve la lista de archivos."""
    os.makedirs(directory, exist_ok=True)
    base = os.path.join(directory, basename)
    snapshot = mapper.snapshot()
    walls = mapper.walls
    files = list(write_pgm_yaml(base, snapshot))
    files.append(render_png(base + '.png', snapshot, walls, mapper.trajectory))
    files.append(render_svg(base + '.svg', snapshot, walls, mapper.trajectory,
                            title=f'Mapa {basename}'))
    meta = {
        'resolucion_m': snapshot.resolution,
        'origen_ned': [snapshot.origin_n, snapshot.origin_e],
        'celdas': list(snapshot.values.shape),
        'paredes_planas': sum(1 for w in walls if w.kind == FLAT),
        'paredes_irregulares': sum(1 for w in walls if w.kind == IRREGULAR),
        'obstaculos': sum(1 for w in walls if w.kind == OBSTACLE),
        'nubes_integradas': mapper.clouds_integrated,
        'puntos_integrados': mapper.points_integrated,
        'trayectoria_ned': [[round(p[0], 2), round(p[1], 2), round(p[2], 2)] for p in mapper.trajectory],
    }
    if extra_meta:
        meta.update(extra_meta)
    json_path = base + '_walls.json'
    with open(json_path, 'w', encoding='utf-8') as fh:
        json.dump({'meta': meta, 'paredes': walls_to_dict(walls, mapper.home_down or 0.0)},
                  fh, ensure_ascii=False, indent=2)
    files.append(json_path)
    voxels = mapper.voxels.points(mapper.cfg.voxel_min_hits)
    files.append(write_ply(base + '_cloud.ply', voxels, mapper.home_down or 0.0))
    return files
