#!/usr/bin/env python3
"""Extracción y clasificación de paredes a partir del mapa. Python puro + numpy.

Entrada: los centros de las celdas ocupadas de la grilla 2D (metros, NED), la
dirección desde la que el sensor vio cada celda y, si está, el mapa 3D de
vóxeles. Salida: una lista de `Wall` de tres tipos:

- **FLAT** (plana): un segmento recto. Con datos 3D se ajusta un plano a los
  puntos de la pared (así una pared inclinada también es plana, y se mide su
  inclinación); sin ellos, una recta en planta. Se exige poca dispersión por
  cara (`flat_max_rms`), que no esté combada (`flat_max_bow`: la flecha de una
  parábola ajustada separa una pared curva de una recta con ruido), un espesor
  razonable y un largo mínimo.
- **IRREGULAR**: una polilínea. Paredes con salientes, dentadas, curvas, de
  escombros, o cualquier tramo que no pase la prueba de "plana".
- **OBSTACLE**: algo sólido y compacto (una columna, una caja): un polígono.

"Por cara" importa: si el dron pasó por los dos lados de una pared de 30 cm,
sus puntos forman dos rectas paralelas y el rms contra la del medio da ~15 cm
aunque la pared sea perfectamente plana. Las dos caras se separan usando
**desde qué lado las vio el sensor**: cada cara de una pared gruesa se ve
desde su propio lado, mientras que los dientes de una pared irregular se ven
todos desde el mismo. Sin esa información se cae a una partición geométrica
(2-medias) con un chequeo de alternancia, que es más fácil de engañar.

El algoritmo, en orden:

1. Las celdas se agrupan en nodos de `spine_resolution` (más gruesos que la
   grilla) y los nodos se unen en componentes conexas con radio
   `link_distance`: así una pared con un hueco chico sigue siendo una sola.
2. A cada componente se le saca un **esqueleto**: el camino más largo dentro del
   grafo (dos barridas de Dijkstra). Lo que el esqueleto no cubre (la otra
   mitad de una habitación cerrada, una T) se vuelve a procesar como rama.
3. El esqueleto se simplifica con Ramer-Douglas-Peucker (`rdp_tolerance`).
4. Cada punto se asigna a la arista más cercana y cada arista se prueba como
   pared plana. Las puntas de cada arista (`corner_trim`) no cuentan para la
   dispersión: ahí se cuelan celdas de la pared que dobla en la esquina.
5. Aristas planas consecutivas casi alineadas se funden; aristas no planas
   consecutivas forman una sola pared irregular.
6. Post-proceso global: se unen paredes planas colineales separadas por menos de
   `merge_gap` (huecos de lectura, no puertas), y todo lo que no es plano se
   reagrupa con un radio mayor (`join_distance`): los fragmentos de una pared
   de escombros vuelven a ser una sola pared irregular, y lo compacto queda
   como obstáculo.
"""

import heapq
import math
from collections import deque
from dataclasses import dataclass, field

import numpy as np

FLAT = 0
IRREGULAR = 1
OBSTACLE = 2
KIND_NAMES = {FLAT: 'plana', IRREGULAR: 'irregular', OBSTACLE: 'obstaculo'}


@dataclass
class Wall:
    kind: int
    vertices: np.ndarray            # (k, 2) NE. Plana: 2 extremos. Obstáculo: polígono
    length: float
    thickness: float
    rms: float
    max_deviation: float
    bow: float
    cells: int
    points: np.ndarray = field(repr=False)   # (N, 4): norte, este, vista_n, vista_e
    id: int = -1
    height_min: float = float('nan')
    height_max: float = float('nan')
    tilt_deg: float = float('nan')

    @property
    def kind_name(self):
        return KIND_NAMES[self.kind]

    @property
    def centroid(self):
        return self.points[:, :2].mean(axis=0) if len(self.points) else self.vertices.mean(axis=0)


# ----------------------------------------------------------------------
# Utilidades geométricas
# ----------------------------------------------------------------------
def point_segment_distances(points, a, b):
    """Distancia de cada punto (N, 2) al segmento a-b."""
    p = np.asarray(points, dtype=float).reshape(-1, 2)
    a = np.asarray(a, dtype=float)
    b = np.asarray(b, dtype=float)
    ab = b - a
    denom = float(ab @ ab)
    if denom < 1e-12:
        return np.hypot(p[:, 0] - a[0], p[:, 1] - a[1])
    t = np.clip(((p - a) @ ab) / denom, 0.0, 1.0)
    proj = a + t[:, None] * ab
    return np.hypot(p[:, 0] - proj[:, 0], p[:, 1] - proj[:, 1])


def polyline_distances(points, vertices):
    """Distancia de cada punto a una polilínea (mínimo sobre sus aristas)."""
    points = np.asarray(points, dtype=float).reshape(-1, 2)
    vertices = np.asarray(vertices, dtype=float).reshape(-1, 2)
    if len(vertices) == 1:
        return np.hypot(points[:, 0] - vertices[0, 0], points[:, 1] - vertices[0, 1])
    best = np.full(len(points), np.inf)
    for i in range(len(vertices) - 1):
        best = np.minimum(best, point_segment_distances(points, vertices[i], vertices[i + 1]))
    return best


def polyline_length(vertices):
    v = np.asarray(vertices, dtype=float).reshape(-1, 2)
    return float(np.sum(np.hypot(*np.diff(v, axis=0).T))) if len(v) > 1 else 0.0


def rdp(vertices, tolerance):
    """Ramer-Douglas-Peucker iterativo. Devuelve los índices conservados."""
    n = len(vertices)
    if n <= 2:
        return list(range(n))
    keep = np.zeros(n, dtype=bool)
    keep[0] = keep[-1] = True
    stack = [(0, n - 1)]
    while stack:
        start, end = stack.pop()
        if end - start < 2:
            continue
        d = point_segment_distances(vertices[start + 1:end], vertices[start], vertices[end])
        k = int(np.argmax(d))
        if d[k] > tolerance:
            mid = start + 1 + k
            keep[mid] = True
            stack.append((start, mid))
            stack.append((mid, end))
    return list(np.nonzero(keep)[0])


def convex_hull(points):
    """Cápsula convexa (Andrew). Devuelve los vértices en orden, sin cerrar."""
    pts = sorted(set(map(tuple, np.round(np.asarray(points, dtype=float), 6).tolist())))
    if len(pts) <= 2:
        return np.array(pts, dtype=float).reshape(-1, 2)

    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    lower, upper = [], []
    for p in pts:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 0:
            lower.pop()
        lower.append(p)
    for p in reversed(pts):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 0:
            upper.pop()
        upper.append(p)
    return np.array(lower[:-1] + upper[:-1], dtype=float)


# ----------------------------------------------------------------------
# Métricas de planitud
# ----------------------------------------------------------------------
def grouped_rms(residuals, groups):
    """rms de los residuos respecto de la media de su propio grupo."""
    r = np.asarray(residuals, dtype=float)
    total = 0.0
    for g in np.unique(groups):
        sel = r[groups == g]
        total += float(np.sum((sel - sel.mean()) ** 2))
    return math.sqrt(total / max(len(r), 1))


def two_face_rms(residuals, along=None, bin_size=0.5):
    """Partición geométrica en dos caras, para cuando no se sabe desde dónde se
    vio cada punto: 2-medias exacto en 1D, rechazado si los dos grupos se
    alternan a lo largo de la pared (eso es una pared con salientes)."""
    r = np.asarray(residuals, dtype=float)
    n = len(r)
    if n < 4:
        return float(np.std(r)) if n else 0.0
    single = float(np.std(r))
    order = np.argsort(r)
    rs = r[order]
    csum = np.cumsum(rs)
    csum2 = np.cumsum(rs * rs)
    k = np.arange(2, n - 1)
    if not len(k):
        return single
    left = csum2[k - 1] - csum[k - 1] ** 2 / k
    right = (csum2[-1] - csum2[k - 1]) - (csum[-1] - csum[k - 1]) ** 2 / (n - k)
    best = int(np.argmin(left + right))
    split = math.sqrt(max(float(left[best] + right[best]), 0.0) / n)
    if split >= single:
        return single
    if along is not None:
        labels = np.zeros(n, dtype=bool)
        labels[order[k[best]:]] = True
        t = np.asarray(along, dtype=float)
        bins = np.floor((t - t.min()) / bin_size).astype(np.int64)
        n_bins = int(bins.max()) + 1
        has_a = np.bincount(bins[~labels], minlength=n_bins) > 0
        has_b = np.bincount(bins[labels], minlength=n_bins) > 0
        seq = [('A' if a and not b else 'B' if b and not a else 'AB')
               for a, b in zip(has_a, has_b) if a or b]
        if sum(1 for x, y in zip(seq[:-1], seq[1:]) if {x, y} == {'A', 'B'}) >= 1:
            return single
    return split


def face_rms(residuals, along, views, normal2d):
    """Dispersión por cara: por lado de observación si se sabe, si no geométrica."""
    if views is not None and len(views):
        v = np.asarray(views, dtype=float)
        strength = np.hypot(v[:, 0], v[:, 1])
        if np.mean(strength > 0.2) > 0.8:
            side = (v @ np.asarray(normal2d, dtype=float)) > 0.0
            return grouped_rms(residuals, side)
    return two_face_rms(residuals, along)


def _bow(t, r, half_length):
    """Flecha de la parábola que mejor ajusta r(t), escalada al largo completo:
    ~0 en una recta con ruido, la sagita del arco en una curva. El coeficiente
    cuadrático no depende del largo, así que se puede estimar con el tramo
    central (sin esquinas) y escalar a toda la pared."""
    if len(t) < 6 or half_length < 1e-6:
        return 0.0
    coeffs = np.polyfit(t - (t.max() + t.min()) / 2.0, r, 2)
    return float(abs(coeffs[0]) * half_length * half_length)


@dataclass
class LineFit:
    origin: np.ndarray
    direction: np.ndarray
    t_min: float
    t_max: float
    rms: float
    max_deviation: float
    thickness: float
    bow: float
    face_rms: float = 0.0

    @property
    def length(self):
        return self.t_max - self.t_min

    @property
    def normal(self):
        return np.array([-self.direction[1], self.direction[0]])

    @property
    def endpoints(self):
        return np.vstack((self.origin + self.direction * self.t_min,
                          self.origin + self.direction * self.t_max))


def fit_line(points, trim=0.0, views=None):
    """Recta por mínimos cuadrados totales (PCA) + métricas de planitud.

    Las métricas (no los extremos) se calculan sin los `trim` metros de cada
    punta, donde se cuelan celdas de la pared que dobla en la esquina.
    """
    p = np.asarray(points, dtype=float).reshape(-1, 2)
    center = p.mean(axis=0)
    if len(p) < 2:
        return LineFit(center, np.array([1.0, 0.0]), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    q = p - center
    _, eigvecs = np.linalg.eigh(q.T @ q / len(p))
    direction = eigvecs[:, 1]
    t = q @ direction
    t_min, t_max = float(t.min()), float(t.max())
    core = np.ones(len(p), dtype=bool)
    if trim > 0 and (t_max - t_min) > 2 * trim + 0.5:
        core = (t >= t_min + trim) & (t <= t_max - trim)
        if core.sum() >= 4:
            # Reajuste con el centro: las esquinas también tuercen la dirección.
            qc = p[core] - p[core].mean(axis=0)
            _, ev = np.linalg.eigh(qc.T @ qc / len(qc))
            if abs(float(ev[:, 1] @ direction)) > 0.9:
                direction = ev[:, 1]
                center = p[core].mean(axis=0)
                q = p - center
                t = q @ direction
                t_min, t_max = float(t.min()), float(t.max())
        else:
            core[:] = True
    normal = np.array([-direction[1], direction[0]])
    r = q @ normal
    rc, tc = r[core], t[core]
    vc = None if views is None else np.asarray(views, dtype=float).reshape(-1, 2)[core]
    return LineFit(center, direction, t_min, t_max, float(np.sqrt(np.mean(rc ** 2))),
                   float(np.abs(rc).max()), float(rc.max() - rc.min()),
                   _bow(tc, rc, (t_max - t_min) / 2.0), face_rms(rc, tc, vc, normal))


@dataclass
class PlaneFit:
    normal: np.ndarray
    rms: float
    face_rms: float
    thickness: float
    bow: float
    tilt_deg: float


def fit_plane(points_xyz, views=None, half_length=None):
    """Plano por PCA sobre puntos (norte, este, altura). La inclinación es el
    ángulo del plano respecto de la vertical (0° = pared a plomo)."""
    p = np.asarray(points_xyz, dtype=float).reshape(-1, 3)
    q = p - p.mean(axis=0)
    eigvals, eigvecs = np.linalg.eigh(q.T @ q / max(len(p), 1))
    normal = eigvecs[:, 0]
    r = q @ normal
    along = np.cross(normal, np.array([0.0, 0.0, 1.0]))
    if np.linalg.norm(along) < 1e-6:
        along = eigvecs[:, 2]
    along = along / np.linalg.norm(along)
    t = q @ along
    if half_length is None:
        half_length = (t.max() - t.min()) / 2.0
    n2 = normal[:2]
    n2 = n2 / np.linalg.norm(n2) if np.linalg.norm(n2) > 1e-6 else np.array([1.0, 0.0])
    return PlaneFit(normal=normal, rms=float(math.sqrt(max(eigvals[0], 0.0))),
                    face_rms=face_rms(r, t, views, n2), thickness=float(r.max() - r.min()),
                    bow=_bow(t, r, half_length),
                    tilt_deg=float(math.degrees(math.asin(min(1.0, abs(normal[2]))))))


# ----------------------------------------------------------------------
# Grafo de nodos gruesos
# ----------------------------------------------------------------------
class _Graph:
    """Celdas agrupadas en nodos de `cell` metros, conectados hasta `link` m."""

    def __init__(self, points_ne, cell, link):
        keys = np.floor(points_ne / cell).astype(np.int64)
        uniq, inverse = np.unique(keys, axis=0, return_inverse=True)
        inverse = inverse.reshape(-1)
        counts = np.bincount(inverse, minlength=len(uniq))
        self.pos = np.zeros((len(uniq), 2))
        np.add.at(self.pos, inverse, points_ne)
        self.pos /= counts[:, None]
        self.keys = [tuple(k) for k in uniq.tolist()]
        self.index = {k: i for i, k in enumerate(self.keys)}
        order = np.argsort(inverse, kind='stable')
        splits = np.cumsum(counts)[:-1]
        self.members = np.split(order, splits)
        radius = max(1, int(math.ceil(link / cell)))
        self.offsets = [(dr, dc, math.hypot(dr, dc))
                        for dr in range(-radius, radius + 1) for dc in range(-radius, radius + 1)
                        if (dr, dc) != (0, 0) and math.hypot(dr, dc) <= radius + 0.5]

    def __len__(self):
        return len(self.keys)

    def neighbors(self, node, allowed):
        r, c = self.keys[node]
        for dr, dc, weight in self.offsets:
            other = self.index.get((r + dr, c + dc))
            if other is not None and other in allowed:
                yield other, weight

    def components(self, nodes):
        allowed = set(nodes)
        seen = set()
        for start in nodes:
            if start in seen:
                continue
            seen.add(start)
            queue = deque([start])
            component = []
            while queue:
                node = queue.popleft()
                component.append(node)
                for other, _ in self.neighbors(node, allowed):
                    if other not in seen:
                        seen.add(other)
                        queue.append(other)
            yield component

    def cells(self, nodes):
        return np.concatenate([self.members[n] for n in nodes]) if nodes else np.zeros(0, dtype=int)

    def _dijkstra(self, source, allowed):
        dist = {source: 0.0}
        parent = {source: None}
        heap = [(0.0, source)]
        while heap:
            d, node = heapq.heappop(heap)
            if d > dist[node]:
                continue
            for other, weight in self.neighbors(node, allowed):
                nd = d + weight
                if nd < dist.get(other, math.inf):
                    dist[other] = nd
                    parent[other] = node
                    heapq.heappush(heap, (nd, other))
        return dist, parent

    def spine(self, nodes):
        """Camino más largo (aprox.): doble barrida de Dijkstra, arrancando desde
        el nodo más extremo sobre el eje principal (en una L o una U eso ya cae
        cerca de una punta)."""
        allowed = set(nodes)
        pos = self.pos[nodes]
        center = pos.mean(axis=0)
        if len(nodes) >= 2:
            _, vecs = np.linalg.eigh(np.cov((pos - center).T) + np.eye(2) * 1e-9)
            start = nodes[int(np.argmin((pos - center) @ vecs[:, 1]))]
        else:
            start = nodes[0]
        dist, _ = self._dijkstra(start, allowed)
        a = max(dist, key=dist.get)
        dist, parent = self._dijkstra(a, allowed)
        node = max(dist, key=dist.get)
        path = []
        while node is not None:
            path.append(node)
            node = parent[node]
        return path[::-1]

    def polylines(self, component, cover_radius, min_nodes, max_branches):
        """Esqueleto principal + ramas para lo que quedó a más de `cover_radius`."""
        out = []
        pending = [list(component)]
        while pending and len(out) < max_branches:
            nodes = pending.pop()
            path = self.spine(nodes)
            spine = self.pos[path]
            out.append(spine)
            on_path = set(path)
            remaining = [n for n in nodes if n not in on_path]
            if not remaining:
                continue
            d = polyline_distances(self.pos[remaining], spine)
            leftovers = [n for n, dist in zip(remaining, d) if dist > cover_radius]
            for sub in self.components(leftovers):
                if len(sub) >= min_nodes:
                    pending.append(sub)
        return out


# ----------------------------------------------------------------------
# Extractor
# ----------------------------------------------------------------------
class WallExtractor:

    def __init__(self, resolution=0.15, spine_resolution=0.3, link_distance=0.6,
                 min_cells=5, rdp_tolerance=0.3, flat_max_rms=0.08,
                 max_wall_thickness=0.8, flat_max_bow=0.06, min_flat_length=1.0,
                 min_wall_length=1.0, max_obstacle_extent=3.0, merge_angle_deg=12.0,
                 merge_gap=0.6, join_distance=1.0, min_irregular_length=0.6,
                 corner_trim=0.4, cover_radius=0.7, max_branches=40,
                 min_voxels_3d=15, min_height_span_3d=0.8):
        self.resolution = float(resolution)
        self.spine_res = max(float(spine_resolution), self.resolution)
        self.link_distance = float(link_distance)
        self.min_cells = int(min_cells)
        self.rdp_tolerance = float(rdp_tolerance)
        self.flat_max_rms = float(flat_max_rms)
        self.max_wall_thickness = float(max_wall_thickness)
        self.flat_max_bow = float(flat_max_bow)
        self.min_flat_length = float(min_flat_length)
        self.min_wall_length = float(min_wall_length)
        self.max_obstacle_extent = float(max_obstacle_extent)
        self.merge_angle = math.radians(merge_angle_deg)
        self.merge_gap = float(merge_gap)
        self.join_distance = float(join_distance)
        self.min_irregular_length = float(min_irregular_length)
        self.corner_trim = float(corner_trim)
        self.cover_radius = float(cover_radius)
        self.max_branches = int(max_branches)
        self.min_voxels_3d = int(min_voxels_3d)
        self.min_height_span_3d = float(min_height_span_3d)
        self._vox = None      # (N, 5): norte, este, altura, vista_n, vista_e

    # ------------------------------------------------------------------
    def extract(self, occupied_ne, views=None, voxels_ned=None, voxel_views=None,
                home_down=0.0, min_height=0.3):
        """Paredes a partir de los centros de celdas ocupadas (N, 2).

        `views`: (N, 2) dirección horizontal media desde el sensor hacia cada
        celda (opcional). `voxels_ned` / `voxel_views`: mapa 3D (opcional).
        """
        pts = np.asarray(occupied_ne, dtype=float).reshape(-1, 2)
        view = (np.zeros_like(pts) if views is None
                else np.asarray(views, dtype=float).reshape(-1, 2))
        pts4 = np.column_stack((pts, view))

        self._vox = None
        if voxels_ned is not None and len(voxels_ned):
            vox = np.asarray(voxels_ned, dtype=float).reshape(-1, 3)
            vv = (np.zeros((len(vox), 2)) if voxel_views is None
                  else np.asarray(voxel_views, dtype=float).reshape(-1, 2))
            heights = home_down - vox[:, 2]
            keep = heights >= min_height
            self._vox = np.column_stack((vox[keep, 0], vox[keep, 1], heights[keep], vv[keep]))
        if len(pts4) < self.min_cells:
            return []

        graph = _Graph(pts4[:, :2], self.spine_res, self.link_distance)
        walls = []
        for component in graph.components(range(len(graph))):
            cells = graph.cells(component)
            if len(cells) < self.min_cells:
                continue
            walls.extend(self._process_component(graph, component, pts4[cells], cells))
        return self._postprocess(walls)

    # ------------------------------------------------------------------
    # Componente -> paredes
    # ------------------------------------------------------------------
    def _obstacle(self, pts4):
        points = pts4[:, :2]
        fit = fit_line(points)
        extent = float(np.ptp(points, axis=0).max()) if len(points) > 1 else 0.0
        return Wall(kind=OBSTACLE, vertices=convex_hull(points), length=extent,
                    thickness=fit.thickness, rms=fit.rms, max_deviation=fit.max_deviation,
                    bow=fit.bow, cells=len(points), points=pts4)

    def _edges(self, graph, component, points):
        min_nodes = max(2, int(math.ceil(self.min_wall_length / self.spine_res / 2)))
        polylines = graph.polylines(component, self.cover_radius, min_nodes, self.max_branches)
        simplified = [s[rdp(s, self.rdp_tolerance)] for s in polylines if len(s) >= 2]
        edges = []
        for poly_id, poly in enumerate(simplified):
            for i in range(len(poly) - 1):
                edges.append((poly_id, poly[i], poly[i + 1]))
        if not edges:
            return simplified, edges, None
        dists = np.column_stack([point_segment_distances(points, a, b) for _, a, b in edges])
        return simplified, edges, np.argmin(dists, axis=1)

    def _process_component(self, graph, component, pts4, _cells):
        points = pts4[:, :2]
        extent = float(np.ptp(points, axis=0).max()) if len(points) > 1 else 0.0
        if extent < self.min_wall_length:
            return [self._obstacle(pts4)]
        simplified, edges, owner = self._edges(graph, component, points)
        if not edges:
            return [self._obstacle(pts4)]

        walls = []
        for poly_id in range(len(simplified)):
            infos = []
            for k, edge in enumerate(edges):
                if edge[0] != poly_id:
                    continue
                sub = pts4[owner == k]
                flat, fit, plane = self._flat_check(sub)
                infos.append({'edge': edge, 'points': sub, 'fit': fit, 'plane': plane,
                              'flat': flat})
            walls.extend(self._group_edges(infos))

        # Algo compacto que no es una sola pared plana (una caja vista de dos
        # lados, una columna gruesa) se dibuja como obstáculo.
        single_flat = len(walls) == 1 and walls[0].kind == FLAT
        if extent < self.max_obstacle_extent and not single_flat:
            return [self._obstacle(pts4)]
        return walls

    def _flat_check(self, pts4):
        """(¿es plana?, ajuste 2D, ajuste 3D o None)."""
        if len(pts4) < 4:
            return False, (fit_line(pts4[:, :2]) if len(pts4) >= 2 else None), None
        fit = fit_line(pts4[:, :2], trim=self.corner_trim, views=pts4[:, 2:4])
        if fit.length < self.min_flat_length:
            return False, fit, None
        plane = self._plane_near(fit)
        limit = self.max_wall_thickness + 2 * self.resolution
        if plane is not None:
            # El espesor del plano se mide perpendicular a él: una pared
            # inclinada no "engorda" acá aunque su huella en planta sea ancha.
            flat = (plane.face_rms <= self.flat_max_rms and plane.bow <= self.flat_max_bow
                    and plane.thickness <= limit)
        else:
            flat = (fit.face_rms <= self.flat_max_rms and fit.bow <= self.flat_max_bow
                    and fit.thickness <= limit)
        return flat, fit, plane

    def _plane_near(self, fit):
        """Ajuste de plano 3D con los vóxeles de este tramo (sin las puntas)."""
        if self._vox is None or not len(self._vox):
            return None
        a, b = fit.endpoints
        buffer = max(0.3, fit.thickness / 2.0 + 0.25)
        lo = np.minimum(a, b) - buffer
        hi = np.maximum(a, b) + buffer
        vox = self._vox
        box = ((vox[:, 0] >= lo[0]) & (vox[:, 0] <= hi[0])
               & (vox[:, 1] >= lo[1]) & (vox[:, 1] <= hi[1]))
        if np.count_nonzero(box) < self.min_voxels_3d:
            return None
        cand = vox[box]
        rel = cand[:, :2] - fit.origin
        t = rel @ fit.direction
        across = np.abs(rel @ fit.normal)
        trim = self.corner_trim if fit.length > 2 * self.corner_trim + 0.5 else 0.0
        cand = cand[(t >= fit.t_min + trim) & (t <= fit.t_max - trim) & (across <= buffer)]
        if len(cand) < self.min_voxels_3d:
            return None
        if float(cand[:, 2].max() - cand[:, 2].min()) < self.min_height_span_3d:
            return None
        return fit_plane(cand[:, :3], views=cand[:, 3:5], half_length=fit.length / 2.0)

    def _flat_wall(self, pts4, fit, plane=None):
        wall = Wall(kind=FLAT, vertices=fit.endpoints, length=fit.length,
                    thickness=fit.thickness, rms=fit.rms, max_deviation=fit.max_deviation,
                    bow=fit.bow, cells=len(pts4), points=pts4)
        if plane is not None:
            wall.tilt_deg = plane.tilt_deg
        return wall

    def _irregular_wall(self, pts4, vertices):
        vertices = np.asarray(vertices, dtype=float)
        d = polyline_distances(pts4[:, :2], vertices) if len(pts4) else np.zeros(1)
        return Wall(kind=IRREGULAR, vertices=vertices, length=polyline_length(vertices),
                    thickness=float(2.0 * d.max()), rms=float(np.sqrt(np.mean(d ** 2))),
                    max_deviation=float(d.max()), bow=float('nan'), cells=len(pts4), points=pts4)

    def _group_edges(self, infos):
        """Agrupa aristas consecutivas: planas alineadas se funden, las
        irregulares consecutivas forman una sola pared."""
        # Un tramo plano corto pegado a uno irregular es una faceta de la misma
        # pared irregular (un diente, un tramo de una curva), no una pared plana
        # aparte. Se propaga hasta estabilizar.
        changed = True
        while changed:
            changed = False
            for k, info in enumerate(infos):
                if not info['flat'] or info['fit'].length >= 2.0 * self.min_flat_length:
                    continue
                if ((k > 0 and not infos[k - 1]['flat'])
                        or (k + 1 < len(infos) and not infos[k + 1]['flat'])):
                    info['flat'] = False
                    changed = True

        has_flat = any(info['flat'] for info in infos)
        walls = []
        i = 0
        while i < len(infos):
            if infos[i]['flat']:
                pts, fit, plane = infos[i]['points'], infos[i]['fit'], infos[i]['plane']
                j = i + 1
                while j < len(infos) and infos[j]['flat']:
                    nxt = infos[j]['fit']
                    if math.acos(min(1.0, abs(float(fit.direction @ nxt.direction)))) > self.merge_angle:
                        break
                    merged = np.vstack((pts, infos[j]['points']))
                    flat, merged_fit, merged_plane = self._flat_check(merged)
                    if not flat:
                        break
                    pts, fit, plane = merged, merged_fit, merged_plane
                    j += 1
                walls.append(self._flat_wall(pts, fit, plane))
                i = j
                continue

            j = i
            while j < len(infos) and not infos[j]['flat']:
                j += 1
            group = infos[i:j]
            pts = np.vstack([g['points'] for g in group])
            vertices = [group[0]['edge'][1]] + [g['edge'][2] for g in group]
            # Una arista corta no plana en una esquina entre paredes planas es un
            # artefacto (se mezclan celdas de las dos): no se dibuja como pared.
            # Sus celdas siguen en el mapa y en la evasión.
            if not (has_flat and polyline_length(vertices) < self.min_flat_length) and len(pts):
                walls.append(self._irregular_wall(pts, vertices))
            i = j
        return walls

    # ------------------------------------------------------------------
    # Post-proceso global
    # ------------------------------------------------------------------
    def _postprocess(self, walls):
        flats = self._merge_collinear([w for w in walls if w.kind == FLAT])
        others = [w for w in walls if w.kind != FLAT]
        regrouped = self._regroup(np.vstack([w.points for w in others])) if others else []
        return flats + [w for w in regrouped
                        if not (w.kind == IRREGULAR and w.length < self.min_irregular_length)]

    def _merge_collinear(self, flats):
        """Une paredes planas casi alineadas separadas por un hueco chico (falta
        de lectura, o las dos mitades de una habitación). Un hueco mayor a
        `merge_gap` se respeta: puede ser una puerta."""
        flats = list(flats)
        merged = True
        while merged:
            merged = False
            for a in range(len(flats)):
                for b in range(a + 1, len(flats)):
                    wa, wb = flats[a], flats[b]
                    da = wa.vertices[1] - wa.vertices[0]
                    db = wb.vertices[1] - wb.vertices[0]
                    na, nb = np.linalg.norm(da), np.linalg.norm(db)
                    if na < 1e-6 or nb < 1e-6:
                        continue
                    if math.acos(min(1.0, abs(float(da @ db) / (na * nb)))) > self.merge_angle:
                        continue
                    ua = da / na
                    normal = np.array([-ua[1], ua[0]])
                    offset = float(np.max(np.abs((wb.vertices - wa.vertices[0]) @ normal)))
                    tolerance = max(wa.thickness, wb.thickness) / 2.0 + self.max_wall_thickness / 2.0
                    if offset > tolerance:
                        continue
                    ta = sorted(float((v - wa.vertices[0]) @ ua) for v in wa.vertices)
                    tb = sorted(float((v - wa.vertices[0]) @ ua) for v in wb.vertices)
                    if max(tb[0] - ta[1], ta[0] - tb[1]) > self.merge_gap:
                        continue
                    pts = np.vstack((wa.points, wb.points))
                    flat, fit, plane = self._flat_check(pts)
                    if flat:
                        flats[a] = self._flat_wall(pts, fit, plane)
                        flats.pop(b)
                        merged = True
                        break
                if merged:
                    break
        return flats

    def _regroup(self, pts4):
        """Reagrupa todo lo no plano con radio `join_distance`: los fragmentos de
        una pared de escombros vuelven a ser una pared irregular, y lo compacto
        (columnas, cajas) queda como obstáculo."""
        if len(pts4) == 0:
            return []
        graph = _Graph(pts4[:, :2], self.spine_res, self.join_distance)
        out = []
        for component in graph.components(range(len(graph))):
            sub = pts4[graph.cells(component)]
            extent = float(np.ptp(sub[:, :2], axis=0).max()) if len(sub) > 1 else 0.0
            if extent < self.max_obstacle_extent:
                if len(sub) >= self.min_cells:
                    out.append(self._obstacle(sub))
                continue
            simplified, edges, owner = self._edges(graph, component, sub[:, :2])
            if not edges:
                out.append(self._obstacle(sub))
                continue
            for poly_id, poly in enumerate(simplified):
                ids = [k for k, e in enumerate(edges) if e[0] == poly_id]
                assigned = sub[np.isin(owner, ids)]
                if len(assigned):
                    out.append(self._irregular_wall(assigned, poly))
        return out


# ----------------------------------------------------------------------
# Datos 3D por pared
# ----------------------------------------------------------------------
def annotate_3d(walls, voxels_ned, home_down=0.0, min_points=8):
    """Completa altura mínima/máxima (y la inclinación si faltaba) con el mapa 3D.

    Alturas sobre el punto de despegue (positivas hacia arriba).
    """
    if voxels_ned is None or len(voxels_ned) == 0:
        return walls
    vox = np.asarray(voxels_ned, dtype=float)
    heights = home_down - vox[:, 2]
    for wall in walls:
        buffer = max(0.35, wall.thickness / 2.0 + 0.15)
        lo = wall.vertices.min(axis=0) - buffer
        hi = wall.vertices.max(axis=0) + buffer
        box = ((vox[:, 0] >= lo[0]) & (vox[:, 0] <= hi[0])
               & (vox[:, 1] >= lo[1]) & (vox[:, 1] <= hi[1]))
        if not np.any(box):
            continue
        candidates = vox[box]
        cand_h = heights[box]
        outline = wall.vertices
        if wall.kind == OBSTACLE and len(outline) >= 3:
            outline = np.vstack((outline, outline[:1]))
        near = polyline_distances(candidates[:, :2], outline) <= buffer
        if np.count_nonzero(near) < min_points:
            continue
        h = cand_h[near]
        wall.height_min = float(np.percentile(h, 2))
        wall.height_max = float(np.percentile(h, 98))
        if wall.kind == FLAT and not math.isfinite(wall.tilt_deg) and np.ptp(h) >= 0.8:
            wall.tilt_deg = fit_plane(np.column_stack((candidates[near][:, :2], h))).tilt_deg
    return walls


class WallTracker:
    """Ids estables entre extracciones sucesivas, para que los marcadores de
    Foxglove no titilen ni cambien de color cada segundo."""

    def __init__(self, match_distance=1.5):
        self.match_distance = float(match_distance)
        self._previous = []
        self._next_id = 0

    def assign(self, walls):
        used = set()
        for wall in sorted(walls, key=lambda w: -w.cells):
            best, best_d = None, self.match_distance
            for prev_id, prev_kind, prev_centroid in self._previous:
                if prev_id in used or prev_kind != wall.kind:
                    continue
                d = float(np.hypot(*(wall.centroid - prev_centroid)))
                if d < best_d:
                    best, best_d = prev_id, d
            if best is None:
                best = self._next_id
                self._next_id += 1
            used.add(best)
            wall.id = best
        self._previous = [(w.id, w.kind, w.centroid) for w in walls]
        return walls

    def reset(self):
        self._previous = []
        self._next_id = 0
