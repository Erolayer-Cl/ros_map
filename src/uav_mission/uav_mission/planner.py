#!/usr/bin/env python3
"""Planificador global A* sobre la grilla de ocupación. Python puro + numpy.

Resuelve lo que la evasión reactiva no puede: trampas en U, pasillos que doblan,
habitaciones con una sola puerta. Se replanifica cada `nav.replan_period`
segundos con el mapa que se va construyendo, así que el camino mejora a medida
que el dron ve más.

Detalles que importan:

- Se planifica a una resolución más gruesa que el mapa (`plan.resolution`) y
  sólo en una ventana alrededor de inicio y objetivo: A* en Python sobre la
  grilla completa a 0.15 m sería demasiado lento para la Jetson.
- Los obstáculos se inflan `inflation_radius` (radio del dron + margen) y
  además hay un costo suave hasta `cost_radius`: el camino prefiere ir por el
  medio de un pasillo en vez de rozar una pared.
- Lo desconocido se puede cruzar pero cuesta más (`unknown_penalty`): es la
  hipótesis optimista que necesita la exploración. Si resulta haber una pared,
  la próxima replanificación la esquiva.
- Si el dron ya está dentro de la zona inflada (pasa al bordear de cerca), se
  arranca desde la celda libre más cercana en vez de fallar.
- La distancia a los obstáculos se mide en la grilla FINA (la del mapa) y
  recién después se baja a la gruesa: medirla entre celdas gruesas "comía"
  hasta media celda de cada lado de una puerta, y con inflado 0.9 m el ancho
  mínimo de paso era ~2.4 m. Ahora es ≈ 2·(inflation_radius + resolution/2).
"""

import heapq
import math
from collections import deque
from dataclasses import dataclass, field

import numpy as np

SQRT2 = math.sqrt(2.0)
_MOVES = [(-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
          (-1, -1, SQRT2), (-1, 1, SQRT2), (1, -1, SQRT2), (1, 1, SQRT2)]


@dataclass
class PlanResult:
    path: list = field(default_factory=list)   # [(n, e), ...] incluyendo inicio y objetivo
    ok: bool = False
    reason: str = ''
    expansions: int = 0
    goal_adjusted: bool = False


def _dilate(mask):
    out = mask.copy()
    out[1:, :] |= mask[:-1, :]
    out[:-1, :] |= mask[1:, :]
    out[:, 1:] |= mask[:, :-1]
    out[:, :-1] |= mask[:, 1:]
    return out


def _dilate8(mask):
    out = _dilate(mask)
    out[1:, 1:] |= mask[:-1, :-1]
    out[1:, :-1] |= mask[:-1, 1:]
    out[:-1, 1:] |= mask[1:, :-1]
    out[:-1, :-1] |= mask[1:, 1:]
    return out


def distance_cells(occupied, max_cells):
    """Distancia aproximada (en celdas) a la celda ocupada más cercana.

    Dilataciones alternando vecindad 4 y 8: da un octógono, bastante parecido a
    la distancia euclídea y sin necesitar scipy.
    """
    dist = np.full(occupied.shape, np.inf)
    frontier = occupied.copy()
    dist[frontier] = 0.0
    for k in range(1, max_cells + 1):
        grown = _dilate8(frontier) if k % 2 else _dilate(frontier)
        new = grown & ~np.isfinite(dist)
        dist[new] = k
        frontier = grown
        if not np.any(new):
            break
    return dist


class GridPlanner:

    def __init__(self, resolution=0.3, inflation_radius=0.65, cost_radius=2.0,
                 proximity_weight=2.0, unknown_penalty=0.5, occupied_threshold=65,
                 max_expansions=60000, window_margin=12.0, goal_tolerance=1.5):
        self.resolution = float(resolution)
        self.inflation_radius = float(inflation_radius)
        self.cost_radius = max(float(cost_radius), self.inflation_radius)
        self.proximity_weight = float(proximity_weight)
        self.unknown_penalty = float(unknown_penalty)
        self.occupied_threshold = int(occupied_threshold)
        self.max_expansions = int(max_expansions)
        self.window_margin = float(window_margin)
        self.goal_tolerance = float(goal_tolerance)

    # ------------------------------------------------------------------
    def _window(self, snapshot, start, goal):
        """Recorte de la grilla alrededor de inicio y objetivo, a resolución gruesa.

        Lo que cae fuera de la grilla cuenta como desconocido: el objetivo puede
        estar en una zona que todavía nadie vio.
        """
        factor = max(1, int(round(self.resolution / snapshot.resolution)))
        res = snapshot.resolution * factor
        n0 = min(start[0], goal[0]) - self.window_margin
        e0 = min(start[1], goal[1]) - self.window_margin
        n1 = max(start[0], goal[0]) + self.window_margin
        e1 = max(start[1], goal[1]) + self.window_margin
        # Alinear al grillado fino para que las celdas gruesas coincidan con el mapa.
        r0 = int(math.floor((n0 - snapshot.origin_n) / snapshot.resolution))
        c0 = int(math.floor((e0 - snapshot.origin_e) / snapshot.resolution))
        rows = int(math.ceil((n1 - n0) / res)) * factor
        cols = int(math.ceil((e1 - e0) / res)) * factor
        fine = np.full((rows, cols), -1, dtype=np.int16)
        vr0, vc0 = max(0, r0), max(0, c0)
        vr1 = min(snapshot.values.shape[0], r0 + rows)
        vc1 = min(snapshot.values.shape[1], c0 + cols)
        if vr0 < vr1 and vc0 < vc1:
            fine[vr0 - r0:vr1 - r0, vc0 - c0:vc1 - c0] = snapshot.values[vr0:vr1, vc0:vc1]
        shape = (rows // factor, factor, cols // factor, factor)
        occupied = (fine >= self.occupied_threshold).reshape(shape).any(axis=(1, 3))
        unknown = (fine < 0).reshape(shape).all(axis=(1, 3))
        origin_n = snapshot.origin_n + r0 * snapshot.resolution
        origin_e = snapshot.origin_e + c0 * snapshot.resolution
        self._fine = (fine >= self.occupied_threshold, factor, snapshot.resolution)
        return occupied, unknown, origin_n, origin_e, res

    def _clearance(self, occupied, res):
        """Distancia (m) de cada celda gruesa al obstáculo más cercano, medida
        en la grilla fina: mínimo del bloque más media celda gruesa (≈ la
        distancia desde el centro de la celda)."""
        fine_occ, factor, fine_res = self._fine
        if factor <= 1:
            return distance_cells(occupied, int(math.ceil(self.cost_radius / res)) + 1) * res
        dist = distance_cells(fine_occ, int(math.ceil(self.cost_radius / fine_res)) + 2) * fine_res
        rows, cols = occupied.shape
        block = dist.reshape(rows, factor, cols, factor).min(axis=(1, 3))
        out = block + res / 2.0
        out[occupied] = 0.0
        return out

    def plan(self, snapshot, start_ne, goal_ne):
        start_ne = (float(start_ne[0]), float(start_ne[1]))
        goal_ne = (float(goal_ne[0]), float(goal_ne[1]))
        occupied, unknown, on, oe, res = self._window(snapshot, start_ne, goal_ne)
        rows, cols = occupied.shape

        infl_cells = int(math.ceil(self.inflation_radius / res))
        dist = self._clearance(occupied, res)
        blocked = dist < self.inflation_radius
        proximity = np.clip((self.cost_radius - dist) / max(self.cost_radius - self.inflation_radius, 1e-3),
                            0.0, 1.0) * self.proximity_weight
        proximity[~np.isfinite(dist)] = 0.0
        step_cost = 1.0 + proximity + np.where(unknown, self.unknown_penalty, 0.0)

        def cell(p):
            return (int(math.floor((p[0] - on) / res)), int(math.floor((p[1] - oe) / res)))

        def center(rc):
            return (on + (rc[0] + 0.5) * res, oe + (rc[1] + 0.5) * res)

        start = cell(start_ne)
        goal = cell(goal_ne)
        result = PlanResult()
        if not (0 <= start[0] < rows and 0 <= start[1] < cols):
            result.reason = 'inicio fuera de la ventana'
            return result

        prefix = []
        if blocked[start]:
            escape = self._nearest_free(blocked, occupied, start, max(infl_cells * 3, 4))
            if escape is None:
                result.reason = 'el dron está rodeado de obstáculos inflados'
                return result
            prefix = [start_ne]
            start = escape

        if blocked[goal]:
            moved = self._nearest_free(blocked, None, goal,
                                       int(math.ceil(self.goal_tolerance / res)))
            if moved is None:
                result.reason = 'el objetivo está dentro de un obstáculo'
                return result
            goal = moved
            result.goal_adjusted = True

        came_from, expansions = self._astar(blocked, step_cost, start, goal)
        result.expansions = expansions
        if goal not in came_from:
            result.reason = ('sin camino' if expansions < self.max_expansions
                             else f'búsqueda cortada a {self.max_expansions} expansiones')
            return result

        cells = []
        node = goal
        while node is not None:
            cells.append(node)
            node = came_from[node]
        cells.reverse()
        cells = self._shortcut(cells, blocked, unknown)
        path = prefix + [start_ne if not prefix else center(cells[0])]
        path += [center(rc) for rc in cells[1:-1]]
        path.append(goal_ne if not result.goal_adjusted else center(cells[-1]))
        result.path = path
        result.ok = True
        result.reason = 'ok'
        return result

    def _nearest_free(self, blocked, passable, origin, max_radius):
        """BFS a la celda no bloqueada más cercana (atravesando sólo `passable`)."""
        rows, cols = blocked.shape
        seen = {origin}
        queue = deque([(origin, 0)])
        while queue:
            (r, c), depth = queue.popleft()
            if not blocked[r, c]:
                return (r, c)
            if depth >= max_radius:
                continue
            for dr, dc, _ in _MOVES:
                nr, nc = r + dr, c + dc
                if 0 <= nr < rows and 0 <= nc < cols and (nr, nc) not in seen:
                    if passable is not None and passable[nr, nc]:
                        continue
                    seen.add((nr, nc))
                    queue.append(((nr, nc), depth + 1))
        return None

    def _astar(self, blocked, step_cost, start, goal):
        rows, cols = blocked.shape
        g = {start: 0.0}
        came_from = {start: None}
        heap = [(0.0, 0.0, start)]
        closed = set()
        expansions = 0
        gr, gc = goal
        while heap:
            _, cost, node = heapq.heappop(heap)
            if node in closed:
                continue
            if node == goal:
                break
            closed.add(node)
            expansions += 1
            if expansions >= self.max_expansions:
                break
            r, c = node
            for dr, dc, length in _MOVES:
                nr, nc = r + dr, c + dc
                if not (0 <= nr < rows and 0 <= nc < cols) or blocked[nr, nc]:
                    continue
                if dr and dc and (blocked[r + dr, c] or blocked[r, c + dc]):
                    continue    # no cortar esquinas en diagonal
                new_cost = cost + length * step_cost[nr, nc]
                if new_cost < g.get((nr, nc), math.inf):
                    g[(nr, nc)] = new_cost
                    came_from[(nr, nc)] = node
                    dr_h, dc_h = abs(nr - gr), abs(nc - gc)
                    h = (dr_h + dc_h) + (SQRT2 - 2.0) * min(dr_h, dc_h)
                    heapq.heappush(heap, (new_cost + h, new_cost, (nr, nc)))
        return came_from, expansions

    def _line_free(self, a, b, blocked, unknown):
        steps = int(max(abs(b[0] - a[0]), abs(b[1] - a[1]))) * 2 + 1
        rr = np.rint(np.linspace(a[0], b[0], steps)).astype(int)
        cc = np.rint(np.linspace(a[1], b[1], steps)).astype(int)
        # El atajo no puede cruzar obstáculos ni meterse más en lo desconocido
        # que el camino original (si no, se perdería el costo de lo desconocido).
        return not np.any(blocked[rr, cc]) and not np.any(unknown[rr, cc] & ~unknown[a] & ~unknown[b])

    def _shortcut(self, cells, blocked, unknown):
        if len(cells) <= 2:
            return cells
        out = [cells[0]]
        i = 0
        while i < len(cells) - 1:
            j = len(cells) - 1
            while j > i + 1 and not self._line_free(cells[i], cells[j], blocked, unknown):
                j -= 1
            out.append(cells[j])
            i = j
        return out


def path_lookahead(path, position_ne, lookahead):
    """Punto del camino a `lookahead` metros por delante de la proyección del dron."""
    if not path:
        return None
    if len(path) == 1:
        return path[0]
    pts = np.asarray(path, dtype=float)
    pos = np.asarray(position_ne, dtype=float)
    best_i, best_t, best_d = 0, 0.0, math.inf
    for i in range(len(pts) - 1):
        a, b = pts[i], pts[i + 1]
        ab = b - a
        denom = float(ab @ ab)
        t = 0.0 if denom < 1e-9 else min(1.0, max(0.0, float((pos - a) @ ab) / denom))
        d = float(np.hypot(*(a + t * ab - pos)))
        if d < best_d:
            best_i, best_t, best_d = i, t, d
    remaining = lookahead
    point = pts[best_i] + best_t * (pts[best_i + 1] - pts[best_i])
    i = best_i
    while i < len(pts) - 1:
        seg_end = pts[i + 1]
        seg = float(np.hypot(*(seg_end - point)))
        if seg >= remaining:
            direction = (seg_end - point) / max(seg, 1e-9)
            return tuple(point + direction * remaining)
        remaining -= seg
        point = seg_end
        i += 1
    return tuple(pts[-1])
