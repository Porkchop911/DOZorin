"""Cost-weighted eight-connected A* and collision-aware path simplification."""
from __future__ import annotations

import heapq
import math

import numpy as np


def astar(costmap: np.ndarray, start: tuple[int, int], goal: tuple[int, int],
          lethal: int = 200, cost_weight: float = 3.0,
          allow_trapped_start: bool = False) -> list[tuple[int, int]]:
    costs = np.asarray(costmap)
    h, w = costs.shape
    for index, point in enumerate((start, goal)):
        blocked_start = (allow_trapped_start
                         and (index == 0 or (index == 1 and goal == start)))
        if (not (0 <= point[0] < w and 0 <= point[1] < h)
                or (costs[point[1], point[0]] >= lethal
                    and not blocked_start)):
            return []
    queue = [(0.0, start)]
    came_from = {start: None}
    best = {start: 0.0}
    neighbors = ((1, 0), (-1, 0), (0, 1), (0, -1),
                 (1, 1), (1, -1), (-1, 1), (-1, -1))
    while queue:
        _, current = heapq.heappop(queue)
        if current == goal:
            break
        for dx, dy in neighbors:
            nxt = current[0] + dx, current[1] + dy
            if not (0 <= nxt[0] < w and 0 <= nxt[1] < h):
                continue
            cell_cost = int(costs[nxt[1], nxt[0]])
            current_cost = int(costs[current[1], current[0]])
            if current_cost >= lethal and allow_trapped_start:
                # Inflation commonly contains flat inscribed-cost plateaus.
                # Crossing an equal non-obstacle plateau is an outward-safe
                # escape as long as cost never rises.  A true 255 obstacle
                # cell remains stricter: it must descend immediately.
                if (cell_cost > current_cost
                        or (current_cost >= 255 and cell_cost >= current_cost)):
                    continue
            elif cell_cost >= lethal:
                continue
            step = math.hypot(dx, dy) * (1.0 + cost_weight * cell_cost / 255.0)
            candidate = best[current] + step
            if candidate < best.get(nxt, math.inf):
                best[nxt] = candidate
                came_from[nxt] = current
                heuristic = math.hypot(goal[0] - nxt[0], goal[1] - nxt[1])
                heapq.heappush(queue, (candidate + heuristic, nxt))
    if goal not in came_from:
        return []
    path, current = [], goal
    while current is not None:
        path.append(current)
        current = came_from[current]
    return path[::-1]


def _clear_line(costmap, start, end, lethal):
    distance = max(abs(end[0] - start[0]), abs(end[1] - start[1]))
    if distance == 0:
        return True
    xs = np.rint(np.linspace(start[0], end[0], distance + 1)).astype(int)
    ys = np.rint(np.linspace(start[1], end[1], distance + 1)).astype(int)
    return bool((costmap[ys, xs] < lethal).all())


def simplify_path(costmap: np.ndarray, path: list[tuple[int, int]],
                  lethal: int = 200) -> list[tuple[int, int]]:
    if len(path) <= 2:
        return path
    output, anchor = [path[0]], 0
    while anchor < len(path) - 1:
        candidate = len(path) - 1
        while candidate > anchor + 1 and not _clear_line(costmap, path[anchor],
                                                          path[candidate], lethal):
            candidate -= 1
        output.append(path[candidate])
        anchor = candidate
    return output
