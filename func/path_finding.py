import math
import heapq

def octile_heuristic(a, b):
    (x1, y1), (x2, y2) = a, b
    dx, dy = abs(x1 - x2), abs(y1 - y2)
    return (dx + dy) + (math.sqrt(2) - 2) * min(dx, dy)

def astar(start, goal, grid_w, grid_h, blocked, moves_8=True):
    if start == goal:
        return [start]

    if moves_8:
        nbrs = [(1,0),(-1,0),(0,1),(0,-1),(1,1),(1,-1),(-1,1),(-1,-1)]
        def step_cost(dx, dy):
            return math.sqrt(2) if dx != 0 and dy != 0 else 1.0
        h = octile_heuristic
    else:
        nbrs = [(1,0),(-1,0),(0,1),(0,-1)]
        def step_cost(dx, dy):
            return 1.0
        h = lambda a, b: abs(a[0]-b[0]) + abs(a[1]-b[1])

    open_heap = [(h(start, goal), 0.0, start)]
    came_from = {}
    gscore = {start: 0.0}
    closed = set()

    while open_heap:
        _, g, cur = heapq.heappop(open_heap)
        if cur in closed:
            continue
        closed.add(cur)

        if cur == goal:
            path = [cur]
            while cur in came_from:
                cur = came_from[cur]
                path.append(cur)
            path.reverse()
            return path

        cx, cy = cur
        for dx, dy in nbrs:
            nx, ny = cx + dx, cy + dy
            if not (0 <= nx < grid_w and 0 <= ny < grid_h):
                continue

            nxt = (nx, ny)
            if nxt in blocked and nxt != goal:
                continue

            ng = g + step_cost(dx, dy)
            if ng < gscore.get(nxt, float("inf")):
                gscore[nxt] = ng
                came_from[nxt] = cur
                heapq.heappush(open_heap, (ng + h(nxt, goal), ng, nxt))

    return None