import random
import yaml
from pathlib import Path
import pygame
import math
import heapq

def load_config(path: str) -> dict:
    # sensible defaults (used if yaml omits a field)
    cfg = {
        "seed": 42,
        "fps": 30,
        "grid": {"width": 15, "height": 15, "cell_px": 50},
        "mother": {"start": None},  # None = center
        "child": {"start": None},
        "food": {"positions": []},
        "threats": {"positions": []},
        "nest": {"position": None},
        "colors": {
            "bg": [248, 248, 248],
            "grid": [220, 220, 220],
            "mother": [235, 87, 87],
            "child": [87, 87, 235],
            "food": [76, 175, 80],
            "threat": [244, 67, 54],
            "nest": [255, 193, 7],
            "outline": [120, 120, 120],
        },
    }
    if path and Path(path).is_file():
        with open(path, "r", encoding="utf-8") as f:
            user = yaml.safe_load(f) or {}
        # Deep merge for nested dictionaries
        def deep_merge(base, update):
            for key, value in update.items():
                if key in base and isinstance(base[key], dict) and isinstance(value, dict):
                    deep_merge(base[key], value)
                else:
                    base[key] = value
        deep_merge(cfg, user)
    return cfg

# ===================================================================
# -------- Rendering --------
# ===================================================================

def draw_grid(surface, grid_w, grid_h, cell_px, bg, grid_color, outline):
    surface.fill(bg)
    pygame.draw.rect(surface, outline, (0, 0, grid_w * cell_px, grid_h * cell_px), width=2)
    for x in range(1, grid_w):
        pygame.draw.line(surface, grid_color, (x * cell_px, 0), (x * cell_px, grid_h * cell_px))
    for y in range(1, grid_h):
        pygame.draw.line(surface, grid_color, (0, y * cell_px), (grid_w * cell_px, y * cell_px))

def draw_mother(surface, mother, cell_px, mother_color, outline_color, label="M"):
    """Draw mother agent"""
    cx = mother.x * cell_px + cell_px // 2
    cy = mother.y * cell_px + cell_px // 2
    r = cell_px // 2.5
    # Draw mother as a circle
    pygame.draw.circle(surface, mother_color, (cx, cy), r)
    pygame.draw.circle(surface, outline_color, (cx, cy), r, width=2)

    # Label Mother
    font_size = int(r * 1.5)
    font = pygame.font.SysFont(None, font_size)
    text_surface = font.render(label, True, [255,255,255])
    text_rect = text_surface.get_rect(center=(cx, cy))
    surface.blit(text_surface, text_rect)
    
    # Child Carried
    if mother.child is not None and mother.child.is_carried:
        pygame.draw.circle(surface, (154, 205, 50), (cx, cy), r + 4, width= 4)
        # pygame.draw.circle(surface, outline_color, (cx, cy), r + 4, width=1)

    if mother.pick_food:
        pygame.draw.circle(surface, (0,0,0), (cx, cy), r + 3, width=4)

def draw_child(surface, child, cell_px, child_color, outline_color, label="C"):
    """Draw child agent"""

    if child.is_carried or not child.is_alive():
        return
    
    cx = child.x * cell_px + cell_px // 2
    cy = child.y * cell_px + cell_px // 2
    r = cell_px // 3.5  # Child is smaller than mother
    pygame.draw.circle(surface, child_color, (cx, cy), r)
    pygame.draw.circle(surface, outline_color, (cx, cy), r, width=2)

    # Draw perception range (dashed circle)
    # perception_r = r + 30
    # pygame.draw.circle(surface, child_color, (cx, cy), int(perception_r), width=1)

    # Label Child
    font_size = int(r * 1.5)
    font = pygame.font.SysFont(None, font_size)
    text_surface = font.render(label, True, [255,255,255])
    text_rect = text_surface.get_rect(center=(cx, cy))
    surface.blit(text_surface, text_rect)

def draw_food(surface, food, cell_px, food_color, outline_color):
    """Draw food entity"""
    if food.collected:
        return  # Don't draw collected food
    cx = food.x * cell_px + cell_px // 2
    cy = food.y * cell_px + cell_px // 2
    # Draw food as a small square
    size = cell_px // 2
    rect = pygame.Rect(cx - size // 2, cy - size // 2, size, size)
    pygame.draw.rect(surface, food_color, rect)
    pygame.draw.rect(surface, outline_color, rect, width=1)

def draw_threat(surface, threat, cell_px, threat_color, outline_color, perception_range):
    """Draw threat entity"""
    cx = threat.x * cell_px + cell_px // 2
    cy = threat.y * cell_px + cell_px // 2
    # Draw threat as a triangle (warning symbol)
    r = cell_px // 3
    points = [
        (cx, cy - r),  # Top
        (cx - r, cy + r),  # Bottom left
        (cx + r, cy + r)   # Bottom right
    ]
    pygame.draw.polygon(surface, threat_color, points)
    pygame.draw.polygon(surface, outline_color, points, width=2)

    # # Draw perception range (dashed circle)
    perception_r = r + perception_range
    pygame.draw.circle(surface, outline_color, (cx, cy), int(perception_r), width=1)

# def draw_nest(surface, nest, cell_px, nest_color, outline_color):
#     """Draw nest entity"""
#     # Draw nest as a 3x3 square
#     pygame.draw.rect(surface, nest_color, ((nest.x-1) * cell_px, (nest.y-1) * cell_px, cell_px*3, cell_px*3))
#     cx = nest.x * cell_px + cell_px // 2
#     cy = nest.y * cell_px + cell_px // 2    
#     r = cell_px 
#     # Draw inner circle for nest pattern
#     inner_r = r 
#     pygame.draw.circle(surface, outline_color, (cx, cy), inner_r, width=1)

def intensity_to_color(value, vmin=0, vmax=100):
    value = max(vmin, min(vmax, value))
    t = (value - vmin) / (vmax - vmin)
    if t < 0.5:
        # Yellow -> Green
        r = int(255 * (1 - t / 0.5))
        g = 255
        b = 0
    else:
        # Green -> Blue
        r = 0
        g = int(255 * (1 - (t - 0.5) / 0.5))
        b = int(255 * ((t - 0.5) / 0.5))
    return (r, g, b)


# ===================================================================



# ===================================================================
# Initialize
# ===================================================================

def random_unique_positions(n, grid_w, grid_h, occupied=None):
    """
    Generate n unique positions on a grid, avoiding occupied cells.
    """
    if occupied is None:
        occupied = set()

    all_cells = [(x, y) for x in range(grid_w) for y in range(grid_h)]
    free_cells = list(set(all_cells) - occupied)

    if n > len(free_cells):
        raise ValueError("Not enough free cells to place all entities.")

    chosen = random.sample(free_cells, n)
    occupied.update(chosen)

    return [[x, y] for x, y in chosen], occupied



# ===================================================================
# Path planning
# ===================================================================

def octile_heuristic(a, b):
    (x1, y1), (x2, y2) = a, b
    dx, dy = abs(x1 - x2), abs(y1 - y2)
    # diagonal cost ~ sqrt(2), straight cost 1
    return (dx + dy) + (math.sqrt(2) - 2) * min(dx, dy)

def astar(start, goal, grid_w, grid_h, blocked, moves_8=True):
    if start == goal:
        return [start]

    if moves_8:
        nbrs = [(1,0),(-1,0),(0,1),(0,-1),(1,1),(1,-1),(-1,1),(-1,-1)]
        def step_cost(dx, dy):  # diagonal costs more
            return math.sqrt(2) if dx != 0 and dy != 0 else 1.0
        h = octile_heuristic
    else:
        nbrs = [(1,0),(-1,0),(0,1),(0,-1)]
        def step_cost(dx, dy):
            return 1.0
        h = lambda a, b: abs(a[0]-b[0]) + abs(a[1]-b[1])

    open_heap = []
    heapq.heappush(open_heap, (h(start, goal), 0.0, start))
    came_from = {}
    gscore = {start: 0.0}
    closed = set()

    while open_heap:
        _, g, cur = heapq.heappop(open_heap)
        if cur in closed:
            continue
        closed.add(cur)

        if cur == goal:
            # reconstruct
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

            # blocked cells are not allowed EXCEPT goal
            if nxt in blocked and nxt != goal:
                continue

            ng = g + step_cost(dx, dy)
            if ng < gscore.get(nxt, float("inf")):
                gscore[nxt] = ng
                came_from[nxt] = cur
                f = ng + h(nxt, goal)
                heapq.heappush(open_heap, (f, ng, nxt))

    return None  # no path

