import yaml
from pathlib import Path
import random

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

