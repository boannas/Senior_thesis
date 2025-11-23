import yaml
from pathlib import Path
import pygame
import math

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


# -------- Rendering --------
def draw_grid(surface, grid_w, grid_h, cell_px, bg, grid_color, outline):
    surface.fill(bg)
    pygame.draw.rect(surface, outline, (0, 0, grid_w * cell_px, grid_h * cell_px), width=2)
    for x in range(1, grid_w):
        pygame.draw.line(surface, grid_color, (x * cell_px, 0), (x * cell_px, grid_h * cell_px))
    for y in range(1, grid_h):
        pygame.draw.line(surface, grid_color, (0, y * cell_px), (grid_w * cell_px, y * cell_px))

def draw_agent(surface, agent, cell_px, agent_color, outline):
    """Draw a generic agent (legacy function for backward compatibility)"""
    cx = agent.x * cell_px + cell_px // 2
    cy = agent.y * cell_px + cell_px // 2
    r = cell_px // 3
    pygame.draw.circle(surface, agent_color, (cx, cy), r)
    pygame.draw.circle(surface, outline, (cx, cy), r, width=2)

def draw_mother(surface, mother, cell_px, mother_color, outline_color, perception_r=150):
    """Draw mother agent"""
    cx = mother.x * cell_px + cell_px // 2
    cy = mother.y * cell_px + cell_px // 2
    r = cell_px // 3
    # Draw mother as a circle
    pygame.draw.circle(surface, mother_color, (cx, cy), r)
    pygame.draw.circle(surface, outline_color, (cx, cy), r, width=2)
    # Draw indicator if carrying child or food
    if mother.has_child:
        # Small circle on top for child indicator
        pygame.draw.circle(surface, (255, 255, 0), (cx, cy - r - 3), 4)
    if mother.has_food:
        # Small circle on bottom for food indicator
        pygame.draw.circle(surface, (0, 255, 0), (cx, cy + r + 3), 4)

    # Draw perception range (dashed circle)
    pygame.draw.circle(surface, mother_color, (cx, cy), int(perception_r), width=1)
    

def draw_child(surface, child, cell_px, child_color, outline_color):
    """Draw child agent"""
    if child.is_carried:
        # Don't draw if being carried (mother indicator shows it)
        return
    cx = child.x * cell_px + cell_px // 2
    cy = child.y * cell_px + cell_px // 2
    r = cell_px // 4  # Child is smaller than mother
    pygame.draw.circle(surface, child_color, (cx, cy), r)
    pygame.draw.circle(surface, outline_color, (cx, cy), r, width=2)
    # Draw perception range (dashed circle)
    perception_r = r + 30
    pygame.draw.circle(surface, child_color, (cx, cy), int(perception_r), width=1)

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

def draw_threat(surface, threat, cell_px, threat_color, outline_color):
    """Draw threat entity"""
    if not threat.active:
        return
    cx = threat.x * cell_px + cell_px // 2
    cy = threat.y * cell_px + cell_px // 2
    # Draw threat as a triangle (warning symbol)
    r = cell_px // 2.5
    points = [
        (cx, cy - r),  # Top
        (cx - r, cy + r),  # Bottom left
        (cx + r, cy + r)   # Bottom right
    ]
    pygame.draw.polygon(surface, threat_color, points)
    pygame.draw.polygon(surface, outline_color, points, width=2)

    # Draw perception range (dashed circle)
    perception_r = r + 40
    pygame.draw.circle(surface, outline_color, (cx, cy), int(perception_r), width=1, )

def draw_nest(surface, nest, cell_px, nest_color, outline_color):
    """Draw nest entity"""
    # Draw nest as a 3x3 square
    pygame.draw.rect(surface, nest_color, ((nest.x-1) * cell_px, (nest.y-1) * cell_px, cell_px*3, cell_px*3))
    cx = nest.x * cell_px + cell_px // 2
    cy = nest.y * cell_px + cell_px // 2    
    r = cell_px 
    # Draw inner circle for nest pattern
    inner_r = r 
    pygame.draw.circle(surface, outline_color, (cx, cy), inner_r, width=1)