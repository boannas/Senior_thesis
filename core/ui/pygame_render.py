import pygame

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
    r = cell_px // 3
    # Draw mother as a circle
    pygame.draw.circle(surface, mother_color, (cx, cy), r)
    pygame.draw.circle(surface, outline_color, (cx, cy), r, width=2)

    # Label Mother
    font_size = int(r * 1.5)
    font = pygame.font.SysFont(None, font_size)
    text_surface = font.render(label, True, [255, 255, 255])
    text_rect = text_surface.get_rect(center=(cx, cy))
    surface.blit(text_surface, text_rect)
    
    # Child Indicator
    if mother.child is not None and mother.child.is_carried:
        pygame.draw.circle(
            surface, (0, 170, 230), (cx, cy), r + 4, width=2
        )

    # Food Indicator
    if mother.holding_food:
        pygame.draw.circle(
            surface, (230, 140, 40), (cx, cy), r + 8, width=2
        )

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

    # Draw threat as a triangle
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

    # if getattr(threat, 'patrol_goal', None) is not None:
    #     gx, gy = threat.patrol_goal

    #     tx = gx * cell_px + cell_px // 2
    #     ty = gy * cell_px + cell_px // 2

    #     size = cell_px // 4

    #     # draw cross marker
    #     pygame.draw.line(surface, outline_color, (tx-size, ty), (tx+size, ty), 2)
    #     pygame.draw.line(surface, outline_color, (tx, ty-size), (tx, ty+size), 2)

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

