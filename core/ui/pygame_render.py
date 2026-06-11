"""
Pygame rendering functions for the grid world simulation.
Draws the grid, agents (mother, child, threat), food, and HUD clock.
"""

import pygame


def draw_grid(surface, grid_w, grid_h, cell_px, bg_color, grid_color, outline_color):
    """Draw the grid background with lines and border."""
    surface.fill(bg_color)
    pygame.draw.rect(surface, outline_color, (0, 0, grid_w * cell_px, grid_h * cell_px), width=2)

    for x in range(1, grid_w):
        pygame.draw.line(surface, grid_color, (x * cell_px, 0), (x * cell_px, grid_h * cell_px))
    for y in range(1, grid_h):
        pygame.draw.line(surface, grid_color, (0, y * cell_px), (grid_w * cell_px, y * cell_px))


def draw_mother(surface, mother, cell_px, mother_color, outline_color, label="M"):
    """Draw mother agent as a circle with optional child/food indicators."""
    if not mother.is_alive():
        return

    center_x = mother.x * cell_px + cell_px // 2
    center_y = mother.y * cell_px + cell_px // 2
    radius = cell_px // 3

    # Mother body (circle)
    pygame.draw.circle(surface, mother_color, (center_x, center_y), radius)
    pygame.draw.circle(surface, outline_color, (center_x, center_y), radius, width=2)

    # Label
    font_size = int(radius * 1.5)
    font = pygame.font.SysFont(None, font_size)
    text_surface = font.render(label, True, (255, 255, 255))
    text_rect = text_surface.get_rect(center=(center_x, center_y))
    surface.blit(text_surface, text_rect)

    # Child carried indicator (blue ring)
    if mother.child is not None and mother.child.is_carried and mother.child.alive:
        pygame.draw.circle(surface, (0, 170, 230), (center_x, center_y), radius + 4, width=2)

    # Food inventory indicator (orange ring)
    if mother.food_inventory > 0:
        pygame.draw.circle(surface, (230, 140, 40), (center_x, center_y), radius + 8, width=2)


def draw_child(surface, child, cell_px, child_color, outline_color, label="C"):
    """Draw child agent as a smaller circle (hidden when carried or dead)."""
    if child.is_carried or not child.is_alive():
        return

    center_x = child.x * cell_px + cell_px // 2
    center_y = child.y * cell_px + cell_px // 2
    radius = cell_px // 3.5

    pygame.draw.circle(surface, child_color, (center_x, center_y), radius)
    pygame.draw.circle(surface, outline_color, (center_x, center_y), radius, width=2)

    # Label
    font_size = int(radius * 1.5)
    font = pygame.font.SysFont(None, font_size)
    text_surface = font.render(label, True, (0, 0, 0))
    text_rect = text_surface.get_rect(center=(center_x, center_y))
    surface.blit(text_surface, text_rect)


def draw_food(surface, food, cell_px, food_color, outline_color):
    """Draw food entity as a small square."""
    if food.collected:
        return

    center_x = food.x * cell_px + cell_px // 2
    center_y = food.y * cell_px + cell_px // 2
    size = cell_px // 2
    rect = pygame.Rect(center_x - size // 2, center_y - size // 2, size, size)

    pygame.draw.rect(surface, food_color, rect)
    pygame.draw.rect(surface, outline_color, rect, width=1)


def draw_threat(surface, threat, cell_px, threat_color, outline_color, perception_range):
    """Draw threat agent as a triangle with perception range circle."""
    center_x = threat.x * cell_px + cell_px // 2
    center_y = threat.y * cell_px + cell_px // 2
    radius = cell_px // 3

    # Threat body (triangle)
    points = [
        (center_x, center_y - radius),           # Top
        (center_x - radius, center_y + radius),   # Bottom left
        (center_x + radius, center_y + radius),   # Bottom right
    ]
    pygame.draw.polygon(surface, threat_color, points)
    pygame.draw.polygon(surface, outline_color, points, width=2)

    # Perception range circle
    perception_radius = radius + perception_range
    pygame.draw.circle(surface, outline_color, (center_x, center_y), int(perception_radius), width=1)

    # Patrol goal marker (cross)
    if getattr(threat, 'patrol_goal', None) is not None:
        goal_x, goal_y = threat.patrol_goal
        marker_x = goal_x * cell_px + cell_px // 2
        marker_y = goal_y * cell_px + cell_px // 2
        marker_size = cell_px // 4

        pygame.draw.line(surface, outline_color, (marker_x - marker_size, marker_y), (marker_x + marker_size, marker_y), 2)
        pygame.draw.line(surface, outline_color, (marker_x, marker_y - marker_size), (marker_x, marker_y + marker_size), 2)


def intensity_to_color(value, vmin=0, vmax=100):
    """Map a 0–100 value to a Yellow → Green → Blue color gradient."""
    value = max(vmin, min(vmax, value))
    t = (value - vmin) / (vmax - vmin)

    if t < 0.5:
        # Yellow → Green
        r = int(255 * (1 - t / 0.5))
        g = 255
        b = 0
    else:
        # Green → Blue
        r = 0
        g = int(255 * (1 - (t - 0.5) / 0.5))
        b = int(255 * ((t - 0.5) / 0.5))

    return (r, g, b)


def get_clock_time(time_of_day, day_length):
    """Convert simulation tick-of-day to hours and minutes."""
    total_minutes = (time_of_day / day_length) * 24 * 60
    hours = int(total_minutes // 60) % 24
    minutes = int(total_minutes % 60)
    return hours, minutes


def draw_clock_text(screen, world):
    """Draw the day/time HUD text in the top-left corner."""
    font = pygame.font.SysFont(None, 14)
    hours, minutes = get_clock_time(world.time_of_day, world.day_step)
    phase = "Day" if world.is_day else "Night"
    text = f"Day {world.day_count} | {hours:02d}:{minutes:02d} | {phase}"
    surface = font.render(text, True, (0, 0, 255))
    screen.blit(surface, (10, 10))
