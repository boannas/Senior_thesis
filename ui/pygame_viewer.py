
import pygame
import sys
from pathlib import Path
import random
import numpy as np

# Add parent directory to path to import modules
sys.path.insert(0, str(Path(__file__).parent.parent))

from core.world import World
from func.function_code import (
    load_config, draw_grid, draw_mother, draw_child, 
    draw_food, draw_threat, intensity_to_color, random_unique_positions
)


def main(threat_positions=None):

    # Load configuration
    config_path = Path(__file__).parent.parent / "base.yaml"
    cfg = load_config(str(config_path))
    
    # Extract configuration
    grid_w = cfg["grid"]["width"]
    grid_h = cfg["grid"]["height"]
    cell_px = cfg["grid"]["cell_px"]
    fps = cfg["simulation"]["fps"]
    dt = cfg["simulation"]["dt"]
    seed = cfg["simulation"]["seed"]

    random.seed(seed)   # Fixed seed for reproducibility
    occupied = set()

    # mother_starts = cfg['mothers']['starts']
    mother_starts, occupied = random_unique_positions(
        n=2, grid_w=grid_w, grid_h=grid_h, occupied=occupied
    )

    # child_starts = cfg['children']['starts']
    child_starts, occupied = random_unique_positions(
        n=2, grid_w=grid_w, grid_h=grid_h, occupied=occupied
    )

    threat_starts, occupied = random_unique_positions(
        n=0, grid_w=grid_w, grid_h=grid_h, occupied=occupied
    )
    # Entity positions
    # food_positions = cfg["food"].get("positions", [])
    food_positions, occupied = random_unique_positions(
        n=5, grid_w=grid_w, grid_h=grid_h, occupied=occupied
    )

    # print("food_positions:", food_positions)
    # Colors
    bg_color = tuple(cfg["colors"]["bg"])
    grid_color = tuple(cfg["colors"]["grid"])
    mother_color = tuple(cfg["colors"]["mother"])
    threat_colot = tuple(cfg["colors"]["threat"])
    food_color = tuple(cfg["colors"]["food"])
    outline_color = tuple(cfg["colors"]["outline"])
    
    # Initialize pygame
    pygame.init()
    
    # Calculate window size
    window_w = grid_w * cell_px
    window_h = grid_h * cell_px
    screen = pygame.display.set_mode((window_w, window_h))
    pygame.display.set_caption("Grid World: Mother, Child, Food, Threat, and Nest")
    clock = pygame.time.Clock()
    
    # Create world with all entities
    world = World(
        grid_w, grid_h,
        mother_starts=mother_starts,
        child_start=child_starts,
        food_positions=food_positions,
        threat_starts=threat_starts,
    )
    
    # Main game loop
    running = True
    accumulator = 0.0

    # ==============================================
    # ------------- Main Update Loop ---------------
    # ==============================================
    while running:
        frame_time = clock.tick(fps) / 1000.0  # seconds
        accumulator += frame_time
        
        # Handle events
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    running = False
        
        while accumulator >= dt:
            accumulator -= dt
            world.step(dt)

        # ==============================================
        # ------------- Rendering ----------------
        # ==============================================

        # Render in order (background to foreground)
        draw_grid(screen, grid_w, grid_h, cell_px, bg_color, grid_color, outline_color)
      
        for food in world.foods:
            draw_food(screen, food, cell_px, food_color, outline_color)
        
        # Draw mother (on top)
        for i, m in enumerate(world.mothers):
            draw_mother(screen, m, cell_px, mother_color, outline_color, label=f"M{i}")

        # Draw child (on top)
        for i, c in enumerate(world.children):
            # c.distress += np.random.uniform(1, 3)           # dynamic distress testing
            color = intensity_to_color(c.energy)
            draw_child(screen, c, cell_px, color, outline_color, label=f"C{i}")

        # Draw threat
        for i, t in enumerate(world.threats):
            percept_range = cell_px * 1
            draw_threat(screen, t, cell_px, threat_colot, outline_color, perception_range=percept_range)
        # Update display
        pygame.display.flip()
        clock.tick(fps)
    
    pygame.quit()
    sys.exit()


if __name__ == "__main__":
    main()

