
import pygame
import sys
from pathlib import Path
import random

# Add parent directory to path to import modules
sys.path.insert(0, str(Path(__file__).parent.parent))

from core.world import World
from func.function_code import (
    load_config, draw_grid, draw_mother, draw_child, 
    draw_food, draw_threat, draw_nest
)


def main(threat_positions=None):

    # Load configuration
    config_path = Path(__file__).parent.parent / "base.yaml"
    cfg = load_config(str(config_path))
    
    # Extract configuration
    grid_w = cfg["grid"]["width"]
    grid_h = cfg["grid"]["height"]
    cell_px = cfg["grid"]["cell_px"]
    fps = cfg["fps"]
    seed = cfg["seed"]
    dt = cfg["simulation"]["dt"]

    # Agent positions
    mother_starts = cfg["mothers"]["starts"]
        
    # Entity positions
    food_positions = cfg["food"].get("positions", [])
    
    # Colors
    bg_color = tuple(cfg["colors"]["bg"])
    grid_color = tuple(cfg["colors"]["grid"])
    mother_color = tuple(cfg["colors"]["mother"])
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
        food_positions=food_positions,
        threat_positions=threat_positions,
        seed=seed
    )
    
    # Main game loop
    running = True
    accumulator = 0.0

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

        # Render in order (background to foreground)
        draw_grid(screen, grid_w, grid_h, cell_px, bg_color, grid_color, outline_color)
      
        for food in world.foods:
            draw_food(screen, food, cell_px, food_color, outline_color)
        
        # Draw mother (on top)
        for m in world.mothers:
            draw_mother(screen, m, cell_px, mother_color, outline_color)

        # Update display
        pygame.display.flip()
        clock.tick(fps)
    
    pygame.quit()
    sys.exit()


if __name__ == "__main__":
    main()

