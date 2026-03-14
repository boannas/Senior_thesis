import pygame
import sys
from pathlib import Path
import random
import os 

# Add parent directory to path to import modules
sys.path.insert(0, str(Path(__file__).parent.parent))

from core.world import World
from core.ui.pygame_render import (
    draw_grid, draw_mother, draw_child, 
    draw_food, draw_threat, intensity_to_color,
    draw_clock_text
)
from core.config.config import load_config, random_unique_positions
from func.live_plot import MotherStatePlotter, MotherMotivationPlotter, ChildStatePlotter
import random
import numpy as np

random.seed(42)
np.random.seed(42)

def main():

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
    day_step = cfg["days"]["ticks_per_day"]


    random.seed(seed)   # Fixed seed for reproducibility
    occupied = set()

    # mother_starts = cfg['mothers']['starts']
    mother_starts, occupied = random_unique_positions(
        n=1, grid_w=grid_w, grid_h=grid_h, occupied=occupied
    )

    # child_starts = cfg['children']['starts']
    child_starts, occupied = random_unique_positions(
        n=1, grid_w=grid_w, grid_h=grid_h, occupied=occupied
    )

    threat_starts, occupied = random_unique_positions(
        n=1 , grid_w=grid_w, grid_h=grid_h, occupied=occupied
    )
    # Entity positions
    # food_positions = cfg["food"].get("positions", [])
    food_positions, occupied = random_unique_positions(
        n=0, grid_w=grid_w, grid_h=grid_h, occupied=occupied
    )

    # Colors
    bg_color = tuple(cfg["colors"]["bg"])
    grid_color = tuple(cfg["colors"]["grid"])
    mother_color = tuple(cfg["colors"]["mother"])
    threat_colot = tuple(cfg["colors"]["threat"])
    food_color = tuple(cfg["colors"]["food"])
    outline_color = tuple(cfg["colors"]["outline"])
    
    # Initialize pygame
    os.environ["SDL_VIDEO_WINDOW_POS"] = "900,630"
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
        day_step=day_step
    )
    plotter = MotherStatePlotter(world)
    # mot_plotter = MotherMotivationPlotter(world)
    child_plotter = ChildStatePlotter(world)
    

    # Main game loop
    running = True
    accumulator = 0.0
    time = 0.0


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
            if world.tick % 1 == 0:
                plotter.update()
                # mot_plotter.update()
                child_plotter.update()

        
        # ==============================================
        # ------------- Rendering ----------------
        # ==============================================

        # Render in order (background to foreground)
        draw_grid(screen, grid_w, grid_h, cell_px, bg_color, grid_color, outline_color)
      
        for food in world.foods:
            draw_food(screen, food, cell_px, food_color, outline_color)
        
        # Draw mother 
        for i, m in enumerate(world.mothers):
            draw_mother(screen, m, cell_px, mother_color, outline_color, label=f"M{i}")

        # Draw child 
        for i, c in enumerate(world.children):
            color = intensity_to_color(c.energy)
            draw_child(screen, c, cell_px, color, outline_color, label=f"C{i}")

        # Draw threat
        for i, t in enumerate(world.threats):
            percept_range = cell_px * t.perception_range
            draw_threat(screen, t, cell_px, threat_colot, outline_color, perception_range=percept_range)
        draw_clock_text(screen, world)

        # Update display
        pygame.display.flip()
        clock.tick(fps)
    
    pygame.quit()
    sys.exit()


if __name__ == "__main__":
    main()

