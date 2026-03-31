"""
Main Pygame application — initializes the world, runs the simulation
loop, renders the grid, and manages logging/plotting.
"""

import pygame
import sys
import os
from pathlib import Path

# Add project root to path
sys.path.insert(0, str(Path(__file__).parent.parent.parent))

from core.seed import init_seed
from core.world import World
from core.ui.pygame_render import (
    draw_grid, draw_mother, draw_child,
    draw_food, draw_threat, intensity_to_color,
    draw_clock_text,
)
from core.config.config import load_config, random_unique_positions
from func.live_plot import MotherStatePlotter, MotherMotivationPlotter, ChildStatePlotter, FixedVsPlasticPlotter
from func.run_logger import RunLogger

# ─── Run Configuration ─────────────────────────────────────────────
# Logging mode: "csv" (only CSV), "plot" (only realtime plots), "both", or None
LOG_RUN_MODE = "csv"
LOG_CSV_PATH = "run_log.csv"
SHOW_PLOTS = LOG_RUN_MODE != "csv"


def main():
    """Initialize and run the maternal instinct grid-world simulation."""

    # --- Load configuration ---
    config_path = Path(__file__).parent.parent / "base.yaml"
    cfg = load_config(str(config_path))

    grid_w = cfg["grid"]["width"]
    grid_h = cfg["grid"]["height"]
    cell_px = cfg["grid"]["cell_px"]
    fps = cfg["simulation"]["fps"]
    dt = cfg["simulation"]["dt"]
    seed = cfg["simulation"]["seed"]
    day_step = cfg["days"]["ticks_per_day"]

    # --- Initialize random seeds (once, globally) ---
    init_seed(seed)

    # --- Generate random starting positions ---
    occupied = set()

    mother_starts, occupied = random_unique_positions(
        n=1, grid_w=grid_w, grid_h=grid_h, occupied=occupied
    )
    child_starts, occupied = random_unique_positions(
        n=1, grid_w=grid_w, grid_h=grid_h, occupied=occupied
    )
    threat_starts, occupied = random_unique_positions(
        n=0, grid_w=grid_w, grid_h=grid_h, occupied=occupied
    )
    food_positions, occupied = random_unique_positions(
        n=0, grid_w=grid_w, grid_h=grid_h, occupied=occupied
    )

    # --- Colors from config ---
    bg_color = tuple(cfg["colors"]["bg"])
    grid_color = tuple(cfg["colors"]["grid"])
    mother_color = tuple(cfg["colors"]["mother"])
    threat_color = tuple(cfg["colors"]["threat"])
    food_color = tuple(cfg["colors"]["food"])
    outline_color = tuple(cfg["colors"]["outline"])

    # --- Initialize Pygame ---
    os.environ["SDL_VIDEO_WINDOW_POS"] = "900,630"
    pygame.init()

    window_w = grid_w * cell_px
    window_h = grid_h * cell_px
    screen = pygame.display.set_mode((window_w, window_h))
    pygame.display.set_caption("Maternal Instinct Simulation")
    clock = pygame.time.Clock()

    # --- Create world ---
    world = World(
        grid_w, grid_h,
        mother_starts=mother_starts,
        child_start=child_starts,
        food_positions=food_positions,
        threat_starts=threat_starts,
        day_step=day_step,
    )

    # --- Initialize plotters and logger ---
    if SHOW_PLOTS:
        plotter = MotherStatePlotter(world)
        child_plotter = ChildStatePlotter(world)
        weight_plotter = FixedVsPlasticPlotter(world)
    else:
        plotter = child_plotter = weight_plotter = None

    run_logger = RunLogger(
        world,
        mode=LOG_RUN_MODE or "csv",
        csv_path=LOG_CSV_PATH,
    ) if LOG_RUN_MODE else None

    # ═══════════════════════════════════════════════════════════════
    # Main Simulation Loop
    # ═══════════════════════════════════════════════════════════════
    running = True
    accumulator = 0.0

    while running:
        frame_time = clock.tick(fps) / 1000.0
        accumulator += frame_time

        # --- Handle events ---
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    running = False

        # --- Fixed-timestep simulation updates ---
        while accumulator >= dt:
            accumulator -= dt
            world.step(dt)

            if SHOW_PLOTS and world.tick % 1 == 0:
                plotter.update()
                child_plotter.update()
                weight_plotter.update()

            if run_logger:
                run_logger.update()

        # --- Rendering ---
        draw_grid(screen, grid_w, grid_h, cell_px, bg_color, grid_color, outline_color)

        for food in world.foods:
            draw_food(screen, food, cell_px, food_color, outline_color)

        for i, mother in enumerate(world.mothers):
            draw_mother(screen, mother, cell_px, mother_color, outline_color, label=f"M{i}")

        for i, child in enumerate(world.children):
            color = intensity_to_color(child.energy)
            draw_child(screen, child, cell_px, color, outline_color, label=f"C{i}")

        for i, threat in enumerate(world.threats):
            perception_px = cell_px * threat.perception_range
            draw_threat(screen, threat, cell_px, threat_color, outline_color, perception_range=perception_px)

        draw_clock_text(screen, world)
        pygame.display.flip()
        clock.tick(fps)

    # --- Cleanup ---
    if run_logger:
        run_logger.close()
    pygame.quit()
    sys.exit()


if __name__ == "__main__":
    main()
