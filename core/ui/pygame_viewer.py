import pygame
import sys
import yaml
from pathlib import Path
import random
import os 

# Add parent directory to path to import modules
sys.path.insert(0, str(Path(__file__).parent.parent.parent))

from core.world import World
from core.ui.pygame_render import (
    draw_grid, draw_mother, draw_child, 
    draw_food, draw_threat, intensity_to_color,
    draw_clock_text
)
from core.config.config import load_config, random_unique_positions
from func.live_plot import MotherStatePlotter, MotherMotivationPlotter, ChildStatePlotter, FixedVsPlasticPlotter
from func.run_logger import RunLogger
import random
import numpy as np

random.seed(42)
np.random.seed(42)

# --- Visualization mode: use experiment scenario and fixed weight? ---
# Set True to load func/experiment_config.yaml for n_mothers, n_children, n_threats, n_food_initial
USE_EXPERIMENT_SCENARIO = False
# Set True for fixed weights (no plasticity); False to use plasticity_rule from config or default
VISUALIZE_FIXED_WEIGHT = True

# Run logging: "csv" (only CSV), "plot" (only realtime plots), "both"
LOG_RUN_MODE = "csv"
LOG_RUN_CSV_PATH = "hebbian.csv"
# When in CSV-only mode, no matplotlib windows (simulation only)
SHOW_PLOTS = LOG_RUN_MODE != "csv"

FUNC_DIR = Path(__file__).resolve().parent.parent.parent / "func"
EXPERIMENT_CONFIG_PATH = FUNC_DIR / "experiment_config.yaml"


def _load_experiment_scenario():
    """Load scenario from func/experiment_config.yaml. Return dict or None if not used."""
    if not USE_EXPERIMENT_SCENARIO or not EXPERIMENT_CONFIG_PATH.is_file():
        return None
    with open(EXPERIMENT_CONFIG_PATH, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}
    return {
        "n_mothers": data.get("n_mothers", 1),
        "n_children": data.get("n_children", 1),
        "n_threats": data.get("n_threats", 1),
        "n_food_initial": data.get("n_food_initial", 0),
        "food_spawn_interval": data.get("food_spawn_interval"),
        "food_spawn_n": data.get("food_spawn_n", 1),
        "scenario_name": data.get("scenario_name", "baseline"),
    }


def main():
    # Load configuration (base.yaml in project root or core/)
    repo_root = Path(__file__).resolve().parent.parent.parent
    config_path = repo_root / "core" / "base.yaml"
    if not config_path.is_file():
        config_path = repo_root / "base.yaml"
    cfg = load_config(str(config_path))
    grid_w = cfg["grid"]["width"]
    grid_h = cfg["grid"]["height"] 
    cell_px = cfg["grid"]["cell_px"]
    fps = cfg["simulation"]["fps"]
    dt = cfg["simulation"]["dt"]
    seed = cfg["simulation"]["seed"]
    day_step = cfg["days"]["ticks_per_day"]

    # Scenario: from experiment config or defaults
    scenario = _load_experiment_scenario()
    if scenario:
        n_mothers = int(scenario["n_mothers"])
        n_children = int(scenario["n_children"])
        n_threats = int(scenario["n_threats"])
        n_food_initial = int(scenario["n_food_initial"])
        food_spawn_interval = scenario.get("food_spawn_interval")
        food_spawn_n = int(scenario.get("food_spawn_n", 1))
        print("Using experiment scenario:", scenario.get("scenario_name", "?"), f"(n_mothers={n_mothers}, n_children={n_children}, n_threats={n_threats})")
    else:
        n_mothers, n_children, n_threats, n_food_initial = 1, 1, 1, 0
        food_spawn_interval, food_spawn_n = None, 1

    random.seed(seed)
    occupied = set()
    mother_starts, occupied = random_unique_positions(n_mothers, grid_w, grid_h, occupied)
    child_starts,  occupied = random_unique_positions(n_children, grid_w, grid_h, occupied)
    threat_starts, occupied = random_unique_positions(n_threats, grid_w, grid_h, occupied)
    food_positions, _       = random_unique_positions(n_food_initial, grid_w, grid_h, occupied)

    # Colors
    bg_color = tuple(cfg["colors"]["bg"])
    grid_color = tuple(cfg["colors"]["grid"])
    mother_color = tuple(cfg["colors"]["mother"])
    threat_colot = tuple(cfg["colors"]["threat"])
    food_color = tuple(cfg["colors"]["food"])
    outline_color = tuple(cfg["colors"]["outline"])
    
    # Initialize pygame
    os.environ["SDL_VIDEO_WINDOW_POS"] = "900,250"
    pygame.init()


    # Calculate window size
    window_w = grid_w * cell_px
    window_h = grid_h * cell_px
    screen = pygame.display.set_mode((window_w, window_h))
    pygame.display.set_caption("Grid World: Mother, Child, Food, Threat, and Nest")
    clock = pygame.time.Clock()
    
    # Plasticity: fixed weight for visualization, or from scenario/config
    plasticity_rule = None if VISUALIZE_FIXED_WEIGHT else "hebbian"
    if scenario and not VISUALIZE_FIXED_WEIGHT:
        plasticity_rule = scenario.get("plasticity_rule") or plasticity_rule

    # Create world with all entities
    world_kw = dict(
        grid_w=grid_w, grid_h=grid_h,
        mother_starts=mother_starts,
        child_start=child_starts,
        food_positions=food_positions,
        threat_starts=threat_starts,
        day_step=day_step,
        plasticity_rule=plasticity_rule,
    )
    if scenario:
        world_kw["food_spawn_interval"] = food_spawn_interval
        world_kw["food_spawn_n"] = food_spawn_n
    world = World(**world_kw)
    if SHOW_PLOTS:
        plotter = MotherStatePlotter(world)
        mot_plotter = MotherMotivationPlotter(world)
        child_plotter = ChildStatePlotter(world)
        # weight_plotter = FixedVsPlasticPlotter(world)
        pass
    else:
        plotter = child_plotter = weight_plotter = None
    run_logger = RunLogger(
        world,
        mode=LOG_RUN_MODE or "csv",
        csv_path=LOG_RUN_CSV_PATH,
        mother_slots=list(range(len(world.mothers))),
        child_slots=list(range(len(world.children))),
    ) if LOG_RUN_MODE else None

    # Main game loop
    running = True
    accumulator = 0.0
    time = 0.0

    # plasticity_rule already set at World creation (fixed or hebbian/outcome)

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
            if world.tick % 3 == 0 and SHOW_PLOTS:
                plotter.update()
                child_plotter.update()
                mot_plotter.update()
                weight_plotter.update()
                pass
            if run_logger:
                run_logger.update()

        
        # ==============================================
        # ------------- Rendering ----------------
        # ==============================================

        # Render in order (background to foreground)
        draw_grid(screen, grid_w, grid_h, cell_px, bg_color, grid_color, outline_color)
      
        for food in world.foods:
            draw_food(screen, food, cell_px, food_color, outline_color)
        
        # Draw mother (use agent id; only alive so list indices stay M0,M1,M2)
        for m in world.mothers:
            if not m.is_alive():
                continue
            draw_mother(screen, m, cell_px, mother_color, outline_color, label=m.id)

        # Draw child
        for c in world.children:
            if not c.is_alive():
                continue
            color = intensity_to_color(c.energy)
            draw_child(screen, c, cell_px, color, outline_color, label=c.id)

        # Draw threat
        for i, t in enumerate(world.threats):
            percept_range = cell_px * t.perception_range
            draw_threat(screen, t, cell_px, threat_colot, outline_color, perception_range=percept_range)
        draw_clock_text(screen, world)

        # Update display
        pygame.display.flip()
        clock.tick(fps)
    
    # if run_logger:
    #     run_logger.close()
    pygame.quit()
    sys.exit()


if __name__ == "__main__":
    main()

