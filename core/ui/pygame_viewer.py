"""
Main Pygame application — initializes the world, runs the simulation
loop, renders the grid, and manages logging/plotting.
"""

import argparse
import json
import os
import sys
from pathlib import Path

import pygame

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


def main(world_overrides=None, episode=None, window_title=None):
    """
    Initialize and run the maternal instinct grid-world simulation.

    Parameters
    ----------
    world_overrides : dict, optional
        Extra keyword arguments for ``World()`` (e.g. baseline_weights from evolution).
    episode : dict, optional
        If set, must be ``{"init_seed": int, "world": {World kwargs}}`` from
        ``run_evolve_lineage`` watch mode (matches evolution layout and rules).
    window_title : str, optional
        Override Pygame window title.
    """

    # --- Load configuration ---
    config_path = Path(__file__).parent.parent / "base.yaml"
    cfg = load_config(str(config_path))

    cell_px = cfg["grid"]["cell_px"]
    fps = cfg["simulation"]["fps"]
    dt = cfg["simulation"]["dt"]
    day_step = cfg["days"]["ticks_per_day"]

    if episode is not None:
        init_seed(int(episode["init_seed"]))
        world_kw = dict(episode["world"])
        grid_w = int(world_kw["grid_w"])
        grid_h = int(world_kw["grid_h"])
    else:
        grid_w = cfg["grid"]["width"]
        grid_h = cfg["grid"]["height"]
        seed = cfg["simulation"]["seed"]
        day_step = cfg["days"]["ticks_per_day"]
        num_threats = 1
        food_at_start = 0
        food_spawn_interval = None
        food_spawn_n = 1

        # Optional CLI/world overrides
        if world_overrides:
            if "grid_w" in world_overrides:
                grid_w = int(world_overrides["grid_w"])
            if "grid_h" in world_overrides:
                grid_h = int(world_overrides["grid_h"])
            if "seed" in world_overrides:
                seed = int(world_overrides["seed"])
            if "day_step" in world_overrides:
                day_step = int(world_overrides["day_step"])
            if "num_threats" in world_overrides:
                num_threats = int(world_overrides["num_threats"])
            if "food_at_start" in world_overrides:
                food_at_start = int(world_overrides["food_at_start"])
            if "food_spawn_interval" in world_overrides:
                food_spawn_interval = world_overrides["food_spawn_interval"]
            if "food_spawn_n" in world_overrides:
                food_spawn_n = int(world_overrides["food_spawn_n"])

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
            n=num_threats, grid_w=grid_w, grid_h=grid_h, occupied=occupied
        )
        food_positions, occupied = random_unique_positions(
            n=food_at_start, grid_w=grid_w, grid_h=grid_h, occupied=occupied
        )

        world_kw = dict(
            grid_w=grid_w,
            grid_h=grid_h,
            mother_starts=mother_starts,
            child_start=child_starts,
            food_positions=food_positions,
            threat_starts=threat_starts,
            seed=seed,
            day_step=day_step,
            food_spawn_interval=food_spawn_interval,
            food_spawn_n=food_spawn_n,
        )
        # Keep other world_overrides (e.g. baseline_weights) but do not override
        # the position lists we just generated.
        if world_overrides:
            for k, v in world_overrides.items():
                if k in ("mother_starts", "child_start", "food_positions", "threat_starts"):
                    continue
                world_kw[k] = v

    # --- Colors from config ---
    bg_color = tuple(cfg["colors"]["bg"])
    grid_color = tuple(cfg["colors"]["grid"])
    mother_color = tuple(cfg["colors"]["mother"])
    threat_color = tuple(cfg["colors"]["threat"])
    food_color = tuple(cfg["colors"]["food"])
    outline_color = tuple(cfg["colors"]["outline"])

    # --- Initialize Pygame ---
    # Center on the primary display (fixed coords can place the window off-screen on some setups).
    os.environ.setdefault("SDL_VIDEO_CENTERED", "1")
    pygame.init()

    window_w = grid_w * cell_px
    window_h = grid_h * cell_px
    screen = pygame.display.set_mode((window_w, window_h))
    pygame.display.set_caption(window_title or "Maternal Instinct Simulation")
    clock = pygame.time.Clock()

    # --- Create world ---
    world = World(**world_kw)

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
                if weight_plotter is not None:
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


def _cli_overrides():
    parser = argparse.ArgumentParser(
        description="Maternal instinct gridworld (Pygame). Use --genome or --episode (from evolve watch)."
    )
    parser.add_argument(
        "--episode",
        type=str,
        default=None,
        help="JSON from run_evolve_lineage watch: {init_seed, world} matching one evolution rollout.",
    )
    parser.add_argument(
        "--genome",
        type=str,
        default=None,
        help="Path to final_genome.json from run_evolve_lineage.py (motivation weights / baseline_weights).",
    )
    parser.add_argument(
        "--plasticity",
        choices=["outcome", "none"],
        default="outcome",
        help="Plasticity rule for the mother (default: outcome). Ignored if --episode is set.",
    )
    parser.add_argument("--seed", type=int, default=None, help="RNG seed for layout when using --genome.")
    parser.add_argument("--grid-w", type=int, default=None, help="Grid width when using --genome.")
    parser.add_argument("--grid-h", type=int, default=None, help="Grid height when using --genome.")
    parser.add_argument("--threats", type=int, default=None, help="Number of threats when using --genome.")
    parser.add_argument("--food-start", type=int, default=None, help="Food placed at start when using --genome.")
    parser.add_argument(
        "--food-spawn-interval",
        type=str,
        default=None,
        help="Food spawn interval ticks when using --genome. Use 'none' to disable.",
    )
    parser.add_argument("--food-spawn-n", type=int, default=None, help="Food spawned each interval when using --genome.")
    parser.add_argument("--day-step", type=int, default=None, help="Ticks per day when using --genome.")
    parser.add_argument(
        "--title",
        type=str,
        default=None,
        help="Pygame window title.",
    )
    args = parser.parse_args()
    if args.episode:
        path = Path(args.episode)
        if not path.is_file():
            print(f"Episode file not found: {path}", file=sys.stderr)
            sys.exit(1)
        with open(path, encoding="utf-8") as f:
            ep = json.load(f)
        if "init_seed" not in ep or "world" not in ep:
            print("--episode JSON must contain init_seed and world", file=sys.stderr)
            sys.exit(1)
        return {"episode": ep, "title": args.title}
    if not args.genome:
        return {"episode": None, "overrides": None, "title": args.title}
    path = Path(args.genome)
    if not path.is_file():
        print(f"Genome file not found: {path}", file=sys.stderr)
        sys.exit(1)
    with open(path, encoding="utf-8") as f:
        genome = json.load(f)
    ov = {
        "baseline_weights": genome,
        "use_fixed_weights": True,
        "plasticity_rule": None if args.plasticity == "none" else "outcome",
    }
    if args.seed is not None:
        ov["seed"] = int(args.seed)
    if args.grid_w is not None:
        ov["grid_w"] = int(args.grid_w)
    if args.grid_h is not None:
        ov["grid_h"] = int(args.grid_h)
    if args.threats is not None:
        ov["num_threats"] = int(args.threats)
    if args.food_start is not None:
        ov["food_at_start"] = int(args.food_start)
    if args.food_spawn_interval is not None:
        s = str(args.food_spawn_interval).strip().lower()
        ov["food_spawn_interval"] = None if s in ("none", "null", "") else int(s)
    if args.food_spawn_n is not None:
        ov["food_spawn_n"] = int(args.food_spawn_n)
    if args.day_step is not None:
        ov["day_step"] = int(args.day_step)
    return {"episode": None, "overrides": ov, "title": args.title}


if __name__ == "__main__":
    cli = _cli_overrides()
    main(
        world_overrides=cli.get("overrides"),
        episode=cli.get("episode"),
        window_title=cli.get("title"),
    )
