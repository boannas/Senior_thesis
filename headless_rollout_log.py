"""
Headless grid-world rollout with the same per-tick CSV schema as pygame_viewer
(RunLogger). No Pygame window.

Use evolved weights (final_genome.json) or an episode JSON from evolve watch
(_watch_episode.json: { "init_seed", "world" }).

Then plot motivation / states:

  python plot_logged_run.py path/to/out.csv --out figures/

Or only save PNGs (no interactive windows) by setting MPLBACKEND=Agg before
running plot_logged_run.py if needed.

E0a-style plasticity + full RunLogger columns (for plot_weight_drift.py --runlog-glob):

  python headless_rollout_log.py --genome path/to/final_genome.json --seed 10000 `
    --max-ticks 1000 --csv result_experiment/E0a_runlogs/run_10000.csv `
    --plasticity outcome_adaptive --deficit-signal local --learn-w off `
    --update-mode segment_capped --segment-kmax 20

Multiple seeds in one command (--seed is the first seed; files run_S.csv, run_S+1.csv, …):

  python headless_rollout_log.py --genome path/to/final_genome.json --seed 10000 --n-seeds 32 `
    --out-dir result_experiment/E0a_runlogs --max-ticks 1000 --grid-w 10 --grid-h 10 `
    --plasticity outcome_adaptive --deficit-signal local --learn-w off `
    --update-mode segment_capped --segment-kmax 20 --threats 1

  python plot_weight_drift.py --runlog-glob "result_experiment/E0a_runlogs/*.csv" `
    --out result_experiment/figures/E0a_runlog_drift.png

Difference vs baseline_passive_lower_bound.py: same World + plasticity knobs, but this writes
one wide CSV per run (tick, all m0_u_fixed_*, m0_u_plastic_*, motivations, states).
Baseline writes lighter timeseries + survival batch CSVs for plot_conditions.py.
"""

from __future__ import annotations

import argparse
import copy
import json
import os
import sys

import numpy as np
import pandas as pd

from core.seed import init_seed
from core.world import World
from func.run_logger import build_row

# Defaults aligned with run_evolve_lineage.py
DEFAULT_GRID_W = 10
DEFAULT_GRID_H = 10
DEFAULT_NUM_MOTHERS = 1
DEFAULT_NUM_CHILDREN = 1
DEFAULT_NUM_THREATS = 1
DEFAULT_FOOD_AT_START = 0
DEFAULT_FOOD_SPAWN_INTERVAL = 20
DEFAULT_FOOD_SPAWN_N = 1
DEFAULT_DAY_STEP = 100
DEFAULT_MAX_TICKS = 10000


def _world_from_genome_and_seed(seed: int, genome: dict, cfg: dict) -> World:
    init_seed(seed)
    rng = np.random.RandomState(seed)
    grid_w = int(cfg["grid_w"])
    grid_h = int(cfg["grid_h"])
    num_mothers = int(cfg["num_mothers"])
    num_children = int(cfg["num_children"])
    num_threats = int(cfg["num_threats"])
    food_at_start = int(cfg["food_at_start"])

    mother_starts = [(rng.randint(0, grid_w), rng.randint(0, grid_h)) for _ in range(num_mothers)]
    child_start = [(rng.randint(0, grid_w), rng.randint(0, grid_h)) for _ in range(num_children)]
    food_positions = [
        (rng.randint(0, grid_w), rng.randint(0, grid_h)) for _ in range(food_at_start)
    ]
    threat_starts = [
        (rng.randint(0, grid_w), rng.randint(0, grid_h)) for _ in range(num_threats)
    ]

    return World(
        grid_w=grid_w,
        grid_h=grid_h,
        mother_starts=mother_starts,
        child_start=child_start,
        food_positions=food_positions,
        threat_starts=threat_starts,
        seed=seed,
        day_step=int(cfg["day_step"]),
        plasticity_rule=cfg.get("plasticity_rule", None),
        plasticity_deficit_signal=cfg.get("plasticity_deficit_signal", "global"),
        plasticity_learn_w=bool(cfg.get("plasticity_learn_w", False)),
        plasticity_update_mode=cfg.get("plasticity_update_mode", "per_tick"),
        plasticity_segment_kmax=int(cfg.get("plasticity_segment_kmax", 20)),
        food_spawn_interval=cfg.get("food_spawn_interval", None),
        food_spawn_n=int(cfg.get("food_spawn_n", 1)),
        use_fixed_weights=True,
        baseline_weights=copy.deepcopy(genome),
    )


def collect_rollout_dataframe(
    world: World,
    max_ticks: int,
    *,
    stop_when_both_dead: bool = False,
) -> pd.DataFrame:
    """
    Step the world up to ``max_ticks`` times and return the same columns as
    ``RunLogger`` CSV (via :func:`build_row`).
    """
    mother = world.mothers[0] if world.mothers else None
    child = world.children[0] if world.children else None
    rows: list = []
    mother_slots = None
    child_slots = None
    for _ in range(max_ticks):
        world.step()
        row = build_row(world, mother_slots, child_slots)
        if mother_slots is None:
            mother_slots = list(range(len(world.mothers)))
            child_slots = list(range(len(world.children)))
        rows.append(row)
        if stop_when_both_dead:
            m_dead = mother is None or not mother.is_alive()
            c_dead = child is None or not child.is_alive()
            if m_dead and c_dead:
                break
    return pd.DataFrame(rows)


def _parse_args():
    p = argparse.ArgumentParser(description="Headless rollout → run_log CSV (no Pygame).")
    g = p.add_mutually_exclusive_group(required=True)
    g.add_argument(
        "--genome",
        type=str,
        help="Path to motivation genome JSON (e.g. test_results/evolve_lineage/final_genome.json).",
    )
    g.add_argument(
        "--episode",
        type=str,
        help="Episode JSON from evolve watch: { init_seed, world } (same as pygame --episode).",
    )
    p.add_argument(
        "--seed",
        type=int,
        default=42000,
        help="First episode RNG seed when using --genome. With --n-seeds N, runs seeds S, S+1, …, S+N-1.",
    )
    p.add_argument(
        "--n-seeds",
        type=int,
        default=1,
        metavar="N",
        help="Number of rollouts (only with --genome). Each uses a distinct seed. Default 1 = single --csv.",
    )
    p.add_argument(
        "--out-dir",
        type=str,
        default=None,
        help="Required when --n-seeds > 1. Writes out-dir/run_<seed>.csv for each seed.",
    )
    p.add_argument("--csv", type=str, default="headless_run_log.csv", help="Output CSV path when --n-seeds is 1.")
    p.add_argument("--max-ticks", type=int, default=DEFAULT_MAX_TICKS, help="Max ticks (upper bound).")
    p.add_argument("--grid-w", type=int, default=DEFAULT_GRID_W)
    p.add_argument("--grid-h", type=int, default=DEFAULT_GRID_H)
    p.add_argument("--mothers", type=int, default=DEFAULT_NUM_MOTHERS)
    p.add_argument("--children", type=int, default=DEFAULT_NUM_CHILDREN)
    p.add_argument("--threats", type=int, default=DEFAULT_NUM_THREATS)
    p.add_argument("--food-start", type=int, default=DEFAULT_FOOD_AT_START)
    p.add_argument("--food-spawn-interval", type=int, default=DEFAULT_FOOD_SPAWN_INTERVAL)
    p.add_argument("--food-spawn-n", type=int, default=DEFAULT_FOOD_SPAWN_N)
    p.add_argument("--day-step", type=int, default=DEFAULT_DAY_STEP)
    p.add_argument(
        "--stop-when-both-dead",
        action="store_true",
        help="Stop early when mother and child are both dead (like fitness rollouts).",
    )
    p.add_argument(
        "--plasticity",
        choices=["none", "outcome", "outcome_adaptive", "outcome_signed", "outcome_adaptive_signed"],
        default="none",
        help="Lifetime plasticity rule (none = frozen u_plastic, same as evolve_lineage genes-only).",
    )
    p.add_argument(
        "--deficit-signal",
        choices=["global", "local"],
        default="global",
        help="Plasticity deficit: global=overall; local=motivation-aligned (matches baseline_passive_lower_bound).",
    )
    p.add_argument(
        "--learn-w",
        choices=["on", "off"],
        default="off",
        help="Whether psych weights w update (off = only motivation u; typical for E0a).",
    )
    p.add_argument(
        "--update-mode",
        choices=["per_tick", "segment", "segment_capped"],
        default="per_tick",
        help="Plasticity credit assignment (segment_capped recommended; matches baseline).",
    )
    p.add_argument(
        "--segment-kmax",
        type=int,
        default=20,
        help="For segment_capped: max ticks per segment before forced update.",
    )
    return p.parse_args()


def main():
    args = _parse_args()
    max_ticks = int(args.max_ticks)
    n_seeds = int(args.n_seeds)

    if args.episode:
        if n_seeds != 1:
            print("--n-seeds > 1 is only supported with --genome (episode fixes one layout).", file=sys.stderr)
            sys.exit(1)
        path = os.path.abspath(args.episode)
        if not os.path.isfile(path):
            print("Episode file not found:", path, file=sys.stderr)
            sys.exit(1)
        with open(path, encoding="utf-8") as f:
            ep = json.load(f)
        if "init_seed" not in ep or "world" not in ep:
            print("Episode JSON must contain init_seed and world", file=sys.stderr)
            sys.exit(1)
        init_seed(int(ep["init_seed"]))
        world = World(**dict(ep["world"]))
        df = collect_rollout_dataframe(world, max_ticks, stop_when_both_dead=bool(args.stop_when_both_dead))
        df.to_csv(args.csv, index=False)
        print(f"Wrote {os.path.abspath(args.csv)}")
        print("Plot: python plot_logged_run.py", args.csv, "--out <dir>")
        return

    if n_seeds < 1:
        print("--n-seeds must be >= 1", file=sys.stderr)
        sys.exit(1)
    if n_seeds > 1 and not args.out_dir:
        print("When --n-seeds > 1, set --out-dir (writes run_<seed>.csv there).", file=sys.stderr)
        sys.exit(1)

    gpath = os.path.abspath(args.genome)
    if not os.path.isfile(gpath):
        print("Genome file not found:", gpath, file=sys.stderr)
        sys.exit(1)
    with open(gpath, encoding="utf-8") as f:
        genome = json.load(f)
    if not isinstance(genome, dict):
        print("Genome JSON must be a dict (baseline_weights / motivation genome).", file=sys.stderr)
        sys.exit(1)

    rule = None if args.plasticity == "none" else args.plasticity
    cfg = {
        "grid_w": args.grid_w,
        "grid_h": args.grid_h,
        "num_mothers": args.mothers,
        "num_children": args.children,
        "num_threats": args.threats,
        "food_at_start": args.food_start,
        "food_spawn_interval": args.food_spawn_interval,
        "food_spawn_n": args.food_spawn_n,
        "day_step": args.day_step,
        "plasticity_rule": rule,
        "plasticity_deficit_signal": args.deficit_signal,
        "plasticity_learn_w": args.learn_w == "on",
        "plasticity_update_mode": args.update_mode,
        "plasticity_segment_kmax": int(args.segment_kmax),
    }

    base_seed = int(args.seed)
    if n_seeds > 1:
        out_dir = os.path.abspath(args.out_dir)
        os.makedirs(out_dir, exist_ok=True)
        for i in range(n_seeds):
            s = base_seed + i
            csv_path = os.path.join(out_dir, f"run_{s}.csv")
            world = _world_from_genome_and_seed(s, genome, cfg)
            df = collect_rollout_dataframe(world, max_ticks, stop_when_both_dead=bool(args.stop_when_both_dead))
            df.to_csv(csv_path, index=False)
            print(f"Wrote {csv_path}")
        print(f"Done ({n_seeds} seeds). Example: python plot_weight_drift.py --runlog-glob \"{out_dir}\\*.csv\" --out <png>")
    else:
        world = _world_from_genome_and_seed(base_seed, genome, cfg)
        df = collect_rollout_dataframe(world, max_ticks, stop_when_both_dead=bool(args.stop_when_both_dead))
        df.to_csv(args.csv, index=False)
        print(f"Wrote {os.path.abspath(args.csv)}")
        print("Plot: python plot_logged_run.py", args.csv, "--out <dir>")


if __name__ == "__main__":
    main()
