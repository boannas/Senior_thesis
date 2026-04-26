"""
Single-lineage evolution from Baseline-0 genes (motivation weights = 0.5).

Darwinian setup (as discussed):
- Genome = mother's fixed motivation weights U (nested dict matching MotherAgent).
- Psych weights W stay at 0.5 (same as baseline_weights branch in MotherAgent).
- Plasticity runs within each episode; each new World resets plastic = copy of fixed.
- One lineage: propose Gaussian mutation on U; accept if mean fitness improves across seeds.

Usage:
  python run_evolve_lineage.py
  python run_evolve_lineage.py --viz              # after evolution, open Pygame (blocks until window closed)
  python run_evolve_lineage.py --viz-only       # only open Pygame with existing final_genome.json
  python run_evolve_lineage.py --watch-every 1  # pause after each generation: Pygame shows current best (close to continue)

Outputs:
  test_results/evolve_lineage/lineage_generations.csv
  test_results/evolve_lineage/lineage_plot.png  (if matplotlib available)
  test_results/evolve_lineage/final_genome.json  (for Pygame: --genome)

Pygame after a run:
  python -m core.ui.pygame_viewer --genome test_results/evolve_lineage/final_genome.json
"""

from __future__ import annotations

import argparse
import ast
import copy
import csv
import json
import os
import subprocess
import sys
from datetime import datetime

import numpy as np

from core.seed import init_seed
from core.world import World

# --- Ecology (match baseline_0_runner unless you change these) ---
GRID_W = 10
GRID_H = 10
NUM_MOTHERS = 1
NUM_CHILDREN = 1
NUM_THREATS = 1
FOOD_AT_START = 1
FOOD_SPAWN_INTERVAL = 30
FOOD_SPAWN_N = 1
DAY_STEP = 100
MAX_TICKS = 1000

# Evolution
NUM_GENERATIONS = 100
EPISODES_PER_EVAL = 10  # random seeds per fitness estimate
MUTATION_SIGMA = 0.05
SEED_MASTER = 42

# Plasticity: "outcome" = within-life learning; None = genes only
PLASTICITY_RULE = None

# Fitness = mean(child_survival) - LAMBDA_INJURY * mean(final child injury normalized)
LAMBDA_INJURY = 0.002

OUTPUT_DIR = "test_results/evolve_lineage"


def episode_seeds(generation: int) -> list[int]:
    """Fixed evaluation seeds for a generation (same as fitness eval)."""
    return [SEED_MASTER * 1000 + generation * 100 + i for i in range(EPISODES_PER_EVAL)]


def baseline_zero_motivation_genome() -> dict:
    """Same motivation structure as Baseline-0 (all 0.5). Passed as baseline_weights."""
    return {
        "forage": {"child_hunger": 0.5, "energy_deficit": 0.5, "low_fear": 0.5},
        "care": {"child_warmth": 0.5, "closeness_deficit": 0.5, "bonding": 0.5},
        "self": {"fatigue": 0.5, "fear": 0.5, "stress": 0.5},
        "protect": {
            "child_injury": 0.5,
            "fear": 0.5,
            "closeness_deficit": 0.5,
            "bonding": 0.5,
        },
    }


def _flatten_keys(d: dict, prefix=()):
    """Yield (tuple path, value) for nested dict of floats."""
    for k, v in d.items():
        if isinstance(v, dict):
            yield from _flatten_keys(v, prefix + (k,))
        else:
            yield prefix + (k,), float(v)


def _set_path(d: dict, path: tuple, value: float) -> None:
    cur = d
    for p in path[:-1]:
        cur = cur[p]
    cur[path[-1]] = value


def mutate_genome(genome: dict, rng: np.random.RandomState, sigma: float) -> dict:
    """Independent Gaussian noise on each weight; clamp to [0.05, 1.0]."""
    out = copy.deepcopy(genome)
    for path, v in _flatten_keys(out):
        noise = rng.normal(0.0, sigma)
        nv = max(0.05, min(1.0, v + noise))
        _set_path(out, path, nv)
    return out


def run_episode(seed: int, motivation_genome: dict) -> dict:
    """One rollout; returns child_survival (0/1), final injury, motivation counts."""
    init_seed(seed)
    rng = np.random.RandomState(seed)

    mother_starts = [(rng.randint(0, GRID_W), rng.randint(0, GRID_H)) for _ in range(NUM_MOTHERS)]
    child_start = [(rng.randint(0, GRID_W), rng.randint(0, GRID_H)) for _ in range(NUM_CHILDREN)]
    food_positions = [
        (rng.randint(0, GRID_W), rng.randint(0, GRID_H)) for _ in range(FOOD_AT_START)
    ]
    threat_starts = [
        (rng.randint(0, GRID_W), rng.randint(0, GRID_H)) for _ in range(NUM_THREATS)
    ]

    world = World(
        grid_w=GRID_W,
        grid_h=GRID_H,
        mother_starts=mother_starts,
        child_start=child_start,
        food_positions=food_positions,
        threat_starts=threat_starts,
        seed=seed,
        day_step=DAY_STEP,
        plasticity_rule=PLASTICITY_RULE,
        food_spawn_interval=FOOD_SPAWN_INTERVAL,
        food_spawn_n=FOOD_SPAWN_N,
        use_fixed_weights=True,
        baseline_weights=copy.deepcopy(motivation_genome),
    )

    mother = world.mothers[0]
    child = world.children[0]
    mot_counts = {"Forage": 0, "Care": 0, "Self": 0, "Protect": 0}

    for _ in range(MAX_TICKS):
        world.step()
        if mother and mother.is_alive():
            sel = getattr(mother, "selected_motivation", None)
            if sel in mot_counts:
                mot_counts[sel] += 1

    child_alive = bool(child and child.is_alive())
    injury = float(child.injury) if child else 100.0
    return {
        "child_survival": 1.0 if child_alive else 0.0,
        "child_injury": injury,
        "mot_counts": mot_counts,
    }


def evaluate_genome(motivation_genome: dict, seeds: list[int], rng: np.random.RandomState) -> tuple[float, dict]:
    """Mean fitness and summary stats."""
    surv = []
    inj = []
    for s in seeds:
        r = run_episode(s, motivation_genome)
        surv.append(r["child_survival"])
        inj.append(r["child_injury"])
    mean_surv = float(np.mean(surv))
    mean_inj = float(np.mean(inj))
    fitness = mean_surv - LAMBDA_INJURY * mean_inj
    return fitness, {
        "mean_child_survival": mean_surv,
        "mean_child_injury": mean_inj,
        "std_child_survival": float(np.std(surv)),
    }


def build_watch_episode_payload(preview_seed: int, motivation_genome: dict) -> dict:
    """
    Same layout and rules as run_episode / fitness eval, for Pygame --episode.
    JSON-serializable (nested lists for positions).
    """
    init_seed(preview_seed)
    rng = np.random.RandomState(preview_seed)

    mother_starts = [(rng.randint(0, GRID_W), rng.randint(0, GRID_H)) for _ in range(NUM_MOTHERS)]
    child_start = [(rng.randint(0, GRID_W), rng.randint(0, GRID_H)) for _ in range(NUM_CHILDREN)]
    food_positions = [
        (rng.randint(0, GRID_W), rng.randint(0, GRID_H)) for _ in range(FOOD_AT_START)
    ]
    threat_starts = [
        (rng.randint(0, GRID_W), rng.randint(0, GRID_H)) for _ in range(NUM_THREATS)
    ]

    world = {
        "grid_w": GRID_W,
        "grid_h": GRID_H,
        "mother_starts": [list(t) for t in mother_starts],
        "child_start": [list(t) for t in child_start],
        "food_positions": [list(t) for t in food_positions],
        "threat_starts": [list(t) for t in threat_starts],
        "seed": preview_seed,
        "day_step": DAY_STEP,
        "plasticity_rule": PLASTICITY_RULE,
        "food_spawn_interval": FOOD_SPAWN_INTERVAL,
        "food_spawn_n": FOOD_SPAWN_N,
        "use_fixed_weights": True,
        "baseline_weights": copy.deepcopy(motivation_genome),
    }
    return {"init_seed": preview_seed, "world": world}


def _repo_root() -> str:
    return os.path.dirname(os.path.abspath(__file__))


def _genome_from_summary_txt(txt_path: str) -> dict | None:
    """Parse motivation dict from final_genome.txt (repr line)."""
    try:
        with open(txt_path, encoding="utf-8") as f:
            for line in f:
                line = line.strip()
                if line.startswith("{") and line.endswith("}"):
                    g = ast.literal_eval(line)
                    if isinstance(g, dict):
                        return g
    except (OSError, SyntaxError, ValueError, TypeError):
        pass
    return None


def resolve_genome_json_for_viz(json_path: str) -> str | None:
    """
    Return a path to final_genome.json, creating it from final_genome.txt if needed
    (older runs only wrote .txt).
    """
    json_path = os.path.abspath(json_path)
    if os.path.isfile(json_path):
        return json_path
    txt_path = os.path.join(os.path.dirname(json_path), "final_genome.txt")
    genome = _genome_from_summary_txt(txt_path)
    if genome is None:
        print(
            f"Cannot visualize: missing {json_path} and could not parse {txt_path}",
            file=sys.stderr,
        )
        return None
    with open(json_path, "w", encoding="utf-8") as f:
        json.dump(genome, f, indent=2)
    print(f"Wrote {json_path} (reconstructed from final_genome.txt for visualization)")
    return json_path


def _write_watch_episode_file(payload: dict) -> str:
    os.makedirs(OUTPUT_DIR, exist_ok=True)
    path = os.path.join(OUTPUT_DIR, "_watch_episode.json")
    with open(path, "w", encoding="utf-8") as f:
        json.dump(payload, f, indent=2)
    return path


def launch_pygame_episode(ep_path: str, title: str | None = None) -> None:
    """Run Pygame with a full episode JSON (same ecology as evolution). Blocks until window closes."""
    ep_path = os.path.abspath(ep_path)
    cmd = [sys.executable, "-m", "core.ui.pygame_viewer", "--episode", ep_path]
    if title:
        cmd.extend(["--title", title])
    print("Launching Pygame viewer (close window when done)...")
    print(" ", " ".join(cmd))
    try:
        subprocess.run(cmd, cwd=_repo_root(), check=False)
    except OSError as e:
        print(f"Failed to start viewer: {e}", file=sys.stderr)


def launch_pygame_visualizer(
    json_path: str,
    preview_seed: int | None = None,
    title: str | None = None,
) -> None:
    """
    Open Pygame with genome JSON using one evolution-matched rollout (food/threats/places like eval).
    """
    resolved = resolve_genome_json_for_viz(json_path)
    if not resolved:
        return
    with open(resolved, encoding="utf-8") as f:
        genome = json.load(f)
    seed = episode_seeds(0)[0] if preview_seed is None else preview_seed
    payload = build_watch_episode_payload(seed, genome)
    ep_path = _write_watch_episode_file(payload)
    launch_pygame_episode(ep_path, title=title or "Evolved genome (eval-matched episode)")


def main(open_viz_after: bool = False, watch_every: int = 0):
    os.makedirs(OUTPUT_DIR, exist_ok=True)
    rng = np.random.RandomState(SEED_MASTER)

    genome = baseline_zero_motivation_genome()
    gen_id = 0

    seeds0 = episode_seeds(0)
    fit, stats0 = evaluate_genome(genome, seeds0, rng)
    rows = [
        {
            "generation": gen_id,
            "fitness": fit,
            "accepted": True,
            "mean_child_survival": stats0["mean_child_survival"],
            "mean_child_injury": stats0["mean_child_injury"],
            "std_child_survival": stats0["std_child_survival"],
            "mutation_sigma": 0.0,
        }
    ]
    print(f"[gen {gen_id}] fitness={fit:.4f} mean_surv={stats0['mean_child_survival']:.3f}")

    best_genome = copy.deepcopy(genome)
    best_fit = fit

    if watch_every > 0 and gen_id % watch_every == 0:
        ep = build_watch_episode_payload(episode_seeds(0)[0], best_genome)
        ep_path = _write_watch_episode_file(ep)
        launch_pygame_episode(
            ep_path,
            title=f"Evolve watch — generation {gen_id} (close window to continue)",
        )

    for gen_id in range(1, NUM_GENERATIONS + 1):
        candidate = mutate_genome(best_genome, rng, MUTATION_SIGMA)
        seeds = episode_seeds(gen_id)
        fit_c, stats_c = evaluate_genome(candidate, seeds, rng)
        accepted = fit_c >= best_fit
        if accepted:
            best_fit = fit_c
            best_genome = candidate
            print(
                f"[gen {gen_id}] ACCEPT fitness={fit_c:.4f} mean_surv={stats_c['mean_child_survival']:.3f}"
            )
        else:
            print(
                f"[gen {gen_id}] reject fitness={fit_c:.4f} (best={best_fit:.4f})"
            )

        rows.append(
            {
                "generation": gen_id,
                "fitness": fit_c,
                "accepted": accepted,
                "mean_child_survival": stats_c["mean_child_survival"],
                "mean_child_injury": stats_c["mean_child_injury"],
                "std_child_survival": stats_c["std_child_survival"],
                "mutation_sigma": MUTATION_SIGMA,
            }
        )

        if watch_every > 0 and gen_id % watch_every == 0:
            ep = build_watch_episode_payload(episode_seeds(gen_id)[0], best_genome)
            ep_path = _write_watch_episode_file(ep)
            launch_pygame_episode(
                ep_path,
                title=f"Evolve watch — generation {gen_id} (close window to continue)",
            )

    csv_path = os.path.join(OUTPUT_DIR, "lineage_generations.csv")
    with open(csv_path, "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)

    # Save final genome as text summary
    summary_path = os.path.join(OUTPUT_DIR, "final_genome.txt")
    with open(summary_path, "w", encoding="utf-8") as f:
        f.write(f"# evolve_lineage run {datetime.now().isoformat()}\n")
        f.write(f"best_fitness={best_fit}\n")
        f.write(repr(best_genome))
        f.write("\n")

    json_path = os.path.join(OUTPUT_DIR, "final_genome.json")
    with open(json_path, "w", encoding="utf-8") as f:
        json.dump(best_genome, f, indent=2)

    print(f"Wrote {csv_path}")
    print(f"Wrote {summary_path}")
    print(f"Wrote {json_path}")

    _plot_lineage(rows, OUTPUT_DIR)

    if open_viz_after:
        launch_pygame_visualizer(
            json_path,
            preview_seed=episode_seeds(NUM_GENERATIONS)[0],
            title="Evolve finished — best genome (close window to exit)",
        )


def _plot_lineage(rows: list[dict], output_dir: str) -> None:
    """Save fitness / survival vs generation; mark accepted mutations."""
    try:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("matplotlib not installed; skip lineage_plot.png (pip install matplotlib)")
        return

    gens = [int(r["generation"]) for r in rows]
    fitness = [float(r["fitness"]) for r in rows]
    surv = [float(r["mean_child_survival"]) for r in rows]
    acc = [bool(r.get("accepted", True)) for r in rows]

    best_so_far = []
    b = -1e9
    for f in fitness:
        b = max(b, f)
        best_so_far.append(b)

    fig, axes = plt.subplots(2, 1, figsize=(9, 6), sharex=True)

    ax0 = axes[0]
    ax0.plot(gens, fitness, "o-", color="steelblue", alpha=0.85, label="evaluated fitness", markersize=4)
    ax0.plot(gens, best_so_far, "--", color="darkgreen", linewidth=1.5, label="best so far")
    for g, f, ok in zip(gens, fitness, acc):
        if ok:
            ax0.scatter([g], [f], color="green", s=36, zorder=5, edgecolors="black", linewidths=0.5)
    ax0.set_ylabel("Fitness")
    ax0.legend(loc="lower right", fontsize=8)
    ax0.grid(True, alpha=0.3)
    ax0.set_title("Single-lineage evolution (motivation genome)")

    ax1 = axes[1]
    ax1.plot(gens, surv, "s-", color="coral", alpha=0.85, markersize=4, label="mean child survival (eval batch)")
    ax1.set_xlabel("Generation")
    ax1.set_ylabel("Mean child survival")
    ax1.set_ylim(-0.05, 1.05)
    ax1.legend(loc="lower right", fontsize=8)
    ax1.grid(True, alpha=0.3)

    fig.tight_layout()
    out_png = os.path.join(output_dir, "lineage_plot.png")
    fig.savefig(out_png, dpi=150)
    plt.close(fig)
    print(f"Wrote {out_png}")


def _parse_args():
    p = argparse.ArgumentParser(description="Single-lineage evolution of motivation weights.")
    p.add_argument(
        "--viz",
        action="store_true",
        help="After evolution, open Pygame with final genome (eval-matched episode).",
    )
    p.add_argument(
        "--viz-only",
        action="store_true",
        dest="viz_only",
        help="Skip evolution; only open Pygame using existing test_results/evolve_lineage/final_genome.json",
    )
    p.add_argument(
        "--watch-every",
        type=int,
        default=0,
        metavar="N",
        help=(
            "Pause after every N generations and open Pygame with the current best genome "
            "(same random layout rules as fitness evaluation). Close the window to continue. 0 = off."
        ),
    )
    args = p.parse_args()
    if args.watch_every < 0:
        p.error("--watch-every must be >= 0")
    return args


if __name__ == "__main__":
    args = _parse_args()
    if args.viz_only:
        json_path = os.path.join(OUTPUT_DIR, "final_genome.json")
        launch_pygame_visualizer(json_path)
    else:
        main(open_viz_after=args.viz, watch_every=args.watch_every)
