"""
Single-lineage evolution from Baseline-0 genes (motivation weights = 0.5).

Darwinian setup (as discussed):
- Genome = mother's fixed motivation weights U (nested dict matching MotherAgent).
- Psych weights W stay at 0.5 (same as baseline_weights branch in MotherAgent).
- Plasticity is OFF by default (genes only). For Baldwin / lifetime learning during evolution, pass e.g.
    --plasticity outcome_adaptive --deficit-signal local --learn-w off --update-mode segment_capped
- One lineage: propose Gaussian mutation on U; accept if mean fitness improves across seeds.

Usage:
  python run_evolve_lineage.py
  # E3 Baldwin (plasticity on during fitness rollouts; freeze w by default):
  python run_evolve_lineage.py --plasticity outcome_adaptive --deficit-signal local --learn-w off \\
      --update-mode segment_capped --segment-kmax 20 --episodes 32 --output-dir result_experiment/E3_baldwin
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

from core.policies.mother import compute_local_deficit, compute_overall_deficit
from core.seed import init_seed
from core.world import World

# --- Defaults (override via CLI) ---
DEFAULT_GRID_W = 10
DEFAULT_GRID_H = 10
DEFAULT_NUM_MOTHERS = 1
DEFAULT_NUM_CHILDREN = 1
DEFAULT_NUM_THREATS = 1
DEFAULT_FOOD_AT_START = 0
DEFAULT_FOOD_SPAWN_INTERVAL = 20
DEFAULT_FOOD_SPAWN_N = 1
DEFAULT_DAY_STEP = 100
DEFAULT_MAX_TICKS = 1000

# Evolution defaults
DEFAULT_NUM_GENERATIONS = 100
DEFAULT_EPISODES_PER_EVAL = 10  # seeds per fitness estimate
DEFAULT_MUTATION_SIGMA = 0.05
DEFAULT_SEED_MASTER = 42

# Plasticity: always off for this script unless explicitly overridden
DEFAULT_PLASTICITY_RULE = 'outcome_adaptive'

# Fitness = mean(child_survival) - LAMBDA_INJURY * mean(final child injury normalized)
LAMBDA_INJURY = 0.002

DEFAULT_OUTPUT_DIR = "test_results/evolve_lineage"

# Fitness modes:
# - "binary_child": 1 if child alive at end, else 0 (original behavior)
# - "ttd_child": normalized child time-to-death in [0,1]
# - "ttd_overall": weighted normalized time-to-death (child + mother)
DEFAULT_FITNESS_MODE = "ttd_overall"
DEFAULT_ALPHA_CHILD = 0.7


def episode_seeds(generation: int, *, seed_master: int, episodes_per_eval: int) -> list[int]:
    """Fixed evaluation seeds for a generation (same as fitness eval)."""
    return [seed_master * 1000 + generation * 100 + i for i in range(episodes_per_eval)]


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


def _mean_abs_diff_nested(a: dict, b: dict) -> float:
    """Mean |a−b| over matching nested dict leaves (motivation u_fixed vs u_plastic)."""
    vals = []
    for k, v in a.items():
        if isinstance(v, dict) and isinstance(b.get(k), dict):
            vals.append(_mean_abs_diff_nested(v, b[k]))
        else:
            try:
                vals.append(abs(float(v) - float(b.get(k, v))))
            except (TypeError, ValueError):
                continue
    vals = [x for x in vals if np.isfinite(x)]
    return float(np.mean(vals)) if vals else float("nan")


def mutate_genome(genome: dict, rng: np.random.RandomState, sigma: float) -> dict:
    """Independent Gaussian noise on each weight; clamp to [0.05, 1.0]."""
    out = copy.deepcopy(genome)
    for path, v in _flatten_keys(out):
        noise = rng.normal(0.0, sigma)
        nv = max(0.05, min(1.0, v + noise))
        _set_path(out, path, nv)
    return out


def run_episode(seed: int, motivation_genome: dict, cfg: dict) -> dict:
    """One rollout; returns survival/time-to-death signals and motivation counts."""
    init_seed(seed)
    rng = np.random.RandomState(seed)

    grid_w = int(cfg["grid_w"])
    grid_h = int(cfg["grid_h"])
    num_mothers = int(cfg["num_mothers"])
    num_children = int(cfg["num_children"])
    num_threats = int(cfg["num_threats"])
    food_at_start = int(cfg["food_at_start"])
    max_ticks = int(cfg["max_ticks"])

    mother_starts = [(rng.randint(0, grid_w), rng.randint(0, grid_h)) for _ in range(num_mothers)]
    child_start = [(rng.randint(0, grid_w), rng.randint(0, grid_h)) for _ in range(num_children)]
    food_positions = [
        (rng.randint(0, grid_w), rng.randint(0, grid_h)) for _ in range(food_at_start)
    ]
    threat_starts = [
        (rng.randint(0, grid_w), rng.randint(0, grid_h)) for _ in range(num_threats)
    ]

    world = World(
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
        baseline_weights=copy.deepcopy(motivation_genome),
    )

    mother = world.mothers[0]
    child = world.children[0]
    mot_counts = {"Forage": 0, "Care": 0, "Self": 0, "Protect": 0}

    child_death_tick = None
    mother_death_tick = None
    overall_deficit_samples: list[float] = []
    local_deficit_samples: list[float] = []

    for tick in range(max_ticks):
        world.step()
        if mother and mother.is_alive():
            overall_deficit_samples.append(float(compute_overall_deficit(mother)))
            sel = getattr(mother, "selected_motivation", None)
            local_deficit_samples.append(float(compute_local_deficit(mother, sel)))
            if sel in mot_counts:
                mot_counts[sel] += 1

        # record first death times (if any)
        if child_death_tick is None and (not child or not child.is_alive()):
            child_death_tick = tick
        if mother_death_tick is None and (not mother or not mother.is_alive()):
            mother_death_tick = tick
        if (child_death_tick is not None) and (mother_death_tick is not None):
            break

    child_alive = bool(child and child.is_alive())
    injury = float(child.injury) if child else 100.0
    # right-censor at max_ticks if still alive
    if child_death_tick is None:
        child_death_tick = max_ticks
    if mother_death_tick is None:
        mother_death_tick = max_ticks

    child_ttd_norm = float(child_death_tick) / float(max_ticks) if max_ticks > 0 else 0.0
    mother_ttd_norm = float(mother_death_tick) / float(max_ticks) if max_ticks > 0 else 0.0
    u_drift_end = float("nan")
    if mother is not None and hasattr(mother, "motivation_weights_fixed") and hasattr(mother, "motivation_weights_plastic"):
        u_drift_end = _mean_abs_diff_nested(mother.motivation_weights_fixed, mother.motivation_weights_plastic)

    def _mean_last(xs: list[float]) -> float:
        if not xs:
            return float("nan")
        return float(np.mean(xs))

    def _last(xs: list[float]) -> float:
        return float(xs[-1]) if xs else float("nan")

    return {
        "child_survival": 1.0 if child_alive else 0.0,
        "child_injury": injury,
        "child_death_tick": int(child_death_tick),
        "mother_death_tick": int(mother_death_tick),
        "child_ttd_norm": float(child_ttd_norm),
        "mother_ttd_norm": float(mother_ttd_norm),
        "mot_counts": mot_counts,
        "u_drift_end": float(u_drift_end),
        "mean_overall_deficit_episode": _mean_last(overall_deficit_samples),
        "overall_deficit_end": _last(overall_deficit_samples),
        "mean_local_deficit_episode": _mean_last(local_deficit_samples),
        "local_deficit_end": _last(local_deficit_samples),
    }


def evaluate_genome(motivation_genome: dict, seeds: list[int], rng: np.random.RandomState, cfg: dict) -> tuple[float, dict]:
    """Mean fitness and summary stats."""
    surv = []
    inj = []
    child_ttd = []
    mother_ttd = []
    u_drifts = []
    odef_m = []
    odef_e = []
    ldef_m = []
    ldef_e = []
    for s in seeds:
        r = run_episode(s, motivation_genome, cfg)
        surv.append(r["child_survival"])
        inj.append(r["child_injury"])
        child_ttd.append(r["child_ttd_norm"])
        mother_ttd.append(r["mother_ttd_norm"])
        u_drifts.append(float(r.get("u_drift_end", float("nan"))))
        odef_m.append(float(r.get("mean_overall_deficit_episode", float("nan"))))
        odef_e.append(float(r.get("overall_deficit_end", float("nan"))))
        ldef_m.append(float(r.get("mean_local_deficit_episode", float("nan"))))
        ldef_e.append(float(r.get("local_deficit_end", float("nan"))))
    mean_surv = float(np.mean(surv))
    mean_inj = float(np.mean(inj))
    mean_child_ttd = float(np.mean(child_ttd)) if child_ttd else 0.0
    mean_mother_ttd = float(np.mean(mother_ttd)) if mother_ttd else 0.0

    fitness_mode = str(cfg.get("fitness_mode", DEFAULT_FITNESS_MODE))
    alpha_child = float(cfg.get("alpha_child", DEFAULT_ALPHA_CHILD))
    alpha_child = max(0.0, min(1.0, alpha_child))

    if fitness_mode == "binary_child":
        base = mean_surv
    elif fitness_mode == "ttd_child":
        base = mean_child_ttd
    elif fitness_mode == "ttd_overall":
        base = alpha_child * mean_child_ttd + (1.0 - alpha_child) * mean_mother_ttd
    else:
        raise ValueError(f"Unknown fitness_mode: {fitness_mode!r}")

    fitness = float(base) - float(LAMBDA_INJURY) * mean_inj

    return fitness, {
        "mean_child_survival": mean_surv,
        "mean_child_ttd_norm": mean_child_ttd,
        "mean_mother_ttd_norm": mean_mother_ttd,
        "mean_child_injury": mean_inj,
        "std_child_survival": float(np.std(surv)),
        "mean_u_drift_end": float(np.nanmean(u_drifts)),
        "std_u_drift_end": float(np.nanstd(u_drifts, ddof=0)),
        "mean_overall_deficit_episode": float(np.nanmean(odef_m)),
        "std_overall_deficit_episode": float(np.nanstd(odef_m, ddof=0)),
        "mean_overall_deficit_end": float(np.nanmean(odef_e)),
        "std_overall_deficit_end": float(np.nanstd(odef_e, ddof=0)),
        "mean_local_deficit_episode": float(np.nanmean(ldef_m)),
        "std_local_deficit_episode": float(np.nanstd(ldef_m, ddof=0)),
        "mean_local_deficit_end": float(np.nanmean(ldef_e)),
        "std_local_deficit_end": float(np.nanstd(ldef_e, ddof=0)),
    }


def build_watch_episode_payload(preview_seed: int, motivation_genome: dict, cfg: dict) -> dict:
    """
    Same layout and rules as run_episode / fitness eval, for Pygame --episode.
    JSON-serializable (nested lists for positions).
    """
    init_seed(preview_seed)
    rng = np.random.RandomState(preview_seed)

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

    world = {
        "grid_w": grid_w,
        "grid_h": grid_h,
        "mother_starts": [list(t) for t in mother_starts],
        "child_start": [list(t) for t in child_start],
        "food_positions": [list(t) for t in food_positions],
        "threat_starts": [list(t) for t in threat_starts],
        "seed": preview_seed,
        "day_step": int(cfg["day_step"]),
        "plasticity_rule": cfg.get("plasticity_rule", None),
        "plasticity_deficit_signal": cfg.get("plasticity_deficit_signal", "global"),
        "plasticity_learn_w": bool(cfg.get("plasticity_learn_w", False)),
        "plasticity_update_mode": cfg.get("plasticity_update_mode", "per_tick"),
        "plasticity_segment_kmax": int(cfg.get("plasticity_segment_kmax", 20)),
        "food_spawn_interval": cfg.get("food_spawn_interval", None),
        "food_spawn_n": int(cfg.get("food_spawn_n", 1)),
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


def _write_watch_episode_file(payload: dict, output_dir: str) -> str:
    os.makedirs(output_dir, exist_ok=True)
    path = os.path.join(output_dir, "_watch_episode.json")
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
    # Fallback config for viz-only (keeps original defaults).
    cfg = _default_cfg()
    seed = episode_seeds(0, seed_master=cfg["seed_master"], episodes_per_eval=cfg["episodes_per_eval"])[0] if preview_seed is None else preview_seed
    payload = build_watch_episode_payload(seed, genome, cfg)
    ep_path = _write_watch_episode_file(payload, os.path.dirname(json_path))
    launch_pygame_episode(ep_path, title=title or "Evolved genome (eval-matched episode)")


def _default_cfg() -> dict:
    return {
        "grid_w": DEFAULT_GRID_W,
        "grid_h": DEFAULT_GRID_H,
        "num_mothers": DEFAULT_NUM_MOTHERS,
        "num_children": DEFAULT_NUM_CHILDREN,
        "num_threats": DEFAULT_NUM_THREATS,
        "food_at_start": DEFAULT_FOOD_AT_START,
        "food_spawn_interval": DEFAULT_FOOD_SPAWN_INTERVAL,
        "food_spawn_n": DEFAULT_FOOD_SPAWN_N,
        "day_step": DEFAULT_DAY_STEP,
        "max_ticks": DEFAULT_MAX_TICKS,
        "num_generations": DEFAULT_NUM_GENERATIONS,
        "episodes_per_eval": DEFAULT_EPISODES_PER_EVAL,
        "mutation_sigma": DEFAULT_MUTATION_SIGMA,
        "seed_master": DEFAULT_SEED_MASTER,
        "plasticity_rule": DEFAULT_PLASTICITY_RULE,
        "plasticity_deficit_signal": "global",
        "plasticity_learn_w": False,
        "plasticity_update_mode": "per_tick",
        "plasticity_segment_kmax": 20,
        "fitness_mode": DEFAULT_FITNESS_MODE,
        "alpha_child": DEFAULT_ALPHA_CHILD,
        "output_dir": DEFAULT_OUTPUT_DIR,
    }


def main(cfg: dict, open_viz_after: bool = False, watch_every: int = 0):
    output_dir = str(cfg["output_dir"])
    os.makedirs(output_dir, exist_ok=True)
    rng = np.random.RandomState(int(cfg["seed_master"]))

    genome = baseline_zero_motivation_genome()
    gen_id = 0

    threats_schedule: list[tuple[int, int]] = list(cfg.get("threats_schedule", []) or [])
    base_threats = int(cfg.get("num_threats", DEFAULT_NUM_THREATS))
    if threats_schedule:
        print(
            f"[threats schedule] base={base_threats}  steps="
            + ", ".join(f"gen{g}->{n}" for g, n in threats_schedule)
        )

    cfg["num_threats"] = _threats_at_gen(gen_id, threats_schedule, base_threats)
    active_threats_now = int(cfg["num_threats"])
    seeds0 = episode_seeds(0, seed_master=int(cfg["seed_master"]), episodes_per_eval=int(cfg["episodes_per_eval"]))
    fit, stats0 = evaluate_genome(genome, seeds0, rng, cfg)
    rows = [
        {
            "generation": gen_id,
            "fitness": fit,
            "accepted": True,
            "num_threats_active": active_threats_now,
            "mean_child_survival": stats0["mean_child_survival"],
            "mean_child_ttd_norm": stats0.get("mean_child_ttd_norm", np.nan),
            "mean_mother_ttd_norm": stats0.get("mean_mother_ttd_norm", np.nan),
            "mean_child_injury": stats0["mean_child_injury"],
            "std_child_survival": stats0["std_child_survival"],
            "mean_u_drift_end": stats0.get("mean_u_drift_end", np.nan),
            "std_u_drift_end": stats0.get("std_u_drift_end", np.nan),
            "mean_overall_deficit_episode": stats0.get("mean_overall_deficit_episode", np.nan),
            "std_overall_deficit_episode": stats0.get("std_overall_deficit_episode", np.nan),
            "mean_overall_deficit_end": stats0.get("mean_overall_deficit_end", np.nan),
            "std_overall_deficit_end": stats0.get("std_overall_deficit_end", np.nan),
            "mean_local_deficit_episode": stats0.get("mean_local_deficit_episode", np.nan),
            "std_local_deficit_episode": stats0.get("std_local_deficit_episode", np.nan),
            "mean_local_deficit_end": stats0.get("mean_local_deficit_end", np.nan),
            "std_local_deficit_end": stats0.get("std_local_deficit_end", np.nan),
            "mutation_sigma": 0.0,
        }
    ]
    print(f"[gen {gen_id}] fitness={fit:.4f} mean_surv={stats0['mean_child_survival']:.3f}")

    best_genome = copy.deepcopy(genome)
    best_fit = fit
    best_gen = 0

    def _write_checkpoint(gen_id_: int, best_fit_: float, best_genome_: dict) -> None:
        ck_dir = os.path.join(output_dir, "checkpoints")
        os.makedirs(ck_dir, exist_ok=True)
        tag = f"gen{gen_id_:04d}"
        ck_json = os.path.join(ck_dir, f"best_{tag}.json")
        ck_txt = os.path.join(ck_dir, f"best_{tag}.txt")
        with open(ck_json, "w", encoding="utf-8") as f:
            json.dump(best_genome_, f, indent=2)
        with open(ck_txt, "w", encoding="utf-8") as f:
            f.write(f"# checkpoint {datetime.now().isoformat()}\n")
            f.write(f"generation={gen_id_}\n")
            f.write(f"best_fitness={best_fit_}\n")
            f.write(f"fitness_mode={cfg.get('fitness_mode')!r}\n")
            f.write(f"alpha_child={cfg.get('alpha_child')!r}\n")
            f.write(f"plasticity_rule={cfg.get('plasticity_rule', None)!r}\n")
            f.write(repr(best_genome_))
            f.write("\n")

    if watch_every > 0 and gen_id % watch_every == 0:
        ep = build_watch_episode_payload(seeds0[0], best_genome, cfg)
        ep_path = _write_watch_episode_file(ep, output_dir)
        launch_pygame_episode(
            ep_path,
            title=f"Evolve watch — generation {gen_id} (close window to continue)",
        )

    checkpoint_every = int(cfg.get("checkpoint_every", 0) or 0)
    if checkpoint_every > 0 and gen_id % checkpoint_every == 0:
        _write_checkpoint(gen_id, best_fit, best_genome)

    for gen_id in range(1, int(cfg["num_generations"]) + 1):
        new_threats = _threats_at_gen(gen_id, threats_schedule, base_threats)
        if new_threats != int(cfg["num_threats"]):
            print(f"[gen {gen_id}] threats: {int(cfg['num_threats'])} -> {new_threats}")
        cfg["num_threats"] = new_threats
        active_threats_now = int(cfg["num_threats"])

        candidate = mutate_genome(best_genome, rng, float(cfg["mutation_sigma"]))
        seeds = episode_seeds(gen_id, seed_master=int(cfg["seed_master"]), episodes_per_eval=int(cfg["episodes_per_eval"]))
        fit_c, stats_c = evaluate_genome(candidate, seeds, rng, cfg)
        accepted = fit_c >= best_fit
        if accepted:
            best_fit = fit_c
            best_genome = candidate
            best_gen = gen_id
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
                "num_threats_active": active_threats_now,
                "mean_child_survival": stats_c["mean_child_survival"],
                "mean_child_ttd_norm": stats_c.get("mean_child_ttd_norm", np.nan),
                "mean_mother_ttd_norm": stats_c.get("mean_mother_ttd_norm", np.nan),
                "mean_child_injury": stats_c["mean_child_injury"],
                "std_child_survival": stats_c["std_child_survival"],
                "mean_u_drift_end": stats_c.get("mean_u_drift_end", np.nan),
                "std_u_drift_end": stats_c.get("std_u_drift_end", np.nan),
                "mean_overall_deficit_episode": stats_c.get("mean_overall_deficit_episode", np.nan),
                "std_overall_deficit_episode": stats_c.get("std_overall_deficit_episode", np.nan),
                "mean_overall_deficit_end": stats_c.get("mean_overall_deficit_end", np.nan),
                "std_overall_deficit_end": stats_c.get("std_overall_deficit_end", np.nan),
                "mean_local_deficit_episode": stats_c.get("mean_local_deficit_episode", np.nan),
                "std_local_deficit_episode": stats_c.get("std_local_deficit_episode", np.nan),
                "mean_local_deficit_end": stats_c.get("mean_local_deficit_end", np.nan),
                "std_local_deficit_end": stats_c.get("std_local_deficit_end", np.nan),
                "mutation_sigma": float(cfg["mutation_sigma"]),
            }
        )

        if watch_every > 0 and gen_id % watch_every == 0:
            ep = build_watch_episode_payload(seeds[0], best_genome, cfg)
            ep_path = _write_watch_episode_file(ep, output_dir)
            launch_pygame_episode(
                ep_path,
                title=f"Evolve watch — generation {gen_id} (close window to continue)",
            )

        if checkpoint_every > 0 and gen_id % checkpoint_every == 0:
            _write_checkpoint(gen_id, best_fit, best_genome)

    csv_path = os.path.join(output_dir, "lineage_generations.csv")
    with open(csv_path, "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)

    # Save final genome as text summary
    summary_path = os.path.join(output_dir, "final_genome.txt")
    with open(summary_path, "w", encoding="utf-8") as f:
        f.write(f"# evolve_lineage run {datetime.now().isoformat()}\n")
        f.write(f"# plasticity_rule={cfg.get('plasticity_rule', None)!r}\n")
        f.write(f"# best_generation={best_gen}\n")
        f.write(f"best_fitness={best_fit}\n")
        f.write(repr(best_genome))
        f.write("\n")

    json_path = os.path.join(output_dir, "final_genome.json")
    with open(json_path, "w", encoding="utf-8") as f:
        json.dump(best_genome, f, indent=2)

    print(f"Wrote {csv_path}")
    print(f"Wrote {summary_path}")
    print(f"Wrote {json_path}")

    _plot_lineage(rows, output_dir)

    if open_viz_after:
        launch_pygame_visualizer(
            json_path,
            preview_seed=episode_seeds(int(cfg["num_generations"]), seed_master=int(cfg["seed_master"]), episodes_per_eval=int(cfg["episodes_per_eval"]))[0],
            title="Evolve finished — best genome (close window to exit)",
        )


def _plot_lineage(rows: list[dict], output_dir: str) -> None:
    """Save evolution traces vs generation; mark accepted mutations."""
    try:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("matplotlib not installed; skip lineage_plot.png (pip install matplotlib)")
        return

    gens = [int(r["generation"]) for r in rows]
    fitness = [float(r["fitness"]) for r in rows]
    acc = [bool(r.get("accepted", True)) for r in rows]

    # Optional traces depending on fitness mode / available columns
    def _col(name: str):
        vals = []
        ok = True
        for r in rows:
            if name not in r:
                ok = False
                break
            vals.append(r[name])
        return ok, vals

    has_surv, surv = _col("mean_child_survival")
    has_cttd, cttd = _col("mean_child_ttd_norm")
    has_mttd, mttd = _col("mean_mother_ttd_norm")
    has_ud, udrift = _col("mean_u_drift_end")
    has_ud_std, udrift_std = _col("std_u_drift_end")

    best_so_far = []
    b = -1e9
    for f in fitness:
        b = max(b, f)
        best_so_far.append(b)

    # 3 panels when TTD available; otherwise fall back to 2 panels.
    n_panels = 3 if (has_cttd or has_mttd) else 2
    fig, axes = plt.subplots(n_panels, 1, figsize=(9, 3.2 * n_panels), sharex=True)
    if n_panels == 1:
        axes = [axes]

    ax0 = axes[0]
    ax0.plot(gens, fitness, "o-", color="steelblue", alpha=0.85, label="evaluated fitness", markersize=4)
    ax0.plot(gens, best_so_far, "--", color="darkgreen", linewidth=1.5, label="best so far")
    for g, f, ok in zip(gens, fitness, acc):
        if ok:
            ax0.scatter([g], [f], color="green", s=36, zorder=5, edgecolors="black", linewidths=0.5)
    ax0.set_ylabel("Fitness")
    ax0.legend(loc="lower left", fontsize=8)
    ax0.grid(True, alpha=0.3)
    ax0.set_title("Single-lineage evolution (motivation genome)")
    if has_ud:
        ax0u = ax0.twinx()
        ud_vals = [float(x) for x in udrift]
        ax0u.plot(gens, ud_vals, "o-", color="darkmagenta", alpha=0.75, markersize=3, label="mean |Δu| at episode end")
        if has_ud_std:
            sds = [float(x) for x in udrift_std]
            lo = [max(0.0, u - s) for u, s in zip(ud_vals, sds)]
            hi = [u + s for u, s in zip(ud_vals, sds)]
            ax0u.fill_between(gens, lo, hi, color="darkmagenta", alpha=0.12, linewidth=0)
        ax0u.set_ylabel("Plasticity: mean |u_plas−u_fix| (eval batch)")
        ax0u.legend(loc="upper right", fontsize=8)

    ax1 = axes[1]
    if has_cttd or has_mttd:
        # Prefer TTD traces (dense signal)
        if has_cttd:
            ax1.plot(gens, [float(x) for x in cttd], "s-", color="tab:orange", alpha=0.9, markersize=4, label="mean child TTD (norm)")
        if has_mttd:
            ax1.plot(gens, [float(x) for x in mttd], "D-", color="tab:purple", alpha=0.85, markersize=3.5, label="mean mother TTD (norm)")
        ax1.set_ylabel("Mean TTD (0..1)")
        ax1.set_ylim(-0.05, 1.05)
        ax1.legend(loc="lower right", fontsize=8)
        ax1.grid(True, alpha=0.3)
        if has_surv and n_panels >= 3:
            ax2 = axes[2]
            ax2.plot(gens, [float(x) for x in surv], "^-", color="coral", alpha=0.85, markersize=4, label="mean child survival (binary)")
            ax2.set_xlabel("Generation")
            ax2.set_ylabel("Mean survival")
            ax2.set_ylim(-0.05, 1.05)
            ax2.legend(loc="lower right", fontsize=8)
            ax2.grid(True, alpha=0.3)
        else:
            ax1.set_xlabel("Generation")
    else:
        # Legacy binary survival panel
        if has_surv:
            ax1.plot(gens, [float(x) for x in surv], "s-", color="coral", alpha=0.85, markersize=4, label="mean child survival (eval batch)")
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


def _parse_threats_schedule(spec: str | None) -> list[tuple[int, int]]:
    """Parse a "gen:threats,gen:threats,..." schedule.

    Returns a list of (start_generation, num_threats) tuples sorted ascending by
    start_generation. Active threat count for a generation g is the value from
    the entry with the largest start_generation <= g (stepwise / "last seen
    wins"). Anything before the first start_generation falls back to the CLI
    --threats default.

    Example: "0:0,500:1,1000:2" means threats=0 for [0..499], 1 for [500..999],
    2 for [1000..end].
    """
    if spec is None or not str(spec).strip():
        return []
    out: list[tuple[int, int]] = []
    raw = str(spec).replace(";", ",")
    for chunk in raw.split(","):
        chunk = chunk.strip()
        if not chunk:
            continue
        if ":" not in chunk:
            raise ValueError(
                f"--threats-schedule entry '{chunk}' must be 'gen:threats' (e.g. '500:1')"
            )
        g_str, n_str = chunk.split(":", 1)
        try:
            g = int(g_str.strip())
            n = int(n_str.strip())
        except ValueError as e:
            raise ValueError(
                f"--threats-schedule entry '{chunk}' must be integers 'gen:threats'"
            ) from e
        if g < 0 or n < 0:
            raise ValueError(
                f"--threats-schedule entry '{chunk}' must have non-negative gen and threats"
            )
        out.append((g, n))
    out.sort(key=lambda t: t[0])
    return out


def _threats_at_gen(
    gen_id: int,
    schedule: list[tuple[int, int]],
    default_n: int,
) -> int:
    """Return active threat count at gen_id given a schedule (stepwise / hold)."""
    if not schedule:
        return int(default_n)
    n = int(default_n)
    for start_g, count in schedule:
        if gen_id >= start_g:
            n = int(count)
        else:
            break
    return n


def _parse_args():
    p = argparse.ArgumentParser(description="Single-lineage evolution of motivation weights.")
    p.add_argument("--generations", type=int, default=DEFAULT_NUM_GENERATIONS, help="Number of generations.")
    p.add_argument("--episodes", type=int, default=DEFAULT_EPISODES_PER_EVAL, help="Episodes (seeds) per fitness evaluation.")
    p.add_argument("--sigma", type=float, default=DEFAULT_MUTATION_SIGMA, help="Gaussian mutation sigma for motivation weights.")
    p.add_argument("--seed-master", type=int, default=DEFAULT_SEED_MASTER, help="Master seed for deterministic evaluation batches.")
    p.add_argument("--max-ticks", type=int, default=DEFAULT_MAX_TICKS, help="Max ticks per episode.")
    p.add_argument("--grid-w", type=int, default=DEFAULT_GRID_W, help="Grid width.")
    p.add_argument("--grid-h", type=int, default=DEFAULT_GRID_H, help="Grid height.")
    p.add_argument("--threats", type=int, default=DEFAULT_NUM_THREATS, help="Number of threats (used when --threats-schedule is empty or before first scheduled generation).")
    p.add_argument(
        "--threats-schedule",
        type=str,
        default="",
        metavar="GEN:N[,GEN:N,...]",
        help=(
            "Stepwise schedule for the number of threats vs. generation. "
            "Format: 'gen:threats' pairs separated by commas (or semicolons). "
            "Example: '0:0,500:1,1000:2' = 0 threats from gen 0-499, 1 threat from gen 500-999, "
            "2 threats from gen 1000+. The schedule overrides --threats whenever an entry "
            "applies; before the first entry, --threats is used."
        ),
    )
    p.add_argument("--food-start", type=int, default=DEFAULT_FOOD_AT_START, help="Food placed at start.")
    p.add_argument("--food-spawn-interval", type=int, default=DEFAULT_FOOD_SPAWN_INTERVAL, help="Food spawn interval.")
    p.add_argument("--food-spawn-n", type=int, default=DEFAULT_FOOD_SPAWN_N, help="Food spawned each interval.")
    p.add_argument(
        "--fitness-mode",
        choices=["binary_child", "ttd_child", "ttd_overall"],
        default=DEFAULT_FITNESS_MODE,
        help="Fitness target. Default ttd_overall = weighted (child+mother) normalized time-to-death.",
    )
    p.add_argument(
        "--alpha-child",
        type=float,
        default=DEFAULT_ALPHA_CHILD,
        help="Weight on child in ttd_overall fitness (0..1). Example 0.7 = 70% child, 30% mother.",
    )
    p.add_argument(
        "--checkpoint-every",
        type=int,
        default=0,
        metavar="N",
        help="Write best-genome checkpoints every N generations into output_dir/checkpoints/. 0 = off.",
    )
    p.add_argument("--output-dir", type=str, default=DEFAULT_OUTPUT_DIR, help="Output directory under test_results/.")
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
    p.add_argument(
        "--plasticity",
        choices=["none", "outcome", "outcome_adaptive", "outcome_signed", "outcome_adaptive_signed"],
        default="none",
        help="Lifetime plasticity during each fitness episode (none = genes-only, default). E3 Baldwin: outcome_adaptive.",
    )
    p.add_argument(
        "--deficit-signal",
        choices=["global", "local"],
        default="global",
        help="Plasticity deficit for episodes when --plasticity is not none.",
    )
    p.add_argument(
        "--learn-w",
        choices=["on", "off"],
        default="off",
        help="Update psych weights w during episodes (off = only motivation u; recommended for E3).",
    )
    p.add_argument(
        "--update-mode",
        choices=["per_tick", "segment", "segment_capped"],
        default="per_tick",
        help="Plasticity credit assignment during episodes.",
    )
    p.add_argument(
        "--segment-kmax",
        type=int,
        default=20,
        help="For segment_capped: max ticks per motivation segment.",
    )
    args = p.parse_args()
    if args.watch_every < 0:
        p.error("--watch-every must be >= 0")
    try:
        args.threats_schedule_parsed = _parse_threats_schedule(args.threats_schedule)
    except ValueError as e:
        p.error(str(e))
    return args


if __name__ == "__main__":
    args = _parse_args()
    if args.viz_only:
        json_path = os.path.join(args.output_dir, "final_genome.json")
        launch_pygame_visualizer(json_path)
    else:
        cfg = _default_cfg()
        rule = None if args.plasticity == "none" else args.plasticity
        cfg.update(
            {
                "num_generations": int(args.generations),
                "episodes_per_eval": int(args.episodes),
                "mutation_sigma": float(args.sigma),
                "seed_master": int(args.seed_master),
                "max_ticks": int(args.max_ticks),
                "grid_w": int(args.grid_w),
                "grid_h": int(args.grid_h),
                "num_threats": int(args.threats),
                "threats_schedule": list(args.threats_schedule_parsed),
                "food_at_start": int(args.food_start),
                "food_spawn_interval": int(args.food_spawn_interval),
                "food_spawn_n": int(args.food_spawn_n),
                "output_dir": str(args.output_dir),
                "fitness_mode": str(args.fitness_mode),
                "alpha_child": float(args.alpha_child),
                "checkpoint_every": int(args.checkpoint_every),
                "plasticity_rule": rule,
                "plasticity_deficit_signal": str(args.deficit_signal),
                "plasticity_learn_w": args.learn_w == "on",
                "plasticity_update_mode": str(args.update_mode),
                "plasticity_segment_kmax": int(args.segment_kmax),
            }
        )
        main(cfg, open_viz_after=args.viz, watch_every=args.watch_every)
