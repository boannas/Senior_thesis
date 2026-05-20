"""
Single-lineage evolution of motivation weights (mother).

Darwinian setup:
- Genome = mother's fixed motivation weights U (nested dict matching MotherAgent).
- Psych weights W stay at the hard-coded defaults (genes-only evolution on U).
- Plasticity is OFF by default. For Baldwin / lifetime learning during evolution:
    --plasticity outcome_adaptive --deficit-signal local --learn-w off --update-mode segment_capped
- One lineage: propose Gaussian mutation on U; accept if mean fitness improves across seeds.

Initialization modes (--init-mode):
- baseline_zero   : all 13 weights = 0.5 (legacy default; mother starts moderately functional)
- random_uniform  : every weight ~ U(init_low, init_high) [reproducible via --init-seed]
- anti_maternal   : Care/Protect weights LOW, Self/Forage weights HIGH (de novo emergence test)
- pro_maternal    : Care/Protect weights HIGH (control: starts already maternal)
- from_json       : load genome from --init-from PATH

Reproducibility:
- All RNGs are derived from --seed-master (and --init-seed for genome init only).
- The full run is captured in run_config.json (CLI, cfg, python, git hash) and init_genome.json.
- --checkpoint-every N saves best-genome snapshots to checkpoints/ for later non-learner tests.

Crash safety / resume:
- lineage_generations.csv is written incrementally (one row per generation, fsync'd).
- After every generation latest_state.json is overwritten atomically with the
  best genome, best fitness, current generation index, and the numpy RNG state.
- Re-run with --resume (and the same --output-dir) to continue an interrupted
  run byte-identically from the last completed generation. If --resume is set
  but no latest_state.json exists, the run starts fresh with a warning.

Logging (per generation, to lineage_generations.csv):
- Fitness, child/mother TTD, child survival, child injury
- Plasticity diagnostics (u_drift, du_plastic, lr_eff, plastic_active_frac, peak_u_drift)
- Behavior fractions: frac_forage, frac_care, frac_self, frac_protect, maternal_fraction
- Candidate genome weights flattened: u_forage_child_hunger, ..., u_protect_bonding
- Active number of threats (supports --threats-schedule for non-stationary runs)

Usage:
  python run_evolve_lineage.py
  python run_evolve_lineage.py --plasticity outcome_adaptive --deficit-signal global --learn-w off \\
      --update-mode per_tick --episodes 10 --output-dir results/E3_baldwin
  python run_evolve_lineage.py --init-mode anti_maternal --init-seed 7 --init-noise 0.05
  python run_evolve_lineage.py --viz              # after evolution, open Pygame (blocks until window closed)
  python run_evolve_lineage.py --viz-only         # only open Pygame with existing final_genome.json
  python run_evolve_lineage.py --watch-every 1    # pause after each generation: Pygame shows current best

Outputs (in --output-dir):
  lineage_generations.csv
  lineage_plot.png            (if matplotlib available)
  init_genome.json            (gen-0 starting genome, for reproducibility)
  final_genome.json           (best genome found; for Pygame: --genome)
  final_genome.txt            (human-readable summary)
  run_config.json             (CLI args, cfg, python version, best-effort git hash)
  latest_state.json           (resume state: best_genome + RNG, refreshed every generation)
  checkpoints/best_genXXXX.json   (if --checkpoint-every > 0)

Pygame after a run:
  python -m core.ui.pygame_viewer --genome <output_dir>/final_genome.json
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


# Min/max for the genome clamp window (matches mutate_genome's clamp).
_GENOME_LO = 0.05
_GENOME_HI = 1.0


def _empty_motivation_genome() -> dict:
    """An empty (all-NaN-equivalent) motivation skeleton used for initialization templates."""
    return {
        "forage": {"child_hunger": 0.0, "energy_deficit": 0.0, "low_fear": 0.0},
        "care": {"child_warmth": 0.0, "closeness_deficit": 0.0, "bonding": 0.0},
        "self": {"fatigue": 0.0, "fear": 0.0, "stress": 0.0},
        "protect": {
            "child_injury": 0.0,
            "fear": 0.0,
            "closeness_deficit": 0.0,
            "bonding": 0.0,
        },
    }


def _clamp_genome(g: dict, lo: float = _GENOME_LO, hi: float = _GENOME_HI) -> dict:
    """Clamp every leaf weight to [lo, hi]."""
    out = copy.deepcopy(g)
    for path, v in _flatten_keys(out):
        _set_path(out, path, max(lo, min(hi, float(v))))
    return out


def _add_noise_to_genome(g: dict, rng: np.random.RandomState, sigma: float) -> dict:
    """Add Gaussian N(0, sigma) noise to each weight, then clamp."""
    if sigma <= 0.0:
        return copy.deepcopy(g)
    out = copy.deepcopy(g)
    for path, v in _flatten_keys(out):
        _set_path(out, path, float(v) + float(rng.normal(0.0, sigma)))
    return _clamp_genome(out)


def random_uniform_motivation_genome(
    rng: np.random.RandomState,
    low: float = _GENOME_LO,
    high: float = _GENOME_HI,
) -> dict:
    """Every motivation weight is independently sampled from U(low, high)."""
    g = _empty_motivation_genome()
    for path, _ in _flatten_keys(g):
        _set_path(g, path, float(rng.uniform(low, high)))
    return g


def anti_maternal_motivation_genome() -> dict:
    """Mother starts non-maternal: low Care/Protect, high Self/Forage-self.

    Use to test *de novo* emergence of maternal behavior under selection
    (the question of interest is whether evolution + plasticity moves the
    lineage toward a maternal phenotype starting from an actively
    self-prioritizing prior).
    """
    return {
        "forage": {"child_hunger": 0.10, "energy_deficit": 0.90, "low_fear": 0.50},
        "care": {"child_warmth": 0.10, "closeness_deficit": 0.10, "bonding": 0.10},
        "self": {"fatigue": 0.90, "fear": 0.90, "stress": 0.90},
        "protect": {
            "child_injury": 0.10,
            "fear": 0.10,
            "closeness_deficit": 0.10,
            "bonding": 0.10,
        },
    }


def pro_maternal_motivation_genome() -> dict:
    """Mother starts already maternal: high Care/Protect, moderate Self.

    Useful as a CONTROL for the anti_maternal condition: if both reach the
    same asymptote, that is evidence the model converges to the maternal
    attractor regardless of initial state.
    """
    return {
        "forage": {"child_hunger": 0.90, "energy_deficit": 0.50, "low_fear": 0.50},
        "care": {"child_warmth": 0.90, "closeness_deficit": 0.90, "bonding": 0.90},
        "self": {"fatigue": 0.30, "fear": 0.30, "stress": 0.30},
        "protect": {
            "child_injury": 0.90,
            "fear": 0.50,
            "closeness_deficit": 0.90,
            "bonding": 0.90,
        },
    }


def _load_genome_json(path: str) -> dict:
    """Load a motivation genome from a JSON file. Validates against the canonical shape."""
    with open(path, encoding="utf-8") as f:
        data = json.load(f)
    if not isinstance(data, dict):
        raise ValueError(f"--init-from JSON must be a dict, got {type(data).__name__}")
    template = baseline_zero_motivation_genome()
    out = _empty_motivation_genome()
    for cat, leaves in template.items():
        if cat not in data or not isinstance(data[cat], dict):
            raise ValueError(f"--init-from JSON missing category '{cat}'")
        for k in leaves.keys():
            if k not in data[cat]:
                raise ValueError(f"--init-from JSON missing key '{cat}.{k}'")
            out[cat][k] = float(data[cat][k])
    return _clamp_genome(out)


def initialize_genome(
    init_mode: str,
    init_seed: int,
    init_noise: float = 0.0,
    init_from: str | None = None,
    init_low: float = _GENOME_LO,
    init_high: float = _GENOME_HI,
) -> dict:
    """Build the gen-0 genome based on --init-mode and friends.

    Parameters
    ----------
    init_mode : one of {"baseline_zero", "random_uniform", "anti_maternal",
                         "pro_maternal", "from_json"}
    init_seed : RNG seed used only by random_uniform and by the init_noise
                jitter on the deterministic templates. Independent of seed_master
                so the user can vary init seed across replicates without breaking
                evaluation reproducibility.
    init_noise : Gaussian sigma added to every weight after the template is
                 chosen (no-op if 0.0). Only applied to template-based modes
                 (baseline_zero / anti_maternal / pro_maternal / from_json).
    init_from : JSON path used iff init_mode == "from_json".
    init_low / init_high : range for random_uniform mode.
    """
    rng = np.random.RandomState(int(init_seed))
    mode = str(init_mode).lower()

    if mode == "baseline_zero":
        g = baseline_zero_motivation_genome()
    elif mode == "random_uniform":
        g = random_uniform_motivation_genome(rng, low=float(init_low), high=float(init_high))
    elif mode == "anti_maternal":
        g = anti_maternal_motivation_genome()
    elif mode == "pro_maternal":
        g = pro_maternal_motivation_genome()
    elif mode == "from_json":
        if not init_from:
            raise ValueError("--init-mode from_json requires --init-from PATH")
        g = _load_genome_json(str(init_from))
    else:
        raise ValueError(f"Unknown --init-mode: {init_mode!r}")

    if init_noise and init_noise > 0.0 and mode != "random_uniform":
        g = _add_noise_to_genome(g, rng, float(init_noise))

    return _clamp_genome(g)


# Stable, deterministic order for genome leaves (used to flatten u_fix into CSV columns).
_GENOME_LEAVES_ORDER: list[tuple[str, str]] = [
    ("forage", "child_hunger"),
    ("forage", "energy_deficit"),
    ("forage", "low_fear"),
    ("care", "child_warmth"),
    ("care", "closeness_deficit"),
    ("care", "bonding"),
    ("self", "fatigue"),
    ("self", "fear"),
    ("self", "stress"),
    ("protect", "child_injury"),
    ("protect", "fear"),
    ("protect", "closeness_deficit"),
    ("protect", "bonding"),
]


def _genome_to_flat_columns(g: dict) -> dict:
    """Flatten a motivation genome dict to {'u_forage_child_hunger': 0.5, ...}."""
    out: dict[str, float] = {}
    for cat, leaf in _GENOME_LEAVES_ORDER:
        try:
            out[f"u_{cat}_{leaf}"] = float(g[cat][leaf])
        except (KeyError, TypeError, ValueError):
            out[f"u_{cat}_{leaf}"] = float("nan")
    return out


_GENOME_FLAT_COLUMN_NAMES: list[str] = [f"u_{c}_{k}" for c, k in _GENOME_LEAVES_ORDER]


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


def _flat_leaf_values(d: dict) -> list[float]:
    """Flatten a nested dict-of-dicts-of-floats to a deterministic list of leaf floats.

    Used to snapshot motivation_weights_plastic cheaply each tick (no deepcopy)
    and to compute L1 / Linf differences between consecutive snapshots.
    """
    out: list[float] = []
    for k in sorted(d.keys()):
        v = d[k]
        if isinstance(v, dict):
            out.extend(_flat_leaf_values(v))
        else:
            try:
                out.append(float(v))
            except (TypeError, ValueError):
                pass
    return out


def _l1_diff_flat(a: list[float], b: list[float]) -> float:
    """Sum of |a_i - b_i|, robust to length mismatch (uses min length)."""
    n = min(len(a), len(b))
    if n == 0:
        return 0.0
    s = 0.0
    for i in range(n):
        s += abs(a[i] - b[i])
    return s


_LR_DEFAULT_BASE = 0.02
_LR_DEFAULT_MIN = 0.002
_LR_DEFAULT_MAX = 0.05
_LR_DEFAULT_DEFICIT_SCALE = 2.0


def _resolve_lr_overrides(cfg: dict) -> dict | None:
    """Compute the resolved LR overrides (or None if nothing should be overridden).

    Returns a dict with keys: base, lr_min, lr_max, deficit_scale (any subset that was
    actually requested). If the user didn't pass any --lr* flags, returns None.
    """
    base = cfg.get("lr_base", None)
    lr_min = cfg.get("lr_min", None)
    lr_max = cfg.get("lr_max", None)
    deficit_scale = cfg.get("lr_deficit_scale", None)
    scale = cfg.get("lr_scale", None)

    if base is None and lr_min is None and lr_max is None and deficit_scale is None and scale is None:
        return None

    eff_base = float(base) if base is not None else _LR_DEFAULT_BASE
    eff_min = float(lr_min) if lr_min is not None else _LR_DEFAULT_MIN
    eff_max = float(lr_max) if lr_max is not None else _LR_DEFAULT_MAX
    eff_dscale = float(deficit_scale) if deficit_scale is not None else _LR_DEFAULT_DEFICIT_SCALE

    if scale is not None:
        s = float(scale)
        eff_base *= s
        eff_min *= s
        eff_max *= s

    return {
        "base": eff_base,
        "lr_min": eff_min,
        "lr_max": eff_max,
        "deficit_scale": eff_dscale,
    }


def _apply_lr_overrides_to_mothers(mothers, cfg: dict) -> None:
    """Apply --lr* overrides to every mother in the world (no-op if not requested)."""
    overrides = _resolve_lr_overrides(cfg)
    if overrides is None:
        return
    for m in mothers:
        m.learning_rate = float(overrides["base"])
        m.learning_rate_min = float(overrides["lr_min"])
        m.learning_rate_max = float(overrides["lr_max"])
        m.learning_deficit_scale = float(overrides["deficit_scale"])


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

    _apply_lr_overrides_to_mothers(world.mothers, cfg)

    mother = world.mothers[0]
    child = world.children[0]
    mot_counts = {"Forage": 0, "Care": 0, "Self": 0, "Protect": 0}

    child_death_tick = None
    mother_death_tick = None
    overall_deficit_samples: list[float] = []
    local_deficit_samples: list[float] = []
    # Plastic-activity diagnostics (Baldwin "panic and learn, then assimilate")
    du_plastic_samples: list[float] = []
    lr_eff_samples: list[float] = []
    u_drift_samples: list[float] = []
    plastic_active_ticks = 0

    plasticity_on = mother is not None and hasattr(mother, "motivation_weights_plastic")
    prev_flat: list[float] | None = (
        _flat_leaf_values(mother.motivation_weights_plastic) if plasticity_on else None
    )

    for tick in range(max_ticks):
        world.step()
        if mother and mother.is_alive():
            overall_deficit_samples.append(float(compute_overall_deficit(mother)))
            sel = getattr(mother, "selected_motivation", None)
            local_deficit_samples.append(float(compute_local_deficit(mother, sel, world)))
            if sel in mot_counts:
                mot_counts[sel] += 1

            if plasticity_on:
                cur_flat = _flat_leaf_values(mother.motivation_weights_plastic)
                if prev_flat is not None:
                    du = _l1_diff_flat(cur_flat, prev_flat)
                    du_plastic_samples.append(du)
                    if du > 0.0:
                        plastic_active_ticks += 1
                prev_flat = cur_flat

                lr_eff_attr = getattr(mother, "_last_learning_rate_eff", None)
                if lr_eff_attr is not None:
                    try:
                        lr_eff_samples.append(float(lr_eff_attr))
                    except (TypeError, ValueError):
                        pass

                if hasattr(mother, "motivation_weights_fixed"):
                    u_drift_samples.append(
                        _mean_abs_diff_nested(
                            mother.motivation_weights_fixed,
                            mother.motivation_weights_plastic,
                        )
                    )

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

    def _safe_max(xs: list[float]) -> float:
        finite = [x for x in xs if np.isfinite(x)]
        return float(max(finite)) if finite else float("nan")

    n_ticks_lived = max(1, len(du_plastic_samples))
    plastic_active_frac = (
        float(plastic_active_ticks) / float(n_ticks_lived)
        if du_plastic_samples
        else float("nan")
    )

    # Behavior fractions: how much time (alive ticks) the mother spent in each motivation.
    total_mot_ticks = max(1, sum(int(v) for v in mot_counts.values()))
    frac_forage = float(mot_counts.get("Forage", 0)) / float(total_mot_ticks)
    frac_care = float(mot_counts.get("Care", 0)) / float(total_mot_ticks)
    frac_self = float(mot_counts.get("Self", 0)) / float(total_mot_ticks)
    frac_protect = float(mot_counts.get("Protect", 0)) / float(total_mot_ticks)
    maternal_fraction = frac_care + frac_protect

    return {
        "child_survival": 1.0 if child_alive else 0.0,
        "child_injury": injury,
        "child_death_tick": int(child_death_tick),
        "mother_death_tick": int(mother_death_tick),
        "child_ttd_norm": float(child_ttd_norm),
        "mother_ttd_norm": float(mother_ttd_norm),
        "mot_counts": mot_counts,
        "frac_forage": frac_forage,
        "frac_care": frac_care,
        "frac_self": frac_self,
        "frac_protect": frac_protect,
        "maternal_fraction": maternal_fraction,
        "u_drift_end": float(u_drift_end),
        "mean_overall_deficit_episode": _mean_last(overall_deficit_samples),
        "overall_deficit_end": _last(overall_deficit_samples),
        "mean_local_deficit_episode": _mean_last(local_deficit_samples),
        "local_deficit_end": _last(local_deficit_samples),
        # Baldwin-effect plastic-activity diagnostics:
        "mean_du_plastic_episode": _mean_last(du_plastic_samples),
        "peak_du_plastic_episode": _safe_max(du_plastic_samples),
        "plastic_active_frac": plastic_active_frac,
        "mean_lr_eff_episode": _mean_last(lr_eff_samples),
        "peak_u_drift_episode": _safe_max(u_drift_samples),
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
    # Plastic-activity diagnostics (per-episode samples)
    du_plast_m: list[float] = []
    du_plast_p: list[float] = []
    plast_active_f: list[float] = []
    lr_eff_m: list[float] = []
    u_drift_peak: list[float] = []
    # Behavior fraction samples (per episode), used for emergence metrics
    frac_for: list[float] = []
    frac_car: list[float] = []
    frac_slf: list[float] = []
    frac_pro: list[float] = []
    matern: list[float] = []
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
        du_plast_m.append(float(r.get("mean_du_plastic_episode", float("nan"))))
        du_plast_p.append(float(r.get("peak_du_plastic_episode", float("nan"))))
        plast_active_f.append(float(r.get("plastic_active_frac", float("nan"))))
        lr_eff_m.append(float(r.get("mean_lr_eff_episode", float("nan"))))
        u_drift_peak.append(float(r.get("peak_u_drift_episode", float("nan"))))
        frac_for.append(float(r.get("frac_forage", float("nan"))))
        frac_car.append(float(r.get("frac_care", float("nan"))))
        frac_slf.append(float(r.get("frac_self", float("nan"))))
        frac_pro.append(float(r.get("frac_protect", float("nan"))))
        matern.append(float(r.get("maternal_fraction", float("nan"))))
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
        # Baldwin plastic-activity diagnostics (mean/std over evaluation seeds)
        "mean_du_plastic_episode": float(np.nanmean(du_plast_m)) if du_plast_m else float("nan"),
        "std_du_plastic_episode": float(np.nanstd(du_plast_m, ddof=0)) if du_plast_m else float("nan"),
        "mean_peak_du_plastic_episode": float(np.nanmean(du_plast_p)) if du_plast_p else float("nan"),
        "std_peak_du_plastic_episode": float(np.nanstd(du_plast_p, ddof=0)) if du_plast_p else float("nan"),
        "mean_plastic_active_frac": float(np.nanmean(plast_active_f)) if plast_active_f else float("nan"),
        "std_plastic_active_frac": float(np.nanstd(plast_active_f, ddof=0)) if plast_active_f else float("nan"),
        "mean_lr_eff_episode": float(np.nanmean(lr_eff_m)) if lr_eff_m else float("nan"),
        "std_lr_eff_episode": float(np.nanstd(lr_eff_m, ddof=0)) if lr_eff_m else float("nan"),
        "mean_peak_u_drift_episode": float(np.nanmean(u_drift_peak)) if u_drift_peak else float("nan"),
        "std_peak_u_drift_episode": float(np.nanstd(u_drift_peak, ddof=0)) if u_drift_peak else float("nan"),
        # Behavior fractions averaged over evaluation seeds (emergence metrics)
        "frac_forage": float(np.nanmean(frac_for)) if frac_for else float("nan"),
        "frac_care": float(np.nanmean(frac_car)) if frac_car else float("nan"),
        "frac_self": float(np.nanmean(frac_slf)) if frac_slf else float("nan"),
        "frac_protect": float(np.nanmean(frac_pro)) if frac_pro else float("nan"),
        "maternal_fraction": float(np.nanmean(matern)) if matern else float("nan"),
        "std_maternal_fraction": float(np.nanstd(matern, ddof=0)) if matern else float("nan"),
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
        "lr_base": None,
        "lr_min": None,
        "lr_max": None,
        "lr_deficit_scale": None,
        "lr_scale": None,
        "fitness_mode": DEFAULT_FITNESS_MODE,
        "alpha_child": DEFAULT_ALPHA_CHILD,
        "output_dir": DEFAULT_OUTPUT_DIR,
        "init_mode": "baseline_zero",
        "init_seed": DEFAULT_SEED_MASTER,
        "init_noise": 0.0,
        "init_from": None,
        "init_low": _GENOME_LO,
        "init_high": _GENOME_HI,
        "checkpoint_every": 0,
        "resume": False,
        "threats_schedule": [],
    }


def _git_commit_short() -> str | None:
    """Best-effort: return short git hash, or None if not available."""
    try:
        out = subprocess.check_output(
            ["git", "rev-parse", "--short", "HEAD"],
            cwd=_repo_root(),
            stderr=subprocess.DEVNULL,
        ).decode().strip()
        return out or None
    except (subprocess.CalledProcessError, OSError, FileNotFoundError):
        return None


def _write_run_config(output_dir: str, cfg: dict, init_genome: dict) -> None:
    """Write run_config.json + init_genome.json + cli_args.txt in the output dir.

    These files together let any future reader reproduce the run exactly:
      - run_config.json : full cfg, CLI argv, python version, git hash, timestamp
      - init_genome.json : the gen-0 genome the run started from
      - cli_args.txt    : the literal command line that launched this run
    """
    os.makedirs(output_dir, exist_ok=True)

    # 1) run_config.json
    payload = {
        "timestamp": datetime.now().isoformat(),
        "python_version": sys.version,
        "git_commit": _git_commit_short(),
        "cli_argv": list(sys.argv),
        "cfg": copy.deepcopy(cfg),
    }
    cfg_path = os.path.join(output_dir, "run_config.json")
    try:
        with open(cfg_path, "w", encoding="utf-8") as f:
            json.dump(payload, f, indent=2, default=str)
        print(f"Wrote {cfg_path}")
    except OSError as e:
        print(f"[warn] could not write {cfg_path}: {e}", file=sys.stderr)

    # 2) init_genome.json
    init_path = os.path.join(output_dir, "init_genome.json")
    try:
        with open(init_path, "w", encoding="utf-8") as f:
            json.dump(init_genome, f, indent=2)
        print(f"Wrote {init_path}")
    except OSError as e:
        print(f"[warn] could not write {init_path}: {e}", file=sys.stderr)

    # 3) cli_args.txt
    txt_path = os.path.join(output_dir, "cli_args.txt")
    try:
        with open(txt_path, "w", encoding="utf-8") as f:
            f.write(" ".join(sys.argv))
            f.write("\n")
    except OSError as e:
        print(f"[warn] could not write {txt_path}: {e}", file=sys.stderr)


# --------------------------------------------------------------------------
# Crash-safe persistence helpers (incremental CSV + resumable state).
#
# After every generation we (a) append one row to lineage_generations.csv and
# (b) overwrite latest_state.json atomically.  If power dies, latest_state.json
# is the source of truth: it tells us the last fully-completed generation, the
# current best genome, and the RNG state needed to reproduce future mutations
# exactly.  --resume reads it back and continues from gen+1.
# --------------------------------------------------------------------------


def _serialize_rng_state(rng: np.random.RandomState) -> dict:
    """Serialize a legacy MT19937 RandomState to a JSON-friendly dict."""
    name, arr, pos, has_gauss, cached_gauss = rng.get_state()
    return {
        "name": str(name),
        "arr": np.asarray(arr, dtype=np.uint32).tolist(),
        "pos": int(pos),
        "has_gauss": int(has_gauss),
        "cached_gauss": float(cached_gauss),
    }


def _deserialize_rng_state(d: dict) -> tuple:
    return (
        str(d["name"]),
        np.array(d["arr"], dtype=np.uint32),
        int(d["pos"]),
        int(d["has_gauss"]),
        float(d["cached_gauss"]),
    )


def _atomic_write_json(path: str, payload: dict) -> None:
    """Write JSON atomically (write to .tmp, fsync, os.replace)."""
    tmp = path + ".tmp"
    with open(tmp, "w", encoding="utf-8") as f:
        json.dump(payload, f, default=str)
        f.flush()
        try:
            os.fsync(f.fileno())
        except OSError:
            pass
    os.replace(tmp, path)


def _save_latest_state(
    output_dir: str,
    *,
    last_completed_gen: int,
    best_gen: int,
    best_fit: float,
    best_genome: dict,
    rng: np.random.RandomState,
    cfg: dict,
) -> None:
    """Atomically persist enough state to resume the run byte-identically."""
    payload = {
        "last_completed_gen": int(last_completed_gen),
        "best_gen": int(best_gen),
        "best_fit": float(best_fit),
        "best_genome": best_genome,
        "rng_state": _serialize_rng_state(rng),
        "current_num_threats": int(cfg.get("num_threats", 0)),
        "timestamp": datetime.now().isoformat(),
    }
    _atomic_write_json(os.path.join(output_dir, "latest_state.json"), payload)


def _load_latest_state(output_dir: str) -> dict | None:
    """Load latest_state.json, or return None if it's missing.

    Raises RuntimeError if the file exists but is corrupt - we never want to
    silently clobber an interrupted run's CSV/state, the user should
    investigate and either delete latest_state.json or fix it manually.
    """
    p = os.path.join(output_dir, "latest_state.json")
    if not os.path.exists(p):
        return None
    try:
        with open(p, encoding="utf-8") as f:
            return json.load(f)
    except (OSError, json.JSONDecodeError) as e:
        raise RuntimeError(
            f"latest_state.json exists at {p} but could not be parsed: {e}. "
            f"Refusing to clobber the run; delete the file by hand if you want "
            f"to start fresh."
        ) from e


def _csv_init(csv_path: str, fieldnames: list[str]) -> None:
    """(Re)create CSV with a header row only."""
    with open(csv_path, "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=fieldnames)
        w.writeheader()
        f.flush()
        try:
            os.fsync(f.fileno())
        except OSError:
            pass


def _csv_append(csv_path: str, fieldnames: list[str], row: dict) -> None:
    """Append a single row and flush+fsync so a kill/power-loss won't lose it."""
    with open(csv_path, "a", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=fieldnames)
        w.writerow(row)
        f.flush()
        try:
            os.fsync(f.fileno())
        except OSError:
            pass


def _read_existing_rows(csv_path: str) -> list[dict]:
    """Read previously-written CSV rows back as plain dicts (string values).

    Light type-coercion is done where downstream code expects native types
    (e.g. ``accepted`` must be a real bool because ``bool('False') == True``).
    """
    if not os.path.exists(csv_path):
        return []
    with open(csv_path, encoding="utf-8") as f:
        rows = list(csv.DictReader(f))
    for r in rows:
        v = r.get("accepted")
        if isinstance(v, str):
            r["accepted"] = v.strip().lower() in ("true", "1", "yes")
    return rows


def main(cfg: dict, open_viz_after: bool = False, watch_every: int = 0):
    output_dir = str(cfg["output_dir"])
    os.makedirs(output_dir, exist_ok=True)

    threats_schedule: list[tuple[int, int]] = list(cfg.get("threats_schedule", []) or [])
    base_threats = int(cfg.get("num_threats", DEFAULT_NUM_THREATS))
    if threats_schedule:
        print(
            f"[threats schedule] base={base_threats}  steps="
            + ", ".join(f"gen{g}->{n}" for g, n in threats_schedule)
        )

    lr_overrides = _resolve_lr_overrides(cfg)
    if lr_overrides is not None:
        print(
            f"[learning-rate override] base={lr_overrides['base']:.6g} "
            f"lr_min={lr_overrides['lr_min']:.6g} "
            f"lr_max={lr_overrides['lr_max']:.6g} "
            f"deficit_scale={lr_overrides['deficit_scale']:.6g}"
            + (f"  (scale={cfg.get('lr_scale')})" if cfg.get("lr_scale") is not None else "")
        )
    else:
        print(
            f"[learning-rate] defaults: base={_LR_DEFAULT_BASE} "
            f"lr_min={_LR_DEFAULT_MIN} lr_max={_LR_DEFAULT_MAX} "
            f"deficit_scale={_LR_DEFAULT_DEFICIT_SCALE}"
        )

    def _build_row(
        gen_id_: int,
        fit_: float,
        stats_: dict,
        accepted_: bool,
        active_threats_: int,
        evaluated_genome_: dict,
        mutation_sigma_: float,
    ) -> dict:
        """Assemble one CSV row for a generation, incl. behavior fractions and flat genome."""
        row = {
            "generation": gen_id_,
            "fitness": fit_,
            "accepted": accepted_,
            "num_threats_active": active_threats_,
            "mean_child_survival": stats_["mean_child_survival"],
            "mean_child_ttd_norm": stats_.get("mean_child_ttd_norm", np.nan),
            "mean_mother_ttd_norm": stats_.get("mean_mother_ttd_norm", np.nan),
            "mean_child_injury": stats_["mean_child_injury"],
            "std_child_survival": stats_["std_child_survival"],
            "mean_u_drift_end": stats_.get("mean_u_drift_end", np.nan),
            "std_u_drift_end": stats_.get("std_u_drift_end", np.nan),
            "mean_overall_deficit_episode": stats_.get("mean_overall_deficit_episode", np.nan),
            "std_overall_deficit_episode": stats_.get("std_overall_deficit_episode", np.nan),
            "mean_overall_deficit_end": stats_.get("mean_overall_deficit_end", np.nan),
            "std_overall_deficit_end": stats_.get("std_overall_deficit_end", np.nan),
            "mean_local_deficit_episode": stats_.get("mean_local_deficit_episode", np.nan),
            "std_local_deficit_episode": stats_.get("std_local_deficit_episode", np.nan),
            "mean_local_deficit_end": stats_.get("mean_local_deficit_end", np.nan),
            "std_local_deficit_end": stats_.get("std_local_deficit_end", np.nan),
            "mean_du_plastic_episode": stats_.get("mean_du_plastic_episode", np.nan),
            "std_du_plastic_episode": stats_.get("std_du_plastic_episode", np.nan),
            "mean_peak_du_plastic_episode": stats_.get("mean_peak_du_plastic_episode", np.nan),
            "std_peak_du_plastic_episode": stats_.get("std_peak_du_plastic_episode", np.nan),
            "mean_plastic_active_frac": stats_.get("mean_plastic_active_frac", np.nan),
            "std_plastic_active_frac": stats_.get("std_plastic_active_frac", np.nan),
            "mean_lr_eff_episode": stats_.get("mean_lr_eff_episode", np.nan),
            "std_lr_eff_episode": stats_.get("std_lr_eff_episode", np.nan),
            "mean_peak_u_drift_episode": stats_.get("mean_peak_u_drift_episode", np.nan),
            "std_peak_u_drift_episode": stats_.get("std_peak_u_drift_episode", np.nan),
            # Behavior fractions (emergence metrics)
            "frac_forage": stats_.get("frac_forage", np.nan),
            "frac_care": stats_.get("frac_care", np.nan),
            "frac_self": stats_.get("frac_self", np.nan),
            "frac_protect": stats_.get("frac_protect", np.nan),
            "maternal_fraction": stats_.get("maternal_fraction", np.nan),
            "std_maternal_fraction": stats_.get("std_maternal_fraction", np.nan),
            "mutation_sigma": float(mutation_sigma_),
        }
        # Flat genome columns (u_<motivation>_<input>) for the candidate evaluated this row.
        row.update(_genome_to_flat_columns(evaluated_genome_))
        return row

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

    csv_path = os.path.join(output_dir, "lineage_generations.csv")
    checkpoint_every = int(cfg.get("checkpoint_every", 0) or 0)

    # ----------------------------------------------------------------
    # Resume vs fresh-start branching.
    # ----------------------------------------------------------------
    resume_requested = bool(cfg.get("resume", False))
    state = _load_latest_state(output_dir) if resume_requested else None

    if state is not None:
        # ------------------- RESUME branch -------------------
        rng = np.random.RandomState(0)
        rng.set_state(_deserialize_rng_state(state["rng_state"]))

        best_genome = copy.deepcopy(state["best_genome"])
        best_fit = float(state["best_fit"])
        best_gen = int(state["best_gen"])
        last_completed_gen = int(state["last_completed_gen"])

        print(
            f"[resume] loaded latest_state.json from {output_dir}\n"
            f"[resume] last_completed_gen={last_completed_gen}  "
            f"best_fit={best_fit:.4f} (gen {best_gen})"
        )

        rows = _read_existing_rows(csv_path)
        if not rows:
            raise RuntimeError(
                f"latest_state.json says gen {last_completed_gen} but "
                f"{csv_path} is missing/empty. Aborting."
            )
        fieldnames = list(rows[0].keys())
        # If a power loss happened between csv-flush and state-flush we may
        # have one extra row in the CSV.  Trim back so the two agree.
        if len(rows) > last_completed_gen + 1:
            print(
                f"[resume] CSV has {len(rows)} rows but state says "
                f"{last_completed_gen + 1}; trimming."
            )
            rows = rows[: last_completed_gen + 1]
            _csv_init(csv_path, fieldnames)
            for r in rows:
                _csv_append(csv_path, fieldnames, r)

        cfg["num_threats"] = _threats_at_gen(last_completed_gen, threats_schedule, base_threats)
        start_gen = last_completed_gen + 1
        gen_id = last_completed_gen
        if start_gen > int(cfg["num_generations"]):
            print(
                f"[resume] target generations={cfg['num_generations']} already "
                f"reached; finalising without further evolution."
            )

    else:
        # ------------------- FRESH-START branch -------------------
        if resume_requested:
            print(
                "[resume] --resume set but no latest_state.json found; starting fresh.",
                file=sys.stderr,
            )

        rng = np.random.RandomState(int(cfg["seed_master"]))

        init_mode = str(cfg.get("init_mode", "baseline_zero"))
        init_seed = int(cfg.get("init_seed", cfg.get("seed_master", DEFAULT_SEED_MASTER)))
        init_noise = float(cfg.get("init_noise", 0.0) or 0.0)
        init_from = cfg.get("init_from", None)
        init_low = float(cfg.get("init_low", _GENOME_LO))
        init_high = float(cfg.get("init_high", _GENOME_HI))

        genome = initialize_genome(
            init_mode=init_mode,
            init_seed=init_seed,
            init_noise=init_noise,
            init_from=init_from,
            init_low=init_low,
            init_high=init_high,
        )
        print(
            f"[init] mode={init_mode}  init_seed={init_seed}  init_noise={init_noise}"
            + (f"  init_from={init_from}" if init_from else "")
        )

        # Reproducibility artifacts (only on fresh start - preserved on resume).
        _write_run_config(output_dir, cfg, genome)

        gen_id = 0
        cfg["num_threats"] = _threats_at_gen(gen_id, threats_schedule, base_threats)
        active_threats_now = int(cfg["num_threats"])
        seeds0 = episode_seeds(0, seed_master=int(cfg["seed_master"]), episodes_per_eval=int(cfg["episodes_per_eval"]))
        fit, stats0 = evaluate_genome(genome, seeds0, rng, cfg)
        rows = [_build_row(gen_id, fit, stats0, True, active_threats_now, genome, 0.0)]
        print(
            f"[gen {gen_id}] fitness={fit:.4f} mean_surv={stats0['mean_child_survival']:.3f} "
            f"maternal={stats0.get('maternal_fraction', float('nan')):.3f}"
        )

        best_genome = copy.deepcopy(genome)
        best_fit = fit
        best_gen = 0

        # Initial CSV header + gen-0 row + initial state, all flushed to disk.
        fieldnames = list(rows[0].keys())
        _csv_init(csv_path, fieldnames)
        _csv_append(csv_path, fieldnames, rows[0])
        _save_latest_state(
            output_dir,
            last_completed_gen=0,
            best_gen=0,
            best_fit=best_fit,
            best_genome=best_genome,
            rng=rng,
            cfg=cfg,
        )

        if watch_every > 0 and gen_id % watch_every == 0:
            ep = build_watch_episode_payload(seeds0[0], best_genome, cfg)
            ep_path = _write_watch_episode_file(ep, output_dir)
            launch_pygame_episode(
                ep_path,
                title=f"Evolve watch — generation {gen_id} (close window to continue)",
            )

        if checkpoint_every > 0 and gen_id % checkpoint_every == 0:
            _write_checkpoint(gen_id, best_fit, best_genome)

        start_gen = 1

    for gen_id in range(start_gen, int(cfg["num_generations"]) + 1):
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
                f"[gen {gen_id}] ACCEPT fitness={fit_c:.4f} "
                f"mean_surv={stats_c['mean_child_survival']:.3f} "
                f"maternal={stats_c.get('maternal_fraction', float('nan')):.3f}"
            )
        else:
            print(
                f"[gen {gen_id}] reject fitness={fit_c:.4f} (best={best_fit:.4f})"
            )

        new_row = _build_row(
            gen_id_=gen_id,
            fit_=fit_c,
            stats_=stats_c,
            accepted_=accepted,
            active_threats_=active_threats_now,
            evaluated_genome_=candidate,
            mutation_sigma_=float(cfg["mutation_sigma"]),
        )
        rows.append(new_row)

        # Persist this generation atomically: append CSV row first, then
        # overwrite latest_state.json. On power loss between the two writes
        # the resume logic detects the extra CSV row and trims it back.
        _csv_append(csv_path, fieldnames, new_row)
        _save_latest_state(
            output_dir,
            last_completed_gen=gen_id,
            best_gen=best_gen,
            best_fit=best_fit,
            best_genome=best_genome,
            rng=rng,
            cfg=cfg,
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

    # CSV was written incrementally (one row per generation, flushed to disk).
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
    has_mat, matf = _col("maternal_fraction")
    has_ff, fff = _col("frac_forage")
    has_fc, fcc = _col("frac_care")
    has_fs, fss = _col("frac_self")
    has_fp, fpp = _col("frac_protect")
    has_behavior = has_mat or (has_ff and has_fc and has_fs and has_fp)

    best_so_far = []
    b = -1e9
    for f in fitness:
        b = max(b, f)
        best_so_far.append(b)

    # 4 panels when behavior fractions are present, else 3 with TTD, else 2.
    if has_behavior:
        n_panels = 4
    elif has_cttd or has_mttd:
        n_panels = 3
    else:
        n_panels = 2
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
            ax2.set_ylabel("Mean survival")
            ax2.set_ylim(-0.05, 1.05)
            ax2.legend(loc="lower right", fontsize=8)
            ax2.grid(True, alpha=0.3)
            if n_panels < 4:
                ax2.set_xlabel("Generation")
        else:
            ax1.set_xlabel("Generation")
    else:
        # Legacy binary survival panel
        if has_surv:
            ax1.plot(gens, [float(x) for x in surv], "s-", color="coral", alpha=0.85, markersize=4, label="mean child survival (eval batch)")
        ax1.set_ylabel("Mean child survival")
        ax1.set_ylim(-0.05, 1.05)
        ax1.legend(loc="lower right", fontsize=8)
        ax1.grid(True, alpha=0.3)
        if n_panels < 4:
            ax1.set_xlabel("Generation")

    if has_behavior and n_panels >= 4:
        ax3 = axes[3]
        if has_ff:
            ax3.plot(gens, [float(x) for x in fff], color="tab:olive", alpha=0.8, linewidth=1.2, label="frac Forage")
        if has_fc:
            ax3.plot(gens, [float(x) for x in fcc], color="tab:green", alpha=0.85, linewidth=1.4, label="frac Care")
        if has_fs:
            ax3.plot(gens, [float(x) for x in fss], color="tab:gray", alpha=0.8, linewidth=1.2, label="frac Self")
        if has_fp:
            ax3.plot(gens, [float(x) for x in fpp], color="tab:red", alpha=0.85, linewidth=1.4, label="frac Protect")
        if has_mat:
            ax3.plot(
                gens, [float(x) for x in matf],
                color="black", alpha=0.95, linewidth=1.8, linestyle="--",
                label="maternal fraction (Care+Protect)",
            )
        ax3.set_xlabel("Generation")
        ax3.set_ylabel("Behavior fraction (per alive tick)")
        ax3.set_ylim(-0.02, 1.02)
        ax3.legend(loc="upper right", fontsize=7, ncol=2)
        ax3.grid(True, alpha=0.3)

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
        help="Weight on child in ttd_overall fitness (0..1). Example 0.7 = 70%% child, 30%% mother.",
    )
    p.add_argument(
        "--checkpoint-every",
        type=int,
        default=0,
        metavar="N",
        help="Write best-genome checkpoints every N generations into output_dir/checkpoints/. 0 = off.",
    )
    p.add_argument(
        "--resume",
        action="store_true",
        help=(
            "Resume an interrupted run from output_dir/latest_state.json. "
            "Restores best_genome, best_fit, RNG state, and the in-memory CSV "
            "rows; the loop continues from the last completed generation+1. "
            "Other CLI flags should match the original run (use cli_args.txt)."
        ),
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
    p.add_argument(
        "--lr",
        type=float,
        default=None,
        metavar="FLOAT",
        help=(
            "Override base learning rate used by 'outcome' / 'outcome_signed' rules "
            f"(default {_LR_DEFAULT_BASE}). Also seeds the start of the adaptive range."
        ),
    )
    p.add_argument(
        "--lr-min",
        type=float,
        default=None,
        metavar="FLOAT",
        help=(
            "Override learning_rate_min for 'outcome_adaptive*' "
            f"(default {_LR_DEFAULT_MIN})."
        ),
    )
    p.add_argument(
        "--lr-max",
        type=float,
        default=None,
        metavar="FLOAT",
        help=(
            "Override learning_rate_max for 'outcome_adaptive*' "
            f"(default {_LR_DEFAULT_MAX})."
        ),
    )
    p.add_argument(
        "--lr-deficit-scale",
        type=float,
        default=None,
        metavar="FLOAT",
        help=(
            "Override deficit-saturation point for adaptive LR "
            f"(default {_LR_DEFAULT_DEFICIT_SCALE})."
        ),
    )
    p.add_argument(
        "--lr-scale",
        type=float,
        default=None,
        metavar="FACTOR",
        help=(
            "Convenience multiplier applied to the resolved LR values (lr, lr-min, lr-max). "
            "E.g. --lr-scale 0.1 makes all learning rates 10x smaller; 0.01 = 100x smaller."
        ),
    )
    p.add_argument(
        "--init-mode",
        choices=["baseline_zero", "random_uniform", "anti_maternal", "pro_maternal", "from_json"],
        default="baseline_zero",
        help=(
            "Initial motivation-weight genome: "
            "baseline_zero (all 0.5; legacy default), "
            "random_uniform (U(init_low,init_high) per weight; uses --init-seed), "
            "anti_maternal (low Care/Protect, high Self -- de novo emergence test), "
            "pro_maternal (high Care/Protect; control), "
            "from_json (load --init-from PATH)."
        ),
    )
    p.add_argument(
        "--init-seed",
        type=int,
        default=None,
        metavar="INT",
        help=(
            "RNG seed used only by random_uniform and the init_noise jitter on templates. "
            "Defaults to --seed-master if omitted, so init varies with the master seed."
        ),
    )
    p.add_argument(
        "--init-noise",
        type=float,
        default=0.0,
        metavar="SIGMA",
        help=(
            "Gaussian sigma added to every weight after the initialization template is "
            "chosen (no-op for random_uniform). Useful to add per-replicate jitter to "
            "anti_maternal / pro_maternal / baseline_zero / from_json starts."
        ),
    )
    p.add_argument(
        "--init-from",
        type=str,
        default=None,
        metavar="PATH",
        help="Path to JSON genome file (used iff --init-mode from_json).",
    )
    p.add_argument(
        "--init-low",
        type=float,
        default=_GENOME_LO,
        metavar="FLOAT",
        help=f"Lower bound for random_uniform init (default {_GENOME_LO}).",
    )
    p.add_argument(
        "--init-high",
        type=float,
        default=_GENOME_HI,
        metavar="FLOAT",
        help=f"Upper bound for random_uniform init (default {_GENOME_HI}).",
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
                "resume": bool(args.resume),
                "plasticity_rule": rule,
                "plasticity_deficit_signal": str(args.deficit_signal),
                "plasticity_learn_w": args.learn_w == "on",
                "plasticity_update_mode": str(args.update_mode),
                "plasticity_segment_kmax": int(args.segment_kmax),
                "lr_base": (None if args.lr is None else float(args.lr)),
                "lr_min": (None if args.lr_min is None else float(args.lr_min)),
                "lr_max": (None if args.lr_max is None else float(args.lr_max)),
                "lr_deficit_scale": (None if args.lr_deficit_scale is None else float(args.lr_deficit_scale)),
                "lr_scale": (None if args.lr_scale is None else float(args.lr_scale)),
                "init_mode": str(args.init_mode),
                "init_seed": int(args.init_seed) if args.init_seed is not None else int(args.seed_master),
                "init_noise": float(args.init_noise),
                "init_from": args.init_from,
                "init_low": float(args.init_low),
                "init_high": float(args.init_high),
            }
        )
        main(cfg, open_viz_after=args.viz, watch_every=args.watch_every)
