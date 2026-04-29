"""
PASSIVE SURVIVAL LOWER BOUND (World difficulty floor)
=====================================================

Purpose
-------
Estimate how long mother/child survive if the mother does *nothing*:
- no movement
- no actions (no forage, no care, no protect)

This gives a "world rule" lower bound that any mechanism should beat.

Implementation note
-------------------
We force passivity by setting mother.fatigue >= 90 every tick. In
`core/policies/mother.py`, this causes the mother to stay in place and
skip action selection.

Output
------
Writes CSV to: test_results/passive_lower_bound/
"""

import os
import csv
import argparse
import numpy as np
import json
from datetime import datetime

from core.world import World
from core.seed import init_seed


# ============================================================================
# DEFAULTS (override via CLI flags)
# ============================================================================
DEFAULT_NUM_REPLICATES = 50
DEFAULT_MAX_TICKS = 10000

DEFAULT_GRID_W = 10
DEFAULT_GRID_H = 10
DEFAULT_NUM_MOTHERS = 1
DEFAULT_NUM_CHILDREN = 1
DEFAULT_NUM_THREATS = 0

DEFAULT_FOOD_AT_START = 0
DEFAULT_FOOD_SPAWN_INTERVAL = 20
DEFAULT_FOOD_SPAWN_N = 1

DEFAULT_DAY_STEP = 100
DEFAULT_SEED_BASE = 1000

DEFAULT_OUTPUT_DIR = "test_results/passive_lower_bound"

# Optional: write a per-tick timeseries CSV for each replicate (useful for plotting).
DEFAULT_LOG_TIMESERIES = True
DEFAULT_LOG_EVERY_N_TICKS = 1


def _random_positions(rng, n, w, h):
    # Note: randint high is exclusive; use [0, w-1], [0, h-1]
    return [(int(rng.randint(0, w)), int(rng.randint(0, h))) for _ in range(n)]

def _mean_abs_diff_nested(a: dict, b: dict) -> float:
    """
    Mean |a-b| over keys in nested dicts.
    Expects a and b to have the same structure (as fixed vs plastic weights).
    """
    vals = []
    for k, v in a.items():
        if isinstance(v, dict) and isinstance(b.get(k), dict):
            vals.append(_mean_abs_diff_nested(v, b[k]))
        else:
            try:
                vals.append(abs(float(v) - float(b.get(k, v))))
            except Exception:
                continue
    vals = [x for x in vals if np.isfinite(x)]
    return float(np.mean(vals)) if vals else float("nan")


def _get_nested(d: dict, *keys, default=np.nan):
    cur = d
    for k in keys:
        if not isinstance(cur, dict) or k not in cur:
            return default
        cur = cur[k]
    return cur


def run_single_replicate(
    replicate_id: int,
    seed: int,
    *,
    grid_w: int,
    grid_h: int,
    num_mothers: int,
    num_children: int,
    num_threats: int,
    food_at_start: int,
    food_spawn_interval,
    food_spawn_n: int,
    day_step: int,
    passive_mothers: bool,
    plasticity_rule,
    plasticity_deficit_signal: str,
    plasticity_learn_w: bool,
    plasticity_update_mode: str,
    plasticity_segment_kmax: int,
    use_fixed_weights: bool,
    baseline_weights: dict | None,
    output_dir: str,
    log_timeseries: bool,
    log_every_n_ticks: int,
    max_ticks: int,
):
    init_seed(seed)
    rng = np.random.RandomState(seed)

    mother_starts = _random_positions(rng, num_mothers, grid_w, grid_h)
    child_start = _random_positions(rng, num_children, grid_w, grid_h)
    threat_starts = _random_positions(rng, num_threats, grid_w, grid_h)
    food_positions = _random_positions(rng, food_at_start, grid_w, grid_h)

    world = World(
        grid_w=grid_w,
        grid_h=grid_h,
        mother_starts=mother_starts,
        child_start=child_start,
        food_positions=food_positions,
        threat_starts=threat_starts,
        seed=seed,
        day_step=day_step,
        plasticity_rule=plasticity_rule,
        plasticity_deficit_signal=plasticity_deficit_signal,
        plasticity_learn_w=plasticity_learn_w,
        plasticity_update_mode=plasticity_update_mode,
        plasticity_segment_kmax=plasticity_segment_kmax,
        food_spawn_interval=food_spawn_interval,
        food_spawn_n=food_spawn_n,
        use_fixed_weights=use_fixed_weights,
        baseline_weights=baseline_weights,
    )
    # Optionally force mothers to take no movement and no actions.
    world.passive_mothers = bool(passive_mothers)

    mother = world.mothers[0] if world.mothers else None
    child = world.children[0] if world.children else None

    mother_death_tick = None
    child_death_tick = None

    ts_rows = []
    for tick in range(max_ticks):
        world.step()

        if log_timeseries and (tick % max(1, log_every_n_ticks) == 0):
            # Use None-safe access so this logger works even if an agent dies mid-run.
            m_alive = bool(mother.is_alive()) if mother is not None else False
            c_alive = bool(child.is_alive()) if child is not None else False

            # Threat distance to child (min across threats). Octile distance to match the sim.
            threat_min_dist = None
            if child is not None and getattr(world, "threats", None):
                if world.threats:
                    threat_min_dist = min(
                        child.distance_to(t.x, t.y, metric="octile") for t in world.threats
                    )

            ts_rows.append({
                "replicate": replicate_id,
                "seed": seed,
                "tick": tick,
                "day": tick / float(day_step),
                # Child
                "child_alive": int(c_alive),
                "child_hunger": float(getattr(child, "hunger", np.nan)) if child is not None else np.nan,
                "child_warmth": float(getattr(child, "warmth", np.nan)) if child is not None else np.nan,
                "child_injury": float(getattr(child, "injury", np.nan)) if child is not None else np.nan,
                "child_threat_close_radius": float(getattr(child, "threat_recovery_radius", np.nan)) if child is not None else np.nan,
                "threat_min_dist_to_child": float(threat_min_dist) if threat_min_dist is not None else np.nan,
                # Mother
                "mother_alive": int(m_alive),
                "mother_energy": float(getattr(mother, "energy", np.nan)) if mother is not None else np.nan,
                "mother_fatigue": float(getattr(mother, "fatigue", np.nan)) if mother is not None else np.nan,
                "mother_bonding": float(getattr(mother, "bonding", np.nan)) if mother is not None else np.nan,
                "mother_fear_threat": float(getattr(mother, "fear_threat", np.nan)) if mother is not None else np.nan,
                "mother_stress": float(getattr(mother, "stress", np.nan)) if mother is not None else np.nan,
                "mother_closeness_child": float(getattr(mother, "closeness_child", np.nan)) if mother is not None else np.nan,
                "mother_oxytocin": float(getattr(mother, "oxytocin", np.nan)) if mother is not None else np.nan,
                "mother_cortisol": float(getattr(mother, "cortisol", np.nan)) if mother is not None else np.nan,
                "mother_selected_motivation": getattr(mother, "selected_motivation", None) if mother is not None else None,
                # Plasticity diagnostics (if enabled)
                "deficit_before": float(getattr(mother, "_last_deficit_before", np.nan)) if mother is not None else np.nan,
                "deficit_after": float(getattr(mother, "_last_deficit_after", np.nan)) if mother is not None else np.nan,
                "learning_rate_eff": float(getattr(mother, "_last_learning_rate_eff", np.nan)) if mother is not None else np.nan,
                "plasticity_rule": getattr(mother, "_last_plasticity_rule", None) if mother is not None else None,
                "plasticity_deficit_signal": getattr(world, "plasticity_deficit_signal", None),
                "plasticity_learn_w": int(bool(getattr(world, "plasticity_learn_w", True))),
                "plasticity_update_mode": getattr(world, "plasticity_update_mode", None),
                "plasticity_segment_kmax": int(getattr(world, "plasticity_segment_kmax", -1)),
                # Weight drift (mean |plastic-fixed|)
                "u_drift": _mean_abs_diff_nested(getattr(mother, "motivation_weights_fixed", {}), getattr(mother, "motivation_weights_plastic", {})) if mother is not None else np.nan,
                "w_drift": _mean_abs_diff_nested(getattr(mother, "psych_weights_fixed", {}), getattr(mother, "psych_weights_plastic", {})) if mother is not None else np.nan,
                # Key weights (fixed/plastic) for interpretation
                "u_forage_child_hunger_fixed": float(_get_nested(getattr(mother, "motivation_weights_fixed", {}), "forage", "child_hunger")) if mother is not None else np.nan,
                "u_forage_child_hunger_plastic": float(_get_nested(getattr(mother, "motivation_weights_plastic", {}), "forage", "child_hunger")) if mother is not None else np.nan,
                "u_care_child_warmth_fixed": float(_get_nested(getattr(mother, "motivation_weights_fixed", {}), "care", "child_warmth")) if mother is not None else np.nan,
                "u_care_child_warmth_plastic": float(_get_nested(getattr(mother, "motivation_weights_plastic", {}), "care", "child_warmth")) if mother is not None else np.nan,
                "w_fear_threat_gain_fixed": float(_get_nested(getattr(mother, "psych_weights_fixed", {}), "fear", "threat_gain")) if mother is not None else np.nan,
                "w_fear_threat_gain_plastic": float(_get_nested(getattr(mother, "psych_weights_plastic", {}), "fear", "threat_gain")) if mother is not None else np.nan,
                "w_fear_decay_fixed": float(_get_nested(getattr(mother, "psych_weights_fixed", {}), "fear", "decay")) if mother is not None else np.nan,
                "w_fear_decay_plastic": float(_get_nested(getattr(mother, "psych_weights_plastic", {}), "fear", "decay")) if mother is not None else np.nan,
                "w_oxytocin_closeness_gain_fixed": float(_get_nested(getattr(mother, "psych_weights_fixed", {}), "oxytocin", "closeness_gain")) if mother is not None else np.nan,
                "w_oxytocin_closeness_gain_plastic": float(_get_nested(getattr(mother, "psych_weights_plastic", {}), "oxytocin", "closeness_gain")) if mother is not None else np.nan,
                "w_oxytocin_decay_fixed": float(_get_nested(getattr(mother, "psych_weights_fixed", {}), "oxytocin", "decay")) if mother is not None else np.nan,
                "w_oxytocin_decay_plastic": float(_get_nested(getattr(mother, "psych_weights_plastic", {}), "oxytocin", "decay")) if mother is not None else np.nan,
                "mother_x": int(getattr(mother, "x", -1)) if mother is not None else -1,
                "mother_y": int(getattr(mother, "y", -1)) if mother is not None else -1,
                "child_x": int(getattr(child, "x", -1)) if child is not None else -1,
                "child_y": int(getattr(child, "y", -1)) if child is not None else -1,
            })

        if child is not None and child_death_tick is None and (not child.is_alive()):
            child_death_tick = tick
        if mother is not None and mother_death_tick is None and (not mother.is_alive()):
            mother_death_tick = tick

        if (child_death_tick is not None) and (mother_death_tick is not None):
            break

    if log_timeseries and ts_rows:
        ts_path = os.path.join(
            output_dir,
            f"passive_lower_bound_timeseries_rep{replicate_id:03d}_seed{seed}.csv",
        )
        with open(ts_path, "w", newline="", encoding="utf-8") as f:
            w = csv.DictWriter(f, fieldnames=list(ts_rows[0].keys()))
            w.writeheader()
            w.writerows(ts_rows)

    # If still alive at end, treat as right-censored at MAX_TICKS
    if child is not None and child_death_tick is None and child.is_alive():
        child_death_tick = max_ticks
    if mother is not None and mother_death_tick is None and mother.is_alive():
        mother_death_tick = max_ticks

    return {
        "replicate": replicate_id,
        "seed": seed,
        "max_ticks": max_ticks,
        "child_death_tick": child_death_tick,
        "mother_death_tick": mother_death_tick,
        "child_alive_end": bool(child.is_alive()) if child is not None else False,
        "mother_alive_end": bool(mother.is_alive()) if mother is not None else False,
        "final_child_injury": float(getattr(child, "injury", 0.0)) if child is not None else 0.0,
        "final_child_hunger": float(getattr(child, "hunger", 0.0)) if child is not None else 0.0,
        "final_mother_energy": float(getattr(mother, "energy", 0.0)) if mother is not None else 0.0,
    }


def main():
    parser = argparse.ArgumentParser(description="Passive/active survival experiments with optional timeseries logging.")
    parser.add_argument("--mode", choices=["passive", "active"], default="passive",
                        help="passive: mother does nothing; active: normal mother policy runs.")
    parser.add_argument("--plasticity", choices=["none", "outcome", "outcome_adaptive", "outcome_signed", "outcome_adaptive_signed"], default="none",
                        help="Learning rule for mother weights (active mode only). Default: none (keep weights fixed).")
    parser.add_argument(
        "--deficit-signal",
        choices=["global", "local"],
        default="global",
        help="Plasticity deficit signal: global=overall deficit; local=motivation-aligned deficit (active mode only).",
    )
    parser.add_argument(
        "--learn-w",
        choices=["on", "off"],
        default="on",
        help="Whether plasticity updates psych/state weights w. 'off' freezes w and learns only motivation weights u (active mode only).",
    )
    parser.add_argument(
        "--update-mode",
        choices=["per_tick", "segment", "segment_capped"],
        default="per_tick",
        help="Plasticity credit assignment timing. per_tick=update each tick; segment=update when motivation changes; segment_capped=segment + update every K ticks.",
    )
    parser.add_argument(
        "--segment-kmax",
        type=int,
        default=20,
        help="For segment_capped: update every K ticks if motivation persists (default 20). Ignored in other modes.",
    )
    parser.add_argument(
        "--genome",
        type=str,
        default=None,
        help="Path to evolved motivation genome JSON (final_genome.json). If provided, uses it as baseline_weights for mothers.",
    )
    parser.add_argument("--threats", type=int, default=DEFAULT_NUM_THREATS, help="Number of threats to spawn.")
    parser.add_argument("--food-start", type=int, default=DEFAULT_FOOD_AT_START, help="Food placed at start.")
    parser.add_argument("--food-spawn-interval", type=int, default=None,
                        help="Spawn interval in ticks (omit/None disables spawning).")
    parser.add_argument("--food-spawn-n", type=int, default=DEFAULT_FOOD_SPAWN_N, help="Food spawned each interval.")
    parser.add_argument("--replicates", type=int, default=DEFAULT_NUM_REPLICATES, help="Number of replicates.")
    parser.add_argument("--max-ticks", type=int, default=DEFAULT_MAX_TICKS, help="Max ticks per replicate.")
    parser.add_argument("--grid-w", type=int, default=DEFAULT_GRID_W)
    parser.add_argument("--grid-h", type=int, default=DEFAULT_GRID_H)
    parser.add_argument("--day-step", type=int, default=DEFAULT_DAY_STEP, help="Ticks per day.")
    parser.add_argument("--seed-base", type=int, default=DEFAULT_SEED_BASE)
    parser.add_argument("--output-dir", type=str, default=DEFAULT_OUTPUT_DIR)
    parser.add_argument("--log-timeseries", action="store_true", default=DEFAULT_LOG_TIMESERIES)
    parser.add_argument("--no-log-timeseries", action="store_false", dest="log_timeseries")
    parser.add_argument("--log-every", type=int, default=DEFAULT_LOG_EVERY_N_TICKS, help="Log every N ticks.")
    args = parser.parse_args()

    os.makedirs(args.output_dir, exist_ok=True)

    baseline_weights = None
    if args.genome:
        with open(args.genome, "r", encoding="utf-8") as f:
            baseline_weights = json.load(f)
        if not isinstance(baseline_weights, dict):
            raise SystemExit("--genome JSON must be a dict of motivation weights (baseline_weights).")

    passive_mothers = (args.mode == "passive")
    plasticity_rule = None
    if not passive_mothers and args.plasticity in ("outcome", "outcome_adaptive", "outcome_signed", "outcome_adaptive_signed"):
        plasticity_rule = args.plasticity
    plasticity_deficit_signal = args.deficit_signal if not passive_mothers else "global"
    plasticity_learn_w = (args.learn_w == "on") if not passive_mothers else False
    plasticity_update_mode = args.update_mode if not passive_mothers else "per_tick"
    plasticity_segment_kmax = int(args.segment_kmax)
    use_fixed_weights = True  # keeps experiments comparable; can be exposed later if needed

    print("=" * 70)
    print("SURVIVAL EXPERIMENT")
    print("=" * 70)
    print(f"Mode: {args.mode}")
    print(f"Replicates: {args.replicates}")
    print(f"Max ticks: {args.max_ticks}")
    print(f"Grid: {args.grid_w}x{args.grid_h} (day_step={args.day_step})")
    print(f"Threats: {args.threats}, Food at start: {args.food_start}")
    print(f"Food spawn: interval={args.food_spawn_interval}, n={args.food_spawn_n}")
    print(f"Output: {args.output_dir}")
    print("=" * 70)

    rows = []
    for rid in range(1, args.replicates + 1):
        seed = args.seed_base + rid
        print(f"Running {rid}/{args.replicates} (seed={seed})...", end=" ", flush=True)
        try:
            r = run_single_replicate(
                rid,
                seed,
                grid_w=args.grid_w,
                grid_h=args.grid_h,
                num_mothers=DEFAULT_NUM_MOTHERS,
                num_children=DEFAULT_NUM_CHILDREN,
                num_threats=args.threats,
                food_at_start=args.food_start,
                food_spawn_interval=args.food_spawn_interval,
                food_spawn_n=args.food_spawn_n,
                day_step=args.day_step,
                passive_mothers=passive_mothers,
                plasticity_rule=plasticity_rule,
                plasticity_deficit_signal=plasticity_deficit_signal,
                plasticity_learn_w=plasticity_learn_w,
                plasticity_update_mode=plasticity_update_mode,
                plasticity_segment_kmax=plasticity_segment_kmax,
                use_fixed_weights=use_fixed_weights,
                baseline_weights=baseline_weights,
                output_dir=args.output_dir,
                log_timeseries=args.log_timeseries,
                log_every_n_ticks=args.log_every,
                max_ticks=args.max_ticks,
            )
            rows.append(r)
            print(f"child_death={r['child_death_tick']}, mother_death={r['mother_death_tick']}")
        except Exception as e:
            import traceback
            print(f"ERROR: {e}")
            traceback.print_exc()

    if not rows:
        print("No results generated.")
        return

    csv_path = os.path.join(args.output_dir, "survival_experiment_replicates.csv")
    with open(csv_path, "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)
    print(f"\nWrote CSV: {csv_path}")

    child_deaths = np.array([r["child_death_tick"] for r in rows], dtype=float)
    mother_deaths = np.array([r["mother_death_tick"] for r in rows], dtype=float)

    def _summary(arr):
        return float(np.mean(arr)), float(np.std(arr)), float(np.median(arr))

    c_mean, c_std, c_med = _summary(child_deaths)
    m_mean, m_std, m_med = _summary(mother_deaths)

    print("\n" + "=" * 70)
    print("SUMMARY")
    print("=" * 70)
    print(f"Child time-to-death:  mean={c_mean:.1f}, std={c_std:.1f}, median={c_med:.1f}")
    print(f"Mother time-to-death: mean={m_mean:.1f}, std={m_std:.1f}, median={m_med:.1f}")

    summary_path = os.path.join(args.output_dir, "survival_experiment_summary.txt")
    with open(summary_path, "w", encoding="utf-8") as f:
        f.write("SURVIVAL EXPERIMENT\n")
        f.write("=" * 70 + "\n")
        f.write(f"Timestamp: {datetime.now().isoformat()}\n")
        f.write(f"Mode: {args.mode}\n")
        f.write(f"Replicates: {args.replicates}\n")
        f.write(f"Max ticks: {args.max_ticks}\n")
        f.write(f"Grid: {args.grid_w}x{args.grid_h} (day_step={args.day_step})\n")
        f.write(f"Threats: {args.threats}, Food at start: {args.food_start}\n")
        f.write(f"Food spawn: interval={args.food_spawn_interval}, n={args.food_spawn_n}\n")
        f.write("\n")
        f.write(f"Child time-to-death:  mean={c_mean:.1f}, std={c_std:.1f}, median={c_med:.1f}\n")
        f.write(f"Mother time-to-death: mean={m_mean:.1f}, std={m_std:.1f}, median={m_med:.1f}\n")
    print(f"Wrote summary: {summary_path}")


if __name__ == "__main__":
    main()

