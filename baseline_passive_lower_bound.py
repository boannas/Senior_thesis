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
import numpy as np
from datetime import datetime

from core.world import World
from core.seed import init_seed


# ============================================================================
# CONFIGURATION (edit these to match your ecology regime)
# ============================================================================
NUM_REPLICATES = 50
MAX_TICKS = 10000

GRID_W = 20
GRID_H = 20
NUM_MOTHERS = 1
NUM_CHILDREN = 1
NUM_THREATS = 0

FOOD_AT_START = 0
# Disable food spawning for passive baseline. If food spawns but mother never acts,
# the grid can fill with uneaten food and end the simulation for the wrong reason.
FOOD_SPAWN_INTERVAL = 30
FOOD_SPAWN_N = 1

DAY_STEP = 100
SEED_BASE = 1000

OUTPUT_DIR = "test_results/passive_lower_bound"
os.makedirs(OUTPUT_DIR, exist_ok=True)


def _random_positions(rng, n, w, h):
    return [(int(rng.randint(0, w)), int(rng.randint(0, h))) for _ in range(n)]


def run_single_replicate(replicate_id: int, seed: int):
    init_seed(seed)
    rng = np.random.RandomState(seed)

    mother_starts = _random_positions(rng, NUM_MOTHERS, GRID_W, GRID_H)
    child_start = _random_positions(rng, NUM_CHILDREN, GRID_W, GRID_H)
    threat_starts = _random_positions(rng, NUM_THREATS, GRID_W, GRID_H)
    food_positions = _random_positions(rng, FOOD_AT_START, GRID_W, GRID_H)

    world = World(
        grid_w=GRID_W,
        grid_h=GRID_H,
        mother_starts=mother_starts,
        child_start=child_start,
        food_positions=food_positions,
        threat_starts=threat_starts,
        seed=seed,
        day_step=DAY_STEP,
        plasticity_rule=None,  # learning off (passive baseline)
        food_spawn_interval=FOOD_SPAWN_INTERVAL,
        food_spawn_n=FOOD_SPAWN_N,
        use_fixed_weights=True,  # doesn't matter; mother won't act
    )
    world.passive_mothers = False

    mother = world.mothers[0] if world.mothers else None
    child = world.children[0] if world.children else None

    mother_death_tick = None
    child_death_tick = None

    for tick in range(MAX_TICKS):
        world.step()
        if child is not None and child_death_tick is None and (not child.is_alive()):
            child_death_tick = tick
        if mother is not None and mother_death_tick is None and (not mother.is_alive()):
            mother_death_tick = tick

        if (child_death_tick is not None) and (mother_death_tick is not None):
            break

    # If still alive at end, treat as right-censored at MAX_TICKS
    if child is not None and child_death_tick is None and child.is_alive():
        child_death_tick = MAX_TICKS
    if mother is not None and mother_death_tick is None and mother.is_alive():
        mother_death_tick = MAX_TICKS

    return {
        "replicate": replicate_id,
        "seed": seed,
        "max_ticks": MAX_TICKS,
        "child_death_tick": child_death_tick,
        "mother_death_tick": mother_death_tick,
        "child_alive_end": bool(child.is_alive()) if child is not None else False,
        "mother_alive_end": bool(mother.is_alive()) if mother is not None else False,
        "final_child_injury": float(getattr(child, "injury", 0.0)) if child is not None else 0.0,
        "final_child_hunger": float(getattr(child, "hunger", 0.0)) if child is not None else 0.0,
        "final_mother_energy": float(getattr(mother, "energy", 0.0)) if mother is not None else 0.0,
    }


def main():
    print("=" * 70)
    print("PASSIVE SURVIVAL LOWER BOUND")
    print("=" * 70)
    print(f"Replicates: {NUM_REPLICATES}")
    print(f"Max ticks: {MAX_TICKS}")
    print(f"Grid: {GRID_W}x{GRID_H}")
    print(f"Threats: {NUM_THREATS}, Food at start: {FOOD_AT_START}")
    print(f"Output: {OUTPUT_DIR}")
    print("=" * 70)

    rows = []
    for rid in range(1, NUM_REPLICATES + 1):
        seed = SEED_BASE + rid
        print(f"Running {rid}/{NUM_REPLICATES} (seed={seed})...", end=" ", flush=True)
        try:
            r = run_single_replicate(rid, seed)
            rows.append(r)
            print(f"child_death={r['child_death_tick']}, mother_death={r['mother_death_tick']}")
        except Exception as e:
            import traceback
            print(f"ERROR: {e}")
            traceback.print_exc()

    if not rows:
        print("No results generated.")
        return

    csv_path = os.path.join(OUTPUT_DIR, "passive_lower_bound_replicates.csv")
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

    summary_path = os.path.join(OUTPUT_DIR, "passive_lower_bound_summary.txt")
    with open(summary_path, "w", encoding="utf-8") as f:
        f.write("PASSIVE SURVIVAL LOWER BOUND\n")
        f.write("=" * 70 + "\n")
        f.write(f"Timestamp: {datetime.now().isoformat()}\n")
        f.write(f"Replicates: {NUM_REPLICATES}\n")
        f.write(f"Max ticks: {MAX_TICKS}\n")
        f.write(f"Grid: {GRID_W}x{GRID_H}\n")
        f.write(f"Threats: {NUM_THREATS}, Food at start: {FOOD_AT_START}\n")
        f.write("\n")
        f.write(f"Child time-to-death:  mean={c_mean:.1f}, std={c_std:.1f}, median={c_med:.1f}\n")
        f.write(f"Mother time-to-death: mean={m_mean:.1f}, std={m_std:.1f}, median={m_med:.1f}\n")
    print(f"Wrote summary: {summary_path}")


if __name__ == "__main__":
    main()

