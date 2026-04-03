"""
BASELINE-0: Fixed Genetic Weights (Controlled Baseline)

Purpose: Establish a baseline with fixed mother genetic weights (0.5 for all)
to isolate environmental factors and remove genetic variance.

Expected Outcome:
- Mother 100% survival
- Child survival ~70% (non-bimodal, predominantly caregiver phenotype)
- Forage% and Care% should be stable across replicates (all mothers similar)

Configuration:
- use_fixed_weights=True (all weights = 0.5)
- 30 replicates
- Grid: 20x20
- Mother: 1, Child: 1, Food: 1, Threats: 0
- Max ticks: 3000

Output: test_results/baseline_0/
"""

import os
import csv
import numpy as np
from datetime import datetime
from core.world import World
from core.entities import Food
from core.seed import init_seed

# ============================================================================
# CONFIGURATION
# ============================================================================
NUM_REPLICATES = 30
MAX_TICKS = 3000
GRID_W = 20
GRID_H = 20
NUM_MOTHERS = 1
NUM_CHILDREN = 1
NUM_THREATS = 0
FOOD_AT_START = 1
FOOD_SPAWN_INTERVAL = None
FOOD_SPAWN_N = 1
USE_FIXED_WEIGHTS = True  # BASELINE-0: Fixed weights
PLASTICITY_RULE = "outcome"
DAY_STEP = 100
SEED_BASE = 42

OUTPUT_DIR = "test_results/baseline_0"
os.makedirs(OUTPUT_DIR, exist_ok=True)

# ============================================================================
# REPLICATE RUNNER
# ============================================================================
def run_single_replicate(replicate_id, seed):
    """
    Run one replicate with fixed genetic weights.
    Returns dict with metrics from final state.
    """
    # Initialize global RNG for this replicate's seed
    init_seed(seed)
    rng = np.random.RandomState(seed)
    
    # Initialize positions
    mother_starts = [(rng.randint(0, GRID_W), rng.randint(0, GRID_H)) for _ in range(NUM_MOTHERS)]
    child_start = [(rng.randint(0, GRID_W), rng.randint(0, GRID_H)) for _ in range(NUM_CHILDREN)]
    
    # Randomize food positions
    food_positions = []
    for _ in range(FOOD_AT_START):
        food_positions.append((rng.randint(0, GRID_W), rng.randint(0, GRID_H)))
    
    threat_starts = []  # No threats in baseline
    
    # Create world with use_fixed_weights=True
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
        use_fixed_weights=USE_FIXED_WEIGHTS,  # BASELINE-0 PARAMETER
    )
    
    mother = world.mothers[0] if world.mothers else None
    child = world.children[0] if world.children else None
    
    # Track motivation counts during simulation
    motivation_counts = {"Forage": 0, "Care": 0, "Self": 0, "Protect": 0}
    
    # Run simulation
    for tick in range(MAX_TICKS):
        world.step()
        
        # Track mother motivations
        if mother and mother.is_alive():
            motivation_counts[mother.selected_motivation] += 1
    
    # Extract final metrics
    metrics = {
        "replicate": replicate_id,
        "seed": seed,
        "max_ticks": MAX_TICKS,
        "mother_alive": mother.alive if mother else False,
        "mother_energy": mother.energy if mother else 0.0,
        "mother_age": mother.age if mother else 0,
        "child_alive": child.alive if child else False,
        "child_energy": child.energy if child else 0.0,
        "child_hunger": child.hunger if child else 0.0,
        "child_warmth": child.warmth if child else 0.0,
        "child_injury": child.injury if child else 0.0,
        "child_age": child.age if child else 0,
    }
    
    # Compute mother policy statistics
    total_decisions = max(1, sum(motivation_counts.values()))
    metrics["mother_forage_pct"] = (motivation_counts["Forage"] / total_decisions) * 100
    metrics["mother_care_pct"] = (motivation_counts["Care"] / total_decisions) * 100
    metrics["mother_explore_pct"] = (0 / total_decisions) * 100  # No "explore" motivation in this model
    metrics["mother_protect_pct"] = (motivation_counts["Protect"] / total_decisions) * 100
    
    # Compute child survival (binary: reached end or died before)
    metrics["child_survival"] = 1 if (child and child.alive) else 0
    
    return metrics

# ============================================================================
# MAIN RUNNER
# ============================================================================
def main():
    print("=" * 70)
    print(f"BASELINE-0: Fixed Genetic Weights (Controlled Baseline)")
    print("=" * 70)
    print(f"Replicates: {NUM_REPLICATES}")
    print(f"Max ticks per replicate: {MAX_TICKS}")
    print(f"Grid: {GRID_W}x{GRID_H}")
    print(f"Use fixed weights: {USE_FIXED_WEIGHTS} (all weights = 0.5)")
    print(f"Output directory: {OUTPUT_DIR}")
    print("=" * 70)
    print()
    
    all_results = []
    
    for replicate_id in range(1, NUM_REPLICATES + 1):
        seed = SEED_BASE + replicate_id
        print(f"Running replicate {replicate_id}/{NUM_REPLICATES} (seed={seed})...", end=" ", flush=True)
        
        try:
            metrics = run_single_replicate(replicate_id, seed)
            all_results.append(metrics)
            
            # Status line
            mother_status = "✓" if metrics["mother_alive"] else "✗"
            child_status = "✓" if metrics["child_alive"] else "✗"
            print(f"Mother: {mother_status} (E={metrics['mother_energy']:.1f}), "
                  f"Child: {child_status} (E={metrics['child_energy']:.1f}), "
                  f"Forage: {metrics['mother_forage_pct']:.1f}%")
        except Exception as e:
            import traceback
            print(f"ERROR: {e}")
            traceback.print_exc()
            all_results.append({
                "replicate": replicate_id,
                "seed": seed,
                "error": str(e),
            })
    
    print()
    print("=" * 70)
    print("Writing results to CSV...")
    print("=" * 70)
    
    # Write detailed results
    if all_results:
        output_file = os.path.join(OUTPUT_DIR, "baseline_0_replicates.csv")
        with open(output_file, "w", newline="") as f:
            fieldnames = all_results[0].keys()
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(all_results)
        print(f"✓ Wrote {len(all_results)} replicates to {output_file}")
    
    # Compute and print summary statistics
    valid_results = [r for r in all_results if "error" not in r]
    if valid_results:
        print()
        print("=" * 70)
        print("SUMMARY STATISTICS")
        print("=" * 70)
        
        mother_alive_pct = (sum(1 for r in valid_results if r["mother_alive"]) / len(valid_results)) * 100
        child_alive_pct = (sum(1 for r in valid_results if r["child_alive"]) / len(valid_results)) * 100
        child_survival_pct = (sum(r["child_survival"] for r in valid_results) / len(valid_results)) * 100
        
        print(f"Mother survival: {mother_alive_pct:.1f}%")
        print(f"Child survival (final state): {child_alive_pct:.1f}%")
        print(f"Child survival (cumulative): {child_survival_pct:.1f}%")
        print()
        
        forage_pcts = [r["mother_forage_pct"] for r in valid_results]
        care_pcts = [r["mother_care_pct"] for r in valid_results]
        
        print(f"Mother Forage: {np.mean(forage_pcts):.1f}% ± {np.std(forage_pcts):.1f}%")
        print(f"Mother Care:   {np.mean(care_pcts):.1f}% ± {np.std(care_pcts):.1f}%")
        print()
        print("Expected Outcome:")
        print("- Mother survival: ~100% (fixed weights enable consistent care)")
        print("- Child survival: ~70% (predominantly caregiver phenotype, no bimodality)")
        print("- Forage%: ~40-50% (stable, no variance across replicates)")
        print("- Care%: ~40-50% (stable, no variance across replicates)")
        print()
        
        # Write summary
        summary_file = os.path.join(OUTPUT_DIR, "baseline_0_summary.txt")
        with open(summary_file, "w", encoding="utf-8") as f:
            f.write("BASELINE-0: Fixed Genetic Weights Summary\n")
            f.write("=" * 70 + "\n")
            f.write(f"Timestamp: {datetime.now().isoformat()}\n")
            f.write(f"Replicates: {len(valid_results)}\n")
            f.write(f"Max ticks: {MAX_TICKS}\n")
            f.write(f"Use fixed weights: {USE_FIXED_WEIGHTS}\n")
            f.write("\n")
            f.write("RESULTS:\n")
            f.write(f"Mother survival: {mother_alive_pct:.1f}%\n")
            f.write(f"Child survival: {child_alive_pct:.1f}%\n")
            f.write(f"Mother Forage: {np.mean(forage_pcts):.1f}% ± {np.std(forage_pcts):.1f}%\n")
            f.write(f"Mother Care:   {np.mean(care_pcts):.1f}% ± {np.std(care_pcts):.1f}%\n")
        print(f"✓ Wrote summary to {summary_file}")
    
    print()
    print("=" * 70)
    print("BASELINE-0 RUN COMPLETE")
    print("=" * 70)

if __name__ == "__main__":
    main()
