"""
BASELINE-1: Random Genetic Weights (Heterogeneous Baseline)

Purpose: Establish a baseline with random mother genetic weights (U(0,1))
to capture phenotypic diversity and validate that genetic variance causes
the observed bimodal distribution in child survival.

Expected Outcome:
- Mother 100% survival
- Child survival ~33-40% (BIMODAL distribution: foragers ~0%, caregivers ~70%)
- Forage% shows two distinct peaks (high: ~75%, low: ~25%)
- Care% shows two distinct peaks (high: ~60%, low: ~5%)
- Phenotype classification:
    - Forager: Forage% > 70% AND Care% < 30%
    - Caregiver: Care% > 50% OR (Forage% < 40% AND Care% > 30%)
    - Mixed: Neither classification

Configuration:
- use_fixed_weights=False (random U(0,1) for each weight)
- 60 replicates (to capture full bimodal distribution with ~30 each phenotype)
- Grid: 20x20
- Mother: 1, Child: 1, Food: 1, Threats: 0
- Max ticks: 3000

Output: test_results/baseline_1/
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
NUM_REPLICATES = 60
MAX_TICKS = 3000
GRID_W = 20
GRID_H = 20
NUM_MOTHERS = 1
NUM_CHILDREN = 1
NUM_THREATS = 0
FOOD_AT_START = 1
FOOD_SPAWN_INTERVAL = None
FOOD_SPAWN_N = 1
USE_FIXED_WEIGHTS = False  # BASELINE-1: Random weights
PLASTICITY_RULE = "outcome"
DAY_STEP = 100
SEED_BASE = 42

OUTPUT_DIR = "test_results/baseline_1"
os.makedirs(OUTPUT_DIR, exist_ok=True)

# Phenotype classification thresholds
FORAGER_FORAGE_MIN = 70.0  # Forage% >= this
FORAGER_CARE_MAX = 30.0    # Care% <= this
CAREGIVER_CARE_MIN = 50.0  # Care% >= this
CAREGIVER_FORAGE_MAX = 40.0 # Forage% <= this

# ============================================================================
# REPLICATE RUNNER
# ============================================================================
def run_single_replicate(replicate_id, seed):
    """
    Run one replicate with random genetic weights.
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
    
    # Create world with use_fixed_weights=False
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
        use_fixed_weights=USE_FIXED_WEIGHTS,  # BASELINE-1 PARAMETER
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
    
    # Classify phenotype
    forage_pct = metrics["mother_forage_pct"]
    care_pct = metrics["mother_care_pct"]
    
    if forage_pct >= FORAGER_FORAGE_MIN and care_pct <= FORAGER_CARE_MAX:
        phenotype = "forager"
    elif care_pct >= CAREGIVER_CARE_MIN or (forage_pct <= CAREGIVER_FORAGE_MAX and care_pct >= 30.0):
        phenotype = "caregiver"
    else:
        phenotype = "mixed"
    
    metrics["phenotype"] = phenotype
    
    return metrics

# ============================================================================
# MAIN RUNNER
# ============================================================================
def main():
    print("=" * 70)
    print(f"BASELINE-1: Random Genetic Weights (Heterogeneous Baseline)")
    print("=" * 70)
    print(f"Replicates: {NUM_REPLICATES}")
    print(f"Max ticks per replicate: {MAX_TICKS}")
    print(f"Grid: {GRID_W}x{GRID_H}")
    print(f"Use fixed weights: {USE_FIXED_WEIGHTS} (random U(0,1))")
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
            phenotype_abbr = metrics["phenotype"][0].upper()  # F, C, or M
            print(f"Mother: {mother_status} (E={metrics['mother_energy']:.1f}), "
                  f"Child: {child_status}, "
                  f"Forage: {metrics['mother_forage_pct']:.1f}%, "
                  f"Phenotype: {phenotype_abbr}")
        except Exception as e:
            print(f"ERROR: {e}")
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
        output_file = os.path.join(OUTPUT_DIR, "baseline_1_replicates.csv")
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
        
        # Phenotype distribution
        phenotypes = [r["phenotype"] for r in valid_results]
        foragers = sum(1 for p in phenotypes if p == "forager")
        caregivers = sum(1 for p in phenotypes if p == "caregiver")
        mixed = sum(1 for p in phenotypes if p == "mixed")
        
        print("Phenotype Distribution:")
        print(f"- Foragers: {foragers} ({foragers/len(valid_results)*100:.1f}%)")
        print(f"- Caregivers: {caregivers} ({caregivers/len(valid_results)*100:.1f}%)")
        print(f"- Mixed: {mixed} ({mixed/len(valid_results)*100:.1f}%)")
        print()
        
        # Child survival by phenotype
        forager_results = [r for r in valid_results if r["phenotype"] == "forager"]
        caregiver_results = [r for r in valid_results if r["phenotype"] == "caregiver"]
        mixed_results = [r for r in valid_results if r["phenotype"] == "mixed"]
        
        print("Child Survival by Phenotype:")
        if forager_results:
            forager_child_survival = (sum(r["child_survival"] for r in forager_results) / len(forager_results)) * 100
            print(f"- Foragers (n={len(forager_results)}): {forager_child_survival:.1f}%")
        if caregiver_results:
            caregiver_child_survival = (sum(r["child_survival"] for r in caregiver_results) / len(caregiver_results)) * 100
            print(f"- Caregivers (n={len(caregiver_results)}): {caregiver_child_survival:.1f}%")
        if mixed_results:
            mixed_child_survival = (sum(r["child_survival"] for r in mixed_results) / len(mixed_results)) * 100
            print(f"- Mixed (n={len(mixed_results)}): {mixed_child_survival:.1f}%")
        print()
        
        print("Expected Outcome:")
        print("- Mother survival: ~100% (random weights still enable survival)")
        print("- Child survival: ~33-40% (BIMODAL: foragers ~0%, caregivers ~70%)")
        print("- Phenotype split: ~50% foragers, ~50% caregivers")
        print("- Forage%: Bimodal (two peaks at ~75% and ~25%)")
        print("- Care%: Bimodal (two peaks at ~60% and ~5%)")
        print()
        
        # Write summary
        summary_file = os.path.join(OUTPUT_DIR, "baseline_1_summary.txt")
        with open(summary_file, "w", encoding="utf-8") as f:
            f.write("BASELINE-1: Random Genetic Weights Summary\n")
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
            f.write("\n")
            f.write("Phenotype Distribution:\n")
            f.write(f"- Foragers: {foragers} ({foragers/len(valid_results)*100:.1f}%)\n")
            f.write(f"- Caregivers: {caregivers} ({caregivers/len(valid_results)*100:.1f}%)\n")
            f.write(f"- Mixed: {mixed} ({mixed/len(valid_results)*100:.1f}%)\n")
            if forager_results:
                forager_child_survival = (sum(r["child_survival"] for r in forager_results) / len(forager_results)) * 100
                f.write(f"Child survival (Foragers): {forager_child_survival:.1f}%\n")
            if caregiver_results:
                caregiver_child_survival = (sum(r["child_survival"] for r in caregiver_results) / len(caregiver_results)) * 100
                f.write(f"Child survival (Caregivers): {caregiver_child_survival:.1f}%\n")
        print(f"✓ Wrote summary to {summary_file}")
    
    print()
    print("=" * 70)
    print("BASELINE-1 RUN COMPLETE")
    print("=" * 70)

if __name__ == "__main__":
    main()
