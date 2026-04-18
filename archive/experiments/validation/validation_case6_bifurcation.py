#!/usr/bin/env python3
"""Validation Case 6: Bifurcation Verification at γ=0.5"""
import os, sys, csv, random, numpy as np
from collections import defaultdict
sys.path.insert(0, os.path.dirname(__file__))
from core.world import World
from BASELINE_0_LOWFEAR_SEARCH_CONFIG import get_phase_3_weights

OUTPUT_DIR = "test_results/validation/bifurcation_check"
os.makedirs(OUTPUT_DIR, exist_ok=True)

results = []

for rep in range(30):
    seed = 500 + rep
    random.seed(seed)
    np.random.seed(seed)
    
    weights = get_phase_3_weights(low_fear_scale_factor=0.5)  # γ=0.5 (BIFURCATION POINT)
    
    world = World(
        grid_w=20, grid_h=20,
        mother_starts=[(10, 10)],
        child_start=[(12, 12)],
        food_positions=[(5, 5), (15, 5), (5, 15), (15, 15)],
        threat_starts=[],
        seed=seed,
        day_step=100,
        use_fixed_weights=False,
        baseline_weights=weights,
    )
    
    for tick in range(3000):
        world.step()
    
    child_alive = 1 if (world.children and world.children[0].alive) else 0
    results.append({'rep': rep, 'seed': seed, 'child_survival': child_alive})

with open(os.path.join(OUTPUT_DIR, 'case_6_bifurcation_gamma_0_5.csv'), 'w', newline='') as f:
    writer = csv.DictWriter(f, fieldnames=['rep', 'seed', 'child_survival'])
    writer.writeheader()
    writer.writerows(results)

survivals = [r['child_survival'] for r in results]
mean_survival = 100.0 * np.mean(survivals)
std_survival = 100.0 * np.std(survivals)

print(f"Case 6 complete: {len(results)} replicates at γ=0.5")
print(f"  Mean survival: {mean_survival:.1f}% ± {std_survival:.1f}%")
print(f"  Bifurcation evidence: {sum(survivals)} successes, {len(results)-sum(survivals)} failures")
