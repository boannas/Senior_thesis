#!/usr/bin/env python3
"""Validation Case 3: No Care Weights (child dies from warmth/bonding deficit)"""
import os, sys, csv, random, numpy as np
sys.path.insert(0, os.path.dirname(__file__))
from core.world import World
from BASELINE_0_LOWFEAR_SEARCH_CONFIG import get_phase_3_weights

OUTPUT_DIR = "test_results/validation/survival_mechanics"
os.makedirs(OUTPUT_DIR, exist_ok=True)

result = []
seed = 43
random.seed(seed)
np.random.seed(seed)

# Get baseline weights then near-zero out care (avoid division by zero)
weights = get_phase_3_weights(low_fear_scale_factor=0.6)
weights['care'] = {'child_warmth': 0.001, 'closeness_deficit': 0.001, 'bonding': 0.001}

world = World(
    grid_w=20, grid_h=20,
    mother_starts=[(10, 10)],
    child_start=[(12, 12)],
    food_positions=[(5, 5), (15, 5), (5, 15), (15, 15)],  # Food present
    threat_starts=[],
    seed=seed,
    day_step=100,
    use_fixed_weights=False,
    baseline_weights=weights,
)

child_died_tick = None
child_warmth_at_death = None

for tick in range(3000):
    if world.children and world.children[0].alive:
        child = world.children[0]
        if not child.alive and child_died_tick is None:
            child_died_tick = tick
            child_warmth_at_death = child.warmth
    else:
        if child_died_tick is None:
            child_died_tick = tick
            if world.children:
                child_warmth_at_death = world.children[0].warmth
        break
    world.step()

result.append({
    'seed': seed,
    'tick_died': child_died_tick if child_died_tick else 3000,
    'warmth_at_death': child_warmth_at_death if child_warmth_at_death else 'N/A',
    'cause': 'WARMTH_DEFICIT' if child_died_tick else 'SURVIVED'
})

with open(os.path.join(OUTPUT_DIR, 'case_3_no_care_weights.csv'), 'w', newline='') as f:
    writer = csv.DictWriter(f, fieldnames=['seed', 'tick_died', 'warmth_at_death', 'cause'])
    writer.writeheader()
    writer.writerows(result)

print(f"Case 3 complete: child died at tick {child_died_tick}")
