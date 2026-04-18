#!/usr/bin/env python3
"""Validation Case 2: No Food (Baseline-0 dies from starvation)"""
import os, sys, csv, random, numpy as np
sys.path.insert(0, os.path.dirname(__file__))
from core.world import World
from BASELINE_0_LOWFEAR_SEARCH_CONFIG import get_phase_3_weights

OUTPUT_DIR = "test_results/validation/survival_mechanics"
os.makedirs(OUTPUT_DIR, exist_ok=True)

result = []
seed = 42
random.seed(seed)
np.random.seed(seed)

weights = get_phase_3_weights(low_fear_scale_factor=0.6)
world = World(
    grid_w=20, grid_h=20,
    mother_starts=[(10, 10)],
    child_start=[(12, 12)],
    food_positions=[],  # NO FOOD
    threat_starts=[],
    seed=seed,
    day_step=100,
    use_fixed_weights=False,
    baseline_weights=weights,
)

child_died_tick = None
child_hunger_at_death = None

for tick in range(3000):
    if world.children and world.children[0].alive:
        child = world.children[0]
        if tick % 100 == 0 or not child.alive:
            if not child.alive and child_died_tick is None:
                child_died_tick = tick
                child_hunger_at_death = child.hunger
    else:
        if child_died_tick is None:
            child_died_tick = tick
            if world.children:
                child_hunger_at_death = world.children[0].hunger
        break
    world.step()

result.append({
    'seed': seed,
    'tick_died': child_died_tick if child_died_tick else 3000,
    'hunger_at_death': child_hunger_at_death if child_hunger_at_death else 'N/A',
    'cause': 'STARVATION' if child_died_tick else 'SURVIVED'
})

with open(os.path.join(OUTPUT_DIR, 'case_2_no_food.csv'), 'w', newline='') as f:
    writer = csv.DictWriter(f, fieldnames=['seed', 'tick_died', 'hunger_at_death', 'cause'])
    writer.writeheader()
    writer.writerows(result)

print(f"Case 2 complete: child died at tick {child_died_tick}")
