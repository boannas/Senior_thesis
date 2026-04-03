#!/usr/bin/env python3
"""
Baseline-0 Neutral Weight Search: Phase 3 Low_Fear Weight Sweep
(Corrected implementation: vary only low_fear weight, not entire forage group)

Hypothesis: Low_fear weight drives forage structural asymmetry.
            Suppressing it (γ < 1.0) should reduce bifurcation and enable
            stable operation in 40-60% child survival band.

Configuration: Use BASELINE_0_LOWFEAR_SEARCH_CONFIG.py as single source of truth
"""

import os
import sys
import csv
import time
import random
import json
import numpy as np
from collections import defaultdict

sys.path.insert(0, os.path.dirname(__file__))

from core.world import World
from BASELINE_0_LOWFEAR_SEARCH_CONFIG import (
    get_phase_3_weights,
    get_phase_3_parameter_grid,
    get_phase_3_fixed_settings,
    log_phase_3_configuration,
)

# ════════════════════════════════════════════════════════════════════════
# PHASE 3 CONFIGURATION (from config module - SINGLE SOURCE OF TRUTH)
# ════════════════════════════════════════════════════════════════════════

PHASE_3_PARAMS = {
    "gamma_values": get_phase_3_parameter_grid(),  # [0.1, 0.2, ..., 1.0]
    "replicates_per_gamma": 30,
    "simulation_ticks": 3000,
    "grid_size": 20,
}

NEUTRALITY_CRITERIA = {
    "child_survival_min": 0.40,
    "child_survival_max": 0.60,
}

OUTPUT_DIR = "test_results/baseline_0_neutral_search_phase3"
FIXED_SETTINGS = get_phase_3_fixed_settings()


# ════════════════════════════════════════════════════════════════════════
# METRICS CLASS
# ════════════════════════════════════════════════════════════════════════

class ReplicateMetrics:
    """Track metrics for a single simulation replicate."""
    
    def __init__(self):
        self.child_alive_ticks = 0
        self.mother_alive_ticks = 0
        self.forage_count = 0
        self.care_count = 0
        self.total_selections = 0
        
    def finalize(self, total_ticks):
        """Compute final percentages."""
        self.child_survival = self.child_alive_ticks / max(1, total_ticks)
        self.mother_survival = self.mother_alive_ticks / max(1, total_ticks)
        if self.total_selections > 0:
            self.forage_percent = 100.0 * self.forage_count / self.total_selections
            self.care_percent = 100.0 * self.care_count / self.total_selections
        else:
            self.forage_percent = self.care_percent = 0.0


# ════════════════════════════════════════════════════════════════════════
# RUN FUNCTION
# ════════════════════════════════════════════════════════════════════════

def run_replicate(low_fear_gamma, seed):
    """
    Run a single simulation replicate.
    
    Parameters
    ----------
    low_fear_gamma : float
        The low_fear weight scale factor (γ) for this run
    seed : int
        Random seed for reproducibility
        
    Returns
    -------
    ReplicateMetrics
        Metrics from the completed simulation
    """
    random.seed(seed)
    np.random.seed(seed)
    
    # Get phase 3 weights using config module
    baseline_weights = get_phase_3_weights(low_fear_scale_factor=low_fear_gamma)
    
    # Create and run world
    world = World(
        grid_w=PHASE_3_PARAMS["grid_size"],
        grid_h=PHASE_3_PARAMS["grid_size"],
        mother_starts=[(10, 10)],
        child_start=[(12, 12)],
        food_positions=[(5, 5), (15, 5), (5, 15), (15, 15)],
        threat_starts=[],
        seed=seed,
        day_step=100,
        plasticity_rule="outcome",
        food_spawn_interval=None,
        food_spawn_n=1,
        use_fixed_weights=False,
        baseline_weights=baseline_weights,
    )
    
    metrics = ReplicateMetrics()
    
    # Simulate
    for tick in range(PHASE_3_PARAMS["simulation_ticks"]):
        if world.mothers and world.children:
            mother = world.mothers[0]
            child = world.children[0]
            
            # Track motivation selection
            if mother.selected_motivation == "Forage":
                metrics.forage_count += 1
            elif mother.selected_motivation == "Care":
                metrics.care_count += 1
            metrics.total_selections += 1
            
            # Track survival
            if child.alive:
                metrics.child_alive_ticks += 1
            metrics.mother_alive_ticks += 1
        
        world.step()
    
    metrics.finalize(PHASE_3_PARAMS["simulation_ticks"])
    return metrics


# ════════════════════════════════════════════════════════════════════════
# MAIN PHASE 3 EXECUTION
# ════════════════════════════════════════════════════════════════════════

def run_phase3():
    """Execute Phase 3 low_fear weight sweep."""
    
    if not os.path.exists(OUTPUT_DIR):
        os.makedirs(OUTPUT_DIR)
    
    print("\n" + "="*80)
    print("BASELINE-0 PHASE 3: LOW_FEAR WEIGHT SWEEP")
    print("="*80)
    print(f"Objective: Suppress low_fear weight to escape bifurcation")
    print(f"Hypothesis: Lower gamma -> reduced forage asymmetry -> stable neutral region")
    print(f"\nConfiguration:")
    print(f"  Care scale (alpha):         {FIXED_SETTINGS['care_scale_alpha']}")
    print(f"  Child_hunger (fixed):       {FIXED_SETTINGS['forage_child_hunger']}")
    print(f"  Energy_deficit (fixed):     {FIXED_SETTINGS['forage_energy_deficit']}")
    print(f"  Low_fear (VARIES):          gamma ∈ {PHASE_3_PARAMS['gamma_values']}")
    print(f"\nExecution:")
    print(f"  Gamma values:               {len(PHASE_3_PARAMS['gamma_values'])}")
    print(f"  Replicates per gamma:       {PHASE_3_PARAMS['replicates_per_gamma']}")
    print(f"  Total runs:                 {len(PHASE_3_PARAMS['gamma_values']) * PHASE_3_PARAMS['replicates_per_gamma']}")
    print(f"  Expected duration:          ~10 minutes")
    print("="*80 + "\n")
    
    all_results = []
    gamma_data = defaultdict(list)
    
    total_runs = len(PHASE_3_PARAMS['gamma_values']) * PHASE_3_PARAMS['replicates_per_gamma']
    run_count = 0
    start_time = time.time()
    
    # ─────────────────────────────────────────────────────────────────
    # MAIN SEARCH LOOP
    # ─────────────────────────────────────────────────────────────────
    
    for gamma in PHASE_3_PARAMS['gamma_values']:
        forage_sum = 0.5 + 0.5 + gamma
        print(f"Gamma = {gamma:.1f}  (forage_sum = {forage_sum:.1f})")
        
        gamma_start = time.time()
        
        for rep in range(PHASE_3_PARAMS['replicates_per_gamma']):
            run_count += 1
            # Deterministic seed: hash(gamma, rep)
            seed = hash((gamma, rep)) % (2**31)
            
            # Run simulation
            metrics = run_replicate(gamma, seed)
            
            # Record result
            all_results.append({
                'gamma': gamma,
                'forage_sum': forage_sum,
                'replicate': rep,
                'seed': seed,
                'child_survival': metrics.child_survival,
                'forage_percent': metrics.forage_percent,
                'care_percent': metrics.care_percent,
            })
            gamma_data[gamma].append(metrics)
            
            # Progress reporting
            if run_count % 30 == 0 or run_count == 1:
                elapsed = time.time() - start_time
                if run_count > 1:
                    eta = (total_runs - run_count) * (elapsed / run_count)
                    print(f"  Run {run_count:3d}/{total_runs}: child={metrics.child_survival:.1%}  ETA {eta/60:.1f}m")
                else:
                    print(f"  Run {run_count:3d}/{total_runs}: child={metrics.child_survival:.1%}")
        
        # Summary for this gamma
        metrics_list = gamma_data[gamma]
        child_survs = [m.child_survival for m in metrics_list]
        forage_pcts = [m.forage_percent for m in metrics_list]
        care_pcts = [m.care_percent for m in metrics_list]
        
        mean_child = np.mean(child_survs)
        std_child = np.std(child_survs)
        
        in_band = (NEUTRALITY_CRITERIA['child_survival_min'] <= mean_child <= 
                   NEUTRALITY_CRITERIA['child_survival_max'])
        status = "IN_BAND" if in_band else "---"
        
        gamma_time = time.time() - gamma_start
        print(f"  --> child={mean_child:.1%}±{std_child:.1%}  forage={np.mean(forage_pcts):.1f}%  "
              f"{status}  ({gamma_time:.0f}s)")
    
    # ─────────────────────────────────────────────────────────────────
    # OUTPUT: FULL RESULTS CSV
    # ─────────────────────────────────────────────────────────────────
    
    csv_path = os.path.join(OUTPUT_DIR, "phase_3_results.csv")
    with open(csv_path, 'w', newline='', encoding='utf-8') as f:
        writer = csv.DictWriter(
            f,
            fieldnames=['gamma', 'forage_sum', 'replicate', 'seed', 'child_survival', 
                       'forage_percent', 'care_percent']
        )
        writer.writeheader()
        writer.writerows(all_results)
    print(f"\n[OK] Full results: {csv_path}")
    
    # ─────────────────────────────────────────────────────────────────
    # OUTPUT: SUMMARY CSV (one row per gamma)
    # ─────────────────────────────────────────────────────────────────
    
    summary_path = os.path.join(OUTPUT_DIR, "phase_3_summary.csv")
    with open(summary_path, 'w', newline='', encoding='utf-8') as f:
        writer = csv.writer(f)
        writer.writerow(['gamma', 'forage_sum', 'child_mean', 'child_std', 
                       'forage_mean', 'care_mean', 'in_band'])
        for gamma in PHASE_3_PARAMS['gamma_values']:
            metrics_list = gamma_data[gamma]
            child_survs = [m.child_survival for m in metrics_list]
            forage_pcts = [m.forage_percent for m in metrics_list]
            care_pcts = [m.care_percent for m in metrics_list]
            
            forage_sum = 0.5 + 0.5 + gamma
            mean_child = np.mean(child_survs)
            std_child = np.std(child_survs)
            in_band = (NEUTRALITY_CRITERIA['child_survival_min'] <= mean_child <= 
                      NEUTRALITY_CRITERIA['child_survival_max'])
            
            writer.writerow([
                f"{gamma:.1f}",
                f"{forage_sum:.1f}",
                f"{mean_child:.3f}",
                f"{std_child:.3f}",
                f"{np.mean(forage_pcts):.1f}",
                f"{np.mean(care_pcts):.1f}",
                "YES" if in_band else "NO",
            ])
    print(f"[OK] Summary:      {summary_path}")
    
    # ─────────────────────────────────────────────────────────────────
    # OUTPUT: CONFIGURATION LOG (for auditability)
    # ─────────────────────────────────────────────────────────────────
    
    config_path = os.path.join(OUTPUT_DIR, "phase_3_config.json")
    config_doc = {
        "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
        "phase": 3,
        "description": "Low_fear weight sweep to escape bifurcation",
        "gamma_values": PHASE_3_PARAMS['gamma_values'],
        "replicates_per_gamma": PHASE_3_PARAMS['replicates_per_gamma'],
        "total_runs": total_runs,
        "simulation_ticks": PHASE_3_PARAMS["simulation_ticks"],
        "grid_size": PHASE_3_PARAMS["grid_size"],
        "fixed_settings": FIXED_SETTINGS,
        "neutrality_criteria": NEUTRALITY_CRITERIA,
    }
    with open(config_path, 'w', encoding='utf-8') as f:
        json.dump(config_doc, f, indent=2)
    print(f"[OK] Config log:   {config_path}")
    
    # ─────────────────────────────────────────────────────────────────
    # CONSOLE SUMMARY TABLE
    # ─────────────────────────────────────────────────────────────────
    
    total_time = time.time() - start_time
    
    print("\n" + "="*90)
    print("PHASE 3 RESULTS SUMMARY")
    print("="*90)
    print(f"{'Gamma':<10} {'Forage':<10} {'Status':<10} {'Child%':<15} {'Variance':<12} {'Forage%':<10}")
    print("-"*90)
    
    for gamma in PHASE_3_PARAMS['gamma_values']:
        metrics_list = gamma_data[gamma]
        child_survs = [m.child_survival for m in metrics_list]
        forage_pcts = [m.forage_percent for m in metrics_list]
        
        mean_child = np.mean(child_survs)
        std_child = np.std(child_survs)
        mean_forage = np.mean(forage_pcts)
        forage_sum = 0.5 + 0.5 + gamma
        
        in_band = (NEUTRALITY_CRITERIA['child_survival_min'] <= mean_child <= 
                  NEUTRALITY_CRITERIA['child_survival_max'])
        status = "IN_BAND" if in_band else "----"
        
        print(f"{gamma:<10.1f} {forage_sum:<10.1f} {status:<10} "
              f"{mean_child*100:<14.1f}% ±{std_child*100:<10.1f}% {mean_forage:<10.1f}%")
    
    print("="*90)
    print(f"\nTotal time: {total_time/60:.1f} minutes")
    print(f"Total runs: {total_runs}")
    
    print("\n" + "="*90)
    print("NEXT STEPS:")
    print("  1. Review phase_3_summary.csv for all gamma values")
    print("  2. Identify gamma with:")
    print("     - Child survival in [40%, 60%]")
    print("     - Variance < 30% (stable, not bifurcated)")
    print("     - Smooth distribution (not binary)")
    print("  3. If found: Recommend baseline (alpha=0.70, gamma=X.X)")
    print("  4. If not found: Proceed to Option 2 or Phase 4 analysis")
    print("="*90 + "\n")


if __name__ == "__main__":
    run_phase3()
