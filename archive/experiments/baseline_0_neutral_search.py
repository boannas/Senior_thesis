#!/usr/bin/env python3
"""
Baseline-0 Neutral Weight Search: Phase 1 Coarse Grid Search
=============================================================

Systematically searches for Care weight scale factor (α) that produces
neutral behavioral balance: ~40-60% child survival, ~35-55% forage,
~25-45% care, with low variance and 100% mother survival.

Phase 1: Coarse grid sweep
- α values: [0.1, 0.3, 0.5, 0.7, 0.9, 1.1, 1.3, 1.5, 1.7, 1.9]
- Replicates per point: 10
- Total runs: 100
- Duration: ~5-10 minutes

USER DECISION CHECKPOINTS:
- Neutrality range approved: 40-60% child survival  ✓
- Search parameter fixed: Care weights α only      ✓
- Forage.low_fear fixed at: 0.5                     ✓
- Replicates per point: 10 (coarse)                 ✓
"""

import os
import sys
import csv
import time
import random
import numpy as np
from collections import defaultdict

# Add workspace to path
sys.path.insert(0, os.path.dirname(__file__))

from core.world import World
from func.BASELINE_0_WEIGHT_CONFIG import get_neutral_motivation_weights, get_phase_1_parameter_grid, log_weight_configuration

# === CONFIGURATION ===
PHASE_1_PARAMS = {
    "alpha_values": get_phase_1_parameter_grid(),  # [0.1, 0.3, ..., 1.9]
    "replicates_per_alpha": 10,
    "simulation_ticks": 3000,
    "grid_size": 20,
}

NEUTRALITY_CRITERIA = {
    "child_survival_min": 0.40,
    "child_survival_max": 0.60,
    "forage_percent_min": 0.35,
    "forage_percent_max": 0.55,
    "care_percent_min": 0.25,
    "care_percent_max": 0.45,
    "mother_survival_min": 0.99,  # Virtually guaranteed
}

OUTPUT_DIR = "test_results/baseline_0_neutral_search"


# === STATE TRACKING ===
class SimulationReplicateMetrics:
    """Tracks metrics for a single simulation replicate."""
    
    def __init__(self):
        self.mother_alive_ticks = 0
        self.child_alive_ticks = 0
        self.total_ticks = 0
        self.forage_selection_count = 0
        self.care_selection_count = 0
        self.self_selection_count = 0
        self.protect_selection_count = 0
        self.total_selections = 0
        
        # Energy tracking
        self.mother_energies = []
        self.child_energies = []
        
        # Child state tracking
        self.child_hunger_samples = []
        self.child_warmth_samples = []
        self.child_injury_samples = []
        
        # Mother state tracking
        self.mother_bonding_samples = []
        self.mother_stress_samples = []
        self.mother_fear_samples = []
        
    def finalize(self, total_ticks):
        """Convert counts to percentages and compute summary stats."""
        self.total_ticks = total_ticks
        self.mother_survival = self.mother_alive_ticks / max(1, total_ticks)
        self.child_survival = self.child_alive_ticks / max(1, total_ticks)
        
        if self.total_selections > 0:
            self.forage_percent = 100.0 * self.forage_selection_count / self.total_selections
            self.care_percent = 100.0 * self.care_selection_count / self.total_selections
            self.self_percent = 100.0 * self.self_selection_count / self.total_selections
            self.protect_percent = 100.0 * self.protect_selection_count / self.total_selections
        else:
            self.forage_percent = self.care_percent = self.self_percent = self.protect_percent = 0.0
        
        # Summary stats
        if self.mother_energies:
            self.mother_mean_energy = np.mean(self.mother_energies)
            self.mother_std_energy = np.std(self.mother_energies)
        else:
            self.mother_mean_energy = self.mother_std_energy = 0.0
            
        if self.child_energies:
            self.child_mean_energy = np.mean(self.child_energies)
            self.child_std_energy = np.std(self.child_energies)
        else:
            self.child_mean_energy = self.child_std_energy = 0.0
            
        if self.mother_bonding_samples:
            self.mother_mean_bonding = np.mean(self.mother_bonding_samples)
            self.mother_mean_stress = np.mean(self.mother_stress_samples)
            self.mother_mean_fear = np.mean(self.mother_fear_samples)
        else:
            self.mother_mean_bonding = self.mother_mean_stress = self.mother_mean_fear = 0.0
            
        if self.child_hunger_samples:
            self.child_mean_hunger = np.mean(self.child_hunger_samples)
            self.child_mean_warmth = np.mean(self.child_warmth_samples)
            self.child_mean_injury = np.mean(self.child_injury_samples)
        else:
            self.child_mean_hunger = self.child_mean_warmth = self.child_mean_injury = 0.0


def run_single_replicate(alpha, seed, grid_size=20, sim_ticks=3000):
    """
    Run a single simulation replicate with given alpha (care scale factor).
    
    Returns: SimulationReplicateMetrics object
    """
    random.seed(seed)
    np.random.seed(seed)
    
    # Get weights using BASELINE_0_WEIGHT_CONFIG
    baseline_weights = get_neutral_motivation_weights(care_scale_factor=alpha)
    
    # Create world with baseline weights
    world = World(
        grid_w=grid_size,
        grid_h=grid_size,
        mother_starts=[(10, 10)],
        child_start=[(12, 12)],
        food_positions=[(5, 5), (15, 5), (5, 15), (15, 15)],
        threat_starts=[],
        seed=seed,
        day_step=100,
        plasticity_rule="outcome",
        food_spawn_interval=None,
        food_spawn_n=1,
        use_fixed_weights=False,  # Still allow plastic learning
        baseline_weights=baseline_weights,  # Pass custom weights
    )
    
    metrics = SimulationReplicateMetrics()
    
    # Run simulation
    for tick in range(sim_ticks):
        # Track agent states
        if world.mothers and world.children:
            mother = world.mothers[0]
            child = world.children[0]
            
            # Record selection (before this tick's policy update)
            metrics.forage_selection_count += (mother.selected_motivation == "Forage")
            metrics.care_selection_count += (mother.selected_motivation == "Care")
            metrics.self_selection_count += (mother.selected_motivation == "Self")
            metrics.protect_selection_count += (mother.selected_motivation == "Protect")
            metrics.total_selections += 1
            
            # Record energy and psychological states
            metrics.mother_energies.append(mother.energy)
            if child.alive:
                metrics.child_energies.append(child.energy)
                metrics.child_alive_ticks += 1
                metrics.mother_alive_ticks += 1
                metrics.mother_bonding_samples.append(mother.bonding)
                metrics.mother_stress_samples.append(mother.stress)
                metrics.mother_fear_samples.append(mother.fear_threat)
                metrics.child_hunger_samples.append(child.hunger)
                metrics.child_warmth_samples.append(child.warmth)
                metrics.child_injury_samples.append(child.injury)
            else:
                metrics.mother_alive_ticks += 1
        
        # Step simulation
        world.step()
    
    # Finalize metrics
    metrics.finalize(sim_ticks)
    return metrics


def run_phase_1_coarse_grid():
    """
    Execute Phase 1: Coarse grid sweep over alpha values.
    
    For each alpha:
    - Run 10 replicates
    - Record: child_survival, forage%, care%, variance
    - Output: summary stats + full data
    """
    if not os.path.exists(OUTPUT_DIR):
        os.makedirs(OUTPUT_DIR)
    
    print("\n" + "="*70)
    print("BASELINE-0 NEUTRAL WEIGHT SEARCH: PHASE 1 COARSE GRID")
    print("="*70)
    print(f"Alpha values: {PHASE_1_PARAMS['alpha_values']}")
    print(f"Replicates per alpha: {PHASE_1_PARAMS['replicates_per_alpha']}")
    print(f"Total simulation runs: {len(PHASE_1_PARAMS['alpha_values']) * PHASE_1_PARAMS['replicates_per_alpha']}")
    print(f"Output directory: {OUTPUT_DIR}")
    print(f"Neutrality target: {NEUTRALITY_CRITERIA['child_survival_min']:.0%} - {NEUTRALITY_CRITERIA['child_survival_max']:.0%} child survival")
    print("="*70 + "\n")
    
    # Storage for all results
    all_results = []
    alpha_summary = defaultdict(list)  # alpha -> list of metrics
    
    # Phase 1 execution
    total_runs = len(PHASE_1_PARAMS['alpha_values']) * PHASE_1_PARAMS['replicates_per_alpha']
    run_count = 0
    start_time = time.time()
    
    for alpha in PHASE_1_PARAMS['alpha_values']:
        print(f"\nAlpha = {alpha:.1f}")
        print("-" * 50)
        
        alpha_results = []
        for rep in range(PHASE_1_PARAMS['replicates_per_alpha']):
            run_count += 1
            seed = hash((alpha, rep)) % (2**31)  # Deterministic seed
            
            # Run replicate
            metrics = run_single_replicate(
                alpha=alpha,
                seed=seed,
                grid_size=PHASE_1_PARAMS['grid_size'],
                sim_ticks=PHASE_1_PARAMS['simulation_ticks'],
            )
            
            # Store results
            result_dict = {
                'alpha': alpha,
                'replicate': rep,
                'seed': seed,
                'child_survival': metrics.child_survival,
                'mother_survival': metrics.mother_survival,
                'forage_percent': metrics.forage_percent,
                'care_percent': metrics.care_percent,
                'self_percent': metrics.self_percent,
                'protect_percent': metrics.protect_percent,
                'mother_mean_energy': metrics.mother_mean_energy,
                'child_mean_energy': metrics.child_mean_energy,
                'mother_mean_bonding': metrics.mother_mean_bonding,
                'mother_mean_stress': metrics.mother_mean_stress,
                'mother_mean_fear': metrics.mother_mean_fear,
                'child_mean_hunger': metrics.child_mean_hunger,
                'child_mean_warmth': metrics.child_mean_warmth,
                'child_mean_injury': metrics.child_mean_injury,
            }
            all_results.append(result_dict)
            alpha_summary[alpha].append(metrics)
            alpha_results.append(result_dict)
            
            elapsed = time.time() - start_time
            eta = (total_runs - run_count) * (elapsed / max(1, run_count))
            print(f"  Rep {rep+1:2d}: child_surv={metrics.child_survival:.2%}  " +
                  f"forage={metrics.forage_percent:5.1f}%  care={metrics.care_percent:5.1f}%  " +
                  f"[{run_count}/{total_runs}] ETA {eta/60:.1f}m")
        
        # Compute alpha summary
        metrics_list = alpha_summary[alpha]
        child_survs = [m.child_survival for m in metrics_list]
        forage_pcts = [m.forage_percent for m in metrics_list]
        care_pcts = [m.care_percent for m in metrics_list]
        
        print(f"  Summary for α={alpha}:")
        print(f"    Child survival: {np.mean(child_survs):.2%} ± {np.std(child_survs):.2%}")
        print(f"    Forage:        {np.mean(forage_pcts):6.1f}% ± {np.std(forage_pcts):5.1f}%")
        print(f"    Care:          {np.mean(care_pcts):6.1f}% ± {np.std(care_pcts):5.1f}%")
    
    # Write full results to CSV
    results_csv = os.path.join(OUTPUT_DIR, "phase_1_full_results.csv")
    if all_results:
        fieldnames = list(all_results[0].keys())
        with open(results_csv, 'w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(all_results)
        print(f"\n✓ Full results written to: {results_csv}")
    
    # Write summary stats per alpha
    summary_csv = os.path.join(OUTPUT_DIR, "phase_1_summary.csv")
    with open(summary_csv, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow([
            'alpha',
            'child_survival_mean', 'child_survival_std', 'child_survival_min', 'child_survival_max',
            'forage_percent_mean', 'forage_percent_std',
            'care_percent_mean', 'care_percent_std',
            'mother_survival_mean',
            'within_neutrality_band',
        ])
        
        for alpha in PHASE_1_PARAMS['alpha_values']:
            metrics_list = alpha_summary[alpha]
            child_survs = [m.child_survival for m in metrics_list]
            forage_pcts = [m.forage_percent for m in metrics_list]
            care_pcts = [m.care_percent for m in metrics_list]
            mother_survs = [m.mother_survival for m in metrics_list]
            
            mean_child_surv = np.mean(child_survs)
            within_band = (
                NEUTRALITY_CRITERIA['child_survival_min'] <= mean_child_surv <= NEUTRALITY_CRITERIA['child_survival_max'] and
                NEUTRALITY_CRITERIA['forage_percent_min'] <= np.mean(forage_pcts) <= NEUTRALITY_CRITERIA['forage_percent_max'] and
                NEUTRALITY_CRITERIA['care_percent_min'] <= np.mean(care_pcts) <= NEUTRALITY_CRITERIA['care_percent_max']
            )
            
            writer.writerow([
                f"{alpha:.1f}",
                f"{np.mean(child_survs):.3f}", f"{np.std(child_survs):.3f}",
                f"{np.min(child_survs):.3f}", f"{np.max(child_survs):.3f}",
                f"{np.mean(forage_pcts):.1f}", f"{np.std(forage_pcts):.1f}",
                f"{np.mean(care_pcts):.1f}", f"{np.std(care_pcts):.1f}",
                f"{np.mean(mother_survs):.3f}",
                "YES" if within_band else "NO",
            ])
    
    print(f"✓ Summary stats written to: {summary_csv}\n")
    
    # Print key findings
    print("\n" + "="*70)
    print("PHASE 1 COMPLETE: KEY FINDINGS")
    print("="*70)
    
    for alpha in PHASE_1_PARAMS['alpha_values']:
        metrics_list = alpha_summary[alpha]
        child_survs = [m.child_survival for m in metrics_list]
        forage_pcts = [m.forage_percent for m in metrics_list]
        care_pcts = [m.care_percent for m in metrics_list]
        
        mean_child = np.mean(child_survs)
        std_child = np.std(child_survs)
        mean_forage = np.mean(forage_pcts)
        mean_care = np.mean(care_pcts)
        
        status = "✓ NEUTRAL" if (
            NEUTRALITY_CRITERIA['child_survival_min'] <= mean_child <= NEUTRALITY_CRITERIA['child_survival_max']
        ) else "  "
        
        print(f"α={alpha:.1f} {status}  " +
              f"child={mean_child:.2%}±{std_child:.2%}  " +
              f"forage={mean_forage:5.1f}%  care={mean_care:5.1f}%")
    
    print("="*70 + "\n")
    
    # Final recommendation
    print("RECOMMENDATION FOR PHASE 2:")
    print("-" * 70)
    print("Based on the coarse grid results:")
    print("1. Identify the alpha value(s) that fall within the neutrality band")
    print("2. If transition region is clear, recommend ~6 values around transition")
    print("3. Run 30 replicates per value for higher statistical power")
    print("4. If no transition visible, may need different search strategy")
    print("="*70 + "\n")


if __name__ == "__main__":
    run_phase_1_coarse_grid()
