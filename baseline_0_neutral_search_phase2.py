#!/usr/bin/env python3
"""
Baseline-0 Neutral Weight Search: Phase 2 Fine-Grained Refinement
==================================================================

Refines the search around the transition zone discovered in Phase 1.

Phase 2 Parameters (USER-APPROVED):
- Alpha values: {0.65, 0.70, 0.75, 0.80, 0.85, 0.90, 0.95}
- Replicates per alpha: 30 (vs 10 in Phase 1)
- Total runs: 210
- Duration: ~7-10 minutes
- Goal: Identify neutral point with minimal variance in 40-60% band

Phase 1 Key Findings (Corrected Interpretation):
- Lower α → Care motivation AMPLIFIED due to normalization (counter-intuitive!)
- α=0.7: Care sum=2.1, Forage sum=1.5 → care_sum slightly > forage_sum
- α=0.9: Care sum=2.7, Forage sum=1.5 → care clearly competitive
- Transition occurs α ∈ [0.7, 0.9] with α=0.7 at edge of 40-60% band
- High variance at α=0.7 (±45%) suggests bifurcation behavior
- Variance appears to drop toward α≥0.9 as outcomes become deterministic

Phase 2 Hypothesis:
- Lowest-variance neutral point likely α ∈ [0.75, 0.85]
- As we move toward α=0.9, variance decreases but success rate goes over 60%
- Goal: Find the sweet spot where 40-60% overlap exists with lower variance
"""

import os
import sys
import csv
import time
import random
import numpy as np
from collections import defaultdict

sys.path.insert(0, os.path.dirname(__file__))

from core.world import World
from func.BASELINE_0_WEIGHT_CONFIG import get_neutral_motivation_weights, log_weight_configuration

# === PHASE 2 CONFIGURATION (USER-APPROVED) ===
PHASE_2_PARAMS = {
    "alpha_values": [0.65, 0.70, 0.75, 0.80, 0.85, 0.90, 0.95],
    "replicates_per_alpha": 30,  # Higher than Phase 1 for lower variance
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
    "mother_survival_min": 0.99,
}

OUTPUT_DIR = "test_results/baseline_0_neutral_search_phase2"

# === METRICS TRACKING ===
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
        
        self.mother_energies = []
        self.child_energies = []
        self.child_hunger_samples = []
        self.child_warmth_samples = []
        self.child_injury_samples = []
        self.mother_bonding_samples = []
        self.mother_stress_samples = []
        self.mother_fear_samples = []
        
    def finalize(self, total_ticks):
        """Convert counts to percentages."""
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
        
        if self.mother_energies:
            self.mother_mean_energy = np.mean(self.mother_energies)
            self.mother_std_energy = np.std(self.mother_energies)
        else:
            self.mother_mean_energy = self.mother_std_energy = 0.0


def run_single_replicate(alpha, seed, grid_size=20, sim_ticks=3000):
    """Run a single replicate at given alpha."""
    random.seed(seed)
    np.random.seed(seed)
    
    baseline_weights = get_neutral_motivation_weights(care_scale_factor=alpha)
    
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
        use_fixed_weights=False,
        baseline_weights=baseline_weights,
    )
    
    metrics = SimulationReplicateMetrics()
    
    for tick in range(sim_ticks):
        if world.mothers and world.children:
            mother = world.mothers[0]
            child = world.children[0]
            
            metrics.forage_selection_count += (mother.selected_motivation == "Forage")
            metrics.care_selection_count += (mother.selected_motivation == "Care")
            metrics.self_selection_count += (mother.selected_motivation == "Self")
            metrics.protect_selection_count += (mother.selected_motivation == "Protect")
            metrics.total_selections += 1
            
            metrics.mother_energies.append(mother.energy)
            if child.alive:
                metrics.child_energies.append(child.energy)
                metrics.child_alive_ticks += 1
                metrics.mother_alive_ticks += 1
                metrics.child_hunger_samples.append(child.hunger)
                metrics.child_warmth_samples.append(child.warmth)
                metrics.child_injury_samples.append(child.injury)
                metrics.mother_bonding_samples.append(mother.bonding)
                metrics.mother_stress_samples.append(mother.stress)
                metrics.mother_fear_samples.append(mother.fear_threat)
            else:
                metrics.mother_alive_ticks += 1
        
        world.step()
    
    metrics.finalize(sim_ticks)
    return metrics


def run_phase_2_fine_grid():
    """Execute Phase 2: Fine-grained refinement search."""
    if not os.path.exists(OUTPUT_DIR):
        os.makedirs(OUTPUT_DIR)
    
    print("\n" + "="*80)
    print("BASELINE-0 NEUTRAL WEIGHT SEARCH: PHASE 2 FINE-GRAINED REFINEMENT")
    print("="*80)
    print(f"Alpha values: {PHASE_2_PARAMS['alpha_values']}")
    print(f"Replicates per alpha: {PHASE_2_PARAMS['replicates_per_alpha']}")
    print(f"Total simulation runs: {len(PHASE_2_PARAMS['alpha_values']) * PHASE_2_PARAMS['replicates_per_alpha']}")
    print(f"Output directory: {OUTPUT_DIR}")
    print(f"Neutrality target: {NEUTRALITY_CRITERIA['child_survival_min']:.0%} - {NEUTRALITY_CRITERIA['child_survival_max']:.0%} child survival")
    print(f"Corrected interpretation: Lower α → Care amplified by normalization")
    print("="*80 + "\n")
    
    all_results = []
    alpha_summary = defaultdict(list)
    
    total_runs = len(PHASE_2_PARAMS['alpha_values']) * PHASE_2_PARAMS['replicates_per_alpha']
    run_count = 0
    start_time = time.time()
    
    for alpha in PHASE_2_PARAMS['alpha_values']:
        print(f"\nAlpha = {alpha:.2f}")
        print("-" * 80)
        
        alpha_results = []
        for rep in range(PHASE_2_PARAMS['replicates_per_alpha']):
            run_count += 1
            seed = hash((alpha, rep)) % (2**31)
            
            metrics = run_single_replicate(
                alpha=alpha,
                seed=seed,
                grid_size=PHASE_2_PARAMS['grid_size'],
                sim_ticks=PHASE_2_PARAMS['simulation_ticks'],
            )
            
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
            }
            all_results.append(result_dict)
            alpha_summary[alpha].append(metrics)
            alpha_results.append(result_dict)
            
            elapsed = time.time() - start_time
            eta = (total_runs - run_count) * (elapsed / max(1, run_count))
            
            if (rep + 1) % 10 == 0 or rep == PHASE_2_PARAMS['replicates_per_alpha'] - 1:
                print(f"  Rep {rep+1:2d}/{PHASE_2_PARAMS['replicates_per_alpha']}: child_surv={metrics.child_survival:.2%}  " +
                      f"forage={metrics.forage_percent:5.1f}%  care={metrics.care_percent:5.1f}%  " +
                      f"[{run_count}/{total_runs}] ETA {eta/60:.1f}m")
        
        # Summary for this alpha
        metrics_list = alpha_summary[alpha]
        child_survs = [m.child_survival for m in metrics_list]
        forage_pcts = [m.forage_percent for m in metrics_list]
        care_pcts = [m.care_percent for m in metrics_list]
        
        print(f"  Summary for α={alpha:.2f}:")
        print(f"    Child survival:  {np.mean(child_survs):.2%} ± {np.std(child_survs):.2%}  " +
              f"(range: {np.min(child_survs):.2%} - {np.max(child_survs):.2%})")
        print(f"    Forage:          {np.mean(forage_pcts):6.1f}% ± {np.std(forage_pcts):5.1f}%")
        print(f"    Care:            {np.mean(care_pcts):6.1f}% ± {np.std(care_pcts):5.1f}%")
        
        # Check neutrality
        in_band = (
            NEUTRALITY_CRITERIA['child_survival_min'] <= np.mean(child_survs) <=  NEUTRALITY_CRITERIA['child_survival_max'] and
            NEUTRALITY_CRITERIA['forage_percent_min'] <= np.mean(forage_pcts) <= NEUTRALITY_CRITERIA['forage_percent_max'] and
            NEUTRALITY_CRITERIA['care_percent_min'] <= np.mean(care_pcts) <= NEUTRALITY_CRITERIA['care_percent_max']
        )
        variance_status = f"std={np.std(child_survs):.1%}" 
        print(f"    Neutrality band: {'YES' if in_band else 'NO'} ({variance_status})")
    
    # Write full results
    results_csv = os.path.join(OUTPUT_DIR, "phase_2_full_results.csv")
    if all_results:
        fieldnames = list(all_results[0].keys())
        with open(results_csv, 'w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(all_results)
        print(f"\n[OK] Full results written to: {results_csv}")
    
    # Write summary stats
    summary_csv = os.path.join(OUTPUT_DIR, "phase_2_summary.csv")
    with open(summary_csv, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow([
            'alpha',
            'child_survival_mean', 'child_survival_std', 'child_survival_min', 'child_survival_max',
            'forage_percent_mean', 'forage_percent_std',
            'care_percent_mean', 'care_percent_std',
            'in_neutrality_band',
            'variance_trend',
        ])
        
        for alpha in PHASE_2_PARAMS['alpha_values']:
            metrics_list = alpha_summary[alpha]
            child_survs = [m.child_survival for m in metrics_list]
            forage_pcts = [m.forage_percent for m in metrics_list]
            care_pcts = [m.care_percent for m in metrics_list]
            
            mean_child = np.mean(child_survs)
            std_child = np.std(child_survs)
            
            in_band = (
                NEUTRALITY_CRITERIA['child_survival_min'] <= mean_child <= NEUTRALITY_CRITERIA['child_survival_max'] and
                NEUTRALITY_CRITERIA['forage_percent_min'] <= np.mean(forage_pcts) <= NEUTRALITY_CRITERIA['forage_percent_max'] and
                NEUTRALITY_CRITERIA['care_percent_min'] <= np.mean(care_pcts) <= NEUTRALITY_CRITERIA['care_percent_max']
            )
            
            writer.writerow([
                f"{alpha:.2f}",
                f"{mean_child:.3f}", f"{std_child:.3f}",
                f"{np.min(child_survs):.3f}", f"{np.max(child_survs):.3f}",
                f"{np.mean(forage_pcts):.1f}", f"{np.std(forage_pcts):.1f}",
                f"{np.mean(care_pcts):.1f}", f"{np.std(care_pcts):.1f}",
                "YES" if in_band else "NO",
                f"mean={mean_child:.1%} std={std_child:.1%}",
            ])
    
    print(f"[OK] Summary stats written to: {summary_csv}\n")
    
    # Final summary
    print("\n" + "="*80)
    print("PHASE 2 COMPLETE: REFINED RESULTS")
    print("="*80)
    
    for alpha in PHASE_2_PARAMS['alpha_values']:
        metrics_list = alpha_summary[alpha]
        child_survs = [m.child_survival for m in metrics_list]
        forage_pcts = [m.forage_percent for m in metrics_list]
        care_pcts = [m.care_percent for m in metrics_list]
        
        mean_child = np.mean(child_survs)
        std_child = np.std(child_survs)
        mean_forage = np.mean(forage_pcts)
        mean_care = np.mean(care_pcts)
        
        in_band = (
            NEUTRALITY_CRITERIA['child_survival_min'] <= mean_child <= NEUTRALITY_CRITERIA['child_survival_max']
        )
        
        status = "OK" if in_band else "  "
        print(f"α={alpha:.2f} {status}  child={mean_child:.2%}±{std_child:.2%}  " +
              f"forage={mean_forage:5.1f}%  care={mean_care:5.1f}%")
    
    print("="*80)
    print("\nPhase 2 Interpretation Guidance:")
    print("-" * 80)
    print("""
Based on Phase 1 corrected interpretation:
- Lower care weight sums (lower α) → Care motivation amplified
- Higher care weight sums (higher α) → Care motivation reduced
- Optimal neutral point balances both survival and caregiving needs

Expected patterns:
- α=0.65-0.70: Care still dominant, forage low → child likely dies
- α=0.75-0.80: Transitional region → mixed outcomes (50/50 success?  
- α=0.85-0.90: Care/forage balanced → mostly success toward 100%
- α=0.95:    Approaching saturation → near 100% success

If lowest variance in 40-60% band found:
→ That is your RECOMMENDED NEUTRAL POINT

If no point in 40-60% with low variance:
→ System may be fundamentally bifurcating, not stable neutral
→ Report: "Bifurcation detected; true neutral point may not exist"
""")
    
    print("="*80 + "\n")


if __name__ == "__main__":
    run_phase_2_fine_grid()
