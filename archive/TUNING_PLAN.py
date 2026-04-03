"""
TUNING PLAN: Beyond Baseline
=============================

This document outlines future experiments to systematically explore:
1. Parameter sensitivity
2. Robust regions
3. Failure modes
4. Optimal configuration ranges

Based on: 30-replicate baseline stability test
Date: 2026-04-03
"""

import yaml
from pathlib import Path

TUNING_PLAN = {
    
    "Phase_1_Perception_Sensitivity": {
        "objective": "How does bounded perception affect search difficulty and forage/care balance?",
        "rationale": "Current baseline uses perception=8. Test if this is optimal or if other values reveal new behaviors.",
        "parameters_to_vary": {
            "perception_range": {
                "baseline": 8,
                "test_values": [3, 5, 8, 12, 15],
                "hypothesis": "Smaller range → harder search → higher forage time; larger range → trivial search",
            }
        },
        "fixed_parameters": {
            "grid_size": "20x20",
            "move_cost_energy": -0.3,
            "n_food_initial": 1,
            "n_threats": 0,
            "max_ticks": 3000,
            "n_replicates": 30,
        },
        "output_metrics": [
            "child_survival_rate",
            "child_avg_hunger",
            "mother_avg_energy",
            "forage_percentage",
            "care_percentage",
            "search_time_to_food",
        ],
        "expected_results": {
            "perception=3": "Extreme search difficulty; forage dominates (>90%); child may starve",
            "perception=5": "High search difficulty; forage ~70-80%",
            "perception=8": "Baseline; forage ~75%, care ~23%",
            "perception=12": "Lower search difficulty; forage ~60%, care increases",
            "perception=15": "Trivial search; forage ~40-50%",
        },
        "failure_mode_check": "Does any perception range cause child starvation rate >50%?",
        "robust_region": "Find stable range where child survival >66% AND forage/care balance exists",
    },
    
    "Phase_2_Movement_Cost_Sensitivity": {
        "objective": "How does movement cost affect energy balance and behavioral flexibility?",
        "rationale": "Current baseline uses -0.3 (tuned from -1.0). Test if this is sweet spot.",
        "parameters_to_vary": {
            "move_cost_energy": {
                "baseline": -0.3,
                "test_values": [-0.15, -0.3, -0.5, -0.75, -1.0],
                "hypothesis": "Higher cost → mother fatigue/stress increase → self motivation increases → child care drops",
            }
        },
        "fixed_parameters": {
            "grid_size": "20x20",
            "perception_range": 8,
            "n_food_initial": 1,
            "n_threats": 0,
            "max_ticks": 3000,
            "n_replicates": 30,
        },
        "output_metrics": [
            "mother_avg_energy",
            "mother_avg_fatigue",
            "mother_avg_stress",
            "self_motivation_percentage",
            "child_survival_rate",
        ],
        "expected_results": {
            "move_cost=-0.15": "Mother has excess energy; Self motivation <1%; Care high",
            "move_cost=-0.3": "Baseline; balanced",
            "move_cost=-0.5": "Mother energy drops; Self motivation ~3-5%",
            "move_cost=-0.75": "Mother fatigue/stress increase; Self motivation ~10%",
            "move_cost=-1.0": "Original setting; mother may struggle; child care compromised",
        },
        "failure_mode_check": "Beyond what movement cost does mother energy spiral to 0?",
        "robust_region": "Find range where mother energy stays [20, 95] and Self <5%",
    },
    
    "Phase_3_Food_Availability_Sensitivity": {
        "objective": "How does food scarcity affect forage/care dynamics?",
        "rationale": "Baseline has 1 food. Test if more/less food changes behavior structure.",
        "parameters_to_vary": {
            "n_food_initial": {
                "baseline": 1,
                "test_values": [0, 1, 2, 3, 5],
                "hypothesis": "More food → easier forage → less care-pressure; less food → hard forage → more care",
            }
        },
        "fixed_parameters": {
            "grid_size": "20x20",
            "perception_range": 8,
            "move_cost_energy": -0.3,
            "n_threats": 0,
            "max_ticks": 3000,
            "n_replicates": 30,
        },
        "output_metrics": [
            "forage_percentage",
            "care_percentage",
            "child_avg_hunger",
            "child_survival_rate",
            "mother_avg_energy",
            "food_collection_frequency",
        ],
        "expected_results": {
            "n_food=0": "Only warmth-care possible; child starvation inevitable if not fed; baseline test FAILS",
            "n_food=1": "Baseline; forage ~75%, care ~23%",
            "n_food=2": "Easier forage; forage →80%, care →15%",
            "n_food=3": "Very easy foraging; forage →85%, minimal care",
            "n_food=5": "Trivial foraging; devolves to pure care logistics",
        },
        "failure_mode_check": "Does n_food=0 cause 100% child starvation?",
        "robust_region": "Find food count where both forage AND care occur meaningfully",
    },
    
    "Phase_4_Threat_Pressure": {
        "objective": "How do threats affect motivational hierarchy?",
        "rationale": "Baseline is threat-free. Introduce threats to test protection motivation.",
        "parameters_to_vary": {
            "n_threats": {
                "baseline": 0,
                "test_values": [0, 1, 2],
                "hypothesis": "Threats → protection motivation activates → forage/care disrupted",
            },
            "threat_perception_radius": {
                "baseline": 10,
                "test_values": [5, 10, 15],
                "hypothesis": "Larger radius → threats perceived from farther → more defensive behavior",
            }
        },
        "fixed_parameters": {
            "grid_size": "20x20",
            "perception_range": 8,
            "n_food_initial": 1,
            "move_cost_energy": -0.3,
            "max_ticks": 3000,
            "n_replicates": 20,
        },
        "output_metrics": [
            "protect_motivation_percentage",
            "fear_level_avg",
            "stress_level_avg",
            "child_injury_rate",
            "child_survival_rate",
            "mother_energy_drop_during_threat",
        ],
        "expected_results": {
            "n_threats=0": "Baseline; no fear/protect",
            "n_threats=1, radius=5": "Protect motivation activates only when close; ~2-5% time",
            "n_threats=1, radius=10": "More frequent threat detection; ~5-15% protect time",
            "n_threats=2, radius=15": "Constant threat awareness; >20% protect time; mother stress high",
        },
        "failure_mode_check": "Do threats cause mother to abandon child entirely? What's the tipping point?",
        "robust_region": "Find threat intensity where protection is real but doesn't collapse forage/care",
    },
    
    "Phase_5_Grid_Scale_Effects": {
        "objective": "How does world size affect search difficulty and outcomes?",
        "rationale": "Baseline 20x20. Test if 15x15 (smaller) or 30x30 (larger) changes behavior.",
        "parameters_to_vary": {
            "grid_size": {
                "baseline": "20x20",
                "test_values": ["15x15", "20x20", "30x30"],
                "note": "Also adjust perception_range to maintain ~0.4 ratio",
                "perception_ratios": {
                    "15x15": 6,   # diagonal ~21, ratio ~0.28
                    "20x20": 8,   # diagonal ~28, ratio ~0.29
                    "30x30": 12,  # diagonal ~42, ratio ~0.29
                }
            }
        },
        "fixed_parameters": {
            "move_cost_energy": -0.3,
            "n_food_initial": 1,
            "n_threats": 0,
            "max_ticks": 3000,
            "n_replicates": 20,
        },
        "output_metrics": [
            "search_time_to_food",
            "mother_avg_movement_per_tick",
            "forage_percentage",
            "care_percentage",
            "child_survival_rate",
        ],
        "expected_results": {
            "15x15": "Smaller grid = confined space; easier search; forage % high",
            "20x20": "Baseline",
            "30x30": "Larger grid = more space to search; harder navigation; forage % may increase due to search effort",
        },
        "robust_region": "Find scale where results don't change drastically; confirm scaling laws",
    },
    
    "Phase_6_Learning_Rate_Exploration": {
        "objective": "How fast should mothers adapt genetic weights via plasticity?",
        "rationale": "Baseline uses learning_rate=0.02 with plasticity_rule='fixed' (disabled).",
        "parameters_to_vary": {
            "learning_rate": {
                "baseline": 0.02,
                "test_values": [0, 0.001, 0.01, 0.02, 0.05, 0.1],
            },
            "plasticity_rule": {
                "baseline": "fixed",
                "test_values": ["fixed", "outcome"],
            }
        },
        "fixed_parameters": {
            "grid_size": "20x20",
            "perception_range": 8,
            "n_food_initial": 1,
            "n_threats": 0,
            "max_ticks": 3000,
            "n_replicates": 30,
        },
        "output_metrics": [
            "final_psych_weights_variance",
            "weight_drift_over_time",
            "child_survival_rate",
            "learning_convergence_rate",
        ],
        "expected_results": {
            "learning_rate=0, rule=fixed": "Baseline; no weight change",
            "learning_rate=0.001, rule=outcome": "Slow learning; weights drift slightly",
            "learning_rate=0.01, rule=outcome": "Moderate learning; 30% weight change by end",
            "learning_rate=0.05, rule=outcome": "Fast learning; weights converge quickly",
            "learning_rate=0.1, rule=outcome": "Very fast learning; may oscillate or become chaotic",
        },
        "robust_region": "Find learning rate where adaptation is visible but not chaotic",
    },
    
    "Phase_7_Child_Decay_Rate_Sensitivity": {
        "objective": "How do child decay rates (hunger, warmth loss) affect pressure dynamics?",
        "rationale": "Baseline: hunger +0.5/tick, warmth -0.2/tick. Test if time pressure is tuned correctly.",
        "parameters_to_vary": {
            "child_hunger_increase_rate": {
                "baseline": 0.5,
                "test_values": [0.2, 0.5, 1.0],
                "hypothesis": "Higher rate = starvation pressure; lower rate = relaxed condition",
            },
            "child_warmth_loss_rate_alone": {
                "baseline": 0.2,
                "test_values": [0.1, 0.2, 0.5],
                "hypothesis": "Higher rate = warmth crisis; lower rate = less care urgency",
            }
        },
        "fixed_parameters": {
            "grid_size": "20x20",
            "perception_range": 8,
            "n_food_initial": 1,
            "move_cost_energy": -0.3,
            "n_threats": 0,
            "max_ticks": 3000,
            "n_replicates": 20,
        },
        "output_metrics": [
            "time_to_child_critical_hunger",
            "time_to_child_critical_warmth",
            "care_percentage",
            "child_survival_rate",
        ],
        "expected_results": {
            "hunger_0.2, warmth_0.1": "Relaxed conditions; care <10%; child may thrive",
            "hunger_0.5, warmth_0.2": "Baseline; balanced pressure",
            "hunger_1.0, warmth_0.5": "High pressure; forage and care both necessary; stress high",
        },
        "robust_region": "Find rates where both forage and care are non-trivial",
    },
}

# ═══════════════════════════════════════════════════════════════════════
# EXPERIMENT EXECUTION ROADMAP
# ═══════════════════════════════════════════════════════════════════════

EXECUTION_ROADMAP = {
    
    "Immediate (Week 1)": [
        "Phase 1: Perception Sensitivity (5 values × 30 reps = 150 runs)",
        "Phase 2: Movement Cost (5 values × 30 reps = 150 runs)",
    ],
    
    "Short-term (Week 2-3)": [
        "Phase 3: Food Availability (5 values × 30 reps = 150 runs)",
        "Phase 4: Threat Pressure (5 values × 20 reps = 100 runs)",
    ],
    
    "Medium-term (Month 2)": [
        "Phase 5: Grid Scale (3 values × 20 reps = 60 runs)",
        "Phase 6: Learning Rate (6 values × 30 reps = 180 runs)",
    ],
    
    "As-needed": [
        "Phase 7: Child Decay Rates (3×3 grid = 9 combos × 20 reps = 180 runs)",
    ],
}

# ═══════════════════════════════════════════════════════════════════════
# ANALYSIS STRATEGY
# ═══════════════════════════════════════════════════════════════════════

ANALYSIS_STRATEGY = {
    
    "For each phase":
        """
        1. Run all parameter values × 30 replicates
        2. Compute summary statistics: mean ± SD for each metric
        3. Generate plots:
           - Line plot: metric vs parameter value (with error bars)
           - Heatmap: multiple metrics vs parameter (if 2D)
           - Histogram: outcome distribution for each parameter value
        4. Statistical test: ANOVA or Kruskal-Wallis to test parameter effect
        5. Identify robust region: parameter range where outcomes are stable
        """,
    
    "Identify failure modes":
        """
        - Look for bimodal or multimodal distributions
        - Note any parameter values causing 100% child deaths
        - Track what motivations become dominant
        - Check for energy spiraling (indicator of non-equilibrium)
        """,
    
    "Robustness region definition":
        """
        A region is 'robust' if:
        - Child survival rate > 66% (both agents survive 2/3 of baseline)
        - Both Forage and Care motivations active (>0.5% each)
        - Mother energy stays in [20, 95] range
        - Outcomes are stable across seeds (SD < mean/3)
        """,
}

# ═══════════════════════════════════════════════════════════════════════
# METRICS DEFINITIONS
# ═══════════════════════════════════════════════════════════════════════

METRICS = {
    
    "child_survival_rate": "Percentage of replicates where child = alive at max_ticks",
    
    "child_avg_hunger": "Mean child hunger over ticks (lower = better)",
    
    "mother_avg_energy": "Mean mother energy over ticks (stable = 30-95 range)",
    
    "forage_percentage": "Percentage of ticks where Forage motivation selected",
    
    "care_percentage": "Percentage of ticks where Care motivation selected",
    
    "search_time_to_food": "Average ticks until mother reaches food (lower = perception easier)",
    
    "mother_avg_fatigue": "Mean fatigue (indicator of rest sufficiency)",
    
    "mother_avg_stress": "Mean stress (indicator of environmental pressure)",
    
    "self_motivation_percentage": "Time spent on Self-care motivation",
    
    "protect_motivation_percentage": "Time spent on Protect motivation",
    
    "child_avg_injury": "Mean injury level (0 if no threats; >0 if threats present)",
}

"""
END TUNING PLAN
"""
