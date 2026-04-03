"""
BASELINE CONTROL VARIABLES & PARAMETERS
========================================

This document defines:
1. LOCKED variables (constant across all baseline experiments)
2. EXPOSED parameters (can be tuned in future experiments)
3. DEPENDENCIES (coupling between variables)

Generated for: Senior Thesis Baseline (Neutral Conditions)
Date: 2026-04-03
Validation: 30-replicate stability test
"""

# ═══════════════════════════════════════════════════════════════════════
# SECTION 1: LOCKED CONTROL VARIABLES
# ═══════════════════════════════════════════════════════════════════════
# These parameters MUST remain constant to maintain baseline neutrality

CONTROL_VARIABLES = {
    
    # --- SPATIAL & TEMPORAL CONSTRAINTS ---
    "grid_width": 20,              # Fixed control variable (spatial constraint)
    "grid_height": 20,             # Fixed control variable (spatial constraint)
    "max_ticks": 3000,             # Fixed (30 simulated days at 100 ticks/day)
    "day_step": 100,               # Fixed (ticks per day for cycle tracking)
    
    # --- THREATS & SAFETY ---
    "n_threats": 0,                # Fixed (neutral: no predation pressure)
    "threat_starts": [],           # Fixed (empty list, baseline is threat-free)
    
    # --- INITIAL RESOURCES ---
    "n_food_initial": 1,           # Fixed (balanced: minimal but addressable resource)
    "food_spawn_interval": None,   # Fixed (static food environment, no spawning)
    "food_spawn_n": 0,             # Fixed (no periodic spawning)
    
    # --- PERCEPTION & SENSING ---
    "perception_range": 8,         # Fixed (bounded spatial awareness, ~octile distance)
    "perception_metric": "octile", # Fixed (8-directional distance)
    
    # --- ENERGY & FATIGUE DYNAMICS (TUNED FOR SUSTAINABILITY & NEUTRALITY) ---
    "move_cost_energy": -0.3,      # Fixed (was -1.0, reduced for care-strategy viability)
    "move_cost_fatigue": 0.5,      # Fixed (fatigue accumulation from movement)
    "idle_recovery_energy": 0.05,  # Fixed (was -0.05, reversed to enable sustainability)
    "idle_recovery_fatigue": -1.0, # Fixed (fatigue recovery during idle)
    "rest_recovery_energy": 0.1,   # Fixed (was -0.05, improved recovery when resting)
    "rest_recovery_fatigue": -3.0, # Fixed (fast fatigue recovery)
    "action_cost_energy": -0.1,    # Fixed (feeding, caring actions)
    "action_cost_fatigue": 0.1,    # Fixed
    
    # --- INITIAL AGENT STATES ---
    "mother_initial_energy": 100,  # Fixed (start healthy)
    "child_initial_hunger": 0,     # Fixed (start well-fed)
    "child_initial_warmth": 50,    # Fixed (start at ideal temperature)
    "child_initial_injury": 0,     # Fixed (start healthy)
    
    # --- BEHAVIORAL DYNAMICS ---
    "child_hunger_increase_rate": 0.5,         # Fixed per-tick hunger gain
    "child_warmth_gain_rate_carried": 0.1,    # Fixed warmth recovery when carried
    "child_warmth_loss_rate_alone": 0.2,      # Fixed warmth loss when alone
    "threat_recovery_radius": 5.0,            # Fixed (injury recovery distance)
    "threat_perception_radius": 10.0,         # Fixed (fear/cortisol trigger distance)
    
    # --- GENETIC/PHENOTYPIC VARIANCE ---
    "genetic_weight_initialization": "random_u01_with_seed",  # Fixed approach (random U(0,1) with seed=42)
    "learning_rate": 0.02,                    # Fixed base learning rate
    "plasticity_rule": "fixed",               # Fixed (no learning in baseline; use 'fixed' mode)
    
    # --- REPRODUCIBILITY ---
    "seed": 42,                               # Fixed base seed (replicates use seed + replicate_id)
    "rng_source": "numpy.random.seed",        # Fixed
    
    # --- HOMOSTATIC SETPOINTS ---
    "ideal_mother_energy": 80,
    "ideal_mother_fatigue": 0,
    "ideal_child_hunger": 0,
    "ideal_child_warmth": 50,
    "ideal_child_injury": 0,
}

# ═══════════════════════════════════════════════════════════════════════
# SECTION 2: RATIONALE FOR CONTROL VARIABLES
# ═══════════════════════════════════════════════════════════════════════

RATIONALE = {
    
    "grid_width & grid_height": 
        "20x20 provides sufficient space for search (perception=8 is non-trivial) "
        "while remaining computationally tractable. Larger grids make food search "
        "too hard; smaller grids make perception trivial.",
    
    "perception_range=8":
        "Balances realism with search difficulty. At 8 cells octile distance, "
        "mother can't see entire grid but must actively search. "
        "Smaller ranges (3-5) make search too hard; larger (20+) too trivial.",
    
    "n_food_initial=1":
        "Single food item enables forage motivation feedback without trivializing "
        "the problem. Zero makes forage useless; multiple food makes search unchallenging.",
    
    "n_threats=0":
        "Neutral condition (no predation). Enables focus on mother-child care dynamics. "
        "Threats can be added in future experiments to test protective conflict.",
    
    "move_cost_energy=-0.3":
        "Reduced from -1.0 to make care-strategy sustainable. At -1.0, mother "
        "starved before completing meaningful behavior. -0.3 allows energy recovery "
        "during idle periods while still making movement costly.",
    
    "idle_recovery_energy=+0.05":
        "Reversed from -0.05 (bug fix). Mother must recover energy during idle "
        "phases to achieve long-term equilibrium. Enables sustainability for 30 days.",
    
    "perception_range=8 vs grid=20":
        "Perception range ~40% of grid diagonal. Non-trivial search: mother can't "
        "see everything but grid is searchable in one day. This enforces spatial "
        "problem without creating extreme difficulty.",
}

# ═══════════════════════════════════════════════════════════════════════
# SECTION 3: PARAMETERS EXPOSED FOR TUNING
# ═══════════════════════════════════════════════════════════════════════
# These can vary in future experiments (beyond baseline)

TUNABLE_PARAMETERS = {
    
    # Perception & Search Difficulty
    "perception_range": {
        "baseline_value": 8,
        "tuning_range": [3, 5, 8, 10, 15],
        "rationale": "Test how search difficulty affects forage/care balance",
        "min": 1,
        "max": 25,
    },
    
    # Movement Cost & Energy Pressure
    "move_cost_energy": {
        "baseline_value": -0.3,
        "tuning_range": [-0.2, -0.3, -0.4, -0.6],
        "rationale": "Extract cost-benefit curve: higher cost → more pressure on locomotion",
        "min": -1.0,
        "max": -0.1,
    },
    
    # Resource Availability
    "n_food_initial": {
        "baseline_value": 1,
        "tuning_range": [0, 1, 2, 3, 5],
        "rationale": "Test food scarcity effects: 0=only warmth care, 5=trivial",
        "min": 0,
        "max": 10,
    },
    
    # Threat Pressure
    "n_threats": {
        "baseline_value": 0,
        "tuning_range": [0, 1, 2],
        "rationale": "Test protective motivation: how do threats override care/forage?",
        "min": 0,
        "max": 5,
    },
    
    # Grid Size (affects search difficulty relative to perception)
    "grid_size": {
        "baseline_value": "20x20",
        "tuning_range": ["15x15", "20x20", "30x30"],
        "rationale": "Test scale effects: larger grids = harder search, more navigation",
        "min": "10x10",
        "max": "50x50",
    },
    
    # Learning Rate (for plasticity experiments)
    "learning_rate": {
        "baseline_value": 0.02,
        "tuning_range": [0.001, 0.01, 0.02, 0.05, 0.1],
        "rationale": "Test learning speed: how quickly does mother adapt weights?",
        "min": 0.0001,
        "max": 0.5,
    },
    
    # Plasticity Rule
    "plasticity_rule": {
        "baseline_value": "fixed",
        "tuning_range": ["fixed", "outcome", "hebbian"],
        "rationale": "Test learning mechanisms: none vs outcome-gated vs associative",
    },
    
    # Child Base Decay Rates
    "child_hunger_increase_rate": {
        "baseline_value": 0.5,
        "tuning_range": [0.2, 0.5, 1.0],
        "rationale": "Test hunger pressure: slow vs moderate vs fast starvation",
        "min": 0.1,
        "max": 2.0,
    },
    
    # Time Scale
    "max_ticks": {
        "baseline_value": 3000,
        "tuning_range": [1000, 3000, 5000],
        "rationale": "Test emergent behavior time scale: do patterns change over longer periods?",
        "min": 500,
        "max": 10000,
    },
}

# ═══════════════════════════════════════════════════════════════════════
# SECTION 4: DEPENDENCIES & COUPLINGS
# ═══════════════════════════════════════════════════════════════════════

DEPENDENCIES = {
    
    "perception_range ↔ grid_size": {
        "coupling": "Non-linear. Perception determines search area relative to grid.",
        "rule": "Keep ratio ~0.4-0.6 (perception = 0.4-0.6 * grid_diagonal)",
        "example": "If grid=20x20 (diagonal~28), perception should be 8-15 cells",
    },
    
    "move_cost_energy ↔ idle_recovery_energy": {
        "coupling": "Critical for sustainability. Must be solvable: move_cost < total_recovery_per_tick",
        "rule": "move_cost_energy + (idle_recovery_energy * rest_fraction) ≈ 0 (equilibrium)",
        "guardians": ["Validate energy doesn't spiral to 0", "Check mother can survive 30 days"],
    },
    
    "n_food_initial ↔ perception_range": {
        "coupling": "Affects forage difficulty. More food but lower perception = easier.",
        "rule": "Compensate: reduce food if perception increases",
    },
    
    "n_threats ↔ threat_perception_radius": {
        "coupling": "Threat distance determines cortisol/fear trigger distance.",
        "rule": "For baseline (0 threats), this is inert. Becomes critical when n_threats > 0",
    },
    
    "learning_rate ↔ plasticity_rule": {
        "coupling": "Only active if plasticity_rule != 'fixed'.",
        "rule": "Baseline must use plasticity_rule='fixed' (learning_rate ignored)",
    },
}

# ═══════════════════════════════════════════════════════════════════════
# SECTION 5: VALIDATION CHECKLIST
# ═══════════════════════════════════════════════════════════════════════

VALIDATION_CHECKLIST = [
    "☑ Grid size is 20x20 (spatial constraint)",
    "☑ Perception range is 8 (bounded, non-trivial search)",
    "☑ Initial food = 1 (balanced resource)",
    "☑ No threats (neutral condition)",
    "☑ Energy recovery is positive (sustainability)",
    "☑ Movement cost is -0.3 (tuned for care viability)",
    "☑ Seed = 42 (reproducible randomness)",
    "☑ Plasticity rule = 'fixed' (no learning in baseline)",
    "☑ Max ticks = 3000 (sufficient for behavior emergence)",
    "☑ Initial hunger = 0, warmth = 50 (unbiased start)",
]

"""
END CONTROL VARIABLES SPECIFICATION
"""
