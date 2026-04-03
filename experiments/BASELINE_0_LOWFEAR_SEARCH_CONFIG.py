"""
BASELINE-0 Phase 3: Low_Fear Weight Search Configuration

This module extends BASELINE_0_WEIGHT_CONFIG.py for Phase 3 experiments.

Phase 3 Strategy:
  - Fixed: Care weights at α=0.70 (from Phase 2 bifurcation point)
  - Fixed: Child_hunger and energy_deficit weights at 0.5
  - Varied: Low_fear weight γ ∈ {0.1, 0.2, ..., 1.0} (10 values)
  - Hypothesis: Suppress low_fear (which drives forage asymmetry) to escape bifurcation

Justification:
  The motivation formula is normalized: score = (weighted_sum) / (weight_sum)
  Uniform scaling of all forage weights cancels out (numerator and denominator both scale by β)
  But varying ONLY low_fear weight does NOT cancel because the denominator changes unevenly

Design principle: One source of truth; import fixed weights from BASELINE_0_WEIGHT_CONFIG
Auditability: Every experiment logs exact γ value and all weight settings
"""

import json
from datetime import datetime
from BASELINE_0_WEIGHT_CONFIG import (
    BASELINE_SELF_WEIGHTS,
    BASELINE_PROTECT_WEIGHTS,
    validate_weight_dict,
)


# ════════════════════════════════════════════════════════════════════════
# PHASE 3 SPECIFIC CONFIGURATION
# ════════════════════════════════════════════════════════════════════════

# Fixed at Phase 2 bifurcation point
PHASE_3_CARE_SCALE_ALPHA = 0.70

# Fixed forage sub-weights (except low_fear which will vary)
PHASE_3_FORAGE_CHILD_HUNGER = 0.5
PHASE_3_FORAGE_ENERGY_DEFICIT = 0.5
# PHASE_3_FORAGE_LOW_FEAR will vary (γ parameter)

# All other weights from baseline (reuse to maintain consistency)
PHASE_3_SELF_WEIGHTS = BASELINE_SELF_WEIGHTS.copy()
PHASE_3_PROTECT_WEIGHTS = BASELINE_PROTECT_WEIGHTS.copy()


# ════════════════════════════════════════════════════════════════════════
# API Functions
# ════════════════════════════════════════════════════════════════════════

def get_phase_3_weights(low_fear_scale_factor=0.5):
    """
    Generate motivation weights for Phase 3 low_fear sweep.
    
    Parameters
    ----------
    low_fear_scale_factor : float
        Multiplier for low_fear weight (γ parameter).
        As γ decreases, low_fear component is suppressed.
        γ=1.0: original baseline (low_fear at full strength)
        γ=0.5: moderate suppression
        γ=0.1: aggressive suppression
        
    Returns
    -------
    dict
        Motivation weights dictionary ready for MotherAgent initialization.
        Structure: {"forage": {...}, "care": {...}, "self": {...}, "protect": {...}}
        
    Notes
    -----
    - Only low_fear weight varies; all other forage sub-weights fixed at 0.5
    - Care weights: all fixed at 0.70 (α from Phase 2)
    - Self, Protect weights: fixed at baseline 0.5 values
    - This setup avoids normalization cancellation because low_fear is the only variable
    """
    care_weights_base = {
        "child_warmth": 1.0,
        "closeness_deficit": 1.0,
        "bonding": 1.0,
    }
    
    return {
        "forage": {
            "child_hunger": PHASE_3_FORAGE_CHILD_HUNGER,
            "energy_deficit": PHASE_3_FORAGE_ENERGY_DEFICIT,
            "low_fear": low_fear_scale_factor,  # ← ONLY THIS VARIES
        },
        "care": {
            k: v * PHASE_3_CARE_SCALE_ALPHA
            for k, v in care_weights_base.items()
        },
        "self": PHASE_3_SELF_WEIGHTS.copy(),
        "protect": PHASE_3_PROTECT_WEIGHTS.copy(),
    }


def get_phase_3_parameter_grid():
    """
    Return the Phase 3 low_fear weight sweep parameters.
    
    Returns
    -------
    list of float
        Low_fear scale factors (γ) to test, in ascending order
        
    Notes
    -----
    10 values spanning full range:
      γ=0.1: Aggressive suppression (forage_sum=1.1)
      γ=0.5: Moderate suppression (forage_sum=1.5, reference point)
      γ=1.0: No suppression (forage_sum=2.0, original Phase 2 bifurcation)
      
    Hypothesis prediction:
      As γ increases toward 1.0, bifurcation re-emerges
      Stable neutral region expected at γ ∈ [0.3-0.5]
    """
    return [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0]


def get_phase_3_fixed_settings():
    """
    Return all FIXED settings for Phase 3 (for logging/documentation).
    
    Returns
    -------
    dict
        Dictionary of all fixed parameters that do NOT vary
        
    Notes
    -----
    Use this for audit trails and experimental documentation.
    All values here should match what's hardcoded in this module and Phase 3 runners.
    """
    return {
        "phase": 3,
        "search_axis": "low_fear_weight (gamma)",
        "care_scale_alpha": PHASE_3_CARE_SCALE_ALPHA,
        "forage_child_hunger": PHASE_3_FORAGE_CHILD_HUNGER,
        "forage_energy_deficit": PHASE_3_FORAGE_ENERGY_DEFICIT,
        "forage_low_fear": "VARIES (gamma)",
        "self_weights": PHASE_3_SELF_WEIGHTS,
        "protect_weights": PHASE_3_PROTECT_WEIGHTS,
        "rationale": "Suppress low_fear weight to escape bifurcation caused by forage asymmetry",
        "expected_outcome": "Identify gamma where child_survival in [40-60%] with variance < 30%",
    }


def log_phase_3_configuration(low_fear_scale_factor, output_file):
    """
    Write the exact weight config used in a Phase 3 experiment to a file.
    
    Parameters
    ----------
    low_fear_scale_factor : float
        The γ value used for this experiment run
    output_file : str
        Path to write JSON config file
        
    Notes
    -----
    Call this once per gamma value (or once per batch) to create auditable record.
    Include the output_file name in experiment results for traceability.
    """
    weights = get_phase_3_weights(low_fear_scale_factor)
    validate_weight_dict(weights)
    
    config = {
        "timestamp": datetime.now().isoformat(),
        "phase": 3,
        "low_fear_scale_factor": low_fear_scale_factor,
        "care_scale_alpha": PHASE_3_CARE_SCALE_ALPHA,
        "forage_sum": sum(weights["forage"].values()),
        "care_sum": sum(weights["care"].values()),
        "self_sum": sum(weights["self"].values()),
        "protect_sum": sum(weights["protect"].values()),
        "weights": weights,
        "fixed_settings": get_phase_3_fixed_settings(),
        "notes": f"Baseline-0 Phase 3 - Low_fear sweep, gamma={low_fear_scale_factor}",
    }
    with open(output_file, "w", encoding="utf-8") as f:
        json.dump(config, f, indent=2)


# ════════════════════════════════════════════════════════════════════════
# Validation & Utilities
# ════════════════════════════════════════════════════════════════════════

def print_phase_3_config_summary(low_fear_scale_factor):
    """
    Print a human-readable summary of the Phase 3 configuration.
    
    Parameters
    ----------
    low_fear_scale_factor : float
        The γ value
    """
    weights = get_phase_3_weights(low_fear_scale_factor)
    print(f"\n{'='*80}")
    print(f"Baseline-0 Phase 3 Configuration (γ = {low_fear_scale_factor})")
    print(f"{'='*80}")
    
    print(f"\nFixed Reference (from Phase 2):")
    print(f"  Care scale alpha (α):    {PHASE_3_CARE_SCALE_ALPHA}")
    
    print(f"\nWeight Groups:")
    for category in ["forage", "care", "self", "protect"]:
        category_weights = weights[category]
        weight_sum = sum(category_weights.values())
        weight_str = ", ".join(f"{k}={v:.1f}" for k, v in category_weights.items())
        print(f"  {category:10s}: {weight_str:50s} (sum={weight_sum:.1f})")
    
    print(f"\n{'='*80}\n")


def print_phase_3_parameter_grid():
    """Print the Phase 3 parameter grid in table format."""
    grid = get_phase_3_parameter_grid()
    print(f"\nPhase 3 Parameter Grid ({len(grid)} gamma values):")
    print(f"{'γ Value':<12} {'Forage Sum':<15} {'Interpretation':<50}")
    print("─" * 77)
    for gamma in grid:
        forage_sum = 0.5 + 0.5 + gamma
        if gamma == 0.5:
            interpretation = "Reference point (original Phase 2 baseline)"
        elif gamma == 1.0:
            interpretation = "Maximum low_fear (Phase 2 bifurcation re-emerges)"
        elif gamma < 0.5:
            interpretation = "Aggressive low_fear suppression"
        else:
            interpretation = "Moderate low_fear suppression"
        print(f"{gamma:<12.1f} {forage_sum:<15.1f} {interpretation:<50}")
    print()


if __name__ == "__main__":
    # Self-test
    print("Testing BASELINE_0_LOWFEAR_SEARCH_CONFIG.py...")
    
    # Test γ = 0.5 (reference point)
    w05 = get_phase_3_weights(0.5)
    validate_weight_dict(w05)
    print("✓ γ=0.5 config valid")
    
    # Test γ = 0.1 (aggressive suppression)
    w01 = get_phase_3_weights(0.1)
    validate_weight_dict(w01)
    print("✓ γ=0.1 config valid")
    
    # Test γ = 1.0 (full strength)
    w10 = get_phase_3_weights(1.0)
    validate_weight_dict(w10)
    print("✓ γ=1.0 config valid")
    
    # Test grid
    grid = get_phase_3_parameter_grid()
    print(f"✓ Phase 3 grid: {len(grid)} points")
    
    # Test fixed settings
    fixed = get_phase_3_fixed_settings()
    print(f"✓ Fixed settings documented: {len(fixed)} items")
    
    # Print examples
    print_phase_3_config_summary(0.5)
    print_phase_3_parameter_grid()
    
    print("All tests passed!")
