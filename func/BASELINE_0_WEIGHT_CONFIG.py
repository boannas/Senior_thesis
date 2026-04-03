"""
BASELINE-0 Weight Configuration: Single Source of Truth
========================================================

Defines all weight configurations for Baseline-0 neutral weight search.
Used by:
- baseline_0_runner.py (Baseline-0 fixed weights)
- baseline_0_neutral_search.py (Phase 1/2 parametric search)
- core/agents.py (MotherAgent initialization)

Key Design Principles:
1. All settings logged and traceable
2. Single source of truth for weight values
3. Supports parametric search via scale factors
4. Validation functions included

Weight System:
- Forage: child_hunger, energy_deficit, low_fear
- Care: child_warmth, closeness_deficit, bonding
- Self: fatigue, fear, stress
- Protect: child_injury, fear, closeness_deficit, bonding
"""

import numpy as np


# === BASELINE-0 FIXED WEIGHTS (Used when no search parameter applied) ===

BASELINE_FORAGE_WEIGHTS = {
    "child_hunger": 0.5,
    "energy_deficit": 0.5,
    "low_fear": 0.5,
}

BASELINE_CARE_WEIGHTS_BASE = {
    "child_warmth": 1.0,      # Base value; will be scaled by α in search
    "closeness_deficit": 1.0,  # Base value; will be scaled by α in search
    "bonding": 1.0,            # Base value; will be scaled by α in search
}

BASELINE_SELF_WEIGHTS = {
    "fatigue": 0.5,
    "fear": 0.5,
    "stress": 0.5,
}

BASELINE_PROTECT_WEIGHTS = {
    "child_injury": 0.5,
    "fear": 0.5,
    "closeness_deficit": 0.5,
    "bonding": 0.5,
}


# === PHASE 1 PARAMETER GRID (Coarse Search) ===
def get_phase_1_parameter_grid():
    """
    Phase 1: Coarse grid search over Care weight scale factor alpha.
    
    Range: 0.1 to 1.9 in steps of 0.2 (10 points total)
    Rationale: Explores full spectrum from minimal care to 1.9x care relative to forage
    
    Returns: List of alpha values to sweep
    """
    return [0.1, 0.3, 0.5, 0.7, 0.9, 1.1, 1.3, 1.5, 1.7, 1.9]


# === WEIGHT GENERATION FUNCTIONS ===

def get_neutral_motivation_weights(care_scale_factor=1.0):
    """
    Generate motivation weight dict for given care scale factor.
    
    Used by: MotherAgent when baseline_weights parameter is provided
    
    Args:
        care_scale_factor (float): Multiplier for care weights
                                   - α=0.5: Care weights 50% of forage
                                   - α=1.0: Fair competition (care=forage)
                                   - α=1.5: Care 150% of forage
    
    Returns:
        dict: Motivation weights with structure:
        {
            "forage": {"child_hunger": 0.5, "energy_deficit": 0.5, "low_fear": 0.5},
            "care": {"child_warmth": α, "closeness_deficit": α, "bonding": α},
            "self": {"fatigue": 0.5, "fear": 0.5, "stress": 0.5},
            "protect": {"child_injury": 0.5, "fear": 0.5, "closeness_deficit": 0.5, "bonding": 0.5},
        }
    """
    return {
        "forage": BASELINE_FORAGE_WEIGHTS.copy(),
        "care": {
            k: v * care_scale_factor
            for k, v in BASELINE_CARE_WEIGHTS_BASE.items()
        },
        "self": BASELINE_SELF_WEIGHTS.copy(),
        "protect": BASELINE_PROTECT_WEIGHTS.copy(),
    }


def validate_weight_dict(weights_dict):
    """
    Validate that a weight dict has correct structure and values.
    
    Returns: (is_valid, error_message)
    """
    required_motivations = {"forage", "care", "self", "protect"}
    if not isinstance(weights_dict, dict):
        return False, "weights_dict must be a dict"
    
    if set(weights_dict.keys()) != required_motivations:
        return False, f"Missing motivations. Expected {required_motivations}, got {set(weights_dict.keys())}"
    
    required_keys = {
        "forage": {"child_hunger", "energy_deficit", "low_fear"},
        "care": {"child_warmth", "closeness_deficit", "bonding"},
        "self": {"fatigue", "fear", "stress"},
        "protect": {"child_injury", "fear", "closeness_deficit", "bonding"},
    }
    
    for motivation, required_subkeys in required_keys.items():
        if not isinstance(weights_dict[motivation], dict):
            return False, f"{motivation} must be a dict"
        if set(weights_dict[motivation].keys()) != required_subkeys:
            return False, f"{motivation}: Expected keys {required_subkeys}, got {set(weights_dict[motivation].keys())}"
        
        for key, value in weights_dict[motivation].items():
            if not isinstance(value, (int, float)):
                return False, f"{motivation}.{key} must be numeric, got {type(value)}"
            if value < 0:
                return False, f"{motivation}.{key} = {value} is negative"
    
    return True, "OK"


def log_weight_configuration(care_scale_factor=1.0):
    """
    Log weight configuration for reproducibility/auditability.
    
    Args:
        care_scale_factor: Alpha value being used
    
    Returns: Formatted string describing configuration
    """
    weights = get_neutral_motivation_weights(care_scale_factor)
    
    log_str = f"""
WEIGHT CONFIGURATION LOG
{'='*60}
Care Scale Factor (α): {care_scale_factor:.2f}

Forage Weights:
  - child_hunger: {weights['forage']['child_hunger']:.2f}
  - energy_deficit: {weights['forage']['energy_deficit']:.2f}
  - low_fear: {weights['forage']['low_fear']:.2f}

Care Weights (scaled by α={care_scale_factor:.2f}):
  - child_warmth: {weights['care']['child_warmth']:.2f}
  - closeness_deficit: {weights['care']['closeness_deficit']:.2f}
  - bonding: {weights['care']['bonding']:.2f}

Self Weights:
  - fatigue: {weights['self']['fatigue']:.2f}
  - fear: {weights['self']['fear']:.2f}
  - stress: {weights['self']['stress']:.2f}

Protect Weights:
  - child_injury: {weights['protect']['child_injury']:.2f}
  - fear: {weights['protect']['fear']:.2f}
  - closeness_deficit: {weights['protect']['closeness_deficit']:.2f}
  - bonding: {weights['protect']['bonding']:.2f}
{'='*60}
"""
    return log_str


# === SELF-TEST ===
if __name__ == "__main__":
    print("\n=== BASELINE-0 WEIGHT CONFIG SELF-TEST ===\n")
    
    # Test phase 1 grid
    print("Phase 1 Parameter Grid:")
    grid = get_phase_1_parameter_grid()
    print(f"  Alpha values: {grid}")
    print(f"  Count: {len(grid)}\n")
    
    # Test weight generation
    print("Testing weight generation at multiple alpha values:")
    for alpha in [0.5, 1.0, 1.5]:
        weights = get_neutral_motivation_weights(alpha)
        is_valid, msg = validate_weight_dict(weights)
        print(f"  α={alpha}: valid={is_valid} ({msg})")
        print(f"    Forage sum: {sum(weights['forage'].values()):.2f}")
        print(f"    Care sum: {sum(weights['care'].values()):.2f}")
    
    # Test validation
    print("\nValidation tests:")
    valid_weights = get_neutral_motivation_weights(1.0)
    is_valid, msg = validate_weight_dict(valid_weights)
    print(f"  Valid dict: {is_valid} - {msg}")
    
    invalid_weights = {"forage": {"child_hunger": -0.5}}  # Missing keys
    is_valid, msg = validate_weight_dict(invalid_weights)
    print(f"  Invalid dict: {is_valid} - {msg}")
    
    print("\nLogging test (α=1.5):")
    print(log_weight_configuration(1.5))
    
    print("✓ Self-test completed successfully\n")
