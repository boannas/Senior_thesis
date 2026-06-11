"""
Deficit functions for comparing current internal states against ideal
homeostatic setpoints. These drive the motivation system.
"""

# Ideal homeostatic setpoints for internal states
IDEAL_VALUE = {
    # Mother
    "M_energy": 80.0,       # Mother preferred energy level
    "M_closeness": 50.0,    # Mother preferred closeness to child
    "M_bonding": 100.0,     # Maximum bonding
    "M_fear": 0.0,          # Ideally no fear
    "M_stress": 0.0,        # Ideally no stress
    "M_fatigue": 0.0,       # Ideally no fatigue

    # Child
    "C_hunger": 0.0,        # Ideally not hungry
    "C_warmth": 50.0,       # Ideal warmth (mid-range)
    "C_injury": 0.0,        # Ideally not injured
}


def deficit_low(current, ideal):
    """
    Deficit when current value is below ideal.
    Returns how far below the ideal the current value is.
    Example: energy deficit (want energy HIGH, deficit when LOW).
    """
    return max(0.0, ideal - current)


def deficit_high(current, ideal):
    """
    Deficit when current value exceeds ideal.
    Returns how far above the ideal the current value is.
    Example: fear deficit (want fear LOW, deficit when HIGH).
    """
    return max(0.0, current - ideal)


def deficit_abs(current, ideal):
    """
    Absolute distance from ideal value.
    Used when both high and low deviations are undesirable.
    Example: warmth (want MID, deficit when too hot or too cold).
    """
    return abs(current - ideal)