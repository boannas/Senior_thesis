
# Ideal homeostatic setpoints for internal states
IDEAL_VALUE = {
    # Mother 
    "M_energy": 80.0,       # Mother preferred energy level
    # "M_injury": 0.0,        
    "M_closeness": 50.0,    # Mother preferred distance to child
    "M_bonding": 100.0,     # Maximum bonding
    "M_fear": 0.0,          # Ideally no fear
    "M_stress": 0.0,        # ideally no stress
    "M_fatigue": 0.0,       # Ideally not fatigue

    # Child
    "C_hunger": 0.0,        # Ideally not hungry
    "C_warmth": 50.0,       # Ideal wamth 
    "C_injury": 0.0         # Ideal not injured
}


def deficit_low(current, ideal):
    """
    Deficit when current value is below ideal.
    Example: energy deficit.
    """
    return max(0, ideal - current)

def deficit_high(current, ideal):
    """
    Deficit when current value exceeds ideal.
    Example: fear or stress.
    """
    return max(0, current - ideal)

def deficit_abs(current, ideal):
    """
    Absolute distance from ideal.
    Used when both high and low values are undesirable.
    Example: temperature or closeness.
    """
    return abs(current - ideal)