IDEAL_VALUE = {
    "M_energy": 80.0,
    "M_injury": 0.0,
    "M_closeness": 50.0,
    "M_bonding": 100.0,
    "M_fear": 0.0,
    "M_stress": 0.0,
    "M_fatigue": 0.0,
    "C_hunger": 0.0,
    "C_warmth": 50.0,
    "C_injury": 0.0
}


def deficit_low(current, ideal):
    return max(0, ideal - current)

def deficit_high(current, ideal):
    return max(0, current - ideal)

def deficit_abs(current, ideal):
    return abs(current - ideal)