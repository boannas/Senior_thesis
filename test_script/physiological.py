import test_script.hormone as hormone
import numpy as np
import matplotlib.pyplot as plt

# Generate hormonal signals
n_days = 10
t, DA  = hormone.hormonal_generator_signal(24 * n_days, dt=1.0 / 60, amplitude=0.3, phase_shift=20)
_, OT  = hormone.hormonal_generator_signal(24 * n_days, dt=1.0 / 60, amplitude=0.3, phase_shift=10)
_, AVP = hormone.hormonal_generator_signal(24 * n_days, dt=1.0 / 60, amplitude=0.3, phase_shift=10)

# Compute pair bonding hormone dynamics
PB = hormone.pair_bonding_hormones(t, OT, AVP)

# Initialize physiological states
wakefullness = 5.0
social_need = 0.0
entertainment = 0.0
pair_bonding = 50.0
User_rejection = 50.0

def wakefullness_dynamics(DA, wakefullness):
    return hormone.wakefullness_dynamics(DA, wakefullness)

def compute_physiological(DA, PB, wakefullness, social_need, entertainment, pair_bonding, User_rejection):
    wakefullness = 5
    social_need = 

