import numpy as np
import matplotlib.pyplot as plt
"""
===========================================================
Hormonal Signal Generators and Pair-Bonding Dynamics   
===========================================================
    Reference source: DOI: 10.1016/j.robot.2024.104687
Human–robot pair-bonding from a neuroendocrine perspective: 
Modeling the effect of oxytocin, arginine vasopressin, 
and dopamine on the social behavior of an autonomous robot
"""

def hormonal_generator_signal(total_hours=24, dt=1.0, amplitude=0.3, phase_shift=0):
    """
    Circadian hormonal signal generator.
    Parameters:
    - total_hours: Total duration for the signal in hours.
    - dt: Time step in hours.
    - amplitude: Amplitude of the hormonal signal.
    - phase_shift: Phase shift of the hormonal signal.
    Returns:
    - t: Time array.    
    - signal: Hormonal signal array. (Clipped between 0.1 and 1.0)
    """
    t = np.arange(0, total_hours, dt)
    signal = 0.05 + amplitude * np.cos(2 * np.pi * (t - phase_shift) / 24)
    signal = np.clip(signal, 0.1, 1.0, out=signal)
    return t, signal

def pair_bonding_hormones(t, OT, AVP, dr = 0.01):
    """
    Pair bonding hormone dynamics influenced by Oxytocin (OT) and Vasopressin (AVP).
    Parameters:
    - t: Time array.    
    - OT: Oxytocin signal array.
    - AVP: Vasopressin signal array.
    - dr: Decay rate of the pair bonding hormone.
    Returns:
    - PB: Pair bonding hormone array. (Clipped between 0 and 100)
    """
    PB = np.zeros_like(t)
    PB[0] = 50
    for i in range(1, len(t)):
        PB[i] = PB[i-1] + 0.2 * OT[i] - 0.2 * AVP[i] - dr
    PB = np.clip(PB, 0, 100, out=PB)
    return PB


# Generate hormonal signals
n_days = 10
t, DA  = hormonal_generator_signal(24 * n_days, dt=1.0 / 60, amplitude=0.3, phase_shift=20)
_, OT  = hormonal_generator_signal(24 * n_days, dt=1.0 / 60, amplitude=0.3, phase_shift=10)
_, AVP = hormonal_generator_signal(24 * n_days, dt=1.0 / 60, amplitude=0.3, phase_shift=10)

# Compute pair bonding hormone dynamics
PB = pair_bonding_hormones(t, OT, AVP)

# Plotting the results
plt.figure(figsize=(14, 4))
plt.plot(t, PB, label='Pair Bonding Hormone', color='purple')
plt.xlabel("Time (hours)")
plt.ylabel("Amplitude")
plt.title("Pair Bonding Hormone")
plt.legend()
plt.grid()

plt.figure(figsize=(14, 4))
plt.plot(t, DA, label='Dopamine')
plt.plot(t, OT, label='Oxytocin')
plt.plot(t, AVP, label='Vasopressin')
plt.xlabel("Time (hours)")
plt.ylabel("Amplitude")
plt.title("Hormonal Signals")
plt.legend()
plt.grid()
plt.show()

