import numpy as np
import matplotlib.pyplot as plt

DT_paper = 0.5 # seconds; time step for update
HORMONE_MIN, HORMONE_MAX = 0.1, 1.0


class Hormone:
    def __init__(self, phase, amplitude=0.3, baseline=0.05):
        self.phase = float(phase)
        self.amplitude = float(amplitude)
        self.baseline = float(baseline)
        
    def circadian(self, t_hours):
        """Generate circadian hormonal signal at time t_hours."""
        signal = self.baseline + self.amplitude * np.cos(2 * np.pi * (t_hours - self.phase) / 24.0)
        signal = np.clip(signal, HORMONE_MIN, HORMONE_MAX)
        return signal
        
# Create multiple hormones
OT  = Hormone(phase=10, amplitude=0.3, baseline=0.05)
AVP = Hormone(phase=10, amplitude=0.3, baseline=0.05)
DA  = Hormone(phase=20, amplitude=0.3, baseline=0.05)

# plot_multiple_hormones_05s({"OT": OT, "AVP": AVP, "DA": DA}, duration_hours=24.0)

def second_to_circadian_hours(t_seconds):
    return (t_seconds / 3600.0 ) % 24.0

def hormone_levels_at_time(t_seconds):
    t_hours = second_to_circadian_hours(t_seconds)
    return {
        "OT": OT.circadian(t_hours),
        "AVP": AVP.circadian(t_hours),
        "DA": DA.circadian(t_hours)
    }
    
    
    
# plot hormone levels over 1 hour with 0.5s timestep
t_seconds = np.arange(0.0, 3600.0 * 24.0 - 1e-9, DT_paper)
t_hours = second_to_circadian_hours(t_seconds)
OT_levels = hormone_levels_at_time(t_seconds)['OT']
DA_levels = hormone_levels_at_time(t_seconds)['DA']
AVP_levels = hormone_levels_at_time(t_seconds)['AVP']
# OT_levels = OT.circadian(t_hours)

print(f"OT levels (first 10 values): {OT_levels[:10]}")
print(f"DA levels (first 10 values): {DA_levels[:10]}")
print(f"AVP levels (first 10 values): {AVP_levels[:10]}")

# window = int(3600*24 // DT_paper)  # 1 hour window = 3600 
# print(window)
# plt.figure(figsize=(12, 4))
# plt.plot(t_hours[:window], DA_levels[:window], label='Dopamine (DA)', color='orange')
# plt.plot(t_hours[:window], AVP_levels[:window], label='Vasopressin (AVP)', color='green')
# plt.plot(t_hours[:window], OT_levels[:window], label='Oxytocin (OT)', color='blue')
# plt.xlabel("Time (hours)")  
# plt.ylabel("Hormone level")
# plt.title(f"Oxytocin (OT) levels over 1 hour (dt={DT_paper}s)")
# plt.grid(True)
# plt.legend()
# plt.tight_layout()
# plt.show()

    