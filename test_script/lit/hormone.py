import math
from collections import deque
import numpy as np
from stim import StimulusTable

# HORMONE SYSTEM
class HormoneSystem:
    def __init__(self, PB=50, history_size=1200):
        self.PB = float(PB)

        # time
        self.time_h = 0.0
        self.elapsed_s = 0.0

        # hormones (secretion rates) in [0.01, 1.0]
        cr_DA, cr_OT, cr_AVP = self.circadian_rhythm(self.time_h)
        self.DA = cr_DA
        self.OT = cr_OT
        self.AVP = cr_AVP

        self.OT_auto = 0.001
        self.table = StimulusTable()

        # stimuli intensities si_k(t) in [0,100]
        self.si = {k: 0.0 for k in ["USER_PRESENCE", "HIT", "CARESS", "CORRECT", "WRONG"]}

        # history for plotting
        self.time_history = deque(maxlen=history_size)

        self.DA_history = deque(maxlen=history_size)
        self.OT_history = deque(maxlen=history_size)
        self.AVP_history = deque(maxlen=history_size)
        self.PB_history = deque(maxlen=history_size)

        self.si_user_history = deque(maxlen=history_size)
        self.si_hit_history = deque(maxlen=history_size)
        self.si_care_history = deque(maxlen=history_size)
        self.si_correct_history = deque(maxlen=history_size)
        self.si_wrong_history = deque(maxlen=history_size)

    def circadian_rhythm(self, t_h):
        cr_DA  = 0.05 + 0.3 * math.cos(2 * math.pi * ((t_h - 20) / 24.0))
        cr_OT  = 0.05 + 0.3 * math.cos(2 * math.pi * ((t_h - 10) / 24.0))
        cr_AVP = 0.05 + 0.3 * math.cos(2 * math.pi * ((t_h - 10) / 24.0))
        return (
            float(np.clip(cr_DA,  0.01, 1.0)),
            float(np.clip(cr_OT,  0.01, 1.0)),
            float(np.clip(cr_AVP, 0.01, 1.0)),
        )

    def eval_beta(self, beta_value):
        if isinstance(beta_value, (int, float)):
            return float(beta_value)
        if isinstance(beta_value, str):
            allowed = {"PB": self.PB}
            try:
                return float(eval(beta_value, {"__builtins__": {}}, allowed))
            except Exception:
                return 1.0
        return 1.0

    # duration stimuli
    def set_duration(self, stim_name, intensity):
        if stim_name in self.si:
            self.si[stim_name] = float(np.clip(intensity, 0, 100))

    # impulse stimuli
    def trigger_impulse(self, stim_name, intensity=100.0):
        if stim_name in self.si:
            self.si[stim_name] = float(np.clip(intensity, 0, 100))

    def update(self, dt_s=0.5):
        # baselines
        cr_DA, cr_OT, cr_AVP = self.circadian_rhythm(self.time_h)

        # stimulus contributions (Eq. 3)
        dDA = dOT = dAVP = 0.0
        for stim_name, si_val in self.si.items():
            if si_val <= 0:
                continue
            for eff in self.table.get_effects(stim_name):
                alpha = eff["alpha"]
                beta = self.eval_beta(eff["beta"])
                se = alpha * beta * si_val  # Eq. (3)
                if eff["target"] == "DA":
                    dDA += se
                elif eff["target"] == "OT":
                    dOT += se
                elif eff["target"] == "AVP":
                    dAVP += se

        # update hormones
        self.DA  = float(np.clip(cr_DA  + dDA,                 0.01, 1.0))
        self.AVP = float(np.clip(cr_AVP + dAVP,                0.01, 1.0))
        self.OT  = float(np.clip(cr_OT  + dOT + self.OT_auto,  0.01, 1.0))

        # advance time
        self.time_h = (self.time_h + dt_s / 3600.0) % 24.0
        self.elapsed_s += dt_s

    def update_PB(self, dt_s=0.5):
        # scale PB dynamics to paper’s typical dt=0.5s
        step_scale = dt_s / 0.5
        dr = 0.01
        delta = (0.2 * self.OT - 0.2 * self.AVP - dr) * step_scale
        self.PB = float(np.clip(self.PB + delta, 0, 100))

    def record(self):
        self.time_history.append(self.elapsed_s)

        self.DA_history.append(self.DA)
        self.OT_history.append(self.OT)
        self.AVP_history.append(self.AVP)
        self.PB_history.append(self.PB)

        self.si_user_history.append(self.si["USER_PRESENCE"])
        self.si_hit_history.append(self.si["HIT"])
        self.si_care_history.append(self.si["CARESS"])
        self.si_correct_history.append(self.si["CORRECT"])
        self.si_wrong_history.append(self.si["WRONG"])
