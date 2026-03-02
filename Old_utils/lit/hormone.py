import math
from collections import deque
import numpy as np
from stim import StimulusTable

# Motivation States
class MotivationState:
    def __init__(self, physio=None, deficit=None):
        self.awake = 0
        self.play = 0
        self.social = 0
        self.relax = 0
        
        self.user_presence = 1
    
    def update(self, physio, deficit):
        # print('physio', physio)
        # print('deficit', deficit)
        # self.awake = float((physio["WK"]))
        # self.play = float((deficit["ENT"] * physio["WK"] * (100 - self.user_presence)))
        # self.social = float((deficit["SN"] * physio["WK"] * self.user_presence))
        # self.relax = float((physio["UR"] * self.user_presence))

        self.awake = float((physio["WK"]))
        self.play = float((deficit["ENT"] * physio["WK"] ))
        self.social = float((deficit["SN"] * physio["WK"]))
        self.relax = float((physio["UR"]))

        print(f"Motivation States - Awake: {self.awake:.2f}, Play: {self.play:.2f}, Social: {self.social:.2f}, Relax: {self.relax:.2f}")

# Biological Process System
class BiologicalProcessSystem:
    def __init__(self, WK=5.0, SN=0.0, ENT=0.0, PB=100.0, UR=50.0):
        # Initial values
        self.state = {
            "WK": float(WK),     # Wakefulness  
            "SN": float(SN),     # Social Need
            "ENT": float(ENT),   # Entertainment
            "PB": float(PB),     # Pair Bonding
            "UR": float(UR),     # User Rejection
        }

        # Ideal values
        self.ideal = {
            "WK": 5.0,
            "SN": 0.0,
            "ENT": 0.0,
            "PB": 100.0,
            "UR": 0.0,
        }
        
        self.deficit = self.deficits()
        # self.deficits = self.deficits()

    def deficits(self):
        return {
            k: float(self.state[k] - self.ideal[k]) 
            for k in self.state
        }
    
    def update(self, DA, OT, AVP, dt_s=0.5):
        # Wakefulness (WK) 
        self.WK = 5.0

        # Social Need (SN)
        self.state["SN"] = self.state["SN"] + 0.02 * OT
        self.state["SN"] = float(np.clip(self.state["SN"], 0, 100))
        
        # Entertainment (ENT)
        self.state["ENT"] = self.state["ENT"] + 0.02 * DA
        self.state["ENT"] = float(np.clip(self.state["ENT"], 0, 100))

        # Pair Bonding (PB) 
        dr = 0.01
        self.state["PB"] = self.state["PB"] + (0.2 * OT - 0.2 * AVP - dr) 
        self.state["PB"] = float(np.clip(self.state["PB"], 0, 100))

        # User Rejection (UR)
        self.state["UR"] = 100 - self.state["PB"]
        self.state["UR"] = float(np.clip(self.state["UR"], 0, 100))

        # update deficits
        self.deficit = self.deficits()

        print(f"Biological States - WK: {self.state['WK']:.2f}, SN: {self.state['SN']:.2f}, ENT: {self.state['ENT']:.2f}, PB: {self.state['PB']:.2f}, UR: {self.state['UR']:.2f}")
        print(f"Deficits - SN: {self.deficit['SN']:.2f}, ENT: {self.deficit['ENT']:.2f}, PB: {self.deficit['PB']:.2f}, UR: {self.deficit['UR']:.2f}")

# HORMONE SYSTEM
class HormoneSystem:
    def __init__(self, PB=50, history_size=1200, bp: BiologicalProcessSystem = None, MS: MotivationState = None):
        self.bp = bp
        self.motivation = MS
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
        # dSN = 0.0  # Social Need
        for stim_name, si_val in self.si.items():
            if si_val <= 0:
                continue
            for eff in self.table.get_effects(stim_name):
                # print(f"Stim: {stim_name}, Effect: {eff}")

                alpha = eff["alpha"]
                beta = self.eval_beta(eff["beta"])
                se = alpha * beta * si_val  # Eq. (3)
                if eff["target"] == "DA":
                    dDA += se
                elif eff["target"] == "OT":
                    dOT += se
                elif eff["target"] == "AVP":
                    dAVP += se
                # print(f"  {stim_name} -> {eff['target']}: {se:.4f}")

        # update hormones
        self.DA  = float(np.clip(cr_DA  + dDA,                 0.01, 1.0))
        self.AVP = float(np.clip(cr_AVP + dAVP,                0.01, 1.0))
        self.OT  = float(np.clip(cr_OT  + dOT + self.OT_auto,  0.01, 1.0))
        
        # update biological processes
        self.bp.update(self.DA, self.OT, self.AVP, dt_s=dt_s) 
        self.motivation.update(self.bp.state, self.bp.deficit)
        # print(physio)
        # print(deficit)
        # print(self.bp.state['PB'])

        # advance time
        self.time_h = (self.time_h + dt_s / 3600.0) % 24.0
        self.elapsed_s += dt_s


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
