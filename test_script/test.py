import numpy as np
import matplotlib.pyplot as plt
import time

# Models
class CircadianHormone:
    def __init__(self, amplitude=0.3, phase_shift=0, baseline=0.05):
        self.amplitude = amplitude
        self.phase_shift = phase_shift
        self.baseline = baseline
        self.t = 0.0

    def step(self, dt):
        self.t += dt
        signal = self.baseline + self.amplitude * np.cos(
            2 * np.pi * (self.t - self.phase_shift) / 24
        )
        return np.clip(signal, 0.1, 1.0)

class PairBonding:
    def __init__(self, initial=50.0, decay_rate=0.01):
        self.PB = initial
        self.decay_rate = decay_rate

    def step(self, OT, AVP, dt):
        self.PB += (0.2 * OT - 0.2 * AVP - self.decay_rate) * dt
        self.PB = np.clip(self.PB, 0, 100)
        return self.PB

class PhysiologicalState:
    def __init__(self, wakefullness=5.0, social_need=0.0, entertainment=0.0, user_rejection=50.0):
        self.wakefullness = wakefullness
        self.social_need = social_need
        self.entertainment = entertainment
        self.user_rejection = user_rejection

    def update(self, DA, PB, OT):
        self.wakefullness = 5
        self.social_need = self.social_need + 0.02 * OT
        self.entertainment = self.entertainment + 0.02 * DA
        self.user_rejection = 100 - PB
        return (self.wakefullness, self.social_need, self.entertainment, self.user_rejection)
    
class stimuli:
    def handle_key(self, event):
        keymap = {
            'up': "Hit",
            'down': "Caress",
            'left': "CorrectAnswer",
            'right': "WrongAnswer",
            'enter': "UserPresence"
        }

        if event.key in keymap:
            stim_name = keymap[event.key]
            # print(f"Stimulus perceived: {stim_name}")
            return stim_name
        return None
    ## Try to create stimulus based on key press and effect on physiological state and hormones
    
# Initialization
dt = 1.0 

OT_gen  = CircadianHormone(phase_shift=10)
AVP_gen = CircadianHormone(phase_shift=10)
DA_gen  = CircadianHormone(phase_shift=20)

pair_bond = PairBonding()

state = PhysiologicalState()

stimulus = stimuli()

def on_key(event):
    stimulus.handle_key(event) 


# REAL-TIME PLOT SETUP
plt.ion()
fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(12, 6))
fig.canvas.mpl_connect('key_press_event', a= on_key)
WINDOW = 24

t_data  = []
OT_data = []
AVP_data = []
DA_data = []
PB_data = []

state_data = []

line_OT,  = ax1.plot([], [], label="OT")
line_AVP, = ax1.plot([], [], label="AVP")
line_DA,  = ax1.plot([], [], label="DA")
line_PB,  = ax2.plot([], [], color="purple", label="Pair Bonding")
line_wakefullness,  = ax3.plot([], [], label="Wakefullness")
line_social_need,   = ax3.plot([], [], label="Social Need")
line_entertainment, = ax3.plot([], [], label="Entertainment")
line_user_rejection,= ax3.plot([], [], label="User Rejection")

ax1.set_ylim(0, 0.5)
ax1.grid()
ax2.set_ylim(0, 100)
ax2.grid()
ax3.set_ylim(0, 60)
ax3.grid()


ax1.legend()
ax2.legend()
ax3.legend()

# MAIN LOOP
step = 0
while True:
    OT  = OT_gen.step(dt)
    AVP = AVP_gen.step(dt)
    DA  = DA_gen.step(dt)
    PB  = pair_bond.step(OT, AVP, dt)

    # Append new data
    t_data.append(step)
    OT_data.append(OT)
    AVP_data.append(AVP)
    DA_data.append(DA)
    PB_data.append(PB)

    state_data.append((
        state.wakefullness,
        state.social_need,
        state.entertainment,
        state.user_rejection
    ))

    # keep only last WINDOW data points 
    t_data  = t_data[-WINDOW:]
    OT_data = OT_data[-WINDOW:]
    AVP_data = AVP_data[-WINDOW:]
    DA_data = DA_data[-WINDOW:]
    PB_data = PB_data[-WINDOW:]
    state_data = state_data[-WINDOW:]


    # Update lines
    line_OT.set_data(t_data, OT_data)
    line_AVP.set_data(t_data, AVP_data)
    line_DA.set_data(t_data, DA_data)
    line_PB.set_data(t_data, PB_data)
    line_wakefullness.set_data(t_data, [s[0] for s in state_data])
    line_social_need.set_data(t_data, [s[1] for s in state_data])
    line_entertainment.set_data(t_data, [s[2] for s in state_data])
    line_user_rejection.set_data(t_data, [s[3] for s in state_data])

    ax1.set_xlim(t_data[0], t_data[-1])
    ax2.set_xlim(t_data[0], t_data[-1])
    ax3.set_xlim(t_data[0], t_data[-1])

    fig.canvas.draw()
    fig.canvas.flush_events()

    step += 1
    time.sleep(0.05)
