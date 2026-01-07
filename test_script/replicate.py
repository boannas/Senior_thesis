import numpy as np
import matplotlib.pyplot as plt
import random
import time

# =========================================
# MODEL COMPONENTS (paper-aligned structure)
# =========================================

class Hormone:
    """
    Hormone level = circadian baseline + instantaneous stimulus effects
    clipped to [0.01, 1.0] (paper configuration).
    """
    def __init__(self, phase, amplitude=0.3, baseline=0.05):
        self.phase = phase
        self.amplitude = amplitude
        self.baseline = baseline
        self.t = 0.0
        self.level = baseline

    def circadian(self):
        return self.baseline + self.amplitude * np.cos(2 * np.pi * (self.t - self.phase) / 24.0)

    def step(self, dt, stim_effect=0.0):
        self.t += dt
        self.level = self.circadian() + stim_effect
        self.level = float(np.clip(self.level, 0.01, 1.0))
        return self.level


class InternalState:
    """
    Pair-bonding (PB) update and internal state accumulation per paper tables:
    SN(t)=SN(t-1)+0.02*OT
    ENT(t)=ENT(t-1)+0.02*DA
    PB(t)=PB(t-1)+0.2*OT-0.2*AVP-dr
    UR=100-PB
    """
    def __init__(self, PB0=50.0, decay=0.01, wakefulness=5.0):
        self.PB = PB0
        self.decay = decay
        self.wakefulness = wakefulness
        self.social_need = 0.0
        self.entertainment = 0.0

    def update(self, OT, AVP, DA):
        self.social_need += 0.02 * OT
        self.entertainment += 0.02 * DA
        self.PB += (0.2 * OT - 0.2 * AVP - self.decay)
        self.PB = float(np.clip(self.PB, 0.0, 100.0))

    @property
    def user_rejection(self):
        return 100.0 - self.PB


class StimulusEffect:
    """
    se_k(t) = alpha_k * beta_k * s_k(t)
    beta for social stimuli can be PB-modulated (allostatic regulation).
    This is a compact, usable mapping consistent with the model structure.
    """
    def __init__(self):
        # stimulus_name -> hormone -> (alpha, beta_mode)
        # beta_mode: float or "PB"
        self.map = {
            "Hit":            {"AVP": (0.0075, 1.0)},
            "Caress":         {"OT":  (0.0075, "PB")},
            "CorrectAnswer":  {"DA":  (0.0050, 1.0)},
            "WrongAnswer":    {"AVP": (0.0020, 1.0)},
            "UserPresence":   {"OT":  (0.0030, "PB"), "DA": (0.0040, 1.0)},
        }

    def compute_delta(self, stim_name, intensity, PB):
        d = {"OT": 0.0, "AVP": 0.0, "DA": 0.0}
        if stim_name is None:
            return d
        if stim_name not in self.map:
            return d

        for h, (alpha, beta_mode) in self.map[stim_name].items():
            if beta_mode == "PB":
                beta = 0.02 * PB  # PB-modulated cognition
            else:
                beta = float(beta_mode)
            d[h] += alpha * beta * float(intensity)
        return d


def compute_motivations(state: InternalState, user_present: float):
    """
    Motivations in a similar spirit to paper: competing drives influenced by deficits,
    user presence, and internal state.
    Output scaled to roughly [0, 100] range for the plot.
    """
    # Use simple bounded scalings for interpretability
    sn = np.clip(abs(state.social_need) * 25.0, 0.0, 100.0)
    ent = np.clip(abs(state.entertainment) * 25.0, 0.0, 100.0)
    ur = np.clip(state.user_rejection, 0.0, 100.0)

    # user_present: 0 or 100
    up = np.clip(user_present / 100.0, 0.0, 1.0)

    play = ent * (1.0 - up)          # more play when user absent
    socialize = sn * up              # more socialize when user present
    relax = ur * up                  # more relax when user present but rejection high

    return float(play), float(socialize), float(relax)


def select_behavior(play, socialize, relax):
    """
    Winner-take-all to yield binary behavior activations like panel (e).
    """
    scores = {"Wait": 0.0, "Play quiz game": play, "Talk": socialize, "Meditate": relax}
    # Give Wait a small floor so it can win when all are near zero
    scores["Wait"] = 5.0

    chosen = max(scores, key=scores.get)

    Wait = 1 if chosen == "Wait" else 0
    PlayB = 1 if chosen == "Play quiz game" else 0
    Talk = 1 if chosen == "Talk" else 0
    Meditate = 1 if chosen == "Meditate" else 0
    return chosen, Wait, PlayB, Talk, Meditate


def apply_behavior_feedback(state: InternalState, chosen_behavior: str):
    """
    Behavior reduces deficits (paper table spirit).
    """
    if chosen_behavior == "Play quiz game":
        state.entertainment -= 0.04
        state.social_need -= 0.03
    elif chosen_behavior == "Talk":
        state.social_need -= 0.05
    # Meditate/Wait: no direct reductions in this minimal setup


# =========================================
# SCRIPTED STIMULI SCHEDULE (to mimic paper)
# =========================================

def scenario_stimuli(t):
    """
    Returns:
      user_presence (0/100),
      event impulses for Hit/Caress/Correct/Wrong (0..100 intensity or 0)
    Designed to create a figure qualitatively similar to the paper example.
    """
    # User presence blocks: ON (50..500), OFF (500..610), ON (610..800)
    user_presence = 100 if (50 <= t < 500) or (610 <= t < 800) else 0

    hit = 0
    caress = 0
    correct = 0
    wrong = 0

    # Early: many hits (stress) around 90..300
    if 90 <= t <= 300 and (t % 20 == 0 or t % 35 == 0):
        hit = random.randint(60, 100)

    # Some wrong answers sprinkled
    if 100 <= t <= 320 and (t % 50 == 0):
        wrong = random.randint(50, 90)

    # Later: caresses (prosocial) around 650..800
    if 650 <= t <= 800 and (t % 40 == 0 or t % 70 == 0):
        caress = random.randint(60, 100)

    # Correct answers occasional
    if 120 <= t <= 240 and (t % 60 == 0):
        correct = random.randint(50, 90)

    return user_presence, hit, caress, correct, wrong


# =========================================
# OPTIONAL: LIVE KEYBOARD INJECTION
# =========================================
pending_events = {"Hit": 0, "Caress": 0, "CorrectAnswer": 0, "WrongAnswer": 0, "UserPresence": None}

def on_key(event):
    """
    Inject stimuli during runtime:
      u: toggle UserPresence (on/off)
      h: Hit impulse
      c: Caress impulse
      a: CorrectAnswer impulse
      w: WrongAnswer impulse
      q: quit
    """
    k = event.key.lower()
    if k == "q":
        plt.close("all")
        raise SystemExit

    if k == "h":
        pending_events["Hit"] = random.randint(60, 100)
        print("Injected: Hit")
    elif k == "c":
        pending_events["Caress"] = random.randint(60, 100)
        print("Injected: Caress")
    elif k == "a":
        pending_events["CorrectAnswer"] = random.randint(50, 90)
        print("Injected: CorrectAnswer")
    elif k == "w":
        pending_events["WrongAnswer"] = random.randint(50, 90)
        print("Injected: WrongAnswer")
    elif k == "u":
        # toggle: None means "use scripted"; otherwise override 0/100
        if pending_events["UserPresence"] is None:
            pending_events["UserPresence"] = 100
        elif pending_events["UserPresence"] == 100:
            pending_events["UserPresence"] = 0
        else:
            pending_events["UserPresence"] = None
        print(f"Injected: UserPresence override = {pending_events['UserPresence']}")


# =========================================
# REAL-TIME PLOT SETUP (panel a–e)
# =========================================
def run_realtime_simulation(T=800, dt=1.0, realtime_factor=0.03, window=800):
    # Init hormones (phases similar to your prototype)
    OT_h = Hormone(phase=10)
    AVP_h = Hormone(phase=10)
    DA_h = Hormone(phase=20)

    stim_model = StimulusEffect()
    state = InternalState(PB0=50.0, decay=0.01, wakefulness=5.0)

    # Data buffers
    t_data = []
    PB_data = []
    OT_data, AVP_data, DA_data = [], [], []

    up_data = []
    hit_data, caress_data, corr_data, wrong_data = [], [], [], []

    play_m, soc_m, relax_m = [], [], []
    wait_b, play_b, talk_b, med_b = [], [], [], []

    # Figure
    plt.ion()
    fig, axes = plt.subplots(5, 1, figsize=(14, 9), sharex=True, gridspec_kw={"hspace": 0.35})
    fig.canvas.mpl_connect("key_press_event", on_key)

    # Pre-create artists (for speed)
    # a)
    (line_pb,) = axes[0].plot([], [], color="purple", lw=2, label="Pair-bonding")
    axes[0].set_ylabel("Pair-bonding")
    axes[0].set_ylim(0, 100)
    axes[0].legend(loc="upper center")
    axes[0].grid(True)
    axes[0].text(-0.05, 0.9, "a)", transform=axes[0].transAxes, fontsize=12)

    # b)
    (line_ot,) = axes[1].plot([], [], color="blue", label="Oxytocin")
    (line_avp,) = axes[1].plot([], [], color="red", label="Arginine vasopressin")
    (line_da,) = axes[1].plot([], [], color="green", label="Dopamine")
    axes[1].set_ylabel("Hormones\nsecretion rate")
    axes[1].set_ylim(0, 1.05)
    axes[1].legend(loc="upper center", ncol=3)
    axes[1].grid(True)
    axes[1].text(-0.05, 0.9, "b)", transform=axes[1].transAxes, fontsize=12)

    # c) stimuli: user presence as step line + event vlines
    (line_up,) = axes[2].step([], [], where="post", color="blue", label="User Presence")
    # We'll re-draw vlines each update (simple, readable)
    axes[2].set_ylabel("Stimuli\nintensity")
    axes[2].set_ylim(0, 110)
    axes[2].legend(loc="upper center", ncol=5)
    axes[2].grid(True)
    axes[2].text(-0.05, 0.9, "c)", transform=axes[2].transAxes, fontsize=12)

    # d) motivations
    (line_play,) = axes[3].plot([], [], color="red", label="Play")
    (line_soc,) = axes[3].plot([], [], color="blue", label="Socialize")
    (line_relax,) = axes[3].plot([], [], color="green", label="Relax")
    axes[3].set_ylabel("Motivations")
    axes[3].set_ylim(0, 110)
    axes[3].legend(loc="upper center", ncol=3)
    axes[3].grid(True)
    axes[3].text(-0.05, 0.9, "d)", transform=axes[3].transAxes, fontsize=12)

    # e) behaviors (binary)
    (line_wait,) = axes[4].step([], [], where="post", color="gold", label="Wait")
    (line_playb,) = axes[4].step([], [], where="post", color="lime", label="Play quiz game")
    (line_talk,) = axes[4].step([], [], where="post", color="cyan", label="Talk")
    (line_med,) = axes[4].step([], [], where="post", color="red", label="Meditate")
    axes[4].set_ylabel("Behavior\nactivation")
    axes[4].set_ylim(0, 1.2)
    axes[4].set_xlabel("Time (s)")
    axes[4].legend(loc="upper center", ncol=4)
    axes[4].grid(True)
    axes[4].text(-0.05, 0.9, "e)", transform=axes[4].transAxes, fontsize=12)

    # Simulation loop
    for t in range(int(T)):
        # Scripted stimuli
        user_presence, hit, caress, corr, wrong = scenario_stimuli(t)

        # Apply optional user presence override
        if pending_events["UserPresence"] is not None:
            user_presence = pending_events["UserPresence"]

        # Merge injected impulses
        if pending_events["Hit"] > 0:
            hit = max(hit, pending_events["Hit"])
            pending_events["Hit"] = 0
        if pending_events["Caress"] > 0:
            caress = max(caress, pending_events["Caress"])
            pending_events["Caress"] = 0
        if pending_events["CorrectAnswer"] > 0:
            corr = max(corr, pending_events["CorrectAnswer"])
            pending_events["CorrectAnswer"] = 0
        if pending_events["WrongAnswer"] > 0:
            wrong = max(wrong, pending_events["WrongAnswer"])
            pending_events["WrongAnswer"] = 0

        # Translate event impulses into hormone effects (allow multiple events same timestep)
        delta = {"OT": 0.0, "AVP": 0.0, "DA": 0.0}

        # Presence as a continuous stimulus while present (use intensity 100 when present)
        if user_presence > 0:
            d = stim_model.compute_delta("UserPresence", intensity=user_presence, PB=state.PB)
            for k in delta:
                delta[k] += d[k]

        # Event impulses
        if hit > 0:
            d = stim_model.compute_delta("Hit", intensity=hit, PB=state.PB)
            for k in delta:
                delta[k] += d[k]
        if caress > 0:
            d = stim_model.compute_delta("Caress", intensity=caress, PB=state.PB)
            for k in delta:
                delta[k] += d[k]
        if corr > 0:
            d = stim_model.compute_delta("CorrectAnswer", intensity=corr, PB=state.PB)
            for k in delta:
                delta[k] += d[k]
        if wrong > 0:
            d = stim_model.compute_delta("WrongAnswer", intensity=wrong, PB=state.PB)
            for k in delta:
                delta[k] += d[k]

        # Step hormones
        OT = OT_h.step(dt, delta["OT"])
        AVP = AVP_h.step(dt, delta["AVP"])
        DA = DA_h.step(dt, delta["DA"])

        # Update internal state
        state.update(OT, AVP, DA)

        # Motivations and behavior
        play, socialize, relax = compute_motivations(state, user_present=user_presence)
        chosen, Wait, PlayB, Talk, Meditate = select_behavior(play, socialize, relax)
        apply_behavior_feedback(state, chosen)

        # Log
        t_data.append(t)
        PB_data.append(state.PB)
        OT_data.append(OT); AVP_data.append(AVP); DA_data.append(DA)

        up_data.append(user_presence)
        hit_data.append(hit); caress_data.append(caress); corr_data.append(corr); wrong_data.append(wrong)

        play_m.append(play); soc_m.append(socialize); relax_m.append(relax)
        wait_b.append(Wait); play_b.append(PlayB); talk_b.append(Talk); med_b.append(Meditate)

        # Apply windowing
        if len(t_data) > window:
            t_data = t_data[-window:]
            PB_data = PB_data[-window:]
            OT_data = OT_data[-window:]; AVP_data = AVP_data[-window:]; DA_data = DA_data[-window:]
            up_data = up_data[-window:]
            hit_data = hit_data[-window:]; caress_data = caress_data[-window:]
            corr_data = corr_data[-window:]; wrong_data = wrong_data[-window:]
            play_m = play_m[-window:]; soc_m = soc_m[-window:]; relax_m = relax_m[-window:]
            wait_b = wait_b[-window:]; play_b = play_b[-window:]; talk_b = talk_b[-window:]; med_b = med_b[-window:]

        # Update lines
        line_pb.set_data(t_data, PB_data)
        line_ot.set_data(t_data, OT_data)
        line_avp.set_data(t_data, AVP_data)
        line_da.set_data(t_data, DA_data)
        line_up.set_data(t_data, up_data)

        line_play.set_data(t_data, play_m)
        line_soc.set_data(t_data, soc_m)
        line_relax.set_data(t_data, relax_m)

        line_wait.set_data(t_data, wait_b)
        line_playb.set_data(t_data, play_b)
        line_talk.set_data(t_data, talk_b)
        line_med.set_data(t_data, med_b)

        # Refresh stimuli vlines (simple approach)
        for coll in list(axes[2].collections):
            coll.remove()


        tt = np.array(t_data)
        axes[2].vlines(tt, 0, hit_data, color="magenta", label="Hits")
        axes[2].vlines(tt, 0, caress_data, color="orange", label="Caresses")
        axes[2].vlines(tt, 0, corr_data, color="green", label="Correct Answers")
        axes[2].vlines(tt, 0, wrong_data, color="red", label="Wrong Answers")

        # X-limits
        if len(t_data) > 1:
            xmin = t_data[0]
            xmax = t_data[-1]
        else:
            xmin = 0
            xmax = 1

        for ax in axes:
            ax.set_xlim(xmin, xmax)


        fig.canvas.draw()
        fig.canvas.flush_events()

        # real-time pacing (accelerated)
        time.sleep(realtime_factor)

    plt.ioff()
    plt.show()


if __name__ == "__main__":
    print("Controls: h=Hit, c=Caress, a=Correct, w=Wrong, u=toggle presence override, q=quit")
    run_realtime_simulation(T=800, dt=1.0, realtime_factor=0.02, window=800)
