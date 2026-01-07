import numpy as np
import matplotlib.pyplot as plt
import random
import time
from matplotlib.lines import Line2D

# ============================================================
# Paper-faithful implementation (Maroto-Gómez et al., 2024)
# - dt = 0.5 s time step for applying effects (Table 5 text)
# - Hormones: circadian + stimulus effects (Eq. 3) + OT auto-stim (Table 1)
# - Processes: Table 2 + PB decay dr=0.01 (Eq. 1)
# - Deficits: Eq. 2
# - Motivations: Table 4 thresholds + WTA; 6 motivations total (incl. Complain/Thank)
# - Behaviors: Table 5 effects; reactive behaviors can overlap with long-lasting
# ============================================================

# -------------------------
# Paper constants / toggles
# -------------------------
DT_PAPER = 0.5  # seconds (paper applies behavior effects every 0.5 s step)
HORMONE_MIN, HORMONE_MAX = 0.01, 1.0
PROC_MIN, PROC_MAX = 0.0, 100.0

DR = 0.01  # PB decay rate in Eq. (1)
OT_AUTO_PER_STEP = 0.001  # Table 1 auto-stimulation (per time step)

# Paper inconsistency: Table 3 vs narrative example (Section 4.3.1)
USE_TABLE3_CARESS_PARAMS = True
# If False: uses narrative example values (0.0075 for OT caress and -0.02*intensity for SN)

# -------------------------
# Hormone circadian baseline
# -------------------------
class Hormone:
    def __init__(self, phase, amplitude=0.3, baseline=0.05):
        self.phase = float(phase)
        self.amplitude = float(amplitude)
        self.baseline = float(baseline)

    def circadian(self, t_hours):
        return self.baseline + self.amplitude * np.cos(2 * np.pi * (t_hours - self.phase) / 24.0)

def simulate_24h_hormones(OT_params, AVP_params, DA_params, dt_hours=1/60):
    t_axis = np.arange(0.0, 24.0 + 1e-9, dt_hours)
    OT_h = Hormone(**OT_params)
    AVP_h = Hormone(**AVP_params)
    DA_h = Hormone(**DA_params)
    OT_buf = np.array([OT_h.circadian(t) for t in t_axis], dtype=float)
    AVP_buf = np.array([AVP_h.circadian(t) for t in t_axis], dtype=float)
    DA_buf = np.array([DA_h.circadian(t) for t in t_axis], dtype=float)
    return t_axis, OT_buf, AVP_buf, DA_buf

def seconds_to_circadian_hours(sim_seconds):
    return (sim_seconds / 3600.0) % 24.0

def sample_circadian(t_hours, t_axis, hormone_buf):
    return float(np.interp(t_hours, t_axis, hormone_buf))

# -------------------------
# Internal processes (Table 2)
# -------------------------
class InternalState:
    def __init__(self, PB0=50.0):
        # Processes (range 0..100)
        self.WK = 5.0          # constant (Table 2)
        self.SN = 0.0          # Social need (Table 2)
        self.ENT = 0.0         # Entertainment (Table 2)
        self.PB = float(PB0)   # Pair-bonding (Table 2)
        self._clip_all()

    def _clip_all(self):
        self.SN = float(np.clip(self.SN, PROC_MIN, PROC_MAX))
        self.ENT = float(np.clip(self.ENT, PROC_MIN, PROC_MAX))
        self.PB = float(np.clip(self.PB, PROC_MIN, PROC_MAX))

    @property
    def UR(self):
        # Table 2
        return float(100.0 - self.PB)

    def update_processes(self, OT, AVP, DA, delta_SN_direct=0.0, dt_scale=1.0):
        """
        dt_scale = dt_seconds / DT_PAPER to keep per-step effects consistent
        when dt is changed. For paper equivalence: dt_seconds == DT_PAPER => dt_scale=1
        """
        # Table 2 equations
        self.SN += dt_scale * (0.02 * OT) + float(delta_SN_direct)
        self.ENT += dt_scale * (0.02 * DA)
        # Eq. (1): PB(t)=PB(t-1)+0.2*OT -0.2*AVP - dr
        self.PB += dt_scale * (0.2 * OT - 0.2 * AVP - DR)

        self._clip_all()

    def deficits(self):
        # Eq. (2): d_i(t) = |cv_i(t) - iv_i|
        # Ideals in Table 2: SN ideal=0, ENT ideal=0, WK ideal=5
        sn_def = abs(self.SN - 0.0)
        ent_def = abs(self.ENT - 0.0)
        wk_def = abs(self.WK - 5.0)  # will be 0 because WK constant
        return float(sn_def), float(ent_def), float(wk_def)

# -------------------------
# Stimulus effects (Eq. 3 + Table 3)
# -------------------------
class StimulusEffect:
    def __init__(self):
        # Table 3
        if USE_TABLE3_CARESS_PARAMS:
            caress_ot_alpha = 0.001
            caress_sn_alpha = -0.002
        else:
            # Narrative example (Section 4.3.1) uses 0.0075 and -0.02
            caress_ot_alpha = 0.0075
            caress_sn_alpha = -0.02

        self.caress_sn_alpha = caress_sn_alpha

        # stimulus -> list of (hormone, alpha, beta_mode)
        # beta_mode: 1.0 or "PB" where beta = 0.02*PB (Table 3)
        self.map = {
            "Hit": [
                ("AVP", +0.0075, 1.0),
                ("OT",  +0.0075, "PB"),
            ],
            "Caress": [
                ("OT",  caress_ot_alpha, "PB"),
            ],
            "CorrectAnswer": [
                ("DA", +0.005, 1.0),
            ],
            "WrongAnswer": [
                ("AVP", +0.002, 1.0),
            ],
            "UserPresence": [
                ("AVP", +0.003, 1.0),
                ("OT",  +0.003, "PB"),
                ("DA",  +0.004, 1.0),
            ],
        }

    def _beta(self, beta_mode, PB):
        if beta_mode == "PB":
            b = 0.02 * float(PB)  # Table 3
            # Paper text says beta cannot be 0; protect against PB=0 edge case
            if abs(b) < 1e-9:
                b = 1e-9
            return b
        return float(beta_mode)

    def apply(self, stim_name, intensity, PB):
        """
        Returns:
          hormone_delta dict for OT/AVP/DA
          delta_SN_direct for Social need (Caress in Table 3)
        """
        d = {"OT": 0.0, "AVP": 0.0, "DA": 0.0}
        delta_sn_direct = 0.0

        if stim_name not in self.map or intensity <= 0:
            return d, delta_sn_direct

        for (h, alpha, beta_mode) in self.map[stim_name]:
            beta = self._beta(beta_mode, PB)
            d[h] += float(alpha) * float(beta) * float(intensity)

        # Table 3: Caress directly affects Social need (alpha negative, beta=1)
        if stim_name == "Caress":
            delta_sn_direct += float(self.caress_sn_alpha) * float(intensity)

        return d, delta_sn_direct

# -------------------------
# Motivations (Table 4 + text)
# -------------------------
def compute_motivations(state: InternalState, user_presence_value, hit_intensity, caress_intensity):
    """
    Implements 6 motivations:
      Awake, Play, Socialize, Relax, Complain, Thank
    Table 4 gives explicit formulas for Awake/Play/Socialize/Relax.
    Text says Complain/Thank intensity fully depends on hit/caress intensity.
    Motivations range 0..100 (clip).
    """
    sn_def, ent_def, _wk_def = state.deficits()
    WK = state.WK
    UP = float(np.clip(user_presence_value, 0.0, 100.0))
    UR = state.UR

    # Table 4
    awake = WK
    play = ent_def * WK * (100.0 - UP)
    socialize = sn_def * WK * UP
    relax = UR * UP

    # Text: Complain/Thank depend fully on hit/caress intensity
    # Table 5 says "User present" needed -> gate by UP>0
    complain = float(hit_intensity) if (UP > 0 and hit_intensity > 0) else 0.0
    thank = float(caress_intensity) if (UP > 0 and caress_intensity > 0) else 0.0

    # Clip motivations to 0..100 (paper states motivation intensity range 0..100)
    def clip01(x): return float(np.clip(x, 0.0, 100.0))
    return {
        "Awake": clip01(awake),
        "Play": clip01(play),
        "Socialize": clip01(socialize),
        "Relax": clip01(relax),
        "Complain": clip01(complain),
        "Thank": clip01(thank),
    }

def select_long_lasting_behavior(motivations):
    """
    Winner-take-all among ACTIVE motivations above thresholds (Table 4 + text):
      Awake threshold 5
      Play threshold 30
      Socialize threshold 30
      Relax threshold 80
    Reactive motivations (Complain/Thank) are handled separately and can overlap.
    """
    thresholds = {"Awake": 5.0, "Play": 30.0, "Socialize": 30.0, "Relax": 80.0}

    candidates = {}
    for m, thr in thresholds.items():
        val = motivations[m]
        if val >= thr:
            candidates[m] = val

    # Fallback: if nothing above threshold, keep Awake available at least
    if not candidates:
        candidates["Awake"] = motivations["Awake"]

    dominant = max(candidates, key=candidates.get)

    # Map motivation -> behavior (paper’s scenario)
    if dominant == "Play":
        behavior = "Play quiz game"
    elif dominant == "Socialize":
        behavior = "Talk"
    elif dominant == "Relax":
        behavior = "Meditate"
    else:
        behavior = "Wait"

    return behavior, dominant

def apply_behavior_effects(state: InternalState, long_behavior, dt_scale=1.0):
    """
    Table 5 effects applied every time step of 0.5 s ONLY if behavior active.
    We apply to processes SN/ENT (range 0..100).
    """
    if long_behavior == "Play quiz game":
        state.ENT -= dt_scale * 0.04
        state.SN  -= dt_scale * 0.03
    elif long_behavior == "Talk":
        state.SN  -= dt_scale * 0.05

    # Wait/Meditate: no process effects in Table 5
    state._clip_all()

# -------------------------
# Example user-profile stimuli generator (Table 6-inspired)
# -------------------------
def base_user_presence(t_s):
    # Similar to your earlier demo: presence in two intervals
    t = int(round(t_s))
    return 100 if (50 <= t < 500) or (610 <= t < 800) else 0

def profile_stimuli(profile, t_s, user_present, prev_long_behavior):
    """
    Hits/caresses independent of behavior (but require user present).
    Answers occur mainly when playing (stimulus definition includes quiz answers).
    """
    hit = caress = correct = wrong = 0

    if user_present <= 0:
        return hit, caress, correct, wrong

    # tactile
    if profile == "friendly":
        # mostly caresses
        if random.random() < 0.10:  # ~10% chance each 0.5s step
            caress = random.randint(60, 100)
    elif profile == "aversive":
        # mostly hits
        if random.random() < 0.12:
            hit = random.randint(60, 100)
    else:  # naive
        # mix
        r = random.random()
        if r < 0.06:
            hit = random.randint(50, 100)
        elif r < 0.12:
            caress = random.randint(50, 100)

    # answers (quiz) -> only when playing
    if prev_long_behavior == "Play quiz game":
        if profile == "friendly":
            if random.random() < 0.25:
                correct = random.randint(60, 100)
        elif profile == "aversive":
            # does not answer
            pass
        else:  # naive
            r = random.random()
            if r < 0.12:
                correct = random.randint(50, 100)
            elif r < 0.22:
                wrong = random.randint(50, 100)

    return hit, caress, correct, wrong

# -------------------------
# Live keyboard injection
# -------------------------
pending_events = {"Hit": 0, "Caress": 0, "CorrectAnswer": 0, "WrongAnswer": 0, "UserPresence": None}

def on_key(event):
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
        pending_events["CorrectAnswer"] = random.randint(50, 100)
        print("Injected: CorrectAnswer")
    elif k == "w":
        pending_events["WrongAnswer"] = random.randint(50, 100)
        print("Injected: WrongAnswer")
    elif k == "u":
        if pending_events["UserPresence"] is None:
            pending_events["UserPresence"] = 100
        elif pending_events["UserPresence"] == 100:
            pending_events["UserPresence"] = 0
        else:
            pending_events["UserPresence"] = None
        print(f"Injected: UserPresence override = {pending_events['UserPresence']}")

# -------------------------
# Real-time simulation
# -------------------------
def run_realtime_simulation(
    sim_duration_seconds=800.0,
    dt_real_seconds=0.5,         # keep 0.5 for paper equivalence
    realtime_factor=0.02,
    window_points=1600,
    circadian_dt_hours=1/60,
    profile="naive",             # "friendly", "aversive", "naive"
):
    # Table 1 circadian parameters
    OT_params = {"phase": 10, "amplitude": 0.3, "baseline": 0.05}
    AVP_params = {"phase": 10, "amplitude": 0.3, "baseline": 0.05}
    DA_params = {"phase": 20, "amplitude": 0.3, "baseline": 0.05}

    circ_t, OT_buf, AVP_buf, DA_buf = simulate_24h_hormones(OT_params, AVP_params, DA_params, dt_hours=circadian_dt_hours)

    stim_model = StimulusEffect()
    state = InternalState(PB0=50.0)

    # Scale relative to paper timestep (0.5 s)
    dt_scale = float(dt_real_seconds / DT_PAPER)

    # Logs
    t_data, PB_data = [], []
    OT_data, AVP_data, DA_data = [], [], []

    up_data, hit_data, caress_data, corr_data, wrong_data = [], [], [], [], []

    # motivations (6)
    m_awake, m_play, m_soc, m_relax, m_comp, m_thank = [], [], [], [], [], []

    # behaviors (4 long-lasting + 2 reactive)
    b_wait, b_play, b_talk, b_med, b_comp, b_thank = [], [], [], [], [], []

    # Plot
    plt.ion()
    fig, axes = plt.subplots(5, 1, figsize=(14, 10), sharex=True, gridspec_kw={"hspace": 0.35})
    fig.canvas.mpl_connect("key_press_event", on_key)

    (line_pb,) = axes[0].plot([], [], color="purple", lw=2, label="Pair-bonding (PB)")
    axes[0].set_ylabel("PB (0–100)"); axes[0].set_ylim(0, 100)
    axes[0].legend(loc="upper right"); axes[0].grid(True)

    (line_ot,) = axes[1].plot([], [], color="blue", label="Oxytocin (OT)")
    (line_avp,) = axes[1].plot([], [], color="red", label="Arginine vasopressin (AVP)")
    (line_da,) = axes[1].plot([], [], color="green", label="Dopamine (DA)")
    axes[1].set_ylabel("Hormone\nsecretion rate"); axes[1].set_ylim(0, 1.05)
    axes[1].legend(loc="upper right", ncol=3); axes[1].grid(True)

    (line_up,) = axes[2].step([], [], where="post", color="blue", label="User Presence")
    axes[2].set_ylabel("Stimuli\nintensity"); axes[2].set_ylim(0, 110)
    axes[2].grid(True)
    stim_legend_handles = [
        Line2D([0], [0], color="blue", lw=2, label="User Presence"),
        Line2D([0], [0], color="magenta", lw=2, label="Hits"),
        Line2D([0], [0], color="orange", lw=2, label="Caresses"),
        Line2D([0], [0], color="green", lw=2, label="Correct Answers"),
        Line2D([0], [0], color="red", lw=2, label="Wrong Answers"),
    ]
    axes[2].legend(handles=stim_legend_handles, loc="upper right", ncol=3)

    # motivations (6)
    (l_awake,) = axes[3].plot([], [], label="Awake")
    (l_play,)  = axes[3].plot([], [], label="Play")
    (l_soc,)   = axes[3].plot([], [], label="Socialize")
    (l_relax,) = axes[3].plot([], [], label="Relax")
    (l_comp,)  = axes[3].plot([], [], label="Complain")
    (l_thank,) = axes[3].plot([], [], label="Thank")
    axes[3].set_ylabel("Motivations\n(0–100)"); axes[3].set_ylim(0, 110)
    # show key thresholds
    axes[3].axhline(30, linestyle="--", alpha=0.3)
    axes[3].axhline(80, linestyle=":", alpha=0.3)
    axes[3].legend(loc="upper right", ncol=3); axes[3].grid(True)

    # behaviors (4 long-lasting + 2 reactive)
    (lw,) = axes[4].step([], [], where="post", label="Wait")
    (lp,) = axes[4].step([], [], where="post", label="Play quiz game")
    (lt,) = axes[4].step([], [], where="post", label="Talk")
    (lm,) = axes[4].step([], [], where="post", label="Meditate")
    (lc,) = axes[4].step([], [], where="post", label="Complain (react)")
    (lth,) = axes[4].step([], [], where="post", label="Thank (react)")
    axes[4].set_ylabel("Behaviors\n(active)"); axes[4].set_ylim(0, 1.2)
    axes[4].set_xlabel("Time (s)")
    axes[4].legend(loc="upper right", ncol=3); axes[4].grid(True)

    # Simulation loop
    sim_time_seconds = 0.0
    n_steps = int(np.ceil(sim_duration_seconds / dt_real_seconds))

    prev_long_behavior = "Wait"

    for _ in range(n_steps):
        # --- stimuli (presence + profile-based)
        user_presence = base_user_presence(sim_time_seconds)

        # override
        if pending_events["UserPresence"] is not None:
            user_presence = pending_events["UserPresence"]

        hit, caress, corr, wrong = profile_stimuli(profile, sim_time_seconds, user_presence, prev_long_behavior)

        # injected impulses
        if pending_events["Hit"] > 0:
            hit = max(hit, pending_events["Hit"]); pending_events["Hit"] = 0
        if pending_events["Caress"] > 0:
            caress = max(caress, pending_events["Caress"]); pending_events["Caress"] = 0
        if pending_events["CorrectAnswer"] > 0:
            corr = max(corr, pending_events["CorrectAnswer"]); pending_events["CorrectAnswer"] = 0
        if pending_events["WrongAnswer"] > 0:
            wrong = max(wrong, pending_events["WrongAnswer"]); pending_events["WrongAnswer"] = 0

        # --- stimulus effects
        delta_h = {"OT": 0.0, "AVP": 0.0, "DA": 0.0}
        delta_sn_direct = 0.0

        def accumulate(stim_name, intensity):
            nonlocal delta_sn_direct
            d, dsn = stim_model.apply(stim_name, intensity=intensity, PB=state.PB)
            for k in delta_h:
                delta_h[k] += d[k]
            delta_sn_direct += dsn

        if user_presence > 0: accumulate("UserPresence", user_presence)
        if hit > 0:          accumulate("Hit", hit)
        if caress > 0:       accumulate("Caress", caress)
        if corr > 0:         accumulate("CorrectAnswer", corr)
        if wrong > 0:        accumulate("WrongAnswer", wrong)

        # --- circadian sampling
        t_hours = seconds_to_circadian_hours(sim_time_seconds)
        OT_base = sample_circadian(t_hours, circ_t, OT_buf)
        AVP_base = sample_circadian(t_hours, circ_t, AVP_buf)
        DA_base = sample_circadian(t_hours, circ_t, DA_buf)

        # --- hormones = baseline + stimuli + auto (OT only), then clip
        OT = float(np.clip(OT_base + delta_h["OT"] + dt_scale * OT_AUTO_PER_STEP, HORMONE_MIN, HORMONE_MAX))
        AVP = float(np.clip(AVP_base + delta_h["AVP"], HORMONE_MIN, HORMONE_MAX))
        DA = float(np.clip(DA_base + delta_h["DA"], HORMONE_MIN, HORMONE_MAX))

        # --- update processes
        state.update_processes(OT, AVP, DA, delta_SN_direct=delta_sn_direct, dt_scale=dt_scale)

        # --- motivations (6) + behaviors
        motivations = compute_motivations(state, user_presence, hit, caress)
        long_behavior, dominant_motivation = select_long_lasting_behavior(motivations)

        # reactive behaviors can overlap
        complain_active = 1 if motivations["Complain"] > 0 else 0
        thank_active = 1 if motivations["Thank"] > 0 else 0

        # apply long-lasting behavior effects (Table 5)
        apply_behavior_effects(state, long_behavior, dt_scale=dt_scale)

        # log
        t_data.append(sim_time_seconds)
        PB_data.append(state.PB)
        OT_data.append(OT); AVP_data.append(AVP); DA_data.append(DA)

        up_data.append(user_presence)
        hit_data.append(hit); caress_data.append(caress); corr_data.append(corr); wrong_data.append(wrong)

        m_awake.append(motivations["Awake"])
        m_play.append(motivations["Play"])
        m_soc.append(motivations["Socialize"])
        m_relax.append(motivations["Relax"])
        m_comp.append(motivations["Complain"])
        m_thank.append(motivations["Thank"])

        b_wait.append(1 if long_behavior == "Wait" else 0)
        b_play.append(1 if long_behavior == "Play quiz game" else 0)
        b_talk.append(1 if long_behavior == "Talk" else 0)
        b_med.append(1 if long_behavior == "Meditate" else 0)
        b_comp.append(complain_active)
        b_thank.append(thank_active)

        prev_long_behavior = long_behavior

        # windowing
        if len(t_data) > window_points:
            sl = slice(-window_points, None)
            t_data = t_data[sl]; PB_data = PB_data[sl]
            OT_data = OT_data[sl]; AVP_data = AVP_data[sl]; DA_data = DA_data[sl]
            up_data = up_data[sl]
            hit_data = hit_data[sl]; caress_data = caress_data[sl]; corr_data = corr_data[sl]; wrong_data = wrong_data[sl]
            m_awake = m_awake[sl]; m_play = m_play[sl]; m_soc = m_soc[sl]; m_relax = m_relax[sl]; m_comp = m_comp[sl]; m_thank = m_thank[sl]
            b_wait = b_wait[sl]; b_play = b_play[sl]; b_talk = b_talk[sl]; b_med = b_med[sl]; b_comp = b_comp[sl]; b_thank = b_thank[sl]

        # update plots
        line_pb.set_data(t_data, PB_data)
        line_ot.set_data(t_data, OT_data)
        line_avp.set_data(t_data, AVP_data)
        line_da.set_data(t_data, DA_data)
        line_up.set_data(t_data, up_data)

        l_awake.set_data(t_data, m_awake)
        l_play.set_data(t_data, m_play)
        l_soc.set_data(t_data, m_soc)
        l_relax.set_data(t_data, m_relax)
        l_comp.set_data(t_data, m_comp)
        l_thank.set_data(t_data, m_thank)

        lw.set_data(t_data, b_wait)
        lp.set_data(t_data, b_play)
        lt.set_data(t_data, b_talk)
        lm.set_data(t_data, b_med)
        lc.set_data(t_data, b_comp)
        lth.set_data(t_data, b_thank)

        # refresh vlines on stimuli axis
        for coll in list(axes[2].collections):
            coll.remove()
        tt = np.array(t_data, dtype=float)
        axes[2].vlines(tt, 0, hit_data, color="magenta")
        axes[2].vlines(tt, 0, caress_data, color="orange")
        axes[2].vlines(tt, 0, corr_data, color="green")
        axes[2].vlines(tt, 0, wrong_data, color="red")

        xmin = t_data[0] if t_data else 0.0
        xmax = t_data[-1] if t_data else 1.0
        for ax in axes:
            ax.set_xlim(xmin, xmax)

        fig.canvas.draw()
        fig.canvas.flush_events()

        time.sleep(realtime_factor)
        sim_time_seconds += dt_real_seconds

    plt.ioff()
    plt.show()

if __name__ == "__main__":
    print("Paper-faithful Mini model (Maroto-Gómez et al., 2024)")
    print("Controls: h=Hit, c=Caress, a=Correct, w=Wrong, u=toggle presence override, q=quit")
    print(f"USE_TABLE3_CARESS_PARAMS = {USE_TABLE3_CARESS_PARAMS}")
    run_realtime_simulation(
        sim_duration_seconds=800.0,
        dt_real_seconds=0.5,
        realtime_factor=0.02,
        window_points=1600,
        circadian_dt_hours=1/60,
        profile="naive",  # try: "friendly", "aversive", "naive"
    )
