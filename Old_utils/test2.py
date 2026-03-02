from pynput import keyboard as pynput_keyboard
import time
import math
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from collections import deque
import threading
import numpy as np


# STIMULUS TABLE (Eq. 3 mapping)
class StimulusTable:
    def __init__(self):
        self.table = {
            "HIT": [
                {"target": "AVP", "alpha": 0.0075, "beta": 1.0}
            ],
            "CARESS": [
                {"target": "OT", "alpha": 0.0075, "beta": "0.02 * PB"}
            ],
            "CORRECT": [
                {"target": "OT", "alpha": 0.001, "beta": "0.02 * PB"},
                {"target": "DA", "alpha": 0.005, "beta": 1.0},
            ],
            "WRONG": [
                {"target": "AVP", "alpha": 0.002, "beta": 1.0}
            ],
            "USER_PRESENCE": [
                {"target": "AVP", "alpha": 0.003, "beta": 1.0},
                {"target": "OT",  "alpha": 0.003, "beta": "0.02 * PB"},
                {"target": "DA",  "alpha": 0.004, "beta": 1.0},
            ],
        }

    def get_effects(self, stim):
        return self.table.get(stim, [])


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


# KEYBOARD INPUT
class KeyboardInput:
    def __init__(self):
        self.running = True
        self.user_present = False

        self.cares_active = False
        self.cares_intensity = 85.0
        self.cares_latch_steps = 0

        self.pending_impulses = deque()  # queue of (stim, intensity)

    def on_press(self, key):
        try:
            k = key.char.lower()
        except AttributeError:
            return

        if k == "q":
            self.running = False
            return False

        if k == "u":
            self.user_present = not self.user_present
            print(f"[TOGGLE] USER_PRESENCE={self.user_present}")
            return

        if k == "c":
            self.cares_active = True
            self.cares_latch_steps = 1  # guarantee at least 1 sampled step
            return

        if k == "h":
            self.pending_impulses.append(("HIT", 100.0))
        elif k == "a":
            self.pending_impulses.append(("CORRECT", 100.0))
        elif k == "w":
            self.pending_impulses.append(("WRONG", 100.0))

    def on_release(self, key):
        try:
            k = key.char.lower()
        except AttributeError:
            return
        if k == "c":
            self.cares_active = False

    def start(self):
        listener = pynput_keyboard.Listener(on_press=self.on_press, on_release=self.on_release)
        listener.start()


# -----------------------------
# REAL-TIME FIG.4-STYLE PLOTTER
# - bottom panel: presence as line, others as spikes via vlines
# -----------------------------
class Fig4RealtimePlotter:
    def __init__(self, hs: HormoneSystem, window_s=60):
        self.hs = hs
        self.window_s = float(window_s)

        self.fig, (self.ax_pb, self.ax_h, self.ax_si) = plt.subplots(
            3, 1, figsize=(16, 8.5), sharex=True
        )

        # (a) PB
        self.line_PB, = self.ax_pb.plot([], [], linewidth=2.5, label="Pair-bonding")
        self.ax_pb.set_ylabel("Pair-bonding")
        self.ax_pb.set_ylim(0, 105)
        self.ax_pb.grid(True, alpha=0.25)
        self.ax_pb.legend(loc="upper center")

        # (b) hormones
        self.line_OT,  = self.ax_h.plot([], [], linewidth=2.0, label="Oxytocin")
        self.line_AVP, = self.ax_h.plot([], [], linewidth=2.0, label="Arginine vasopressin")
        self.line_DA,  = self.ax_h.plot([], [], linewidth=2.0, label="Dopamine")
        self.ax_h.set_ylabel("Hormones\nsecretion rate")
        self.ax_h.set_ylim(0, 1.05)
        self.ax_h.grid(True, alpha=0.25)
        self.ax_h.legend(loc="upper center", ncol=3)

        # (c) stimuli
        self.line_user, = self.ax_si.plot([], [], linewidth=2.0, label="User Presence")
        self.line_care, = self.ax_si.plot([], [], linewidth=1.5, label="Caresses (held)")
        self.ax_si.set_ylabel("Stimuli\nintensity")
        self.ax_si.set_xlabel("Time (s)")
        self.ax_si.set_ylim(0, 110)
        self.ax_si.grid(True, alpha=0.25)

        # panel labels
        self.ax_pb.text(-0.06, 0.5, "a)", transform=self.ax_pb.transAxes,
                        fontsize=14, fontweight="bold", va="center")
        self.ax_h.text(-0.06, 0.5, "b)", transform=self.ax_h.transAxes,
                       fontsize=14, fontweight="bold", va="center")
        self.ax_si.text(-0.06, 0.5, "c)", transform=self.ax_si.transAxes,
                        fontsize=14, fontweight="bold", va="center")

        # legend for impulses as spikes (we add proxy artists)
        from matplotlib.lines import Line2D
        proxies = [
            Line2D([0], [0], color="C1", lw=2, label="Hits (spike)"),
            Line2D([0], [0], color="C3", lw=2, label="Correct (spike)"),
            Line2D([0], [0], color="C4", lw=2, label="Wrong (spike)"),
        ]
        handles, labels = self.ax_si.get_legend_handles_labels()
        self.ax_si.legend(handles + proxies, labels + [p.get_label() for p in proxies],
                          loc="upper center", ncol=5)

        # keep spike artists to delete each frame
        self.spike_artists = []

        plt.tight_layout()

    def init_plot(self):
        for ln in [self.line_PB, self.line_OT, self.line_AVP, self.line_DA, self.line_user, self.line_care]:
            ln.set_data([], [])
        return (self.line_PB, self.line_OT, self.line_AVP, self.line_DA, self.line_user, self.line_care)

    def _clear_spikes(self):
        for a in self.spike_artists:
            try:
                a.remove()
            except Exception:
                pass
        self.spike_artists.clear()

    def _draw_spikes(self, x, y, color):
        idx = np.where(y > 0)[0]
        if len(idx) == 0:
            return
        art = self.ax_si.vlines(x[idx], 0, y[idx], colors=color, linewidth=2.0)
        self.spike_artists.append(art)

    def update_plot(self, _):
        if len(self.hs.time_history) < 2:
            return self.init_plot()

        t = np.array(self.hs.time_history)

        # window
        t_end = t[-1]
        t_start = max(0.0, t_end - self.window_s)
        mask = (t >= t_start)

        tw = t[mask]

        PB = np.array(self.hs.PB_history)[mask]
        OT = np.array(self.hs.OT_history)[mask]
        AVP = np.array(self.hs.AVP_history)[mask]
        DA = np.array(self.hs.DA_history)[mask]

        si_user = np.array(self.hs.si_user_history)[mask]
        si_hit = np.array(self.hs.si_hit_history)[mask]
        si_care = np.array(self.hs.si_care_history)[mask]
        si_corr = np.array(self.hs.si_correct_history)[mask]
        si_wrong = np.array(self.hs.si_wrong_history)[mask]

        # top panels
        self.line_PB.set_data(tw, PB)

        self.line_OT.set_data(tw, OT)
        self.line_AVP.set_data(tw, AVP)
        self.line_DA.set_data(tw, DA)

        # bottom: continuous signals
        self.line_user.set_data(tw, si_user)
        self.line_care.set_data(tw, si_care)

        # bottom: spikes for impulses
        self._clear_spikes()
        self._draw_spikes(tw, si_hit,  "C1")  # hits
        self._draw_spikes(tw, si_corr, "C3")  # correct
        self._draw_spikes(tw, si_wrong,"C4")  # wrong

        # x-limits
        self.ax_si.set_xlim(t_start, t_end + 2)

        return (self.line_PB, self.line_OT, self.line_AVP, self.line_DA, self.line_user, self.line_care)

    def start(self):
        self.ani = FuncAnimation(
            self.fig,
            self.update_plot,
            init_func=self.init_plot,
            interval=50,
            blit=False,
            cache_frame_data=False
        )
        plt.show()

# SIM LOOP
def simulation_loop(hs: HormoneSystem, kb: KeyboardInput, dt_s=0.5):
    while kb.running:
        # duration signals
        hs.set_duration("USER_PRESENCE", 100.0 if kb.user_present else 0.0)

        # caress with latch (so quick tap still appears)
        caress_on = kb.cares_active or kb.cares_latch_steps > 0
        hs.set_duration("CARESS", kb.cares_intensity if caress_on else 0.0)
        if kb.cares_latch_steps > 0:
            kb.cares_latch_steps -= 1

        # impulses (queue)
        while kb.pending_impulses:
            stim, inten = kb.pending_impulses.popleft()
            hs.trigger_impulse(stim, inten)

        # update
        hs.update(dt_s=dt_s)
        hs.update_PB(dt_s=dt_s)

        # record BEFORE resetting impulses
        hs.record()

        # reset impulses AFTER recording (so they appear for 1 sample)
        for k in ["HIT", "CORRECT", "WRONG"]:
            hs.si[k] = 0.0

        time.sleep(dt_s)


def main():
    print("=" * 70)
    print("Controls:")
    print("  [U] toggle presence (continuous)")
    print("  hold [C] caress (continuous while held, plus 1-step latch)")
    print("  [H] hit spike | [A] correct spike | [W] wrong spike")
    print("  [Q] quit")
    print("=" * 70)

    hs = HormoneSystem(PB=50, history_size=2400)
    kb = KeyboardInput()
    kb.start()

    th = threading.Thread(target=simulation_loop, args=(hs, kb, 0.5), daemon=True)
    th.start()

    plotter = Fig4RealtimePlotter(hs, window_s=60)
    plotter.start()

    kb.running = False
    print("[END]")


if __name__ == "__main__":
    main()
