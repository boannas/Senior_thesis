
import matplotlib.pyplot as plt

class MotherStatePlotter:
    def __init__(self, world):
        self.world = world

        self.state_names = [
            "energy",
            "fatigue",
            "bonding",
            "fear_threat",
            "stress",
            "closeness_child",
            "OT",
            "CORT",
        ]

        plt.ion()


        self.fig, self.axes = plt.subplots(4, 2, figsize=(7, 7 ))
        self.axes = self.axes.flatten()

        self.lines = {}  # (mother_id, state_name) -> line
        manager = plt.get_current_fig_manager()
        manager.window.wm_geometry("+0+0")

        for ax, state_name in zip(self.axes, self.state_names):
            ax.set_title(state_name)
            ax.set_xlabel("Tick")
            ax.set_ylabel("Value")
            ax.set_ylim(0, 100)
            ax.grid(True)

        for m in self.world.mothers:
            for ax, state_name in zip(self.axes, self.state_names):
                line, = ax.plot([], [], label=m.id)
                self.lines[(m.id, state_name)] = line

        for ax in self.axes:
            ax.legend(loc="upper right")

        self.fig.tight_layout()

    def update(self):
        x = list(self.world.tick_history)
        if not x:
            return

        for m in self.world.mothers:
            hist = self.world.mother_history[m.id]

            for ax, state_name in zip(self.axes, self.state_names):
                y = list(hist[state_name])
                line = self.lines[(m.id, state_name)]
                line.set_xdata(x)
                line.set_ydata(y)
                ax.set_xlim(max(0, x[0]), max(10, x[-1]))

        self.fig.canvas.draw()
        self.fig.canvas.flush_events()
        plt.pause(0.001)


import matplotlib.pyplot as plt

class MotherMotivationPlotter:
    def __init__(self, world):
        self.world = world

        plt.ion()

        self.fig, self.axes = plt.subplots(2, 1, figsize=(8, 4), sharex=True)
        self.ax_values = self.axes[0]
        self.ax_active = self.axes[1]

        manager = plt.get_current_fig_manager()
        manager.window.wm_geometry("+880+0")

        self.ax_values.set_title("Motivation values")
        self.ax_values.set_ylabel("Value")
        self.ax_values.set_ylim(0, 100)
        self.ax_values.grid(True)

        self.ax_active.set_title("Motivation activation")
        self.ax_active.set_xlabel("Tick")
        self.ax_active.set_ylabel("Activation")
        self.ax_active.set_ylim(-0.05, 1.05)
        self.ax_active.grid(True)

        self.mot_keys = ["Forage", "Care", "Self", "Protect"]

        # choose your fixed colors here
        self.colors = {
            "Forage": "gold",
            "Care": "limegreen",
            "Self": "deepskyblue",
            "Protect": "red",
        }

        self.lines_values = {}   # (mother_id, motivation_name) -> line
        self.lines_active = {}   # (mother_id, motivation_name) -> line

        for m in self.world.mothers:
            for mot in self.mot_keys:
                line_val, = self.ax_values.plot(
                    [], [], label=mot, color=self.colors[mot]
                )
                self.lines_values[(m.id, mot)] = line_val

                line_act, = self.ax_active.plot(
                    [], [], label=mot, color=self.colors[mot]
                )
                self.lines_active[(m.id, mot)] = line_act

            # if multiple mothers, only first mother's labels should appear in legend
            # break

        self.ax_values.legend(loc="upper right", ncol=4)
        self.ax_active.legend(loc="upper right", ncol=4)

        self.fig.tight_layout()

    # def update(self, mother_id=None):
    #     x = list(self.world.tick_history)
    #     if not x:
    #         return

    #     # default: plot first mother
    #     if mother_id is None:
    #         if not self.world.mothers:
    #             return
    #         mother_id = self.world.mothers[0].id

    #     hist = self.world.mother_history[mother_id]

    #     for mot in self.mot_keys:
    #         y_val = list(hist[f"mot_{mot.lower()}"])
    #         self.lines_values[(mother_id, mot)].set_xdata(x)
    #         self.lines_values[(mother_id, mot)].set_ydata(y_val)

    #         y_act = list(hist[f"sel_{mot.lower()}"])
    #         self.lines_active[(mother_id, mot)].set_xdata(x)
    #         self.lines_active[(mother_id, mot)].set_ydata(y_act)

    #     self.ax_values.set_xlim(max(0, x[0]), max(10, x[-1]))
    #     self.ax_active.set_xlim(max(0, x[0]), max(10, x[-1]))

    #     self.fig.canvas.draw()
    #     self.fig.canvas.flush_events()
    #     plt.pause(0.001)

    def update(self, mother_id=None):

        x = list(self.world.tick_history)
        if not x:
            return

        # if no mother specified, use first alive mother
        if mother_id is None:
            if not self.world.mothers:
                return
            mother_id = self.world.mothers[0].id

        hist = self.world.mother_history.get(mother_id)
        if hist is None:
            return

        for mot in self.mot_keys:

            y_val = list(hist[f"mot_{mot.lower()}"])
            self.lines_values[(mother_id, mot)].set_xdata(x)
            self.lines_values[(mother_id, mot)].set_ydata(y_val)

            y_act = list(hist[f"sel_{mot.lower()}"])
            self.lines_active[(mother_id, mot)].set_xdata(x)
            self.lines_active[(mother_id, mot)].set_ydata(y_act)

        self.ax_values.set_xlim(max(0, x[0]), max(10, x[-1]))
        self.ax_active.set_xlim(max(0, x[0]), max(10, x[-1]))

        self.fig.canvas.draw()
        self.fig.canvas.flush_events()
        plt.pause(0.001)


class FixedVsPlasticPlotter:
    """
    Compare fixed (constant line) vs plastic (over time) for u and w weights.
    Fixed = inherited gene (unchanged); plastic = learnable overlay.
    """
    def __init__(self, world, mother_id=None):
        self.world = world
        self.mother_id = mother_id or (world.mothers[0].id if world.mothers else None)
        plt.ion()
        self._build_u_figure()
        self._build_w_figure()
        self.fig_u.canvas.draw()
        self.fig_w.canvas.draw()

    def _build_u_figure(self):
        self.fig_u, self.axes_u = plt.subplots(2, 2, figsize=(10, 6), sharex=True)
        self.axes_u = self.axes_u.flatten()
        mot_order = ["forage", "care", "self", "protect"]
        self.u_mot_keys = {
            "forage": ["child_hunger", "energy_deficit", "low_fear"],
            "care": ["child_warmth", "closeness_deficit", "bonding"],
            "self": ["fatigue", "fear", "stress"],
            "protect": ["child_injury", "fear", "closeness_deficit", "bonding"],
        }
        for ax, mot in zip(self.axes_u, mot_order):
            ax.set_title(f"u — {mot.capitalize()}")
            ax.set_xlabel("Tick")
            ax.set_ylabel("Weight")
            ax.set_ylim(-0.05, 2.1)
            ax.grid(True, alpha=0.3)
        self.lines_u = {}
        colors = plt.cm.tab10.colors
        for i, mot in enumerate(mot_order):
            for j, key in enumerate(self.u_mot_keys[mot]):
                c = colors[(i * 4 + j) % len(colors)]
                ax = self.axes_u[i]
                l_fixed, = ax.plot([], [], linestyle="--", color=c, alpha=0.9, label=f"{key} (fixed)")
                l_plastic, = ax.plot([], [], linestyle="-", color=c, alpha=0.7, label=f"{key} (plastic)")
                self.lines_u[(mot, key, "fixed")] = l_fixed
                self.lines_u[(mot, key, "plastic")] = l_plastic
            self.axes_u[i].legend(loc="upper right", fontsize=7)
        self.fig_u.suptitle("Motivation weights u: fixed (dashed) vs plastic (solid)")
        self.fig_u.tight_layout()
        try:
            manager = plt.get_current_fig_manager()
            manager.window.wm_geometry("+0+400")
        except Exception:
            pass

    def _build_w_figure(self):
        self.fig_w, self.axes_w = plt.subplots(2, 3, figsize=(11, 6), sharex=True)
        self.axes_w = self.axes_w.flatten()
        w_cats = ["child_need", "ot", "bonding", "cort", "stress", "fear"]
        self.w_cat_keys = {}
        if self.world.mothers and hasattr(self.world.mothers[0], "w_fixed"):
            for cat in w_cats:
                if cat in self.world.mothers[0].w_fixed:
                    self.w_cat_keys[cat] = list(self.world.mothers[0].w_fixed[cat].keys())
        else:
            self.w_cat_keys = {
                "child_need": ["hunger", "warmth", "injury"],
                "ot": ["closeness_gain", "decay"],
                "bonding": ["ot_gain", "child_need_decay", "child_absent_decay"],
                "cort": ["threat_gain", "child_need_gain", "energy_deficit_gain", "decay"],
                "stress": ["cort_gain", "fear_gain", "child_need_gain", "decay"],
                "fear": ["threat_gain", "decay"],
            }
        self.lines_w = {}
        colors = plt.cm.tab10.colors
        for i, cat in enumerate(w_cats):
            ax = self.axes_w[i]
            ax.set_title(f"w — {cat}")
            ax.set_xlabel("Tick")
            ax.set_ylabel("Weight")
            ax.set_ylim(-0.05, 2.1)
            ax.grid(True, alpha=0.3)
            keys = self.w_cat_keys.get(cat, [])
            for j, key in enumerate(keys):
                c = colors[j % len(colors)]
                l_fixed, = ax.plot([], [], linestyle="--", color=c, alpha=0.9, label=f"{key} (fixed)")
                l_plastic, = ax.plot([], [], linestyle="-", color=c, alpha=0.7, label=f"{key} (plastic)")
                self.lines_w[(cat, key, "fixed")] = l_fixed
                self.lines_w[(cat, key, "plastic")] = l_plastic
            ax.legend(loc="upper right", fontsize=6)
        self.fig_w.suptitle("Psych weights w: fixed (dashed) vs plastic (solid)")
        self.fig_w.tight_layout()
        try:
            manager = plt.get_current_fig_manager()
            manager.window.wm_geometry("+520+400")
        except Exception:
            pass

    def update(self, mother_id=None):
        mid = mother_id or self.mother_id
        if not mid:
            return
        x = list(self.world.tick_history)
        if not x:
            return
        hist = self.world.mother_history.get(mid)
        if hist is None:
            return
        n = len(x)
        mot_order = ["forage", "care", "self", "protect"]
        for i, mot in enumerate(mot_order):
            for key in self.u_mot_keys.get(mot, []):
                k_f, k_p = f"u_fixed_{mot}_{key}", f"u_plastic_{mot}_{key}"
                if k_f not in hist or k_p not in hist:
                    continue
                y_f = list(hist[k_f])
                y_p = list(hist[k_p])
                if not y_f or not y_p:
                    continue
                x_plot = x[-len(y_f):] if len(y_f) < n else x
                y_fixed = [y_f[0]] * len(x_plot) if y_f else []
                self.lines_u[(mot, key, "fixed")].set_xdata(x_plot)
                self.lines_u[(mot, key, "fixed")].set_ydata(y_fixed)
                self.lines_u[(mot, key, "plastic")].set_xdata(x[-len(y_p):])
                self.lines_u[(mot, key, "plastic")].set_ydata(y_p)
            self.axes_u[i].set_xlim(max(0, x[0]), max(10, x[-1]))
        for i, cat in enumerate(["child_need", "ot", "bonding", "cort", "stress", "fear"]):
            for key in self.w_cat_keys.get(cat, []):
                k_f, k_p = f"w_fixed_{cat}_{key}", f"w_plastic_{cat}_{key}"
                if k_f not in hist or k_p not in hist:
                    continue
                y_f = list(hist[k_f])
                y_p = list(hist[k_p])
                if not y_f or not y_p:
                    continue
                x_plot = x[-len(y_f):] if len(y_f) < n else x
                y_fixed = [y_f[0]] * len(x_plot) if y_f else []
                self.lines_w[(cat, key, "fixed")].set_xdata(x_plot)
                self.lines_w[(cat, key, "fixed")].set_ydata(y_fixed)
                self.lines_w[(cat, key, "plastic")].set_xdata(x[-len(y_p):])
                self.lines_w[(cat, key, "plastic")].set_ydata(y_p)
            self.axes_w[i].set_xlim(max(0, x[0]), max(10, x[-1]))
        self.fig_u.canvas.draw()
        self.fig_w.canvas.draw()
        self.fig_u.canvas.flush_events()
        self.fig_w.canvas.flush_events()
        plt.pause(0.001)


class ChildStatePlotter:
    def __init__(self, world):
        self.world = world

        self.state_names = [
            "hunger",
            "warmth",
            "injury",
            # "carried",
        ]

        plt.ion()

        self.fig, self.axes = plt.subplots(1, 3, figsize=(8, 3))
        self.axes = self.axes.flatten()

        manager = plt.get_current_fig_manager()
        manager.window.wm_geometry("+880+0")

        self.lines = {}

        for ax, state_name in zip(self.axes, self.state_names):
            ax.set_title(state_name)
            ax.set_xlabel("Tick")
            ax.set_ylabel("Value")
            ax.grid(True)

            if state_name != "carried":
                ax.set_ylim(0, 100)
            else:
                ax.set_ylim(-0.1, 1.1)

        for c in self.world.children:
            for ax, state_name in zip(self.axes, self.state_names):
                line, = ax.plot([], [], label=c.id)
                self.lines[(c.id, state_name)] = line

        for ax in self.axes:
            ax.legend()

        self.fig.tight_layout()

    def update(self):
        x = list(self.world.tick_history)
        if not x:
            return

        for c in self.world.children:
            hist = self.world.child_history[c.id]

            for ax, state_name in zip(self.axes, self.state_names):
                y = list(hist[state_name])
                x_local = x[-len(y):] if len(y) > 0 else []
                line = self.lines[(c.id, state_name)]
                line.set_xdata(x_local)
                line.set_ydata(y)

                ax.set_xlim(max(0, x[0]), max(10, x[-1]))

        self.fig.canvas.draw()
        self.fig.canvas.flush_events()
        plt.pause(0.001)
