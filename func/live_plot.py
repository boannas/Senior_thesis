"""
Real-time matplotlib plotters for monitoring simulation state.

Plotters:
- MotherStatePlotter       : energy, fatigue, bonding, fear, stress, closeness, oxytocin, cortisol
- MotherMotivationPlotter  : motivation values + selected motivation
- ChildStatePlotter        : hunger, warmth, injury
- FixedVsPlasticPlotter    : fixed vs plastic weights (u and w)
"""

import matplotlib.pyplot as plt


# ═══════════════════════════════════════════════════════════════════════
# Mother State Plotter
# ═══════════════════════════════════════════════════════════════════════

class MotherStatePlotter:
    """Real-time plot of mother physiological, psychological, and hormonal states."""

    STATE_NAMES = [
        "energy", "fatigue", "bonding", "fear_threat",
        "stress", "closeness_child", "oxytocin", "cortisol",
    ]

    def __init__(self, world):
        self.world = world
        plt.ion()

        self.fig, self.axes = plt.subplots(4, 2, figsize=(7, 7))
        self.axes = self.axes.flatten()

        manager = plt.get_current_fig_manager()
        manager.window.wm_geometry("+0+0")

        self.lines = {}
        for ax, state_name in zip(self.axes, self.STATE_NAMES):
            ax.set_title(state_name)
            ax.set_xlabel("Tick")
            ax.set_ylabel("Value")
            ax.set_ylim(0, 100)
            ax.grid(True)

        for mother in self.world.mothers:
            for ax, state_name in zip(self.axes, self.STATE_NAMES):
                line, = ax.plot([], [], label=mother.id)
                self.lines[(mother.id, state_name)] = line

        for ax in self.axes:
            ax.legend(loc="upper right")
        self.fig.tight_layout()

    def update(self):
        """Update all state plots with latest data."""
        ticks = list(self.world.tick_history)
        if not ticks:
            return

        for mother in self.world.mothers:
            history = self.world.mother_history[mother.id]
            for ax, state_name in zip(self.axes, self.STATE_NAMES):
                values = list(history[state_name])
                line = self.lines[(mother.id, state_name)]
                line.set_xdata(ticks)
                line.set_ydata(values)
                ax.set_xlim(max(0, ticks[0]), max(10, ticks[-1]))

        self.fig.canvas.draw()
        self.fig.canvas.flush_events()
        plt.pause(0.001)


# ═══════════════════════════════════════════════════════════════════════
# Mother Motivation Plotter
# ═══════════════════════════════════════════════════════════════════════

class MotherMotivationPlotter:
    """Real-time plot of motivation values and which motivation is selected."""

    MOTIVATION_KEYS = ["Forage", "Care", "Self", "Protect"]
    MOTIVATION_COLORS = {
        "Forage": "gold",
        "Care": "limegreen",
        "Self": "deepskyblue",
        "Protect": "red",
    }

    def __init__(self, world):
        self.world = world
        plt.ion()

        self.fig, self.axes = plt.subplots(2, 1, figsize=(8, 4), sharex=True)
        self.ax_values = self.axes[0]
        self.ax_selected = self.axes[1]

        manager = plt.get_current_fig_manager()
        manager.window.wm_geometry("+880+0")

        self.ax_values.set_title("Motivation Values")
        self.ax_values.set_ylabel("Value")
        self.ax_values.set_ylim(0, 100)
        self.ax_values.grid(True)

        self.ax_selected.set_title("Selected Motivation (1 = active)")
        self.ax_selected.set_xlabel("Tick")
        self.ax_selected.set_ylabel("Active")
        self.ax_selected.set_ylim(-0.05, 1.05)
        self.ax_selected.grid(True)

        self.lines_values = {}
        self.lines_selected = {}

        for mother in self.world.mothers:
            for mot in self.MOTIVATION_KEYS:
                color = self.MOTIVATION_COLORS[mot]
                line_val, = self.ax_values.plot([], [], label=mot, color=color)
                self.lines_values[(mother.id, mot)] = line_val

                line_sel, = self.ax_selected.plot([], [], label=mot, color=color)
                self.lines_selected[(mother.id, mot)] = line_sel

        self.ax_values.legend(loc="upper right", ncol=4)
        self.ax_selected.legend(loc="upper right", ncol=4)
        self.fig.tight_layout()

    def update(self, mother_id=None):
        """Update motivation plots with latest data."""
        ticks = list(self.world.tick_history)
        if not ticks:
            return

        if mother_id is None:
            if not self.world.mothers:
                return
            mother_id = self.world.mothers[0].id

        history = self.world.mother_history.get(mother_id)
        if history is None:
            return

        for mot in self.MOTIVATION_KEYS:
            values = list(history[f"mot_{mot.lower()}"])
            self.lines_values[(mother_id, mot)].set_xdata(ticks)
            self.lines_values[(mother_id, mot)].set_ydata(values)

            selected = list(history[f"sel_{mot.lower()}"])
            self.lines_selected[(mother_id, mot)].set_xdata(ticks)
            self.lines_selected[(mother_id, mot)].set_ydata(selected)

        self.ax_values.set_xlim(max(0, ticks[0]), max(10, ticks[-1]))
        self.ax_selected.set_xlim(max(0, ticks[0]), max(10, ticks[-1]))

        self.fig.canvas.draw()
        self.fig.canvas.flush_events()
        plt.pause(0.001)


# ═══════════════════════════════════════════════════════════════════════
# Child State Plotter
# ═══════════════════════════════════════════════════════════════════════

class ChildStatePlotter:
    """Real-time plot of child hunger, warmth, and injury states."""

    STATE_NAMES = ["hunger", "warmth", "injury"]

    def __init__(self, world):
        self.world = world
        plt.ion()

        self.fig, self.axes = plt.subplots(1, 3, figsize=(8, 3))
        self.axes = self.axes.flatten()

        manager = plt.get_current_fig_manager()
        manager.window.wm_geometry("+880+0")

        self.lines = {}
        for ax, state_name in zip(self.axes, self.STATE_NAMES):
            ax.set_title(state_name)
            ax.set_xlabel("Tick")
            ax.set_ylabel("Value")
            ax.set_ylim(0, 100)
            ax.grid(True)

        for child in self.world.children:
            for ax, state_name in zip(self.axes, self.STATE_NAMES):
                line, = ax.plot([], [], label=child.id)
                self.lines[(child.id, state_name)] = line

        for ax in self.axes:
            ax.legend()
        self.fig.tight_layout()

    def update(self):
        """Update child state plots with latest data."""
        ticks = list(self.world.tick_history)
        if not ticks:
            return

        for child in self.world.children:
            history = self.world.child_history[child.id]
            for ax, state_name in zip(self.axes, self.STATE_NAMES):
                values = list(history[state_name])
                x_data = ticks[-len(values):] if values else []
                line = self.lines[(child.id, state_name)]
                line.set_xdata(x_data)
                line.set_ydata(values)
                ax.set_xlim(max(0, ticks[0]), max(10, ticks[-1]))

        self.fig.canvas.draw()
        self.fig.canvas.flush_events()
        plt.pause(0.001)


# ═══════════════════════════════════════════════════════════════════════
# Fixed vs Plastic Weight Plotter
# ═══════════════════════════════════════════════════════════════════════

class FixedVsPlasticPlotter:
    """
    Compare fixed (constant) vs plastic (learned) weights over time.
    Fixed = inherited gene (dashed); Plastic = learned overlay (solid).
    """

    def __init__(self, world, mother_id=None):
        self.world = world
        self.mother_id = mother_id or (world.mothers[0].id if world.mothers else None)
        plt.ion()
        self._build_motivation_weight_figure()
        self._build_psych_weight_figure()
        self.fig_motivation.canvas.draw()
        self.fig_psych.canvas.draw()

    def _build_motivation_weight_figure(self):
        """Build the motivation weights (u) figure: fixed vs plastic."""
        self.fig_motivation, self.axes_motivation = plt.subplots(2, 2, figsize=(10, 6), sharex=True)
        self.axes_motivation = self.axes_motivation.flatten()

        motivation_order = ["forage", "care", "self", "protect"]
        self.motivation_weight_keys = {
            "forage":  ["child_hunger", "energy_deficit", "low_fear"],
            "care":    ["child_warmth", "closeness_deficit", "bonding"],
            "self":    ["fatigue", "fear", "stress"],
            "protect": ["child_injury", "fear", "closeness_deficit", "bonding"],
        }

        self.lines_motivation = {}
        colors = plt.cm.tab10.colors

        for i, mot in enumerate(motivation_order):
            ax = self.axes_motivation[i]
            ax.set_title(f"u — {mot.capitalize()}")
            ax.set_xlabel("Tick")
            ax.set_ylabel("Weight")
            ax.set_ylim(-0.05, 2.1)
            ax.grid(True, alpha=0.3)

            for j, key in enumerate(self.motivation_weight_keys[mot]):
                color = colors[(i * 4 + j) % len(colors)]
                line_fixed, = ax.plot([], [], linestyle="--", color=color, alpha=0.9, label=f"{key} (fixed)")
                line_plastic, = ax.plot([], [], linestyle="-", color=color, alpha=0.7, label=f"{key} (plastic)")
                self.lines_motivation[(mot, key, "fixed")] = line_fixed
                self.lines_motivation[(mot, key, "plastic")] = line_plastic

            ax.legend(loc="upper right", fontsize=7)

        self.fig_motivation.suptitle("Motivation weights: fixed (dashed) vs plastic (solid)")
        self.fig_motivation.tight_layout()
        try:
            manager = plt.get_current_fig_manager()
            manager.window.wm_geometry("+0+400")
        except Exception:
            pass

    def _build_psych_weight_figure(self):
        """Build the psych weights (w) figure: fixed vs plastic."""
        self.fig_psych, self.axes_psych = plt.subplots(2, 3, figsize=(11, 6), sharex=True)
        self.axes_psych = self.axes_psych.flatten()

        psych_categories = ["child_need", "oxytocin", "bonding", "cortisol", "stress", "fear"]
        self.psych_weight_keys = {}

        if self.world.mothers and hasattr(self.world.mothers[0], "psych_weights_fixed"):
            for category in psych_categories:
                if category in self.world.mothers[0].psych_weights_fixed:
                    self.psych_weight_keys[category] = list(
                        self.world.mothers[0].psych_weights_fixed[category].keys()
                    )
        else:
            self.psych_weight_keys = {
                "child_need": ["hunger", "warmth", "injury"],
                "oxytocin":   ["closeness_gain", "decay"],
                "bonding":    ["oxytocin_gain", "child_need_decay", "child_absent_decay"],
                "cortisol":   ["threat_gain", "child_need_gain", "energy_deficit_gain", "decay"],
                "stress":     ["cortisol_gain", "fear_gain", "child_need_gain", "decay"],
                "fear":       ["threat_gain", "decay"],
            }

        self.lines_psych = {}
        colors = plt.cm.tab10.colors

        for i, category in enumerate(psych_categories):
            ax = self.axes_psych[i]
            ax.set_title(f"w — {category}")
            ax.set_xlabel("Tick")
            ax.set_ylabel("Weight")
            ax.set_ylim(-0.05, 2.1)
            ax.grid(True, alpha=0.3)

            keys = self.psych_weight_keys.get(category, [])
            for j, key in enumerate(keys):
                color = colors[j % len(colors)]
                line_fixed, = ax.plot([], [], linestyle="--", color=color, alpha=0.9, label=f"{key} (fixed)")
                line_plastic, = ax.plot([], [], linestyle="-", color=color, alpha=0.7, label=f"{key} (plastic)")
                self.lines_psych[(category, key, "fixed")] = line_fixed
                self.lines_psych[(category, key, "plastic")] = line_plastic
            ax.legend(loc="upper right", fontsize=6)

        self.fig_psych.suptitle("Psych weights: fixed (dashed) vs plastic (solid)")
        self.fig_psych.tight_layout()
        try:
            manager = plt.get_current_fig_manager()
            manager.window.wm_geometry("+520+400")
        except Exception:
            pass

    def update(self, mother_id=None):
        """Update both weight figures with latest data."""
        mid = mother_id or self.mother_id
        if not mid:
            return
        ticks = list(self.world.tick_history)
        if not ticks:
            return
        history = self.world.mother_history.get(mid)
        if history is None:
            return

        num_ticks = len(ticks)

        # --- Update motivation weights (u) ---
        for i, mot in enumerate(["forage", "care", "self", "protect"]):
            for key in self.motivation_weight_keys.get(mot, []):
                key_fixed = f"u_fixed_{mot}_{key}"
                key_plastic = f"u_plastic_{mot}_{key}"
                if key_fixed not in history or key_plastic not in history:
                    continue

                y_fixed_data = list(history[key_fixed])
                y_plastic_data = list(history[key_plastic])
                if not y_fixed_data or not y_plastic_data:
                    continue

                x_plot = ticks[-len(y_fixed_data):] if len(y_fixed_data) < num_ticks else ticks
                y_constant = [y_fixed_data[0]] * len(x_plot)

                self.lines_motivation[(mot, key, "fixed")].set_xdata(x_plot)
                self.lines_motivation[(mot, key, "fixed")].set_ydata(y_constant)
                self.lines_motivation[(mot, key, "plastic")].set_xdata(ticks[-len(y_plastic_data):])
                self.lines_motivation[(mot, key, "plastic")].set_ydata(y_plastic_data)

            self.axes_motivation[i].set_xlim(max(0, ticks[0]), max(10, ticks[-1]))

        # --- Update psych weights (w) ---
        for i, category in enumerate(["child_need", "oxytocin", "bonding", "cortisol", "stress", "fear"]):
            for key in self.psych_weight_keys.get(category, []):
                key_fixed = f"w_fixed_{category}_{key}"
                key_plastic = f"w_plastic_{category}_{key}"
                if key_fixed not in history or key_plastic not in history:
                    continue

                y_fixed_data = list(history[key_fixed])
                y_plastic_data = list(history[key_plastic])
                if not y_fixed_data or not y_plastic_data:
                    continue

                x_plot = ticks[-len(y_fixed_data):] if len(y_fixed_data) < num_ticks else ticks
                y_constant = [y_fixed_data[0]] * len(x_plot)

                self.lines_psych[(category, key, "fixed")].set_xdata(x_plot)
                self.lines_psych[(category, key, "fixed")].set_ydata(y_constant)
                self.lines_psych[(category, key, "plastic")].set_xdata(ticks[-len(y_plastic_data):])
                self.lines_psych[(category, key, "plastic")].set_ydata(y_plastic_data)

            self.axes_psych[i].set_xlim(max(0, ticks[0]), max(10, ticks[-1]))

        # --- Redraw ---
        self.fig_motivation.canvas.draw()
        self.fig_psych.canvas.draw()
        self.fig_motivation.canvas.flush_events()
        self.fig_psych.canvas.flush_events()
        plt.pause(0.001)
