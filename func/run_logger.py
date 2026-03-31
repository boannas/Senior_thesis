"""
Run logger: log each simulation tick to CSV and/or update realtime plots.
Mode: "csv" (only CSV), "plot" (only realtime plots), "both".
"""

import csv
import os
from collections import OrderedDict

try:
    import matplotlib.pyplot as plt
    HAS_MATPLOTLIB = True
except ImportError:
    HAS_MATPLOTLIB = False


# ─── Helpers ────────────────────────────────────────────────────────────

def _flatten_keys(nested_dict, prefix=""):
    """Flatten nested dict to key list: prefix_category_key."""
    keys = []
    for category, inner in nested_dict.items():
        for key in inner:
            keys.append(f"{prefix}{category}_{key}")
    return keys


def _flatten_values(nested_dict):
    """Flatten nested dict to value list (same order as _flatten_keys)."""
    values = []
    for category, inner in nested_dict.items():
        for key in inner:
            values.append(nested_dict[category][key])
    return values


def build_row(world):
    """Build one row dict: tick + mother variables + child variables + weights."""
    row = OrderedDict()
    row["tick"] = world.tick

    for i, mother in enumerate(world.mothers):
        prefix = f"m{i}_"

        # Physiological
        row[prefix + "energy"] = mother.energy
        row[prefix + "fatigue"] = mother.fatigue

        # Psychological
        row[prefix + "bonding"] = mother.bonding
        row[prefix + "fear_threat"] = mother.fear_threat
        row[prefix + "stress"] = mother.stress
        row[prefix + "closeness_child"] = mother.closeness_child

        # Hormones
        row[prefix + "oxytocin"] = mother.oxytocin
        row[prefix + "cortisol"] = mother.cortisol

        # Motivation values
        row[prefix + "mot_Forage"] = mother.motivations["Forage"]
        row[prefix + "mot_Care"] = mother.motivations["Care"]
        row[prefix + "mot_Self"] = mother.motivations["Self"]
        row[prefix + "mot_Protect"] = mother.motivations["Protect"]

        # Selected motivation (one-hot)
        selected = max(mother.motivations, key=mother.motivations.get)
        for mot in ("Forage", "Care", "Self", "Protect"):
            row[prefix + "sel_" + mot] = 1 if selected == mot else 0

        # Motivation weights (u): fixed and plastic
        if hasattr(mother, "motivation_weights_fixed"):
            for key, value in zip(
                _flatten_keys(mother.motivation_weights_fixed, prefix + "u_fixed_"),
                _flatten_values(mother.motivation_weights_fixed),
            ):
                row[key] = value
            for key, value in zip(
                _flatten_keys(mother.motivation_weights_plastic, prefix + "u_plastic_"),
                _flatten_values(mother.motivation_weights_plastic),
            ):
                row[key] = value

        # Psych weights (w): fixed and plastic
        if hasattr(mother, "psych_weights_fixed"):
            for key, value in zip(
                _flatten_keys(mother.psych_weights_fixed, prefix + "w_fixed_"),
                _flatten_values(mother.psych_weights_fixed),
            ):
                row[key] = value
            for key, value in zip(
                _flatten_keys(mother.psych_weights_plastic, prefix + "w_plastic_"),
                _flatten_values(mother.psych_weights_plastic),
            ):
                row[key] = value

    for i, child in enumerate(world.children):
        prefix = f"c{i}_"
        row[prefix + "hunger"] = child.hunger
        row[prefix + "warmth"] = child.warmth
        row[prefix + "injury"] = child.injury

    return row


# ═══════════════════════════════════════════════════════════════════════
# RunLogger
# ═══════════════════════════════════════════════════════════════════════

class RunLogger:
    """
    Log simulation run to CSV and/or show realtime matplotlib plots.

    Parameters
    ----------
    world : World
        The simulation world instance.
    mode : str
        "csv", "plot", or "both".
    csv_path : str
        Path for the CSV output file.
    plot_interval : int
        Update plots every N ticks.
    """

    def __init__(self, world, mode="csv", csv_path="run_log.csv", plot_interval=1):
        self.world = world
        self.mode = mode.lower()
        self.csv_path = csv_path
        self.plot_interval = max(1, int(plot_interval))

        self._csv_file = None
        self._csv_writer = None
        self._header_written = False
        self._plot_lines = {}
        self._figures = []

        if self.mode in ("csv", "both"):
            dirname = os.path.dirname(csv_path)
            if dirname and not os.path.isdir(dirname):
                os.makedirs(dirname, exist_ok=True)
            self._csv_file = open(csv_path, "w", newline="", encoding="utf-8")

        if self.mode in ("plot", "both") and HAS_MATPLOTLIB:
            self._init_plots()

    def _init_plots(self):
        """Create matplotlib figures for realtime monitoring."""
        plt.ion()

        # Figure 1: mother + child states
        fig1, ax1 = plt.subplots(4, 3, figsize=(10, 8), sharex=True)
        ax1 = ax1.flatten()

        mother_states = ["energy", "fatigue", "bonding", "fear_threat", "stress", "closeness_child", "oxytocin", "cortisol"]
        child_states = ["hunger", "warmth", "injury"]
        all_states = mother_states + child_states

        for ax, name in zip(ax1, all_states):
            ax.set_title(name)
            ax.set_ylabel("Value")
            ax.set_ylim(0, 100)
            ax.grid(True, alpha=0.3)
            line, = ax.plot([], [], "b-")
            self._plot_lines[name] = (ax, line)

        ax1[-1].set_xlabel("Tick")
        fig1.suptitle("Mother & Child States")
        fig1.tight_layout()
        self._figures.append(fig1)

        # Figure 2: motivation values + selected
        fig2, ax2 = plt.subplots(2, 1, figsize=(8, 4), sharex=True)
        ax2[0].set_title("Motivation Values")
        ax2[0].set_ylabel("Value")
        ax2[0].set_ylim(0, 100)
        ax2[0].grid(True, alpha=0.3)

        for mot, color in [("Forage", "gold"), ("Care", "limegreen"), ("Self", "deepskyblue"), ("Protect", "red")]:
            line, = ax2[0].plot([], [], label=mot, color=color)
            self._plot_lines["mot_" + mot] = (ax2[0], line)
        ax2[0].legend(loc="upper right", ncol=4)

        ax2[1].set_title("Selected Motivation (1 = active)")
        ax2[1].set_ylabel("Active")
        ax2[1].set_ylim(-0.05, 1.05)
        ax2[1].set_xlabel("Tick")
        ax2[1].grid(True, alpha=0.3)

        for mot, color in [("Forage", "gold"), ("Care", "limegreen"), ("Self", "deepskyblue"), ("Protect", "red")]:
            line, = ax2[1].plot([], [], label=mot, color=color)
            self._plot_lines["sel_" + mot] = (ax2[1], line)
        ax2[1].legend(loc="upper right", ncol=4)

        fig2.tight_layout()
        self._figures.append(fig2)

        # Figure 3: motivation weights (u) fixed vs plastic
        if self.world.mothers and hasattr(self.world.mothers[0], "motivation_weights_fixed"):
            mother = self.world.mothers[0]
            fig3, ax3 = plt.subplots(2, 2, figsize=(9, 5), sharex=True)
            ax3 = ax3.flatten()

            for idx, mot in enumerate(["forage", "care", "self", "protect"]):
                if mot not in mother.motivation_weights_fixed:
                    continue
                ax3[idx].set_title(f"u — {mot}")
                ax3[idx].set_ylabel("Weight")
                ax3[idx].set_ylim(-0.05, 2.1)
                ax3[idx].grid(True, alpha=0.3)

                for key in mother.motivation_weights_fixed[mot]:
                    line_fixed, = ax3[idx].plot([], [], "--", alpha=0.8, label=f"{key} fixed")
                    line_plastic, = ax3[idx].plot([], [], "-", alpha=0.7, label=f"{key} plastic")
                    self._plot_lines[f"u_{mot}_{key}_fixed"] = (ax3[idx], line_fixed)
                    self._plot_lines[f"u_{mot}_{key}_plastic"] = (ax3[idx], line_plastic)
                ax3[idx].legend(fontsize=6)

            ax3[0].set_xlabel("Tick")
            fig3.suptitle("Motivation weights u: fixed (dashed) vs plastic (solid)")
            fig3.tight_layout()
            self._figures.append(fig3)

    def _write_header_and_row(self, row):
        """Write CSV header (first call only) and data row."""
        if not self._csv_file:
            return
        if not self._header_written:
            self._csv_writer = csv.DictWriter(self._csv_file, fieldnames=list(row.keys()))
            self._csv_writer.writeheader()
            self._header_written = True
        self._csv_writer.writerow(row)

    def update(self):
        """Log current tick: write to CSV and/or update plots."""
        row = build_row(self.world)

        if self.mode in ("csv", "both"):
            self._write_header_and_row(row)

        if self.mode in ("plot", "both") and HAS_MATPLOTLIB and self.world.tick % self.plot_interval == 0:
            self._update_plots(row)

    def _update_plots(self, row):
        """Refresh all plot lines with latest history data."""
        tick = row["tick"]
        ticks = list(self.world.tick_history)
        if not ticks:
            return

        # Mother states
        for name in ["energy", "fatigue", "bonding", "fear_threat", "stress", "closeness_child", "oxytocin", "cortisol"]:
            if f"m0_{name}" not in row:
                continue
            ax, line = self._plot_lines.get(name, (None, None))
            if ax is None:
                continue
            mother_id = self.world.mothers[0].id if self.world.mothers else None
            history = self.world.mother_history.get(mother_id)
            if history and name in history:
                values = list(history[name])
                line.set_xdata(ticks[-len(values):])
                line.set_ydata(values)
            ax.set_xlim(max(0, ticks[0]), max(10, ticks[-1]))

        # Child states
        for name in ["hunger", "warmth", "injury"]:
            if f"c0_{name}" not in row:
                continue
            ax, line = self._plot_lines.get(name, (None, None))
            if ax is None:
                continue
            if self.world.children and self.world.child_history:
                child_id = self.world.children[0].id
                history = self.world.child_history.get(child_id, {})
                if name in history:
                    values = list(history[name])
                    line.set_xdata(ticks[-len(values):])
                    line.set_ydata(values)
            ax.set_xlim(max(0, ticks[0]), max(10, ticks[-1]))

        # Motivations
        for mot in ("Forage", "Care", "Self", "Protect"):
            for prefix in ("mot_", "sel_"):
                key = prefix + mot
                ax, line = self._plot_lines.get(key, (None, None))
                if ax is None:
                    continue
                mother_id = self.world.mothers[0].id if self.world.mothers else None
                history = self.world.mother_history.get(mother_id)
                if not history:
                    continue
                history_key = f"mot_{mot.lower()}" if prefix == "mot_" else f"sel_{mot.lower()}"
                if history_key in history:
                    values = list(history[history_key])
                    line.set_xdata(ticks[-len(values):])
                    line.set_ydata(values)
                ax.set_xlim(max(0, ticks[0]), max(10, ticks[-1]))

        # Motivation weights (u)
        if self.world.mothers and hasattr(self.world.mothers[0], "motivation_weights_fixed"):
            mother = self.world.mothers[0]
            history = self.world.mother_history.get(mother.id)
            if history:
                for mot in ["forage", "care", "self", "protect"]:
                    if mot not in mother.motivation_weights_fixed:
                        continue
                    for key in mother.motivation_weights_fixed[mot]:
                        key_fixed = f"u_fixed_{mot}_{key}"
                        key_plastic = f"u_plastic_{mot}_{key}"
                        if key_fixed not in history or key_plastic not in history:
                            continue
                        y_fixed = list(history[key_fixed])
                        y_plastic = list(history[key_plastic])
                        if not y_fixed or not y_plastic:
                            continue
                        x_plot = ticks[-len(y_fixed):]
                        y_constant = [y_fixed[0]] * len(x_plot)

                        line_fixed = self._plot_lines.get(f"u_{mot}_{key}_fixed", (None, None))[1]
                        line_plastic = self._plot_lines.get(f"u_{mot}_{key}_plastic", (None, None))[1]
                        if line_fixed is not None:
                            line_fixed.set_xdata(x_plot)
                            line_fixed.set_ydata(y_constant)
                        if line_plastic is not None:
                            line_plastic.set_xdata(ticks[-len(y_plastic):])
                            line_plastic.set_ydata(y_plastic)

        for fig in self._figures:
            fig.canvas.draw()
            fig.canvas.flush_events()
        plt.pause(0.001)

    def close(self):
        """Close CSV file and matplotlib figures."""
        if self._csv_file:
            try:
                self._csv_file.close()
            except Exception:
                pass
            self._csv_file = None

        if HAS_MATPLOTLIB:
            for fig in self._figures:
                try:
                    plt.close(fig)
                except Exception:
                    pass
            self._figures.clear()
