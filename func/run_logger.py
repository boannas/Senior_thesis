"""
Run logger: log each tick to CSV and/or update realtime plots.
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


def _flat_keys(d, prefix=""):
    """Flatten nested dict to key list: prefix_cat_key."""
    out = []
    for cat, inner in d.items():
        for k in inner:
            out.append(f"{prefix}{cat}_{k}")
    return out


def _flat_vals(d):
    """Flatten nested dict to value list in same order as _flat_keys."""
    out = []
    for cat, inner in d.items():
        for k in inner:
            out.append(d[cat][k])
    return out


def build_row(world):
    """Build one row dict: tick, mother vars, child vars, u/w fixed/plastic."""
    row = OrderedDict()
    row["tick"] = world.tick
    for i, m in enumerate(world.mothers):
        pre = f"m{i}_"
        row[pre + "energy"] = m.energy
        row[pre + "fatigue"] = m.fatigue
        row[pre + "bonding"] = m.bonding
        row[pre + "fear_threat"] = m.fear_threat
        row[pre + "stress"] = m.stress
        row[pre + "closeness_child"] = m.closeness_child
        row[pre + "OT"] = m.OT
        row[pre + "CORT"] = m.CORT
        row[pre + "mot_Forage"] = m.motivations["Forage"]
        row[pre + "mot_Care"] = m.motivations["Care"]
        row[pre + "mot_Self"] = m.motivations["Self"]
        row[pre + "mot_Protect"] = m.motivations["Protect"]
        sel = max(m.motivations, key=m.motivations.get)
        for mot in ("Forage", "Care", "Self", "Protect"):
            row[pre + "sel_" + mot] = 1 if sel == mot else 0
        if hasattr(m, "u_fixed"):
            for k, v in zip(_flat_keys(m.u_fixed, pre + "u_fixed_"), _flat_vals(m.u_fixed)):
                row[k] = v
            for k, v in zip(_flat_keys(m.u_plastic, pre + "u_plastic_"), _flat_vals(m.u_plastic)):
                row[k] = v
        if hasattr(m, "w_fixed"):
            for k, v in zip(_flat_keys(m.w_fixed, pre + "w_fixed_"), _flat_vals(m.w_fixed)):
                row[k] = v
            for k, v in zip(_flat_keys(m.w_plastic, pre + "w_plastic_"), _flat_vals(m.w_plastic)):
                row[k] = v
    for i, c in enumerate(world.children):
        pre = f"c{i}_"
        row[pre + "hunger"] = c.hunger
        row[pre + "warmth"] = c.warmth
        row[pre + "injury"] = c.injury
    return row


class RunLogger:
    """
    Log run to CSV and/or show realtime plots.
    mode: "csv" | "plot" | "both"
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
        self._figs = []
        self._axes = []

        if self.mode in ("csv", "both"):
            dirname = os.path.dirname(csv_path)
            if dirname and not os.path.isdir(dirname):
                os.makedirs(dirname, exist_ok=True)
            self._csv_file = open(csv_path, "w", newline="", encoding="utf-8")

        if self.mode in ("plot", "both") and HAS_MATPLOTLIB:
            self._init_plots()

    def _init_plots(self):
        plt.ion()
        # Figure 1: mother + child states
        fig1, ax1 = plt.subplots(4, 3, figsize=(10, 8), sharex=True)
        ax1 = ax1.flatten()
        mother_states = ["energy", "fatigue", "bonding", "fear_threat", "stress", "closeness_child", "OT", "CORT"]
        child_states = ["hunger", "warmth", "injury"]
        titles = mother_states + child_states[:3]
        for i, (ax, name) in enumerate(zip(ax1, titles)):
            ax.set_title(name)
            ax.set_ylabel("Value")
            ax.set_ylim(0, 100)
            ax.grid(True, alpha=0.3)
            line, = ax.plot([], [], "b-")
            self._plot_lines[name] = (ax, line)
        ax1[-1].set_xlabel("Tick")
        fig1.suptitle("Mother & child states")
        fig1.tight_layout()
        self._figs.append(fig1)

        # Figure 2: motivation values + selected
        fig2, ax2 = plt.subplots(2, 1, figsize=(8, 4), sharex=True)
        ax2[0].set_title("Motivation values")
        ax2[0].set_ylabel("Value")
        ax2[0].set_ylim(0, 100)
        ax2[0].grid(True, alpha=0.3)
        for mot, col in [("Forage", "gold"), ("Care", "limegreen"), ("Self", "deepskyblue"), ("Protect", "red")]:
            line, = ax2[0].plot([], [], label=mot, color=col)
            self._plot_lines["mot_" + mot] = (ax2[0], line)
        ax2[0].legend(loc="upper right", ncol=4)
        ax2[1].set_title("Selected motivation (1=active)")
        ax2[1].set_ylabel("Active")
        ax2[1].set_ylim(-0.05, 1.05)
        ax2[1].set_xlabel("Tick")
        ax2[1].grid(True, alpha=0.3)
        for mot, col in [("Forage", "gold"), ("Care", "limegreen"), ("Self", "deepskyblue"), ("Protect", "red")]:
            line, = ax2[1].plot([], [], label=mot, color=col)
            self._plot_lines["sel_" + mot] = (ax2[1], line)
        ax2[1].legend(loc="upper right", ncol=4)
        fig2.tight_layout()
        self._figs.append(fig2)

        # Figure 3: weights u fixed vs plastic (first mother, first motivation keys)
        if self.world.mothers and hasattr(self.world.mothers[0], "u_fixed"):
            m = self.world.mothers[0]
            fig3, ax3 = plt.subplots(2, 2, figsize=(9, 5), sharex=True)
            ax3 = ax3.flatten()
            for idx, mot in enumerate(["forage", "care", "self", "protect"]):
                if mot not in m.u_fixed:
                    continue
                ax3[idx].set_title(f"u — {mot}")
                ax3[idx].set_ylabel("Weight")
                ax3[idx].set_ylim(-0.05, 2.1)
                ax3[idx].grid(True, alpha=0.3)
                for key in m.u_fixed[mot]:
                    lf, = ax3[idx].plot([], [], "--", alpha=0.8, label=f"{key} fixed")
                    lp, = ax3[idx].plot([], [], "-", alpha=0.7, label=f"{key} plastic")
                    self._plot_lines[f"u_{mot}_{key}_fixed"] = (ax3[idx], lf)
                    self._plot_lines[f"u_{mot}_{key}_plastic"] = (ax3[idx], lp)
                ax3[idx].legend(fontsize=6)
            ax3[0].set_xlabel("Tick")
            fig3.suptitle("Weights u: fixed (dashed) vs plastic (solid)")
            fig3.tight_layout()
            self._figs.append(fig3)

    def _write_header_and_row(self, row):
        if not self._csv_file:
            return
        if not self._header_written:
            self._csv_writer = csv.DictWriter(self._csv_file, fieldnames=list(row.keys()))
            self._csv_writer.writeheader()
            self._header_written = True
        self._csv_writer.writerow(row)

    def update(self):
        row = build_row(self.world)
        if self.mode in ("csv", "both"):
            self._write_header_and_row(row)
        if self.mode in ("plot", "both") and HAS_MATPLOTLIB and self.world.tick % self.plot_interval == 0:
            self._update_plots(row)

    def _update_plots(self, row):
        tick = row["tick"]
        x = list(self.world.tick_history)
        if not x:
            return
        # Mother states
        for name in ["energy", "fatigue", "bonding", "fear_threat", "stress", "closeness_child", "OT", "CORT"]:
            key = f"m0_{name}"
            if key not in row:
                continue
            ax, line = self._plot_lines.get(name, (None, None))
            if ax is None:
                continue
            hist = self.world.mother_history.get(self.world.mothers[0].id if self.world.mothers else None)
            if hist and name in hist:
                y = list(hist[name])
                line.set_xdata(x[-len(y):])
                line.set_ydata(y)
            ax.set_xlim(max(0, x[0]), max(10, x[-1]))
        # Child states
        for name in ["hunger", "warmth", "injury"]:
            key = f"c0_{name}"
            if key not in row:
                continue
            ax, line = self._plot_lines.get(name, (None, None))
            if ax is None:
                continue
            if self.world.children and self.world.child_history:
                cid = self.world.children[0].id
                hist = self.world.child_history.get(cid, {})
                if name in hist:
                    y = list(hist[name])
                    line.set_xdata(x[-len(y):])
                    line.set_ydata(y)
            ax.set_xlim(max(0, x[0]), max(10, x[-1]))
        # Motivation
        for mot in ("Forage", "Care", "Self", "Protect"):
            for p in ("mot_", "sel_"):
                key = p + mot
                ax, line = self._plot_lines.get(key, (None, None))
                if ax is None:
                    continue
                col = f"m0_{p.rstrip('_')}_{mot}" if p == "mot_" else f"m0_sel_{mot}"
                if col not in row:
                    continue
                hist = self.world.mother_history.get(self.world.mothers[0].id) if self.world.mothers else None
                if not hist:
                    continue
                hkey = f"mot_{mot.lower()}" if p == "mot_" else f"sel_{mot.lower()}"
                if hkey in hist:
                    y = list(hist[hkey])
                    line.set_xdata(x[-len(y):])
                    line.set_ydata(y)
                ax.set_xlim(max(0, x[0]), max(10, x[-1]))
        # Weights u: fixed constant line, plastic over time
        if self.world.mothers and hasattr(self.world.mothers[0], "u_fixed"):
            m = self.world.mothers[0]
            hist = self.world.mother_history.get(m.id)
            if hist:
                for mot in ["forage", "care", "self", "protect"]:
                    if mot not in m.u_fixed:
                        continue
                    for key in m.u_fixed[mot]:
                        k_f, k_p = f"u_fixed_{mot}_{key}", f"u_plastic_{mot}_{key}"
                        if k_f not in hist or k_p not in hist:
                            continue
                        yf = list(hist[k_f])
                        yp = list(hist[k_p])
                        if not yf or not yp:
                            continue
                        x_plot = x[-len(yf):]
                        y_fixed = [yf[0]] * len(x_plot)
                        lf = self._plot_lines.get(f"u_{mot}_{key}_fixed", (None, None))[1]
                        lp = self._plot_lines.get(f"u_{mot}_{key}_plastic", (None, None))[1]
                        if lf is not None:
                            lf.set_xdata(x_plot)
                            lf.set_ydata(y_fixed)
                        if lp is not None:
                            lp.set_xdata(x[-len(yp):])
                            lp.set_ydata(yp)
        for fig in self._figs:
            fig.canvas.draw()
            fig.canvas.flush_events()
        plt.pause(0.001)

    def close(self):
        if self._csv_file:
            try:
                self._csv_file.close()
            except Exception:
                pass
            self._csv_file = None
        if HAS_MATPLOTLIB:
            for fig in self._figs:
                try:
                    plt.close(fig)
                except Exception:
                    pass
            self._figs.clear()
