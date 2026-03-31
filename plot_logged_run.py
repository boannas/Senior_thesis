<<<<<<< Updated upstream
"""
Load a run log CSV and plot mother states, child states, motivation,
selected motivation, and weights (fixed vs plastic) for analysis.
Usage: python plot_logged_run.py [path/to/run_log.csv] [--out dir]
"""
import argparse
import os
import sys
import numpy as np
import pandas as pd

try:
    HAS_PANDAS = True
except ImportError:
    HAS_PANDAS = False

try:
    import matplotlib.pyplot as plt
    HAS_MATPLOTLIB = True
except ImportError:
    HAS_MATPLOTLIB = False


def load_csv(path):
    if HAS_PANDAS:
        return pd.read_csv(path)
    import csv
    with open(path, "r", encoding="utf-8") as f:
        r = csv.DictReader(f)
        rows = list(r)
    if not rows:
        return None
    return pd.DataFrame(rows)


def plot_run(df, csv_path, out_dir=None):
    if not HAS_MATPLOTLIB:
        print("matplotlib required for plotting. Install: pip install matplotlib")
        return
    if df is None or (hasattr(df, "empty") and df.empty):
        print("No data to plot.")
        return
    # Ensure numeric
    for c in df.columns:
        if c == "tick":
            df[c] = pd.to_numeric(df[c], errors="coerce")
        else:
            df[c] = pd.to_numeric(df[c], errors="coerce")
    x = df["tick"].values

    def save_fig(fig, name):
        if out_dir:
            path = os.path.join(out_dir, name + ".png")
            fig.savefig(path, dpi=150, bbox_inches="tight")
            print("Saved", path)

    # ----- 1. Mother states -----
    mother_cols = [c for c in df.columns if c.startswith("m0_") and c.split("_")[1] in (
        "energy", "fatigue", "bonding", "fear_threat", "stress", "closeness_child", "OT", "CORT"
    )]
    if mother_cols:
        fig1, axes = plt.subplots(3, 2, figsize=(7, 4), sharex=True)
        axes = axes.flatten()
        for ax, col in zip(axes, mother_cols):
            ax.set_title(col.replace("m0_", ""))
            ax.plot(x, df[col].values, "b-", alpha=0.8)
            ax.set_ylabel("Value")
            ax.set_ylim(0, 100)
            ax.grid(True, alpha=0.3)
        axes[-1].set_xlabel("Tick")
        fig1.suptitle("Mother states")
        fig1.tight_layout()
        save_fig(fig1, "mother_states")
        plt.show(block=False)

    # ----- 2. Child states -----
    child_cols = [c for c in df.columns if c.startswith("c0_") and any(s in c for s in ("hunger", "warmth", "injury"))]
    if child_cols:
        fig2, axes = plt.subplots(1, len(child_cols), figsize=(3 * len(child_cols), 2), sharex=True)
        if len(child_cols) == 1:
            axes = [axes]
        for ax, col in zip(axes, child_cols):
            ax.set_title(col.replace("c0_", ""))
            ax.plot(x, df[col].values, "g-", alpha=0.8)
            ax.set_ylabel("Value")
            ax.set_ylim(0, 100)
            ax.grid(True, alpha=0.3)
        axes[-1].set_xlabel("Tick")
        fig2.suptitle("Child states")
        fig2.tight_layout()
        save_fig(fig2, "child_states")
        plt.show(block=False)

    # ----- 3. Motivation values -----
    mot_cols = [c for c in df.columns if c.startswith("m0_mot_")]
    if mot_cols:
        fig3, ax = plt.subplots(figsize=(6, 3))
        colors = {"Forage": "gold", "Care": "limegreen", "Self": "deepskyblue", "Protect": "red"}
        for col in mot_cols:
            mot = col.replace("m0_mot_", "")
            ax.plot(x, df[col].values, label=mot, color=colors.get(mot, "gray"), alpha=0.8)
        ax.set_title("Motivation values")
        ax.set_ylabel("Value")
        ax.set_xlabel("Tick")
        ax.set_ylim(0, 100)
        ax.legend(loc="upper right", ncol=4)
        ax.grid(True, alpha=0.3)
        fig3.tight_layout()
        save_fig(fig3, "motivation_values")
        plt.show(block=False)

    # ----- 4. Selected motivation (1=active) -----
    sel_cols = [c for c in df.columns if c.startswith("m0_sel_")]
    if sel_cols:
        fig4, ax = plt.subplots(figsize=(6, 2))
        colors = {"Forage": "gold", "Care": "limegreen", "Self": "deepskyblue", "Protect": "red"}
        for col in sel_cols:
            mot = col.replace("m0_sel_", "")
            ax.plot(x, df[col].values, label=mot, color=colors.get(mot, "gray"), alpha=0.8)
        ax.set_title("Selected motivation (1 = active)")
        ax.set_ylabel("Active")
        ax.set_xlabel("Tick")
        ax.set_ylim(-0.05, 1.05)
        ax.legend(loc="upper right", ncol=4)
        ax.grid(True, alpha=0.3)
        fig4.tight_layout()
        save_fig(fig4, "selected_motivation")
        plt.show(block=False)

    # ----- 5. Weights u: fixed vs plastic -----
    u_fixed = [c for c in df.columns if c.startswith("m0_u_fixed_")]
    u_plastic = [c for c in df.columns if c.startswith("m0_u_plastic_")]
    if u_fixed and u_plastic:
        # Group by motivation (forage, care, self, protect)
        by_mot = {}
        for col in u_fixed:
            part = col.replace("m0_u_fixed_", "")
            mot = part.split("_")[0]
            key = part[len(mot) + 1:]
            if mot not in by_mot:
                by_mot[mot] = []
            by_mot[mot].append(("fixed", col, key))
        for col in u_plastic:
            part = col.replace("m0_u_plastic_", "")
            mot = part.split("_")[0]
            key = part[len(mot) + 1:]
            if mot not in by_mot:
                by_mot[mot] = []
            by_mot[mot].append(("plastic", col, key))
        mot_order = ["forage", "care", "self", "protect"]
        n = len([m for m in mot_order if m in by_mot])
        if n:
            fig5, axes = plt.subplots(2, 2, figsize=(8, 4), sharex=True)
            axes = axes.flatten()
            colors = plt.cm.tab10.colors
            for idx, mot in enumerate(mot_order):
                if mot not in by_mot:
                    continue
                ax = axes[idx]
                ax.set_title(f"u — {mot}")
                ax.set_ylabel("Weight")
                ax.set_ylim(-0.05, 2.1)
                ax.grid(True, alpha=0.3)
                for j, (typ, col, key) in enumerate(by_mot[mot]):
                    c = colors[j % len(colors)]
                    ls = "--" if typ == "fixed" else "-"
                    if typ == "fixed":
                        y = df[col].values
                        ax.plot(x, [y[0]] * len(x), ls=ls, color=c, alpha=0.9, label=f"{key} (fixed)")
                    else:
                        ax.plot(x, df[col].values, ls=ls, color=c, alpha=0.7, label=f"{key} (plastic)")
                ax.legend(fontsize=6)
            axes[0].set_xlabel("Tick")
            fig5.suptitle("Weights u: fixed (dashed) vs plastic (solid)")
            fig5.tight_layout()
            save_fig(fig5, "weights_u")
            plt.show(block=False)

    # ----- 6. Weights w: fixed vs plastic -----
    w_fixed = [c for c in df.columns if c.startswith("m0_w_fixed_")]
    w_plastic = [c for c in df.columns if c.startswith("m0_w_plastic_")]
    if w_fixed and w_plastic:
        by_cat = {}
        for col in w_fixed:
            part = col.replace("m0_w_fixed_", "")
            cat = part.split("_")[0]
            key = part[len(cat) + 1:] if "_" in part[len(cat):] else part
            # e.g. child_need_hunger -> cat=child, key=need_hunger? No: child_need is category
            bits = part.split("_")
            if len(bits) >= 2:
                cat = bits[0] + "_" + bits[1] if bits[0] == "child" else bits[0]
                key = "_".join(bits[2:]) if len(bits) > 2 else bits[1]
            else:
                cat = bits[0]
                key = ""
            if cat not in by_cat:
                by_cat[cat] = []
            by_cat[cat].append(("fixed", col, key))
        for col in w_plastic:
            part = col.replace("m0_w_plastic_", "")
            bits = part.split("_")
            if len(bits) >= 2:
                cat = bits[0] + "_" + bits[1] if bits[0] == "child" else bits[0]
                key = "_".join(bits[2:]) if len(bits) > 2 else bits[1]
            else:
                cat = bits[0]
                key = ""
            if cat not in by_cat:
                by_cat[cat] = []
            by_cat[cat].append(("plastic", col, key))
        cats = list(by_cat.keys())
        ncat = len(cats)
        if ncat:
            fig6, axes = plt.subplots(2, 3, figsize=(8, 4), sharex=True)
            axes = axes.flatten()
            for idx, cat in enumerate(cats[:6]):
                ax = axes[idx]
                ax.set_title(f"w — {cat}")
                ax.set_ylabel("Weight")
                ax.set_ylim(-0.05, 2.1)
                ax.grid(True, alpha=0.3)
                for j, (typ, col, key) in enumerate(by_cat[cat]):
                    c = plt.cm.tab10.colors[j % 10]
                    ls = "--" if typ == "fixed" else "-"
                    if typ == "fixed":
                        y = df[col].values
                        ax.plot(x, [y[0]] * len(x), ls=ls, color=c, alpha=0.9, label=f"{key} (fixed)")
                    else:
                        ax.plot(x, df[col].values, ls=ls, color=c, alpha=0.7, label=f"{key} (plastic)")
                ax.legend(fontsize=6)
            for j in range(ncat, 6):
                axes[j].set_visible(False)
            axes[0].set_xlabel("Tick")
            fig6.suptitle("Weights w: fixed (dashed) vs plastic (solid)")
            fig6.tight_layout()
            save_fig(fig6, "weights_w")
            plt.show(block=False)

    # ----- 7. Overall deficit (same formula as plasticity: all mother + all child deficits) -----
    n = 100.0
    need = ["m0_energy", "m0_closeness_child", "m0_fear_threat", "m0_stress", "m0_fatigue", "m0_bonding"]
    if all(c in df.columns for c in need):
        e = df["m0_energy"].values
        close = df["m0_closeness_child"].values
        fear = df["m0_fear_threat"].values
        stress = df["m0_stress"].values
        fatigue = df["m0_fatigue"].values
        bond = df["m0_bonding"].values
        m_energy_def = np.maximum(0, 80.0 - e) / n
        m_closeness_def = np.abs(close - 50.0) / n
        m_fear_def = np.maximum(0, fear - 0.0) / n
        m_stress_def = np.maximum(0, stress - 0.0) / n
        m_fatigue_def = np.maximum(0, fatigue - 0.0) / n
        m_bonding_def = np.maximum(0, 100.0 - bond) / n
        total = m_energy_def + m_closeness_def + m_fear_def + m_stress_def + m_fatigue_def + m_bonding_def
        if "c0_hunger" in df.columns and "c0_warmth" in df.columns and "c0_injury" in df.columns:
            h = df["c0_hunger"].values
            w = df["c0_warmth"].values
            inj = df["c0_injury"].values
            total = total + np.maximum(0, h - 0.0) / n + np.abs(w - 50.0) * 2 / n + np.maximum(0, inj - 0.0) / n
        fig7, ax = plt.subplots(figsize=(8, 2.5))
        ax.plot(x, total, "k-", alpha=0.7, label="overall deficit")
        ax.set_title("Overall deficit (all mother + all child deficits, same as plasticity)")
        ax.set_ylabel("Deficit")
        ax.set_xlabel("Tick")
        ax.legend()
        ax.grid(True, alpha=0.3)
        fig7.tight_layout()
        save_fig(fig7, "overall_deficit")
        plt.show(block=False)

    plt.show(block=True)


def main():
    parser = argparse.ArgumentParser(description="Plot logged run CSV")
    parser.add_argument("csv_path", nargs="?", default="run_log.csv", help="Path to run_log.csv")
    parser.add_argument("--out", "-o", default=None, help="Output directory for PNG figures")
    args = parser.parse_args()
    if not os.path.isfile(args.csv_path):
        print("File not found:", args.csv_path)
        sys.exit(1)
    if not HAS_PANDAS:
        print("Pandas required. Install: pip install pandas")
        sys.exit(1)
    df = load_csv(args.csv_path)
    plot_run(df, args.csv_path, out_dir=args.out)


if __name__ == "__main__":
    main()
=======
"""
Load a run log CSV and plot mother states, child states, motivation,
selected motivation, and weights (fixed vs plastic) for analysis.
Usage: python plot_logged_run.py [path/to/run_log.csv] [--out dir]
"""
import argparse
import os
import sys
import numpy as np
import pandas as pd

try:
    HAS_PANDAS = True
except ImportError:
    HAS_PANDAS = False

try:
    import matplotlib.pyplot as plt
    HAS_MATPLOTLIB = True
except ImportError:
    HAS_MATPLOTLIB = False


def load_csv(path):
    if HAS_PANDAS:
        return pd.read_csv(path)
    import csv
    with open(path, "r", encoding="utf-8") as f:
        r = csv.DictReader(f)
        rows = list(r)
    if not rows:
        return None
    return pd.DataFrame(rows)


def plot_run(df, csv_path, out_dir=None):
    if not HAS_MATPLOTLIB:
        print("matplotlib required for plotting. Install: pip install matplotlib")
        return
    if df is None or (hasattr(df, "empty") and df.empty):
        print("No data to plot.")
        return
    # Ensure numeric
    for c in df.columns:
        if c == "tick":
            df[c] = pd.to_numeric(df[c], errors="coerce")
        else:
            df[c] = pd.to_numeric(df[c], errors="coerce")
    x = df["tick"].values

    def save_fig(fig, name):
        if out_dir:
            path = os.path.join(out_dir, name + ".png")
            fig.savefig(path, dpi=150, bbox_inches="tight")
            print("Saved", path)

    # ----- 1. Mother states -----
    mother_cols = [c for c in df.columns if c.startswith("m0_") and c.split("_")[1] in (
        "energy", "fatigue", "bonding", "fear_threat", "stress", "closeness_child", "OT", "CORT"
    )]
    if mother_cols:
        fig1, axes = plt.subplots(3, 2, figsize=(7, 4), sharex=True)
        axes = axes.flatten()
        for ax, col in zip(axes, mother_cols):
            ax.set_title(col.replace("m0_", ""))
            ax.plot(x, df[col].values, "b-", alpha=0.8)
            ax.set_ylabel("Value")
            ax.set_ylim(0, 100)
            ax.grid(True, alpha=0.3)
        axes[-1].set_xlabel("Tick")
        fig1.suptitle("Mother states")
        fig1.tight_layout()
        save_fig(fig1, "mother_states")
        plt.show(block=False)

    # ----- 2. Child states -----
    child_cols = [c for c in df.columns if c.startswith("c0_") and any(s in c for s in ("hunger", "warmth", "injury"))]
    if child_cols:
        fig2, axes = plt.subplots(1, len(child_cols), figsize=(3 * len(child_cols), 2), sharex=True)
        if len(child_cols) == 1:
            axes = [axes]
        for ax, col in zip(axes, child_cols):
            ax.set_title(col.replace("c0_", ""))
            ax.plot(x, df[col].values, "g-", alpha=0.8)
            ax.set_ylabel("Value")
            ax.set_ylim(0, 100)
            ax.grid(True, alpha=0.3)
        axes[-1].set_xlabel("Tick")
        fig2.suptitle("Child states")
        fig2.tight_layout()
        save_fig(fig2, "child_states")
        plt.show(block=False)

    # ----- 3. Motivation values -----
    mot_cols = [c for c in df.columns if c.startswith("m0_mot_")]
    if mot_cols:
        fig3, ax = plt.subplots(figsize=(6, 3))
        colors = {"Forage": "gold", "Care": "limegreen", "Self": "deepskyblue", "Protect": "red"}
        for col in mot_cols:
            mot = col.replace("m0_mot_", "")
            ax.plot(x, df[col].values, label=mot, color=colors.get(mot, "gray"), alpha=0.8)
        ax.set_title("Motivation values")
        ax.set_ylabel("Value")
        ax.set_xlabel("Tick")
        ax.set_ylim(0, 100)
        ax.legend(loc="upper right", ncol=4)
        ax.grid(True, alpha=0.3)
        fig3.tight_layout()
        save_fig(fig3, "motivation_values")
        plt.show(block=False)

    # ----- 4. Selected motivation (1=active) -----
    sel_cols = [c for c in df.columns if c.startswith("m0_sel_")]
    if sel_cols:
        fig4, ax = plt.subplots(figsize=(6, 2))
        colors = {"Forage": "gold", "Care": "limegreen", "Self": "deepskyblue", "Protect": "red"}
        for col in sel_cols:
            mot = col.replace("m0_sel_", "")
            ax.plot(x, df[col].values, label=mot, color=colors.get(mot, "gray"), alpha=0.8)
        ax.set_title("Selected motivation (1 = active)")
        ax.set_ylabel("Active")
        ax.set_xlabel("Tick")
        ax.set_ylim(-0.05, 1.05)
        ax.legend(loc="upper right", ncol=4)
        ax.grid(True, alpha=0.3)
        fig4.tight_layout()
        save_fig(fig4, "selected_motivation")
        plt.show(block=False)

    # ----- 5. Weights u: fixed vs plastic -----
    u_fixed = [c for c in df.columns if c.startswith("m0_u_fixed_")]
    u_plastic = [c for c in df.columns if c.startswith("m0_u_plastic_")]
    if u_fixed and u_plastic:
        # Group by motivation (forage, care, self, protect)
        by_mot = {}
        for col in u_fixed:
            part = col.replace("m0_u_fixed_", "")
            mot = part.split("_")[0]
            key = part[len(mot) + 1:]
            if mot not in by_mot:
                by_mot[mot] = []
            by_mot[mot].append(("fixed", col, key))
        for col in u_plastic:
            part = col.replace("m0_u_plastic_", "")
            mot = part.split("_")[0]
            key = part[len(mot) + 1:]
            if mot not in by_mot:
                by_mot[mot] = []
            by_mot[mot].append(("plastic", col, key))
        mot_order = ["forage", "care", "self", "protect"]
        n = len([m for m in mot_order if m in by_mot])
        if n:
            fig5, axes = plt.subplots(2, 2, figsize=(8, 4), sharex=True)
            axes = axes.flatten()
            colors = plt.cm.tab10.colors
            for idx, mot in enumerate(mot_order):
                if mot not in by_mot:
                    continue
                ax = axes[idx]
                ax.set_title(f"u — {mot}")
                ax.set_ylabel("Weight")
                ax.set_ylim(-0.05, 2.1)
                ax.grid(True, alpha=0.3)
                for j, (typ, col, key) in enumerate(by_mot[mot]):
                    c = colors[j % len(colors)]
                    ls = "--" if typ == "fixed" else "-"
                    if typ == "fixed":
                        y = df[col].values
                        ax.plot(x, [y[0]] * len(x), ls=ls, color=c, alpha=0.9, label=f"{key} (fixed)")
                    else:
                        ax.plot(x, df[col].values, ls=ls, color=c, alpha=0.7, label=f"{key} (plastic)")
                ax.legend(fontsize=6)
            axes[0].set_xlabel("Tick")
            fig5.suptitle("Weights u: fixed (dashed) vs plastic (solid)")
            fig5.tight_layout()
            save_fig(fig5, "weights_u")
            plt.show(block=False)

    # ----- 6. Weights w: fixed vs plastic -----
    w_fixed = [c for c in df.columns if c.startswith("m0_w_fixed_")]
    w_plastic = [c for c in df.columns if c.startswith("m0_w_plastic_")]
    if w_fixed and w_plastic:
        by_cat = {}
        for col in w_fixed:
            part = col.replace("m0_w_fixed_", "")
            cat = part.split("_")[0]
            key = part[len(cat) + 1:] if "_" in part[len(cat):] else part
            # e.g. child_need_hunger -> cat=child, key=need_hunger? No: child_need is category
            bits = part.split("_")
            if len(bits) >= 2:
                cat = bits[0] + "_" + bits[1] if bits[0] == "child" else bits[0]
                key = "_".join(bits[2:]) if len(bits) > 2 else bits[1]
            else:
                cat = bits[0]
                key = ""
            if cat not in by_cat:
                by_cat[cat] = []
            by_cat[cat].append(("fixed", col, key))
        for col in w_plastic:
            part = col.replace("m0_w_plastic_", "")
            bits = part.split("_")
            if len(bits) >= 2:
                cat = bits[0] + "_" + bits[1] if bits[0] == "child" else bits[0]
                key = "_".join(bits[2:]) if len(bits) > 2 else bits[1]
            else:
                cat = bits[0]
                key = ""
            if cat not in by_cat:
                by_cat[cat] = []
            by_cat[cat].append(("plastic", col, key))
        cats = list(by_cat.keys())
        ncat = len(cats)
        if ncat:
            fig6, axes = plt.subplots(2, 3, figsize=(8, 4), sharex=True)
            axes = axes.flatten()
            for idx, cat in enumerate(cats[:6]):
                ax = axes[idx]
                ax.set_title(f"w — {cat}")
                ax.set_ylabel("Weight")
                ax.set_ylim(-0.05, 2.1)
                ax.grid(True, alpha=0.3)
                for j, (typ, col, key) in enumerate(by_cat[cat]):
                    c = plt.cm.tab10.colors[j % 10]
                    ls = "--" if typ == "fixed" else "-"
                    if typ == "fixed":
                        y = df[col].values
                        ax.plot(x, [y[0]] * len(x), ls=ls, color=c, alpha=0.9, label=f"{key} (fixed)")
                    else:
                        ax.plot(x, df[col].values, ls=ls, color=c, alpha=0.7, label=f"{key} (plastic)")
                ax.legend(fontsize=6)
            for j in range(ncat, 6):
                axes[j].set_visible(False)
            axes[0].set_xlabel("Tick")
            fig6.suptitle("Weights w: fixed (dashed) vs plastic (solid)")
            fig6.tight_layout()
            save_fig(fig6, "weights_w")
            plt.show(block=False)

    # ----- 7. Overall deficit (same formula as plasticity: all mother + all child deficits) -----
    n = 100.0
    need = ["m0_energy", "m0_closeness_child", "m0_fear_threat", "m0_stress", "m0_fatigue", "m0_bonding"]
    if all(c in df.columns for c in need):
        e = df["m0_energy"].values
        close = df["m0_closeness_child"].values
        fear = df["m0_fear_threat"].values
        stress = df["m0_stress"].values
        fatigue = df["m0_fatigue"].values
        bond = df["m0_bonding"].values
        m_energy_def = np.maximum(0, 80.0 - e) / n
        m_closeness_def = np.abs(close - 50.0) / n
        m_fear_def = np.maximum(0, fear - 0.0) / n
        m_stress_def = np.maximum(0, stress - 0.0) / n
        m_fatigue_def = np.maximum(0, fatigue - 0.0) / n
        m_bonding_def = np.maximum(0, 100.0 - bond) / n
        total = m_energy_def + m_closeness_def + m_fear_def + m_stress_def + m_fatigue_def + m_bonding_def
        if "c0_hunger" in df.columns and "c0_warmth" in df.columns and "c0_injury" in df.columns:
            h = df["c0_hunger"].values
            w = df["c0_warmth"].values
            inj = df["c0_injury"].values
            total = total + np.maximum(0, h - 0.0) / n + np.abs(w - 50.0) * 2 / n + np.maximum(0, inj - 0.0) / n
        fig7, ax = plt.subplots(figsize=(8, 2.5))
        ax.plot(x, total, "k-", alpha=0.7, label="overall deficit")
        ax.set_title("Overall deficit (all mother + all child deficits, same as plasticity)")
        ax.set_ylabel("Deficit")
        ax.set_xlabel("Tick")
        ax.legend()
        ax.grid(True, alpha=0.3)
        fig7.tight_layout()
        save_fig(fig7, "overall_deficit")
        plt.show(block=False)

    plt.show(block=True)


def main():
    parser = argparse.ArgumentParser(description="Plot logged run CSV")
    parser.add_argument("csv_path", nargs="?", default="run_log.csv", help="Path to run_log.csv")
    parser.add_argument("--out", "-o", default=None, help="Output directory for PNG figures")
    args = parser.parse_args()
    if not os.path.isfile(args.csv_path):
        print("File not found:", args.csv_path)
        sys.exit(1)
    if not HAS_PANDAS:
        print("Pandas required. Install: pip install pandas")
        sys.exit(1)
    df = load_csv(args.csv_path)
    plot_run(df, args.csv_path, out_dir=args.out)


if __name__ == "__main__":
    main()
>>>>>>> Stashed changes
