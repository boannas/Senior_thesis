"""
Load a run log CSV and plot mother states, child states, motivation,
selected motivation, and weights (fixed vs plastic) for analysis.
Usage: python plot_logged_run.py [path/to/run_log.csv] [--out dir]
"""
import argparse
import os
import sys
import numpy as np

try:
    import pandas as pd
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


# Default: plot all. Override with --mother-states / --child-states (comma-separated).
ALL_MOTHER_STATES = ("energy", "fatigue", "bonding", "fear_threat", "stress", "closeness_child", "OT", "CORT")
ALL_CHILD_STATES = ("hunger", "warmth", "injury")


def _agent_indices(df, prefix):
    """Return sorted list of agent indices present in columns (e.g. m0_, m1_ -> [0,1])."""
    out = set()
    for c in df.columns:
        if c.startswith(prefix) and len(c) > len(prefix):
            after = c[len(prefix) :].split("_", 1)[0]
            if after.isdigit():
                out.add(int(after))
    return sorted(out)


def plot_run(
    df,
    csv_path,
    out_dir=None,
    mother_states=None,
    child_states=None,
    mother_indices=None,
    child_indices=None,
    mother_viz="together",
    child_viz="together",
):
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

    moms = mother_indices if mother_indices is not None else _agent_indices(df, "m")
    kids = child_indices if child_indices is not None else _agent_indices(df, "c")
    if not moms:
        moms = [0] if any(c.startswith("m0_") for c in df.columns) else []
    if not kids:
        kids = [0] if any(c.startswith("c0_") for c in df.columns) else []
    colors_m = plt.cm.tab10(np.linspace(0, 1, max(len(moms), 1)))

    # ----- 1. Mother states -----
    want_m = mother_states if mother_states is not None else ALL_MOTHER_STATES
    if moms:
        if mother_viz == "together":
            # One subplot per state; each subplot has one line per mother (m0, m1, m2)
            state_cols = [s for s in want_m if any(f"m{i}_{s}" in df.columns for i in moms)]
            if state_cols:
                n = len(state_cols)
                ncol = min(2, n)
                nrow = max(1, (n + ncol - 1) // ncol)
                fig1, axes = plt.subplots(nrow, ncol, figsize=(4 * ncol, 3 * nrow), sharex=True)
                axes = np.atleast_1d(axes).flatten()
                for ax, state_name in zip(axes, state_cols):
                    ax.set_title(state_name)
                    ax.set_ylabel("Value")
                    ax.set_ylim(0, 100)
                    ax.grid(True, alpha=0.3)
                    for i, mi in enumerate(moms):
                        col = f"m{mi}_{state_name}"
                        if col in df.columns:
                            ax.plot(x, df[col].values, "-", color=colors_m[i % len(colors_m)], alpha=0.8, label=f"M{mi}")
                    ax.legend(loc="upper right", fontsize=7)
                for j in range(len(state_cols), len(axes)):
                    axes[j].set_visible(False)
                axes[min(n - 1, len(axes) - 1)].set_xlabel("Tick")
                fig1.suptitle("Mother states (all)")
                fig1.tight_layout()
                save_fig(fig1, "mother_states")
                plt.show(block=False)
        else:
            # Separate: one figure per mother
            for mi in moms:
                mother_cols = [f"m{mi}_{s}" for s in want_m if f"m{mi}_{s}" in df.columns]
                if not mother_cols:
                    continue
                n = len(mother_cols)
                ncol = min(2, n)
                nrow = max(1, (n + ncol - 1) // ncol)
                fig1, axes = plt.subplots(nrow, ncol, figsize=(4 * ncol, 3 * nrow), sharex=True)
                axes = np.atleast_1d(axes).flatten()
                for ax, col in zip(axes, mother_cols):
                    ax.set_title(col.replace(f"m{mi}_", ""))
                    ax.plot(x, df[col].values, "b-", alpha=0.8)
                    ax.set_ylabel("Value")
                    ax.set_ylim(0, 100)
                    ax.grid(True, alpha=0.3)
                for j in range(len(mother_cols), len(axes)):
                    axes[j].set_visible(False)
                axes[min(n - 1, len(axes) - 1)].set_xlabel("Tick")
                fig1.suptitle(f"Mother M{mi} states")
                fig1.tight_layout()
                save_fig(fig1, f"mother_states_m{mi}")
                plt.show(block=False)

    # ----- 2. Child states -----
    want_c = child_states if child_states is not None else ALL_CHILD_STATES
    colors_c = plt.cm.Set2(np.linspace(0, 1, max(len(kids), 1)))
    if kids:
        if child_viz == "together":
            state_cols = [s for s in want_c if any(f"c{i}_{s}" in df.columns for i in kids)]
            if state_cols:
                n = len(state_cols)
                fig2, axes = plt.subplots(1, n, figsize=(3 * n, 2.5), sharex=True)
                if n == 1:
                    axes = [axes]
                for ax, state_name in zip(axes, state_cols):
                    ax.set_title(state_name)
                    ax.set_ylabel("Value")
                    ax.set_ylim(0, 100)
                    ax.grid(True, alpha=0.3)
                    for i, ki in enumerate(kids):
                        col = f"c{ki}_{state_name}"
                        if col in df.columns:
                            ax.plot(x, df[col].values, "-", color=colors_c[i % len(colors_c)], alpha=1.0, label=f"C{ki}")
                    ax.legend(loc="upper right", fontsize=7)
                axes[-1].set_xlabel("Tick")
                fig2.suptitle("Child states (all)")
                fig2.tight_layout()
                save_fig(fig2, "child_states")
                plt.show(block=False)
        else:
            for ki in kids:
                child_cols = [f"c{ki}_{s}" for s in want_c if f"c{ki}_{s}" in df.columns]
                if not child_cols:
                    continue
                fig2, axes = plt.subplots(1, len(child_cols), figsize=(3 * len(child_cols), 2), sharex=True)
                if len(child_cols) == 1:
                    axes = [axes]
                for ax, col in zip(axes, child_cols):
                    ax.set_title(col.replace(f"c{ki}_", ""))
                    ax.plot(x, df[col].values, "g-", alpha=0.8)
                    ax.set_ylabel("Value")
                    ax.set_ylim(0, 100)
                    ax.grid(True, alpha=0.3)
                axes[-1].set_xlabel("Tick")
                fig2.suptitle(f"Child C{ki} states")
                fig2.tight_layout()
                save_fig(fig2, f"child_states_c{ki}")
                plt.show(block=False)

    # ----- 3. Motivation values (all selected mothers: together or one line per mother) -----
    mot_names = ["Forage", "Care", "Self", "Protect"]
    if mother_viz == "together" and len(moms) > 1:
        fig3, axes = plt.subplots(2, 2, figsize=(8, 4), sharex=True)
        axes = axes.flatten()
        for ai, mot in enumerate(mot_names):
            ax = axes[ai]
            ax.set_title(mot)
            ax.set_ylabel("Value")
            ax.set_ylim(0, 100)
            ax.grid(True, alpha=0.3)
            for i, mi in enumerate(moms):
                col = f"m{mi}_mot_{mot}"
                if col in df.columns:
                    ax.plot(x, df[col].values, "-", color=colors_m[i % len(colors_m)], alpha=0.8, label=f"M{mi}")
            ax.legend(loc="upper right", fontsize=7)
        axes[-1].set_xlabel("Tick")
        fig3.suptitle("Motivation values (all)")
        fig3.tight_layout()
        save_fig(fig3, "motivation_values")
        plt.show(block=False)
    else:
        for mi in (moms if mother_viz == "separate" else [moms[0]] if moms else []):
            mot_cols = [f"m{mi}_mot_{mot}" for mot in mot_names if f"m{mi}_mot_{mot}" in df.columns]
            if mot_cols:
                fig3, ax = plt.subplots(figsize=(6, 3))
                colors = {"Forage": "gold", "Care": "limegreen", "Self": "deepskyblue", "Protect": "red"}
                for col in mot_cols:
                    mot = col.replace(f"m{mi}_mot_", "")
                    ax.plot(x, df[col].values, label=mot, color=colors.get(mot, "gray"), alpha=0.8)
                ax.set_title(f"Motivation values M{mi}" if len(moms) > 1 else "Motivation values")
                ax.set_ylabel("Value")
                ax.set_xlabel("Tick")
                ax.set_ylim(0, 100)
                ax.legend(loc="upper right", ncol=4)
                ax.grid(True, alpha=0.3)
                fig3.tight_layout()
                save_fig(fig3, f"motivation_values_m{mi}" if len(moms) > 1 else "motivation_values")
                plt.show(block=False)

    # ----- 4. Selected motivation (1=active) -----
    if mother_viz == "together" and len(moms) > 1:
        fig4, ax = plt.subplots(figsize=(8, 2.5))
        for i, mi in enumerate(moms):
            for mot in mot_names:
                col = f"m{mi}_sel_{mot}"
                if col in df.columns:
                    ax.plot(x, df[col].values, "-", color=colors_m[i % len(colors_m)], alpha=0.7, label=f"M{mi} {mot}")
        ax.set_title("Selected motivation (1 = active, all mothers)")
        ax.set_ylabel("Active")
        ax.set_xlabel("Tick")
        ax.set_ylim(-0.05, 1.05)
        ax.legend(loc="upper right", ncol=min(4, len(moms) * 4), fontsize=6)
        ax.grid(True, alpha=0.3)
        fig4.tight_layout()
        save_fig(fig4, "selected_motivation")
        plt.show(block=False)
    else:
        for mi in (moms if mother_viz == "separate" else [moms[0]] if moms else []):
            sel_cols = [f"m{mi}_sel_{mot}" for mot in mot_names if f"m{mi}_sel_{mot}" in df.columns]
            if sel_cols:
                fig4, ax = plt.subplots(figsize=(6, 2))
                colors = {"Forage": "gold", "Care": "limegreen", "Self": "deepskyblue", "Protect": "red"}
                for col in sel_cols:
                    mot = col.replace(f"m{mi}_sel_", "")
                    ax.plot(x, df[col].values, label=mot, color=colors.get(mot, "gray"), alpha=0.8)
                ax.set_title(f"Selected motivation M{mi}" if len(moms) > 1 else "Selected motivation (1 = active)")
                ax.set_ylabel("Active")
                ax.set_xlabel("Tick")
                ax.set_ylim(-0.05, 1.05)
                ax.legend(loc="upper right", ncol=4)
                ax.grid(True, alpha=0.3)
                fig4.tight_layout()
                save_fig(fig4, f"selected_motivation_m{mi}" if len(moms) > 1 else "selected_motivation")
                plt.show(block=False)

    # ----- 5. Weights u: fixed vs plastic (per selected mother(s)) -----
    def _weight_u_fig(mi, suptitle_suffix=""):
        u_fixed = [c for c in df.columns if c.startswith(f"m{mi}_u_fixed_")]
        u_plastic = [c for c in df.columns if c.startswith(f"m{mi}_u_plastic_")]
        if not u_fixed or not u_plastic:
            return
        by_mot = {}
        for col in u_fixed:
            part = col.replace(f"m{mi}_u_fixed_", "")
            mot = part.split("_")[0]
            key = part[len(mot) + 1:]
            if mot not in by_mot:
                by_mot[mot] = []
            by_mot[mot].append(("fixed", col, key))
        for col in u_plastic:
            part = col.replace(f"m{mi}_u_plastic_", "")
            mot = part.split("_")[0]
            key = part[len(mot) + 1:]
            if mot not in by_mot:
                by_mot[mot] = []
            by_mot[mot].append(("plastic", col, key))
        mot_order = ["forage", "care", "self", "protect"]
        n = len([m for m in mot_order if m in by_mot])
        if not n:
            return
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
        fig5.suptitle(f"Weights u M{mi}{suptitle_suffix}: fixed (dashed) vs plastic (solid)")
        fig5.tight_layout()
        save_fig(fig5, f"weights_u_m{mi}" if suptitle_suffix.strip() else "weights_u")
        plt.show(block=False)

    if mother_viz == "together" and len(moms) > 1:
        for mi in moms:
            _weight_u_fig(mi, " (one fig per mother)")
    else:
        for mi in (moms if mother_viz == "separate" else [moms[0]] if moms else []):
            _weight_u_fig(mi, f" _m{mi}" if len(moms) > 1 else "")

    # ----- 6. Weights w: fixed vs plastic -----
    def _weight_w_fig(mi, suptitle_suffix=""):
        w_fixed = [c for c in df.columns if c.startswith(f"m{mi}_w_fixed_")]
        w_plastic = [c for c in df.columns if c.startswith(f"m{mi}_w_plastic_")]
        if not w_fixed or not w_plastic:
            return
        by_cat = {}
        pre = f"m{mi}_w_fixed_"
        for col in w_fixed:
            part = col.replace(pre, "")
            bits = part.split("_")
            if len(bits) >= 2 and bits[0] == "child" and bits[1] == "need":
                cat, key = "child_need", "_".join(bits[2:])
            elif len(bits) >= 1:
                cat, key = bits[0], "_".join(bits[1:]) if len(bits) > 1 else ""
            else:
                cat, key = "?", part
            if cat not in by_cat:
                by_cat[cat] = []
            by_cat[cat].append(("fixed", col, key or col))
        pre = f"m{mi}_w_plastic_"
        for col in w_plastic:
            part = col.replace(pre, "")
            bits = part.split("_")
            if len(bits) >= 2 and bits[0] == "child" and bits[1] == "need":
                cat, key = "child_need", "_".join(bits[2:])
            elif len(bits) >= 1:
                cat, key = bits[0], "_".join(bits[1:]) if len(bits) > 1 else ""
            else:
                cat, key = "?", part
            if cat not in by_cat:
                by_cat[cat] = []
            by_cat[cat].append(("plastic", col, key or col))
        cats = list(by_cat.keys())
        ncat = len(cats)
        if not ncat:
            return
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
        fig6.suptitle(f"Weights w M{mi}{suptitle_suffix}: fixed (dashed) vs plastic (solid)")
        fig6.tight_layout()
        save_fig(fig6, f"weights_w_m{mi}" if suptitle_suffix.strip() else "weights_w")
        plt.show(block=False)

    if mother_viz == "together" and len(moms) > 1:
        for mi in moms:
            _weight_w_fig(mi, " (one fig per mother)")
    else:
        for mi in (moms if mother_viz == "separate" else [moms[0]] if moms else []):
            _weight_w_fig(mi)

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
    parser.add_argument(
        "--mother-states",
        default=None,
        help="Comma-separated mother states to plot (default: all). e.g. energy,fatigue,stress",
    )
    parser.add_argument(
        "--child-states",
        default=None,
        help="Comma-separated child states to plot (default: all). e.g. hunger,warmth,injury",
    )
    parser.add_argument(
        "--mothers",
        default=None,
        help="Mother indices to include: comma-separated (e.g. 0,1,2) or 'all' (default: all in CSV)",
    )
    parser.add_argument(
        "--children",
        default=None,
        help="Child indices to include: comma-separated (e.g. 0,1) or 'all' (default: all in CSV)",
    )
    parser.add_argument(
        "--mother-viz",
        choices=("together", "separate"),
        default="together",
        help="Mother states/motivation: 'together' (M0,M1,M2 on same plot) or 'separate' (one figure per mother)",
    )
    parser.add_argument(
        "--child-viz",
        choices=("together", "separate"),
        default="together",
        help="Child states: 'together' (C0,C1 on same plot) or 'separate' (one figure per child)",
    )
    args = parser.parse_args()
    if not os.path.isfile(args.csv_path):
        print("File not found:", args.csv_path)
        sys.exit(1)
    if not HAS_PANDAS:
        print("Pandas required. Install: pip install pandas")
        sys.exit(1)
    mother_states = [s.strip() for s in args.mother_states.split(",")] if args.mother_states else None
    child_states = [s.strip() for s in args.child_states.split(",")] if args.child_states else None
    mother_indices = None
    if args.mothers is not None:
        mother_indices = [int(s.strip()) for s in args.mothers.split(",")] if args.mothers != "all" else None
    child_indices = None
    if args.children is not None:
        child_indices = [int(s.strip()) for s in args.children.split(",")] if args.children != "all" else None
    df = load_csv(args.csv_path)
    plot_run(
        df,
        args.csv_path,
        out_dir=args.out,
        mother_states=mother_states,
        child_states=child_states,
        mother_indices=mother_indices,
        child_indices=child_indices,
        mother_viz=args.mother_viz,
        child_viz=args.child_viz,
    )


if __name__ == "__main__":
    main()
