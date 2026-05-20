#!/usr/bin/env python3
"""
Improved emergence-style TTD boxplots (paper-like aesthetics)
"""

from __future__ import annotations

import argparse
import glob
import os
import re
import sys

import numpy as np

try:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import pandas as pd
except ImportError as e:
    raise SystemExit(f"Requires pandas+matplotlib: {e}") from e

# Paper-like styling (similar to thesis diagrams)
plt.rcParams.update(
    {
        "font.family": "serif",
        "font.serif": ["Times New Roman", "Times", "DejaVu Serif"],
        "font.size": 11,
        "axes.titlesize": 12,
        "axes.labelsize": 11,
        "xtick.labelsize": 10,
        "ytick.labelsize": 10,
        "axes.spines.top": False,
        "axes.spines.right": False,
        "axes.linewidth": 0.9,
        "axes.grid": True,
        "grid.color": "0.90",
        "grid.linewidth": 0.8,
        "grid.alpha": 1.0,
        "figure.facecolor": "white",
        "axes.facecolor": "white",
        "savefig.facecolor": "white",
    }
)

INIT_ORDER = ["anti_maternal", "random_uniform", "pro_maternal"]
PLAST_ORDER = ["E2", "P1", "P2"]

# Softer academic palette
COLORS = {
    "E2": "#4C78A8",
    "P1": "#E45756",
    "P2": "#54A24B",
}

RUN_RE = re.compile(
    r"^(?P<plast>E2|P1|P2)_(?P<init>anti_maternal|random_uniform|pro_maternal)_seed(?P<seed>\d+)$"
)


def _load_all_rows(root: str, day_step: float) -> pd.DataFrame:
    paths = sorted(glob.glob(os.path.join(root, "*", "ttd_across_seeds.csv")))
    if not paths:
        raise SystemExit(f"No CSV found under {root}")

    rows = []
    for p in paths:
        run_dir = os.path.basename(os.path.dirname(p))
        m = RUN_RE.match(run_dir)
        if not m:
            continue

        plast = m.group("plast")
        init = m.group("init")
        evolve_seed = int(m.group("seed"))

        df = pd.read_csv(p)

        for _, r in df.iterrows():
            try:
                max_tick = float(r["max_tick"])
                child_tick = float(r["child_death_tick"])
                mother_tick = float(r["mother_death_tick"])
            except Exception:
                continue

            if not np.isfinite(max_tick) or max_tick <= 0:
                continue

            rows.append(
                dict(
                    plasticity=plast,
                    init=init,
                    evolve_seed=evolve_seed,
                    child_ttd_norm=child_tick / max_tick,
                    mother_ttd_norm=mother_tick / max_tick,
                    child_days=child_tick / day_step,
                    mother_days=mother_tick / day_step,
                )
            )

    return pd.DataFrame(rows)


def _styled_boxplot(ax, data, labels, colors):
    bp = ax.boxplot(
        data,
        labels=labels,
        patch_artist=True,
        widths=0.6,
        showmeans=True,
        medianprops=dict(color="#F28E2B", linewidth=1.8),  # orange median
        meanprops=dict(
            marker="^",
            markerfacecolor="#2ca02c",
            markeredgecolor="#2ca02c",
            markersize=7,
        ),
        whiskerprops=dict(color="0.25", linewidth=0.9),
        capprops=dict(color="0.25", linewidth=0.9),
        flierprops=dict(
            marker="o",
            markerfacecolor="none",
            markeredgecolor="0.20",
            markersize=5,
            linestyle="none",
            alpha=0.18,
        ),
    )

    # Color boxes
    for i, patch in enumerate(bp["boxes"]):
        col = colors[i]
        patch.set_facecolor(col)
        patch.set_alpha(0.16)
        patch.set_edgecolor(col)
        patch.set_linewidth(1.1)

    return bp


def _emergence_plot(df, value_col, ylabel, title, out_path, *, layout: str = "split_init"):
    rng = np.random.default_rng(0)

    fig, axes = plt.subplots(1, 3, figsize=(13.5, 4.5), sharey=True)

    layout = str(layout).lower()
    if layout not in ("split_init", "split_plasticity"):
        raise ValueError("layout must be split_init or split_plasticity")

    if layout == "split_init":
        panels = INIT_ORDER
        panel_key = "init"
        within_order = PLAST_ORDER
        within_key = "plasticity"
        within_labels = [x for x in PLAST_ORDER]
        within_colors = [COLORS[x] for x in PLAST_ORDER]
    else:
        panels = PLAST_ORDER
        panel_key = "plasticity"
        within_order = INIT_ORDER
        within_key = "init"
        within_labels = [x.replace("_", " ") for x in INIT_ORDER]
        within_colors = ["0.45"] * len(INIT_ORDER)  # keep one color per panel; points are ghosted

    for ax, panel_val in zip(axes, panels):
        sub = df[df[panel_key] == panel_val]

        data = []
        cols = []

        for w in within_order:
            vals = sub[sub[within_key] == w][value_col].dropna().to_numpy()
            data.append(vals)
            if layout == "split_init":
                cols.append(COLORS[w])
            else:
                cols.append(COLORS[panel_val])

        labels = within_labels

        _styled_boxplot(ax, data, labels, cols)

        # Scatter overlay (subtle; makes outliers more "ghost")
        for i, vals in enumerate(data):
            if len(vals) == 0:
                continue
            x = rng.normal(i + 1, 0.04, size=len(vals))
            ax.scatter(
                x,
                vals,
                s=10,
                alpha=0.14,
                color=cols[i] if layout == "split_init" else "0.2",
                edgecolors="none",
            )

        # Styling
        ax.set_title(str(panel_val).replace("_", " "), fontsize=12, pad=8)
        ax.grid(True, axis="y", alpha=0.8)
        ax.grid(False, axis="x")

    axes[0].set_ylabel(ylabel, fontsize=12)

    # Clean spines
    for ax in axes:
        ax.spines["top"].set_visible(False)
        ax.spines["right"].set_visible(False)

    fig.suptitle(title, fontsize=13, y=1.02)
    fig.tight_layout()

    fig.savefig(out_path, dpi=200, bbox_inches="tight")
    plt.close(fig)

    print("Saved:", out_path)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--root", required=True)
    ap.add_argument("--units", choices=["norm", "days"], default="norm")
    ap.add_argument("--layout", choices=["split_init", "split_plasticity"], default="split_init")
    ap.add_argument("--day-step", type=float, default=100.0)
    ap.add_argument("--title-prefix", default="")
    args = ap.parse_args()

    df = _load_all_rows(args.root, args.day_step)

    out_dir = os.path.join(args.root, "figures")
    os.makedirs(out_dir, exist_ok=True)

    if args.units == "days":
        child_col = "child_days"
        mother_col = "mother_days"
        ylabel = "TTD (days)"
    else:
        child_col = "child_ttd_norm"
        mother_col = "mother_ttd_norm"
        ylabel = "TTD (normalized 0–1)"

    prefix = (args.title_prefix + " — ") if args.title_prefix else ""
    _emergence_plot(
        df,
        child_col,
        ylabel,
        f"{prefix}Child TTD by init × plasticity (final genomes; headless rollouts)",
        os.path.join(out_dir, "boxplot_child_ttd_norm.png" if args.units == "norm" else "boxplot_child_ttd_days.png"),
        layout=args.layout,
    )

    _emergence_plot(
        df,
        mother_col,
        ylabel,
        f"{prefix}Mother TTD by init × plasticity (final genomes; headless rollouts)",
        os.path.join(out_dir, "boxplot_mother_ttd_norm.png" if args.units == "norm" else "boxplot_mother_ttd_days.png"),
        layout=args.layout,
    )


if __name__ == "__main__":
    main()