#!/usr/bin/env python3
"""
plot_e4_death_cause_summary_slide.py
====================================
Create a slide-ready summary figure for E4 child death causes.

Uses the trend CSVs written by `plot_e4_child_death_causes_all.py`:
  - E4_child_death_causes_trends__by_world_init_plasticity.csv
  - E4_child_death_causes_trends__by_food_init_plasticity.csv

Output:
  <out-dir>/E4_child_death_causes_summary_slide.png

Figure design (for first-time readers):
  - 3×3 grid: rows = init (anti/random/pro), cols = plasticity (E2/P1/P2)
  - Left half: three mini stacked bars (easy/normal/hard) per cell
  - Right half: three mini stacked bars (food 5/15/25) per cell
"""

from __future__ import annotations

import argparse
import os

import numpy as np

try:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import pandas as pd
except ImportError as e:
    raise SystemExit(f"Requires pandas+matplotlib: {e}") from e


CAUSE_ORDER = ["alive", "injury", "hunger", "cold", "heat", "unknown_nonfinite", "unknown_missing"]
CAUSE_COLORS = {
    "alive": "#7f7f7f",
    "injury": "#9467bd",
    "hunger": "#e15759",
    "cold": "#4e79a7",
    "heat": "#f28e2b",
    "unknown_nonfinite": "#9c755f",
    "unknown_missing": "#bab0ac",
}

INIT_ORDER = ("anti_maternal", "random_uniform", "pro_maternal")
INIT_LABELS = {
    "anti_maternal": "anti-maternal",
    "random_uniform": "random",
    "pro_maternal": "pro-maternal",
}
PLAST_ORDER = ("E2", "P1", "P2")
PLAST_TITLES = {"E2": "E2", "P1": "P1", "P2": "P2"}
WORLD_ORDER = ("easy", "normal", "hard")
FOOD_ORDER = (5, 15, 25)


def _stacked_mini_bars(ax: plt.Axes, df: pd.DataFrame, *, x_keys: list, x_labels: list[str]) -> None:
    """
    Draw three (or N) stacked bars on given axis using frac_* columns.
    df is expected to have one row per x_key in x_keys.
    """
    x = np.arange(len(x_keys), dtype=float)
    bottom = np.zeros(len(x_keys), dtype=float)

    idx = {k: i for i, k in enumerate(x_keys)}
    for cause in CAUSE_ORDER:
        vals = np.zeros(len(x_keys), dtype=float)
        col = f"frac_{cause}"
        if col not in df.columns:
            continue
        for _, r in df.iterrows():
            k = r.iloc[0]  # not used; we access by key column below
        for _, r in df.iterrows():
            # row key can be 'world' or 'food', so just detect which exists
            if "world" in df.columns:
                k = r["world"]
            else:
                k = int(r["food"])
            if k not in idx:
                continue
            vals[idx[k]] = float(r[col])

        ax.bar(x, vals, bottom=bottom, color=CAUSE_COLORS.get(cause, "#999999"), linewidth=0.0)
        bottom += vals

    ax.set_ylim(0.0, 1.0)
    ax.set_xticks(x)
    ax.set_xticklabels(x_labels, fontsize=8)
    ax.set_yticks([0.0, 0.5, 1.0])
    ax.set_yticklabels(["0", "0.5", "1"], fontsize=7)
    ax.grid(True, axis="y", color="0.93")


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--in-dir", default="E4_figures/death_causes", help="Directory containing trend CSVs")
    ap.add_argument("--out-dir", default="E4_figures/death_causes", help="Where to write the slide PNG")
    ap.add_argument("--title", default="E4 child death causes (unseen conditions) — summary", help="Figure title")
    args = ap.parse_args(argv)

    in_dir = os.path.abspath(args.in_dir)
    out_dir = os.path.abspath(args.out_dir)
    os.makedirs(out_dir, exist_ok=True)

    world_csv = os.path.join(in_dir, "E4_child_death_causes_trends__by_world_init_plasticity.csv")
    food_csv = os.path.join(in_dir, "E4_child_death_causes_trends__by_food_init_plasticity.csv")
    if not os.path.isfile(world_csv) or not os.path.isfile(food_csv):
        raise SystemExit(
            "Missing trend CSVs. Run:\n"
            "  python3 plot_e4_child_death_causes_all.py --out-dir E4_figures/death_causes\n"
            "Then re-run this script."
        )

    by_world = pd.read_csv(world_csv)
    by_food = pd.read_csv(food_csv)

    # Normalize types for joins/filters
    by_world["init"] = by_world["init"].astype(str)
    by_world["plasticity"] = by_world["plasticity"].astype(str)
    by_world["world"] = by_world["world"].astype(str)

    by_food["init"] = by_food["init"].astype(str)
    by_food["plasticity"] = by_food["plasticity"].astype(str)
    by_food["food"] = pd.to_numeric(by_food["food"], errors="coerce").astype(int)

    # Figure: 3 rows (init) × 3 cols (plast) × 2 panels (world/food) inside each cell.
    fig, axes = plt.subplots(
        len(INIT_ORDER),
        len(PLAST_ORDER) * 2,
        figsize=(18.0, 8.0),
        sharey=False,
        constrained_layout=True,
    )

    for ri, init in enumerate(INIT_ORDER):
        for ci, plast in enumerate(PLAST_ORDER):
            ax_w = axes[ri, ci * 2]
            ax_f = axes[ri, ci * 2 + 1]

            subw = by_world[(by_world["init"] == init) & (by_world["plasticity"] == plast)].copy()
            subf = by_food[(by_food["init"] == init) & (by_food["plasticity"] == plast)].copy()

            _stacked_mini_bars(ax_w, subw, x_keys=list(WORLD_ORDER), x_labels=["easy", "normal", "hard"])
            _stacked_mini_bars(ax_f, subf, x_keys=list(FOOD_ORDER), x_labels=["5", "15", "25"])

            # Titles / labels
            if ri == 0:
                ax_w.set_title(f"{PLAST_TITLES.get(plast, plast)} | by world", fontsize=10)
                ax_f.set_title(f"{PLAST_TITLES.get(plast, plast)} | by food", fontsize=10)
            if ci == 0:
                ax_w.set_ylabel(INIT_LABELS.get(init, init), fontsize=10)

            # Clean up y labels on non-first columns to reduce clutter
            if ci != 0:
                ax_w.set_yticklabels([])
                ax_f.set_yticklabels([])

    # Legend (global)
    handles = [plt.Rectangle((0, 0), 1, 1, color=CAUSE_COLORS[c], label=c) for c in CAUSE_ORDER]
    fig.legend(handles=handles, loc="lower center", ncol=len(CAUSE_ORDER), frameon=False, fontsize=9)
    fig.suptitle(args.title, fontsize=14)

    out_png = os.path.join(out_dir, "E4_child_death_causes_summary_slide.png")
    fig.savefig(out_png, dpi=220, bbox_inches="tight")
    plt.close(fig)
    print("[ok] wrote", out_png)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

