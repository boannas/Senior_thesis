#!/usr/bin/env python3
"""
plot_conditional_motivation_stacks.py
======================================
One-call figure for **all extended state masks** × **four motivations**
(P(sel=M | mask)), comparing **init types** within each **plasticity** panel.

Uses the same tick-level definitions as ``plot_runlog_conditional_behavior.py``
(``--extended``): child_hungry, child_cold, child_injured, mother_fear_hi,
mother_energy_lo, mother_stress_hi.

Aggregation
-----------
For each ``run_*.csv`` row in the collector, metrics are already run-level means.
We average within ``(plasticity, init, evolve_seed)``, then average across seeds,
so each genome contributes equally.

Outputs (under ``--out-dir``)
-----------------------------
  - ``motivation_stacks_by_mask.png`` — 100% stacked bars per mask × init
  - ``motivation_heatmap_by_mask.png`` — 3×3 grid: plasticity × init, cells = P(sel|mask)
  - ``motivation_by_mask_summary.csv`` — numeric table used for the plots

Either pass ``--root`` (recomputes from run logs) or ``--from-csv`` (expects
extended ``P_sel_*_given_*`` columns, e.g. from ``conditional_metrics_single.csv``).
"""

from __future__ import annotations

import argparse
import os
import sys

import numpy as np

try:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import pandas as pd
except ImportError as e:
    raise SystemExit(f"Requires pandas+matplotlib: {e}") from e

import plot_runlog_conditional_behavior as crb

plt.rcParams.update(
    {
        "font.family": "serif",
        "font.serif": ["Times New Roman", "Times", "DejaVu Serif"],
        "font.size": 10,
        "axes.titlesize": 11,
        "axes.labelsize": 10,
        "axes.spines.top": False,
        "axes.spines.right": False,
        "axes.linewidth": 0.9,
        "axes.grid": False,
        "figure.facecolor": "white",
        "axes.facecolor": "white",
        "savefig.facecolor": "white",
    }
)

# Same order as extended metrics in plot_runlog_conditional_behavior.py
CONDITION_KEYS = [
    "child_hungry",
    "child_cold",
    "child_injured",
    "mother_fear_hi",
    "mother_energy_lo",
    "mother_stress_hi",
]
CONDITION_SHORT = [
    "Child hungry",
    "Child cold",
    "Child injured",
    "Mother fear high",
    "Mother energy low",
    "Mother stress high",
]

MOT_COLORS = {
    "Forage": "#4C78A8",
    "Care": "#F58518",
    "Self": "#54A24B",
    "Protect": "#B279A2",
}


def _psel_columns() -> list[str]:
    cols = []
    for ck in CONDITION_KEYS:
        for mot in crb.MOTIVATIONS:
            cols.append(f"P_sel_{mot}_given_{ck}")
    return cols


def _load_frame(
    *,
    root: str | None,
    csv_path: str | None,
    load_kw: dict,
) -> pd.DataFrame:
    if csv_path:
        df = pd.read_csv(csv_path)
        need = _psel_columns()
        missing = [c for c in need if c not in df.columns]
        if missing:
            raise SystemExit(
                f"--from-csv missing extended columns (need --extended run). First missing: {missing[:3]} ..."
            )
        return df
    if not root:
        raise SystemExit("Provide --root or --from-csv")
    return crb._collect_root(os.path.abspath(root), **load_kw)


def _aggregate(df: pd.DataFrame) -> pd.DataFrame:
    pcols = _psel_columns()
    for c in pcols:
        df[c] = pd.to_numeric(df[c], errors="coerce")
    keep = ["plasticity", "init", "evolve_seed"] + pcols
    keep = [c for c in keep if c in df.columns]
    if "evolve_seed" not in df.columns:
        raise SystemExit("DataFrame must contain plasticity, init, evolve_seed (use --root collection).")
    per_seed = df.groupby(["plasticity", "init", "evolve_seed"], as_index=False)[pcols].mean()
    out = per_seed.groupby(["plasticity", "init"], as_index=False)[pcols].mean()
    return out


def _plot_stacked(summary: pd.DataFrame, *, title: str, out_png: str) -> None:
    n_cond = len(CONDITION_KEYS)
    x = np.arange(n_cond, dtype=float)
    n_init = len(crb.INIT_ORDER)
    group_w = 0.72
    bar_w = group_w / n_init

    fig, axes = plt.subplots(1, 3, figsize=(14.5, 4.8), sharey=True)
    for ax, plast in zip(axes, crb.PLAST_ORDER):
        sub = summary[summary["plasticity"] == plast]
        for j, init in enumerate(crb.INIT_ORDER):
            row = sub[sub["init"] == init]
            if row.empty:
                continue
            row = row.iloc[0]
            offset = (j - (n_init - 1) / 2.0) * bar_w
            xs = x + offset
            bottom = np.zeros(n_cond)
            for mot in crb.MOTIVATIONS:
                ck_list = CONDITION_KEYS
                vals = np.array(
                    [float(row.get(f"P_sel_{mot}_given_{ck}", np.nan)) for ck in ck_list],
                    dtype=float,
                )
                vals = np.nan_to_num(vals, nan=0.0)
                ax.bar(
                    xs,
                    vals,
                    width=bar_w,
                    bottom=bottom,
                    color=MOT_COLORS[mot],
                    edgecolor="white",
                    linewidth=0.4,
                    label=None,
                )
                bottom += vals
        ax.set_xticks(x)
        ax.set_xticklabels(CONDITION_SHORT, rotation=28, ha="right", fontsize=8.5)
        ax.set_title(plast, fontsize=12)
        ax.set_ylim(0, 1.0)
        ax.set_ylabel("mean P(selected motivation | mask)")
        ax.axhline(1.0, color="0.5", linewidth=0.6, linestyle=":")

    handles = [plt.Rectangle((0, 0), 1, 1, fc=MOT_COLORS[m], ec="none") for m in crb.MOTIVATIONS]
    fig.legend(
        handles,
        list(crb.MOTIVATIONS),
        loc="center left",
        bbox_to_anchor=(1.02, 0.5),
        frameon=False,
        title="Motivation",
    )
    fig.suptitle(title, fontsize=12, y=1.02)
    fig.tight_layout()
    fig.savefig(out_png, dpi=200, bbox_inches="tight")
    plt.close(fig)
    print("[ok] wrote", out_png)


def _plot_heatmaps(summary: pd.DataFrame, *, title: str, out_png: str) -> None:
    """One row per plasticity; each panel stacks three inits (4 motivation rows each)."""
    fig, axes = plt.subplots(
        len(crb.PLAST_ORDER),
        1,
        figsize=(10.5, 10.0),
        sharex=True,
        layout="constrained",
    )
    if len(crb.PLAST_ORDER) == 1:
        axes = [axes]
    ims = []
    for ax, plast in zip(axes, crb.PLAST_ORDER):
        sub = summary[summary["plasticity"] == plast]
        blocks = []
        ylabels = []
        for init in crb.INIT_ORDER:
            row = sub[sub["init"] == init]
            mat = np.full((len(crb.MOTIVATIONS), len(CONDITION_KEYS)), np.nan, dtype=float)
            if not row.empty:
                r0 = row.iloc[0]
                for mi, mot in enumerate(crb.MOTIVATIONS):
                    for ci, ck in enumerate(CONDITION_KEYS):
                        v = r0.get(f"P_sel_{mot}_given_{ck}", np.nan)
                        mat[mi, ci] = float(v) if pd.notna(v) else np.nan
            blocks.append(mat)
            tag = init.replace("_", " ").split()[0][:4]
            for mot in crb.MOTIVATIONS:
                ylabels.append(f"{tag}… {mot}")
        big = np.vstack(blocks)
        im = ax.imshow(big, aspect="auto", vmin=0, vmax=1, cmap="viridis")
        ims.append(im)
        ax.set_yticks(np.arange(big.shape[0]))
        ax.set_yticklabels(ylabels, fontsize=7)
        ax.set_title(plast, loc="left", fontsize=11)
        for y in range(1, len(crb.INIT_ORDER)):
            ax.axhline(y * len(crb.MOTIVATIONS) - 0.5, color="white", linewidth=1.2)

    axes[-1].set_xticks(np.arange(len(CONDITION_SHORT)))
    axes[-1].set_xticklabels(CONDITION_SHORT, rotation=30, ha="right", fontsize=9)
    fig.suptitle(title + "\n(rows: init × motivation; columns: state mask)", fontsize=11)
    cbar = fig.colorbar(ims[-1], ax=axes, fraction=0.04, pad=0.02, label="P(sel | mask)")
    cbar.ax.tick_params(labelsize=8)
    fig.savefig(out_png, dpi=200, bbox_inches="tight")
    plt.close(fig)
    print("[ok] wrote", out_png)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--root", default=None, help="Rollout root (same layout as plot_runlog_conditional_behavior.py).")
    ap.add_argument(
        "--from-csv",
        default=None,
        help="Use precomputed conditional_metrics_single.csv (must include extended P_sel_* columns).",
    )
    ap.add_argument(
        "--out-dir",
        default=None,
        help="Output directory (default: <root>/figures_conditional_motivation_masks or next to CSV).",
    )
    ap.add_argument("--title", default="", help="Extra title string for figures.")
    ap.add_argument("--hunger-tau", type=float, default=50.0)
    ap.add_argument("--fear-tau", type=float, default=10.0)
    ap.add_argument("--warmth-cold-tau", type=float, default=crb.DEFAULT_WARMTH_COLD_TAU)
    ap.add_argument("--injury-tau", type=float, default=crb.DEFAULT_INJURY_TAU)
    ap.add_argument("--energy-low-tau", type=float, default=crb.DEFAULT_ENERGY_LOW_TAU)
    ap.add_argument("--stress-high-tau", type=float, default=crb.DEFAULT_STRESS_HIGH_TAU)
    args = ap.parse_args()

    load_kw = dict(
        hunger_tau=args.hunger_tau,
        fear_tau=args.fear_tau,
        extended=True,
        warmth_cold_tau=args.warmth_cold_tau,
        injury_tau=args.injury_tau,
        energy_low_tau=args.energy_low_tau,
        stress_high_tau=args.stress_high_tau,
    )

    df = _load_frame(root=args.root, csv_path=args.from_csv, load_kw=load_kw)
    summary = _aggregate(df)

    if args.out_dir:
        out_dir = os.path.abspath(args.out_dir)
    elif args.root:
        out_dir = os.path.join(os.path.abspath(args.root), "figures_conditional_motivation_masks")
    else:
        out_dir = os.path.join(os.path.dirname(os.path.abspath(args.from_csv)), "figures_conditional_motivation_masks")
    os.makedirs(out_dir, exist_ok=True)

    prefix = (args.title + " — ") if args.title else ""
    base = prefix + "P(selected motivation | state mask)"

    csv_out = os.path.join(out_dir, "motivation_by_mask_summary.csv")
    summary.to_csv(csv_out, index=False)
    print("[ok] wrote", csv_out)

    _plot_stacked(summary, title=base, out_png=os.path.join(out_dir, "motivation_stacks_by_mask.png"))
    _plot_heatmaps(summary, title=base, out_png=os.path.join(out_dir, "motivation_heatmap_by_mask.png"))

    return 0


if __name__ == "__main__":
    sys.exit(main())
