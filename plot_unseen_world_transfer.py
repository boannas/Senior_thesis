#!/usr/bin/env python3
"""
plot_unseen_world_transfer.py
=============================
Compare TTD of the **same final genomes** across multiple rollout worlds:

  * base    — the training-world rollout root (e.g. FinalGenomeRollouts_normal_seen)
  * unseen  — one or more alternative-world rollout roots (e.g. _unseen_food30, _unseen_hard)

For each genome (plasticity × init × evolve_seed) we use its **mean TTD over
rollout worlds** in each root, then form

    delta_unseen = mean_TTD(unseen) - mean_TTD(base)

This script writes:
  - <out-dir>/transfer_delta_child_<unseen>.png        : box+jitter, panels=plasticity, x=init
  - <out-dir>/transfer_delta_mother_<unseen>.png       : same for mother
  - <out-dir>/transfer_forest_child_<unseen>.png       : forest of bootstrap median CI per (plasticity, init)
  - <out-dir>/transfer_forest_mother_<unseen>.png      : same for mother
  - <out-dir>/transfer_summary_<unseen>.csv            : per-(plasticity, init) summary

Inputs assume each per-genome folder already has ttd_across_seeds.csv
(produced by plot_runlog_ttd_across_seeds.py).

Usage
-----
  python3 plot_unseen_world_transfer.py \\
      --base FinalGenomeRollouts_normal_seen \\
      --unseen unseen_hard=FinalGenomeRollouts_unseen_hard \\
      --unseen unseen_food30=FinalGenomeRollouts_unseen_food30 \\
      --out-dir FinalGenomeRollouts_transfer/figures
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

from plot_finalgenome_rollout_ttd_emergence_style import INIT_ORDER, PLAST_ORDER, _load_all_rows

plt.rcParams.update(
    {
        "font.family": "serif",
        "font.serif": ["Times New Roman", "Times", "DejaVu Serif"],
        "font.size": 11,
        "axes.titlesize": 12,
        "axes.labelsize": 11,
        "axes.spines.top": False,
        "axes.spines.right": False,
        "axes.linewidth": 0.9,
        "axes.grid": True,
        "grid.color": "0.90",
        "grid.linewidth": 0.8,
        "figure.facecolor": "white",
        "axes.facecolor": "white",
        "savefig.facecolor": "white",
    }
)

INIT_COLORS = {"anti_maternal": "#c0392b", "random_uniform": "#2980b9", "pro_maternal": "#27ae60"}


def _genome_means(root: str, *, value_col: str, day_step: float) -> pd.DataFrame:
    df = _load_all_rows(root, day_step)
    if df.empty:
        raise SystemExit(f"no rows in {root}")
    agg = (
        df.groupby(["plasticity", "init", "evolve_seed"], as_index=False)[value_col]
        .mean()
        .rename(columns={value_col: "mean_ttd"})
    )
    return agg


def _join_delta(base: pd.DataFrame, other: pd.DataFrame) -> pd.DataFrame:
    m = base.merge(
        other,
        on=["plasticity", "init", "evolve_seed"],
        how="inner",
        suffixes=("_base", "_other"),
    )
    m["delta"] = m["mean_ttd_other"] - m["mean_ttd_base"]
    return m


def _bootstrap_median_ci(x: np.ndarray, *, n_boot: int, rng: np.random.Generator) -> tuple[float, float, float]:
    x = x[np.isfinite(x)]
    if len(x) == 0:
        return float("nan"), float("nan"), float("nan")
    med = float(np.median(x))
    if len(x) == 1:
        return med, med, med
    s = np.empty(n_boot, dtype=float)
    for b in range(n_boot):
        s[b] = float(np.median(rng.choice(x, size=len(x), replace=True)))
    lo, hi = np.percentile(s, [2.5, 97.5])
    return med, float(lo), float(hi)


def _plot_box_panels(merged: pd.DataFrame, *, ylabel: str, title: str, out_png: str) -> None:
    rng = np.random.default_rng(0)
    fig, axes = plt.subplots(1, 3, figsize=(13.5, 4.2), sharey=True)
    for ax, plast in zip(axes, PLAST_ORDER):
        sub = merged[merged["plasticity"] == plast]
        data = []
        for init in INIT_ORDER:
            vals = sub.loc[sub["init"] == init, "delta"].dropna().to_numpy(dtype=float)
            data.append(vals)
        if not any(len(d) for d in data):
            ax.set_visible(False)
            continue
        labels = [s.replace("_", " ") for s in INIT_ORDER]
        bp = ax.boxplot(
            data,
            labels=labels,
            patch_artist=True,
            widths=0.6,
            showmeans=True,
            medianprops=dict(color="#F28E2B", linewidth=1.6),
            meanprops=dict(marker="^", markerfacecolor="#2ca02c", markeredgecolor="#2ca02c", markersize=6),
            whiskerprops=dict(color="0.25", linewidth=0.8),
            capprops=dict(color="0.25", linewidth=0.8),
            flierprops=dict(marker="o", markerfacecolor="none", markeredgecolor="0.2", markersize=4, linestyle="none", alpha=0.18),
        )
        for patch, init in zip(bp["boxes"], INIT_ORDER):
            patch.set_facecolor(INIT_COLORS[init])
            patch.set_alpha(0.18)
            patch.set_edgecolor(INIT_COLORS[init])
        for i, vals in enumerate(data):
            if len(vals) == 0:
                continue
            x = rng.normal(i + 1, 0.05, size=len(vals))
            ax.scatter(x, vals, s=10, alpha=0.22, color=INIT_COLORS[INIT_ORDER[i]], edgecolors="none", zorder=3)
        ax.axhline(0.0, color="0.35", linewidth=1.0)
        ax.set_title(plast, fontsize=12)
    axes[0].set_ylabel(ylabel)
    fig.suptitle(title, fontsize=12, y=1.02)
    fig.tight_layout()
    fig.savefig(out_png, dpi=200, bbox_inches="tight")
    plt.close(fig)
    print("Wrote", out_png)


def _plot_forest(merged: pd.DataFrame, *, ylabel: str, title: str, out_png: str, n_boot: int) -> pd.DataFrame:
    rng = np.random.default_rng(1)
    rows = []
    for plast in PLAST_ORDER:
        sub = merged[merged["plasticity"] == plast]
        for init in INIT_ORDER:
            x = sub.loc[sub["init"] == init, "delta"].to_numpy(dtype=float)
            med, lo, hi = _bootstrap_median_ci(x, n_boot=n_boot, rng=rng)
            rows.append({"plasticity": plast, "init": init, "n": int(np.isfinite(x).sum()), "median": med, "lo": lo, "hi": hi})
    summ = pd.DataFrame(rows)
    y_labels = [f"{r['plasticity']}  {r['init'].replace('_',' ')}" for _, r in summ.iterrows()]
    y = np.arange(len(summ), dtype=float)[::-1]
    med = summ["median"].to_numpy()
    lo = summ["lo"].to_numpy()
    hi = summ["hi"].to_numpy()
    err = np.vstack([med - lo, hi - med])

    fig, ax = plt.subplots(figsize=(8.5, 5.4))
    cols = [INIT_COLORS[r["init"]] for _, r in summ.iterrows()]
    for yi, mi, ei_lo, ei_hi, c in zip(y, med, med - lo, hi - med, cols):
        ax.errorbar([mi], [yi], xerr=[[ei_lo], [ei_hi]], fmt="o", color=c, ecolor=c, alpha=0.9, capsize=4, markersize=7)
    ax.axvline(0.0, color="0.35", linewidth=1.0)
    ax.set_yticks(y)
    ax.set_yticklabels(y_labels, fontsize=10)
    ax.set_xlabel(ylabel)
    ax.set_title(title, fontsize=12)
    ax.grid(True, axis="x", alpha=0.85)
    fig.tight_layout()
    fig.savefig(out_png, dpi=200, bbox_inches="tight")
    plt.close(fig)
    print("Wrote", out_png)
    return summ


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--base", required=True, help="Training-world root (has ttd_across_seeds.csv per genome)")
    ap.add_argument(
        "--unseen",
        action="append",
        required=True,
        help="Repeatable: '<label>=<root>' (e.g. unseen_hard=FinalGenomeRollouts_unseen_hard)",
    )
    ap.add_argument("--out-dir", default=None, help="Output directory (default: <base>/figures_transfer)")
    ap.add_argument("--units", choices=["norm", "days"], default="norm")
    ap.add_argument("--day-step", type=float, default=100.0)
    ap.add_argument("--bootstrap", type=int, default=4000)
    ap.add_argument("--title-prefix", default="")
    args = ap.parse_args()

    base = os.path.abspath(args.base)
    out_dir = os.path.abspath(args.out_dir) if args.out_dir else os.path.join(base, "figures_transfer")
    os.makedirs(out_dir, exist_ok=True)

    if args.units == "days":
        child_col, mother_col = "child_days", "mother_days"
        ylab_c, ylab_m = "Δ child TTD (days)", "Δ mother TTD (days)"
    else:
        child_col, mother_col = "child_ttd_norm", "mother_ttd_norm"
        ylab_c, ylab_m = "Δ child TTD (normalized)", "Δ mother TTD (normalized)"

    base_child = _genome_means(base, value_col=child_col, day_step=args.day_step)
    base_mother = _genome_means(base, value_col=mother_col, day_step=args.day_step)
    base_child.to_csv(os.path.join(out_dir, "base_genome_means_child.csv"), index=False)
    base_mother.to_csv(os.path.join(out_dir, "base_genome_means_mother.csv"), index=False)

    prefix = (args.title_prefix + " — ") if args.title_prefix else ""

    for spec in args.unseen:
        if "=" not in spec:
            print(f"[error] bad --unseen spec (label=path): {spec}", file=sys.stderr)
            return 2
        label, path = spec.split("=", 1)
        label = label.strip()
        path = os.path.abspath(path.strip())

        oc = _genome_means(path, value_col=child_col, day_step=args.day_step)
        om = _genome_means(path, value_col=mother_col, day_step=args.day_step)

        merged_c = _join_delta(base_child, oc)
        merged_m = _join_delta(base_mother, om)
        merged_c.to_csv(os.path.join(out_dir, f"transfer_paired_child_{label}.csv"), index=False)
        merged_m.to_csv(os.path.join(out_dir, f"transfer_paired_mother_{label}.csv"), index=False)

        _plot_box_panels(
            merged_c,
            ylabel=ylab_c,
            title=f"{prefix}Δ child TTD: {label} − base (paired by evolve_seed)",
            out_png=os.path.join(out_dir, f"transfer_delta_child_{label}.png"),
        )
        _plot_box_panels(
            merged_m,
            ylabel=ylab_m,
            title=f"{prefix}Δ mother TTD: {label} − base (paired by evolve_seed)",
            out_png=os.path.join(out_dir, f"transfer_delta_mother_{label}.png"),
        )
        summ_c = _plot_forest(
            merged_c,
            ylabel=f"Bootstrap median {ylab_c} (95% CI)",
            title=f"{prefix}Forest: median Δ child TTD ({label} − base)",
            out_png=os.path.join(out_dir, f"transfer_forest_child_{label}.png"),
            n_boot=int(args.bootstrap),
        )
        summ_m = _plot_forest(
            merged_m,
            ylabel=f"Bootstrap median {ylab_m} (95% CI)",
            title=f"{prefix}Forest: median Δ mother TTD ({label} − base)",
            out_png=os.path.join(out_dir, f"transfer_forest_mother_{label}.png"),
            n_boot=int(args.bootstrap),
        )
        summ_c.to_csv(os.path.join(out_dir, f"transfer_forest_child_{label}_summary.csv"), index=False)
        summ_m.to_csv(os.path.join(out_dir, f"transfer_forest_mother_{label}_summary.csv"), index=False)

    return 0


if __name__ == "__main__":
    sys.exit(main())
