#!/usr/bin/env python3
"""
plot_rollout_ttd_emergence_full_set.py
=====================================
From a root folder that contains per-run ``ttd_across_seeds.csv`` files
(same layout as ``plot_finalgenome_rollout_ttd_emergence_style.py``), write a
**full figure set** for thesis-style emergence / convergence claims:

  1) Child + mother TTD boxplots (split_plasticity and split_init layouts)
  2) Paired ΔTTD vs random_uniform, matched by evolve_seed (child + mother)
  3) Forest plot: median Δ with bootstrap 95% CI
  4) Bland–Altman panels (anti vs random, pro vs random), per plasticity

Genome-level pairing: for each (plasticity, evolve_seed, init) we take the
**mean** TTD across rollout seeds in ``ttd_across_seeds.csv``, then require all
three inits present for that (plasticity, evolve_seed) before forming deltas.

Usage
-----
  python3 plot_rollout_ttd_emergence_full_set.py --root FinalGenomeRollouts_normal

  python3 plot_rollout_ttd_emergence_full_set.py \\
      --root FinalGenomeRollouts_normal \\
      --out-dir FinalGenomeRollouts_normal/figures_thesis_set \\
      --equivalence-band 0.02
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

from plot_finalgenome_rollout_ttd_emergence_style import (
    INIT_ORDER,
    PLAST_ORDER,
    _emergence_plot,
    _load_all_rows,
)

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

COLORS = {"anti": "#4C78A8", "pro": "#54A24B", "ref": "0.35"}


def _aggregate_genome_means(df: pd.DataFrame, value_col: str) -> pd.DataFrame:
    return (
        df.groupby(["plasticity", "init", "evolve_seed"], as_index=False)[value_col]
        .mean()
        .rename(columns={value_col: "mean_ttd"})
    )


def _paired_wide(agg: pd.DataFrame) -> pd.DataFrame:
    wide = agg.pivot(index=["plasticity", "evolve_seed"], columns="init", values="mean_ttd")
    for c in INIT_ORDER:
        if c not in wide.columns:
            wide[c] = np.nan
    wide = wide.reset_index()
    mask = wide[INIT_ORDER].notna().all(axis=1)
    wide = wide.loc[mask].copy()
    wide["delta_anti"] = wide["anti_maternal"] - wide["random_uniform"]
    wide["delta_pro"] = wide["pro_maternal"] - wide["random_uniform"]
    return wide


def _bootstrap_median_ci(x: np.ndarray, *, n_boot: int, rng: np.random.Generator) -> tuple[float, float, float]:
    x = x[np.isfinite(x)]
    if len(x) == 0:
        return float("nan"), float("nan"), float("nan")
    med = float(np.median(x))
    if len(x) == 1:
        return med, med, med
    stats = np.empty(n_boot, dtype=float)
    for b in range(n_boot):
        samp = rng.choice(x, size=len(x), replace=True)
        stats[b] = float(np.median(samp))
    lo, hi = np.percentile(stats, [2.5, 97.5])
    return med, float(lo), float(hi)


def _plot_paired_deltas(
    wide: pd.DataFrame,
    *,
    value_name: str,
    out_path: str,
    equivalence_band: float | None,
    title_prefix: str,
) -> None:
    rng = np.random.default_rng(0)
    fig, axes = plt.subplots(1, 3, figsize=(13.5, 4.2), sharey=True)
    labels = ["Δ anti − random", "Δ pro − random"]
    for ax, plast in zip(axes, PLAST_ORDER):
        sub = wide[wide["plasticity"] == plast]
        d1 = sub["delta_anti"].to_numpy(dtype=float)
        d2 = sub["delta_pro"].to_numpy(dtype=float)
        data = [d1[np.isfinite(d1)], d2[np.isfinite(d2)]]
        bp = ax.boxplot(
            data,
            labels=labels,
            patch_artist=True,
            widths=0.55,
            showmeans=True,
            medianprops=dict(color="#F28E2B", linewidth=1.8),
            meanprops=dict(marker="^", markerfacecolor="#2ca02c", markeredgecolor="#2ca02c", markersize=6),
        )
        colors = [COLORS["anti"], COLORS["pro"]]
        for patch, c in zip(bp["boxes"], colors):
            patch.set_facecolor(c)
            patch.set_alpha(0.2)
            patch.set_edgecolor(c)
        for i, vals in enumerate(data):
            if len(vals) == 0:
                continue
            x = rng.normal(i + 1, 0.05, size=len(vals))
            ax.scatter(x, vals, s=12, alpha=0.25, color=colors[i], edgecolors="none", zorder=3)
        ax.axhline(0.0, color=COLORS["ref"], linewidth=1.0, zorder=1)
        if equivalence_band is not None and equivalence_band > 0:
            ax.axhspan(-equivalence_band, equivalence_band, color="0.75", alpha=0.25, zorder=0)
        ax.set_title(plast, fontsize=12)
        ax.grid(True, axis="y", alpha=0.85)
        ax.grid(False, axis="x")
    axes[0].set_ylabel(f"Paired Δ {value_name} (genome mean over rollouts)")
    prefix = (title_prefix + " — ") if title_prefix else ""
    cnts = wide.groupby("plasticity").size().reindex(PLAST_ORDER).fillna(0).astype(int).to_dict()
    cnt_str = ", ".join(f"{p}:{cnts[p]}" for p in PLAST_ORDER)
    fig.suptitle(f"{prefix}Paired Δ vs random init (matched evolve_seed; n per plasticity: {cnt_str})", fontsize=11, y=1.02)
    fig.tight_layout()
    fig.savefig(out_path, dpi=200, bbox_inches="tight")
    plt.close(fig)
    print("Wrote", out_path)


def _plot_forest(
    wide: pd.DataFrame,
    *,
    value_name: str,
    out_path: str,
    n_boot: int,
    title_prefix: str,
) -> None:
    rng = np.random.default_rng(1)
    rows = []
    for plast in PLAST_ORDER:
        sub = wide[wide["plasticity"] == plast]
        for key, lab in [("delta_anti", "anti − random"), ("delta_pro", "pro − random")]:
            x = sub[key].to_numpy(dtype=float)
            med, lo, hi = _bootstrap_median_ci(x, n_boot=n_boot, rng=rng)
            rows.append({"plasticity": plast, "contrast": lab, "median": med, "lo": lo, "hi": hi, "n": int(np.isfinite(x).sum())})

    summ = pd.DataFrame(rows)
    y_labels = [f"{r['plasticity']}  {r['contrast']}" for _, r in summ.iterrows()]
    y = np.arange(len(summ), dtype=float)[::-1]
    med = summ["median"].to_numpy()
    lo = summ["lo"].to_numpy()
    hi = summ["hi"].to_numpy()
    err = np.vstack([med - lo, hi - med])

    fig, ax = plt.subplots(figsize=(7.5, 4.8))
    ax.errorbar(med, y, xerr=err, fmt="o", color="0.2", ecolor="0.35", capsize=4, markersize=7)
    ax.axvline(0.0, color=COLORS["ref"], linewidth=1.0)
    ax.set_yticks(y)
    ax.set_yticklabels(y_labels, fontsize=10)
    ax.set_xlabel(f"Bootstrap median of paired Δ {value_name} (95% CI)")
    prefix = (title_prefix + " — ") if title_prefix else ""
    ax.set_title(f"{prefix}Forest: median paired Δ vs random (genome-level)", fontsize=12)
    ax.grid(True, axis="x", alpha=0.85)
    fig.tight_layout()
    fig.savefig(out_path, dpi=200, bbox_inches="tight")
    plt.close(fig)
    print("Wrote", out_path)
    return summ


def _plot_bland_altman(
    wide: pd.DataFrame,
    *,
    compare_init: str,
    value_name: str,
    out_path: str,
    title_prefix: str,
) -> None:
    """
    compare_init: 'anti_maternal' or 'pro_maternal' vs random_uniform on same row.
    x = 0.5*(other + random), y = other - random
    """
    other = compare_init
    fig, axes = plt.subplots(1, 3, figsize=(13.5, 4.0), sharey=True)
    for ax, plast in zip(axes, PLAST_ORDER):
        sub = wide[wide["plasticity"] == plast]
        a = sub[other].to_numpy(dtype=float)
        r = sub["random_uniform"].to_numpy(dtype=float)
        x = 0.5 * (a + r)
        y = a - r
        m = np.isfinite(x) & np.isfinite(y)
        x, y = x[m], y[m]
        ax.scatter(x, y, s=22, alpha=0.45, edgecolors="0.25", linewidths=0.4, facecolors="#4C78A8")
        ax.axhline(0.0, color=COLORS["ref"], linewidth=1.0)
        mean_d = float(np.mean(y)) if len(y) else float("nan")
        std_d = float(np.std(y, ddof=1)) if len(y) > 1 else float("nan")
        if len(y) > 1 and np.isfinite(std_d) and std_d > 0:
            ax.axhline(mean_d + 1.96 * std_d, color="#c62828", linewidth=0.9, linestyle="--", alpha=0.85)
            ax.axhline(mean_d - 1.96 * std_d, color="#c62828", linewidth=0.9, linestyle="--", alpha=0.85)
        ax.set_title(f"{plast} (n={len(y)})", fontsize=12)
        ax.grid(True, alpha=0.85)
    axes[1].set_xlabel(f"Mean of paired genome means ({value_name})")
    axes[0].set_ylabel(f"Difference ({other.replace('_', ' ')} − random)")
    prefix = (title_prefix + " — ") if title_prefix else ""
    lab = other.replace("_", " ")
    fig.suptitle(f"{prefix}Bland–Altman: {lab} vs random (genome-level)", fontsize=12, y=1.02)
    fig.tight_layout()
    fig.savefig(out_path, dpi=200, bbox_inches="tight")
    plt.close(fig)
    print("Wrote", out_path)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--root", required=True, help="Root with <E2|P1|P2>_*_seed*/ttd_across_seeds.csv")
    ap.add_argument("--out-dir", default=None, help="Output directory (default: <root>/figures_thesis_set)")
    ap.add_argument("--day-step", type=float, default=100.0)
    ap.add_argument("--units", choices=["norm", "days"], default="norm", help="TTD scale for all figures")
    ap.add_argument("--equivalence-band", type=float, default=None, help="Shade ±band on paired Δ plots (e.g. 0.02)")
    ap.add_argument("--bootstrap", type=int, default=4000, help="Bootstrap resamples for forest CIs")
    ap.add_argument("--title-prefix", default="", help="Optional prefix for figure titles")
    args = ap.parse_args()

    root = os.path.abspath(args.root)
    if not os.path.isdir(root):
        print(f"[error] not a directory: {root}", file=sys.stderr)
        return 2

    out_dir = os.path.abspath(args.out_dir) if args.out_dir else os.path.join(root, "figures_thesis_set")
    os.makedirs(out_dir, exist_ok=True)

    df = _load_all_rows(root, args.day_step)
    if df.empty:
        print("[error] no rows after load", file=sys.stderr)
        return 2

    if args.units == "days":
        child_col, mother_col = "child_days", "mother_days"
        ylab = "TTD (days)"
    else:
        child_col, mother_col = "child_ttd_norm", "mother_ttd_norm"
        ylab = "TTD (normalized 0–1)"

    prefix = args.title_prefix

    # --- 1) Overview boxplots (both layouts) ---
    for layout in ("split_plasticity", "split_init"):
        suf = layout
        _emergence_plot(
            df,
            child_col,
            ylab,
            f"{prefix + ' — ' if prefix else ''}Child TTD by init × plasticity (final genomes; headless rollouts)",
            os.path.join(out_dir, f"01_child_ttd_boxplot_{suf}.png"),
            layout=layout,
        )
        _emergence_plot(
            df,
            mother_col,
            ylab,
            f"{prefix + ' — ' if prefix else ''}Mother TTD by init × plasticity (final genomes; headless rollouts)",
            os.path.join(out_dir, f"02_mother_ttd_boxplot_{suf}.png"),
            layout=layout,
        )

    # --- Paired tables ---
    agg_c = _aggregate_genome_means(df, child_col)
    agg_m = _aggregate_genome_means(df, mother_col)
    wide_c = _paired_wide(agg_c)
    wide_m = _paired_wide(agg_m)

    wide_c.to_csv(os.path.join(out_dir, "03_paired_genome_means_child.csv"), index=False)
    wide_m.to_csv(os.path.join(out_dir, "03_paired_genome_means_mother.csv"), index=False)

    band = args.equivalence_band
    vn = "child TTD" if args.units == "norm" else "child TTD (days)"
    vm = "mother TTD" if args.units == "norm" else "mother TTD (days)"

    _plot_paired_deltas(wide_c, value_name=vn, out_path=os.path.join(out_dir, "04_paired_delta_child.png"), equivalence_band=band, title_prefix=prefix)
    _plot_paired_deltas(wide_m, value_name=vm, out_path=os.path.join(out_dir, "05_paired_delta_mother.png"), equivalence_band=band, title_prefix=prefix)

    forest_c = _plot_forest(
        wide_c,
        value_name=vn,
        out_path=os.path.join(out_dir, "06_forest_median_delta_child.png"),
        n_boot=int(args.bootstrap),
        title_prefix=prefix,
    )
    forest_m = _plot_forest(
        wide_m,
        value_name=vm,
        out_path=os.path.join(out_dir, "07_forest_median_delta_mother.png"),
        n_boot=int(args.bootstrap),
        title_prefix=prefix,
    )
    forest_c.to_csv(os.path.join(out_dir, "06_forest_median_delta_child_summary.csv"), index=False)
    forest_m.to_csv(os.path.join(out_dir, "07_forest_median_delta_mother_summary.csv"), index=False)

    _plot_bland_altman(
        wide_c,
        compare_init="anti_maternal",
        value_name=vn,
        out_path=os.path.join(out_dir, "08_bland_altman_child_anti_vs_random.png"),
        title_prefix=prefix,
    )
    _plot_bland_altman(
        wide_c,
        compare_init="pro_maternal",
        value_name=vn,
        out_path=os.path.join(out_dir, "09_bland_altman_child_pro_vs_random.png"),
        title_prefix=prefix,
    )
    _plot_bland_altman(
        wide_m,
        compare_init="anti_maternal",
        value_name=vm,
        out_path=os.path.join(out_dir, "10_bland_altman_mother_anti_vs_random.png"),
        title_prefix=prefix,
    )
    _plot_bland_altman(
        wide_m,
        compare_init="pro_maternal",
        value_name=vm,
        out_path=os.path.join(out_dir, "11_bland_altman_mother_pro_vs_random.png"),
        title_prefix=prefix,
    )

    # small readme of n paired
    with open(os.path.join(out_dir, "00_README.txt"), "w", encoding="utf-8") as f:
        f.write("Generated by plot_rollout_ttd_emergence_full_set.py\n\n")
        f.write(f"Paired rows (child): {len(wide_c)}  (one per plasticity×evolve_seed with all 3 inits)\n")
        f.write(f"Paired rows (mother): {len(wide_m)}\n")
        f.write(f"Per-plasticity counts (child): {wide_c.groupby('plasticity').size().to_dict()}\n")
        f.write("\nFiles:\n")
        f.write("  01-02: overview boxplots (child/mother × layout)\n")
        f.write("  03: paired genome-level CSVs\n")
        f.write("  04-05: paired Δ vs random (box + jitter)\n")
        f.write("  06-07: forest median Δ + bootstrap CI\n")
        f.write("  08-11: Bland–Altman vs random\n")

    print(f"[ok] all figures under {out_dir}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
