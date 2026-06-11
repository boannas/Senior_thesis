"""
Plot motivation / psych weight drift for plasticity diagnostics (E0a-style).

Reads outputs from baseline_passive_lower_bound.py when --log-timeseries is on:
  passive_lower_bound_timeseries_repXXX_seedYYYY.csv

Each row includes:
  - u_drift, w_drift  (mean |plastic - fixed| over nested u / w)
  - optional u_*_fixed / u_*_plastic pairs (e.g. forage child_hunger)

Optionally reads RunLogger-style CSVs (headless_rollout_log / pygame) with columns
  m0_u_fixed_* and m0_u_plastic_* for full per-term breakdown.

Usage (after E0a):
  python plot_weight_drift.py --input-dir result_experiment/E0a_plastic_on_evolved --out result_experiment/figures/E0a_weight_drift.png

  python plot_weight_drift.py --input-dir result_experiment/E0a_plastic_on_evolved \\
      --runlog-glob "my_run*.csv" --out result_experiment/figures/
"""

from __future__ import annotations

import argparse
import glob
import os
import sys

import numpy as np

try:
    import matplotlib.pyplot as plt
    import pandas as pd
except ImportError as e:
    print("Requires pandas and matplotlib:", e, file=sys.stderr)
    sys.exit(1)


def _stack_column_series(dfs: list[pd.DataFrame], col: str, T: int) -> np.ndarray:
    mat = np.full((len(dfs), T), np.nan, dtype=float)
    for i, df in enumerate(dfs):
        if col not in df.columns:
            continue
        s = pd.to_numeric(df[col], errors="coerce").to_numpy(dtype=float)
        L = min(len(s), T)
        if L:
            mat[i, :L] = s[:L]
    return mat


def _nanmean_std(mat: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    m = np.nanmean(mat, axis=0)
    s = np.nanstd(mat, axis=0, ddof=0)
    return m, s


def _band(ax, x, mean, std, color, label):
    ax.fill_between(x, mean - std, mean + std, color=color, alpha=0.22, linewidth=0)
    ax.plot(x, mean, color=color, label=label, linewidth=1.5)


def _find_fixed_plastic_pairs(columns: list[str]) -> list[tuple[str, str, str]]:
    """Return list of (short_label, fixed_col, plastic_col) for baseline CSV naming."""
    out = []
    seen = set()
    for c in columns:
        if not c.endswith("_plastic"):
            continue
        base = c[: -len("_plastic")]
        fixed = base + "_fixed"
        if fixed not in columns:
            continue
        key = (fixed, c)
        if key in seen:
            continue
        seen.add(key)
        short = base.replace("u_", "", 1) if base.startswith("u_") else base
        out.append((short, fixed, c))
    return sorted(out, key=lambda t: t[0])


def _runlog_u_pairs(columns: list[str]) -> list[tuple[str, str, str]]:
    """Pairs m0_u_fixed_XXX <-> m0_u_plastic_XXX; label = XXX."""
    pre_f = "m0_u_fixed_"
    pre_p = "m0_u_plastic_"
    out = []
    for c in columns:
        if not c.startswith(pre_p):
            continue
        suffix = c[len(pre_p) :]
        fcol = pre_f + suffix
        if fcol not in columns:
            continue
        out.append((suffix.replace("_", " "), fcol, c))
    return sorted(out, key=lambda t: t[0])


def plot_baseline_timeseries_dir(input_dir: str, out_path: str) -> bool:
    pattern = os.path.join(input_dir, "passive_lower_bound_timeseries_*.csv")
    paths = sorted(glob.glob(pattern))
    if not paths:
        print("No baseline timeseries files match:", pattern, file=sys.stderr)
        return False

    dfs = [pd.read_csv(p) for p in paths]
    T = max(len(d) for d in dfs)
    x = np.arange(T, dtype=float)

    fig, axes = plt.subplots(2, 1, figsize=(9, 5), sharex=True)
    for ax, col, title in (
        (axes[0], "u_drift", "Motivation weights: mean |u_plastic − u_fixed| (aggregate)"),
        (axes[1], "w_drift", "Psych weights: mean |w_plastic − w_fixed| (aggregate)"),
    ):
        if col not in dfs[0].columns:
            ax.set_visible(False)
            continue
        mat = _stack_column_series(dfs, col, T)
        mean, std = _nanmean_std(mat)
        color = "steelblue" if col == "u_drift" else "darkorange"
        _band(ax, x, mean, std, color, f"{col} mean ± std (n={len(paths)})")
        ax.set_ylabel(col)
        ax.set_title(title)
        ax.grid(True, alpha=0.3)
        ax.legend(loc="upper right", fontsize=8)
    axes[-1].set_xlabel("Tick")
    fig.suptitle(f"Weight drift — {os.path.basename(input_dir)} ({len(paths)} replicates)", fontsize=11)
    fig.tight_layout()
    os.makedirs(os.path.dirname(os.path.abspath(out_path)) or ".", exist_ok=True)
    fig.savefig(out_path, dpi=150, bbox_inches="tight")
    plt.close(fig)
    print("Saved", os.path.abspath(out_path))

    # Per-term pairs available in baseline CSV (sparse)
    pairs = _find_fixed_plastic_pairs(list(dfs[0].columns))
    if len(pairs) <= 0:
        return True

    max_pairs = 12
    pairs_plot = pairs[:max_pairs]
    if len(pairs) > max_pairs:
        print(f"Note: plotting first {max_pairs} of {len(pairs)} per-term traces.", file=sys.stderr)
    ncol = min(3, len(pairs_plot))
    nrow = (len(pairs_plot) + ncol - 1) // ncol
    fig2, axes2 = plt.subplots(nrow, ncol, figsize=(3.6 * ncol, 2.8 * nrow), sharex=True)
    axes2 = np.atleast_1d(axes2).flatten()
    for ax, (short, fc, pc) in zip(axes2, pairs_plot):
        mf = _stack_column_series(dfs, fc, T)
        mp = _stack_column_series(dfs, pc, T)
        delta = mp - mf
        mean, std = _nanmean_std(delta)
        _band(ax, x, mean, std, "darkgreen", "Δ = plastic − fixed")
        ax.axhline(0, color="gray", linestyle="--", linewidth=0.8)
        ax.set_title(short[:40])
        ax.set_ylabel("Δ weight")
        ax.grid(True, alpha=0.3)
        ax.legend(loc="upper right", fontsize=7)
    for j in range(len(pairs_plot), len(axes2)):
        axes2[j].set_visible(False)
    axes2[min(len(pairs_plot) - 1, len(axes2) - 1)].set_xlabel("Tick")
    fig2.suptitle("Per-term plastic − fixed (baseline timeseries columns)", fontsize=11)
    fig2.tight_layout()
    path2 = out_path.replace(".png", "_per_term.png")
    fig2.savefig(path2, dpi=150, bbox_inches="tight")
    plt.close(fig2)
    print("Saved", os.path.abspath(path2))
    return True


def plot_runlog_glob(runlog_glob: str, out_path: str) -> bool:
    paths = sorted(glob.glob(runlog_glob))
    if not paths:
        print("No runlog files match:", runlog_glob, file=sys.stderr)
        return False
    dfs = [pd.read_csv(p) for p in paths]
    ref_cols = list(dfs[0].columns)
    pairs = _runlog_u_pairs(ref_cols)
    if not pairs:
        print("No m0_u_fixed_ / m0_u_plastic_ column pairs in first CSV.", file=sys.stderr)
        return False

    T = max(len(d) for d in dfs)
    x = np.arange(T, dtype=float)

    # Final-tick bar: mean delta at last row per file
    n_terms = len(pairs)
    means = []
    stds = []
    labels = []
    for lbl, fc, pc in pairs:
        finals = []
        for df in dfs:
            if fc not in df.columns or pc not in df.columns:
                continue
            fp = pd.to_numeric(df[fc], errors="coerce")
            pp = pd.to_numeric(df[pc], errors="coerce")
            # last finite row
            for i in range(len(df) - 1, -1, -1):
                if np.isfinite(fp.iloc[i]) and np.isfinite(pp.iloc[i]):
                    finals.append(float(pp.iloc[i] - fp.iloc[i]))
                    break
        if finals:
            means.append(float(np.mean(finals)))
            stds.append(float(np.std(finals, ddof=0)))
            labels.append(lbl[:28])

    fig, ax = plt.subplots(figsize=(max(8, 0.35 * n_terms), 4))
    xb = np.arange(len(means))
    ax.bar(xb, means, yerr=stds, capsize=3, color="steelblue", alpha=0.85, ecolor="black")
    ax.set_xticks(xb)
    ax.set_xticklabels(labels, rotation=45, ha="right", fontsize=8)
    ax.axhline(0, color="gray", linestyle="--")
    ax.set_ylabel("Mean Δ (plastic − fixed) at episode end")
    ax.set_title(f"Motivation weight drift at end — n={len(paths)} runlogs")
    ax.grid(True, axis="y", alpha=0.3)
    fig.tight_layout()
    bar_path = out_path.replace(".png", "_runlog_end_delta.png")
    fig.savefig(bar_path, dpi=150, bbox_inches="tight")
    plt.close(fig)
    print("Saved", os.path.abspath(bar_path))

    # Small multiples: first up to 8 terms delta vs time
    show = pairs[:8]
    nrow, ncol = 2, 4
    fig2, axes = plt.subplots(nrow, ncol, figsize=(14, 6), sharex=True)
    axes = axes.flatten()
    for ax, (lbl, fc, pc) in zip(axes, show):
        matf = _stack_column_series(dfs, fc, T)
        matp = _stack_column_series(dfs, pc, T)
        mean, std = _nanmean_std(matp - matf)
        _band(ax, x, mean, std, "purple", "Δ")
        ax.axhline(0, color="gray", linestyle="--", linewidth=0.7)
        ax.set_title(lbl[:22], fontsize=9)
        ax.grid(True, alpha=0.3)
    for j in range(len(show), len(axes)):
        axes[j].set_visible(False)
    axes[3].set_xlabel("Tick")
    fig2.suptitle("RunLogger: plastic − fixed over time (subset of terms)", fontsize=11)
    fig2.tight_layout()
    ts_path = out_path.replace(".png", "_runlog_delta_timeseries.png")
    fig2.savefig(ts_path, dpi=150, bbox_inches="tight")
    plt.close(fig2)
    print("Saved", os.path.abspath(ts_path))
    return True


def main():
    p = argparse.ArgumentParser(description="Plot plastic weight drift from experiment CSVs.")
    p.add_argument(
        "--input-dir",
        type=str,
        default=None,
        help="Folder with passive_lower_bound_timeseries_*.csv (baseline_passive_lower_bound output).",
    )
    p.add_argument(
        "--out",
        type=str,
        default="figures/weight_drift.png",
        help="Output PNG path (per-term figure adds _per_term.png).",
    )
    p.add_argument(
        "--runlog-glob",
        type=str,
        default=None,
        help="Glob for RunLogger CSVs, e.g. result_experiment/csvs/run_*.csv",
    )
    args = p.parse_args()

    ok = False
    if args.input_dir:
        d = os.path.abspath(args.input_dir)
        if not os.path.isdir(d):
            print("Not a directory:", d, file=sys.stderr)
            sys.exit(1)
        ok = plot_baseline_timeseries_dir(d, args.out) or ok
    if args.runlog_glob:
        ok = plot_runlog_glob(args.runlog_glob, args.out) or ok

    if not ok:
        print("Nothing plotted. Provide --input-dir and/or --runlog-glob.", file=sys.stderr)
        sys.exit(1)


if __name__ == "__main__":
    main()
