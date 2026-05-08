"""
plot_baldwin_heatmap.py
=======================
Aggregate (world x alpha_child) heatmaps from a Baldwin lineage sweep.

Reads lineage_summary.csv, then for each run loads its lineage_generations.csv
and computes asymptotic statistics over the last K generations (default K=1000).

Produces four headline heatmaps:
  H1: Asymptotic fitness  -- E2  (no plasticity baseline)
  H2: Asymptotic fitness  -- P1  (outcome_adaptive + global)
  H3: Delta fitness        -- P1 - E2  (Baldwin advantage)
  H4: Asymptotic u_drift   -- P1  (within-life plastic excursion)

By default rows=world (easy/normal/hard) and cols=alpha_child (0.3/0.5/0.7).
Cell value is averaged over (grid, food_interval, seed) for that
(plasticity, world, alpha) combination.

Also writes a tidy `heatmap_data.csv` next to the figures so the numbers
behind every cell are inspectable.

Usage (Windows PowerShell):
  python plot_baldwin_heatmap.py `
      --summary Baldwin_Experiment/lineage_summary.csv `
      --asymp-window 1000 `
      --out-dir Baldwin_Experiment/figures/heatmaps

Usage (Linux/macOS):
  python3 plot_baldwin_heatmap.py \
      --summary Baldwin_Experiment/lineage_summary.csv \
      --asymp-window 1000 \
      --out-dir Baldwin_Experiment/figures/heatmaps
"""

from __future__ import annotations

import argparse
import os
import sys
from pathlib import Path

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.colors as mcolors


WORLD_ORDER = ["easy", "normal", "hard"]
ALPHA_ORDER = [0.3, 0.5, 0.7]
PLASTICITY_ORDER = ["E2", "P1", "P2"]


def _normalize_path(p: str, base: Path) -> Path:
    """Turn a Windows-style relative path from the summary CSV into a real path."""
    p = str(p).replace("\\", "/")
    pp = Path(p)
    if pp.is_absolute():
        return pp
    return (base / pp).resolve()


def _asymptotic_stats(csv_path: Path, window: int) -> dict:
    """Mean fitness and mean u_drift over last `window` generations of one run."""
    try:
        df = pd.read_csv(csv_path)
    except Exception as e:  # noqa: BLE001
        print(f"[warn] could not read {csv_path}: {e}", file=sys.stderr)
        return {"asymp_fitness": np.nan, "asymp_drift": np.nan, "n_gens": 0}

    if "fitness" not in df.columns:
        return {"asymp_fitness": np.nan, "asymp_drift": np.nan, "n_gens": 0}

    n = len(df)
    k = min(window, n)
    tail = df.tail(k)

    fitness = pd.to_numeric(tail["fitness"], errors="coerce")
    if "mean_u_drift_end" in tail.columns:
        drift = pd.to_numeric(tail["mean_u_drift_end"], errors="coerce")
    else:
        drift = pd.Series([np.nan] * k)

    return {
        "asymp_fitness": float(fitness.mean()) if fitness.notna().any() else np.nan,
        "asymp_drift": float(drift.mean()) if drift.notna().any() else np.nan,
        "n_gens": int(n),
    }


def _build_per_run_table(summary: pd.DataFrame, base: Path, window: int) -> pd.DataFrame:
    rows = []
    for _, r in summary.iterrows():
        csv_path = _normalize_path(r["csv_path"], base)
        stats = _asymptotic_stats(csv_path, window)
        rows.append({
            "plasticity": r["plasticity"],
            "world": r["world"],
            "threats": r.get("threats", np.nan),
            "grid": r.get("grid", np.nan),
            "food_interval": r.get("food_interval", np.nan),
            "alpha_child": float(r["alpha_child"]),
            "seed": r.get("seed", np.nan),
            "asymp_fitness": stats["asymp_fitness"],
            "asymp_drift": stats["asymp_drift"],
            "n_gens": stats["n_gens"],
            "run_dir": r.get("run_dir", ""),
        })
    return pd.DataFrame(rows)


def _aggregate_world_alpha(per_run: pd.DataFrame) -> pd.DataFrame:
    """Mean over (grid, food_interval, seed) inside each (plasticity, world, alpha)."""
    g = per_run.groupby(["plasticity", "world", "alpha_child"], dropna=False)
    return g.agg(
        asymp_fitness=("asymp_fitness", "mean"),
        asymp_drift=("asymp_drift", "mean"),
        n_runs=("asymp_fitness", "count"),
        std_fitness=("asymp_fitness", "std"),
        std_drift=("asymp_drift", "std"),
    ).reset_index()


def _pivot_world_alpha(agg: pd.DataFrame, plasticity: str, value: str) -> pd.DataFrame:
    sub = agg[agg["plasticity"] == plasticity]
    pv = sub.pivot(index="world", columns="alpha_child", values=value)
    pv = pv.reindex(index=WORLD_ORDER, columns=ALPHA_ORDER)
    return pv


def _draw_heatmap(
    ax,
    matrix: pd.DataFrame,
    title: str,
    cmap: str = "viridis",
    fmt: str = "{:.3f}",
    vmin: float | None = None,
    vmax: float | None = None,
    diverging: bool = False,
    cbar_label: str = "",
):
    data = matrix.to_numpy(dtype=float)

    if diverging:
        if vmin is None or vmax is None:
            absmax = float(np.nanmax(np.abs(data))) if np.isfinite(data).any() else 1.0
            absmax = max(absmax, 1e-6)
            vmin, vmax = -absmax, absmax
        norm = mcolors.TwoSlopeNorm(vmin=vmin, vcenter=0.0, vmax=vmax)
        im = ax.imshow(data, cmap=cmap, norm=norm, aspect="auto")
    else:
        im = ax.imshow(data, cmap=cmap, vmin=vmin, vmax=vmax, aspect="auto")

    ax.set_xticks(range(matrix.shape[1]))
    ax.set_xticklabels([f"alpha={c}" for c in matrix.columns])
    ax.set_yticks(range(matrix.shape[0]))
    ax.set_yticklabels([str(r) for r in matrix.index])
    ax.set_title(title, fontsize=11)

    for i in range(matrix.shape[0]):
        for j in range(matrix.shape[1]):
            v = data[i, j]
            if np.isfinite(v):
                lum_norm = im.norm(v)
                color = "white" if lum_norm < 0.35 or lum_norm > 0.85 else "black"
                ax.text(j, i, fmt.format(v), ha="center", va="center",
                        color=color, fontsize=10)
            else:
                ax.text(j, i, "n/a", ha="center", va="center",
                        color="gray", fontsize=9)

    cbar = plt.colorbar(im, ax=ax, fraction=0.046, pad=0.04)
    if cbar_label:
        cbar.set_label(cbar_label, fontsize=9)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument("--summary", default="Baldwin_Experiment/lineage_summary.csv",
                    help="Path to lineage_summary.csv")
    ap.add_argument("--asymp-window", type=int, default=1000,
                    help="Number of trailing generations to average for asymptote")
    ap.add_argument("--out-dir", default="Baldwin_Experiment/figures/heatmaps",
                    help="Output directory for heatmaps and aggregated CSV")
    ap.add_argument("--cmap-fitness", default="viridis", help="Colormap for fitness heatmaps")
    ap.add_argument("--cmap-drift", default="magma", help="Colormap for u_drift heatmaps")
    ap.add_argument("--cmap-delta", default="RdBu_r", help="Colormap for delta-fitness heatmaps")
    args = ap.parse_args()

    summary_path = Path(args.summary).resolve()
    base = summary_path.parent

    if not summary_path.exists():
        print(f"[error] summary not found: {summary_path}", file=sys.stderr)
        sys.exit(1)

    out_dir = Path(args.out_dir).resolve()
    out_dir.mkdir(parents=True, exist_ok=True)

    print(f"[info] loading {summary_path}")
    summary = pd.read_csv(summary_path)
    print(f"[info] {len(summary)} runs in summary")

    print(f"[info] computing asymptotic stats over last {args.asymp_window} generations per run")
    per_run = _build_per_run_table(summary, base, args.asymp_window)

    per_run_path = out_dir / "per_run_asymp.csv"
    per_run.to_csv(per_run_path, index=False)
    print(f"[info] wrote {per_run_path}")

    agg = _aggregate_world_alpha(per_run)
    agg_path = out_dir / "heatmap_data.csv"
    agg.to_csv(agg_path, index=False)
    print(f"[info] wrote {agg_path}")

    available_plast = [p for p in PLASTICITY_ORDER if p in agg["plasticity"].unique()]

    fit_e2 = _pivot_world_alpha(agg, "E2", "asymp_fitness") if "E2" in available_plast else None
    fit_p1 = _pivot_world_alpha(agg, "P1", "asymp_fitness") if "P1" in available_plast else None
    fit_p2 = _pivot_world_alpha(agg, "P2", "asymp_fitness") if "P2" in available_plast else None

    drift_p1 = _pivot_world_alpha(agg, "P1", "asymp_drift") if "P1" in available_plast else None
    drift_p2 = _pivot_world_alpha(agg, "P2", "asymp_drift") if "P2" in available_plast else None

    delta_p1 = (fit_p1 - fit_e2) if (fit_p1 is not None and fit_e2 is not None) else None
    delta_p2 = (fit_p2 - fit_e2) if (fit_p2 is not None and fit_e2 is not None) else None

    if fit_e2 is not None and fit_p1 is not None:
        v_lo = float(np.nanmin([fit_e2.values, fit_p1.values]))
        v_hi = float(np.nanmax([fit_e2.values, fit_p1.values]))
    else:
        v_lo, v_hi = 0.0, 1.0

    fig, axes = plt.subplots(1, 3, figsize=(15.5, 4.5))

    if fit_e2 is not None:
        _draw_heatmap(axes[0], fit_e2,
                      title=f"E2 (no plasticity)  asymp fitness  [last {args.asymp_window} gens]",
                      cmap=args.cmap_fitness, vmin=v_lo, vmax=v_hi,
                      cbar_label="fitness")
    if fit_p1 is not None:
        _draw_heatmap(axes[1], fit_p1,
                      title=f"P1 (outcome_adaptive + global)  asymp fitness",
                      cmap=args.cmap_fitness, vmin=v_lo, vmax=v_hi,
                      cbar_label="fitness")
    if delta_p1 is not None:
        _draw_heatmap(axes[2], delta_p1,
                      title="Baldwin advantage  =  P1 - E2",
                      cmap=args.cmap_delta, diverging=True,
                      cbar_label="delta fitness (P1-E2)")
    fig.suptitle(
        f"Asymptotic fitness landscape  (mean over grid x food x seed; last {args.asymp_window} gens)",
        fontsize=12,
    )
    fig.tight_layout(rect=(0.0, 0.0, 1.0, 0.94))
    out_a = out_dir / "heatmap_fitness_E2_P1_delta.png"
    fig.savefig(out_a, dpi=180)
    plt.close(fig)
    print(f"[info] wrote {out_a}")

    if fit_p2 is not None and delta_p2 is not None:
        fig, axes = plt.subplots(1, 2, figsize=(10.5, 4.5))
        _draw_heatmap(axes[0], fit_p2,
                      title="P2 (outcome + local)  asymp fitness",
                      cmap=args.cmap_fitness, vmin=v_lo, vmax=v_hi,
                      cbar_label="fitness")
        _draw_heatmap(axes[1], delta_p2,
                      title="Baldwin advantage  =  P2 - E2",
                      cmap=args.cmap_delta, diverging=True,
                      cbar_label="delta fitness (P2-E2)")
        fig.suptitle("P2 fitness and Baldwin advantage", fontsize=12)
        fig.tight_layout(rect=(0.0, 0.0, 1.0, 0.94))
        out_b = out_dir / "heatmap_fitness_P2_delta.png"
        fig.savefig(out_b, dpi=180)
        plt.close(fig)
        print(f"[info] wrote {out_b}")

    if drift_p1 is not None or drift_p2 is not None:
        ncols = sum(x is not None for x in (drift_p1, drift_p2))
        fig, axes = plt.subplots(1, ncols, figsize=(5.2 * ncols, 4.5), squeeze=False)
        ax_iter = iter(axes[0])

        drift_vals = []
        if drift_p1 is not None:
            drift_vals.append(drift_p1.values)
        if drift_p2 is not None:
            drift_vals.append(drift_p2.values)
        d_lo = float(np.nanmin(drift_vals)) if drift_vals else 0.0
        d_hi = float(np.nanmax(drift_vals)) if drift_vals else 1.0

        if drift_p1 is not None:
            ax = next(ax_iter)
            _draw_heatmap(ax, drift_p1,
                          title="P1  asymp |u_plastic - u_fixed|  (mean within ep)",
                          cmap=args.cmap_drift, vmin=d_lo, vmax=d_hi,
                          cbar_label="u_drift")
        if drift_p2 is not None:
            ax = next(ax_iter)
            _draw_heatmap(ax, drift_p2,
                          title="P2  asymp |u_plastic - u_fixed|",
                          cmap=args.cmap_drift, vmin=d_lo, vmax=d_hi,
                          cbar_label="u_drift")
        fig.suptitle(
            f"Asymptotic plastic excursion (last {args.asymp_window} gens)", fontsize=12
        )
        fig.tight_layout(rect=(0.0, 0.0, 1.0, 0.94))
        out_c = out_dir / "heatmap_u_drift.png"
        fig.savefig(out_c, dpi=180)
        plt.close(fig)
        print(f"[info] wrote {out_c}")

    print()
    print("=" * 78)
    print("HEATMAP DATA SUMMARY")
    print("=" * 78)
    if fit_e2 is not None:
        print("\nE2 asymptotic fitness:\n", fit_e2.round(3))
    if fit_p1 is not None:
        print("\nP1 asymptotic fitness:\n", fit_p1.round(3))
    if delta_p1 is not None:
        print("\nDelta fitness (P1 - E2):\n", delta_p1.round(3))
    if drift_p1 is not None:
        print("\nP1 asymptotic u_drift:\n", drift_p1.round(3))
    if fit_p2 is not None:
        print("\nP2 asymptotic fitness:\n", fit_p2.round(3))
    if delta_p2 is not None:
        print("\nDelta fitness (P2 - E2):\n", delta_p2.round(3))
    if drift_p2 is not None:
        print("\nP2 asymptotic u_drift:\n", drift_p2.round(3))


if __name__ == "__main__":
    main()
