#!/usr/bin/env python3
"""
plot_emergence_summary.py
=========================
Summarize and plot emergence experiments under e.g. Emergence_results/.

Expected run folder names (same as run_evolve_lineage --output-dir):
    <plasticity>_<init>_seed<NN>
  e.g. E2_random_uniform_seed44, P1_anti_maternal_seed42, P2_pro_maternal_seed43

Walks --root recursively for lineage_generations.csv, parses the parent directory name,
computes late-trajectory means per run, writes emergence_summary.csv, and saves figures:
  - emergence_heatmap_fitness.png
  - emergence_heatmap_maternal_fraction.png
  - emergence_heatmap_child_ttd_norm.png / emergence_heatmap_mother_ttd_norm.png (if columns present)
  - emergence_heatmap_du_plastic.png / emergence_heatmap_u_drift_end.png (plasticity diagnostics; if present)
  - emergence_boxplot_fitness.png
  - emergence_boxplot_maternal_fraction.png
  - emergence_boxplot_child_ttd_norm.png / emergence_boxplot_mother_ttd_norm.png
  - emergence_boxplot_du_plastic.png
  - emergence_sample_counts.csv + console table (n runs per init × plasticity)
  - emergence_grid_fitness_ma.png   (3×3 Baldwin-style: MA per run, mean ± IQR across seeds)
  - emergence_grid_child_ttd_ma.png / emergence_grid_mother_ttd_ma.png (mean TTD per episode, smoothed)
  - emergence_grid_u_drift_ma.png   (same for mean_u_drift_end; E2 cells often empty)
  - emergence_grid_du_plastic_ma.png (mean L1 |Δu_plastic| per tick; E2 often empty)
  - emergence_grid_weight_<u_*>.png — motivation weight vs generation (subset or all ``u_`` columns;
    optional per-seed faint lines, optional Δ from gen 0)

Each run uses the same simulator settings from ``run_evolve_lineage``; different folders differ by
plasticity (E2/P1/P2), initialization (anti/random/pro maternal), and RNG seed. Many seeds = independent
replicates for the same world/config.

Distribution panels use **boxplot + jittered points** by default; pass ``--distribution-plot violin`` for
violins (all distribution panels). For small n per cell (~5 seeds), boxplots are usually easier to read;
violins can imply a smooth density that is poorly identified from a handful of replicates.

Usage:
  python3 plot_emergence_summary.py --root Emergence_results/normal
  python3 plot_emergence_summary.py --root Emergence_results --asymp-window 500 --ma-window 50 --out-dir Emergence_results/figures
  python3 plot_emergence_summary.py --root Emergence_results/normal --weight-grids all
  python3 plot_emergence_summary.py --root Emergence_results/normal --weight-show-seeds --weight-delta
  python3 plot_emergence_summary.py --root Emergence_results/normal --weight-cols u_care_bonding,u_protect_bonding

Sweep (world × α × E2/P1/P2): use ``summarize_lineage_runs.py --root Sweep_results`` then
``plot_baldwin_heatmap.py`` / ``plot_baldwin_grid.py --root Sweep_results`` (not this script).
"""

from __future__ import annotations

import argparse
import os
import re
import sys
from glob import glob

import numpy as np
import pandas as pd

try:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
except ImportError as e:
    raise SystemExit(f"matplotlib required: {e}") from e


INIT_ORDER = ["anti_maternal", "random_uniform", "pro_maternal"]
PLASTIC_ORDER = ["E2", "P1", "P2"]

# Match plot_baldwin_grid.py plasticity colors
PLAST_COLOR = {
    "E2": "#1f77b4",
    "P1": "#d62728",
    "P2": "#2ca02c",
}

# Default weight columns for ``--weight-grids subset`` (order preserved; skipped if missing in CSV).
DEFAULT_WEIGHT_GRID_COLS = [
    "u_care_bonding",
    "u_protect_bonding",
    "u_care_child_warmth",
    "u_forage_child_hunger",
    "u_self_fear",
]

RUN_DIR_RE = re.compile(
    r"^(?P<plasticity>E2|P1|P2)_(?P<init>.+)_seed(?P<seed>\d+)$"
)

WORLD_TAGS = frozenset({"easy", "normal", "hard"})


def _world_from_path(csv_path: str, root: str) -> str:
    """Infer easy/normal/hard from path under root, if present."""
    try:
        rel = os.path.relpath(os.path.dirname(csv_path), root)
    except ValueError:
        return ""
    for part in rel.split(os.sep):
        if part in WORLD_TAGS:
            return part
    return ""


def _parse_run_dir(name: str) -> dict | None:
    m = RUN_DIR_RE.match(name)
    if not m:
        return None
    return {
        "plasticity": m.group("plasticity"),
        "init": m.group("init"),
        "seed": int(m.group("seed")),
    }


def _tail_mean(series: pd.Series, window: int) -> float:
    s = pd.to_numeric(series, errors="coerce").dropna()
    if s.empty:
        return float("nan")
    k = min(int(window), len(s))
    return float(s.iloc[-k:].mean())


def _summarize_one_csv(csv_path: str, root: str, asymp_window: int) -> dict | None:
    parent = os.path.basename(os.path.dirname(csv_path))
    meta = _parse_run_dir(parent)
    if meta is None:
        return None
    try:
        df = pd.read_csv(csv_path)
    except Exception:
        return None
    if "generation" not in df.columns or "fitness" not in df.columns:
        return None
    df = df.sort_values("generation")

    row = {
        "run_dir": parent,
        "csv_path": csv_path,
        "world": _world_from_path(csv_path, root),
        "plasticity": meta["plasticity"],
        "init": meta["init"],
        "seed": meta["seed"],
        "n_generations": len(df),
        "asymp_fitness": _tail_mean(df["fitness"], asymp_window),
        "asymp_maternal_fraction": _tail_mean(df.get("maternal_fraction", pd.Series(dtype=float)), asymp_window),
        "asymp_frac_care": _tail_mean(df.get("frac_care", pd.Series(dtype=float)), asymp_window),
        "asymp_frac_protect": _tail_mean(df.get("frac_protect", pd.Series(dtype=float)), asymp_window),
        "asymp_frac_forage": _tail_mean(df.get("frac_forage", pd.Series(dtype=float)), asymp_window),
        "asymp_frac_self": _tail_mean(df.get("frac_self", pd.Series(dtype=float)), asymp_window),
        "asymp_child_ttd_norm": _tail_mean(df.get("mean_child_ttd_norm", pd.Series(dtype=float)), asymp_window),
        "asymp_mother_ttd_norm": _tail_mean(df.get("mean_mother_ttd_norm", pd.Series(dtype=float)), asymp_window),
        "asymp_u_drift_end": _tail_mean(df.get("mean_u_drift_end", pd.Series(dtype=float)), asymp_window),
        "asymp_du_plastic": _tail_mean(df.get("mean_du_plastic_episode", pd.Series(dtype=float)), asymp_window),
        "asymp_du_plastic_peak": _tail_mean(df.get("mean_peak_du_plastic_episode", pd.Series(dtype=float)), asymp_window),
        "asymp_plastic_active_frac": _tail_mean(df.get("mean_plastic_active_frac", pd.Series(dtype=float)), asymp_window),
        "asymp_lr_eff": _tail_mean(df.get("mean_lr_eff_episode", pd.Series(dtype=float)), asymp_window),
    }
    return row


def _collect_summaries(root: str, asymp_window: int) -> pd.DataFrame:
    root = os.path.abspath(root)
    pattern = os.path.join(root, "**", "lineage_generations.csv")
    csvs = sorted(glob(pattern, recursive=True))
    rows = []
    skipped = []
    for path in csvs:
        r = _summarize_one_csv(path, root, asymp_window)
        if r is None:
            skipped.append(path)
            continue
        rows.append(r)
    if not rows:
        print("[error] no valid emergence runs found.", file=sys.stderr)
        if skipped:
            print(f"  (skipped {len(skipped)} paths; check folder names match E2_init_seedNN)", file=sys.stderr)
        sys.exit(2)
    out = pd.DataFrame(rows)
    # Normalize init labels for ordering (unknown inits sort last)
    out["init"] = out["init"].astype(str)
    return out


def _pivot_mean(df: pd.DataFrame, value: str) -> pd.DataFrame:
    g = df.groupby(["init", "plasticity"], dropna=False)[value].mean().reset_index()
    pv = g.pivot(index="init", columns="plasticity", values=value)
    pv = pv.reindex(index=[i for i in INIT_ORDER if i in pv.index])
    pv = pv.reindex(columns=[p for p in PLASTIC_ORDER if p in pv.columns])
    return pv


def _plot_heatmap(pv: pd.DataFrame, title: str, cmap: str, out_path: str) -> None:
    fig, ax = plt.subplots(figsize=(5.5, 4.2))
    data = pv.to_numpy(dtype=float)
    im = ax.imshow(data, cmap=cmap, aspect="auto")
    ax.set_xticks(range(pv.shape[1]))
    ax.set_xticklabels(list(pv.columns))
    ax.set_yticks(range(pv.shape[0]))
    ax.set_yticklabels(list(pv.index))
    ax.set_xlabel("Plasticity")
    ax.set_ylabel("Initialization")
    ax.set_title(title, fontsize=11)
    for i in range(pv.shape[0]):
        for j in range(pv.shape[1]):
            v = data[i, j]
            if np.isfinite(v):
                ax.text(j, i, f"{v:.3f}", ha="center", va="center", color="black", fontsize=9)
            else:
                ax.text(j, i, "n/a", ha="center", va="center", color="gray", fontsize=9)
    plt.colorbar(im, ax=ax, fraction=0.046, pad=0.04)
    fig.tight_layout()
    fig.savefig(out_path, dpi=150)
    plt.close(fig)


def _plot_distribution_panels(
    df: pd.DataFrame,
    value: str,
    ylabel: str,
    title: str,
    out_path: str,
    *,
    kind: str = "box",
) -> None:
    """One row of panels (one per init); within each panel, E2 / P1 / P2 across seeds."""
    kind = kind.lower()
    if kind not in ("box", "violin"):
        raise ValueError(f"kind must be 'box' or 'violin', got {kind!r}")
    rng = np.random.default_rng(0)
    fig, axes = plt.subplots(1, len(INIT_ORDER), figsize=(4.2 * len(INIT_ORDER), 4.0), sharey=True)
    if len(INIT_ORDER) == 1:
        axes = [axes]
    for ax, init in zip(axes, INIT_ORDER):
        sub = df[df["init"] == init]
        data: list[np.ndarray] = []
        labels: list[str] = []
        for p in PLASTIC_ORDER:
            vals = pd.to_numeric(sub.loc[sub["plasticity"] == p, value], errors="coerce").dropna()
            data.append(vals.to_numpy())
            labels.append(p)
        if not any(len(d) > 0 for d in data):
            ax.set_visible(False)
            continue
        if kind == "box":
            # `labels` works across matplotlib versions; `tick_labels` is 3.9+
            ax.boxplot(data, labels=labels, showmeans=True)
        else:
            positions: list[int] = []
            vp_data: list[np.ndarray] = []
            for i, d in enumerate(data):
                if len(d) > 0:
                    vp_data.append(d)
                    positions.append(i + 1)
            if vp_data:
                parts = ax.violinplot(
                    vp_data,
                    positions=positions,
                    showmeans=True,
                    showmedians=True,
                    widths=0.72,
                )
                for pc in parts.get("bodies", []):
                    pc.set_alpha(0.55)
            ax.set_xticks(np.arange(1, len(labels) + 1))
            ax.set_xticklabels(labels)
        ax.set_title(init.replace("_", " "), fontsize=10)
        ax.grid(True, alpha=0.3)
        for i, p in enumerate(PLASTIC_ORDER):
            vals = pd.to_numeric(sub.loc[sub["plasticity"] == p, value], errors="coerce").dropna()
            if len(vals) == 0:
                continue
            x = rng.normal(i + 1, 0.04, size=len(vals))
            ax.scatter(x, vals, alpha=0.65, s=22, zorder=3, edgecolors="none")
    axes[0].set_ylabel(ylabel)
    fig.suptitle(title, fontsize=12, y=1.02)
    fig.tight_layout()
    fig.savefig(out_path, dpi=150, bbox_inches="tight")
    plt.close(fig)


def _plot_convergence_by_init(
    df: pd.DataFrame,
    *,
    value: str,
    ylabel: str,
    title: str,
    out_path: str,
    kind: str = "box",
    trend: str = "none",
) -> None:
    """
    Single figure: x-axis = init (anti/random/pro), within each init show E2/P1/P2 distributions.
    This is the clearest “do different inits converge?” view.
    """
    kind = kind.lower()
    if kind not in ("box", "violin"):
        raise ValueError(f"kind must be 'box' or 'violin', got {kind!r}")
    trend = str(trend).lower()
    if trend not in ("none", "median", "mean"):
        raise ValueError(f"trend must be none|median|mean, got {trend!r}")
    rng = np.random.default_rng(0)

    fig, ax = plt.subplots(figsize=(8.2, 4.4))
    base_pos = np.arange(len(INIT_ORDER), dtype=float)
    offsets = np.array([-0.25, 0.0, 0.25], dtype=float)
    width = 0.20

    # Collect data per (init, plast)
    data = {
        (init, plast): pd.to_numeric(
            df.loc[(df["init"] == init) & (df["plasticity"] == plast), value],
            errors="coerce",
        ).dropna().to_numpy()
        for init in INIT_ORDER
        for plast in PLASTIC_ORDER
    }

    plotted_any = False
    for j, plast in enumerate(PLASTIC_ORDER):
        positions = base_pos + offsets[j]
        series = [data[(init, plast)] for init in INIT_ORDER]
        if not any(len(s) for s in series):
            continue
        plotted_any = True
        color = PLAST_COLOR.get(plast, "C0")

        if kind == "box":
            bp = ax.boxplot(
                series,
                positions=positions,
                widths=width,
                patch_artist=True,
                showmeans=True,
                manage_ticks=False,
            )
            for box in bp.get("boxes", []):
                box.set_facecolor(color)
                box.set_alpha(0.30)
                box.set_edgecolor(color)
            for w in bp.get("whiskers", []):
                w.set_color(color)
                w.set_alpha(0.7)
            for cap in bp.get("caps", []):
                cap.set_color(color)
                cap.set_alpha(0.7)
            for med in bp.get("medians", []):
                med.set_color(color)
                med.set_linewidth(1.4)
            for meanline in bp.get("means", []):
                meanline.set_color(color)
                meanline.set_linewidth(1.2)
        else:
            vp_data = [s for s in series if len(s)]
            vp_pos = [positions[i] for i, s in enumerate(series) if len(s)]
            if vp_data:
                parts = ax.violinplot(
                    vp_data,
                    positions=vp_pos,
                    widths=width * 2.0,
                    showmeans=True,
                    showmedians=True,
                )
                for pc in parts.get("bodies", []):
                    pc.set_facecolor(color)
                    pc.set_edgecolor(color)
                    pc.set_alpha(0.25)

        # Jittered seed points
        for i, init in enumerate(INIT_ORDER):
            vals = data[(init, plast)]
            if len(vals) == 0:
                continue
            x = rng.normal(positions[i], 0.03, size=len(vals))
            ax.scatter(x, vals, s=18, alpha=0.70, color=color, edgecolors="none", zorder=3)

        # Median/mean trend line across init order (anti -> random -> pro)
        if trend != "none":
            stat = []
            xs = []
            for i, init in enumerate(INIT_ORDER):
                vals = data[(init, plast)]
                if len(vals) == 0:
                    stat.append(np.nan)
                else:
                    stat.append(float(np.median(vals)) if trend == "median" else float(np.mean(vals)))
                xs.append(float(positions[i]))
            stat_arr = np.array(stat, dtype=float)
            xs_arr = np.array(xs, dtype=float)
            m = np.isfinite(stat_arr)
            if np.any(m):
                ax.plot(
                    xs_arr[m],
                    stat_arr[m],
                    color=color,
                    lw=1.7,
                    alpha=0.95,
                    zorder=4,
                )
                ax.scatter(xs_arr[m], stat_arr[m], color=color, s=22, zorder=5, edgecolors="none")

    if not plotted_any:
        plt.close(fig)
        return

    ax.set_xticks(base_pos)
    ax.set_xticklabels([i.replace("_", " ") for i in INIT_ORDER])
    ax.set_xlabel("Initialization")
    ax.set_ylabel(ylabel)
    ax.set_title(title, fontsize=11)
    ax.grid(True, axis="y", alpha=0.25)

    # Legend
    handles = [
        plt.Line2D([0], [0], color=PLAST_COLOR.get(p, "C0"), lw=6, alpha=0.35, label=p)
        for p in PLASTIC_ORDER
    ]
    ax.legend(handles=handles, loc="best", framealpha=0.9, fontsize=8)

    fig.tight_layout()
    fig.savefig(out_path, dpi=160, bbox_inches="tight")
    plt.close(fig)


def _plot_convergence_split_by_plasticity(
    df: pd.DataFrame,
    *,
    value: str,
    ylabel: str,
    title: str,
    out_path: str,
    kind: str = "box",
    trend: str = "none",
) -> None:
    """
    Three panels (E2/P1/P2). In each panel: x-axis = init (anti/random/pro) with boxes + points,
    plus optional median/mean trend line across init order.
    """
    kind = kind.lower()
    if kind not in ("box", "violin"):
        raise ValueError(f"kind must be 'box' or 'violin', got {kind!r}")
    trend = str(trend).lower()
    if trend not in ("none", "median", "mean"):
        raise ValueError(f"trend must be none|median|mean, got {trend!r}")

    rng = np.random.default_rng(0)
    fig, axes = plt.subplots(1, len(PLASTIC_ORDER), figsize=(11.4, 4.2), sharey=True)
    if len(PLASTIC_ORDER) == 1:
        axes = [axes]

    for ax, plast in zip(axes, PLASTIC_ORDER):
        subp = df[df["plasticity"] == plast]
        series = [
            pd.to_numeric(subp.loc[subp["init"] == init, value], errors="coerce").dropna().to_numpy()
            for init in INIT_ORDER
        ]
        if not any(len(s) for s in series):
            ax.set_visible(False)
            continue

        color = PLAST_COLOR.get(plast, "C0")
        positions = np.arange(1, len(INIT_ORDER) + 1, dtype=float)

        if kind == "box":
            bp = ax.boxplot(
                series,
                labels=[i.replace("_", " ") for i in INIT_ORDER],
                showmeans=True,
                patch_artist=True,
            )
            for box in bp.get("boxes", []):
                box.set_facecolor(color)
                box.set_alpha(0.25)
                box.set_edgecolor(color)
            for med in bp.get("medians", []):
                med.set_color(color)
                med.set_linewidth(1.4)
            for meanline in bp.get("means", []):
                meanline.set_color(color)
                meanline.set_linewidth(1.2)
            for w in bp.get("whiskers", []):
                w.set_color(color)
                w.set_alpha(0.7)
            for cap in bp.get("caps", []):
                cap.set_color(color)
                cap.set_alpha(0.7)
        else:
            parts = ax.violinplot(series, positions=positions, showmeans=True, showmedians=True, widths=0.80)
            for pc in parts.get("bodies", []):
                pc.set_facecolor(color)
                pc.set_edgecolor(color)
                pc.set_alpha(0.22)
            ax.set_xticks(positions)
            ax.set_xticklabels([i.replace("_", " ") for i in INIT_ORDER])

        # jitter points + optional trend
        stats = []
        for i, init in enumerate(INIT_ORDER):
            vals = series[i]
            if len(vals):
                x = rng.normal(i + 1, 0.045, size=len(vals))
                ax.scatter(x, vals, s=18, alpha=0.70, color=color, edgecolors="none", zorder=3)
                if trend == "median":
                    stats.append(float(np.median(vals)))
                elif trend == "mean":
                    stats.append(float(np.mean(vals)))
                else:
                    stats.append(np.nan)
            else:
                stats.append(np.nan)

        if trend != "none":
            y = np.array(stats, dtype=float)
            m = np.isfinite(y)
            if np.any(m):
                ax.plot(positions[m], y[m], color=color, lw=1.8, zorder=4)
                ax.scatter(positions[m], y[m], color=color, s=24, zorder=5, edgecolors="none")

        ax.set_title(plast, fontsize=10)
        ax.grid(True, axis="y", alpha=0.25)

    axes[0].set_ylabel(ylabel)
    fig.suptitle(title, fontsize=12, y=1.02)
    fig.tight_layout()
    fig.savefig(out_path, dpi=160, bbox_inches="tight")
    plt.close(fig)


def _plot_convergence_trajectories(
    df_summary: pd.DataFrame,
    *,
    plasticity: str,
    ma_window: int,
    max_gen: int | None,
    dpi: int,
    out_path: str,
    title: str,
    left_col: str,
    left_ylabel: str,
    right_col: str,
    right_ylabel: str,
    show_iqr: bool = True,
    show_seed_lines: bool = True,
) -> None:
    """
    Two-panel convergence plot like the screenshot:
      - left: metric vs generation (anti/random/pro lines)
      - right: plasticity metric vs generation (anti/random/pro lines)
    For each init, we pool all seeds for the requested plasticity (E2/P1/P2),
    draw faint per-seed traces, then a bold mean (+ optional IQR band).
    """
    plast = str(plasticity)
    if plast not in PLASTIC_ORDER:
        raise ValueError(f"unknown plasticity {plast!r}")
    color_map = {
        "anti_maternal": "#d62728",   # red
        "random_uniform": "#1f77b4",  # blue
        "pro_maternal": "#2ca02c",    # green
    }
    fig, (ax0, ax1) = plt.subplots(1, 2, figsize=(11.2, 4.2), sharex=True)

    def _plot_one(ax, col: str, ylabel: str) -> tuple[float, float] | None:
        gmin, gmax = float("inf"), float("-inf")
        any_data = False
        for init in INIT_ORDER:
            sub = df_summary[(df_summary["plasticity"] == plast) & (df_summary["init"] == init)]
            paths = sub["csv_path"].tolist()
            traces: list[np.ndarray] = []
            for p in paths:
                try:
                    raw = pd.read_csv(p)
                except Exception:
                    continue
                tr = _series_smoothed(raw, col, ma_window, max_gen)
                if tr is not None and np.any(np.isfinite(tr)):
                    traces.append(tr)
            if not traces:
                continue
            any_data = True
            stacked = _stack_traces(traces)
            mean = np.nanmean(stacked, axis=0)
            x = np.arange(len(mean))
            c = color_map.get(init, "0.2")

            if show_seed_lines:
                for tr in traces:
                    ax.plot(np.arange(len(tr)), tr, color=c, alpha=0.10, lw=0.6, zorder=1)
            if show_iqr and stacked.size:
                lo = np.nanpercentile(stacked, 25, axis=0)
                hi = np.nanpercentile(stacked, 75, axis=0)
                ax.fill_between(x, lo, hi, color=c, alpha=0.18, linewidth=0, zorder=2)

            ax.plot(x, mean, color=c, lw=2.2, zorder=3, label=f"{init} (n={len(traces)})")
            fin = stacked[np.isfinite(stacked)]
            if fin.size:
                gmin = min(gmin, float(np.min(fin)))
                gmax = max(gmax, float(np.max(fin)))

        if not any_data or not np.isfinite(gmin) or not np.isfinite(gmax):
            return None
        span = gmax - gmin
        pad = max(span * 0.05, 1e-9)
        if span < 1e-12:
            pad = 0.05
        ax.set_ylim(gmin - pad, gmax + pad)
        ax.set_ylabel(ylabel)
        ax.grid(True, alpha=0.25)
        ax.legend(loc="best", fontsize=8, framealpha=0.9)
        return (gmin - pad, gmax + pad)

    ok0 = _plot_one(ax0, left_col, left_ylabel)
    ok1 = _plot_one(ax1, right_col, right_ylabel)
    if ok0 is None and ok1 is None:
        plt.close(fig)
        return

    ax0.set_title("Fitness (across seeds) — MA", fontsize=10)
    ax1.set_title("Plasticity metric (across seeds) — MA", fontsize=10)
    ax0.set_xlabel("Generation")
    ax1.set_xlabel("Generation")

    fig.suptitle(f"{title}  [{plast}]", fontsize=12, y=1.02)
    fig.tight_layout()
    fig.savefig(out_path, dpi=int(dpi), bbox_inches="tight")
    plt.close(fig)


def _write_sample_counts(df: pd.DataFrame, out_path: str) -> None:
    """Long-form CSV + console pivot: how many run folders (seeds) per condition."""
    counts = df.groupby(["init", "plasticity"], dropna=False).size().reset_index(name="n_runs")
    counts = counts.sort_values(["init", "plasticity"])
    counts.to_csv(out_path, index=False)

    pivot = (
        counts.pivot(index="init", columns="plasticity", values="n_runs")
        .fillna(0)
        .astype(int)
        .reindex(index=[i for i in INIT_ORDER if i in counts["init"].unique()])
        .reindex(columns=[p for p in PLASTIC_ORDER if p in counts["plasticity"].unique()])
    )
    print("\n[sample counts] n_runs per (init × plasticity); one row per seed / output folder:")
    print(pivot.to_string())
    print(f"\n  total runs in summary: {len(df)}")


def _smooth_1d(arr: np.ndarray, window: int) -> np.ndarray:
    if window <= 1:
        return arr.astype(float)
    return pd.Series(arr).rolling(window=window, min_periods=1).mean().to_numpy()


def _series_smoothed(
    df: pd.DataFrame,
    col: str,
    ma_window: int,
    max_gen: int | None,
    *,
    delta_from_start: bool = False,
) -> np.ndarray | None:
    df = df.sort_values("generation")
    if col not in df.columns:
        return None
    vals = pd.to_numeric(df[col], errors="coerce").to_numpy(dtype=float)
    if max_gen is not None and max_gen >= 0:
        vals = vals[: int(max_gen) + 1]
    if vals.size == 0 or not np.any(np.isfinite(vals)):
        return None
    out = _smooth_1d(vals, ma_window)
    if delta_from_start and out.size and np.isfinite(out[0]):
        out = out.astype(float) - float(out[0])
    return out


def _u_weight_columns_from_csv(csv_path: str) -> list[str]:
    """Column names logged as flattened motivation weights (``u_forage_...``, etc.)."""
    try:
        header = pd.read_csv(csv_path, nrows=0)
    except Exception:
        return []
    return [c for c in header.columns if c.startswith("u_")]


def _pick_sample_csv_for_weights(df_summary: pd.DataFrame) -> str | None:
    if df_summary.empty or "csv_path" not in df_summary.columns:
        return None
    for p in df_summary["csv_path"].tolist():
        if p and os.path.isfile(p):
            return p
    return None


def _resolve_weight_grid_columns(
    df_summary: pd.DataFrame,
    mode: str,
    explicit: str | None,
) -> list[str]:
    if mode == "none" and not (explicit and explicit.strip()):
        return []
    sample = _pick_sample_csv_for_weights(df_summary)
    if not sample:
        return []
    discovered = _u_weight_columns_from_csv(sample)
    if not discovered:
        return []
    if explicit:
        want = [x.strip() for x in explicit.split(",") if x.strip()]
        bad = [c for c in want if c not in discovered]
        if bad:
            raise ValueError(f"unknown weight column(s) {bad!r}; not in lineage CSV header.")
        return want
    if mode == "all":
        return discovered
    # subset
    return [c for c in DEFAULT_WEIGHT_GRID_COLS if c in discovered]


def _weight_col_pretty_label(col: str) -> str:
    return col.replace("u_", "").replace("_", " ")


def _stack_traces(traces: list[np.ndarray]) -> np.ndarray:
    if not traces:
        return np.empty((0, 0))
    max_len = max(len(t) for t in traces)
    out = np.full((len(traces), max_len), np.nan, dtype=float)
    for i, t in enumerate(traces):
        out[i, : len(t)] = t
    return out


def _plot_lineage_grid_ma(
    df_summary: pd.DataFrame,
    *,
    col: str,
    ylabel: str,
    suptitle: str,
    out_path: str,
    ma_window: int,
    max_gen: int | None,
    dpi: int,
    per_seed_lines: bool = False,
    delta_from_start: bool = False,
) -> None:
    """
    Rows = init (anti / random / pro), cols = E2 / P1 / P2.
    Each panel: per-seed series smoothed with MA, then mean line + IQR band across seeds.
    All panels share one y-axis range (min/max over mean and IQR bands + small padding).
    Optional: faint per-seed trajectories; optional Δ from generation 0 after smoothing.
    """
    n_r, n_c = len(INIT_ORDER), len(PLASTIC_ORDER)
    fig, axes = plt.subplots(
        n_r,
        n_c,
        figsize=(4.2 * n_c, 3.2 * n_r),
        sharex=True,
        squeeze=False,
    )

    global_y_min = float("inf")
    global_y_max = float("-inf")

    for i, init in enumerate(INIT_ORDER):
        for j, plast in enumerate(PLASTIC_ORDER):
            ax = axes[i][j]
            sub = df_summary[(df_summary["init"] == init) & (df_summary["plasticity"] == plast)]
            paths = sub["csv_path"].tolist()
            traces: list[np.ndarray] = []
            for path in paths:
                try:
                    raw = pd.read_csv(path)
                except Exception:
                    continue
                tr = _series_smoothed(
                    raw, col, ma_window, max_gen, delta_from_start=delta_from_start
                )
                if tr is not None and np.any(np.isfinite(tr)):
                    traces.append(tr)

            color = PLAST_COLOR.get(plast, "C0")
            title = f"{plast}\n{init}"

            if not traces:
                ax.text(0.5, 0.5, "no data", ha="center", va="center", transform=ax.transAxes, fontsize=9, color="gray")
                ax.set_title(title, fontsize=8)
                ax.grid(True, alpha=0.25)
                continue

            stacked = _stack_traces(traces)
            mean = np.nanmean(stacked, axis=0)
            lo = np.nanpercentile(stacked, 25, axis=0)
            hi = np.nanpercentile(stacked, 75, axis=0)
            if per_seed_lines:
                fin = stacked[np.isfinite(stacked)]
            else:
                band = np.concatenate([mean, lo, hi])
                fin = band[np.isfinite(band)]
            if fin.size:
                global_y_min = min(global_y_min, float(np.min(fin)))
                global_y_max = max(global_y_max, float(np.max(fin)))
            x = np.arange(len(mean))
            if per_seed_lines:
                for tr in traces:
                    ax.plot(np.arange(len(tr)), tr, color=color, alpha=0.22, lw=0.85, zorder=1)
            ax.fill_between(x, lo, hi, alpha=0.22, color=color, linewidth=0, zorder=2)
            ax.plot(x, mean, color=color, lw=2.0, zorder=3, label=f"n={len(traces)}")
            ax.set_title(title, fontsize=8)
            ax.grid(True, alpha=0.3)
            if j == 0:
                ax.set_ylabel(ylabel, fontsize=9)
            if i == n_r - 1:
                ax.set_xlabel("Generation", fontsize=9)

    if np.isfinite(global_y_min) and np.isfinite(global_y_max):
        span = global_y_max - global_y_min
        pad = max(span * 0.05, 1e-9)
        if span < 1e-12:
            pad = 0.05
        y0, y1 = global_y_min - pad, global_y_max + pad
        for i in range(n_r):
            for j in range(n_c):
                axes[i][j].set_ylim(y0, y1)

    fig.suptitle(suptitle, fontsize=11, y=1.01)
    fig.tight_layout()
    fig.savefig(out_path, dpi=dpi, bbox_inches="tight")
    plt.close(fig)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--root", default="Emergence_results/normal", help="Directory tree containing run folders")
    ap.add_argument(
        "--out-dir",
        default=None,
        help="Figures + CSV output directory (default: <root>/emergence_figures)",
    )
    ap.add_argument(
        "--asymp-window",
        type=int,
        default=200,
        help="Trailing generations to average for asymptotic metrics (default 200)",
    )
    ap.add_argument(
        "--world",
        default=None,
        choices=["easy", "normal", "hard"],
        help="If set, keep only runs whose path contains this world folder (e.g. normal).",
    )
    ap.add_argument(
        "--ma-window",
        type=int,
        default=50,
        help="Moving-average window (generations) for lineage grid plots (default 50).",
    )
    ap.add_argument(
        "--max-gen",
        type=int,
        default=None,
        help="Clip each run to generations 0..max_gen before smoothing (optional).",
    )
    ap.add_argument(
        "--no-lineage-grids",
        action="store_true",
        help="Skip Baldwin-style lineage grids (fitness, TTD, drift, and weight trajectories).",
    )
    ap.add_argument(
        "--weight-grids",
        choices=["none", "subset", "all"],
        default="subset",
        help="Plot motivation weight vs generation (3×3: init × E2/P1/P2). "
        "subset = key u_* columns; all = every u_* in CSV (many files).",
    )
    ap.add_argument(
        "--weight-cols",
        default=None,
        metavar="COLS",
        help="Comma-separated u_* columns; overrides --weight-grids (e.g. u_care_bonding,u_protect_bonding).",
    )
    ap.add_argument(
        "--weight-show-seeds",
        action="store_true",
        help="On weight grids: faint line per seed behind mean+IQR.",
    )
    ap.add_argument(
        "--weight-delta",
        action="store_true",
        help="On weight grids: y = smoothed weight minus smoothed gen-0 value (easier compare random init).",
    )
    ap.add_argument("--dpi", type=int, default=150, help="PNG resolution for figures.")
    ap.add_argument("--no-plots", action="store_true", help="Only write CSV")
    ap.add_argument(
        "--distribution-plot",
        choices=["box", "violin"],
        default="box",
        help="Boxplot (default) or violin for fitness / maternal / TTD distribution panels. "
        "With few seeds per cell, box + jitter is usually clearer than violin.",
    )
    ap.add_argument(
        "--convergence-trend",
        choices=["none", "median", "mean"],
        default="none",
        help="For convergence-by-init plots: overlay a line connecting per-init medians (recommended) or means.",
    )
    ap.add_argument(
        "--convergence-layout",
        choices=["grouped", "split_plasticity"],
        default="grouped",
        help="Convergence plots layout: grouped = all E2/P1/P2 on one axis; split_plasticity = 3 panels (E2,P1,P2).",
    )
    args = ap.parse_args()

    root = os.path.abspath(args.root)
    if not os.path.isdir(root):
        print(f"[error] not a directory: {root}", file=sys.stderr)
        return 2

    out_dir = os.path.abspath(args.out_dir) if args.out_dir else os.path.join(root, "emergence_figures")
    os.makedirs(out_dir, exist_ok=True)

    df = _collect_summaries(root, args.asymp_window)
    if args.world:
        df = df[df["world"] == args.world]
        if df.empty:
            print(f"[error] no runs left after --world {args.world}", file=sys.stderr)
            return 2
    csv_out = os.path.join(out_dir, "emergence_summary.csv")
    df.to_csv(csv_out, index=False)
    print(f"[ok] wrote {csv_out}  (n_runs={len(df)})")

    counts_path = os.path.join(out_dir, "emergence_sample_counts.csv")
    _write_sample_counts(df, counts_path)
    print(f"     {counts_path}")

    if args.no_plots:
        return 0

    dist_kind = str(args.distribution_plot)

    # Only include inits that appear in data for heatmaps
    pv_fit = _pivot_mean(df, "asymp_fitness")
    if pv_fit.shape[0] > 0 and pv_fit.shape[1] > 0:
        _plot_heatmap(
            pv_fit,
            f"Asymptotic fitness (mean over seeds, last {args.asymp_window} gens)",
            "viridis",
            os.path.join(out_dir, "emergence_heatmap_fitness.png"),
        )
        print(f"     {os.path.join(out_dir, 'emergence_heatmap_fitness.png')}")

    pv_mat = _pivot_mean(df, "asymp_maternal_fraction")
    if pv_mat.shape[0] > 0 and pv_mat.shape[1] > 0 and pv_mat.notna().any().any():
        _plot_heatmap(
            pv_mat,
            f"Maternal fraction Care+Protect (mean over seeds, last {args.asymp_window} gens)",
            "magma",
            os.path.join(out_dir, "emergence_heatmap_maternal_fraction.png"),
        )
        print(f"     {os.path.join(out_dir, 'emergence_heatmap_maternal_fraction.png')}")

    if df.get("asymp_child_ttd_norm") is not None and df["asymp_child_ttd_norm"].notna().any():
        pv_c = _pivot_mean(df, "asymp_child_ttd_norm")
        if pv_c.shape[0] > 0 and pv_c.shape[1] > 0:
            _plot_heatmap(
                pv_c,
                f"Child TTD norm — mean over seeds (last {args.asymp_window} gens)",
                "cividis",
                os.path.join(out_dir, "emergence_heatmap_child_ttd_norm.png"),
            )
            print(f"     {os.path.join(out_dir, 'emergence_heatmap_child_ttd_norm.png')}")

    if df.get("asymp_mother_ttd_norm") is not None and df["asymp_mother_ttd_norm"].notna().any():
        pv_m = _pivot_mean(df, "asymp_mother_ttd_norm")
        if pv_m.shape[0] > 0 and pv_m.shape[1] > 0:
            _plot_heatmap(
                pv_m,
                f"Mother TTD norm — mean over seeds (last {args.asymp_window} gens)",
                "cividis",
                os.path.join(out_dir, "emergence_heatmap_mother_ttd_norm.png"),
            )
            print(f"     {os.path.join(out_dir, 'emergence_heatmap_mother_ttd_norm.png')}")

    if df.get("asymp_du_plastic") is not None and df["asymp_du_plastic"].notna().any():
        pv_du = _pivot_mean(df, "asymp_du_plastic")
        if pv_du.shape[0] > 0 and pv_du.shape[1] > 0 and pv_du.notna().any().any():
            _plot_heatmap(
                pv_du,
                f"du_plastic — mean L1 |Δu_plastic| per tick (last {args.asymp_window} gens)",
                "magma",
                os.path.join(out_dir, "emergence_heatmap_du_plastic.png"),
            )
            print(f"     {os.path.join(out_dir, 'emergence_heatmap_du_plastic.png')}")

    if df.get("asymp_u_drift_end") is not None and df["asymp_u_drift_end"].notna().any():
        pv_ud = _pivot_mean(df, "asymp_u_drift_end")
        if pv_ud.shape[0] > 0 and pv_ud.shape[1] > 0 and pv_ud.notna().any().any():
            _plot_heatmap(
                pv_ud,
                f"u_drift — mean |u_plastic − u_fixed| (last {args.asymp_window} gens)",
                "magma",
                os.path.join(out_dir, "emergence_heatmap_u_drift_end.png"),
            )
            print(f"     {os.path.join(out_dir, 'emergence_heatmap_u_drift_end.png')}")

    _plot_distribution_panels(
        df,
        "asymp_fitness",
        "Asymptotic fitness",
        f"Fitness by init × plasticity (last {args.asymp_window} gens; points = seeds)",
        os.path.join(out_dir, "emergence_boxplot_fitness.png"),
        kind=dist_kind,
    )
    print(f"     {os.path.join(out_dir, 'emergence_boxplot_fitness.png')}")

    if df["asymp_maternal_fraction"].notna().any():
        _plot_distribution_panels(
            df,
            "asymp_maternal_fraction",
            "Maternal fraction",
            f"Maternal fraction by init × plasticity (last {args.asymp_window} gens)",
            os.path.join(out_dir, "emergence_boxplot_maternal_fraction.png"),
            kind=dist_kind,
        )
        print(f"     {os.path.join(out_dir, 'emergence_boxplot_maternal_fraction.png')}")

    if df.get("asymp_child_ttd_norm") is not None and df["asymp_child_ttd_norm"].notna().any():
        _plot_distribution_panels(
            df,
            "asymp_child_ttd_norm",
            "Child TTD (normalized)",
            f"Child TTD by init × plasticity (last {args.asymp_window} gens; points = seeds)",
            os.path.join(out_dir, "emergence_boxplot_child_ttd_norm.png"),
            kind=dist_kind,
        )
        print(f"     {os.path.join(out_dir, 'emergence_boxplot_child_ttd_norm.png')}")

    if df.get("asymp_mother_ttd_norm") is not None and df["asymp_mother_ttd_norm"].notna().any():
        _plot_distribution_panels(
            df,
            "asymp_mother_ttd_norm",
            "Mother TTD (normalized)",
            f"Mother TTD by init × plasticity (last {args.asymp_window} gens; points = seeds)",
            os.path.join(out_dir, "emergence_boxplot_mother_ttd_norm.png"),
            kind=dist_kind,
        )
        print(f"     {os.path.join(out_dir, 'emergence_boxplot_mother_ttd_norm.png')}")

    if df.get("asymp_du_plastic") is not None and df["asymp_du_plastic"].notna().any():
        _plot_distribution_panels(
            df,
            "asymp_du_plastic",
            "du_plastic (mean L1 |Δu| per tick)",
            f"du_plastic by init × plasticity (last {args.asymp_window} gens; points = seeds)",
            os.path.join(out_dir, "emergence_boxplot_du_plastic.png"),
            kind=dist_kind,
        )
        print(f"     {os.path.join(out_dir, 'emergence_boxplot_du_plastic.png')}")

    # Convergence summaries: swap axes so init is on x.
    conv_trend = str(args.convergence_trend)
    conv_layout = str(args.convergence_layout)
    conv_fn = _plot_convergence_by_init if conv_layout == "grouped" else _plot_convergence_split_by_plasticity

    conv_fit = os.path.join(out_dir, "emergence_convergence_fitness.png")
    conv_fn(
        df,
        value="asymp_fitness",
        ylabel="Asymptotic fitness",
        title=f"Convergence across initializations (last {args.asymp_window} gens; points = seeds)",
        out_path=conv_fit,
        kind=dist_kind,
        trend=conv_trend,
    )
    print(f"     {conv_fit}")

    if df.get("asymp_child_ttd_norm") is not None and df["asymp_child_ttd_norm"].notna().any():
        conv_cttd = os.path.join(out_dir, "emergence_convergence_child_ttd_norm.png")
        conv_fn(
            df,
            value="asymp_child_ttd_norm",
            ylabel="Asymptotic child TTD (normalized)",
            title=f"Child TTD convergence across initializations (last {args.asymp_window} gens; points = seeds)",
            out_path=conv_cttd,
            kind=dist_kind,
            trend=conv_trend,
        )
        print(f"     {conv_cttd}")

    if df.get("asymp_mother_ttd_norm") is not None and df["asymp_mother_ttd_norm"].notna().any():
        conv_mttd = os.path.join(out_dir, "emergence_convergence_mother_ttd_norm.png")
        conv_fn(
            df,
            value="asymp_mother_ttd_norm",
            ylabel="Asymptotic mother TTD (normalized)",
            title=f"Mother TTD convergence across initializations (last {args.asymp_window} gens; points = seeds)",
            out_path=conv_mttd,
            kind=dist_kind,
            trend=conv_trend,
        )
        print(f"     {conv_mttd}")

    if df.get("asymp_u_drift_end") is not None and df["asymp_u_drift_end"].notna().any():
        conv_ud = os.path.join(out_dir, "emergence_convergence_u_drift_end.png")
        conv_fn(
            df,
            value="asymp_u_drift_end",
            ylabel="Asymptotic u_drift_end (mean |u_plas−u_fix|)",
            title=f"Convergence of plasticity drift (last {args.asymp_window} gens; points = seeds)",
            out_path=conv_ud,
            kind=dist_kind,
            trend=conv_trend,
        )
        print(f"     {conv_ud}")

    if df.get("asymp_du_plastic") is not None and df["asymp_du_plastic"].notna().any():
        conv_du = os.path.join(out_dir, "emergence_convergence_du_plastic.png")
        conv_fn(
            df,
            value="asymp_du_plastic",
            ylabel="Asymptotic du_plastic (mean L1 |Δu_plastic| per tick)",
            title=f"Convergence of plasticity activity (last {args.asymp_window} gens; points = seeds)",
            out_path=conv_du,
            kind=dist_kind,
            trend=conv_trend,
        )
        print(f"     {conv_du}")

    if not args.no_lineage_grids:
        ma = max(1, int(args.ma_window))
        max_g = args.max_gen
        fit_path = os.path.join(out_dir, "emergence_grid_fitness_ma.png")
        _plot_lineage_grid_ma(
            df,
            col="fitness",
            ylabel="Fitness (MA)",
            suptitle=f"Fitness vs generation (per-run MA w={ma}; band = IQR across seeds)",
            out_path=fit_path,
            ma_window=ma,
            max_gen=max_g,
            dpi=int(args.dpi),
        )
        print(f"     {fit_path}")

        drift_path = os.path.join(out_dir, "emergence_grid_u_drift_ma.png")
        _plot_lineage_grid_ma(
            df,
            col="mean_u_drift_end",
            ylabel="Mean |u_plas−u_fix| (MA)",
            suptitle=f"Plasticity drift end-of-episode (per-run MA w={ma}; E2 often empty)",
            out_path=drift_path,
            ma_window=ma,
            max_gen=max_g,
            dpi=int(args.dpi),
        )
        print(f"     {drift_path}")

        du_path = os.path.join(out_dir, "emergence_grid_du_plastic_ma.png")
        _plot_lineage_grid_ma(
            df,
            col="mean_du_plastic_episode",
            ylabel="Mean L1 |Δu_plastic| per tick (MA)",
            suptitle=f"Plasticity activity du_plastic (per-run MA w={ma}; E2 often empty)",
            out_path=du_path,
            ma_window=ma,
            max_gen=max_g,
            dpi=int(args.dpi),
        )
        print(f"     {du_path}")

        # Convergence trajectories: compare anti/random/pro within each plasticity (E2/P1/P2).
        for plast in PLASTIC_ORDER:
            # Fitness + u_drift
            p_out = os.path.join(out_dir, f"emergence_convergence_traj_{plast}_fitness_u_drift.png")
            _plot_convergence_trajectories(
                df,
                plasticity=plast,
                ma_window=ma,
                max_gen=max_g,
                dpi=int(args.dpi),
                out_path=p_out,
                title=f"Convergence trajectories across initializations (MA w={ma})",
                left_col="fitness",
                left_ylabel="Fitness",
                right_col="mean_u_drift_end",
                right_ylabel="Mean |u_plastic − u_fixed| (end)",
                show_iqr=True,
                show_seed_lines=True,
            )
            print(f"     {p_out}")

            # Fitness + du_plastic
            p_out2 = os.path.join(out_dir, f"emergence_convergence_traj_{plast}_fitness_du_plastic.png")
            _plot_convergence_trajectories(
                df,
                plasticity=plast,
                ma_window=ma,
                max_gen=max_g,
                dpi=int(args.dpi),
                out_path=p_out2,
                title=f"Convergence trajectories across initializations (MA w={ma})",
                left_col="fitness",
                left_ylabel="Fitness",
                right_col="mean_du_plastic_episode",
                right_ylabel="Mean L1 |Δu_plastic| per tick",
                show_iqr=True,
                show_seed_lines=True,
            )
            print(f"     {p_out2}")

        cttd_path = os.path.join(out_dir, "emergence_grid_child_ttd_ma.png")
        _plot_lineage_grid_ma(
            df,
            col="mean_child_ttd_norm",
            ylabel="Mean child TTD (norm, MA)",
            suptitle=f"Child TTD vs generation (per-run MA w={ma}; band = IQR across seeds)",
            out_path=cttd_path,
            ma_window=ma,
            max_gen=max_g,
            dpi=int(args.dpi),
        )
        print(f"     {cttd_path}")

        mttd_path = os.path.join(out_dir, "emergence_grid_mother_ttd_ma.png")
        _plot_lineage_grid_ma(
            df,
            col="mean_mother_ttd_norm",
            ylabel="Mean mother TTD (norm, MA)",
            suptitle=f"Mother TTD vs generation (per-run MA w={ma}; band = IQR across seeds)",
            out_path=mttd_path,
            ma_window=ma,
            max_gen=max_g,
            dpi=int(args.dpi),
        )
        print(f"     {mttd_path}")

        try:
            wcols = _resolve_weight_grid_columns(df, args.weight_grids, args.weight_cols)
        except ValueError as e:
            print(f"[error] {e}", file=sys.stderr)
            return 2
        if wcols:
            for wcol in wcols:
                label = _weight_col_pretty_label(wcol)
                y_part = f"{label} (MA"
                if args.weight_delta:
                    y_part += ", Δ from g0"
                y_part += ")"
                sub = "Δ from gen 0; " if args.weight_delta else ""
                wpath = os.path.join(out_dir, f"emergence_grid_weight_{wcol}_ma.png")
                _plot_lineage_grid_ma(
                    df,
                    col=wcol,
                    ylabel=y_part,
                    suptitle=f"{label} vs generation ({sub}per-run MA w={ma}; band = IQR across seeds)",
                    out_path=wpath,
                    ma_window=ma,
                    max_gen=max_g,
                    dpi=int(args.dpi),
                    per_seed_lines=bool(args.weight_show_seeds),
                    delta_from_start=bool(args.weight_delta),
                )
                print(f"     {wpath}")
        elif args.weight_grids != "none" or (args.weight_cols and args.weight_cols.strip()):
            print(
                "[warn] no weight columns plotted (missing lineage CSV or no u_* columns).",
                file=sys.stderr,
            )

    # Quick console table: mean fitness by init × plasticity
    print("\n[preview] mean asymptotic fitness (init × plasticity):")
    print(_pivot_mean(df, "asymp_fitness").to_string())
    return 0


if __name__ == "__main__":
    sys.exit(main())
