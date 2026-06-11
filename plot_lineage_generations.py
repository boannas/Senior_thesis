"""
Plot evolve-lineage CSV with raw traces + moving-average trends.

Reads ``lineage_generations.csv`` produced by ``run_evolve_lineage.py`` (same folder as
``lineage_plot.png``). Writes a **new** figure (does not overwrite ``lineage_plot.png``).

Panels:
  1) Fitness — raw (light) + centered moving average + cumulative best-so-far + accepted markers
  2) Plasticity — ``mean_u_drift_end`` raw + MA; optional ± std band from ``std_u_drift_end``
  3–6) If present in CSV (from ``run_evolve_lineage`` ≥ current version): overall / local deficit
       episode mean and end-of-episode batch means — map plasticity regime to need/stress proxies
  7+) Optional: mean child/mother TTD (norm), mean child survival — raw + MA when column exists

Re-run evolution to regenerate ``lineage_generations.csv`` with deficit columns; old CSVs skip those panels.

Usage:
  python plot_lineage_generations.py result_experiment/E3_baldwin_seed42/lineage_generations.csv
  python plot_lineage_generations.py lineage_generations.csv --ma-window 21 --out figures/lineage_ma.png
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
    print("Requires pandas and matplotlib:", e, file=sys.stderr)
    sys.exit(1)


def _rolling_mean_centered(y: np.ndarray, window: int) -> np.ndarray | None:
    """Centered moving average; same length as y. window <= 1 or invalid returns None."""
    y = np.asarray(y, dtype=float)
    n = len(y)
    if window is None or int(window) <= 1 or n == 0:
        return None
    w = min(int(window), n)
    if w % 2 == 0:
        w += 1
    half = w // 2
    out = np.empty(n, dtype=float)
    for i in range(n):
        lo, hi = max(0, i - half), min(n, i + half + 1)
        out[i] = np.nanmean(y[lo:hi])
    return out


def _finite_series(s: pd.Series) -> bool:
    return bool(s.notna().any() and np.isfinite(pd.to_numeric(s, errors="coerce")).any())


# Raw series styling (“ghost” layer behind moving averages)
_RAW_LINE_ALPHA = 0.1
_RAW_LINE_LW = 0.5
_RAW_MARKER_ALPHA = 0.09
_RAW_MARKER_SIZE = 5.0
_RAW_LINE_Z = 1
_MA_Z = 5
_MA_HALO_LW = 4.2
_MA_LINE_LW = 2.6

# MA colors: hue-shifted from raw so the trend pops (orange / teal / blue)
_MA_COLOR_FITNESS = "#e65100"
_MA_COLOR_PLASTIC = "#00838f"
_MA_COLOR_OVERALL_DEF = "#6d4c41"
_MA_COLOR_LOCAL_DEF = "#5e35b1"
_MA_COLOR_OTHER = "#283593"


def _std_col_for_mean(col: str) -> str | None:
    if not col.startswith("mean_"):
        return None
    return "std_" + col[5:]


def _raw_color_for_col(col: str) -> str:
    if "overall_deficit" in col:
        return "#5d4037"
    if "local_deficit" in col:
        return "#4a148c"
    if col == "mean_u_drift_end":
        return "darkmagenta"
    return "#37474f"


def _ma_color_for_col(col: str) -> str:
    if col == "mean_u_drift_end":
        return _MA_COLOR_PLASTIC
    if "overall_deficit" in col:
        return _MA_COLOR_OVERALL_DEF
    if "local_deficit" in col:
        return _MA_COLOR_LOCAL_DEF
    return _MA_COLOR_OTHER


def _plot_ghost_raw(ax, g, y, *, color: str, label: str) -> None:
    """Very faint line + tiny markers so dense points do not hide the MA."""
    ax.plot(
        g,
        y,
        "-",
        color=color,
        alpha=_RAW_LINE_ALPHA,
        linewidth=_RAW_LINE_LW,
        zorder=_RAW_LINE_Z,
        label=label,
    )
    ax.scatter(
        g,
        y,
        s=_RAW_MARKER_SIZE,
        c=color,
        alpha=_RAW_MARKER_ALPHA,
        edgecolors="none",
        zorder=_RAW_LINE_Z,
        rasterized=True,
    )


def _plot_ma_line(ax, g, y_ma, *, color: str, label: str) -> None:
    """Moving average on top: white halo + saturated color (readable over dot clutter)."""
    ax.plot(
        g,
        y_ma,
        "-",
        color="white",
        linewidth=_MA_HALO_LW,
        solid_capstyle="round",
        zorder=_MA_Z - 0.01,
        label="_nolegend_",
    )
    ax.plot(
        g,
        y_ma,
        "-",
        color=color,
        linewidth=_MA_LINE_LW,
        solid_capstyle="round",
        zorder=_MA_Z,
        label=label,
    )


def plot_lineage_csv(
    csv_path: str,
    *,
    out_path: str | None,
    ma_window: int,
    dpi: int,
) -> str:
    df = pd.read_csv(csv_path)
    if "generation" not in df.columns or "fitness" not in df.columns:
        raise SystemExit("CSV must contain columns: generation, fitness")

    g = df["generation"].to_numpy(dtype=float)
    fitness = pd.to_numeric(df["fitness"], errors="coerce").to_numpy(dtype=float)
    acc = df["accepted"].astype(bool).to_numpy() if "accepted" in df.columns else np.ones(len(df), dtype=bool)

    best_so_far = np.maximum.accumulate(np.nan_to_num(fitness, nan=-1e30))

    ma_f = _rolling_mean_centered(fitness, ma_window) if ma_window > 1 else None

    panels: list[tuple[str, str, str | None]] = []
    # (title, ylabel, column name or None for derived)
    panels.append(("Fitness (eval batch)", "Fitness", "__fitness__"))

    has_ud = "mean_u_drift_end" in df.columns and _finite_series(df["mean_u_drift_end"])
    if has_ud:
        panels.append(
            ("Plasticity drift at episode end", "mean |u_plastic − u_fixed|", "mean_u_drift_end")
        )

    for col, title in (
        ("mean_overall_deficit_episode", "Overall deficit (episode mean, eval batch)"),
        ("mean_overall_deficit_end", "Overall deficit (last alive tick, eval batch)"),
        ("mean_local_deficit_episode", "Local deficit (episode mean, eval batch)"),
        ("mean_local_deficit_end", "Local deficit (last alive tick, eval batch)"),
    ):
        if col in df.columns and _finite_series(df[col]):
            panels.append((title, col.replace("_", " ")[:44], col))

    for col, title in (
        ("mean_child_ttd_norm", "Mean child TTD (norm)"),
        ("mean_mother_ttd_norm", "Mean mother TTD (norm)"),
        # ("mean_child_survival", "Mean child survival (eval batch)"),
    ):
        if col in df.columns and _finite_series(df[col]):
            panels.append((title, title.split("(")[0].strip(), col))

    n = len(panels)
    fig, axes = plt.subplots(n, 1, figsize=(9, 2.9 * n), sharex=True)
    if n == 1:
        axes = np.array([axes])

    for ax, (ptitle, ylabel, col) in zip(axes, panels):
        if col == "__fitness__":
            y = fitness
            _plot_ghost_raw(ax, g, y, color="steelblue", label="fitness (raw)")
            if ma_f is not None:
                _plot_ma_line(
                    ax,
                    g,
                    ma_f,
                    color=_MA_COLOR_FITNESS,
                    label=f"fitness MA (w={ma_window})",
                )
            ax.plot(g, best_so_far, "--", color="darkgreen", linewidth=1.4, label="best fitness so far", zorder=2)
            for gi, fi, ok in zip(g, y, acc):
                if ok:
                    ax.scatter(
                        [gi],
                        [fi],
                        color="limegreen",
                        s=22,
                        alpha=0.9,
                        zorder=_MA_Z + 1,
                        edgecolors="darkgreen",
                        linewidths=0.35,
                    )
        else:
            y = pd.to_numeric(df[col], errors="coerce").to_numpy(dtype=float)
            ma_y = _rolling_mean_centered(y, ma_window) if ma_window > 1 else None
            rc = _raw_color_for_col(col)
            _plot_ghost_raw(ax, g, y, color=rc, label=f"{col} (raw)")
            if ma_y is not None:
                ma_color = _ma_color_for_col(col)
                short = "plasticity MA" if col == "mean_u_drift_end" else "MA"
                _plot_ma_line(
                    ax,
                    g,
                    ma_y,
                    color=ma_color,
                    label=f"{short} (w={ma_window})",
                )
            std_col = _std_col_for_mean(col)
            if std_col and std_col in df.columns:
                sds = pd.to_numeric(df[std_col], errors="coerce").to_numpy(dtype=float)
                if np.any(np.isfinite(sds)):
                    lo = y - sds
                    hi = y + sds
                    if col == "mean_u_drift_end":
                        lo = np.maximum(0.0, lo)
                    ax.fill_between(g, lo, hi, color=rc, alpha=0.12, linewidth=0, label="±1 std (batch)")

        ax.set_ylabel(ylabel[:48])
        ax.set_title(ptitle, fontsize=10)
        ax.grid(True, alpha=0.3)
        ax.legend(loc="best", fontsize=7, ncol=2)

    axes[-1].set_xlabel("Generation")
    fig.suptitle(os.path.basename(csv_path), fontsize=9, y=1.01)
    fig.tight_layout()

    if not out_path:
        base = os.path.dirname(os.path.abspath(csv_path)) or "."
        out_path = os.path.join(base, "lineage_plot_ma.png")
    else:
        out_dir = os.path.dirname(os.path.abspath(out_path))
        if out_dir:
            os.makedirs(out_dir, exist_ok=True)

    fig.savefig(out_path, dpi=dpi, bbox_inches="tight")
    plt.close(fig)
    return out_path


def main() -> None:
    p = argparse.ArgumentParser(description="Plot lineage_generations.csv with moving-average trends.")
    p.add_argument("csv", nargs="?", default="lineage_generations.csv", help="Path to lineage_generations.csv")
    p.add_argument(
        "--out",
        "-o",
        default=None,
        help="Output PNG path (default: <csv_dir>/lineage_plot_ma.png)",
    )
    p.add_argument(
        "--ma-window",
        type=int,
        default=15,
        help="Centered moving-average window (generations). Use 0 or 1 to plot raw + best-so-far only.",
    )
    p.add_argument("--dpi", type=int, default=150)
    args = p.parse_args()

    path = args.csv
    if not os.path.isfile(path):
        print("File not found:", path, file=sys.stderr)
        sys.exit(1)

    out = plot_lineage_csv(path, out_path=args.out, ma_window=max(0, int(args.ma_window)), dpi=int(args.dpi))
    print("Wrote", out)


if __name__ == "__main__":
    main()
