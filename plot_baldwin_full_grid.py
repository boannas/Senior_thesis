"""
plot_baldwin_full_grid.py — visualize EVERY (world, alpha, grid, food) cell separately.

Produces a multi-panel image (no aggregation across ecology):
  rows = (world, alpha_child)        e.g. easy α=0.3 ... hard α=0.7   (9 levels)
  cols = (grid_size, food_interval)  e.g. g10 i10 ... g20 i20         (9 levels)
  each panel = ONE exact ecology cell, with up to 3 plasticity colors (E2 / P1 / P2)
  if multiple seeds exist, every seed is drawn as its own pair of traces.

When --ma-window > 1 (default 50): faint raw generation-wise values are drawn
behind a thicker moving-average overlay (same colors, higher z-order).

Useful when you want to inspect *every* run (`--split-by ...` to break the
output into smaller per-world or per-(world,alpha) figures).

Usage:
  # one big 9x9 image (default)
  python plot_baldwin_full_grid.py

  # one figure per world  (3 images, each 3 rows x 9 cols)
  python plot_baldwin_full_grid.py --split-by world

  # one figure per (world, alpha) (9 images, each 3 rows x 3 cols)
  python plot_baldwin_full_grid.py --split-by world,alpha

  # plasticity drift instead of fitness
  python plot_baldwin_full_grid.py --metric u_drift --split-by world

  # same y-scale on every panel (default); optional fixed bounds e.g. survival [0,1]
  python plot_baldwin_full_grid.py --ymin 0 --ymax 1 --metric child_survival --split-by world

  # per-panel autoscale (old behavior)
  python plot_baldwin_full_grid.py --no-unified-y

  # MA only (no faint raw background)
  python plot_baldwin_full_grid.py --no-ghost-raw

  # Fitness (left, solid) overlaid with plasticity drift (right, dashed)
  python plot_baldwin_full_grid.py --overlay --split-by world
"""

from __future__ import annotations

import argparse
import os
import re
import sys
from glob import glob
from itertools import product

import numpy as np
import pandas as pd

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

# ---------- folder/tag conventions (must match summarize_lineage_runs.py) ----------
PLASTIC_TAG = {
    "Evolved_results": "E2",
    "Plastic_1_results": "P1",
    "Plastic_2_results": "P2",
}
WORLD_TAG = {"easy", "normal", "hard"}

WORLD_ORDER = ["easy", "normal", "hard"]
ALPHA_ORDER = [0.3, 0.5, 0.7]
GRID_ORDER = [10, 15, 20]
FOOD_ORDER = [10, 15, 20]
PLAST_ORDER = ["E2", "P1", "P2"]

PLAST_LABEL = {
    "E2": "E2 (genes only)",
    "P1": "P1 (out_adapt + global)",
    "P2": "P2 (out + local)",
}
PLAST_COLOR = {
    "E2": "#1f77b4",
    "P1": "#d62728",
    "P2": "#2ca02c",
}

# Defaults for panel sizes and line widths (overridable via CLI)
_DEFAULT_PANEL_W = 3.0
_DEFAULT_PANEL_H = 2.4
_DEFAULT_MA_LW = 1.3
_DEFAULT_RAW_LW = 0.4
_OVERLAY_LW_SCALE = 0.85   # secondary metric is a touch thinner than primary

# Raw trace “ghost” styling (behind MA)
_RAW_LINE_ALPHA = 0.11
_RAW_Z = 1
_MA_ALPHA = 0.92
_MA_Z = 4
# Overlay (twin axis) styling for the secondary metric
_OV_LINESTYLE = (0, (4, 2))   # dashed
_OV_RAW_ALPHA = 0.10
_OV_MA_ALPHA = 0.92
_OV_RIGHT_AXIS_COLOR = "0.30"

RUN_NAME_RE = re.compile(
    r"thr(?P<thr>\d+)_g(?P<g>\d+)_i(?P<i>\d+)_a(?P<alpha>[0-9]+p[0-9]+)_seed(?P<seed>\d+)"
)

METRIC_INFO = {
    "fitness": {"col": "fitness", "ylabel": "Fitness"},
    "u_drift": {"col": "mean_u_drift_end", "ylabel": "Mean |u_plastic − u_fixed| (end)"},
    "u_drift_peak": {"col": "mean_peak_u_drift_episode", "ylabel": "Peak |u_plastic − u_fixed| (within ep)"},
    "du_plastic": {"col": "mean_du_plastic_episode", "ylabel": "Mean L1 |Δu_plastic| per tick"},
    "du_plastic_peak": {"col": "mean_peak_du_plastic_episode", "ylabel": "Peak L1 |Δu_plastic| per tick"},
    "plastic_active_frac": {"col": "mean_plastic_active_frac", "ylabel": "Plastic-active fraction of ticks"},
    "lr_eff": {"col": "mean_lr_eff_episode", "ylabel": "Mean effective learning rate"},
    "child_survival": {"col": "mean_child_survival", "ylabel": "Mean child survival"},
    "child_ttd": {"col": "mean_child_ttd_norm", "ylabel": "Child TTD (norm)"},
    "mother_ttd": {"col": "mean_mother_ttd_norm", "ylabel": "Mother TTD (norm)"},
}


def _parse_alpha_str(s: str) -> float:
    return float(s.replace("p", "."))


def _detect_plasticity_and_world(csv_path: str, root: str) -> tuple[str | None, str | None]:
    rel = os.path.relpath(csv_path, root)
    parts = rel.split(os.sep)
    plasticity: str | None = None
    world: str | None = None
    for p in parts:
        if p in PLASTIC_TAG and plasticity is None:
            plasticity = PLASTIC_TAG[p]
        if p in WORLD_TAG and world is None:
            world = p
    return plasticity, world


def _smooth(arr: np.ndarray, window: int) -> np.ndarray:
    if window <= 1:
        return arr.astype(float)
    return pd.Series(arr).rolling(window=window, min_periods=1).mean().to_numpy()


def _alpha_to_str(a: float) -> str:
    return f"{a:.1f}".replace(".", "p")


def _raw_series(
    csv_path: str,
    metric_col: str,
    max_gen: int | None,
) -> np.ndarray | None:
    """Load metric column as float array (optionally clipped); no smoothing."""
    try:
        vals = pd.to_numeric(
            pd.read_csv(csv_path).get(metric_col, pd.Series(dtype=float)),
            errors="coerce",
        ).to_numpy()
    except Exception:
        return None
    if vals.size == 0 or not np.any(np.isfinite(vals)):
        return None
    if max_gen is not None:
        vals = vals[: int(max_gen) + 1]
    return vals.astype(float)


def _clip_lower(arr: np.ndarray, min_gen: int | None) -> np.ndarray:
    if min_gen is None or min_gen <= 0:
        return arr
    if min_gen >= arr.size:
        return arr[arr.size:]  # empty
    return arr[min_gen:]


def _compute_unified_ylim(
    cell_runs: dict[tuple[tuple, tuple], list[dict]],
    rows_keys: list[tuple],
    cols_keys: list[tuple],
    metric_col: str,
    ma_window: int,
    min_gen: int | None,
    max_gen: int | None,
    margin: float,
    ghost_raw: bool,
) -> tuple[float, float] | None:
    """Min/max over traces shown in this figure: raw (if ghost) + MA (+ relative margin)."""
    chunks: list[np.ndarray] = []
    for rk in rows_keys:
        for ck in cols_keys:
            for r in cell_runs.get((rk, ck), []):
                raw_full = _raw_series(r["csv_path"], metric_col, max_gen)
                if raw_full is None or raw_full.size == 0:
                    continue
                ma_full = raw_full if ma_window <= 1 else _smooth(raw_full, ma_window)
                raw = _clip_lower(raw_full, min_gen)
                ma = _clip_lower(ma_full, min_gen)
                if ghost_raw and ma_window > 1 and raw.size:
                    chunks.append(raw[np.isfinite(raw)])
                if ma.size:
                    chunks.append(ma[np.isfinite(ma)])
    if not chunks:
        return None
    flat = np.concatenate(chunks)
    if flat.size == 0:
        return None
    ymin, ymax = float(np.min(flat)), float(np.max(flat))
    if ymax <= ymin:
        ymax = ymin + 1e-9
    pad = margin * (ymax - ymin)
    return ymin - pad, ymax + pad


# ---------- single-figure plotter --------------------------------------------------

def _plot_image(
    runs: list[dict],
    *,
    rows_keys: list[tuple],   # list of (world, alpha)
    cols_keys: list[tuple],   # list of (grid, food)
    metric_col: str,
    ylabel: str,
    ma_window: int,
    min_gen: int | None,
    max_gen: int | None,
    out_path: str,
    title_suffix: str,
    dpi: int,
    share_y: bool,
    unified_y: bool,
    y_margin: float,
    y_min: float | None,
    y_max: float | None,
    ghost_raw: bool,
    overlay_metric_col: str | None = None,
    overlay_ylabel: str | None = None,
    panel_w: float = _DEFAULT_PANEL_W,
    panel_h: float = _DEFAULT_PANEL_H,
    ma_lw: float = _DEFAULT_MA_LW,
    raw_lw: float = _DEFAULT_RAW_LW,
    same_yscale: bool = False,
) -> bool:
    nr = len(rows_keys)
    nc = len(cols_keys)
    if nr == 0 or nc == 0:
        return False

    # group runs into the (row, col) cells
    cell_runs: dict[tuple[tuple, tuple], list[dict]] = {}
    for r in runs:
        rk = (r["world"], r["alpha"])
        ck = (r["grid"], r["food"])
        if rk in rows_keys and ck in cols_keys:
            cell_runs.setdefault((rk, ck), []).append(r)

    if not cell_runs:
        return False

    ylim: tuple[float, float] | None = None
    if y_min is not None and y_max is not None:
        ylim = (float(y_min), float(y_max))
    elif unified_y:
        ylim = _compute_unified_ylim(
            cell_runs,
            rows_keys,
            cols_keys,
            metric_col,
            ma_window,
            min_gen,
            max_gen,
            y_margin,
            ghost_raw,
        )

    ylim_overlay: tuple[float, float] | None = None
    if overlay_metric_col is not None and unified_y:
        ylim_overlay = _compute_unified_ylim(
            cell_runs,
            rows_keys,
            cols_keys,
            overlay_metric_col,
            ma_window,
            min_gen,
            max_gen,
            y_margin,
            ghost_raw,
        )

    if same_yscale and overlay_metric_col is not None:
        # Force one shared range across both metrics (left + right axes).
        if y_min is not None and y_max is not None:
            shared = (float(y_min), float(y_max))
        else:
            lo_vals: list[float] = []
            hi_vals: list[float] = []
            if ylim is not None:
                lo_vals.append(ylim[0]); hi_vals.append(ylim[1])
            if ylim_overlay is not None:
                lo_vals.append(ylim_overlay[0]); hi_vals.append(ylim_overlay[1])
            if lo_vals and hi_vals:
                shared = (float(min(lo_vals)), float(max(hi_vals)))
            else:
                shared = None  # type: ignore[assignment]
        if shared is not None:
            ylim = shared
            ylim_overlay = shared

    fig, axes = plt.subplots(
        nrows=nr,
        ncols=nc,
        figsize=(max(1.6, panel_w) * nc, max(1.4, panel_h) * nr),
        sharex=True,
        sharey=share_y,
        squeeze=False,
    )

    ov_ma_lw = ma_lw * _OVERLAY_LW_SCALE
    ov_raw_lw = raw_lw  # keep raw thin/uniform across both metrics

    # for legend collection
    seen_plast: set[str] = set()
    show_ghost = ghost_raw and ma_window > 1
    overlay = overlay_metric_col is not None
    right_axes: list = []

    for ri, rk in enumerate(rows_keys):
        for ci, ck in enumerate(cols_keys):
            ax = axes[ri, ci]
            ax_r = ax.twinx() if overlay else None
            if ax_r is not None:
                right_axes.append((ri, ci, ax_r))
            cell = cell_runs.get((rk, ck), [])
            for r in cell:
                raw_full = _raw_series(r["csv_path"], metric_col, max_gen)
                if raw_full is None:
                    continue
                ma_full = raw_full if ma_window <= 1 else _smooth(raw_full, ma_window)
                raw = _clip_lower(raw_full, min_gen)
                ma = _clip_lower(ma_full, min_gen)
                if ma.size == 0:
                    continue
                lo = int(min_gen) if (min_gen is not None and min_gen > 0) else 0
                gens = np.arange(lo, lo + ma.size)
                color = PLAST_COLOR[r["plast"]]
                if show_ghost:
                    ax.plot(
                        gens,
                        raw,
                        color=color,
                        linewidth=raw_lw,
                        alpha=_RAW_LINE_ALPHA,
                        solid_capstyle="round",
                        zorder=_RAW_Z,
                    )
                ax.plot(
                    gens,
                    ma,
                    color=color,
                    linewidth=ma_lw,
                    alpha=_MA_ALPHA,
                    solid_capstyle="round",
                    zorder=_MA_Z,
                )
                seen_plast.add(r["plast"])

                if overlay:
                    raw_ov_full = _raw_series(r["csv_path"], overlay_metric_col, max_gen)
                    if raw_ov_full is None:
                        continue
                    ma_ov_full = raw_ov_full if ma_window <= 1 else _smooth(raw_ov_full, ma_window)
                    raw_ov = _clip_lower(raw_ov_full, min_gen)
                    ma_ov = _clip_lower(ma_ov_full, min_gen)
                    if ma_ov.size == 0:
                        continue
                    lo_ov = int(min_gen) if (min_gen is not None and min_gen > 0) else 0
                    gens_ov = np.arange(lo_ov, lo_ov + ma_ov.size)
                    if show_ghost:
                        ax_r.plot(
                            gens_ov,
                            raw_ov,
                            color=color,
                            linewidth=ov_raw_lw,
                            alpha=_OV_RAW_ALPHA,
                            linestyle=_OV_LINESTYLE,
                            zorder=_RAW_Z,
                        )
                    ax_r.plot(
                        gens_ov,
                        ma_ov,
                        color=color,
                        linewidth=ov_ma_lw,
                        alpha=_OV_MA_ALPHA,
                        linestyle=_OV_LINESTYLE,
                        zorder=_MA_Z,
                    )

            ax.grid(True, alpha=0.2)
            ax.tick_params(axis="both", labelsize=7)
            if ax_r is not None:
                ax_r.tick_params(axis="y", labelsize=7, colors=_OV_RIGHT_AXIS_COLOR)
                for sp in ("top", "right"):
                    ax_r.spines[sp].set_alpha(0.4)

            # column header on top row
            if ri == 0:
                g, f = ck
                ax.set_title(f"g={g}, i={f}", fontsize=8)
            # row label on left col
            if ci == 0:
                w, a = rk
                ax.set_ylabel(f"{w}\nα={a}\n{ylabel}", fontsize=7)
            else:
                ax.set_ylabel("")
            # right-side axis label only on rightmost column to reduce clutter
            if ax_r is not None and ci == nc - 1 and overlay_ylabel:
                ax_r.set_ylabel(overlay_ylabel, fontsize=7, color=_OV_RIGHT_AXIS_COLOR)
            elif ax_r is not None:
                ax_r.set_ylabel("")
            if ri == nr - 1:
                ax.set_xlabel("Generation", fontsize=8)

    if ylim is not None:
        ymin, ymax = ylim
        for ax in axes.flat:
            ax.set_ylim(ymin, ymax)
    if overlay and ylim_overlay is not None:
        ov_min, ov_max = ylim_overlay
        for _ri, _ci, ax_r in right_axes:
            ax_r.set_ylim(ov_min, ov_max)

    # one legend at top of figure
    handles: list = []
    for p in PLAST_ORDER:
        if p not in seen_plast:
            continue
        handles.append(
            plt.Line2D(
                [0],
                [0],
                color=PLAST_COLOR[p],
                linewidth=ma_lw,
                label=f"{PLAST_LABEL[p]} — {ylabel} (MA)",
            )
        )
    if overlay and overlay_ylabel:
        for p in PLAST_ORDER:
            if p not in seen_plast:
                continue
            handles.append(
                plt.Line2D(
                    [0],
                    [0],
                    color=PLAST_COLOR[p],
                    linewidth=ov_ma_lw,
                    linestyle=_OV_LINESTYLE,
                    label=f"{PLAST_LABEL[p]} — {overlay_ylabel} (MA)",
                )
            )
    if show_ghost:
        handles.insert(
            0,
            plt.Line2D(
                [0],
                [0],
                color="0.45",
                linewidth=max(raw_lw, 0.6),
                alpha=0.75,
                label="Raw (faint)",
            ),
        )
    if handles:
        ncol = min(len(handles), 4 if overlay else 5)
        fig.legend(handles=handles, loc="upper center", ncol=ncol, fontsize=7.5, frameon=False)

    scale_note = f"  [shared L y: [{ylim[0]:.4g}, {ylim[1]:.4g}]]" if ylim else ""
    if overlay and ylim_overlay is not None:
        scale_note += f"  [shared R y: [{ylim_overlay[0]:.4g}, {ylim_overlay[1]:.4g}]]"
    ma_note = f"MA={ma_window}" if ma_window > 1 else "no MA (window=1)"
    ghost_note = " + raw ghost" if show_ghost else ""
    overlay_note = f"  +  {overlay_ylabel} (dashed, right axis)" if overlay and overlay_ylabel else ""
    fig.suptitle(
        f"All conditions — {ylabel}{overlay_note}  ({ma_note}{ghost_note}){scale_note}{title_suffix}",
        fontsize=10,
        y=0.995,
    )
    fig.tight_layout(rect=(0, 0, 1, 0.955))

    parent = os.path.dirname(out_path)
    if parent:
        os.makedirs(parent, exist_ok=True)
    fig.savefig(out_path, dpi=dpi)
    plt.close(fig)
    print(f"[ok] wrote {out_path}  panels={nr}x{nc}")
    return True


# ---------- runs loader -----------------------------------------------------------

def _load_runs(root: str) -> list[dict]:
    csvs = sorted(glob(os.path.join(root, "**", "lineage_generations.csv"), recursive=True))
    runs: list[dict] = []
    for csv_path in csvs:
        plasticity, world = _detect_plasticity_and_world(csv_path, root)
        run_dir = os.path.basename(os.path.dirname(csv_path))
        m = RUN_NAME_RE.search(run_dir)
        if plasticity is None or world is None or m is None:
            continue
        alpha = _parse_alpha_str(m.group("alpha"))
        runs.append(
            {
                "csv_path": csv_path,
                "run_dir": run_dir,
                "plast": plasticity,
                "world": world,
                "alpha": alpha,
                "grid": int(m.group("g")),
                "food": int(m.group("i")),
                "seed": int(m.group("seed")),
                "thr": int(m.group("thr")),
            }
        )
    return runs


def main() -> int:
    p = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    p.add_argument("--root", default="Baldwin_Experiment",
                   help="Root dir containing Evolved_results/, Plastic_1_results/, Plastic_2_results/.")
    p.add_argument("--metric", choices=list(METRIC_INFO.keys()), default="fitness")
    p.add_argument("--out-dir", default=None,
                   help="Output dir (default: <root>/figures/full_grid_<metric>/).")
    p.add_argument("--ma-window", type=int, default=50,
                   help="Smoothing window per run.")
    p.add_argument("--min-gen", type=int, default=None,
                   help="Lower x-axis clip (skip generations 0..min_gen-1). Pair with --max-gen for a window.")
    p.add_argument("--max-gen", type=int, default=None,
                   help="Upper x-axis clip (keep generations 0..max_gen).")
    p.add_argument("--split-by", default="none",
                   choices=["none", "world", "world,alpha"],
                   help="Produce one image (none), one per world, or one per (world, alpha).")
    p.add_argument("--share-y", action="store_true",
                   help="Share Y axis across panels of the same figure.")
    p.add_argument(
        "--no-unified-y",
        dest="unified_y",
        action="store_false",
        help="Let each panel auto-scale its own y-axis (default: one shared scale per figure).",
    )
    p.set_defaults(unified_y=True)
    p.add_argument(
        "--y-margin",
        type=float,
        default=0.05,
        help="Relative padding added above/below data range for unified y (default 0.05 = 5%%).",
    )
    p.add_argument("--ymin", type=float, default=None, metavar="YMIN",
                   help="Force lower y limit (use with --ymax). Overrides auto unified scale.")
    p.add_argument("--ymax", type=float, default=None, metavar="YMAX",
                   help="Force upper y limit (use with --ymin). Overrides auto unified scale.")
    p.add_argument("--dpi", type=int, default=130)
    p.add_argument(
        "--no-ghost-raw",
        dest="ghost_raw",
        action="store_false",
        help="When --ma-window > 1: skip faint raw traces (MA overlay only).",
    )
    p.set_defaults(ghost_raw=True)
    p.add_argument(
        "--overlay",
        action="store_true",
        help="Twin-axis overlay: primary --metric on left (solid), mean_u_drift_end on right (dashed).",
    )
    p.add_argument(
        "--overlay-metric",
        choices=list(METRIC_INFO.keys()),
        default="u_drift",
        help="Secondary metric for --overlay (default: u_drift).",
    )
    p.add_argument(
        "--same-yscale",
        action="store_true",
        help="With --overlay: force left (primary) and right (overlay) axes to share ONE unified y-range "
             "computed from both metrics combined (so both curves are directly comparable).",
    )
    p.add_argument(
        "--panel-w", type=float, default=_DEFAULT_PANEL_W,
        help="Width (inches) per subplot column. Larger = bigger panels.",
    )
    p.add_argument(
        "--panel-h", type=float, default=_DEFAULT_PANEL_H,
        help="Height (inches) per subplot row.",
    )
    p.add_argument(
        "--line-lw", type=float, default=_DEFAULT_MA_LW,
        help="Moving-average line width (primary metric). Overlay uses 0.85x of this.",
    )
    p.add_argument(
        "--raw-lw", type=float, default=_DEFAULT_RAW_LW,
        help="Raw 'ghost' trace line width.",
    )
    args = p.parse_args()

    root = os.path.abspath(args.root)
    if not os.path.isdir(root):
        print(f"[error] root not found: {root}", file=sys.stderr)
        return 2

    runs = _load_runs(root)
    if not runs:
        print(f"[error] no parseable lineage CSVs under {root}", file=sys.stderr)
        return 2

    metric_col = METRIC_INFO[args.metric]["col"]
    ylabel = METRIC_INFO[args.metric]["ylabel"]

    overlay_metric_col: str | None = None
    overlay_ylabel: str | None = None
    if args.overlay:
        if args.overlay_metric == args.metric:
            print("[error] --overlay-metric must differ from --metric", file=sys.stderr)
            return 2
        overlay_metric_col = METRIC_INFO[args.overlay_metric]["col"]
        overlay_ylabel = METRIC_INFO[args.overlay_metric]["ylabel"]

    if (args.ymin is None) ^ (args.ymax is None):
        print("[error] pass both --ymin and --ymax, or neither", file=sys.stderr)
        return 2

    # Filter runs to those that actually have the metric column,
    # without re-reading every CSV here we do it lazily inside _plot_image.
    metric_dir_tag = (
        f"{args.metric}_overlay_{args.overlay_metric}" if args.overlay else args.metric
    )
    out_dir_default = os.path.join(root, "figures", f"full_grid_{metric_dir_tag}")
    out_dir = os.path.abspath(args.out_dir) if args.out_dir else out_dir_default
    os.makedirs(out_dir, exist_ok=True)
    file_tag = metric_dir_tag

    cols_keys = list(product(GRID_ORDER, FOOD_ORDER))  # (g, i)

    if args.split_by == "none":
        rows_keys = list(product(WORLD_ORDER, ALPHA_ORDER))
        out_path = os.path.join(out_dir, f"all_conditions_{file_tag}.png")
        ok = _plot_image(
            runs,
            rows_keys=rows_keys,
            cols_keys=cols_keys,
            metric_col=metric_col,
            ylabel=ylabel,
            ma_window=args.ma_window,
            min_gen=args.min_gen,
            max_gen=args.max_gen,
            out_path=out_path,
            title_suffix="",
            dpi=args.dpi,
            share_y=args.share_y,
            unified_y=args.unified_y,
            y_margin=args.y_margin,
            y_min=args.ymin,
            y_max=args.ymax,
            ghost_raw=args.ghost_raw,
            overlay_metric_col=overlay_metric_col,
            overlay_ylabel=overlay_ylabel,
            panel_w=args.panel_w,
            panel_h=args.panel_h,
            ma_lw=args.line_lw,
            raw_lw=args.raw_lw,
            same_yscale=args.same_yscale,
        )
        if not ok:
            print("[error] no panels produced", file=sys.stderr)
            return 2
        return 0

    if args.split_by == "world":
        any_ok = False
        for w in WORLD_ORDER:
            rows_keys = [(w, a) for a in ALPHA_ORDER]
            out_path = os.path.join(out_dir, f"all_conditions_{file_tag}_{w}.png")
            ok = _plot_image(
                runs,
                rows_keys=rows_keys,
                cols_keys=cols_keys,
                metric_col=metric_col,
                ylabel=ylabel,
                ma_window=args.ma_window,
                min_gen=args.min_gen,
                max_gen=args.max_gen,
                out_path=out_path,
                title_suffix=f"  [{w} world]",
                dpi=args.dpi,
                share_y=args.share_y,
                unified_y=args.unified_y,
                y_margin=args.y_margin,
                y_min=args.ymin,
                y_max=args.ymax,
                ghost_raw=args.ghost_raw,
                overlay_metric_col=overlay_metric_col,
                overlay_ylabel=overlay_ylabel,
                panel_w=args.panel_w,
                panel_h=args.panel_h,
                ma_lw=args.line_lw,
                raw_lw=args.raw_lw,
                same_yscale=args.same_yscale,
            )
            any_ok = any_ok or ok
        return 0 if any_ok else 2

    if args.split_by == "world,alpha":
        any_ok = False
        for w in WORLD_ORDER:
            for a in ALPHA_ORDER:
                rows_keys = [(w, a)]
                out_path = os.path.join(
                    out_dir,
                    f"all_conditions_{file_tag}_{w}_a{_alpha_to_str(a)}.png",
                )
                ok = _plot_image(
                    runs,
                    rows_keys=rows_keys,
                    cols_keys=cols_keys,
                    metric_col=metric_col,
                    ylabel=ylabel,
                    ma_window=args.ma_window,
                    min_gen=args.min_gen,
                    max_gen=args.max_gen,
                    out_path=out_path,
                    title_suffix=f"  [{w} world, α={a}]",
                    dpi=args.dpi,
                    share_y=args.share_y,
                    unified_y=args.unified_y,
                    y_margin=args.y_margin,
                    y_min=args.ymin,
                    y_max=args.ymax,
                    ghost_raw=args.ghost_raw,
                    overlay_metric_col=overlay_metric_col,
                    overlay_ylabel=overlay_ylabel,
                    panel_w=args.panel_w,
                    panel_h=args.panel_h,
                    ma_lw=args.line_lw,
                    raw_lw=args.raw_lw,
                    same_yscale=args.same_yscale,
                )
                any_ok = any_ok or ok
        return 0 if any_ok else 2

    print(f"[error] unknown --split-by mode {args.split_by!r}", file=sys.stderr)
    return 2


if __name__ == "__main__":
    sys.exit(main())
