"""
plot_baldwin_grid.py — Tier-A 9-panel headline figure.

Layout:
    rows = world difficulty   (easy / normal / hard)
    cols = alpha_child        (0.3  / 0.5    / 0.7)
    each panel: smoothed metric vs generation
                3 lines per panel: E2 (genes only) / P1 (out_adapt+global) / P2 (out+local)
                mean across replicates (other ecology cells, seeds), with IQR band.

Default metric is "fitness". Use --metric u_drift to plot mean_u_drift_end
(the canonical Baldwin / genetic-assimilation diagnostic).

All panels in one figure share the same y-axis limits by default (computed from
the plotted mean lines and shaded bands). Use --no-unified-y for per-panel autoscale.

With --overlay, the primary --metric is drawn on the left Y-axis (solid mean lines +
optional band); --overlay-metric is drawn on the right Y-axis (dashed means, same
plasticity colors). Unified limits are computed separately for left and right.

Usage:
  python plot_baldwin_grid.py
  python plot_baldwin_grid.py --metric u_drift
  python plot_baldwin_grid.py --root Baldwin_Experiment --out Baldwin_Experiment/figures/baldwin_grid_fitness.png
  python plot_baldwin_grid.py --metric fitness --overlay --overlay-metric u_drift --min-gen 1000 --max-gen 3000 --line-lw 1.3
"""

from __future__ import annotations

import argparse
import os
import re
import sys
from glob import glob

import numpy as np
import pandas as pd

import matplotlib

matplotlib.use("Agg")  # headless safe
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
PLAST_ORDER = ["E2", "P1", "P2"]

PLAST_LABEL = {
    "E2": "E2 (genes only)",
    "P1": "P1 (outcome_adaptive + global)",
    "P2": "P2 (outcome + local)",
}
PLAST_COLOR = {
    "E2": "#1f77b4",  # blue
    "P1": "#d62728",  # red
    "P2": "#2ca02c",  # green
}

_DEFAULT_LINE_LW = 1.6
_OVERLAY_LW_SCALE = 0.85
_OV_LINESTYLE = (0, (4, 2))
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
    "child_survival": {"col": "mean_child_survival", "ylabel": "Mean child survival (eval batch)"},
    "child_ttd": {"col": "mean_child_ttd_norm", "ylabel": "Child TTD (normalized)"},
    "mother_ttd": {"col": "mean_mother_ttd_norm", "ylabel": "Mother TTD (normalized)"},
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


def _stack_traces(traces: list[np.ndarray]) -> np.ndarray:
    """Stack a list of 1D arrays of varying length into (n_runs, max_len), padded with NaN."""
    if not traces:
        return np.empty((0, 0))
    max_len = max(len(t) for t in traces)
    out = np.full((len(traces), max_len), np.nan, dtype=float)
    for i, t in enumerate(traces):
        out[i, : len(t)] = t
    return out


def _collect_ylim_from_plotted(
    stacked: np.ndarray,
    band: str,
    mean: np.ndarray,
) -> list[np.ndarray]:
    """Return arrays whose finite values should contribute to global y-limits."""
    out: list[np.ndarray] = [mean]
    if band == "iqr":
        lo = np.nanpercentile(stacked, 25, axis=0)
        hi = np.nanpercentile(stacked, 75, axis=0)
        out.extend([lo, hi])
    elif band == "minmax":
        lo = np.nanmin(stacked, axis=0)
        hi = np.nanmax(stacked, axis=0)
        out.extend([lo, hi])
    return out


def _series_from_df(
    df: pd.DataFrame,
    col: str,
    ma_window: int,
    max_gen: int | None,
    min_gen: int | None,
) -> np.ndarray | None:
    """Extract column, smooth, then clip to [min_gen, max_gen] in generation index."""
    vals = pd.to_numeric(df[col], errors="coerce").to_numpy()
    if max_gen is not None:
        vals = vals[: int(max_gen) + 1]
    smoothed = _smooth(vals, ma_window)
    if min_gen is not None and int(min_gen) > 0:
        lo = int(min_gen)
        if lo >= len(smoothed):
            return None
        smoothed = smoothed[lo:]
    if smoothed.size == 0 or not np.any(np.isfinite(smoothed)):
        return None
    return smoothed


def main() -> int:
    p = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    p.add_argument("--root", default="Baldwin_Experiment",
                   help="Root dir containing Evolved_results/, Plastic_1_results/, Plastic_2_results/.")
    p.add_argument("--metric", choices=list(METRIC_INFO.keys()), default="fitness")
    p.add_argument("--out", default=None,
                   help="Output PNG path (default: <root>/figures/baldwin_grid_<metric>.png).")
    p.add_argument("--ma-window", type=int, default=50, help="Smoothing window per run before aggregation.")
    p.add_argument(
        "--min-gen",
        type=int,
        default=None,
        help="Lower x clip: drop generations 0..min_gen-1 after smoothing (pair with --max-gen for a window).",
    )
    p.add_argument("--max-gen", type=int, default=None, help="Upper clip on raw series before smoothing (keep 0..max_gen).")
    p.add_argument("--band", choices=["iqr", "minmax", "none"], default="iqr")
    p.add_argument("--share-y", action="store_true", help="Share Y axis across panels (off by default).")
    p.add_argument(
        "--no-unified-y",
        dest="unified_y",
        action="store_false",
        help="Let each panel auto-scale y (default: one shared y-range per figure).",
    )
    p.set_defaults(unified_y=True)
    p.add_argument(
        "--y-margin",
        type=float,
        default=0.05,
        help="Relative padding for unified y-limits (default 5%%).",
    )
    p.add_argument("--ymin", type=float, default=None, help="Fixed lower y (use with --ymax).")
    p.add_argument("--ymax", type=float, default=None, help="Fixed upper y (use with --ymin).")
    p.add_argument(
        "--ymin-right",
        type=float,
        default=None,
        metavar="YMIN_R",
        help="With --overlay: fixed lower limit for right Y-axis (use with --ymax-right).",
    )
    p.add_argument(
        "--ymax-right",
        type=float,
        default=None,
        metavar="YMAX_R",
        help="With --overlay: fixed upper limit for right Y-axis (use with --ymin-right).",
    )
    p.add_argument(
        "--overlay",
        action="store_true",
        help="Twin axis: primary --metric on left (solid + band), --overlay-metric on right (dashed means).",
    )
    p.add_argument(
        "--overlay-metric",
        choices=list(METRIC_INFO.keys()),
        default="u_drift",
        help="Secondary metric for --overlay (default: u_drift). Must differ from --metric.",
    )
    p.add_argument(
        "--same-yscale",
        action="store_true",
        help="With --overlay: force left (primary) and right (overlay) axes to share ONE unified y-range "
             "computed from both metrics combined (so curves are directly comparable on the same scale).",
    )
    p.add_argument(
        "--line-lw",
        type=float,
        default=_DEFAULT_LINE_LW,
        help=f"Mean line width for primary metric (default {_DEFAULT_LINE_LW}). Overlay uses {_OVERLAY_LW_SCALE:.2f}× this.",
    )
    p.add_argument(
        "--panel-w",
        type=float,
        default=5.0,
        help="Subplot width (inches) per column.",
    )
    p.add_argument(
        "--panel-h",
        type=float,
        default=3.4,
        help="Subplot height (inches) per row.",
    )
    p.add_argument("--dpi", type=int, default=150)
    args = p.parse_args()

    if (args.ymin is None) ^ (args.ymax is None):
        print("[error] pass both --ymin and --ymax, or neither", file=sys.stderr)
        return 2
    if (args.ymin_right is None) ^ (args.ymax_right is None):
        print("[error] pass both --ymin-right and --ymax-right, or neither", file=sys.stderr)
        return 2
    if args.overlay and args.overlay_metric == args.metric:
        print("[error] --overlay-metric must differ from --metric", file=sys.stderr)
        return 2

    root = os.path.abspath(args.root)
    if not os.path.isdir(root):
        print(f"[error] root not found: {root}", file=sys.stderr)
        return 2
    metric_tag = (
        f"{args.metric}_overlay_{args.overlay_metric}" if args.overlay else args.metric
    )
    out_path = (
        os.path.abspath(args.out)
        if args.out
        else os.path.join(root, "figures", f"baldwin_grid_{metric_tag}.png")
    )

    csvs = sorted(glob(os.path.join(root, "**", "lineage_generations.csv"), recursive=True))
    if not csvs:
        print(f"[error] no lineage_generations.csv found under {root}", file=sys.stderr)
        return 2

    metric_col = METRIC_INFO[args.metric]["col"]
    ylabel = METRIC_INFO[args.metric]["ylabel"]
    overlay_col: str | None = None
    overlay_ylabel: str | None = None
    if args.overlay:
        overlay_col = METRIC_INFO[args.overlay_metric]["col"]
        overlay_ylabel = METRIC_INFO[args.overlay_metric]["ylabel"]

    # group: {(world, alpha, plasticity): [smoothed trace, ...]}
    groups: dict[tuple[str, float, str], list[np.ndarray]] = {}
    groups_ov: dict[tuple[str, float, str], list[np.ndarray]] = {}
    skipped = 0
    for csv_path in csvs:
        plasticity, world = _detect_plasticity_and_world(csv_path, root)
        run_dir = os.path.basename(os.path.dirname(csv_path))
        m = RUN_NAME_RE.search(run_dir)
        if plasticity is None or world is None or m is None:
            skipped += 1
            continue
        alpha = _parse_alpha_str(m.group("alpha"))
        if alpha not in ALPHA_ORDER:
            skipped += 1
            continue
        try:
            df = pd.read_csv(csv_path)
        except Exception:
            skipped += 1
            continue
        if metric_col not in df.columns or "generation" not in df.columns:
            skipped += 1
            continue
        if args.overlay and (overlay_col is None or overlay_col not in df.columns):
            skipped += 1
            continue
        df = df.sort_values("generation")
        tr = _series_from_df(df, metric_col, args.ma_window, args.max_gen, args.min_gen)
        if tr is None:
            skipped += 1
            continue
        if args.overlay and overlay_col is not None:
            tro = _series_from_df(df, overlay_col, args.ma_window, args.max_gen, args.min_gen)
            if tro is None:
                skipped += 1
                continue
            groups_ov.setdefault((world, alpha, plasticity), []).append(tro)
        groups.setdefault((world, alpha, plasticity), []).append(tr)

    if not groups:
        print("[error] no valid traces collected", file=sys.stderr)
        return 2

    fig, axes = plt.subplots(
        nrows=len(WORLD_ORDER),
        ncols=len(ALPHA_ORDER),
        figsize=(
            max(2.0, args.panel_w) * len(ALPHA_ORDER),
            max(2.0, args.panel_h) * len(WORLD_ORDER),
        ),
        sharex=True,
        sharey=bool(args.share_y),
    )
    if len(WORLD_ORDER) == 1:
        axes = np.array([axes])
    if len(ALPHA_ORDER) == 1:
        axes = axes.reshape(-1, 1)

    gen0 = int(args.min_gen) if (args.min_gen is not None and int(args.min_gen) > 0) else 0
    ov_lw = float(args.line_lw) * _OVERLAY_LW_SCALE

    plotted_any = False
    ylim_candidates: list[float] = []
    ylim_candidates_r: list[float] = []
    right_axes: list = []
    x_max_seen = gen0

    for r, world in enumerate(WORLD_ORDER):
        for c, alpha in enumerate(ALPHA_ORDER):
            ax = axes[r, c]
            ax_r = ax.twinx() if args.overlay else None
            if ax_r is not None:
                right_axes.append(ax_r)
                ax_r.tick_params(axis="y", labelsize=8, colors=_OV_RIGHT_AXIS_COLOR)
                for sp in ("top", "right"):
                    ax_r.spines[sp].set_alpha(0.4)

            for plast in PLAST_ORDER:
                key = (world, alpha, plast)
                traces = groups.get(key)
                if not traces:
                    continue
                stacked = _stack_traces(traces)
                gens = gen0 + np.arange(stacked.shape[1])
                x_max_seen = max(x_max_seen, int(gens[-1]) if gens.size else gen0)
                mean = np.nanmean(stacked, axis=0)
                color = PLAST_COLOR[plast]
                if args.band == "iqr":
                    lo = np.nanpercentile(stacked, 25, axis=0)
                    hi = np.nanpercentile(stacked, 75, axis=0)
                    ax.fill_between(gens, lo, hi, color=color, alpha=0.18, linewidth=0)
                elif args.band == "minmax":
                    lo = np.nanmin(stacked, axis=0)
                    hi = np.nanmax(stacked, axis=0)
                    ax.fill_between(gens, lo, hi, color=color, alpha=0.10, linewidth=0)
                ax.plot(
                    gens,
                    mean,
                    color=color,
                    linewidth=float(args.line_lw),
                    label=f"{PLAST_LABEL[plast]}  (n={stacked.shape[0]})",
                )
                if args.unified_y and args.ymin is None:
                    for arr in _collect_ylim_from_plotted(stacked, args.band, mean):
                        finite = arr[np.isfinite(arr)]
                        if finite.size:
                            ylim_candidates.extend(finite.tolist())
                plotted_any = True

                if args.overlay and ax_r is not None and overlay_ylabel:
                    traces_o = groups_ov.get(key)
                    if not traces_o:
                        continue
                    stacked_o = _stack_traces(traces_o)
                    gens_o = gen0 + np.arange(stacked_o.shape[1])
                    x_max_seen = max(x_max_seen, int(gens_o[-1]) if gens_o.size else gen0)
                    mean_o = np.nanmean(stacked_o, axis=0)
                    ax_r.plot(
                        gens_o,
                        mean_o,
                        color=color,
                        linewidth=ov_lw,
                        linestyle=_OV_LINESTYLE,
                        alpha=0.92,
                        zorder=3,
                    )
                    if args.unified_y and args.ymin_right is None:
                        finite_o = mean_o[np.isfinite(mean_o)]
                        if finite_o.size:
                            ylim_candidates_r.extend(finite_o.tolist())

            ax.grid(True, alpha=0.25)
            if r == 0:
                ax.set_title(f"α_child = {alpha}")
            if c == 0:
                ax.set_ylabel(f"{world.capitalize()}\n{ylabel}", fontsize=10)
            else:
                ax.set_ylabel("")
            if r == len(WORLD_ORDER) - 1:
                ax.set_xlabel("Generation")
            if ax_r is not None:
                if c == len(ALPHA_ORDER) - 1:
                    ax_r.set_ylabel(overlay_ylabel or "", fontsize=9, color=_OV_RIGHT_AXIS_COLOR)
                else:
                    ax_r.set_ylabel("")
            if not args.overlay:
                ax.legend(loc="best", fontsize=7, framealpha=0.85)

    if not plotted_any:
        print(
            "[error] nothing to plot — none of the (world, alpha, plasticity) cells had data.",
            file=sys.stderr,
        )
        return 2

    if args.max_gen is not None:
        x_max_seen = max(x_max_seen, int(args.max_gen))

    for ax in axes.flat:
        ax.set_xlim(gen0, x_max_seen)

    ylim_note = ""
    if args.ymin is not None and args.ymax is not None:
        y_lo, y_hi = float(args.ymin), float(args.ymax)
        for r in range(len(WORLD_ORDER)):
            for c in range(len(ALPHA_ORDER)):
                axes[r, c].set_ylim(y_lo, y_hi)
        ylim_note = f"  [y = [{y_lo:.4g}, {y_hi:.4g}]]"
    elif args.unified_y and ylim_candidates:
        ymin, ymax = float(min(ylim_candidates)), float(max(ylim_candidates))
        if ymax <= ymin:
            ymax = ymin + 1e-9
        pad = args.y_margin * (ymax - ymin)
        y_lo, y_hi = ymin - pad, ymax + pad
        for r in range(len(WORLD_ORDER)):
            for c in range(len(ALPHA_ORDER)):
                axes[r, c].set_ylim(y_lo, y_hi)
        ylim_note = f"  [shared y: [{y_lo:.4g}, {y_hi:.4g}]]"

    ylim_note_r = ""
    if args.overlay and right_axes:
        if args.same_yscale:
            combined = list(ylim_candidates) + list(ylim_candidates_r)
            if args.ymin is not None and args.ymax is not None:
                yr_lo, yr_hi = float(args.ymin), float(args.ymax)
            elif combined:
                ymin_b = float(min(combined))
                ymax_b = float(max(combined))
                if ymax_b <= ymin_b:
                    ymax_b = ymin_b + 1e-9
                pad_b = args.y_margin * (ymax_b - ymin_b)
                yr_lo, yr_hi = ymin_b - pad_b, ymax_b + pad_b
            else:
                yr_lo, yr_hi = (0.0, 1.0)
            for r in range(len(WORLD_ORDER)):
                for c in range(len(ALPHA_ORDER)):
                    axes[r, c].set_ylim(yr_lo, yr_hi)
            for ax_r in right_axes:
                ax_r.set_ylim(yr_lo, yr_hi)
            ylim_note = f"  [shared y (both axes): [{yr_lo:.4g}, {yr_hi:.4g}]]"
            ylim_note_r = ""
        elif args.ymin_right is not None and args.ymax_right is not None:
            yr_lo, yr_hi = float(args.ymin_right), float(args.ymax_right)
            for ax_r in right_axes:
                ax_r.set_ylim(yr_lo, yr_hi)
            ylim_note_r = f"  [right y: [{yr_lo:.4g}, {yr_hi:.4g}]]"
        elif args.unified_y and ylim_candidates_r:
            ymin_r, ymax_r = float(min(ylim_candidates_r)), float(max(ylim_candidates_r))
            if ymax_r <= ymin_r:
                ymax_r = ymin_r + 1e-9
            pad_r = args.y_margin * (ymax_r - ymin_r)
            yr_lo, yr_hi = ymin_r - pad_r, ymax_r + pad_r
            for ax_r in right_axes:
                ax_r.set_ylim(yr_lo, yr_hi)
            ylim_note_r = f"  [shared right y: [{yr_lo:.4g}, {yr_hi:.4g}]]"

    overlay_note = ""
    if args.overlay and overlay_ylabel:
        overlay_note = f"  +  {overlay_ylabel} (dashed, right axis)"

    band_label = "IQR" if args.band == "iqr" else (args.band if args.band != "none" else "no band")
    fig.suptitle(
        f"Baldwin grid — {ylabel}{overlay_note}  "
        f"(mean ± {band_label}, MA window={args.ma_window}){ylim_note}{ylim_note_r}",
        fontsize=12,
    )

    if args.overlay and overlay_ylabel:
        leg_handles: list = []
        for plast in PLAST_ORDER:
            color = PLAST_COLOR[plast]
            leg_handles.append(
                plt.Line2D(
                    [0],
                    [0],
                    color=color,
                    linewidth=float(args.line_lw),
                    label=f"{PLAST_LABEL[plast]} — {ylabel}",
                )
            )
            leg_handles.append(
                plt.Line2D(
                    [0],
                    [0],
                    color=color,
                    linewidth=ov_lw,
                    linestyle=_OV_LINESTYLE,
                    label=f"{PLAST_LABEL[plast]} — {overlay_ylabel}",
                )
            )
        ncol = min(len(leg_handles), 6)
        fig.legend(
            handles=leg_handles,
            loc="upper center",
            ncol=ncol,
            fontsize=7,
            frameon=False,
            bbox_to_anchor=(0.5, 1.02),
        )

    fig.tight_layout(rect=(0, 0, 1, 0.93 if args.overlay else 0.965))

    parent = os.path.dirname(out_path)
    if parent:
        os.makedirs(parent, exist_ok=True)
    fig.savefig(out_path, dpi=args.dpi)
    plt.close(fig)
    print(f"[ok] wrote {out_path}")
    print(f"     groups={len(groups)}  skipped={skipped}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
