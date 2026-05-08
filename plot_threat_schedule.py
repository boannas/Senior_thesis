"""
plot_threat_schedule.py -- visualize evolutionary lineages with a threat-introduction schedule.

Designed for runs produced by:
    run_evolve_lineage.py --threats-schedule "0:0,500:1,1500:2" ...

Discovery (no schedule-only assumption -- also works for static-threat runs):
  * Default mode: glob `<root>/**/lineage_generations.csv` and keep CSVs under
    `Evolved_results/`, `Plastic_1_results/`, or `Plastic_2_results/` whose folder
    name has the form `..._g<G>_i<I>_a<NpN>_seed<N>` (the threats segment is
    parsed from `thr<digits>` OR `thrSCHED-<digits>(-<digits>)*`).
  * Explicit mode (`--csv ...`): plot exactly the listed CSVs.

Each panel shows generations vs `--metric` (default fitness). Lines are colored by
plasticity (E2 / P1 / P2). When `--ma-window > 1` the moving average is drawn over
faint raw traces, identical to plot_baldwin_full_grid.py.

Regime change points (where `num_threats_active` changes generation-over-generation,
read directly from each CSV) are drawn as vertical dashed lines, optionally annotated
with `threats=N`. Override with `--regime-lines GEN[,GEN,...]` (manual) or
`--no-regime-lines` (off).

Layout:
  rows = world  (any of easy/normal/hard found in the discovered runs)
  cols = alpha  (any of 0.3/0.5/0.7 found in the discovered runs)
  one panel per (world, alpha); E2/P1/P2 colors inside each panel.

Usage:
  python plot_threat_schedule.py
  python plot_threat_schedule.py --root . --metric fitness --overlay --overlay-metric u_drift
  python plot_threat_schedule.py --csv Plastic_1_results/easy/<run1>/lineage_generations.csv \
                                 --csv Plastic_2_results/easy/<run2>/lineage_generations.csv
  python plot_threat_schedule.py --worlds easy --alphas 0.3,0.5
  python plot_threat_schedule.py --regime-lines 500,1500       # manual override
  python plot_threat_schedule.py --no-regime-lines             # disable
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

matplotlib.use("Agg")
import matplotlib.pyplot as plt

# ---------- conventions (same as plot_baldwin_*.py) -------------------------------
PLASTIC_TAG = {
    "Evolved_results": "E2",
    "Plastic_1_results": "P1",
    "Plastic_2_results": "P2",
}
WORLD_TAG = {"easy", "normal", "hard"}
WORLD_ORDER = ["easy", "normal", "hard"]
ALPHA_ORDER_DEFAULT = [0.3, 0.5, 0.7]
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

# Plot defaults (overridable via CLI)
_DEFAULT_PANEL_W = 4.4
_DEFAULT_PANEL_H = 3.0
_DEFAULT_MA_LW = 1.4
_DEFAULT_RAW_LW = 0.4
_OVERLAY_LW_SCALE = 0.85

_RAW_LINE_ALPHA = 0.12
_RAW_Z = 1
_MA_ALPHA = 0.92
_MA_Z = 4
_OV_LINESTYLE = (0, (4, 2))
_OV_RAW_ALPHA = 0.10
_OV_MA_ALPHA = 0.92
_OV_RIGHT_AXIS_COLOR = "0.30"

# Regime line styling
_REGIME_COLOR = "0.35"
_REGIME_LS = (0, (3, 3))
_REGIME_LW = 0.9
_REGIME_ALPHA = 0.85
_REGIME_LABEL_FS = 7

# Accept either thr<digits> (static) or thrSCHED-<digits>(-<digits>)*
RUN_NAME_RE = re.compile(
    r"thr(?P<thr>(?:SCHED-[\d-]+|\d+))_g(?P<g>\d+)_i(?P<i>\d+)_"
    r"a(?P<alpha>[0-9]+p[0-9]+)_seed(?P<seed>\d+)"
)

METRIC_INFO = {
    "fitness": {"col": "fitness", "ylabel": "Fitness"},
    "u_drift": {"col": "mean_u_drift_end", "ylabel": "Mean |u_plastic - u_fixed| (end)"},
    "u_drift_peak": {"col": "mean_peak_u_drift_episode", "ylabel": "Peak |u_plastic - u_fixed| (within ep)"},
    "du_plastic": {"col": "mean_du_plastic_episode", "ylabel": "Mean L1 |delta u_plastic| per tick"},
    "du_plastic_peak": {"col": "mean_peak_du_plastic_episode", "ylabel": "Peak L1 |delta u_plastic| per tick"},
    "plastic_active_frac": {"col": "mean_plastic_active_frac", "ylabel": "Plastic-active fraction of ticks"},
    "lr_eff": {"col": "mean_lr_eff_episode", "ylabel": "Mean effective learning rate"},
    "child_survival": {"col": "mean_child_survival", "ylabel": "Mean child survival"},
    "child_ttd": {"col": "mean_child_ttd_norm", "ylabel": "Child TTD (norm)"},
    "mother_ttd": {"col": "mean_mother_ttd_norm", "ylabel": "Mother TTD (norm)"},
}


# ---------- helpers ---------------------------------------------------------------

def _parse_alpha_str(s: str) -> float:
    return float(s.replace("p", "."))


def _alpha_to_str(a: float) -> str:
    return f"{a:.1f}".replace(".", "p")


def _smooth(arr: np.ndarray, window: int) -> np.ndarray:
    if window <= 1:
        return arr.astype(float)
    return pd.Series(arr).rolling(window=window, min_periods=1).mean().to_numpy()


def _clip_lower(arr: np.ndarray, min_gen: int | None) -> np.ndarray:
    if min_gen is None or min_gen <= 0:
        return arr
    if min_gen >= arr.size:
        return arr[arr.size:]
    return arr[min_gen:]


def _detect_plasticity_and_world(csv_path: str) -> tuple[str | None, str | None]:
    parts = os.path.normpath(csv_path).split(os.sep)
    plasticity: str | None = None
    world: str | None = None
    for p in parts:
        if p in PLASTIC_TAG and plasticity is None:
            plasticity = PLASTIC_TAG[p]
        if p in WORLD_TAG and world is None:
            world = p
    return plasticity, world


def _detect_regime_changes_from_csv(csv_path: str) -> list[tuple[int, int]]:
    """Read `num_threats_active` and return list of (gen, new_value) at every change.

    The list excludes the initial value at gen 0 (we only want change points).
    """
    try:
        df = pd.read_csv(csv_path, usecols=lambda c: c in ("generation", "num_threats_active"))
    except Exception:
        return []
    if "num_threats_active" not in df.columns or "generation" not in df.columns:
        return []
    df = df.sort_values("generation")
    gen = df["generation"].astype(int).to_numpy()
    nta = pd.to_numeric(df["num_threats_active"], errors="coerce").to_numpy()
    if gen.size == 0 or not np.any(np.isfinite(nta)):
        return []
    changes: list[tuple[int, int]] = []
    prev = nta[0]
    for i in range(1, gen.size):
        if np.isfinite(nta[i]) and nta[i] != prev:
            changes.append((int(gen[i]), int(nta[i])))
            prev = nta[i]
    return changes


def _raw_series(csv_path: str, metric_col: str, max_gen: int | None) -> np.ndarray | None:
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


def _load_runs(
    root: str | None,
    csvs_explicit: list[str] | None,
) -> list[dict]:
    if csvs_explicit:
        csv_paths = [os.path.abspath(c) for c in csvs_explicit]
    else:
        if root is None:
            root = "."
        csv_paths = sorted(glob(os.path.join(root, "**", "lineage_generations.csv"), recursive=True))
    runs: list[dict] = []
    for csv_path in csv_paths:
        plasticity, world = _detect_plasticity_and_world(csv_path)
        run_dir = os.path.basename(os.path.dirname(csv_path))
        m = RUN_NAME_RE.search(run_dir)
        if plasticity is None or m is None:
            continue
        if world is None:
            world = "any"
        try:
            alpha = _parse_alpha_str(m.group("alpha"))
        except ValueError:
            continue
        thr = m.group("thr")
        is_sched = thr.startswith("SCHED-")
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
                "thr_raw": thr,
                "is_sched": is_sched,
            }
        )
    return runs


def _parse_int_list(spec: str | None) -> list[int] | None:
    if spec is None or not spec.strip():
        return None
    out: list[int] = []
    for chunk in str(spec).replace(";", ",").split(","):
        chunk = chunk.strip()
        if not chunk:
            continue
        try:
            out.append(int(chunk))
        except ValueError as e:
            raise ValueError(f"could not parse '{chunk}' as int") from e
    return out


def _parse_float_list(spec: str | None) -> list[float] | None:
    if spec is None or not spec.strip():
        return None
    out: list[float] = []
    for chunk in str(spec).replace(";", ",").split(","):
        chunk = chunk.strip()
        if not chunk:
            continue
        try:
            out.append(float(chunk))
        except ValueError as e:
            raise ValueError(f"could not parse '{chunk}' as float") from e
    return out


def _parse_str_list(spec: str | None) -> list[str] | None:
    if spec is None or not spec.strip():
        return None
    return [c.strip() for c in str(spec).replace(";", ",").split(",") if c.strip()]


# ---------- plotting -------------------------------------------------------------

def _draw_regime_lines(
    ax,
    regime_lines: list[tuple[int, int | None]],
    annotate: bool,
    visible_xrange: tuple[float, float],
):
    if not regime_lines:
        return
    x_lo, x_hi = visible_xrange
    for gen, new_n in regime_lines:
        if gen < x_lo or gen > x_hi:
            continue
        ax.axvline(
            gen,
            color=_REGIME_COLOR,
            linestyle=_REGIME_LS,
            linewidth=_REGIME_LW,
            alpha=_REGIME_ALPHA,
            zorder=2,
        )
        if annotate and new_n is not None:
            ax.text(
                gen,
                1.0,
                f"thr→{new_n}",
                transform=ax.get_xaxis_transform(),
                ha="left",
                va="bottom",
                fontsize=_REGIME_LABEL_FS,
                color=_REGIME_COLOR,
                rotation=0,
                clip_on=False,
            )


def _aggregate_regimes(runs_in_cell: list[dict]) -> list[tuple[int, int | None]]:
    """Union of all regime change points across runs in this cell.

    Returns list of (gen, new_threat_count) sorted ascending. If multiple runs
    report different counts at the same gen, we keep the most common (else first).
    """
    by_gen: dict[int, list[int]] = {}
    for r in runs_in_cell:
        for gen, n in _detect_regime_changes_from_csv(r["csv_path"]):
            by_gen.setdefault(gen, []).append(n)
    items: list[tuple[int, int | None]] = []
    for gen in sorted(by_gen.keys()):
        vals = by_gen[gen]
        if vals:
            # mode-ish: pick value that appears most; ties → first observed
            from collections import Counter
            c = Counter(vals)
            n_pick = c.most_common(1)[0][0]
            items.append((gen, int(n_pick)))
        else:
            items.append((gen, None))
    return items


def _compute_unified_ylim(
    cell_runs: dict[tuple[str, float], list[dict]],
    metric_col: str,
    ma_window: int,
    min_gen: int | None,
    max_gen: int | None,
    margin: float,
    ghost_raw: bool,
) -> tuple[float, float] | None:
    chunks: list[np.ndarray] = []
    for runs in cell_runs.values():
        for r in runs:
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


def _plot_image(
    runs: list[dict],
    *,
    rows_keys: list[str],
    cols_keys: list[float],
    metric_col: str,
    ylabel: str,
    ma_window: int,
    min_gen: int | None,
    max_gen: int | None,
    out_path: str,
    title_suffix: str,
    dpi: int,
    panel_w: float,
    panel_h: float,
    ma_lw: float,
    raw_lw: float,
    unified_y: bool,
    y_margin: float,
    y_min: float | None,
    y_max: float | None,
    ghost_raw: bool,
    overlay_metric_col: str | None,
    overlay_ylabel: str | None,
    same_yscale: bool,
    regime_lines_override: list[int] | None,
    show_regime_lines: bool,
    annotate_regime: bool,
) -> bool:
    nr = len(rows_keys)
    nc = len(cols_keys)
    if nr == 0 or nc == 0:
        return False

    cell_runs: dict[tuple[str, float], list[dict]] = {}
    for r in runs:
        key = (r["world"], r["alpha"])
        if r["world"] in rows_keys and r["alpha"] in cols_keys:
            cell_runs.setdefault(key, []).append(r)
    if not cell_runs:
        return False

    ylim: tuple[float, float] | None = None
    if y_min is not None and y_max is not None:
        ylim = (float(y_min), float(y_max))
    elif unified_y:
        ylim = _compute_unified_ylim(
            cell_runs, metric_col, ma_window, min_gen, max_gen, y_margin, ghost_raw,
        )

    ylim_overlay: tuple[float, float] | None = None
    if overlay_metric_col is not None and unified_y:
        ylim_overlay = _compute_unified_ylim(
            cell_runs, overlay_metric_col, ma_window, min_gen, max_gen, y_margin, ghost_raw,
        )

    if same_yscale and overlay_metric_col is not None:
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
        figsize=(max(2.0, panel_w) * nc, max(1.6, panel_h) * nr),
        sharex=True,
        sharey=False,
        squeeze=False,
    )

    ov_ma_lw = ma_lw * _OVERLAY_LW_SCALE
    ov_raw_lw = raw_lw

    seen_plast: set[str] = set()
    show_ghost = ghost_raw and ma_window > 1
    overlay = overlay_metric_col is not None
    right_axes: list = []
    x_max_seen = 0
    x_min_seen = int(min_gen) if (min_gen is not None and min_gen > 0) else 0

    for ri, world in enumerate(rows_keys):
        for ci, alpha in enumerate(cols_keys):
            ax = axes[ri, ci]
            ax_r = ax.twinx() if overlay else None
            if ax_r is not None:
                right_axes.append((ri, ci, ax_r))
                ax_r.tick_params(axis="y", labelsize=7, colors=_OV_RIGHT_AXIS_COLOR)
                for sp in ("top", "right"):
                    ax_r.spines[sp].set_alpha(0.4)

            cell = cell_runs.get((world, alpha), [])

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
                x_max_seen = max(x_max_seen, int(gens[-1]))
                color = PLAST_COLOR[r["plast"]]
                if show_ghost:
                    ax.plot(
                        gens, raw, color=color, linewidth=raw_lw,
                        alpha=_RAW_LINE_ALPHA, solid_capstyle="round", zorder=_RAW_Z,
                    )
                ax.plot(
                    gens, ma, color=color, linewidth=ma_lw,
                    alpha=_MA_ALPHA, solid_capstyle="round", zorder=_MA_Z,
                )
                seen_plast.add(r["plast"])

                if overlay and ax_r is not None:
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
                            gens_ov, raw_ov, color=color, linewidth=ov_raw_lw,
                            alpha=_OV_RAW_ALPHA, linestyle=_OV_LINESTYLE, zorder=_RAW_Z,
                        )
                    ax_r.plot(
                        gens_ov, ma_ov, color=color, linewidth=ov_ma_lw,
                        alpha=_OV_MA_ALPHA, linestyle=_OV_LINESTYLE, zorder=_MA_Z,
                    )

            # regime lines (per panel — based on its own runs)
            if show_regime_lines:
                if regime_lines_override is not None:
                    panel_lines: list[tuple[int, int | None]] = [
                        (int(g), None) for g in regime_lines_override
                    ]
                else:
                    panel_lines = _aggregate_regimes(cell)
                _draw_regime_lines(
                    ax,
                    panel_lines,
                    annotate=annotate_regime,
                    visible_xrange=(x_min_seen, max(x_max_seen, x_min_seen + 1)),
                )

            ax.grid(True, alpha=0.2)
            ax.tick_params(axis="both", labelsize=7)
            if ri == 0:
                ax.set_title(f"α_child = {alpha}", fontsize=9)
            if ci == 0:
                ax.set_ylabel(f"{world}\n{ylabel}", fontsize=8)
            else:
                ax.set_ylabel("")
            if ax_r is not None and ci == nc - 1 and overlay_ylabel:
                ax_r.set_ylabel(overlay_ylabel, fontsize=8, color=_OV_RIGHT_AXIS_COLOR)
            if ri == nr - 1:
                ax.set_xlabel("Generation", fontsize=8)

    if x_max_seen <= x_min_seen:
        x_max_seen = x_min_seen + 1
    for ax in axes.flat:
        ax.set_xlim(x_min_seen, x_max_seen)

    if ylim is not None:
        for ax in axes.flat:
            ax.set_ylim(ylim[0], ylim[1])
    if overlay and ylim_overlay is not None:
        for _ri, _ci, ax_r in right_axes:
            ax_r.set_ylim(ylim_overlay[0], ylim_overlay[1])

    handles: list = []
    for p in PLAST_ORDER:
        if p not in seen_plast:
            continue
        handles.append(
            plt.Line2D([0], [0], color=PLAST_COLOR[p], linewidth=ma_lw,
                       label=f"{PLAST_LABEL[p]} — {ylabel} (MA)")
        )
    if overlay and overlay_ylabel:
        for p in PLAST_ORDER:
            if p not in seen_plast:
                continue
            handles.append(
                plt.Line2D([0], [0], color=PLAST_COLOR[p], linewidth=ov_ma_lw,
                           linestyle=_OV_LINESTYLE,
                           label=f"{PLAST_LABEL[p]} — {overlay_ylabel} (MA)")
            )
    if show_ghost:
        handles.insert(
            0,
            plt.Line2D([0], [0], color="0.45", linewidth=max(raw_lw, 0.6),
                       alpha=0.75, label="Raw (faint)"),
        )
    if show_regime_lines:
        handles.append(
            plt.Line2D([0], [0], color=_REGIME_COLOR, linestyle=_REGIME_LS,
                       linewidth=_REGIME_LW, alpha=_REGIME_ALPHA,
                       label="Regime change (threats↑)")
        )
    if handles:
        ncol = min(len(handles), 4 if overlay else 5)
        fig.legend(handles=handles, loc="upper center", ncol=ncol,
                   fontsize=7.5, frameon=False, bbox_to_anchor=(0.5, 1.0))

    ma_note = f"MA={ma_window}" if ma_window > 1 else "no MA (window=1)"
    ghost_note = " + raw ghost" if show_ghost else ""
    overlay_note = f"  +  {overlay_ylabel} (dashed, right axis)" if overlay and overlay_ylabel else ""
    fig.suptitle(
        f"Threat-schedule lineages — {ylabel}{overlay_note}  ({ma_note}{ghost_note}){title_suffix}",
        fontsize=10,
        y=0.995,
    )
    fig.tight_layout(rect=(0, 0, 1, 0.93))
    parent = os.path.dirname(out_path)
    if parent:
        os.makedirs(parent, exist_ok=True)
    fig.savefig(out_path, dpi=dpi)
    plt.close(fig)
    print(f"[ok] wrote {out_path}  panels={nr}x{nc}")
    return True


# ---------- main ------------------------------------------------------------------

def main() -> int:
    p = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    p.add_argument("--root", default=".",
                   help="Root dir to glob (default '.'). Looks for **/lineage_generations.csv "
                        "under Evolved_results/, Plastic_1_results/, Plastic_2_results/.")
    p.add_argument("--csv", action="append", default=[],
                   help="Explicit lineage_generations.csv path (repeatable). "
                        "When provided, --root globbing is skipped.")
    p.add_argument("--metric", choices=list(METRIC_INFO.keys()), default="fitness")
    p.add_argument("--out", default=None,
                   help="Output PNG path (default: <root>/figures/threat_schedule_<metric>[_overlay_<x>].png).")
    p.add_argument("--ma-window", type=int, default=50)
    p.add_argument("--min-gen", type=int, default=None)
    p.add_argument("--max-gen", type=int, default=None)
    p.add_argument("--no-unified-y", dest="unified_y", action="store_false",
                   help="Per-panel y-axis autoscale (default: shared y-range).")
    p.set_defaults(unified_y=True)
    p.add_argument("--y-margin", type=float, default=0.05)
    p.add_argument("--ymin", type=float, default=None)
    p.add_argument("--ymax", type=float, default=None)
    p.add_argument("--no-ghost-raw", dest="ghost_raw", action="store_false",
                   help="With --ma-window > 1: hide faint raw traces (MA only).")
    p.set_defaults(ghost_raw=True)
    p.add_argument("--overlay", action="store_true",
                   help="Twin Y-axis: --metric on left (solid), --overlay-metric on right (dashed).")
    p.add_argument("--overlay-metric", choices=list(METRIC_INFO.keys()), default="u_drift")
    p.add_argument("--same-yscale", action="store_true",
                   help="With --overlay: force both axes onto one shared range.")
    p.add_argument("--panel-w", type=float, default=_DEFAULT_PANEL_W)
    p.add_argument("--panel-h", type=float, default=_DEFAULT_PANEL_H)
    p.add_argument("--line-lw", type=float, default=_DEFAULT_MA_LW)
    p.add_argument("--raw-lw", type=float, default=_DEFAULT_RAW_LW)
    p.add_argument("--dpi", type=int, default=130)

    # Filters
    p.add_argument("--worlds", default=None,
                   help="Comma-separated subset of worlds (any of easy,normal,hard).")
    p.add_argument("--alphas", default=None,
                   help="Comma-separated subset of alpha values (e.g. 0.3,0.5).")
    p.add_argument("--plasticities", default=None,
                   help="Comma-separated subset of plasticities (any of E2,P1,P2).")
    p.add_argument("--only-sched", action="store_true",
                   help="Keep only runs whose folder name has thrSCHED-...")
    p.add_argument("--include-static", action="store_true",
                   help="Also include static-threat runs (default: include both unless --only-sched).")

    # Regime lines
    p.add_argument("--no-regime-lines", dest="show_regime_lines", action="store_false",
                   help="Hide vertical dashed lines at threat-schedule change points.")
    p.set_defaults(show_regime_lines=True)
    p.add_argument("--no-regime-labels", dest="annotate_regime", action="store_false",
                   help="Hide 'thr->N' labels on regime lines.")
    p.set_defaults(annotate_regime=True)
    p.add_argument("--regime-lines", default=None,
                   help="Comma-separated generation indices for manual regime lines "
                        "(disables auto-detect from num_threats_active).")

    args = p.parse_args()

    if (args.ymin is None) ^ (args.ymax is None):
        print("[error] pass both --ymin and --ymax, or neither", file=sys.stderr)
        return 2
    if args.overlay and args.overlay_metric == args.metric:
        print("[error] --overlay-metric must differ from --metric", file=sys.stderr)
        return 2

    try:
        regime_override = _parse_int_list(args.regime_lines)
    except ValueError as e:
        print(f"[error] --regime-lines: {e}", file=sys.stderr)
        return 2

    runs = _load_runs(args.root, args.csv if args.csv else None)
    if not runs:
        print("[error] no runs discovered", file=sys.stderr)
        return 2

    # Apply filters
    try:
        worlds_filter = _parse_str_list(args.worlds)
        alphas_filter = _parse_float_list(args.alphas)
        plast_filter = _parse_str_list(args.plasticities)
    except ValueError as e:
        print(f"[error] filter parse: {e}", file=sys.stderr)
        return 2

    if worlds_filter:
        runs = [r for r in runs if r["world"] in worlds_filter]
    if alphas_filter:
        runs = [r for r in runs if any(abs(r["alpha"] - a) < 1e-9 for a in alphas_filter)]
    if plast_filter:
        runs = [r for r in runs if r["plast"] in plast_filter]
    if args.only_sched:
        runs = [r for r in runs if r["is_sched"]]
    elif not args.include_static:
        # Default: keep schedule runs if any are present, else fall back to all.
        sched_runs = [r for r in runs if r["is_sched"]]
        if sched_runs:
            runs = sched_runs

    if not runs:
        print("[error] all runs filtered out", file=sys.stderr)
        return 2

    # Build axis ordering from what's actually present
    worlds_present = [w for w in WORLD_ORDER if any(r["world"] == w for r in runs)]
    if not worlds_present:
        worlds_present = sorted({r["world"] for r in runs})
    alphas_present = sorted({r["alpha"] for r in runs})

    metric_col = METRIC_INFO[args.metric]["col"]
    ylabel = METRIC_INFO[args.metric]["ylabel"]
    overlay_metric_col: str | None = None
    overlay_ylabel: str | None = None
    if args.overlay:
        overlay_metric_col = METRIC_INFO[args.overlay_metric]["col"]
        overlay_ylabel = METRIC_INFO[args.overlay_metric]["ylabel"]

    # Default output path
    metric_tag = (
        f"{args.metric}_overlay_{args.overlay_metric}" if args.overlay else args.metric
    )
    if args.out:
        out_path = os.path.abspath(args.out)
    else:
        figdir = os.path.join(os.path.abspath(args.root), "figures")
        out_path = os.path.join(figdir, f"threat_schedule_{metric_tag}.png")

    title_suffix = ""
    if args.only_sched:
        title_suffix = "  [scheduled-threat runs only]"

    ok = _plot_image(
        runs,
        rows_keys=worlds_present,
        cols_keys=alphas_present,
        metric_col=metric_col,
        ylabel=ylabel,
        ma_window=args.ma_window,
        min_gen=args.min_gen,
        max_gen=args.max_gen,
        out_path=out_path,
        title_suffix=title_suffix,
        dpi=args.dpi,
        panel_w=args.panel_w,
        panel_h=args.panel_h,
        ma_lw=args.line_lw,
        raw_lw=args.raw_lw,
        unified_y=args.unified_y,
        y_margin=args.y_margin,
        y_min=args.ymin,
        y_max=args.ymax,
        ghost_raw=args.ghost_raw,
        overlay_metric_col=overlay_metric_col,
        overlay_ylabel=overlay_ylabel,
        same_yscale=args.same_yscale,
        regime_lines_override=regime_override,
        show_regime_lines=args.show_regime_lines,
        annotate_regime=args.annotate_regime,
    )
    if not ok:
        print("[error] no panels produced", file=sys.stderr)
        return 2

    # Discovery summary
    n_sched = sum(1 for r in runs if r["is_sched"])
    n_static = len(runs) - n_sched
    print(f"     discovered={len(runs)}  scheduled={n_sched}  static={n_static}  "
          f"worlds={worlds_present}  alphas={alphas_present}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
