#!/usr/bin/env python3
"""
plot_joint_state_motivation_heatmap.py
=====================================
Build *joint-probability style* heatmaps from tick-level run logs (`run_*.csv`),
e.g. a binned response curve:

  P(sel = Motivation | state in bin)

This complements correlation plots:
  - correlation: one number per run (monotone association)
  - heatmap: full response shape across the state range (nonlinear, thresholds)

Computation
-----------
For each run_*.csv:
  - restrict to "alive ticks" where both mother+child values are finite
  - bin a chosen state column (default c0_hunger)
  - compute per-bin fractions of selected motivations (Forage/Care/Self/Protect)

Then:
  - average across rollout seeds -> per-genome heatmap
  - take median across genomes -> per (condition, plasticity, init) heatmap

Outputs
-------
Under --out-dir:
  - heatmap_<state>__<condition>__<plast>.png      (3 subplots: init columns)
  - heatmap_<state>__<condition>__<plast>.csv      (median probabilities per bin)

Example
-------
python3 plot_joint_state_motivation_heatmap.py \\
  --cond base=FinalGenomeRollouts_normal_seen \\
  --cond easy_threat0=FinalGenomeRollouts_easy_threat0_seed42_49,FinalGenomeRollouts_easy_threat0_seed50_51 \\
  --state c0_hunger --bins 20 \\
  --out-dir paper_figures_joint_heatmaps \\
  --condition-order easy_threat0,base
"""

from __future__ import annotations

import argparse
import glob
import os
import re
import sys
from typing import Iterable

import numpy as np

try:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import pandas as pd
except ImportError as e:
    raise SystemExit(f"Requires pandas+matplotlib: {e}") from e


INIT_ORDER = ["anti_maternal", "random_uniform", "pro_maternal"]
PLAST_ORDER = ["E2", "P1", "P2"]
PLAST_TITLES = {"E2": "E2 (no plasticity)", "P1": "P1", "P2": "P2"}
MOTS = ["Forage", "Care", "Self", "Protect"]

RUN_RE = re.compile(r"^(?P<plast>E2|P1|P2)_(?P<init>anti_maternal|random_uniform|pro_maternal)_seed(?P<seed>\d+)$")

# Common state columns in this repo's run_*.csv logs.
DEFAULT_STATES = [
    "c0_hunger",
    "c0_warmth",
    "c0_injury",
    "m0_energy",
    "m0_stress",
    "m0_fear_threat",
    "m0_closeness_child",
    "m0_bonding",
    "m0_fatigue",
]

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
        "axes.grid": False,
        "figure.facecolor": "white",
        "axes.facecolor": "white",
        "savefig.facecolor": "white",
    }
)


def _parse_kv(spec: str) -> tuple[str, list[str]]:
    if "=" not in spec:
        raise SystemExit(f"Expected label=path, got: {spec!r}")
    k, v = spec.split("=", 1)
    roots = []
    for part in v.split(","):
        part = part.strip()
        if part:
            roots.append(os.path.abspath(part))
    if not roots:
        raise SystemExit(f"Expected non-empty path(s) after '=', got: {spec!r}")
    return k.strip(), roots


def _alive_mask(df: "pd.DataFrame") -> np.ndarray:
    m = pd.to_numeric(df.get("m0_energy"), errors="coerce").to_numpy(dtype=float)
    c = pd.to_numeric(df.get("c0_hunger"), errors="coerce").to_numpy(dtype=float)
    return np.isfinite(m) & np.isfinite(c)


def _bin_edges(x: np.ndarray, *, bins: int, clip_min: float | None, clip_max: float | None) -> np.ndarray:
    x = x[np.isfinite(x)]
    if x.size == 0:
        return np.array([0.0, 1.0], dtype=float)
    lo = float(np.nanmin(x)) if clip_min is None else float(clip_min)
    hi = float(np.nanmax(x)) if clip_max is None else float(clip_max)
    if not np.isfinite(lo) or not np.isfinite(hi) or hi <= lo:
        lo, hi = (0.0, 1.0)
    return np.linspace(lo, hi, int(bins) + 1, dtype=float)


def _per_run_heatmap(df: "pd.DataFrame", *, state_col: str, edges: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """
    Return (P matrix, n_per_bin) where P has shape (4 motivations, n_bins).
    """
    alive = _alive_mask(df)
    x = pd.to_numeric(df.get(state_col), errors="coerce").to_numpy(dtype=float)
    m = alive & np.isfinite(x)
    x = x[m]
    if x.size < 10:
        nb = len(edges) - 1
        return np.full((len(MOTS), nb), np.nan, dtype=float), np.zeros(nb, dtype=int)

    # Bin indices 0..nb-1, ignore outside.
    nb = len(edges) - 1
    b = np.digitize(x, edges, right=False) - 1
    ok = (b >= 0) & (b < nb)
    b = b[ok]
    if b.size == 0:
        return np.full((len(MOTS), nb), np.nan, dtype=float), np.zeros(nb, dtype=int)

    n_per = np.bincount(b, minlength=nb).astype(int)
    P = np.full((len(MOTS), nb), np.nan, dtype=float)
    for mi, mot in enumerate(MOTS):
        y = pd.to_numeric(df.get(f"m0_sel_{mot}"), errors="coerce").fillna(0.0).to_numpy(dtype=float)[m]
        y = y[ok]
        # sum y in each bin / count
        s = np.bincount(b, weights=y, minlength=nb)
        with np.errstate(divide="ignore", invalid="ignore"):
            P[mi, :] = s / np.maximum(1, n_per)
        P[mi, n_per == 0] = np.nan
    return P, n_per


def _collect_per_run(root: str, *, condition: str, state_col: str, edges: np.ndarray) -> "pd.DataFrame":
    paths = sorted(glob.glob(os.path.join(root, "*", "run_*.csv")))
    if not paths:
        raise SystemExit(f"No run_*.csv under {root}/*/")
    rows = []
    skipped = 0
    for p in paths:
        run_dir = os.path.basename(os.path.dirname(p))
        m = RUN_RE.match(run_dir)
        if not m:
            skipped += 1
            continue
        try:
            df = pd.read_csv(p)
        except Exception:
            skipped += 1
            continue
        P, nbin = _per_run_heatmap(df, state_col=state_col, edges=edges)
        rec = {
            "condition": condition,
            "run_dir": run_dir,
            "plasticity": m.group("plast"),
            "init": m.group("init"),
            "evolve_seed": int(m.group("seed")),
        }
        # store as wide columns: p_<mot>_b<i>, n_b<i>
        for bi in range(P.shape[1]):
            rec[f"n_b{bi}"] = int(nbin[bi])
        for mi, mot in enumerate(MOTS):
            for bi in range(P.shape[1]):
                rec[f"p_{mot}_b{bi}"] = float(P[mi, bi]) if np.isfinite(P[mi, bi]) else float("nan")
        rows.append(rec)
    if not rows:
        raise SystemExit(f"No usable runs (skipped={skipped})")
    out = pd.DataFrame(rows)
    print(f"[ok] {condition}: loaded {len(out)} run logs (skipped={skipped})")
    return out


def _per_genome_mean(df: "pd.DataFrame") -> "pd.DataFrame":
    key = ["condition", "plasticity", "init", "evolve_seed"]
    metric_cols = [c for c in df.columns if c.startswith(("p_", "n_b"))]
    d = df[key + metric_cols].copy()
    for c in metric_cols:
        d[c] = pd.to_numeric(d[c], errors="coerce")
    return d.groupby(key, as_index=False).mean(numeric_only=True)


def _intersection_filter(pg: "pd.DataFrame", condition_order: list[str]) -> "pd.DataFrame":
    key_cols = ["plasticity", "init", "evolve_seed"]
    conds = [c for c in condition_order if c in set(pg["condition"].astype(str))]
    if not conds:
        return pg
    counts = (
        pg[pg["condition"].astype(str).isin(conds)][["condition"] + key_cols]
        .drop_duplicates()
        .groupby(key_cols)["condition"]
        .nunique()
        .reset_index(name="n_conditions")
    )
    keep = counts[counts["n_conditions"] == len(conds)][key_cols]
    if keep.empty:
        print("[warn] intersection empty; not filtering", file=sys.stderr)
        return pg
    out = pg.merge(keep, on=key_cols, how="inner")
    return out


def _median_heatmap(pg: "pd.DataFrame", *, plast: str, init: str, n_bins: int) -> tuple[np.ndarray, np.ndarray]:
    """
    Returns (P_med, n_med) where P_med is shape (4, n_bins) median across genomes,
    and n_med is median alive-ticks-per-bin (from n_b* columns, averaged over rollouts).
    """
    sub = pg[(pg["plasticity"] == plast) & (pg["init"] == init)]
    if sub.empty:
        return np.full((len(MOTS), n_bins), np.nan, dtype=float), np.full(n_bins, np.nan, dtype=float)

    P = np.full((len(MOTS), n_bins), np.nan, dtype=float)
    for mi, mot in enumerate(MOTS):
        cols = [f"p_{mot}_b{bi}" for bi in range(n_bins) if f"p_{mot}_b{bi}" in sub.columns]
        if len(cols) != n_bins:
            # missing columns => likely wrong bin count
            continue
        arr = sub[cols].to_numpy(dtype=float)
        P[mi, :] = np.nanmedian(arr, axis=0)

    ncols = [f"n_b{bi}" for bi in range(n_bins) if f"n_b{bi}" in sub.columns]
    if len(ncols) == n_bins:
        n_med = np.nanmedian(sub[ncols].to_numpy(dtype=float), axis=0)
    else:
        n_med = np.full(n_bins, np.nan, dtype=float)
    return P, n_med


def _plot_condition_heatmaps(
    pg: "pd.DataFrame",
    *,
    condition: str,
    edges: np.ndarray,
    out_dir: str,
    state_col: str,
) -> None:
    n_bins = len(edges) - 1
    bin_centers = 0.5 * (edges[:-1] + edges[1:])
    xlabels = [f"{v:.1f}" for v in bin_centers]

    for plast in PLAST_ORDER:
        fig, axes = plt.subplots(1, 3, figsize=(13.2, 4.0), sharey=True)
        for ax, init in zip(axes, INIT_ORDER):
            sub = pg[pg["condition"] == condition]
            P, n_med = _median_heatmap(sub, plast=plast, init=init, n_bins=n_bins)
            im = ax.imshow(P, aspect="auto", vmin=0.0, vmax=1.0, cmap="viridis")
            ax.set_title(init.replace("_", " "))
            ax.set_yticks(np.arange(len(MOTS)))
            ax.set_yticklabels(MOTS)
            # reduce x ticks for readability
            step = max(1, int(n_bins / 8))
            xt = np.arange(0, n_bins, step)
            ax.set_xticks(xt)
            ax.set_xticklabels([xlabels[i] for i in xt], rotation=30, ha="right")
            ax.set_xlabel(f"{state_col} bin center")
            # Small annotation: median counts per bin (overall)
            if np.isfinite(n_med).any():
                ax.text(
                    0.99,
                    0.02,
                    f"med n/bin≈{np.nanmedian(n_med):.0f}",
                    transform=ax.transAxes,
                    ha="right",
                    va="bottom",
                    fontsize=8,
                    color="white",
                    bbox=dict(boxstyle="round,pad=0.2", facecolor="black", alpha=0.25, linewidth=0),
                )

        fig.suptitle(f"{condition}: P(sel=motivation | {state_col} bin) — {PLAST_TITLES.get(plast, plast)}", y=1.02)
        cbar = fig.colorbar(im, ax=axes.ravel().tolist(), shrink=0.92, pad=0.02)
        cbar.set_label("probability")
        fig.tight_layout()
        out_png = os.path.join(out_dir, f"heatmap_{state_col}__{condition}__{plast}.png")
        fig.savefig(out_png, dpi=200, bbox_inches="tight")
        plt.close(fig)

        # also write CSV for this plast/condition: one row per init×motivation×bin
        rows = []
        subc = pg[pg["condition"] == condition]
        for init in INIT_ORDER:
            P, n_med = _median_heatmap(subc, plast=plast, init=init, n_bins=n_bins)
            for mi, mot in enumerate(MOTS):
                for bi in range(n_bins):
                    rows.append(
                        {
                            "condition": condition,
                            "plasticity": plast,
                            "init": init,
                            "motivation": mot,
                            "bin_idx": bi,
                            "bin_lo": float(edges[bi]),
                            "bin_hi": float(edges[bi + 1]),
                            "p_median": float(P[mi, bi]) if np.isfinite(P[mi, bi]) else float("nan"),
                            "n_alive_ticks_median": float(n_med[bi]) if np.isfinite(n_med[bi]) else float("nan"),
                        }
                    )
        out_csv = os.path.join(out_dir, f"heatmap_{state_col}__{condition}__{plast}.csv")
        pd.DataFrame(rows).to_csv(out_csv, index=False)


def main(argv: Iterable[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--cond", action="append", required=True, help="Repeatable: label=/path/to/root1,/path/to/root2,...")
    ap.add_argument("--out-dir", required=True)
    ap.add_argument(
        "--state",
        action="append",
        default=[],
        help="Repeatable state column to bin (e.g. --state c0_hunger --state m0_energy). Use --state all for all default states.",
    )
    ap.add_argument("--bins", type=int, default=20)
    ap.add_argument("--clip-min", type=float, default=None)
    ap.add_argument("--clip-max", type=float, default=None)
    ap.add_argument("--condition-order", default="", help="Comma-separated order of condition labels (default: input order).")
    ap.add_argument("--intersection-only", action="store_true", help="Restrict to genomes present in all conditions.")
    args = ap.parse_args(list(argv) if argv is not None else None)

    out_dir = os.path.abspath(args.out_dir)
    os.makedirs(out_dir, exist_ok=True)

    cond_roots: dict[str, list[str]] = {}
    input_order: list[str] = []
    for raw in args.cond:
        label, roots = _parse_kv(raw)
        if label not in cond_roots:
            cond_roots[label] = []
            input_order.append(label)
        cond_roots[label].extend(roots)

    if args.condition_order.strip():
        condition_order = [c.strip() for c in args.condition_order.split(",") if c.strip()]
        for c in input_order:
            if c not in condition_order:
                condition_order.append(c)
    else:
        condition_order = input_order

    # Resolve which states to plot.
    states = [s.strip() for s in (args.state or []) if s.strip()]
    if not states:
        states = ["c0_hunger"]
    if any(s.lower() == "all" for s in states):
        states = list(DEFAULT_STATES)

    # Build per-state edges from base condition if present, else from first root.
    sample_root = cond_roots["base"][0] if "base" in cond_roots else cond_roots[input_order[0]][0]
    sample_paths = sorted(glob.glob(os.path.join(sample_root, "*", "run_*.csv")))[:60]

    for state_col in states:
        # Estimate edges for this state.
        xs = []
        for p in sample_paths:
            try:
                d = pd.read_csv(p, usecols=[state_col, "m0_energy", "c0_hunger"])
            except Exception:
                continue
            m = _alive_mask(d)
            x = pd.to_numeric(d.get(state_col), errors="coerce").to_numpy(dtype=float)
            xs.append(x[m])
        xcat = np.concatenate(xs) if xs else np.array([], dtype=float)
        edges = _bin_edges(xcat, bins=int(args.bins), clip_min=args.clip_min, clip_max=args.clip_max)

        all_runs = []
        for label in input_order:
            for root in cond_roots[label]:
                if not os.path.isdir(root):
                    print(f"[warn] missing root: {root}", file=sys.stderr)
                    continue
                all_runs.append(_collect_per_run(root, condition=label, state_col=state_col, edges=edges))
        if not all_runs:
            print("[warn] no runs loaded for state:", state_col, file=sys.stderr)
            continue

        df = pd.concat(all_runs, ignore_index=True)
        pg = _per_genome_mean(df)
        if args.intersection_only:
            pg = _intersection_filter(pg, condition_order)

        for cond in condition_order:
            if cond not in set(pg["condition"].astype(str)):
                continue
            _plot_condition_heatmaps(pg, condition=cond, edges=edges, out_dir=out_dir, state_col=state_col)

    # Index
    idx = os.path.join(out_dir, "joint_heatmap_index.txt")
    with open(idx, "w", encoding="utf-8") as f:
        f.write("Joint (binned conditional) heatmap outputs:\n")
        for name in sorted(os.listdir(out_dir)):
            if name.endswith((".png", ".csv", ".txt")):
                f.write(f"  {name}\n")
    print("[ok] wrote", idx)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

