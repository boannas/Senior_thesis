#!/usr/bin/env python3
"""
plot_state_motivation_correlations.py
====================================
Quantify how continuous states correlate with *selected motivation* in tick-level
run logs (`run_*.csv` from `headless_rollout_log.py`), and compare across multiple
world conditions.

Why this exists
---------------
`plot_runlog_conditional_behavior.py` gives thresholded conditional probabilities
like P(sel=Care | child hungry). That is already a *conditional/joint* summary,
but sometimes you want a more general relationship:

  - Does higher hunger correlate with more Care or more Forage?
  - Does fear correlate with Protect?
  - Does low energy correlate with Forage/Self?

We compute, per run log, Spearman correlation between each state value and each
motivation one-hot (m0_sel_*). Then we average correlations across rollout seeds
to get a per-genome summary, and finally take median ± IQR across genomes for each
(condition, plasticity, init).

Outputs (under --out-dir):
  - corr_summary.csv
  - corr_<state>__<mot>.png                 (median±IQR points across conditions)
  - corr_<state>__<mot>_trajectory.png      (line trajectory across ordered conditions)

Notes
-----
Spearman is implemented as rank-Pearson (no scipy). Correlations are computed
over "alive" ticks where both mother and child values are finite (same convention
as other scripts).
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
INIT_COLORS = {"anti_maternal": "#c0392b", "random_uniform": "#2980b9", "pro_maternal": "#27ae60"}
PLAST_TITLES = {"E2": "E2 (no plasticity)", "P1": "P1", "P2": "P2"}
MOTS = ["Forage", "Care", "Self", "Protect"]

RUN_RE = re.compile(r"^(?P<plast>E2|P1|P2)_(?P<init>anti_maternal|random_uniform|pro_maternal)_seed(?P<seed>\d+)$")

DEFAULT_STATE_COLS = [
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
        "axes.grid": True,
        "grid.color": "0.90",
        "grid.linewidth": 0.8,
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


def _spearman(x: np.ndarray, y: np.ndarray) -> float:
    # Spearman = Pearson correlation on ranks (average ranks for ties via pandas)
    x = x.astype(float)
    y = y.astype(float)
    m = np.isfinite(x) & np.isfinite(y)
    x = x[m]
    y = y[m]
    if x.size < 4:
        return float("nan")
    xr = pd.Series(x).rank(method="average").to_numpy(dtype=float)
    yr = pd.Series(y).rank(method="average").to_numpy(dtype=float)
    xs = xr - xr.mean()
    ys = yr - yr.mean()
    den = float(np.sqrt((xs * xs).sum() * (ys * ys).sum()))
    if den <= 0:
        return float("nan")
    return float((xs * ys).sum() / den)


def _load_one_run(csv_path: str, *, state_cols: list[str]) -> dict | None:
    try:
        df = pd.read_csv(csv_path)
    except Exception:
        return None
    needed = ["m0_energy", "c0_hunger"] + [f"m0_sel_{m}" for m in MOTS]
    for c in needed:
        if c not in df.columns:
            return None
    alive = _alive_mask(df)
    n_alive = int(alive.sum())
    if n_alive <= 10:
        return None

    out: dict = {"path": csv_path, "n_alive_ticks": n_alive}
    for sc in state_cols:
        if sc not in df.columns:
            continue
        x = pd.to_numeric(df[sc], errors="coerce").to_numpy(dtype=float)
        for mot in MOTS:
            y = pd.to_numeric(df[f"m0_sel_{mot}"], errors="coerce").fillna(0.0).to_numpy(dtype=float)
            rho = _spearman(x[alive], y[alive])
            out[f"rho_{sc}__{mot}"] = rho
    return out


def _collect_root(root: str, *, condition: str, state_cols: list[str]) -> "pd.DataFrame":
    paths = sorted(glob.glob(os.path.join(root, "*", "run_*.csv")))
    if not paths:
        raise SystemExit(f"No run_*.csv found under {root}/*/")
    rows = []
    skipped = 0
    for p in paths:
        run_dir = os.path.basename(os.path.dirname(p))
        m = RUN_RE.match(run_dir)
        if not m:
            skipped += 1
            continue
        rec = _load_one_run(p, state_cols=state_cols)
        if rec is None:
            skipped += 1
            continue
        rec["condition"] = condition
        rec["run_dir"] = run_dir
        rec["plasticity"] = m.group("plast")
        rec["init"] = m.group("init")
        rec["evolve_seed"] = int(m.group("seed"))
        rows.append(rec)
    if not rows:
        raise SystemExit(f"No usable run logs for condition={condition} (skipped={skipped})")
    df = pd.DataFrame(rows)
    print(f"[ok] {condition}: loaded {len(df)} runs (skipped={skipped})")
    return df


def _per_genome_mean(df: "pd.DataFrame") -> "pd.DataFrame":
    # mean across rollout seeds, per (condition, plasticity, init, evolve_seed)
    key = ["condition", "plasticity", "init", "evolve_seed"]
    metric_cols = [c for c in df.columns if c.startswith("rho_")]
    keep = key + metric_cols
    d = df[keep].copy()
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


def _summarize(pg: "pd.DataFrame", col: str) -> "pd.DataFrame":
    x = pg.dropna(subset=[col]).copy()
    if x.empty:
        return pd.DataFrame(columns=["condition", "plasticity", "init", "n", "median", "q25", "q75"])
    g = x.groupby(["condition", "plasticity", "init"])[col]
    out = pd.concat(
        [
            g.count().rename("n"),
            g.median().rename("median"),
            g.quantile(0.25).rename("q25"),
            g.quantile(0.75).rename("q75"),
        ],
        axis=1,
    ).reset_index()
    return out


def _plot_summary_and_trajectory(summ: "pd.DataFrame", *, label: str, condition_order: list[str], out_dir: str, stem: str) -> None:
    if summ.empty:
        return
    # Points+IQR
    fig, axes = plt.subplots(1, 3, figsize=(12.8, 4.1), sharey=True)
    for ax, plast in zip(axes, PLAST_ORDER):
        subp = summ[summ["plasticity"] == plast]
        for init in INIT_ORDER:
            subi = subp[subp["init"] == init].copy()
            if subi.empty:
                continue
            subi["cond_idx"] = subi["condition"].map({c: i for i, c in enumerate(condition_order)})
            subi = subi.dropna(subset=["cond_idx"])
            xs = subi["cond_idx"].to_numpy(dtype=float)
            ax.vlines(xs, subi["q25"], subi["q75"], color=INIT_COLORS[init], linewidth=2.0, alpha=0.9)
            ax.scatter(xs, subi["median"], s=46, color=INIT_COLORS[init], edgecolors="white", linewidths=0.6)
        ax.axhline(0.0, color="0.35", linewidth=1.0, alpha=0.4)
        ax.set_title(PLAST_TITLES.get(plast, plast))
        ax.set_xticks(np.arange(len(condition_order)))
        ax.set_xticklabels(condition_order, rotation=30, ha="right")
    axes[0].set_ylabel(label + "\nmedian ± IQR (across genomes)")
    fig.suptitle(label + " across conditions", y=1.02, fontsize=12)
    fig.tight_layout()
    p1 = os.path.join(out_dir, f"{stem}.png")
    fig.savefig(p1, dpi=200, bbox_inches="tight")
    plt.close(fig)

    # Trajectory lines
    fig, axes = plt.subplots(1, 3, figsize=(12.8, 4.1), sharey=True)
    x = np.arange(len(condition_order), dtype=float)
    idx = {c: i for i, c in enumerate(condition_order)}
    for ax, plast in zip(axes, PLAST_ORDER):
        subp = summ[summ["plasticity"] == plast]
        for init in INIT_ORDER:
            subi = subp[subp["init"] == init]
            if subi.empty:
                continue
            y_med = np.full(len(condition_order), np.nan, dtype=float)
            y_q25 = np.full(len(condition_order), np.nan, dtype=float)
            y_q75 = np.full(len(condition_order), np.nan, dtype=float)
            for _, r in subi.iterrows():
                c = str(r["condition"])
                if c in idx:
                    j = idx[c]
                    y_med[j] = float(r["median"])
                    y_q25[j] = float(r["q25"])
                    y_q75[j] = float(r["q75"])
            ax.plot(x, y_med, color=INIT_COLORS[init], linewidth=2.0)
            ax.fill_between(x, y_q25, y_q75, color=INIT_COLORS[init], alpha=0.12)
        ax.axhline(0.0, color="0.35", linewidth=1.0, alpha=0.4)
        ax.set_title(PLAST_TITLES.get(plast, plast))
        ax.set_xticks(x)
        ax.set_xticklabels(condition_order, rotation=30, ha="right")
    axes[0].set_ylabel(label + "\nmedian ± IQR (across genomes)")
    fig.suptitle(label + " trajectory (conditions ordered)", y=1.02, fontsize=12)
    fig.tight_layout()
    p2 = os.path.join(out_dir, f"{stem}_trajectory.png")
    fig.savefig(p2, dpi=200, bbox_inches="tight")
    plt.close(fig)


def main(argv: Iterable[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--cond", action="append", required=True, help="Repeatable: label=/path/to/root1,/path/to/root2,...")
    ap.add_argument("--out-dir", required=True)
    ap.add_argument("--condition-order", default="", help="Comma-separated x-axis order (default: input order).")
    ap.add_argument("--intersection-only", action="store_true", help="Restrict to genomes present in all conditions.")
    ap.add_argument("--state-cols", default=",".join(DEFAULT_STATE_COLS), help="Comma-separated list of state columns to correlate.")
    ap.add_argument("--max-plots", type=int, default=60, help="Safety cap on number of (state,mot) plots to write.")
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

    state_cols = [c.strip() for c in args.state_cols.split(",") if c.strip()]

    all_runs = []
    for label in input_order:
        roots = cond_roots[label]
        for root in roots:
            if not os.path.isdir(root):
                print(f"[warn] missing root: {root}", file=sys.stderr)
                continue
            all_runs.append(_collect_root(root, condition=label, state_cols=state_cols))
    if not all_runs:
        print("[error] no runs loaded", file=sys.stderr)
        return 2

    df = pd.concat(all_runs, ignore_index=True)
    pg = _per_genome_mean(df)
    if args.intersection_only:
        pg = _intersection_filter(pg, condition_order)

    rho_cols = sorted([c for c in pg.columns if c.startswith("rho_")])
    if not rho_cols:
        print("[error] no rho_* columns computed (missing state cols?)", file=sys.stderr)
        return 2

    rows = []
    n_written = 0
    for col in rho_cols:
        summ = _summarize(pg, col)
        rows.append(summ.assign(metric=col))
        # cap plots to avoid explosion
        if n_written < int(args.max_plots):
            stem = f"corr_{col.replace('rho_','')}"
            _plot_summary_and_trajectory(summ, label=col.replace("rho_", "Spearman ρ: "), condition_order=condition_order, out_dir=out_dir, stem=stem)
            n_written += 1

    out_csv = os.path.join(out_dir, "corr_summary.csv")
    pd.concat(rows, ignore_index=True).to_csv(out_csv, index=False)
    print("[ok] wrote", out_csv)

    idx = os.path.join(out_dir, "corr_index.txt")
    with open(idx, "w", encoding="utf-8") as f:
        f.write("State–motivation correlation outputs:\n")
        for name in sorted(os.listdir(out_dir)):
            if name.endswith((".png", ".csv", ".txt")):
                f.write(f"  {name}\n")
    print("[ok] wrote", idx)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

