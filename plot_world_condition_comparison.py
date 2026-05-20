#!/usr/bin/env python3
"""
plot_world_condition_comparison.py
==================================
Compare **multiple rollout conditions** (different world configs) using the same
conditional-behavior metrics computed by `plot_runlog_conditional_behavior.py`.

Typical use-case in this repo:
  - base:  FinalGenomeRollouts_normal_seen
  - unseen: FinalGenomeRollouts_unseen_food30 / unseen_hard
  - easy:  FinalGenomeRollouts_easy_threat0 / easy_food_10

This script:
  1) (optional) ensures each rollout root has a metrics CSV by running
     `plot_runlog_conditional_behavior.py --root ... --extended`.
  2) reads each root's `conditional_metrics_single.csv` (extended columns allowed),
  3) aggregates per-genome means (over rollout seeds), then
  4) plots per-(condition, plasticity, init) summaries as median ± IQR.

It also optionally plots **stage trajectories** if you pass stage CSVs produced
by `plot_runlog_conditional_behavior.py --stage-root ...` (e.g. StageRollouts_normal).

Outputs go under --out-dir:
  - compare_outcomes.png
  - compare_responsiveness.png
  - compare_states.png
  - compare_extended_conditions.png (optional; only if extended columns exist)
  - compare_stage_trajectories_<metric>.png (if stage CSVs provided)
  - condition_summary.csv (tidy summary table)

Dependencies: pandas, matplotlib. (No scipy.)
"""

from __future__ import annotations

import argparse
import os
import subprocess
import sys
from dataclasses import dataclass
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


CORE_OUTCOME_METRICS = [
    ("child_ttd_norm", "child TTD (normalized)"),
    ("mother_ttd_norm", "mother TTD (normalized)"),
]

CORE_RESP_METRICS = [
    ("P_maternal_given_hungry", "P(Maternal | child hungry)"),
    ("P_forage_given_hungry", "P(Forage | child hungry)"),
    ("P_protect_given_fear", "P(Protect | mother afraid)"),
]

CORE_STATE_METRICS = [
    ("closeness_mean_alive", "closeness mean (alive)"),
    ("injury_mean_alive", "child injury mean (alive)"),
    ("fear_mean_alive", "mother fear mean (alive)"),
    ("energy_mean_alive", "mother energy mean (alive)"),
    ("stress_mean_alive", "mother stress mean (alive)"),
    ("warmth_mean_alive", "child warmth mean (alive)"),
    ("bonding_mean_alive", "bonding mean (alive)"),
    ("fatigue_mean_alive", "fatigue mean (alive)"),
]


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


def _maybe_run_conditional_metrics(root: str, *, out_dir: str, extended: bool, force: bool) -> str:
    """
    Ensure conditional_metrics_single.csv exists in out_dir.
    Returns CSV path.
    """
    os.makedirs(out_dir, exist_ok=True)
    csv_path = os.path.join(out_dir, "conditional_metrics_single.csv")
    if (not force) and os.path.isfile(csv_path):
        return csv_path

    cmd = [
        sys.executable,
        os.path.join(os.path.dirname(__file__), "plot_runlog_conditional_behavior.py"),
        "--root",
        root,
        "--out-dir",
        out_dir,
    ]
    if extended:
        cmd.append("--extended")

    print("[info] building conditional metrics:", " ".join(cmd))
    p = subprocess.run(cmd, check=False)
    if p.returncode != 0:
        # Common case: rollout root exists but has no run_*.csv yet.
        print(f"[warn] conditional metrics build failed for root={root} (exit={p.returncode})", file=sys.stderr)
    if not os.path.isfile(csv_path):
        # Do not hard-fail; caller can decide to skip this condition.
        print(f"[warn] metrics CSV not found after attempted build: {csv_path}", file=sys.stderr)
    return csv_path


def _read_metrics_csv(csv_path: str, *, condition: str) -> "pd.DataFrame":
    df = pd.read_csv(csv_path)
    df["condition"] = str(condition)
    return df


def _per_genome_means(df: "pd.DataFrame", metric_cols: list[str]) -> "pd.DataFrame":
    needed = {"plasticity", "init", "evolve_seed"}
    missing = needed - set(df.columns)
    if missing:
        raise SystemExit(f"metrics CSV missing columns {missing}")

    keep = ["condition", "plasticity", "init", "evolve_seed"] + metric_cols
    keep = [c for c in keep if c in df.columns]
    df = df[keep].copy()

    # numeric coercion for metrics
    for c in metric_cols:
        if c in df.columns:
            df[c] = pd.to_numeric(df[c], errors="coerce")

    gcols = ["condition", "plasticity", "init", "evolve_seed"]
    agg = df.groupby(gcols, as_index=False).mean(numeric_only=True)
    return agg


def _intersection_filter(pg: "pd.DataFrame", condition_order: list[str]) -> "pd.DataFrame":
    """
    Keep only genomes that appear in *every* condition, intersecting on
    (plasticity, init, evolve_seed). This makes medians comparable across
    conditions even if some roots are missing runs.
    """
    key_cols = ["plasticity", "init", "evolve_seed"]
    need = set(key_cols + ["condition"])
    missing = need - set(pg.columns)
    if missing:
        raise SystemExit(f"intersection filter requires columns {missing}")

    conds = [c for c in condition_order if c in set(pg["condition"].astype(str))]
    if not conds:
        return pg

    # Count in how many conditions each genome appears.
    counts = (
        pg[pg["condition"].astype(str).isin(conds)][["condition"] + key_cols]
        .drop_duplicates()
        .groupby(key_cols)["condition"]
        .nunique()
        .reset_index(name="n_conditions")
    )
    keep = counts[counts["n_conditions"] == len(conds)][key_cols]
    if keep.empty:
        print("[warn] intersection is empty; not filtering", file=sys.stderr)
        return pg

    out = pg.merge(keep, on=key_cols, how="inner")
    dropped = len(pg) - len(out)
    if dropped > 0:
        print(f"[info] intersection-only: dropped {dropped} rows not present in all conditions", file=sys.stderr)
    return out


def _summarize_cells(pg: "pd.DataFrame", metric: str) -> "pd.DataFrame":
    x = pg.dropna(subset=[metric]).copy()
    if x.empty:
        return pd.DataFrame(columns=["condition", "plasticity", "init", "n", "median", "q25", "q75"])
    gcols = ["condition", "plasticity", "init"]
    g = x.groupby(gcols)[metric]
    out = pd.concat(
        [
            g.count().rename("n"),
            g.median().rename("median"),
            g.quantile(0.25).rename("q25"),
            g.quantile(0.75).rename("q75"),
        ],
        axis=1,
    ).reset_index()
    return out.sort_values(["condition", "plasticity", "init"])


def _plot_metric_condition_summary(
    summ: "pd.DataFrame",
    *,
    metric_label: str,
    condition_order: list[str],
    out_png: str,
    title: str,
) -> None:
    if summ.empty:
        print("[warn] empty summary for", metric_label, "skip", out_png, file=sys.stderr)
        return

    fig, axes = plt.subplots(1, 3, figsize=(12.8, 4.1), sharey=True)
    for ax, plast in zip(axes, PLAST_ORDER):
        subp = summ[summ["plasticity"] == plast]
        if subp.empty:
            ax.set_title(PLAST_TITLES.get(plast, plast))
            ax.set_xticks(range(len(condition_order)))
            ax.set_xticklabels(condition_order, rotation=30, ha="right")
            continue

        # points with offsets per init
        base_x = np.arange(len(condition_order), dtype=float)
        offsets = {"anti_maternal": -0.22, "random_uniform": 0.0, "pro_maternal": 0.22}
        for init in INIT_ORDER:
            subi = subp[subp["init"] == init].copy()
            if subi.empty:
                continue
            subi["cond_idx"] = subi["condition"].map({c: i for i, c in enumerate(condition_order)})
            subi = subi.dropna(subset=["cond_idx"])
            xs = subi["cond_idx"].to_numpy(dtype=float) + offsets[init]
            med = subi["median"].to_numpy(dtype=float)
            q25 = subi["q25"].to_numpy(dtype=float)
            q75 = subi["q75"].to_numpy(dtype=float)

            ax.vlines(xs, q25, q75, color=INIT_COLORS[init], linewidth=2.0, alpha=0.9)
            ax.scatter(xs, med, s=46, color=INIT_COLORS[init], edgecolors="white", linewidths=0.6, zorder=3, label=init)

        ax.axhline(0.0, color="0.35", linewidth=1.0, alpha=0.4)
        ax.set_title(PLAST_TITLES.get(plast, plast))
        ax.set_xticks(base_x)
        ax.set_xticklabels(condition_order, rotation=30, ha="right")

    axes[0].set_ylabel(metric_label + "\nmedian ± IQR (per-genome mean over rollout seeds)")
    # Deduplicate legend
    handles = []
    labels = []
    for init in INIT_ORDER:
        handles.append(plt.Line2D([0], [0], marker="o", color="none", markerfacecolor=INIT_COLORS[init], markeredgecolor="white", markersize=7))
        labels.append(init.replace("_", " "))
    fig.legend(handles, labels, loc="center left", bbox_to_anchor=(1.01, 0.5), frameon=False, fontsize=9, title="init")
    fig.suptitle(title, y=1.02, fontsize=12)
    fig.tight_layout()
    fig.savefig(out_png, dpi=200, bbox_inches="tight")
    plt.close(fig)
    print("[ok] wrote", out_png)


def _plot_metric_condition_trajectory(
    summ: "pd.DataFrame",
    *,
    metric_label: str,
    condition_order: list[str],
    out_png: str,
    title: str,
) -> None:
    """
    Line trajectory across conditions (x-axis ordered by condition_order),
    per plasticity panel, one line per init. Shaded band = IQR across genomes.
    """
    if summ.empty:
        print("[warn] empty summary for", metric_label, "skip", out_png, file=sys.stderr)
        return

    fig, axes = plt.subplots(1, 3, figsize=(12.8, 4.1), sharey=True)
    x = np.arange(len(condition_order), dtype=float)
    for ax, plast in zip(axes, PLAST_ORDER):
        subp = summ[summ["plasticity"] == plast]
        if subp.empty:
            ax.set_title(PLAST_TITLES.get(plast, plast))
            ax.set_xticks(x)
            ax.set_xticklabels(condition_order, rotation=30, ha="right")
            continue

        for init in INIT_ORDER:
            subi = subp[subp["init"] == init]
            if subi.empty:
                continue
            # align in condition_order
            idx = {c: i for i, c in enumerate(condition_order)}
            y_med = np.full(len(condition_order), np.nan, dtype=float)
            y_q25 = np.full(len(condition_order), np.nan, dtype=float)
            y_q75 = np.full(len(condition_order), np.nan, dtype=float)
            for _, r in subi.iterrows():
                c = str(r["condition"])
                if c not in idx:
                    continue
                j = idx[c]
                y_med[j] = float(r["median"])
                y_q25[j] = float(r["q25"])
                y_q75[j] = float(r["q75"])
            ax.plot(x, y_med, color=INIT_COLORS[init], linewidth=2.0, label=init)
            ax.fill_between(x, y_q25, y_q75, color=INIT_COLORS[init], alpha=0.12)

        ax.set_title(PLAST_TITLES.get(plast, plast))
        ax.set_xticks(x)
        ax.set_xticklabels(condition_order, rotation=30, ha="right")

    axes[0].set_ylabel(metric_label + "\nmedian ± IQR (across genomes)")
    fig.suptitle(title, y=1.02, fontsize=12)
    handles = []
    labels = []
    for init in INIT_ORDER:
        handles.append(plt.Line2D([0], [0], color=INIT_COLORS[init], linewidth=2.0))
        labels.append(init.replace("_", " "))
    fig.legend(handles, labels, loc="center left", bbox_to_anchor=(1.01, 0.5), frameon=False, fontsize=9, title="init")
    fig.tight_layout()
    fig.savefig(out_png, dpi=200, bbox_inches="tight")
    plt.close(fig)
    print("[ok] wrote", out_png)


def _available_extended_psel_cols(df: "pd.DataFrame") -> list[str]:
    return sorted([c for c in df.columns if c.startswith("P_sel_") and "_given_" in c])


def _plot_group(metrics: list[tuple[str, str]], pg: "pd.DataFrame", *, out_dir: str, condition_order: list[str], prefix: str) -> list["pd.DataFrame"]:
    summaries = []
    for col, lab in metrics:
        if col not in pg.columns:
            continue
        summ = _summarize_cells(pg, col)
        out_png = os.path.join(out_dir, f"{prefix}_{col}.png")
        _plot_metric_condition_summary(
            summ,
            metric_label=lab,
            condition_order=condition_order,
            out_png=out_png,
            title=f"{lab} across conditions (per-genome means)",
        )
        out_traj = os.path.join(out_dir, f"{prefix}_{col}_trajectory.png")
        _plot_metric_condition_trajectory(
            summ,
            metric_label=lab,
            condition_order=condition_order,
            out_png=out_traj,
            title=f"{lab} trajectory across conditions (easy → hard ordering)",
        )
        summ["metric"] = col
        summaries.append(summ)
    return summaries


def _spearman_series(x: "pd.Series", y: "pd.Series") -> float:
    """Spearman rho via rank-Pearson (no scipy)."""
    x = pd.to_numeric(x, errors="coerce")
    y = pd.to_numeric(y, errors="coerce")
    m = x.notna() & y.notna()
    if int(m.sum()) < 4:
        return float("nan")
    xr = x[m].rank(method="average")
    yr = y[m].rank(method="average")
    xs = xr - xr.mean()
    ys = yr - yr.mean()
    den = float(np.sqrt(float((xs * xs).sum()) * float((ys * ys).sum())))
    if den <= 0:
        return float("nan")
    return float((xs * ys).sum() / den)


def _write_cross_metric_correlations(pg: "pd.DataFrame", *, out_dir: str) -> None:
    """
    Correlate per-genome mean states with per-genome mean behavior/outcomes
    *across genomes*, within each (condition, plasticity, init).

    This does NOT use tick-level joint distributions; it answers:
      "Across genomes, do those with higher mean X also have higher mean Y?"
    """
    # pick a small, interpretable set
    states = [
        "closeness_mean_alive",
        "injury_mean_alive",
        "fear_mean_alive",
        "energy_mean_alive",
        "stress_mean_alive",
        "warmth_mean_alive",
        "bonding_mean_alive",
        "fatigue_mean_alive",
    ]
    behaviors = [
        "P_maternal_given_hungry",
        "P_forage_given_hungry",
        "P_protect_given_fear",
        "child_ttd_norm",
        "mother_ttd_norm",
    ]
    states = [c for c in states if c in pg.columns]
    behaviors = [c for c in behaviors if c in pg.columns]
    if not states or not behaviors:
        return

    rows = []
    for (cond, plast, init), sub in pg.groupby(["condition", "plasticity", "init"], dropna=False):
        for sx in states:
            for by in behaviors:
                rho = _spearman_series(sub[sx], sub[by])
                n = int(pd.to_numeric(sub[[sx, by]].notna().all(axis=1), errors="coerce").sum())
                rows.append(
                    {
                        "condition": str(cond),
                        "plasticity": str(plast),
                        "init": str(init),
                        "x": sx,
                        "y": by,
                        "spearman_rho": float(rho),
                        "n_genomes": int(n),
                    }
                )
    out_csv = os.path.join(out_dir, "cross_metric_correlations.csv")
    pd.DataFrame(rows).to_csv(out_csv, index=False)
    print("[ok] wrote", out_csv)


def _plot_stage_trajectories(stage_csvs: dict[str, str], *, out_dir: str) -> None:
    # Concatenate and reuse the existing stage plots style: median ± IQR per genome
    parts = []
    for stage, path in stage_csvs.items():
        if not os.path.isfile(path):
            print("[warn] stage CSV missing:", path, file=sys.stderr)
            continue
        df = pd.read_csv(path)
        if "stage" not in df.columns:
            df["stage"] = stage
        parts.append(df)
    if not parts:
        return
    df_all = pd.concat(parts, ignore_index=True)

    metrics = CORE_OUTCOME_METRICS + CORE_RESP_METRICS + CORE_STATE_METRICS
    metric_cols = [c for c, _ in metrics if c in df_all.columns]
    if "stage" not in df_all.columns:
        print("[warn] stage trajectories requested but no 'stage' column found", file=sys.stderr)
        return

    # Per-genome means, but *keeping stage* as a group key.
    keep = ["stage", "plasticity", "init", "evolve_seed"] + metric_cols
    keep = [c for c in keep if c in df_all.columns]
    df_all = df_all[keep].copy()
    for c in metric_cols:
        if c in df_all.columns:
            df_all[c] = pd.to_numeric(df_all[c], errors="coerce")
    pg = df_all.groupby(["stage", "plasticity", "init", "evolve_seed"], as_index=False).mean(numeric_only=True)

    stage_order = list(stage_csvs.keys())
    for col, lab in metrics:
        if col not in pg.columns or "stage" not in pg.columns:
            continue

        fig, axes = plt.subplots(1, 3, figsize=(12.8, 4.1), sharey=True)
        for ax, plast in zip(axes, PLAST_ORDER):
            subp = pg[pg["plasticity"] == plast]
            if subp.empty:
                ax.set_title(PLAST_TITLES.get(plast, plast))
                continue

            for init in INIT_ORDER:
                subi = subp[subp["init"] == init]
                if subi.empty:
                    continue
                # per-stage per-genome means already; now summarize across genomes
                rows = []
                for st in stage_order:
                    xs = subi[subi["stage"] == st][col].dropna().to_numpy(dtype=float)
                    if xs.size == 0:
                        rows.append((st, np.nan, np.nan, np.nan))
                    else:
                        rows.append((st, float(np.median(xs)), float(np.percentile(xs, 25)), float(np.percentile(xs, 75))))
                med = np.array([r[1] for r in rows], dtype=float)
                q25 = np.array([r[2] for r in rows], dtype=float)
                q75 = np.array([r[3] for r in rows], dtype=float)
                x = np.arange(len(stage_order), dtype=float)
                ax.plot(x, med, color=INIT_COLORS[init], linewidth=2.0, label=init)
                ax.fill_between(x, q25, q75, color=INIT_COLORS[init], alpha=0.12)
            ax.set_title(PLAST_TITLES.get(plast, plast))
            ax.set_xticks(np.arange(len(stage_order)))
            ax.set_xticklabels(stage_order)

        axes[0].set_ylabel(lab + "\nmedian ± IQR (across genomes)")
        fig.suptitle(f"{lab} across stages (normal world)", y=1.02, fontsize=12)
        handles = []
        labels = []
        for init in INIT_ORDER:
            handles.append(plt.Line2D([0], [0], color=INIT_COLORS[init], linewidth=2.0))
            labels.append(init.replace("_", " "))
        fig.legend(handles, labels, loc="center left", bbox_to_anchor=(1.01, 0.5), frameon=False, fontsize=9, title="init")
        fig.tight_layout()
        out_png = os.path.join(out_dir, f"stage_trajectory_{col}.png")
        fig.savefig(out_png, dpi=200, bbox_inches="tight")
        plt.close(fig)
        print("[ok] wrote", out_png)


def main(argv: Iterable[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument(
        "--cond",
        action="append",
        required=True,
        help="Repeatable: label=/path/to/FinalGenomeRollouts_* root",
    )
    ap.add_argument("--out-dir", required=True)
    ap.add_argument("--ensure-metrics", action="store_true", help="If set, compute missing conditional_metrics_single.csv via plot_runlog_conditional_behavior.py")
    ap.add_argument("--force-recompute", action="store_true", help="Recompute metrics even if CSV exists (only with --ensure-metrics)")
    ap.add_argument("--no-extended", action="store_true", help="Do not request --extended when computing metrics (faster; fewer columns)")
    ap.add_argument(
        "--metrics-subdir",
        default="figures_conditional_extended",
        help="Subdir under each root to read/write conditional_metrics_single.csv",
    )
    ap.add_argument(
        "--stage-csv",
        action="append",
        default=[],
        help="Optional: repeatable stage=path for stage trajectories (e.g. initial=.../conditional_metrics_initial.csv)",
    )
    ap.add_argument(
        "--condition-order",
        default="",
        help="Optional comma-separated order of condition labels for x-axis (default: input order).",
    )
    ap.add_argument(
        "--intersection-only",
        action="store_true",
        help="If set, restrict to genomes present in every condition (intersection on plasticity×init×evolve_seed).",
    )
    args = ap.parse_args(list(argv) if argv is not None else None)

    out_dir = os.path.abspath(args.out_dir)
    os.makedirs(out_dir, exist_ok=True)

    # Allow duplicate labels by merging their roots; also allow label=path1,path2,...
    cond_roots: dict[str, list[str]] = {}
    input_order: list[str] = []
    for raw in args.cond:
        label, roots = _parse_kv(raw)
        if label not in cond_roots:
            cond_roots[label] = []
            input_order.append(label)
        cond_roots[label].extend(roots)

    cond_labels = input_order
    cond_label_set = set(cond_labels)

    if args.condition_order.strip():
        condition_order = [c.strip() for c in args.condition_order.split(",") if c.strip()]
        missing = [c for c in condition_order if c not in cond_label_set]
        if missing:
            raise SystemExit(f"--condition-order mentions unknown labels: {missing}")
        # keep any omitted labels at end (in input order)
        for c in cond_labels:
            if c not in condition_order:
                condition_order.append(c)
    else:
        condition_order = cond_labels

    # Load (or compute) metrics for each condition
    dfs = []
    for label in cond_labels:
        roots = cond_roots.get(label, [])
        if not roots:
            continue
        any_loaded = False
        for root in roots:
            if not os.path.isdir(root):
                print(f"[warn] missing root for {label}: {root}", file=sys.stderr)
                continue
            metrics_dir = os.path.join(root, args.metrics_subdir)
            csv_path = os.path.join(metrics_dir, "conditional_metrics_single.csv")
            if args.ensure_metrics:
                csv_path = _maybe_run_conditional_metrics(
                    root,
                    out_dir=metrics_dir,
                    extended=not args.no_extended,
                    force=bool(args.force_recompute),
                )
            if not os.path.isfile(csv_path):
                print(
                    f"[warn] metrics CSV missing for {label}: {csv_path} (use --ensure-metrics)",
                    file=sys.stderr,
                )
                continue
            dfs.append(_read_metrics_csv(csv_path, condition=label))
            any_loaded = True
        if not any_loaded:
            print(f"[warn] no metrics loaded for condition label={label}", file=sys.stderr)

    if not dfs:
        print("[error] no condition metrics CSVs found", file=sys.stderr)
        return 2

    df_all = pd.concat(dfs, ignore_index=True)

    # Choose metric columns available in this combined table.
    metrics = CORE_OUTCOME_METRICS + CORE_RESP_METRICS + CORE_STATE_METRICS
    metric_cols = [c for c, _ in metrics if c in df_all.columns]
    pg = _per_genome_means(df_all, metric_cols + _available_extended_psel_cols(df_all))
    if args.intersection_only:
        pg = _intersection_filter(pg, condition_order)

    # Write a tidy summary for core metrics.
    summaries = []
    summaries.extend(_plot_group(CORE_OUTCOME_METRICS, pg, out_dir=out_dir, condition_order=condition_order, prefix="compare_outcome"))
    summaries.extend(_plot_group(CORE_RESP_METRICS, pg, out_dir=out_dir, condition_order=condition_order, prefix="compare_resp"))
    summaries.extend(_plot_group(CORE_STATE_METRICS, pg, out_dir=out_dir, condition_order=condition_order, prefix="compare_state"))

    # Extended P_sel_* metrics: too many → keep a curated subset if present.
    psel = _available_extended_psel_cols(pg)
    # Prefer "given_child_hungry", "given_child_cold", "given_child_injured" for the Care/Protect tradeoff story.
    curated = []
    for cond in ("child_hungry", "child_cold", "child_injured"):
        for mot in ("Care", "Protect", "Forage", "Self"):
            key = f"P_sel_{mot}_given_{cond}"
            if key in psel:
                curated.append(key)
    if curated:
        for key in curated:
            summ = _summarize_cells(pg, key)
            _plot_metric_condition_summary(
                summ,
                metric_label=key,
                condition_order=condition_order,
                out_png=os.path.join(out_dir, f"compare_extended_{key}.png"),
                title=f"{key} across conditions (per-genome means)",
            )
            summ["metric"] = key
            summaries.append(summ)

    if summaries:
        out_csv = os.path.join(out_dir, "condition_summary.csv")
        pd.concat(summaries, ignore_index=True).to_csv(out_csv, index=False)
        print("[ok] wrote", out_csv)

    _write_cross_metric_correlations(pg, out_dir=out_dir)

    # Optional stage trajectories (normal world): provide stage CSVs from plot_runlog_conditional_behavior.py --stage-root.
    if args.stage_csv:
        stage_csvs = {k: v[0] for (k, v) in (_parse_kv(s) for s in args.stage_csv)}
        _plot_stage_trajectories(stage_csvs, out_dir=out_dir)

    # Index file for convenience.
    idx_path = os.path.join(out_dir, "world_condition_compare_index.txt")
    with open(idx_path, "w", encoding="utf-8") as f:
        f.write("World condition comparison outputs:\n")
        for name in sorted(os.listdir(out_dir)):
            if name.endswith((".png", ".csv", ".txt")):
                f.write(f"  {name}\n")
    print("[ok] wrote", idx_path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

