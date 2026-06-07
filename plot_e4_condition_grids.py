#!/usr/bin/env python3
"""
plot_e4_condition_grids.py
==========================
E4 rollout comparison (Exp3-style trajectories): E2 | P1 | P2 panels, anti/random/pro
lines, median ± IQR across genomes.

Default metrics (one PNG each):
  Mother: energy, fatigue, mother_ttd_norm
  Child: hunger / warmth / injury (mean alive), child_ttd_norm
  Conditional: P(Forage|hungry), P(Care|cold), P(Protect|injured)

Output layout (under --out-dir), one PNG per metric:
  by_world/easy|normal|hard/  — x-axis food 5 / 15 / 25
  food_5/, food_15/, food_25/  — x-axis easy / normal / hard
  all_conditions/             — all 9 world×food points

Use --skip-by-world to omit the by_world folders.
Use --combined-grid for legacy multi-metric grid PNGs under _combined_grids/.

Expects rollout roots:
  E4_easy_5, E4_easy_15, E4_easy_25,
  E4_normal_5, E4_normal_15, E4_normal_25,
  E4_hard_5, E4_hard_15, E4_hard_25

Each root should contain <E2|P1|P2>_<init>_seedN/run_*.csv.
Metrics CSV: <root>/figures_conditional_extended/conditional_metrics_single.csv
(use --ensure-metrics to build on first run).

Usage
-----
  python3 plot_e4_condition_grids.py \\
      --repo-root . \\
      --out-dir E4_figures/condition_grids \\
      --ensure-metrics
"""

from __future__ import annotations

import argparse
import os
import subprocess
import sys
from typing import Iterable

import numpy as np

try:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.gridspec import GridSpecFromSubplotSpec
    import pandas as pd
except ImportError as e:
    raise SystemExit(f"Requires pandas+matplotlib: {e}") from e

import plot_runlog_conditional_behavior as crb

INIT_ORDER = crb.INIT_ORDER
PLAST_ORDER = crb.PLAST_ORDER
PLAST_TITLES = {"E2": "E2 (no plasticity)", "P1": "P1", "P2": "P2"}
INIT_COLORS = {"anti_maternal": "#c0392b", "random_uniform": "#2980b9", "pro_maternal": "#27ae60"}

WORLDS = ("easy", "normal", "hard")
FOODS = (5, 15, 25)

# Default focused metrics (one PNG each).
FOCUSED_METRIC_PANELS: list[tuple[str, str, str]] = [
    ("mother", "energy_mean_alive", "mean m0_energy (alive ticks)"),
    ("mother", "fatigue_mean_alive", "mean m0_fatigue (alive ticks)"),
    ("mother", "mother_ttd_norm", "mother TTD (normalized)"),
    ("child", "hunger_mean_alive", "mean c0_hunger (alive ticks)"),
    ("child", "warmth_mean_alive", "mean c0_warmth (alive ticks)"),
    ("child", "injury_mean_alive", "mean c0_injury (alive ticks)"),
    ("child", "child_ttd_norm", "child TTD (normalized)"),
    ("child", "P_sel_Forage_given_child_hungry", "P(Forage | child hungry)"),
    ("child", "P_sel_Care_given_child_cold", "P(Care | child cold, c0_warmth < τ)"),
    ("child", "P_sel_Protect_given_child_injured", "P(Protect | child injured)"),
]

# Full metric list (optional --all-metrics --combined-grid).
FULL_METRIC_PANELS: list[tuple[str, str, str]] = FOCUSED_METRIC_PANELS + [
    ("mother", "fear_mean_alive", "mean m0_fear_threat (alive ticks)"),
    ("mother", "stress_mean_alive", "mean m0_stress (alive ticks)"),
    ("mother", "bonding_mean_alive", "mean m0_bonding (alive ticks)"),
    ("mother", "closeness_mean_alive", "mean closeness_child (alive ticks)"),
]


def _e4_root(repo_root: str, world: str, food: int) -> str:
    return os.path.join(repo_root, f"E4_{world}_{food}")


def _cond_label(world: str, food: int) -> str:
    return f"{world}_f{food}"


def _all_e4_specs(repo_root: str) -> list[tuple[str, str, int]]:
    out = []
    for w in WORLDS:
        for f in FOODS:
            out.append((_cond_label(w, f), w, f))
    return out


def _maybe_run_metrics(root: str, *, metrics_subdir: str, force: bool) -> str:
    out_dir = os.path.join(root, metrics_subdir)
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
        "--extended",
    ]
    print("[info]", " ".join(cmd))
    p = subprocess.run(cmd, check=False)
    if p.returncode != 0:
        print(f"[warn] metrics build failed for {root} (exit={p.returncode})", file=sys.stderr)
    return csv_path


def _load_all_conditions(repo_root: str, *, metrics_subdir: str, ensure: bool, force: bool) -> pd.DataFrame:
    parts = []
    for label, world, food in _all_e4_specs(repo_root):
        root = _e4_root(repo_root, world, food)
        if not os.path.isdir(root):
            print(f"[warn] missing {root}", file=sys.stderr)
            continue
        csv_path = os.path.join(root, metrics_subdir, "conditional_metrics_single.csv")
        if ensure and (force or not os.path.isfile(csv_path)):
            csv_path = _maybe_run_metrics(root, metrics_subdir=metrics_subdir, force=force)
        if not os.path.isfile(csv_path):
            print(f"[warn] no metrics CSV for {label}: {csv_path}", file=sys.stderr)
            continue
        df = pd.read_csv(csv_path)
        df["condition"] = label
        df["world"] = world
        df["food"] = int(food)
        parts.append(df)
    if not parts:
        raise SystemExit("No E4 condition metrics loaded.")
    out = pd.concat(parts, ignore_index=True)
    print(f"[ok] loaded {out['condition'].nunique()} conditions, {out['path'].nunique() if 'path' in out.columns else '?'} runs")
    return out


def _per_genome_means(df: pd.DataFrame, metric_cols: list[str]) -> pd.DataFrame:
    keep = ["condition", "world", "food", "plasticity", "init", "evolve_seed"] + metric_cols
    keep = [c for c in keep if c in df.columns]
    sub = df[keep].copy()
    for c in metric_cols:
        sub[c] = pd.to_numeric(sub[c], errors="coerce")
    return sub.groupby(["condition", "world", "food", "plasticity", "init", "evolve_seed"], as_index=False).mean(
        numeric_only=True
    )


def _summarize(pg: pd.DataFrame, metric: str) -> pd.DataFrame:
    x = pg.dropna(subset=[metric])
    if x.empty:
        return pd.DataFrame(columns=["condition", "plasticity", "init", "n", "median", "q25", "q75"])
    g = x.groupby(["condition", "plasticity", "init"])[metric]
    return (
        pd.concat(
            [
                g.count().rename("n"),
                g.median().rename("median"),
                g.quantile(0.25).rename("q25"),
                g.quantile(0.75).rename("q75"),
            ],
            axis=1,
        )
        .reset_index()
    )


def _plot_trajectory_cell(ax, summ: pd.DataFrame, *, metric: str, condition_order: list[str]) -> None:
    x = np.arange(len(condition_order), dtype=float)
    idx = {c: i for i, c in enumerate(condition_order)}
    for init in INIT_ORDER:
        subi = summ[(summ["init"] == init)]
        if subi.empty:
            continue
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
        ax.plot(x, y_med, color=INIT_COLORS[init], linewidth=1.8, label=init)
        ax.fill_between(x, y_q25, y_q75, color=INIT_COLORS[init], alpha=0.12)
    ax.set_xticks(x)
    ax.set_xticklabels(condition_order, rotation=35, ha="right", fontsize=7)
    ax.grid(True, axis="y", color="0.92")


def _slug(s: str) -> str:
    return (
        s.replace(" ", "_")
        .replace("(", "")
        .replace(")", "")
        .replace("|", "")
        .replace(",", "")
        .replace("<", "lt")
        .replace("τ", "tau")
        .replace("/", "_")
    )


def _plot_single_metric(
    summ_by_metric: dict[str, pd.DataFrame],
    *,
    metric_key: str,
    metric_label: str,
    condition_order: list[str],
    x_labels: list[str],
    suptitle: str,
    out_png: str,
) -> None:
    """One metric = one image (E2 | P1 | P2), same style as Exp3 reference row."""
    if metric_key not in summ_by_metric:
        print(f"[warn] skip {out_png}: missing {metric_key}", file=sys.stderr)
        return

    fig, axes = plt.subplots(1, 3, figsize=(12.8, 4.0), sharey=True)
    sub = summ_by_metric[metric_key]
    for ax, plast in zip(axes, PLAST_ORDER):
        subp = sub[sub["plasticity"] == plast]
        _plot_trajectory_cell(ax, subp, metric=metric_key, condition_order=condition_order)
        ax.set_xticklabels(x_labels, rotation=30, ha="right", fontsize=9)
        ax.set_title(PLAST_TITLES.get(plast, plast), fontsize=10)
    axes[0].set_ylabel(metric_label + "\nmedian ± IQR (across genomes)", fontsize=9)

    handles = [
        plt.Line2D([0], [0], color=INIT_COLORS[i], linewidth=2.0, label=i.replace("_", " "))
        for i in INIT_ORDER
    ]
    fig.legend(handles=handles, loc="center left", bbox_to_anchor=(1.02, 0.5), frameon=False, fontsize=9, title="init")
    fig.suptitle(suptitle, y=1.02, fontsize=12)
    fig.tight_layout()
    os.makedirs(os.path.dirname(out_png), exist_ok=True)
    fig.savefig(out_png, dpi=200, bbox_inches="tight")
    plt.close(fig)
    print("[ok] wrote", out_png)


def _plot_sets_one_file_per_metric(
    summ_all: dict[str, pd.DataFrame],
    panels: list[tuple[str, str, str]],
    *,
    out_dir: str,
    include_by_world: bool = True,
) -> int:
    """by_world + per food + all 9 conditions. Returns PNG count."""

    def _emit(subdir: str, condition_order: list[str], x_labels: list[str], group_title: str) -> int:
        base = os.path.join(out_dir, subdir)
        os.makedirs(base, exist_ok=True)
        n = 0
        for _col, key, lab in panels:
            if key not in summ_all:
                continue
            fname = f"{_slug(key)}.png"
            _plot_single_metric(
                summ_all,
                metric_key=key,
                metric_label=lab,
                condition_order=condition_order,
                x_labels=x_labels,
                suptitle=f"{group_title}\n{lab}",
                out_png=os.path.join(base, fname),
            )
            n += 1
        return n

    n_png = 0
    if include_by_world:
        for world in WORLDS:
            order = [_cond_label(world, f) for f in FOODS]
            n_png += _emit(
                f"by_world/{world}",
                order,
                [f"food {f}" for f in FOODS],
                f"E4 — world = {world} (x-axis: food 5 / 15 / 25)",
            )

    for food in FOODS:
        order = [_cond_label(w, food) for w in WORLDS]
        n_png += _emit(
            f"food_{food}",
            order,
            list(WORLDS),
            f"E4 — food interval = {food} (x-axis: easy / normal / hard)",
        )

    order = [_cond_label(w, f) for w in WORLDS for f in FOODS]
    n_png += _emit(
        "all_conditions",
        order,
        [f"{w}\nf{f}" for w in WORLDS for f in FOODS],
        "E4 — all 9 conditions (world × food)",
    )
    return n_png


def _plot_metric_grid(
    summ_by_metric: dict[str, pd.DataFrame],
    *,
    metrics: list[tuple[str, str, str]],
    condition_order: list[str],
    x_labels: list[str],
    suptitle: str,
    out_png: str,
) -> None:
    # Order metrics: child column then mother column (like reference 3×2 for first 6, then more rows)
    present = [(col, key, lab) for col, key, lab in metrics if key in summ_by_metric]
    if not present:
        print("[warn] no metrics to plot for", out_png, file=sys.stderr)
        return

    n = len(present)
    nrows = int(np.ceil(n / 2))
    fig, axes = plt.subplots(nrows, 2, figsize=(13.5, 3.2 * nrows), squeeze=False)

    for mi, (col_side, key, lab) in enumerate(present):
        row, col = divmod(mi, 2)
        # Each grid cell: 1×3 plasticity sub-axes
        outer_ax = axes[row, col]
        outer_ax.set_axis_off()
        outer_ax.set_title(lab, fontsize=10, loc="left", pad=8)

        sub_gs = GridSpecFromSubplotSpec(1, 3, subplot_spec=outer_ax.get_subplotspec(), wspace=0.12)
        for pi, plast in enumerate(PLAST_ORDER):
            ax = fig.add_subplot(sub_gs[pi])
            sub = summ_by_metric[key]
            subp = sub[sub["plasticity"] == plast]
            _plot_trajectory_cell(ax, subp, metric=key, condition_order=condition_order)
            ax.set_xticklabels(x_labels, rotation=35, ha="right", fontsize=7)
            ax.set_title(PLAST_TITLES.get(plast, plast), fontsize=9)
            if pi == 0:
                ax.set_ylabel("median ± IQR\n(across genomes)", fontsize=8)

    # Legend once
    handles = [
        plt.Line2D([0], [0], color=INIT_COLORS[i], linewidth=2.0, label=i.replace("_", " "))
        for i in INIT_ORDER
    ]
    fig.legend(handles=handles, loc="upper center", ncol=3, frameon=False, fontsize=9, bbox_to_anchor=(0.5, 1.02))
    fig.suptitle(suptitle, y=1.04, fontsize=13)
    fig.subplots_adjust(top=0.92, hspace=0.55, wspace=0.35)
    fig.savefig(out_png, dpi=200, bbox_inches="tight")
    plt.close(fig)
    print("[ok] wrote", out_png)


def _build_summaries(pg: pd.DataFrame, metrics: list[str]) -> dict[str, pd.DataFrame]:
    out: dict[str, pd.DataFrame] = {}
    for m in metrics:
        if m not in pg.columns:
            continue
        out[m] = _summarize(pg, m)
    return out


def _metrics_available(pg: pd.DataFrame, panels: list[tuple[str, str, str]]) -> list[tuple[str, str, str]]:
    keys = set(pg.columns)
    return [(col, key, lab) for col, key, lab in panels if key in keys]


def main(argv: Iterable[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--repo-root", default=".", help="Repo root containing E4_* folders")
    ap.add_argument("--out-dir", required=True)
    ap.add_argument("--ensure-metrics", action="store_true", help="Build conditional_metrics_single.csv if missing")
    ap.add_argument("--force-recompute", action="store_true")
    ap.add_argument("--metrics-subdir", default="figures_conditional_extended")
    ap.add_argument("--intersection-only", action="store_true", help="Keep only genomes present in all 9 conditions")
    ap.add_argument(
        "--all-metrics",
        action="store_true",
        help="Plot full metric list instead of the default focused metrics",
    )
    ap.add_argument(
        "--combined-grid",
        action="store_true",
        help="Also write multi-metric grid PNGs (default: one PNG per metric only)",
    )
    ap.add_argument(
        "--skip-by-world",
        action="store_true",
        help="Skip by_world/{easy,normal,hard}/ plots (food 5/15/25 on x-axis)",
    )
    args = ap.parse_args(list(argv) if argv is not None else None)

    repo_root = os.path.abspath(args.repo_root)
    out_dir = os.path.abspath(args.out_dir)
    os.makedirs(out_dir, exist_ok=True)

    df = _load_all_conditions(
        repo_root,
        metrics_subdir=args.metrics_subdir,
        ensure=bool(args.ensure_metrics),
        force=bool(args.force_recompute),
    )

    panels_def = FULL_METRIC_PANELS if args.all_metrics else FOCUSED_METRIC_PANELS
    metric_keys = [k for _, k, _ in panels_def if k in df.columns]
    pg = _per_genome_means(df, metric_keys)

    if args.intersection_only:
        key_cols = ["plasticity", "init", "evolve_seed"]
        conds = sorted(pg["condition"].unique())
        sets = [
            set(map(tuple, pg.loc[pg["condition"] == c, key_cols].drop_duplicates().to_numpy()))
            for c in conds
        ]
        inter = set.intersection(*sets) if sets else set()
        if not inter:
            print("[warn] intersection empty; not filtering", file=sys.stderr)
        else:
            m = pg[key_cols].apply(tuple, axis=1).isin(inter)
            pg = pg.loc[m].copy()
            print(f"[info] intersection-only: {len(inter)} genomes kept", file=sys.stderr)

    panels = _metrics_available(pg, panels_def)
    summ_all = _build_summaries(pg, [k for _, k, _ in panels])

    include_by_world = not args.skip_by_world
    n_png = _plot_sets_one_file_per_metric(
        summ_all, panels, out_dir=out_dir, include_by_world=include_by_world
    )

    if args.combined_grid:
        cg = os.path.join(out_dir, "_combined_grids")
        if include_by_world:
            for world in WORLDS:
                order = [_cond_label(world, f) for f in FOODS]
                _plot_metric_grid(
                    summ_all,
                    metrics=panels,
                    condition_order=order,
                    x_labels=[f"food {f}" for f in FOODS],
                    suptitle=f"E4 trajectories — world = {world}",
                    out_png=os.path.join(cg, "by_world", f"grid_{world}.png"),
                )
        for food in FOODS:
            order = [_cond_label(w, food) for w in WORLDS]
            _plot_metric_grid(
                summ_all,
                metrics=panels,
                condition_order=order,
                x_labels=list(WORLDS),
                suptitle=f"E4 trajectories — food = {food}",
                out_png=os.path.join(cg, f"food_{food}", "grid.png"),
            )
        order = [_cond_label(w, f) for w in WORLDS for f in FOODS]
        _plot_metric_grid(
            summ_all,
            metrics=panels,
            condition_order=order,
            x_labels=[f"{w} f{f}" for w in WORLDS for f in FOODS],
            suptitle="E4 trajectories — all 9 conditions",
            out_png=os.path.join(cg, "all_conditions", "grid_all_9.png"),
        )

    print(f"[ok] done → {out_dir} ({n_png} single-metric PNGs)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
