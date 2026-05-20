#!/usr/bin/env python3
"""
plot_init_signature_food_threat.py
==================================
Visualize how **maternal init types** (anti / random / pro) differ as **signatures**
across rollout **worlds** laid out in a 2D ecology plane:

  x = food spawn interval (ticks)
  y = number of threats

Uses the same per-run inputs as ``plot_world_condition_comparison.py``:
``<root>/<metrics-subdir>/conditional_metrics_single.csv`` (extended metrics
recommended).

Outputs (under ``--out-dir``)
-----------------------------
1) **Ecology scatter** (one PNG per plasticity × metric, plus optional combined row):
   ``signature_ecology_scatter__<metric>__<plast>.png``
   - **One point per genome × condition** at ``(food interval, n_threats)`` with
     **deterministic jitter** so clouds are visible (like PCA/LDA scatter).
   - **Color = init**; **marker size = metric** (e.g. conditional P(sel|state)).
   - Small **black diamond** = median per (init, condition) for reference.

2) **PCA** of per-genome vectors (features = **all** extended ``P(sel|*)`` columns,
   core ``P(maternal|hungry)`` / ``P(forage|hungry)`` / ``P(protect|fear)``, alive means,
   TTD norms, and ``n_*`` tick-count masks — each crossed with **conditions**):
   ``embedding_pca__<plast>.png`` + ``pca_loadings__<plast>.csv``

3) **LDA** (supervised by ``init``) on the same matrix:
   ``embedding_lda__<plast>.png`` + ``lda_projected_genomes__<plast>.csv``

4) **Interpretable add-ons**
   - ``signature_vs_world__<metric>__<plast>.png`` — median ± IQR of one metric
     vs **world condition** (one line per init); shows how each type shifts across worlds.
   - ``signature_vs_world_combined__<metric>.png`` — same, **three rows** (E2/P1/P2).
   - ``lda_loadings_ld1_bars__<plast>.png`` — top |LD1| feature weights (from global LDA).
   - ``lda_scatter_per_world__<plast>.png`` — small multiples: **LDA fit only on features
     from that world**, scatter of genomes colored by init (separability *within* each world).

5) Tidy tables: ``per_genome_long.csv``, ``per_genome_wide_features.csv``, ``pca_feature_columns.txt``

World geometry defaults
-----------------------
The script does **not** read grid size from CSV; it maps **condition label → (food, threats)**
via ``DEFAULT_CONDITION_GEOMETRY``. Override with ``--geom label=food,threats``
(repeatable) if your rollouts used different parameters than the repo defaults.

Typical invocation (paths under repo root)
------------------------------------------
python3 plot_init_signature_food_threat.py \\
  --out-dir paper_figures_init_signature_food_threat \\
  --intersection-only \\
  --cond easy_threat0=FinalGenomeRollouts_easy_threat0_seed42_49,FinalGenomeRollouts_easy_threat0_seed50_51 \\
  --cond easy_food10=FinalGenomeRollouts_easy_food_10 \\
  --cond base=FinalGenomeRollouts_normal_seen \\
  --cond unseen_food30=FinalGenomeRollouts_unseen_food30 \\
  --cond unseen_hard=FinalGenomeRollouts_unseen_hard \\
  --metric P_sel_Care_given_child_hungry \\
  --metric P_maternal_given_hungry
"""

from __future__ import annotations

import argparse
import os
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

try:
    from sklearn.decomposition import PCA
    from sklearn.discriminant_analysis import LinearDiscriminantAnalysis
    from sklearn.impute import SimpleImputer
    from sklearn.preprocessing import StandardScaler
except ImportError as e:
    raise SystemExit(f"Requires scikit-learn: {e}") from e

from plot_world_condition_comparison import (
    INIT_COLORS,
    INIT_ORDER,
    PLAST_ORDER,
    PLAST_TITLES,
    _available_extended_psel_cols,
    _intersection_filter,
    _parse_kv,
    _per_genome_means,
    _read_metrics_csv,
)

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

# Matches ``run_headless_rollout_emergence_final_genomes.sh`` style rollouts in this repo
# (15×15 grid, food spawn interval, threat count). Adjust with --geom if yours differ.
DEFAULT_CONDITION_GEOMETRY: dict[str, tuple[float, float]] = {
    "easy_threat0": (15.0, 0.0),
    "easy_food10": (10.0, 1.0),
    "base": (15.0, 1.0),
    "unseen_food30": (30.0, 1.0),
    "unseen_hard": (15.0, 2.0),
}

def _parse_geom(spec: str) -> tuple[str, float, float]:
    if "=" not in spec or "," not in spec:
        raise SystemExit(f"Expected label=food,threats, got: {spec!r}")
    label, rest = spec.split("=", 1)
    a, b = rest.split(",", 1)
    return label.strip(), float(a.strip()), float(b.strip())


def _geom_lookup(
    label: str,
    *,
    overrides: dict[str, tuple[float, float]],
    defaults: dict[str, tuple[float, float]],
) -> tuple[float, float] | None:
    if label in overrides:
        return overrides[label]
    if label in defaults:
        return defaults[label]
    return None


def _load_all_conditions(
    cond_specs: list[str],
    *,
    metrics_subdir: str,
) -> pd.DataFrame:
    cond_roots: dict[str, list[str]] = {}
    order: list[str] = []
    for raw in cond_specs:
        label, roots = _parse_kv(raw)
        if label not in cond_roots:
            cond_roots[label] = []
            order.append(label)
        cond_roots[label].extend(roots)

    dfs = []
    for label in order:
        for root in cond_roots[label]:
            if not os.path.isdir(root):
                print(f"[warn] missing root for {label}: {root}", file=sys.stderr)
                continue
            csv_path = os.path.join(root, metrics_subdir, "conditional_metrics_single.csv")
            if not os.path.isfile(csv_path):
                print(f"[warn] missing metrics CSV: {csv_path}", file=sys.stderr)
                continue
            dfs.append(_read_metrics_csv(csv_path, condition=label))
    if not dfs:
        raise SystemExit("No conditional_metrics_single.csv loaded.")
    return pd.concat(dfs, ignore_index=True)


def _feature_columns(df: pd.DataFrame) -> list[str]:
    """
    Wide PCA/LDA features: all child- and mother-conditioned P(sel|state),
    unconditional-ish responsiveness metrics, alive means, TTD, and n_* tick counts.
    """
    cols: list[str] = []
    for c in (
        "P_maternal_given_hungry",
        "P_forage_given_hungry",
        "P_protect_given_fear",
    ):
        if c in df.columns:
            cols.append(c)
    cols.extend(_available_extended_psel_cols(df))
    for c in (
        "closeness_mean_alive",
        "injury_mean_alive",
        "fear_mean_alive",
        "energy_mean_alive",
        "stress_mean_alive",
        "warmth_mean_alive",
        "bonding_mean_alive",
        "fatigue_mean_alive",
        "child_ttd_norm",
        "mother_ttd_norm",
    ):
        if c in df.columns and c not in cols:
            cols.append(c)
    for c in df.columns:
        if c in ("path", "run_dir", "plasticity", "init", "evolve_seed", "condition"):
            continue
        if c.startswith("n_ticks") or c in ("n_alive_ticks", "n_hungry_alive_ticks", "n_fear_alive_ticks"):
            if c not in cols:
                cols.append(c)
    seen: set[str] = set()
    out: list[str] = []
    for c in cols:
        if c not in seen:
            seen.add(c)
            out.append(c)
    return out


def _stable_jitter(plast: str, init: str, evolve_seed: int, condition: str, *, salt: int) -> tuple[float, float]:
    """Reproducible (food, threat) jitter in data units for scatter visibility."""
    h = hash((str(plast), str(init), int(evolve_seed), str(condition), int(salt))) % (2**31 - 1)
    rng = np.random.RandomState(h)
    jf = float(rng.uniform(-1.25, 1.25))
    jt = float(rng.uniform(-0.14, 0.14))
    return jf, jt


def _wide_pivot(pg: pd.DataFrame, *, metric_cols: list[str], condition_order: list[str]) -> pd.DataFrame:
    """One row per genome; columns = condition__metric."""
    parts = []
    for m in metric_cols:
        sub = pg[pg["condition"].isin(condition_order)].copy()
        if m not in sub.columns:
            continue
        w = sub.pivot_table(
            index=["plasticity", "init", "evolve_seed"],
            columns="condition",
            values=m,
            aggfunc="mean",
        )
        w = w.reindex(columns=[c for c in condition_order if c in w.columns])
        w.columns = [f"{c}__{m}" for c in w.columns]
        parts.append(w)
    if not parts:
        raise SystemExit("No pivotable metric columns after filtering.")
    wide = pd.concat(parts, axis=1)
    return wide.reset_index()


def _plot_signature_ecology_scatter(
    pg: pd.DataFrame,
    *,
    metric: str,
    plast: str,
    condition_order: list[str],
    geom: dict[str, tuple[float, float]],
    out_png: str,
) -> None:
    """Per-genome scatter in (food, threats) with jitter; color=init, size=metric."""
    sub = pg[(pg["plasticity"] == plast) & (pg["condition"].isin(condition_order))].copy()
    if sub.empty or metric not in sub.columns:
        print(f"[warn] skip ecology scatter (empty or missing metric): {plast} {metric}", file=sys.stderr)
        return

    xs: list[float] = []
    ys: list[float] = []
    vals: list[float] = []
    inits: list[str] = []

    for _, row in sub.iterrows():
        cond = str(row["condition"])
        xy = geom.get(cond)
        if xy is None:
            continue
        food, thr = xy
        v = float(row[metric])
        if not np.isfinite(v):
            continue
        jf, jt = _stable_jitter(
            plast,
            str(row["init"]),
            int(row["evolve_seed"]),
            cond,
            salt=42,
        )
        xs.append(food + jf)
        ys.append(float(thr) + jt)
        vals.append(v)
        inits.append(str(row["init"]))

    if not xs:
        print(f"[warn] no finite scatter points: {plast} {metric}", file=sys.stderr)
        return

    fig, ax = plt.subplots(figsize=(7.4, 5.8))
    vmin, vmax = float(np.min(vals)), float(np.max(vals))
    if vmax <= vmin:
        vmax = vmin + 1e-6

    def _sizes(z: np.ndarray) -> np.ndarray:
        z = np.clip((z - vmin) / (vmax - vmin + 1e-9), 0.0, 1.0)
        return 22.0 + 115.0 * z

    for init in INIT_ORDER:
        m = np.array([i == init for i in inits], dtype=bool)
        if not np.any(m):
            continue
        ax.scatter(
            np.array(xs, dtype=float)[m],
            np.array(ys, dtype=float)[m],
            s=_sizes(np.array(vals, dtype=float)[m]),
            alpha=0.62,
            color=INIT_COLORS[init],
            edgecolors="0.25",
            linewidths=0.25,
            label=init.replace("_", " "),
            zorder=2,
        )

    # Median reference markers per (init, condition)
    summ = sub.groupby(["init", "condition"], as_index=False)[metric].median()
    for init in INIT_ORDER:
        for cond in condition_order:
            xy = geom.get(cond)
            if xy is None:
                continue
            sel = summ[(summ["init"] == init) & (summ["condition"] == cond)]
            if sel.empty:
                continue
            v = float(sel.iloc[0][metric])
            if not np.isfinite(v):
                continue
            food, thr = xy
            ax.scatter(
                [food],
                [thr],
                s=140,
                marker="D",
                facecolors="none",
                edgecolors=INIT_COLORS[init],
                linewidths=1.35,
                zorder=5,
            )

    ax.set_xlabel("Food spawn interval (ticks)")
    ax.set_ylabel("Number of threats")
    ax.set_title(
        f"{metric} — {PLAST_TITLES.get(plast, plast)}\n"
        "Each point = one genome×condition (jittered); marker size ∝ metric; diamonds = median(init,condition)"
    )
    ax.legend(frameon=False, loc="best", title="init (color)")
    fig.tight_layout()
    fig.subplots_adjust(bottom=0.14)
    fig.text(0.5, 0.02, f"Larger markers = higher {metric}", ha="center", fontsize=9, color="0.35")
    fig.savefig(out_png, dpi=200, bbox_inches="tight")
    plt.close(fig)
    print("[ok] wrote", out_png)


def _plot_signature_ecology_scatter_combined(
    pg: pd.DataFrame,
    *,
    metric: str,
    condition_order: list[str],
    geom: dict[str, tuple[float, float]],
    out_png: str,
) -> None:
    """One row of panels: E2 / P1 / P2 scatter for the same metric."""
    fig, axes = plt.subplots(1, 3, figsize=(14.5, 5.2), sharey=True, sharex=True)
    all_vals: list[float] = []
    for plast in PLAST_ORDER:
        sub = pg[(pg["plasticity"] == plast) & (pg["condition"].isin(condition_order))].copy()
        if sub.empty or metric not in sub.columns:
            continue
        for _, row in sub.iterrows():
            v = float(row[metric])
            if np.isfinite(v):
                all_vals.append(v)
    if not all_vals:
        plt.close(fig)
        print(f"[warn] skip combined ecology scatter: no data for {metric}", file=sys.stderr)
        return
    vmin, vmax = min(all_vals), max(all_vals)
    if vmax <= vmin:
        vmax = vmin + 1e-6

    def _sizes(z: np.ndarray) -> np.ndarray:
        z = np.clip((z - vmin) / (vmax - vmin + 1e-9), 0.0, 1.0)
        return 18.0 + 95.0 * z

    for ax, plast in zip(axes, PLAST_ORDER):
        sub = pg[(pg["plasticity"] == plast) & (pg["condition"].isin(condition_order))].copy()
        if sub.empty or metric not in sub.columns:
            ax.set_title(PLAST_TITLES.get(plast, plast))
            continue
        xs, ys, vals, inits = [], [], [], []
        for _, row in sub.iterrows():
            cond = str(row["condition"])
            xy = geom.get(cond)
            if xy is None:
                continue
            food, thr = xy
            v = float(row[metric])
            if not np.isfinite(v):
                continue
            jf, jt = _stable_jitter(plast, str(row["init"]), int(row["evolve_seed"]), cond, salt=42)
            xs.append(food + jf)
            ys.append(float(thr) + jt)
            vals.append(v)
            inits.append(str(row["init"]))
        for init in INIT_ORDER:
            m = np.array([i == init for i in inits], dtype=bool)
            if not np.any(m):
                continue
            ax.scatter(
                np.array(xs, dtype=float)[m],
                np.array(ys, dtype=float)[m],
                s=_sizes(np.array(vals, dtype=float)[m]),
                alpha=0.58,
                color=INIT_COLORS[init],
                edgecolors="0.25",
                linewidths=0.2,
                label=init.replace("_", " ") if plast == PLAST_ORDER[0] else None,
                zorder=2,
            )
        summ = sub.groupby(["init", "condition"], as_index=False)[metric].median()
        for init in INIT_ORDER:
            for cond in condition_order:
                xy = geom.get(cond)
                if xy is None:
                    continue
                sel = summ[(summ["init"] == init) & (summ["condition"] == cond)]
                if sel.empty:
                    continue
                v = float(sel.iloc[0][metric])
                if not np.isfinite(v):
                    continue
                food, thr = xy
                ax.scatter(
                    [food],
                    [thr],
                    s=120,
                    marker="D",
                    facecolors="none",
                    edgecolors=INIT_COLORS[init],
                    linewidths=1.2,
                    zorder=5,
                )
        ax.set_title(PLAST_TITLES.get(plast, plast))
        ax.set_xlabel("Food spawn interval (ticks)")
    axes[0].set_ylabel("Number of threats")
    handles, labels = axes[0].get_legend_handles_labels()
    if handles:
        fig.legend(handles, labels, loc="upper center", ncol=3, frameon=False, bbox_to_anchor=(0.5, 1.02), title="init")
    fig.suptitle(f"{metric} in ecology space (per genome×condition); size ∝ metric", y=0.98)
    fig.text(0.5, 0.02, f"Larger markers = higher {metric}", ha="center", fontsize=9, color="0.35")
    fig.subplots_adjust(top=0.88, bottom=0.12, right=0.96)
    fig.savefig(out_png, dpi=200, bbox_inches="tight")
    plt.close(fig)
    print("[ok] wrote", out_png)


def _plot_signature_vs_world(
    pg: pd.DataFrame,
    *,
    metric: str,
    plast: str,
    condition_order: list[str],
    out_png: str,
) -> None:
    """Median and IQR of metric vs world condition; one line per init."""
    sub = pg[(pg["plasticity"] == plast) & (pg["condition"].isin(condition_order))].copy()
    if sub.empty or metric not in sub.columns:
        print(f"[warn] skip signature_vs_world: {plast} {metric}", file=sys.stderr)
        return
    rows = []
    for (init, cond), gdf in sub.groupby(["init", "condition"]):
        x = pd.to_numeric(gdf[metric], errors="coerce").dropna()
        if x.empty:
            continue
        rows.append(
            {
                "init": init,
                "condition": cond,
                "med": float(x.median()),
                "lo": float(x.quantile(0.25)),
                "hi": float(x.quantile(0.75)),
            }
        )
    agg = pd.DataFrame(rows)
    x = np.arange(len(condition_order))
    fig, ax = plt.subplots(figsize=(8.2, 4.8))
    for init in INIT_ORDER:
        med, lo, hi = [], [], []
        for c in condition_order:
            row = agg[(agg["init"] == init) & (agg["condition"] == c)]
            if row.empty:
                med.append(np.nan)
                lo.append(np.nan)
                hi.append(np.nan)
            else:
                med.append(float(row.iloc[0]["med"]))
                lo.append(float(row.iloc[0]["lo"]))
                hi.append(float(row.iloc[0]["hi"]))
        med = np.array(med, dtype=float)
        lo = np.array(lo, dtype=float)
        hi = np.array(hi, dtype=float)
        color = INIT_COLORS[init]
        ax.plot(x, med, "o-", color=color, linewidth=2.0, markersize=7, label=init.replace("_", " "))
        ax.fill_between(x, lo, hi, color=color, alpha=0.18, linewidth=0)
    ax.set_xticks(x)
    ax.set_xticklabels(condition_order, rotation=22, ha="right")
    ax.set_ylabel(metric)
    ax.set_xlabel("Rollout world (condition)")
    ax.set_title(
        f"{metric} vs world — {PLAST_TITLES.get(plast, plast)}\n"
        "(lines = median across genomes; band = IQR)"
    )
    ax.legend(frameon=False, loc="best")
    if str(metric).startswith("P_"):
        ax.set_ylim(0.0, 1.05)
    fig.tight_layout()
    fig.savefig(out_png, dpi=200, bbox_inches="tight")
    plt.close(fig)
    print("[ok] wrote", out_png)


def _plot_signature_vs_world_combined(
    pg: pd.DataFrame,
    *,
    metric: str,
    condition_order: list[str],
    out_png: str,
) -> None:
    fig, axes = plt.subplots(3, 1, figsize=(8.4, 9.2), sharex=True, sharey=False)
    for ax, plast in zip(axes, PLAST_ORDER):
        sub = pg[(pg["plasticity"] == plast) & (pg["condition"].isin(condition_order))].copy()
        if sub.empty or metric not in sub.columns:
            ax.set_title(PLAST_TITLES.get(plast, plast))
            continue
        rows = []
        for (init, cond), gdf in sub.groupby(["init", "condition"]):
            x = pd.to_numeric(gdf[metric], errors="coerce").dropna()
            if x.empty:
                continue
            rows.append(
                {
                    "init": init,
                    "condition": cond,
                    "med": float(x.median()),
                    "lo": float(x.quantile(0.25)),
                    "hi": float(x.quantile(0.75)),
                }
            )
        agg = pd.DataFrame(rows)
        x = np.arange(len(condition_order))
        for init in INIT_ORDER:
            med, lo, hi = [], [], []
            for c in condition_order:
                row = agg[(agg["init"] == init) & (agg["condition"] == c)]
                if row.empty:
                    med.append(np.nan)
                    lo.append(np.nan)
                    hi.append(np.nan)
                else:
                    med.append(float(row.iloc[0]["med"]))
                    lo.append(float(row.iloc[0]["lo"]))
                    hi.append(float(row.iloc[0]["hi"]))
            med = np.array(med, dtype=float)
            lo = np.array(lo, dtype=float)
            hi = np.array(hi, dtype=float)
            color = INIT_COLORS[init]
            ax.plot(x, med, "o-", color=color, linewidth=1.8, markersize=6, label=init.replace("_", " "))
            ax.fill_between(x, lo, hi, color=color, alpha=0.16, linewidth=0)
        ax.set_xticks(x)
        ax.set_ylabel(metric)
        ax.set_title(PLAST_TITLES.get(plast, plast))
        if str(metric).startswith("P_"):
            ax.set_ylim(0.0, 1.05)
    axes[-1].set_xticklabels(condition_order, rotation=22, ha="right")
    axes[-1].set_xlabel("Rollout world (condition)")
    handles, labels = axes[0].get_legend_handles_labels()
    if handles:
        fig.legend(handles, labels, loc="upper center", ncol=3, frameon=False, bbox_to_anchor=(0.5, 1.01), title="init")
    fig.suptitle(f"{metric}: how each init type tracks across worlds", y=1.02)
    fig.tight_layout()
    fig.subplots_adjust(top=0.93)
    fig.savefig(out_png, dpi=200, bbox_inches="tight")
    plt.close(fig)
    print("[ok] wrote", out_png)


def _plot_lda_loadings_bars(
    lda: LinearDiscriminantAnalysis,
    feat_names: list[str],
    *,
    plast: str,
    out_png: str,
    top_k: int = 28,
) -> None:
    """Horizontal bar chart of first LD axis weights (scalings[:,0])."""
    scal = getattr(lda, "scalings_", None)
    if scal is None or scal.shape[1] < 1:
        print(f"[warn] no LDA scalings for loadings plot: {plast}", file=sys.stderr)
        return
    w = scal[:, 0].astype(float)
    pick = np.argsort(np.abs(w))[-top_k:]
    pick = pick[np.argsort(w[pick])]
    names = [feat_names[i] for i in pick]
    vals = w[pick]
    fig, ax = plt.subplots(figsize=(9.0, 6.2))
    y = np.arange(len(names))
    ax.barh(y, vals, color=np.where(vals >= 0, "#2980b9", "#c0392b"), height=0.72)
    ax.set_yticks(y)
    ax.set_yticklabels(names, fontsize=8)
    ax.axvline(0.0, color="0.4", linewidth=0.8)
    ax.set_xlabel("LDA scaling (LD1)")
    ax.set_title(
        f"LDA interpretability — {PLAST_TITLES.get(plast, plast)}\n"
        f"Top {top_k} features by |LD1 weight| (global multi-world LDA)"
    )
    fig.tight_layout()
    fig.savefig(out_png, dpi=200, bbox_inches="tight")
    plt.close(fig)
    print("[ok] wrote", out_png)
    o = np.argsort(np.abs(w))[-top_k:][::-1]
    pd.DataFrame({"feature": [feat_names[i] for i in o], "ld1_weight": w[o]}).to_csv(
        out_png.replace(".png", ".csv"), index=False
    )


def _plot_lda_scatter_per_world(
    Xs: np.ndarray,
    y: np.ndarray,
    feat_only: list[str],
    *,
    plast: str,
    condition_order: list[str],
    out_png: str,
) -> None:
    """LDA fit on features from one world at a time; scatter genomes."""
    feat_idx = {n: i for i, n in enumerate(feat_only)}
    n_cond = len(condition_order)
    fig, axes = plt.subplots(1, n_cond, figsize=(2.9 * n_cond, 4.2), sharey=False, sharex=False)
    if n_cond == 1:
        axes = [axes]
    classes = sorted(set(y.tolist()))
    for ax, cond in zip(axes, condition_order):
        cols = [c for c in feat_only if c.startswith(f"{cond}__")]
        if len(cols) < 4:
            ax.set_title(f"{cond}\n(too few feats)")
            ax.set_axis_off()
            continue
        idxs = [feat_idx[c] for c in cols]
        Xi = Xs[:, idxs]
        if Xi.shape[1] < 2 or len(classes) < 2:
            ax.set_title(cond)
            continue
        n_ld = min(len(classes) - 1, Xi.shape[1], 2)
        try:
            lda_w = LinearDiscriminantAnalysis(n_components=max(1, n_ld), solver="svd")
            Zw = lda_w.fit_transform(Xi, y)
        except Exception as e:
            ax.set_title(f"{cond}\n(LDA failed)")
            ax.text(0.5, 0.5, str(e)[:80], ha="center", va="center", transform=ax.transAxes, fontsize=7)
            continue
        if Zw.shape[1] == 1:
            Z2 = np.column_stack([Zw[:, 0], np.zeros(len(Zw))])
            ax.set_ylabel("(pad)")
        else:
            Z2 = Zw[:, :2]
            ax.set_ylabel("LD2")
        for init in INIT_ORDER:
            m = y == init
            if not np.any(m):
                continue
            ax.scatter(
                Z2[m, 0],
                Z2[m, 1],
                s=28,
                alpha=0.75,
                color=INIT_COLORS[init],
                label=init.replace("_", " ") if cond == condition_order[0] else None,
            )
        ax.set_xlabel("LD1")
        ax.set_title(cond, fontsize=10)
    fig.suptitle(
        f"LDA within each world (features = that world only) — {PLAST_TITLES.get(plast, plast)}\n"
        "Axes are fit separately per panel (LD scales not comparable across worlds).",
        fontsize=11,
        y=1.06,
    )
    handles, labels = axes[0].get_legend_handles_labels()
    if handles:
        fig.legend(handles, labels, loc="upper center", ncol=3, frameon=False, bbox_to_anchor=(0.5, 1.12), title="init")
    fig.tight_layout()
    fig.subplots_adjust(top=0.78)
    fig.savefig(out_png, dpi=200, bbox_inches="tight")
    plt.close(fig)
    print("[ok] wrote", out_png)


def _plot_embedding(
    X: np.ndarray,
    y_init: np.ndarray,
    Z: np.ndarray,
    *,
    title: str,
    xlabel: str,
    ylabel: str,
    out_png: str,
) -> None:
    fig, ax = plt.subplots(figsize=(6.8, 5.6))
    for init in INIT_ORDER:
        m = y_init == init
        if not np.any(m):
            continue
        ax.scatter(Z[m, 0], Z[m, 1], s=36, alpha=0.75, color=INIT_COLORS[init], label=init.replace("_", " "))
    ax.set_xlabel(xlabel)
    ax.set_ylabel(ylabel)
    ax.set_title(title)
    ax.legend(frameon=False, loc="best")
    fig.tight_layout()
    fig.savefig(out_png, dpi=200, bbox_inches="tight")
    plt.close(fig)
    print("[ok] wrote", out_png)


def main(argv: Iterable[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out-dir", required=True)
    ap.add_argument(
        "--cond",
        action="append",
        required=True,
        help="Repeatable: label=/path/to/FinalGenomeRollouts_* (comma-merge like other scripts).",
    )
    ap.add_argument(
        "--metrics-subdir",
        default="figures_conditional_extended",
        help="Subdir under each rollout root with conditional_metrics_single.csv",
    )
    ap.add_argument(
        "--condition-order",
        default="",
        help="Comma order of condition labels for intersection + plots (default: --cond order).",
    )
    ap.add_argument(
        "--intersection-only",
        action="store_true",
        help="Keep genomes present in every loaded condition (same as plot_world_condition_comparison).",
    )
    ap.add_argument(
        "--metric",
        action="append",
        default=[],
        help="Repeatable metric for ecology signature plots (default: P_sel_Care_given_child_hungry).",
    )
    ap.add_argument(
        "--geom",
        action="append",
        default=[],
        help="Repeatable override: label=food,threats (e.g. base=15,1).",
    )
    ap.add_argument(
        "--no-ecology-combined",
        action="store_true",
        help="Skip the 1×3 combined ecology scatter (E2/P1/P2 in one figure).",
    )
    ap.add_argument(
        "--no-signature-world-lines",
        action="store_true",
        help="Skip median±IQR lines vs world condition (per metric).",
    )
    ap.add_argument(
        "--no-lda-loadings",
        action="store_true",
        help="Skip LD1 loading bar chart (global LDA interpretability).",
    )
    ap.add_argument(
        "--no-lda-per-world",
        action="store_true",
        help="Skip per-world LDA scatter panels.",
    )
    args = ap.parse_args(list(argv) if argv is not None else None)

    out_dir = os.path.abspath(args.out_dir)
    os.makedirs(out_dir, exist_ok=True)

    geom = dict(DEFAULT_CONDITION_GEOMETRY)
    for g in args.geom:
        lab, food, thr = _parse_geom(g)
        geom[lab] = (food, thr)

    df = _load_all_conditions(args.cond, metrics_subdir=args.metrics_subdir)
    feat_cols = _feature_columns(df)
    if not feat_cols:
        raise SystemExit("No feature columns found in merged metrics (need extended metrics?).")

    pg = _per_genome_means(df, feat_cols)

    if args.condition_order.strip():
        condition_order = [c.strip() for c in args.condition_order.split(",") if c.strip()]
    else:
        seen: list[str] = []
        for c in pg["condition"].astype(str):
            if c not in seen:
                seen.append(c)
        condition_order = seen

    if args.intersection_only:
        pg = _intersection_filter(pg, condition_order)

    pg.to_csv(os.path.join(out_dir, "per_genome_long.csv"), index=False)

    metrics_sig = [m for m in (args.metric or []) if str(m).strip()]
    if not metrics_sig:
        metrics_sig = ["P_sel_Care_given_child_hungry"]

    with open(os.path.join(out_dir, "pca_feature_columns.txt"), "w", encoding="utf-8") as f:
        f.write("Columns used as features (each crossed with conditions in wide matrix):\n")
        for c in feat_cols:
            f.write(f"  {c}\n")

    for m in metrics_sig:
        if m not in pg.columns:
            print(f"[warn] metric not in data, skip ecology plot: {m}", file=sys.stderr)
            continue
        for plast in PLAST_ORDER:
            out_png = os.path.join(out_dir, f"signature_ecology_scatter__{m}__{plast}.png")
            _plot_signature_ecology_scatter(
                pg,
                metric=m,
                plast=plast,
                condition_order=condition_order,
                geom=geom,
                out_png=out_png,
            )
        if not args.no_ecology_combined:
            out_comb = os.path.join(out_dir, f"signature_ecology_scatter_combined__{m}.png")
            _plot_signature_ecology_scatter_combined(
                pg,
                metric=m,
                condition_order=condition_order,
                geom=geom,
                out_png=out_comb,
            )
        if not args.no_signature_world_lines:
            for plast in PLAST_ORDER:
                out_ln = os.path.join(out_dir, f"signature_vs_world__{m}__{plast}.png")
                _plot_signature_vs_world(
                    pg,
                    metric=m,
                    plast=plast,
                    condition_order=condition_order,
                    out_png=out_ln,
                )
            out_lnc = os.path.join(out_dir, f"signature_vs_world_combined__{m}.png")
            _plot_signature_vs_world_combined(
                pg,
                metric=m,
                condition_order=condition_order,
                out_png=out_lnc,
            )

    # --- PCA / LDA on wide feature matrix ---
    wide = _wide_pivot(pg, metric_cols=feat_cols, condition_order=condition_order)
    wide.to_csv(os.path.join(out_dir, "per_genome_wide_features.csv"), index=False)

    for plast in PLAST_ORDER:
        sub = wide[wide["plasticity"] == plast].copy()
        if sub.empty:
            print(f"[warn] no rows for plasticity={plast}", file=sys.stderr)
            continue
        feat_only = [c for c in sub.columns if c not in ("plasticity", "init", "evolve_seed")]
        feat_only = [c for c in feat_only if pd.to_numeric(sub[c], errors="coerce").notna().any()]
        X = sub[feat_only].to_numpy(dtype=float)
        X = np.where(np.isfinite(X), X, np.nan)
        imp = SimpleImputer(strategy="median")
        X_imp = imp.fit_transform(X)
        mask = np.isfinite(X_imp).all(axis=1)
        sub = sub.loc[mask].reset_index(drop=True)
        X_imp = X_imp[mask]
        if X_imp.shape[0] < 8:
            print(f"[warn] too few genomes for PCA/LDA: {plast} (n={X_imp.shape[0]})", file=sys.stderr)
            continue

        scaler = StandardScaler()
        Xs = scaler.fit_transform(X_imp)

        # PCA
        n_comp = min(8, Xs.shape[1], max(2, Xs.shape[0] - 1))
        pca = PCA(n_components=min(2, n_comp), random_state=0)
        Zp = pca.fit_transform(Xs)
        _plot_embedding(
            Xs,
            sub["init"].to_numpy(),
            Zp,
            title=(
                f"PCA (per genome) — {PLAST_TITLES.get(plast, plast)}\n"
                "All P(sel|child/mother state)×conditions + means + TTD + n_ticks; median-imputed, scaled"
            ),
            xlabel=f"PC1 ({pca.explained_variance_ratio_[0]*100:.1f}% var)",
            ylabel=f"PC2 ({pca.explained_variance_ratio_[1]*100:.1f}% var)" if Zp.shape[1] > 1 else "PC2",
            out_png=os.path.join(out_dir, f"embedding_pca__{plast}.png"),
        )
        load = pd.DataFrame(pca.components_.T, index=feat_only, columns=[f"PC{i+1}" for i in range(pca.components_.shape[0])])
        load.to_csv(os.path.join(out_dir, f"pca_loadings__{plast}.csv"))

        emb = sub[["plasticity", "init", "evolve_seed"]].copy()
        emb["pc1"] = Zp[:, 0]
        emb["pc2"] = Zp[:, 1] if Zp.shape[1] > 1 else np.nan
        emb.to_csv(os.path.join(out_dir, f"embedding_pca_genomes__{plast}.csv"), index=False)

        # LDA (needs >=2 classes present)
        y = sub["init"].to_numpy()
        classes = sorted(set(y.tolist()))
        if len(classes) < 2:
            print(f"[warn] LDA skipped (need 2+ init classes): {plast}", file=sys.stderr)
            continue
        n_ld = min(len(classes) - 1, Xs.shape[1])
        lda = LinearDiscriminantAnalysis(n_components=max(1, min(2, n_ld)), solver="svd")
        Zl = lda.fit_transform(Xs, y)
        if Zl.shape[1] == 1:
            Z2 = np.column_stack([Zl[:, 0], np.zeros(len(Zl))])
            xl, yl = "LD1", "(pad)"
        else:
            Z2 = Zl[:, :2]
            xl, yl = "LD1", "LD2"
        _plot_embedding(
            Xs,
            y,
            Z2,
            title=f"LDA (supervised by init) — {PLAST_TITLES.get(plast, plast)}",
            xlabel=xl,
            ylabel=yl,
            out_png=os.path.join(out_dir, f"embedding_lda__{plast}.png"),
        )
        out_ld = sub[["plasticity", "init", "evolve_seed"]].copy()
        out_ld["ld1"] = Zl[:, 0]
        if Zl.shape[1] > 1:
            out_ld["ld2"] = Zl[:, 1]
        out_ld.to_csv(os.path.join(out_dir, f"lda_projected_genomes__{plast}.csv"), index=False)

        if not args.no_lda_loadings:
            _plot_lda_loadings_bars(
                lda,
                feat_only,
                plast=plast,
                out_png=os.path.join(out_dir, f"lda_loadings_ld1_bars__{plast}.png"),
            )
        if not args.no_lda_per_world:
            _plot_lda_scatter_per_world(
                Xs,
                y,
                feat_only,
                plast=plast,
                condition_order=condition_order,
                out_png=os.path.join(out_dir, f"lda_scatter_per_world__{plast}.png"),
            )

    idx = os.path.join(out_dir, "init_signature_index.txt")
    with open(idx, "w", encoding="utf-8") as f:
        f.write("Init signature (ecology scatter + PCA/LDA) outputs:\n")
        for name in sorted(os.listdir(out_dir)):
            if name.endswith((".png", ".csv", ".txt")):
                f.write(f"  {name}\n")
    print("[ok] wrote", idx)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
