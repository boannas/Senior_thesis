#!/usr/bin/env python3
"""
plot_tier1_paper_figures.py
============================
Tier-1 figures for SAB 2026 maternal-instinct paper.

Outputs (under --out-dir):

  child_ttd_with_bounds_trajectory.png
      [Tier 1.1] Child TTD across stages, per init x plasticity, overlaid with
      passive (lower-bound) and active-strong-genome (reference upper) bands.

  child_ttd_with_bounds_box_<stage>.png
      [Tier 1.1] Per-stage boxplots of child TTD by init x plasticity, with
      reference bands.

  headroom_closed.csv + headroom_closed_heatmap_<first_stage>.png
      + headroom_closed_heatmap_<last_stage>.png (+ legacy headroom_closed_heatmap.png
      = last stage) [Tier 1.2] Same headroom metric; two heatmaps share one color scale
      so initial vs emergent are directly comparable.

  ttd_stats.csv
      [Tier 1.3] For each (plasticity, init) cell: bootstrap 95% CI on median TTD
      at emergent, plus Mann-Whitney U p-value and Cliff's delta vs initial stage.

  signatures_pca.png + signatures_pca_features.csv (+ *_loadings.csv)
      [Tier 1.4a] PCA of behavioral feature vectors per genome at emergent.
      If stage CSVs include extended columns (from plot_runlog_conditional_behavior
      --extended), PCA uses core + P(sel|condition) + physiology means automatically.
      Color = init, marker = plasticity.

  signatures_lda.png + signatures_lda_features.csv (+ *_loadings.csv)
      [Tier 1.4b] LDA on the same scaled matrix, labels = init (three mother types).
      Up to 2 discriminant axes (max for K=3 classes). Same scatter style as PCA.

Inputs
------
--passive-csv          survival_experiment_replicates.csv from passive run
--active-csv           survival_experiment_replicates.csv from active+strong genome run
--stage-csv L=path     repeatable; conditional_metrics_<label>.csv files
--max-ticks N          horizon used for normalization (default 1000)
--out-dir DIR          output directory

Notes
-----
Reference bounds are normalized as child_death_tick / --max-ticks so they line
up with the `child_ttd_norm` column from the stage CSVs.

Dependencies: pandas, matplotlib, scikit-learn. scipy is optional; without it,
Mann-Whitney p-values use a tie-corrected normal approximation (close to scipy
for moderate sample sizes).
"""

from __future__ import annotations

import argparse
import math
import os
import sys
import warnings
from typing import Iterable

import numpy as np

try:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import pandas as pd
    from sklearn.decomposition import PCA
    from sklearn.discriminant_analysis import LinearDiscriminantAnalysis
    from sklearn.preprocessing import LabelEncoder, StandardScaler
except ImportError as e:
    raise SystemExit(f"Requires pandas, matplotlib, scikit-learn: {e}") from e


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


INIT_ORDER = ["anti_maternal", "random_uniform", "pro_maternal"]
PLAST_ORDER = ["E2", "P1", "P2"]
INIT_COLORS = {
    "anti_maternal": "#c0392b",
    "random_uniform": "#2980b9",
    "pro_maternal": "#27ae60",
}
PLAST_MARKERS = {"E2": "o", "P1": "s", "P2": "^"}

PASSIVE_COLOR = "#7f7f7f"
ACTIVE_COLOR = "#9467bd"
# In-axes text for the active reference line; full interpretation in figure caption.
ACTIVE_REF_LINE_LABEL = "active (single pro reference)"
REFERENCE_CAPTION = (
    "Passive = lower bound; upper = one fixed pro genome — "
    "emergent population medians may exceed that single-policy reference."
)


# Core metrics (always used if present).
PCA_CORE_FEATURES = [
    "P_maternal_given_hungry",
    "P_forage_given_hungry",
    "P_protect_given_fear",
    "closeness_mean_alive",
    "injury_mean_alive",
    "fear_mean_alive",
]

# Extended physiology means (optional; from --extended conditional CSV).
PCA_OPTIONAL_MEAN_FEATURES = [
    "warmth_mean_alive",
    "energy_mean_alive",
    "stress_mean_alive",
    "fatigue_mean_alive",
    "bonding_mean_alive",
]


def _pca_behavior_columns(df: pd.DataFrame) -> list[str]:
    """
    Build feature list: core + all P_sel_*_given_* + optional mean_* columns
    present in df. Stable ordering for reproducible loadings.
    """
    out: list[str] = []
    for c in PCA_CORE_FEATURES:
        if c in df.columns:
            out.append(c)
    psel = sorted(
        c
        for c in df.columns
        if c.startswith("P_sel_") and "_given_" in c
    )
    out.extend(psel)
    for c in PCA_OPTIONAL_MEAN_FEATURES:
        if c in df.columns:
            out.append(c)
    return out


def _load_reference(csv_path: str, max_ticks: int, label: str) -> dict | None:
    if not csv_path or not os.path.isfile(csv_path):
        print(f"[warn] {label} CSV not found: {csv_path}", file=sys.stderr)
        return None
    df = pd.read_csv(csv_path)
    if "child_death_tick" not in df.columns:
        print(f"[warn] {label} CSV missing child_death_tick: {csv_path}", file=sys.stderr)
        return None
    arr = pd.to_numeric(df["child_death_tick"], errors="coerce").to_numpy(dtype=float)
    arr = arr[np.isfinite(arr)] / float(max_ticks)
    if arr.size == 0:
        print(f"[warn] {label} CSV has no finite values", file=sys.stderr)
        return None
    return {
        "label": label,
        "values": arr,
        "median": float(np.median(arr)),
        "q25": float(np.percentile(arr, 25)),
        "q75": float(np.percentile(arr, 75)),
        "n": int(arr.size),
    }


def _draw_ref(ax, ref, color, name, *, fontsize: float = 8) -> None:
    if ref is None:
        return
    xlim = ax.get_xlim()
    ax.axhspan(ref["q25"], ref["q75"], color=color, alpha=0.10, zorder=0)
    ax.axhline(ref["median"], color=color, linestyle="--", linewidth=1.4, zorder=1)
    ax.text(
        xlim[1],
        ref["median"],
        f" {name} (med={ref['median']:.2f})",
        color=color,
        fontsize=fontsize,
        va="center",
        ha="left",
        clip_on=False,
    )


def _trajectory_plot(
    stage_dfs: dict[str, pd.DataFrame],
    stage_order: list[str],
    passive: dict | None,
    active: dict | None,
    out_png: str,
    title: str,
    *,
    reference_caption: str | None = None,
) -> None:
    fig, axes = plt.subplots(1, 3, figsize=(14.5, 4.4), sharey=True)
    for ax, plast in zip(axes, PLAST_ORDER):
        for init in INIT_ORDER:
            xs, mids, los, his = [], [], [], []
            for i, stage in enumerate(stage_order):
                df = stage_dfs.get(stage)
                if df is None or df.empty:
                    continue
                sub = df[(df["plasticity"] == plast) & (df["init"] == init)]
                per_genome = sub.groupby("evolve_seed", as_index=False)["child_ttd_norm"].mean()
                vals = per_genome["child_ttd_norm"].dropna().to_numpy(dtype=float)
                if vals.size == 0:
                    continue
                xs.append(i)
                mids.append(float(np.median(vals)))
                los.append(float(np.percentile(vals, 25)))
                his.append(float(np.percentile(vals, 75)))
            if not xs:
                continue
            xs_a = np.asarray(xs, dtype=float)
            ax.fill_between(xs_a, los, his, color=INIT_COLORS[init], alpha=0.18, linewidth=0)
            ax.plot(
                xs_a,
                mids,
                "-o",
                color=INIT_COLORS[init],
                linewidth=1.9,
                markersize=5,
                label=init.replace("_", " "),
            )
        ax.set_title(plast, fontsize=12)
        ax.set_xticks(range(len(stage_order)))
        ax.set_xticklabels(stage_order)
        ax.set_xlim(-0.3, len(stage_order) - 0.7)
        _draw_ref(ax, passive, PASSIVE_COLOR, "passive")
        _draw_ref(
            ax,
            active,
            ACTIVE_COLOR,
            ACTIVE_REF_LINE_LABEL,
            fontsize=6.5,
        )
    axes[0].set_ylabel("child TTD (normalized)")
    axes[-1].legend(loc="lower right", fontsize=9, frameon=False)
    if reference_caption:
        fig.suptitle(f"{title}\n{reference_caption}", fontsize=10, y=1.05)
        fig.tight_layout(rect=[0, 0, 1, 0.90])
    else:
        fig.suptitle(title, fontsize=12, y=1.02)
        fig.tight_layout()
    fig.savefig(out_png, dpi=200, bbox_inches="tight")
    plt.close(fig)
    print("[ok] wrote", out_png)


def _boxplot_with_bounds(
    df: pd.DataFrame,
    *,
    stage_label: str,
    passive: dict | None,
    active: dict | None,
    out_png: str,
) -> None:
    fig, axes = plt.subplots(1, 3, figsize=(13.5, 4.2), sharey=True)
    rng = np.random.default_rng(0)
    for ax, plast in zip(axes, PLAST_ORDER):
        sub = df[df["plasticity"] == plast]
        data = []
        for init in INIT_ORDER:
            vals = pd.to_numeric(
                sub.loc[sub["init"] == init, "child_ttd_norm"], errors="coerce"
            ).dropna().to_numpy(dtype=float)
            data.append(vals)
        labels = [s.replace("_", " ") for s in INIT_ORDER]
        bp = ax.boxplot(
            data,
            labels=labels,
            patch_artist=True,
            widths=0.6,
            showmeans=True,
            medianprops=dict(color="#F28E2B", linewidth=1.6),
            meanprops=dict(
                marker="^",
                markerfacecolor="#2ca02c",
                markeredgecolor="#2ca02c",
                markersize=6,
            ),
        )
        for patch, init in zip(bp["boxes"], INIT_ORDER):
            patch.set_facecolor(INIT_COLORS[init])
            patch.set_alpha(0.18)
            patch.set_edgecolor(INIT_COLORS[init])
            patch.set_linewidth(1.0)
        for i, vals in enumerate(data):
            if len(vals) == 0:
                continue
            x = rng.normal(i + 1, 0.05, size=len(vals))
            ax.scatter(
                x,
                vals,
                s=10,
                alpha=0.22,
                color=INIT_COLORS[INIT_ORDER[i]],
                edgecolors="none",
                zorder=3,
            )
        ax.set_title(plast, fontsize=12)
        ax.set_xlim(0.5, len(INIT_ORDER) + 0.5)
        _draw_ref(ax, passive, PASSIVE_COLOR, "passive")
        _draw_ref(
            ax,
            active,
            ACTIVE_COLOR,
            ACTIVE_REF_LINE_LABEL,
            fontsize=6.5,
        )
    axes[0].set_ylabel("child TTD (normalized)")
    fig.suptitle(
        f"Child TTD with reference bounds — stage = {stage_label}",
        fontsize=12,
        y=1.02,
    )
    fig.tight_layout()
    fig.savefig(out_png, dpi=200, bbox_inches="tight")
    plt.close(fig)
    print("[ok] wrote", out_png)


def _per_genome_ttd(df: pd.DataFrame) -> pd.DataFrame:
    if df.empty:
        return df
    return (
        df.groupby(["plasticity", "init", "evolve_seed"], as_index=False)["child_ttd_norm"]
        .mean()
    )


def _write_headroom_heatmap(
    stage_subset: pd.DataFrame,
    *,
    stage_name: str,
    out_png: str,
    sym_vmax: float,
) -> None:
    if stage_subset.empty:
        print(f"[warn] no headroom rows for stage={stage_name}; skip {out_png}", file=sys.stderr)
        return
    pivot = stage_subset.pivot(
        index="plasticity", columns="init", values="headroom_closed_pct"
    )
    pivot = pivot.reindex(index=PLAST_ORDER, columns=INIT_ORDER)

    fig, ax = plt.subplots(figsize=(6.6, 3.6))
    ax.grid(False)
    arr = pivot.to_numpy(dtype=float)
    im = ax.imshow(
        arr,
        cmap="RdYlGn",
        vmin=-sym_vmax,
        vmax=sym_vmax,
        aspect="auto",
    )
    ax.set_xticks(range(len(INIT_ORDER)))
    ax.set_xticklabels([s.replace("_", " ") for s in INIT_ORDER])
    ax.set_yticks(range(len(PLAST_ORDER)))
    ax.set_yticklabels(PLAST_ORDER)
    for i in range(arr.shape[0]):
        for j in range(arr.shape[1]):
            v = arr[i, j]
            if np.isfinite(v):
                ax.text(
                    j,
                    i,
                    f"{v:.0f}%",
                    ha="center",
                    va="center",
                    color="black",
                    fontsize=10,
                )
    ax.set_title(
        f"Headroom closed — stage = {stage_name}\n"
        "(median TTD − passive) / (active − passive)",
        fontsize=11,
    )
    with warnings.catch_warnings():
        warnings.simplefilter("ignore", category=matplotlib.MatplotlibDeprecationWarning)
        cbar = fig.colorbar(im, ax=ax, label="% headroom closed")
    cbar.ax.grid(False)
    fig.tight_layout()
    fig.savefig(out_png, dpi=200, bbox_inches="tight")
    plt.close(fig)
    print("[ok] wrote", out_png)


def _headroom_table(
    stage_dfs: dict[str, pd.DataFrame],
    stage_order: list[str],
    passive: dict | None,
    active: dict | None,
    out_csv: str,
    *,
    heatmap_dir: str | None = None,
) -> pd.DataFrame:
    if passive is None or active is None:
        print("[warn] skipping headroom: passive or active reference missing", file=sys.stderr)
        return pd.DataFrame()

    p_med = passive["median"]
    a_med = active["median"]
    if a_med <= p_med:
        print(
            f"[warn] active median ({a_med:.3f}) <= passive median ({p_med:.3f}); "
            "headroom undefined",
            file=sys.stderr,
        )
        return pd.DataFrame()

    rows = []
    for stage in stage_order:
        df = stage_dfs.get(stage)
        if df is None or df.empty:
            continue
        pg = _per_genome_ttd(df)
        for plast in PLAST_ORDER:
            for init in INIT_ORDER:
                vals = pg.loc[
                    (pg["plasticity"] == plast) & (pg["init"] == init),
                    "child_ttd_norm",
                ].dropna().to_numpy(dtype=float)
                if vals.size == 0:
                    continue
                med = float(np.median(vals))
                headroom = (med - p_med) / (a_med - p_med)
                rows.append(
                    {
                        "stage": stage,
                        "plasticity": plast,
                        "init": init,
                        "n_genomes": int(vals.size),
                        "median_ttd_norm": med,
                        "passive_median": p_med,
                        "active_median": a_med,
                        "headroom_closed": headroom,
                        "headroom_closed_pct": 100.0 * headroom,
                    }
                )
    out = pd.DataFrame(rows)
    out.to_csv(out_csv, index=False)
    print("[ok] wrote", out_csv)

    hdir = heatmap_dir if heatmap_dir is not None else os.path.dirname(out_csv)
    os.makedirs(hdir, exist_ok=True)

    pct = out["headroom_closed_pct"].to_numpy(dtype=float)
    pct = pct[np.isfinite(pct)]
    sym_vmax = float(np.nanmax(np.abs(pct))) if pct.size else 100.0
    sym_vmax = max(sym_vmax, 100.0)

    first_stage = stage_order[0]
    last_stage = stage_order[-1]

    first_df = out[out["stage"] == first_stage]
    last_df = out[out["stage"] == last_stage]

    # One PNG per distinct stage among {first, last} (shared symmetric color scale).
    seen_stage: list[str] = []
    for st in (first_stage, last_stage):
        if st in seen_stage:
            continue
        seen_stage.append(st)
        sdf = out[out["stage"] == st]
        if sdf.empty:
            continue
        _write_headroom_heatmap(
            sdf,
            stage_name=st,
            out_png=os.path.join(hdir, f"headroom_closed_heatmap_{st}.png"),
            sym_vmax=sym_vmax,
        )

    # Legacy filename (emergent / last stage) for older notes and scripts.
    if not last_df.empty:
        _write_headroom_heatmap(
            last_df,
            stage_name=last_stage,
            out_png=os.path.join(hdir, "headroom_closed_heatmap.png"),
            sym_vmax=sym_vmax,
        )

    return out


def _norm_cdf(z: float) -> float:
    return 0.5 * (1.0 + math.erf(z / math.sqrt(2.0)))


def _mannwhitney_greater_u_p(x: np.ndarray, y: np.ndarray) -> tuple[float, float]:
    """
    Mann-Whitney U, one-sided alternative: x stochastically greater than y.
    Uses scipy when installed; otherwise a tie-corrected normal approximation.
    """
    try:
        from scipy.stats import mannwhitneyu

        r = mannwhitneyu(x, y, alternative="greater")
        return float(r.statistic), float(r.pvalue)
    except ImportError:
        pass
    xa = np.asarray(x, dtype=float)
    ya = np.asarray(y, dtype=float)
    n1, n2 = int(xa.size), int(ya.size)
    N = n1 + n2
    if n1 < 1 or n2 < 1:
        return float("nan"), float("nan")
    comb = np.concatenate([xa, ya])
    ranks = pd.Series(comb).rank(method="average").to_numpy(dtype=float)
    r1 = ranks[:n1]
    u1 = float(np.sum(r1) - n1 * (n1 + 1) / 2.0)
    mu = n1 * n2 / 2.0
    _, counts = np.unique(comb, return_counts=True)
    tie_sum = float(np.sum(counts**3 - counts))
    var = (n1 * n2 / (N * (N - 1))) * ((N**3 - N) / 12.0 - tie_sum / 12.0)
    if var <= 0:
        return u1, float("nan")
    sigma = math.sqrt(var)
    z = (u1 - 0.5 - mu) / sigma
    p = 1.0 - _norm_cdf(z)
    return u1, float(np.clip(p, 0.0, 1.0))


def _cliffs_delta(x: np.ndarray, y: np.ndarray) -> float:
    if x.size == 0 or y.size == 0:
        return float("nan")
    gt = 0
    lt = 0
    for xi in x:
        gt += int(np.sum(xi > y))
        lt += int(np.sum(xi < y))
    return float((gt - lt) / (x.size * y.size))


def _bootstrap_median_ci(
    x: np.ndarray, n_boot: int = 1000, alpha: float = 0.05, seed: int = 0
) -> tuple[float, float]:
    if x.size == 0:
        return (float("nan"), float("nan"))
    rng = np.random.default_rng(seed)
    idx = rng.integers(0, x.size, size=(n_boot, x.size))
    boots = np.median(x[idx], axis=1)
    lo = float(np.percentile(boots, 100 * alpha / 2.0))
    hi = float(np.percentile(boots, 100 * (1 - alpha / 2.0)))
    return (lo, hi)


def _stats_table(
    stage_dfs: dict[str, pd.DataFrame],
    stage_order: list[str],
    out_csv: str,
) -> None:
    if len(stage_order) < 2:
        print("[warn] need at least 2 stages for stats", file=sys.stderr)
        return
    first_stage = stage_order[0]
    last_stage = stage_order[-1]
    df_first = stage_dfs.get(first_stage, pd.DataFrame())
    df_last = stage_dfs.get(last_stage, pd.DataFrame())
    if df_first.empty or df_last.empty:
        print(
            f"[warn] missing stage data ({first_stage}, {last_stage}) for stats",
            file=sys.stderr,
        )
        return

    pg_first = _per_genome_ttd(df_first)
    pg_last = _per_genome_ttd(df_last)

    rows = []
    for plast in PLAST_ORDER:
        for init in INIT_ORDER:
            x_first = pg_first.loc[
                (pg_first["plasticity"] == plast) & (pg_first["init"] == init),
                "child_ttd_norm",
            ].dropna().to_numpy(dtype=float)
            x_last = pg_last.loc[
                (pg_last["plasticity"] == plast) & (pg_last["init"] == init),
                "child_ttd_norm",
            ].dropna().to_numpy(dtype=float)
            if x_first.size == 0 or x_last.size == 0:
                continue
            try:
                u, p = _mannwhitney_greater_u_p(x_last, x_first)
            except Exception:
                u, p = (float("nan"), float("nan"))
            delta = _cliffs_delta(x_last, x_first)
            ci_lo, ci_hi = _bootstrap_median_ci(x_last)
            rows.append(
                {
                    "plasticity": plast,
                    "init": init,
                    "n_first": int(x_first.size),
                    "n_last": int(x_last.size),
                    f"median_{first_stage}": float(np.median(x_first)),
                    f"median_{last_stage}": float(np.median(x_last)),
                    f"ci95_lo_{last_stage}": ci_lo,
                    f"ci95_hi_{last_stage}": ci_hi,
                    f"mannwhitney_u_{last_stage}_gt_{first_stage}": u,
                    f"mannwhitney_p_{last_stage}_gt_{first_stage}": p,
                    f"cliffs_delta_{last_stage}_vs_{first_stage}": delta,
                }
            )
    out = pd.DataFrame(rows)
    out.to_csv(out_csv, index=False)
    print("[ok] wrote", out_csv)


def _emergent_signature_agg_scaled(
    df_emergent: pd.DataFrame,
) -> tuple[pd.DataFrame, list[str], np.ndarray] | None:
    """
    Per-genome mean of behavioral features at emergent + child_ttd_norm;
    drop constant columns; z-score columns. Shared by PCA and LDA.
    """
    if df_emergent.empty:
        print("[warn] empty emergent df; skipping signature projection", file=sys.stderr)
        return None

    feat_cols = _pca_behavior_columns(df_emergent)
    if not feat_cols:
        print("[warn] no PCA feature columns found; skipping", file=sys.stderr)
        return None

    agg = (
        df_emergent.groupby(["plasticity", "init", "evolve_seed"], as_index=False)[
            feat_cols + ["child_ttd_norm"]
        ]
        .mean()
    )
    if len(agg) < 4:
        print(
            f"[warn] only {len(agg)} genomes available for signature projection; skipping",
            file=sys.stderr,
        )
        return None

    for c in feat_cols:
        med = float(np.nanmedian(agg[c].to_numpy(dtype=float)))
        if np.isfinite(med):
            agg[c] = agg[c].fillna(med)
        else:
            agg[c] = agg[c].fillna(0.0)

    X = agg[feat_cols].to_numpy(dtype=float)
    std = X.std(axis=0)
    keep_mask = std > 1e-12
    dropped = [feat_cols[i] for i in range(len(feat_cols)) if not keep_mask[i]]
    if dropped:
        print(
            f"[info] signature projection dropping {len(dropped)} constant features: "
            f"{dropped[:8]}{'...' if len(dropped) > 8 else ''}"
        )
    feat_use = [feat_cols[i] for i in range(len(feat_cols)) if keep_mask[i]]
    if len(feat_use) < 2:
        print(
            "[warn] fewer than 2 non-constant signature features; skipping",
            file=sys.stderr,
        )
        return None

    X = agg[feat_use].to_numpy(dtype=float)
    print(
        f"[info] signature matrix using {len(feat_use)} features "
        f"(extended={any(c.startswith('P_sel_') for c in feat_use)})"
    )
    X_s = StandardScaler().fit_transform(X)
    return agg, feat_use, X_s


def _pca_signatures(
    df_emergent: pd.DataFrame,
    out_png: str,
    out_csv: str,
    *,
    prep: tuple[pd.DataFrame, list[str], np.ndarray] | None = None,
) -> None:
    if prep is None:
        prep = _emergent_signature_agg_scaled(df_emergent)
    if prep is None:
        return
    agg, feat_use, X_s = prep

    pca = PCA(n_components=2, random_state=0)
    Z = pca.fit_transform(X_s)
    evr = pca.explained_variance_ratio_

    agg = agg.copy()
    agg["pc1"] = Z[:, 0]
    agg["pc2"] = Z[:, 1]
    agg["pca_feature_list"] = ",".join(feat_use)
    agg.to_csv(out_csv, index=False)
    print("[ok] wrote", out_csv)

    loadings = pd.DataFrame(
        pca.components_.T,
        columns=["pc1", "pc2"],
        index=feat_use,
    )
    loadings_path = os.path.splitext(out_csv)[0] + "_loadings.csv"
    loadings.to_csv(loadings_path)
    print("[ok] wrote", loadings_path)

    fig, ax = plt.subplots(figsize=(7.2, 5.6))
    for init in INIT_ORDER:
        for plast in PLAST_ORDER:
            sub = agg[(agg["init"] == init) & (agg["plasticity"] == plast)]
            if sub.empty:
                continue
            ax.scatter(
                sub["pc1"],
                sub["pc2"],
                s=42,
                c=INIT_COLORS[init],
                marker=PLAST_MARKERS[plast],
                edgecolors="white",
                linewidths=0.6,
                alpha=0.85,
                label=f"{init.replace('_', ' ')} / {plast}",
            )
    ax.set_xlabel(f"PC1 ({evr[0] * 100:.1f}% var)")
    ax.set_ylabel(f"PC2 ({evr[1] * 100:.1f}% var)")
    ax.set_title(
        "Emergent behavioral signatures (extended features)\n"
        "PCA per genome; color = init, marker = plasticity",
        fontsize=11,
    )
    ax.legend(
        loc="center left",
        bbox_to_anchor=(1.02, 0.5),
        fontsize=8,
        frameon=False,
        ncol=1,
    )
    fig.tight_layout()
    fig.savefig(out_png, dpi=200, bbox_inches="tight")
    plt.close(fig)
    print("[ok] wrote", out_png)


def _lda_signatures(
    df_emergent: pd.DataFrame,
    out_png: str,
    out_csv: str,
    *,
    prep: tuple[pd.DataFrame, list[str], np.ndarray] | None = None,
) -> None:
    """
    Linear discriminant analysis on the same scaled feature matrix as PCA,
    with class label = init (mother type). Visualization matches PCA style.
    """
    if prep is None:
        prep = _emergent_signature_agg_scaled(df_emergent)
    if prep is None:
        return
    agg, feat_use, X_s = prep

    unk = agg["init"].astype(str)
    bad = ~unk.isin(set(INIT_ORDER))
    if bad.any():
        print(
            f"[warn] LDA dropping {int(bad.sum())} rows with init not in {INIT_ORDER}",
            file=sys.stderr,
        )
        agg = agg.loc[~bad].copy()
        X_s = X_s[~bad.to_numpy()]

    if len(agg) < 4:
        print("[warn] too few rows after init filter; skipping LDA", file=sys.stderr)
        return

    le = LabelEncoder()
    y = le.fit_transform(agg["init"].astype(str))
    n_classes = int(len(le.classes_))
    if n_classes < 2:
        print("[warn] LDA needs ≥2 distinct init labels; skipping", file=sys.stderr)
        return

    for lab in le.classes_:
        n_lab = int((agg["init"].astype(str) == lab).sum())
        if n_lab < 2:
            print(
                f"[warn] LDA needs ≥2 genomes per init class (got n={n_lab} for {lab!r}); skipping",
                file=sys.stderr,
            )
            return

    n_comp = min(2, n_classes - 1, X_s.shape[1])
    lda = LinearDiscriminantAnalysis(n_components=n_comp, solver="svd")
    Z = lda.fit_transform(X_s, y)
    evr = getattr(lda, "explained_variance_ratio_", None)
    if evr is None or len(evr) == 0:
        evr = np.ones(max(1, n_comp), dtype=float)

    agg = agg.copy()
    agg["ld1"] = Z[:, 0]
    if n_comp > 1:
        agg["ld2"] = Z[:, 1]
    else:
        agg["ld2"] = 0.0
    agg["lda_label"] = "init"
    agg["lda_feature_list"] = ",".join(feat_use)
    agg.to_csv(out_csv, index=False)
    print("[ok] wrote", out_csv)

    scal = np.asarray(lda.scalings_, dtype=float)
    cols = ["ld1"] + (["ld2"] if scal.shape[1] > 1 else [])
    loadings = pd.DataFrame(scal[:, : len(cols)], columns=cols, index=feat_use)
    loadings_path = os.path.splitext(out_csv)[0] + "_loadings.csv"
    loadings.to_csv(loadings_path)
    print("[ok] wrote", loadings_path)

    rng = np.random.default_rng(0)
    fig, ax = plt.subplots(figsize=(7.2, 5.6))
    for init in INIT_ORDER:
        for plast in PLAST_ORDER:
            sub = agg[(agg["init"] == init) & (agg["plasticity"] == plast)]
            if sub.empty:
                continue
            if n_comp > 1:
                y_plot = sub["ld2"].to_numpy(dtype=float)
            else:
                y_plot = rng.uniform(-0.12, 0.12, size=len(sub))
            ax.scatter(
                sub["ld1"],
                y_plot,
                s=42,
                c=INIT_COLORS[init],
                marker=PLAST_MARKERS[plast],
                edgecolors="white",
                linewidths=0.6,
                alpha=0.85,
                label=f"{init.replace('_', ' ')} / {plast}",
            )

    if n_comp > 1:
        ax.set_xlabel(f"LD1 ({evr[0] * 100:.1f}% between-class var)")
        ax.set_ylabel(f"LD2 ({evr[1] * 100:.1f}% between-class var)")
    else:
        ax.set_xlabel(f"LD1 ({evr[0] * 100:.1f}% between-class var)")
        ax.set_ylabel("vertical jitter (1D LDA only; not an axis)")
        ax.set_yticks([])

    ax.set_title(
        "Emergent behavioral signatures (extended features)\n"
        "LDA per genome (supervised on init); color = init, marker = plasticity",
        fontsize=11,
    )
    ax.legend(
        loc="center left",
        bbox_to_anchor=(1.02, 0.5),
        fontsize=8,
        frameon=False,
        ncol=1,
    )
    fig.tight_layout()
    fig.savefig(out_png, dpi=200, bbox_inches="tight")
    plt.close(fig)
    print("[ok] wrote", out_png)


def _write_pca_and_lda(emergent_df: pd.DataFrame, out_dir: str) -> None:
    """Build one scaled signature matrix; write PCA and LDA figures + CSVs."""
    prep = _emergent_signature_agg_scaled(emergent_df)
    if prep is None:
        return
    _pca_signatures(
        emergent_df,
        out_png=os.path.join(out_dir, "signatures_pca.png"),
        out_csv=os.path.join(out_dir, "signatures_pca_features.csv"),
        prep=prep,
    )
    _lda_signatures(
        emergent_df,
        out_png=os.path.join(out_dir, "signatures_lda.png"),
        out_csv=os.path.join(out_dir, "signatures_lda_features.csv"),
        prep=prep,
    )


def _load_stage_csv(spec: str) -> tuple[str, pd.DataFrame]:
    if "=" not in spec:
        raise SystemExit(f"--stage-csv expects label=path, got: {spec}")
    label, path = spec.split("=", 1)
    label = label.strip()
    path = os.path.abspath(path.strip())
    if not os.path.isfile(path):
        raise SystemExit(f"stage CSV not found: {path}")
    df = pd.read_csv(path)
    needed = {"plasticity", "init", "evolve_seed", "child_ttd_norm"}
    missing = needed - set(df.columns)
    if missing:
        raise SystemExit(f"stage CSV {path} missing columns: {missing}")
    return label, df


def main(argv: Iterable[str] | None = None) -> int:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    ap.add_argument("--passive-csv", required=True)
    ap.add_argument("--active-csv", required=True)
    ap.add_argument(
        "--stage-csv",
        action="append",
        required=True,
        help="Repeatable: label=path/to/conditional_metrics_<label>.csv",
    )
    ap.add_argument("--max-ticks", type=int, default=1000)
    ap.add_argument("--out-dir", required=True)
    args = ap.parse_args(list(argv) if argv is not None else None)

    out_dir = os.path.abspath(args.out_dir)
    os.makedirs(out_dir, exist_ok=True)

    try:
        import scipy  # noqa: F401
    except ImportError:
        print(
            "[info] scipy not installed — Mann-Whitney p-values in ttd_stats.csv use a "
            "normal approximation (pip install scipy to match scipy.stats exactly).",
            file=sys.stderr,
        )

    passive = _load_reference(args.passive_csv, args.max_ticks, "passive")
    active = _load_reference(args.active_csv, args.max_ticks, "active(pro)")

    stages: dict[str, pd.DataFrame] = {}
    for spec in args.stage_csv:
        label, df = _load_stage_csv(spec)
        stages[label] = df
    stage_order = list(stages.keys())
    print(f"[info] stages: {stage_order}")

    _trajectory_plot(
        stages,
        stage_order,
        passive,
        active,
        os.path.join(out_dir, "child_ttd_with_bounds_trajectory.png"),
        title=f"Child TTD across stages (max_ticks={args.max_ticks})",
        reference_caption=REFERENCE_CAPTION,
    )

    for label, df in stages.items():
        _boxplot_with_bounds(
            df,
            stage_label=label,
            passive=passive,
            active=active,
            out_png=os.path.join(out_dir, f"child_ttd_with_bounds_box_{label}.png"),
        )

    _headroom_table(
        stages,
        stage_order,
        passive,
        active,
        out_csv=os.path.join(out_dir, "headroom_closed.csv"),
        heatmap_dir=out_dir,
    )

    _stats_table(stages, stage_order, os.path.join(out_dir, "ttd_stats.csv"))

    emergent_df = stages.get(stage_order[-1], pd.DataFrame())
    _write_pca_and_lda(emergent_df, out_dir)

    summary_lines = []
    summary_lines.append(f"max_ticks: {args.max_ticks}")
    if passive is not None:
        summary_lines.append(
            f"passive: n={passive['n']} median={passive['median']:.4f} "
            f"q25={passive['q25']:.4f} q75={passive['q75']:.4f}"
        )
    if active is not None:
        summary_lines.append(
            f"active(pro): n={active['n']} median={active['median']:.4f} "
            f"q25={active['q25']:.4f} q75={active['q75']:.4f}"
        )
    if passive is not None and active is not None and active["median"] > passive["median"]:
        summary_lines.append(
            f"available headroom: {active['median'] - passive['median']:.4f} "
            f"(normalized TTD units)"
        )
    summary_path = os.path.join(out_dir, "tier1_summary.txt")
    with open(summary_path, "w", encoding="utf-8") as f:
        f.write("\n".join(summary_lines) + "\n")
    print("[ok] wrote", summary_path)
    return 0


if __name__ == "__main__":
    sys.exit(main())
