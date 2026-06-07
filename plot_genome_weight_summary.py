#!/usr/bin/env python3
"""
plot_genome_weight_summary.py
=============================
Summarize evolved motivation genomes (`final_genome.json`) by init group:
anti_maternal vs random_uniform vs pro_maternal.

Outputs (under --out-dir)
------------------------
- genome_weights_long.csv
    One row per genome x weight key (long format).
- genome_weights_summary.csv
    Mean/std/N per (plasticity, init, weight_key).
- genome_weights_mean_std.png
    Errorbar plot: mean ± std for each weight key, colored by init.
    One subplot per plasticity (E2/P1/P2) unless --collapse-plasticity.

Usage
-----
python3 plot_genome_weight_summary.py \
  --root Emergence_results/normal \
  --out-dir Emergence_results/normal/figures_weight_summary
"""

from __future__ import annotations

import argparse
import json
import os
import re
import sys
from glob import glob

import numpy as np

try:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import pandas as pd
except ImportError as e:
    raise SystemExit(f"Requires pandas+matplotlib: {e}") from e


RUN_RE = re.compile(r"^(?P<plast>E2|P1|P2)_(?P<init>anti_maternal|random_uniform|pro_maternal)_seed(?P<seed>\d+)$")
INIT_ORDER = ["anti_maternal", "random_uniform", "pro_maternal"]
PLAST_ORDER = ["E2", "P1", "P2"]
INIT_COLORS = {"anti_maternal": "#c0392b", "random_uniform": "#2980b9", "pro_maternal": "#27ae60"}
ANTI_PRO_ORDER = ["anti_maternal", "pro_maternal"]
ANTI_PRO_COLORS = {"anti_maternal": "#c0392b", "pro_maternal": "#27ae60"}


def _flatten_genome(g: dict) -> dict[str, float]:
    out: dict[str, float] = {}
    for cat, inner in g.items():
        if not isinstance(inner, dict):
            continue
        for k, v in inner.items():
            out[f"{cat}.{k}"] = float(v)
    return out


def _load_json(path: str) -> dict:
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)


def collect(root: str) -> pd.DataFrame:
    root = os.path.abspath(root)
    paths = sorted(glob(os.path.join(root, "*", "final_genome.json")))
    if not paths:
        raise SystemExit(f"No final_genome.json under {root}/*/")

    rows: list[dict] = []
    skipped = 0
    for p in paths:
        run_dir = os.path.basename(os.path.dirname(p))
        m = RUN_RE.match(run_dir)
        if not m:
            skipped += 1
            continue
        plast = m.group("plast")
        init = m.group("init")
        seed = int(m.group("seed"))
        try:
            g = _load_json(p)
        except Exception:
            skipped += 1
            continue
        flat = _flatten_genome(g)
        if not flat:
            skipped += 1
            continue
        for key, val in flat.items():
            rows.append(
                {
                    "plasticity": plast,
                    "init": init,
                    "evolve_seed": seed,
                    "weight_key": key,
                    "value": float(val),
                    "path": p,
                }
            )

    df = pd.DataFrame(rows)
    if df.empty:
        raise SystemExit(f"No parseable genomes under {root} (skipped={skipped}).")

    df["plasticity"] = pd.Categorical(df["plasticity"], categories=PLAST_ORDER, ordered=True)
    df["init"] = pd.Categorical(df["init"], categories=INIT_ORDER, ordered=True)
    df = df.sort_values(["plasticity", "init", "evolve_seed", "weight_key"]).reset_index(drop=True)
    print(f"[ok] {root}: {df['path'].nunique()} genomes, {df['weight_key'].nunique()} weights (skipped={skipped})")
    return df


def summarize(long_df: pd.DataFrame, *, collapse_plasticity: bool) -> pd.DataFrame:
    keys = ["init", "weight_key"] if collapse_plasticity else ["plasticity", "init", "weight_key"]
    g = long_df.groupby(keys, as_index=False)
    out = g.agg(
        n=("value", "count"),
        mean=("value", "mean"),
        std=("value", "std"),
    )
    out["std"] = out["std"].fillna(0.0)
    return out


def _weight_order(keys: list[str]) -> list[str]:
    # Prefer stable, interpretable ordering (matches MotherAgent motivation schema).
    preferred = [
        "forage.child_hunger",
        "forage.energy_deficit",
        "forage.low_fear",
        "care.child_warmth",
        "care.closeness_deficit",
        "care.bonding",
        "self.fatigue",
        "self.fear",
        "self.stress",
        "protect.child_injury",
        "protect.fear",
        "protect.closeness_deficit",
        "protect.bonding",
    ]
    s = set(keys)
    out = [k for k in preferred if k in s]
    out.extend([k for k in keys if k not in set(out)])
    return out


def plot(summary_df: pd.DataFrame, *, out_png: str, collapse_plasticity: bool) -> None:
    weight_keys = _weight_order(sorted(summary_df["weight_key"].unique().tolist()))

    if collapse_plasticity:
        fig, ax = plt.subplots(1, 1, figsize=(16, 5.8), sharey=True)
        axes = [ax]
        plast_levels = ["all"]
    else:
        fig, axes = plt.subplots(1, 3, figsize=(18, 6.0), sharey=True)
        plast_levels = PLAST_ORDER

    xs = np.arange(len(weight_keys), dtype=float)
    offsets = {"anti_maternal": -0.22, "random_uniform": 0.0, "pro_maternal": 0.22}

    for ax, plast in zip(axes, plast_levels):
        sub = summary_df if collapse_plasticity else summary_df[summary_df["plasticity"] == plast]
        for init in INIT_ORDER:
            s2 = sub[sub["init"] == init].set_index("weight_key")
            means = np.array([float(s2.loc[k, "mean"]) if k in s2.index else np.nan for k in weight_keys], dtype=float)
            stds = np.array([float(s2.loc[k, "std"]) if k in s2.index else 0.0 for k in weight_keys], dtype=float)
            ax.errorbar(
                xs + offsets[init],
                means,
                yerr=stds,
                fmt="o",
                markersize=4.5,
                linewidth=1.2,
                capsize=2.5,
                color=INIT_COLORS[init],
                label=init.replace("_", " "),
            )
        ax.set_xticks(xs)
        ax.set_xticklabels([k.replace(".", "\n") for k in weight_keys], fontsize=8)
        ax.set_ylim(-0.05, 1.05)
        ax.set_ylabel("weight value (mean ± std)")
        title = "all plasticities" if collapse_plasticity else f"{plast}"
        ax.set_title(title)
        ax.grid(True, axis="y", color="0.9")

    handles, labels = axes[0].get_legend_handles_labels()
    fig.legend(handles, labels, loc="upper center", ncol=3, frameon=False, fontsize=10)
    fig.suptitle("Evolved motivation weights by init group", y=0.98, fontsize=14)
    fig.tight_layout(rect=(0, 0, 1, 0.92))
    fig.savefig(out_png, dpi=200, bbox_inches="tight")
    plt.close(fig)


def plot_heatmaps(summary_df: pd.DataFrame, *, out_dir: str) -> None:
    """
    Easier-to-interpret plots than the errorbar chart.
    Produces mean and std heatmaps (rows=weights, cols=init), one panel per plasticity.
    Also produces a delta heatmap for (pro - anti).
    """
    weight_keys_all = _weight_order(sorted(summary_df["weight_key"].unique().tolist()))

    def _matrix(value_col: str, plast: str, weight_keys: list[str]) -> np.ndarray:
        sub = summary_df[summary_df["plasticity"] == plast]
        nrows = len(weight_keys)
        mat = np.full((nrows, len(INIT_ORDER)), np.nan, dtype=float)
        for r, wk in enumerate(weight_keys):
            for c, init in enumerate(INIT_ORDER):
                s = sub[(sub["weight_key"] == wk) & (sub["init"] == init)]
                if not s.empty:
                    mat[r, c] = float(s[value_col].iloc[0])
        return mat

    def _heatmap_panels(
        mats: list[np.ndarray],
        *,
        titles: list[str],
        out_png: str,
        cmap: str,
        vmin: float | None,
        vmax: float | None,
        cbar_label: str,
        suptitle: str,
        weight_keys: list[str],
    ) -> None:
        nrows = len(weight_keys)
        fig_h = max(3.6, 0.55 * nrows)
        fig = plt.figure(figsize=(11.4, fig_h))
        gs = fig.add_gridspec(1, 4, width_ratios=[1, 1, 1, 0.06], wspace=0.25)

        axes = [fig.add_subplot(gs[0, 0]), fig.add_subplot(gs[0, 1], sharey=None), fig.add_subplot(gs[0, 2], sharey=None)]
        cax = fig.add_subplot(gs[0, 3])

        im = None
        for ax, mat, t in zip(axes, mats, titles):
            im = ax.imshow(mat, aspect="auto", interpolation="nearest", cmap=cmap, vmin=vmin, vmax=vmax)
            ax.set_title(t)
            ax.set_xticks(range(len(INIT_ORDER)))
            ax.set_xticklabels([i.replace("_", " ") for i in INIT_ORDER], rotation=20, ha="right")
            ax.set_yticks(range(nrows))
            ax.set_yticklabels(weight_keys, fontsize=9)

            ax.set_xticks(np.arange(-0.5, len(INIT_ORDER), 1), minor=True)
            ax.set_yticks(np.arange(-0.5, nrows, 1), minor=True)
            ax.grid(which="minor", color="white", linewidth=0.8)
            ax.tick_params(which="minor", bottom=False, left=False)

        # Only show y tick labels on the leftmost panel (avoid clutter/overlap)
        for ax in axes[1:]:
            ax.tick_params(axis="y", labelleft=False)

        if im is not None:
            cbar = fig.colorbar(im, cax=cax)
            cbar.set_label(cbar_label)

        fig.suptitle(suptitle, y=0.995, fontsize=13)
        fig.subplots_adjust(top=0.90, left=0.16, right=0.98, bottom=0.08)
        fig.savefig(out_png, dpi=220, bbox_inches="tight")
        plt.close(fig)

    # Split into one plot per motivation for readability.
    groups: list[tuple[str, list[str]]] = []
    for mot in ("forage", "care", "self", "protect"):
        keys = [k for k in weight_keys_all if k.startswith(f"{mot}.")]
        if keys:
            groups.append((mot, keys))

    # Also include an "all weights" group (still useful sometimes)
    groups.append(("all", weight_keys_all))

    for name, weight_keys in groups:
        mats_mean = [_matrix("mean", p, weight_keys) for p in PLAST_ORDER]
        _heatmap_panels(
            mats_mean,
            titles=PLAST_ORDER,
            out_png=os.path.join(out_dir, f"genome_weights_heatmap_mean_{name}.png"),
            cmap="viridis",
            vmin=0.0,
            vmax=1.0,
            cbar_label="mean",
            suptitle=f"Mean evolved weights — {name} (rows=weight, cols=init)",
            weight_keys=weight_keys,
        )

        mats_std = [_matrix("std", p, weight_keys) for p in PLAST_ORDER]
        std_max = float(np.nanmax(np.asarray(mats_std))) if mats_std else 1.0
        _heatmap_panels(
            mats_std,
            titles=PLAST_ORDER,
            out_png=os.path.join(out_dir, f"genome_weights_heatmap_std_{name}.png"),
            cmap="magma",
            vmin=0.0,
            vmax=std_max if np.isfinite(std_max) and std_max > 0 else None,
            cbar_label="std",
            suptitle=f"Std-dev of evolved weights — {name} (rows=weight, cols=init)",
            weight_keys=weight_keys,
        )

        # Delta vs anti baseline: each column = (mean init) − (mean anti).
        # anti column is always 0; random/pro show how much each group differs from anti.
        def _matrix_delta_vs_anti(plast: str, weight_keys: list[str]) -> np.ndarray:
            sub = summary_df[summary_df["plasticity"] == plast]
            nrows = len(weight_keys)
            mat = np.full((nrows, len(INIT_ORDER)), np.nan, dtype=float)
            for r, wk in enumerate(weight_keys):
                anti = sub[(sub["weight_key"] == wk) & (sub["init"] == "anti_maternal")]
                if anti.empty:
                    continue
                anti_mean = float(anti["mean"].iloc[0])
                mat[r, 0] = 0.0
                for c, init in enumerate(INIT_ORDER[1:], start=1):
                    s = sub[(sub["weight_key"] == wk) & (sub["init"] == init)]
                    if not s.empty:
                        mat[r, c] = float(s["mean"].iloc[0] - anti_mean)
            return mat

        mats_delta = [_matrix_delta_vs_anti(p, weight_keys) for p in PLAST_ORDER]
        _heatmap_panels(
            mats_delta,
            titles=PLAST_ORDER,
            out_png=os.path.join(out_dir, f"genome_weights_heatmap_delta_pro_minus_anti_{name}.png"),
            cmap="coolwarm",
            vmin=-1.0,
            vmax=1.0,
            cbar_label="Δ mean vs anti",
            suptitle=(
                f"Mean weight minus anti baseline — {name}\n"
                "(anti=0; random/pro = that init − anti)"
            ),
            weight_keys=weight_keys,
        )


def plot_anti_vs_pro_heatmaps(summary_df: pd.DataFrame, *, out_dir: str) -> None:
    """
    Heatmaps with only two columns: anti_maternal and pro_maternal.
    Easier to interpret than the 3-column versions.
    """
    weight_keys_all = _weight_order(sorted(summary_df["weight_key"].unique().tolist()))

    def _matrix(value_col: str, plast: str, weight_keys: list[str]) -> np.ndarray:
        sub = summary_df[summary_df["plasticity"] == plast]
        mat = np.full((len(weight_keys), len(ANTI_PRO_ORDER)), np.nan, dtype=float)
        for r, wk in enumerate(weight_keys):
            for c, init in enumerate(ANTI_PRO_ORDER):
                s = sub[(sub["weight_key"] == wk) & (sub["init"] == init)]
                if not s.empty:
                    mat[r, c] = float(s[value_col].iloc[0])
        return mat

    def _vector_delta_pro_minus_anti(plast: str, weight_keys: list[str]) -> np.ndarray:
        sub = summary_df[summary_df["plasticity"] == plast]
        # single column: delta = pro - anti
        mat = np.full((len(weight_keys), 1), np.nan, dtype=float)
        for r, wk in enumerate(weight_keys):
            anti = sub[(sub["weight_key"] == wk) & (sub["init"] == "anti_maternal")]
            pro = sub[(sub["weight_key"] == wk) & (sub["init"] == "pro_maternal")]
            if anti.empty or pro.empty:
                continue
            anti_mean = float(anti["mean"].iloc[0])
            pro_mean = float(pro["mean"].iloc[0])
            mat[r, 0] = pro_mean - anti_mean
        return mat

    def _heatmap_panels(mats: list[np.ndarray], *, titles: list[str], out_png: str, cmap: str, vmin: float | None, vmax: float | None, cbar_label: str, suptitle: str, weight_keys: list[str], x_labels: list[str]) -> None:
        nrows = len(weight_keys)
        fig_h = max(3.2, 0.55 * nrows)
        fig = plt.figure(figsize=(9.2, fig_h))
        gs = fig.add_gridspec(1, 4, width_ratios=[1, 1, 1, 0.06], wspace=0.25)
        axes = [fig.add_subplot(gs[0, 0]), fig.add_subplot(gs[0, 1], sharey=None), fig.add_subplot(gs[0, 2], sharey=None)]
        cax = fig.add_subplot(gs[0, 3])

        im = None
        for ax, mat, t in zip(axes, mats, titles):
            im = ax.imshow(mat, aspect="auto", interpolation="nearest", cmap=cmap, vmin=vmin, vmax=vmax)
            ax.set_title(t)
            ax.set_xticks(range(len(x_labels)))
            ax.set_xticklabels(x_labels, rotation=20, ha="right")
            ax.set_yticks(range(nrows))
            ax.set_yticklabels(weight_keys, fontsize=9)

            ax.set_xticks(np.arange(-0.5, len(x_labels), 1), minor=True)
            ax.set_yticks(np.arange(-0.5, nrows, 1), minor=True)
            ax.grid(which="minor", color="white", linewidth=0.8)
            ax.tick_params(which="minor", bottom=False, left=False)

        for ax in axes[1:]:
            ax.tick_params(axis="y", labelleft=False)

        if im is not None:
            cbar = fig.colorbar(im, cax=cax)
            cbar.set_label(cbar_label)

        fig.suptitle(suptitle, y=0.995, fontsize=13)
        fig.subplots_adjust(top=0.90, left=0.18, right=0.98, bottom=0.08)
        fig.savefig(out_png, dpi=220, bbox_inches="tight")
        plt.close(fig)

    groups: list[tuple[str, list[str]]] = []
    for mot in ("forage", "care", "self", "protect"):
        keys = [k for k in weight_keys_all if k.startswith(f"{mot}.")]
        if keys:
            groups.append((mot, keys))
    for name, weight_keys in groups:
        mats_mean = [_matrix("mean", p, weight_keys) for p in PLAST_ORDER]
        _heatmap_panels(
            mats_mean,
            titles=PLAST_ORDER,
            out_png=os.path.join(out_dir, f"genome_weights_heatmap_mean_anti_vs_pro_{name}.png"),
            cmap="viridis",
            vmin=0.0,
            vmax=1.0,
            cbar_label="mean",
            suptitle=f"Mean evolved weights — {name} (rows=weight, cols=anti/pro)",
            weight_keys=weight_keys,
            x_labels=["anti maternal", "pro maternal"],
        )

        mats_std = [_matrix("std", p, weight_keys) for p in PLAST_ORDER]
        std_max = float(np.nanmax(np.asarray(mats_std))) if mats_std else 1.0
        _heatmap_panels(
            mats_std,
            titles=PLAST_ORDER,
            out_png=os.path.join(out_dir, f"genome_weights_heatmap_std_anti_vs_pro_{name}.png"),
            cmap="magma",
            vmin=0.0,
            vmax=std_max if np.isfinite(std_max) and std_max > 0 else None,
            cbar_label="std",
            suptitle=f"Std-dev of evolved weights — {name} (rows=weight, cols=anti/pro)",
            weight_keys=weight_keys,
            x_labels=["anti maternal", "pro maternal"],
        )

        mats_delta = [_vector_delta_pro_minus_anti(p, weight_keys) for p in PLAST_ORDER]
        _heatmap_panels(
            mats_delta,
            titles=PLAST_ORDER,
            out_png=os.path.join(out_dir, f"genome_weights_heatmap_delta_pro_minus_anti_singlecol_{name}.png"),
            cmap="coolwarm",
            vmin=-1.0,
            vmax=1.0,
            cbar_label="Δ mean (pro − anti)",
            suptitle=f"Delta mean weights (pro − anti) — {name} (single column)",
            weight_keys=weight_keys,
            x_labels=["pro − anti"],
        )


def main(argv: list[str]) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--root", required=True, help="Directory containing <tag>/final_genome.json (tag like E2_random_uniform_seed42).")
    ap.add_argument("--out-dir", required=True, help="Output directory for PNGs/CSVs.")
    ap.add_argument("--collapse-plasticity", action="store_true", help="Aggregate E2/P1/P2 together into one panel.")
    ap.add_argument("--anti-vs-pro-only", action="store_true", help="Generate only anti vs pro heatmaps (drop random).")
    args = ap.parse_args(argv)

    os.makedirs(args.out_dir, exist_ok=True)

    long_df = collect(args.root)
    summary_df = summarize(long_df, collapse_plasticity=bool(args.collapse_plasticity))

    long_path = os.path.join(args.out_dir, "genome_weights_long.csv")
    sum_path = os.path.join(args.out_dir, "genome_weights_summary.csv")
    long_df.to_csv(long_path, index=False)
    summary_df.to_csv(sum_path, index=False)

    out_png = os.path.join(args.out_dir, "genome_weights_mean_std.png")
    plot(summary_df, out_png=out_png, collapse_plasticity=bool(args.collapse_plasticity))

    if not args.collapse_plasticity:
        if args.anti_vs_pro_only:
            plot_anti_vs_pro_heatmaps(summary_df, out_dir=args.out_dir)
        else:
            plot_heatmaps(summary_df, out_dir=args.out_dir)

    print(f"[ok] wrote:\n  - {long_path}\n  - {sum_path}\n  - {out_png}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))

