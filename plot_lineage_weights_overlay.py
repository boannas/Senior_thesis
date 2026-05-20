#!/usr/bin/env python3
"""
plot_lineage_weights_overlay.py
================================
Overlay motivation weight trajectories (u_fixed genome) across emergence conditions:
same **seed** × **init** (anti_maternal / random_uniform / pro_maternal) × **plasticity** (E2 / P1 / P2).

Reads ``lineage_generations.csv`` from each run (dense per-generation ``u_forage_*``, ... columns),
not sparse checkpoints.

Outputs (per-seed mode, default):
  - lineage_weights_overlay_seed{N}_raw.png   — all conditions overlaid (13 subplots, one per weight)
  - lineage_weights_overlay_seed{N}_ma_wW.png — same with centered moving average
  - lineage_weights_l2_seed{N}_*.png — L2(||u(g)−u(0)||) (unless --no-l2)

Outputs (--aggregate-seeds): one figure set for all seeds combined
  - lineage_weights_overlay_aggregate_raw.png / _ma_wW.png — mean ± band per (plast|init)
  - lineage_weights_l2_aggregate_*.png — mean L2 ± band across seeds

Usage:
  python3 plot_lineage_weights_overlay.py --root Emergence_results/normal --seed 42
  python3 plot_lineage_weights_overlay.py --root Emergence_results/normal --seeds 42 43 44 --ma-window 80
  python3 plot_lineage_weights_overlay.py --root Emergence_results/normal --aggregate-seeds --seeds 42 43 44 45 46
  python3 plot_lineage_weights_overlay.py --root Emergence_results/normal --plasticities E2 P1 --inits anti_maternal random_uniform
"""

from __future__ import annotations

import argparse
import os
import re
import sys
from glob import glob

import numpy as np
import pandas as pd

try:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
except ImportError as e:
    raise SystemExit(f"matplotlib required: {e}") from e

# Same leaf order as run_evolve_lineage / lineage_generations.csv
U_COLUMNS = [
    "u_forage_child_hunger",
    "u_forage_energy_deficit",
    "u_forage_low_fear",
    "u_care_child_warmth",
    "u_care_closeness_deficit",
    "u_care_bonding",
    "u_self_fatigue",
    "u_self_fear",
    "u_self_stress",
    "u_protect_child_injury",
    "u_protect_fear",
    "u_protect_closeness_deficit",
    "u_protect_bonding",
]

RUN_DIR_RE = re.compile(r"^(?P<plast>E2|P1|P2)_(?P<init>.+)_seed(?P<seed>\d+)$")

# Visual encoding: color = plasticity, linestyle = init
PLAST_COLOR = {"E2": "#1f77b4", "P1": "#d62728", "P2": "#2ca02c"}
INIT_STYLE = {
    "anti_maternal": "-",
    "random_uniform": "--",
    "pro_maternal": ":",
}


def _rolling_centered(y: np.ndarray, window: int) -> np.ndarray:
    if window <= 1 or len(y) == 0:
        return y.astype(float)
    w = min(window, len(y))
    if w % 2 == 0:
        w += 1
    half = w // 2
    out = np.empty(len(y), dtype=float)
    for i in range(len(y)):
        lo, hi = max(0, i - half), min(len(y), i + half + 1)
        out[i] = float(np.nanmean(y[lo:hi]))
    return out


def discover_runs(root: str) -> dict[tuple[str, str, int], str]:
    """Map (plasticity, init, seed) -> csv path."""
    root = os.path.abspath(root)
    pattern = os.path.join(root, "**", "lineage_generations.csv")
    out: dict[tuple[str, str, int], str] = {}
    for csv_path in glob(pattern, recursive=True):
        parent = os.path.basename(os.path.dirname(csv_path))
        m = RUN_DIR_RE.match(parent)
        if not m:
            continue
        key = (m.group("plast"), m.group("init"), int(m.group("seed")))
        out[key] = csv_path
    return out


def load_u_trajectory(csv_path: str, max_gen: int | None) -> pd.DataFrame | None:
    try:
        df = pd.read_csv(csv_path)
    except Exception:
        return None
    if "generation" not in df.columns:
        return None
    missing = [c for c in U_COLUMNS if c not in df.columns]
    if missing:
        return None
    df = df.sort_values("generation").reset_index(drop=True)
    if max_gen is not None:
        df = df[df["generation"] <= int(max_gen)].copy()
    return df


def l2_from_t0(mat: np.ndarray) -> np.ndarray:
    """mat shape (n_gen, n_leaf); return L2 norm vs row 0."""
    ref = mat[0]
    out = np.zeros(len(mat))
    for i in range(len(mat)):
        out[i] = float(np.linalg.norm(mat[i] - ref))
    return out


def build_label(plast: str, init: str) -> str:
    return f"{plast} | {init}"


def _band_sem(stack: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """stack shape (n_seed, n_gen); return mean, lo, hi for mean ± SEM."""
    mean = np.nanmean(stack, axis=0)
    n = np.sum(np.isfinite(stack), axis=0)
    n = np.maximum(n, 1)
    std = np.nanstd(stack, axis=0, ddof=1)
    sem = np.where(n > 1, std / np.sqrt(n), 0.0)
    return mean, mean - sem, mean + sem


def _band_iqr(stack: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    mean = np.nanmean(stack, axis=0)
    lo = np.nanpercentile(stack, 25, axis=0)
    hi = np.nanpercentile(stack, 75, axis=0)
    return mean, lo, hi


def _collect_condition_trajectories(
    plast: str,
    init: str,
    seeds: list[int],
    discovered: dict,
    max_gen: int | None,
) -> tuple[list[dict[str, np.ndarray]], int] | tuple[None, int]:
    """Load all seeds for one (plast, init). Each dict maps U_COLUMNS -> 1d array. Returns (list, n_ok)."""
    per_seed: list[dict[str, np.ndarray]] = []
    for seed in seeds:
        csv_path = discovered.get((plast, init, seed))
        if csv_path is None:
            continue
        df = load_u_trajectory(csv_path, max_gen)
        if df is None:
            continue
        cols_data = {c: pd.to_numeric(df[c], errors="coerce").to_numpy(dtype=float) for c in U_COLUMNS}
        cols_data["_generation"] = df["generation"].to_numpy(dtype=float)
        per_seed.append(cols_data)
    if not per_seed:
        return None, 0
    return per_seed, len(per_seed)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--root", required=True, help="Parent folder containing run subdirs (e.g. Emergence_results/normal)")
    ap.add_argument("--seed", type=int, default=None, help="Single seed to plot (default: first of --seeds)")
    ap.add_argument("--seeds", type=int, nargs="*", default=None, help="Multiple seeds → one figure set per seed")
    ap.add_argument(
        "--inits",
        nargs="*",
        default=["anti_maternal", "random_uniform", "pro_maternal"],
        help="Init modes to include",
    )
    ap.add_argument("--plasticities", nargs="*", default=["E2", "P1", "P2"], help="E2 / P1 / P2")
    ap.add_argument("--ma-window", type=int, default=50, help="Moving average window (generations); centered")
    ap.add_argument("--max-gen", type=int, default=None, help="Clip generations to 0..max_gen")
    ap.add_argument("--out-dir", default=None, help="Output directory (default: <root>/weight_overlay_figures)")
    ap.add_argument("--dpi", type=int, default=150)
    ap.add_argument("--no-l2", action="store_true", help="Skip L2 convergence plot")
    ap.add_argument(
        "--aggregate-seeds",
        action="store_true",
        help="Average across all --seeds (mean ± band per plast|init); writes *_aggregate_*.png instead of per-seed files.",
    )
    ap.add_argument(
        "--band",
        choices=("sem", "iqr"),
        default="sem",
        help="Uncertainty band across seeds: SEM (mean ± SEM) or IQR (25–75%%), default sem.",
    )
    args = ap.parse_args()

    root = os.path.abspath(args.root)
    if not os.path.isdir(root):
        print(f"[error] not a directory: {root}", file=sys.stderr)
        return 2

    if args.seeds:
        seeds = list(dict.fromkeys(args.seeds))
    elif args.seed is not None:
        seeds = [args.seed]
    else:
        seeds = [42]

    discovered = discover_runs(root)
    if not discovered:
        print(f"[error] no matching runs under {root}", file=sys.stderr)
        return 2

    out_dir = os.path.abspath(args.out_dir) if args.out_dir else os.path.join(root, "weight_overlay_figures")
    os.makedirs(out_dir, exist_ok=True)

    ma_w = max(1, int(args.ma_window))
    band_fn = _band_sem if args.band == "sem" else _band_iqr
    band_name = "± SEM" if args.band == "sem" else "25–75% IQR"

    if args.aggregate_seeds:
        # --- Aggregate mode: stack seeds per (plast, init), align to global min length ---
        stacks: dict[str, np.ndarray] = {}  # label -> (n_seed, n_gen, n_leaf)
        gens_plot: np.ndarray | None = None
        n_seeds_report: dict[str, int] = {}

        global_min = None
        raw_blocks: dict[str, list[np.ndarray]] = {}

        for plast in args.plasticities:
            for init in args.inits:
                label = build_label(plast, init)
                per_seed, n_ok = _collect_condition_trajectories(
                    plast, init, seeds, discovered, args.max_gen
                )
                if not per_seed or n_ok == 0:
                    print(f"[warn] aggregate: no runs for {label} (seeds {seeds})")
                    continue
                min_len = min(len(s[U_COLUMNS[0]]) for s in per_seed)
                mats = []
                for s in per_seed:
                    g = s["_generation"][:min_len]
                    if gens_plot is None:
                        gens_plot = np.asarray(g, dtype=float)
                    mat = np.column_stack([s[c][:min_len] for c in U_COLUMNS])
                    mats.append(mat)
                block = np.stack(mats, axis=0)  # (n_seed, min_len, n_leaf)
                raw_blocks[label] = block
                n_seeds_report[label] = n_ok
                if global_min is None:
                    global_min = block.shape[1]
                else:
                    global_min = min(global_min, block.shape[1])

        if global_min is None or not raw_blocks:
            print("[error] aggregate: no data for any condition", file=sys.stderr)
            return 2

        gens_plot = gens_plot[:global_min] if gens_plot is not None else np.arange(global_min)

        for label, block in raw_blocks.items():
            stacks[label] = block[:, :global_min, :]

        print(f"[info] aggregate: aligned to {global_min} generations; band = {band_name}")
        for label in sorted(stacks.keys()):
            print(f"       {label}: n_seeds={n_seeds_report.get(label, stacks[label].shape[0])}")

        for use_ma, suffix in ((False, "raw"), (True, f"ma_w{ma_w}")):
            ncols = 4
            nrows = int(np.ceil(len(U_COLUMNS) / ncols))
            fig, axes = plt.subplots(nrows, ncols, figsize=(4 * ncols, 2.8 * nrows), sharex=True, squeeze=False)
            axes_flat = axes.flatten()

            for idx, col in enumerate(U_COLUMNS):
                ax = axes_flat[idx]
                short = col.replace("u_", "").replace("_", " ")
                j = U_COLUMNS.index(col)
                for label in sorted(stacks.keys()):
                    block = stacks[label]  # (n_s, G, L)
                    y_stack = block[:, :, j].copy()
                    if use_ma:
                        for si in range(y_stack.shape[0]):
                            y_stack[si] = _rolling_centered(y_stack[si], ma_w)
                    mean, lo, hi = band_fn(y_stack)
                    parts = label.split(" | ")
                    plast = parts[0] if len(parts) > 1 else label
                    init_s = parts[1] if len(parts) > 1 else ""
                    color = PLAST_COLOR.get(plast, "gray")
                    ls = INIT_STYLE.get(init_s, "-")
                    ax.fill_between(gens_plot, lo, hi, color=color, alpha=0.22, linewidth=0)
                    ax.plot(gens_plot, mean, color=color, linestyle=ls, label=label, linewidth=2.0)
                ax.set_title(short[:44], fontsize=8)
                ax.set_ylim(-0.05, 1.05)
                ax.grid(True, alpha=0.25)
                if idx >= len(U_COLUMNS) - ncols:
                    ax.set_xlabel("Generation")

            for j in range(len(U_COLUMNS), len(axes_flat)):
                axes_flat[j].set_visible(False)

            handles, labels_leg = axes_flat[0].get_legend_handles_labels()
            fig.legend(
                handles,
                labels_leg,
                loc="upper center",
                ncol=min(3, len(labels_leg)),
                fontsize=6.5,
                bbox_to_anchor=(0.5, 1.03),
            )
            ma_title = f"moving avg w={ma_w}" if use_ma else "raw"
            seed_tag = "_".join(str(s) for s in seeds)
            fig.suptitle(
                f"Motivation weights U — mean across seeds [{seed_tag}] ({ma_title}; {band_name})",
                fontsize=10,
                y=1.06,
            )
            fig.tight_layout()
            outp = os.path.join(out_dir, f"lineage_weights_overlay_aggregate_{suffix}.png")
            fig.savefig(outp, dpi=args.dpi, bbox_inches="tight")
            plt.close(fig)
            print(f"[ok] {outp}")

        if not args.no_l2:
            for use_ma, tag in ((False, "raw"), (True, f"ma_w{ma_w}")):
                fig, ax = plt.subplots(figsize=(9, 5))
                for label in sorted(stacks.keys()):
                    block = stacks[label]
                    n_s, G, L = block.shape
                    l2_stack = np.zeros((n_s, G))
                    for si in range(n_s):
                        mat = block[si].copy()
                        if use_ma:
                            for jj in range(L):
                                mat[:, jj] = _rolling_centered(mat[:, jj], ma_w)
                        l2_stack[si] = l2_from_t0(mat)
                    mean, lo, hi = band_fn(l2_stack)
                    parts = label.split(" | ")
                    plast = parts[0]
                    init_s = parts[1] if len(parts) > 1 else ""
                    color = PLAST_COLOR.get(plast, "gray")
                    ls = INIT_STYLE.get(init_s, "-")
                    ax.fill_between(gens_plot, lo, hi, color=color, alpha=0.22, linewidth=0)
                    ax.plot(gens_plot, mean, color=color, linestyle=ls, label=label, linewidth=2.0)
                ax.set_xlabel("Generation")
                ax.set_ylabel(r"L2(||u(g) − u(0)||)")
                seed_tag = "_".join(str(s) for s in seeds)
                ax.set_title(
                    f"L2 drift from gen-0 weights — mean across seeds [{seed_tag}] ({'MA' if use_ma else 'raw'}; {band_name})"
                )
                ax.grid(True, alpha=0.3)
                ax.legend(loc="best", fontsize=7, ncol=2)
                fig.tight_layout()
                outp = os.path.join(out_dir, f"lineage_weights_l2_aggregate_{tag}.png")
                fig.savefig(outp, dpi=args.dpi, bbox_inches="tight")
                plt.close(fig)
                print(f"[ok] {outp}")

        print(f"[info] legend: color = E2 / P1 / P2; linestyle = anti / random / pro")
        return 0

    for seed in seeds:
        series_raw: dict[str, dict[str, np.ndarray]] = {}  # label -> column -> array

        for plast in args.plasticities:
            for init in args.inits:
                key = (plast, init, seed)
                csv_path = discovered.get(key)
                label = build_label(plast, init)
                if csv_path is None:
                    print(f"[warn] missing run: {label} seed{seed}")
                    continue
                df = load_u_trajectory(csv_path, args.max_gen)
                if df is None:
                    print(f"[warn] could not load u_* columns: {csv_path}")
                    continue

                cols_data = {}
                for c in U_COLUMNS:
                    cols_data[c] = pd.to_numeric(df[c], errors="coerce").to_numpy(dtype=float)
                cols_data["_generation"] = df["generation"].to_numpy()
                series_raw[label] = cols_data

        if not series_raw:
            print(f"[error] no series for seed {seed}", file=sys.stderr)
            continue

        # Align all to shortest length; x-axis = generations from shortest run
        lengths_pre = {lb: len(series_raw[lb][U_COLUMNS[0]]) for lb in series_raw}
        min_len = min(lengths_pre.values())
        shortest_lb = min(series_raw.keys(), key=lambda lb: len(series_raw[lb][U_COLUMNS[0]]))
        gens_plot = np.asarray(series_raw[shortest_lb]["_generation"][:min_len], dtype=float)
        for lb in series_raw:
            del series_raw[lb]["_generation"]
            for c in U_COLUMNS:
                series_raw[lb][c] = series_raw[lb][c][:min_len]
        if max(lengths_pre.values()) > min_len:
            print(
                f"[warn] aligned all curves to {min_len} generations "
                f"(shortest run: {shortest_lb})"
            )

        for use_ma, suffix in ((False, "raw"), (True, f"ma_w{ma_w}")):
            panels: dict[str, dict[str, np.ndarray]] = {c: {} for c in U_COLUMNS}

            for label, colmap in series_raw.items():
                mat = np.column_stack([colmap[c][:min_len] for c in U_COLUMNS])
                if use_ma:
                    mat = np.column_stack([_rolling_centered(mat[:, j], ma_w) for j in range(len(U_COLUMNS))])
                for j, c in enumerate(U_COLUMNS):
                    panels[c][label] = mat[:, j]

            # 13 panels: custom loop
            ncols = 4
            nrows = int(np.ceil(len(U_COLUMNS) / ncols))
            fig, axes = plt.subplots(nrows, ncols, figsize=(4 * ncols, 2.8 * nrows), sharex=True, squeeze=False)
            axes_flat = axes.flatten()

            for idx, col in enumerate(U_COLUMNS):
                ax = axes_flat[idx]
                short = col.replace("u_", "").replace("_", " ")
                for label in sorted(panels[col].keys()):
                    arr = panels[col][label]
                    # Style by parsing label "E2 | anti_maternal"
                    parts = label.split(" | ")
                    plast = parts[0] if len(parts) > 1 else label
                    init_s = parts[1] if len(parts) > 1 else ""
                    color = PLAST_COLOR.get(plast, "gray")
                    ls = INIT_STYLE.get(init_s, "-")
                    ax.plot(gens_plot, arr, color=color, linestyle=ls, label=label, linewidth=1.6, alpha=0.92)
                ax.set_title(short[:44], fontsize=8)
                ax.set_ylim(-0.05, 1.05)
                ax.grid(True, alpha=0.25)
                if idx >= len(U_COLUMNS) - ncols:
                    ax.set_xlabel("Generation")

            for j in range(len(U_COLUMNS), len(axes_flat)):
                axes_flat[j].set_visible(False)

            handles, labels = axes_flat[0].get_legend_handles_labels()
            fig.legend(handles, labels, loc="upper center", ncol=min(4, len(labels)), fontsize=6.5, bbox_to_anchor=(0.5, 1.03))
            ma_title = f"moving avg w={ma_w}" if use_ma else "raw"
            fig.suptitle(f"Motivation weights U vs generation (seed={seed}) — {ma_title}", fontsize=11, y=1.06)
            fig.tight_layout()
            outp = os.path.join(out_dir, f"lineage_weights_overlay_seed{seed}_{suffix}.png")
            fig.savefig(outp, dpi=args.dpi, bbox_inches="tight")
            plt.close(fig)
            print(f"[ok] {outp}")

        if not args.no_l2:
            for use_ma, tag in ((False, "raw"), (True, f"ma_w{ma_w}")):
                fig, ax = plt.subplots(figsize=(9, 5))
                for label in sorted(series_raw.keys()):
                    colmap = series_raw[label]
                    mat = np.column_stack([colmap[c][:min_len] for c in U_COLUMNS])
                    if use_ma:
                        mat = np.column_stack([_rolling_centered(mat[:, j], ma_w) for j in range(len(U_COLUMNS))])
                    l2 = l2_from_t0(mat)
                    parts = label.split(" | ")
                    plast = parts[0]
                    init_s = parts[1] if len(parts) > 1 else ""
                    ax.plot(
                        gens_plot,
                        l2,
                        color=PLAST_COLOR.get(plast, "gray"),
                        linestyle=INIT_STYLE.get(init_s, "-"),
                        label=label,
                        linewidth=2.0,
                    )
                ax.set_xlabel("Generation")
                ax.set_ylabel(r"L2(||u(g) − u(0)||)")
                ax.set_title(f"Distance from initial genome weights (seed={seed}) — {'MA' if use_ma else 'raw'}")
                ax.grid(True, alpha=0.3)
                ax.legend(loc="best", fontsize=7, ncol=2)
                fig.tight_layout()
                outp = os.path.join(out_dir, f"lineage_weights_l2_seed{seed}_{tag}.png")
                fig.savefig(outp, dpi=args.dpi, bbox_inches="tight")
                plt.close(fig)
                print(f"[ok] {outp}")

    print(f"[info] legend: color = E2(blue) / P1(red) / P2(green); linestyle = anti(solid) / random(dash) / pro(dotted)")
    return 0


if __name__ == "__main__":
    sys.exit(main())