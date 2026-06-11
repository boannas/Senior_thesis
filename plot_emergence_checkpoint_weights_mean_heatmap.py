#!/usr/bin/env python3
"""
plot_emergence_checkpoint_weights_mean_heatmap.py
=================================================
Aggregate checkpoint genomes across many emergence runs (all seeds / inits /
plasticities under a root), take the **mean** of each motivation weight at each
checkpoint generation, and plot one heatmap (weights × generation).

Each run directory is expected to match emergence naming, e.g.:
  E2_anti_maternal_seed42/checkpoints/best_gen0500.json

Uses the same flattening order as ``plot_checkpoint_weights.py`` /
``run_evolve_lineage`` (13 leaves).

Alignment
---------
  --align nanmean (default): union of all checkpoint generations that appear in
    at least one run; for each (gen, leaf) average only over runs that have that
    checkpoint (missing = ignored in the mean).
  --align intersection: only generations present in **every** included run.

Usage
-----
  python3 plot_emergence_checkpoint_weights_mean_heatmap.py \\
      --root Emergence_results/normal \\
      --out-dir Emergence_results/normal/figures_aggregate

  # Restrict to one plasticity or init
  python3 plot_emergence_checkpoint_weights_mean_heatmap.py \\
      --root Emergence_results/normal --plasticity E2 --align intersection
"""

from __future__ import annotations

import argparse
import csv
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
except ImportError as e:
    raise SystemExit(f"matplotlib required: {e}") from e

from plot_checkpoint_weights import CHECKPOINT_RE, GENOME_LEAVES_ORDER, genome_to_flat

LEAF_KEYS = [f"u_{c}_{k}" for c, k in GENOME_LEAVES_ORDER]

RUN_DIR_RE = re.compile(
    r"^(?P<plast>E2|P1|P2)_(?P<init>anti_maternal|random_uniform|pro_maternal)_seed(?P<seed>\d+)$"
)


def _discover_run_dirs(root: str) -> list[str]:
    root = os.path.abspath(root)
    if not os.path.isdir(root):
        raise SystemExit(f"not a directory: {root}")
    out: list[str] = []
    for name in sorted(os.listdir(root)):
        path = os.path.join(root, name)
        if not os.path.isdir(path):
            continue
        if RUN_DIR_RE.match(name) and os.path.isdir(os.path.join(path, "checkpoints")):
            out.append(path)
    return out


def _load_run_checkpoint_map(checkpoints_dir: str) -> dict[int, dict[str, float]]:
    """gen -> flat leaf dict for one run."""
    by_gen: dict[int, dict[str, float]] = {}
    for path in sorted(glob(os.path.join(checkpoints_dir, "best_gen*.json"))):
        m = CHECKPOINT_RE.search(os.path.basename(path))
        if not m:
            continue
        gen = int(m.group(1))
        try:
            with open(path, encoding="utf-8") as f:
                g = json.load(f)
        except (OSError, json.JSONDecodeError):
            continue
        by_gen[gen] = genome_to_flat(g)
    return by_gen


def _aggregate(
    run_maps: list[dict[int, dict[str, float]]],
    *,
    align: str,
) -> tuple[list[int], np.ndarray, np.ndarray]:
    """
    Returns gens (sorted), mean_mat (n_leaf, n_gen), count_mat (n_leaf, n_gen)
    count_mat = number of runs contributing to each cell (same for all leaves at fixed gen).
    """
    if not run_maps:
        raise ValueError("no run checkpoint data")

    if align == "intersection":
        common = None
        for m in run_maps:
            gens_set = set(m.keys())
            common = gens_set if common is None else common & gens_set
        if not common:
            raise ValueError("intersection of checkpoint generations is empty")
        gens = sorted(common)
    else:
        all_gens: set[int] = set()
        for m in run_maps:
            all_gens.update(m.keys())
        gens = sorted(all_gens)

    n_leaf = len(LEAF_KEYS)
    n_gen = len(gens)
    sum_mat = np.zeros((n_leaf, n_gen), dtype=float)
    cnt_mat = np.zeros((n_leaf, n_gen), dtype=int)

    for j, gen in enumerate(gens):
        for i, key in enumerate(LEAF_KEYS):
            s = 0.0
            c = 0
            for m in run_maps:
                row = m.get(gen)
                if row is None:
                    continue
                v = row.get(key, float("nan"))
                if v is None or (isinstance(v, float) and np.isnan(v)):
                    continue
                s += float(v)
                c += 1
            if c:
                sum_mat[i, j] = s
                cnt_mat[i, j] = c

    mean_mat = np.divide(sum_mat, np.maximum(cnt_mat, 1), out=np.zeros_like(sum_mat), where=cnt_mat > 0)
    if align == "intersection":
        # every run must have every gen; counts should be uniform
        pass
    return gens, mean_mat, cnt_mat


def _plot_mean_heatmap(
    gens: list[int],
    mean_mat: np.ndarray,
    *,
    out_path: str,
    title: str,
    dpi: int,
    vmin: float,
    vmax: float,
) -> None:
    fig, ax = plt.subplots(figsize=(max(8, len(gens) * 0.35), 8))
    im = ax.imshow(mean_mat, aspect="auto", cmap="RdYlBu_r", vmin=vmin, vmax=vmax)
    ax.set_xticks(range(len(gens)))
    ax.set_xticklabels([str(g) for g in gens], rotation=45, ha="right", fontsize=8)
    short_labels = [k.replace("u_", "").replace("_", "\n") for k in LEAF_KEYS]
    ax.set_yticks(range(len(LEAF_KEYS)))
    ax.set_yticklabels(short_labels, fontsize=7)
    ax.set_xlabel("Checkpoint generation (mean across runs)")
    ax.set_title(title, fontsize=11)
    plt.colorbar(im, ax=ax, fraction=0.02, pad=0.02, label="Mean weight")
    fig.tight_layout()
    fig.savefig(out_path, dpi=dpi)
    plt.close(fig)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--root", required=True, help="Folder containing E2_* / P1_* / P2_* run subdirs")
    ap.add_argument("--out-dir", default=None, help="Output directory (default: <root>/figures_checkpoint_mean)")
    ap.add_argument("--out-png", default="emergence_checkpoint_weights_mean_heatmap.png", help="PNG filename")
    ap.add_argument("--dpi", type=int, default=150)
    ap.add_argument("--vmin", type=float, default=0.0)
    ap.add_argument("--vmax", type=float, default=1.0)
    ap.add_argument("--plasticity", choices=["E2", "P1", "P2"], default=None, help="Include only this plasticity")
    ap.add_argument("--init", choices=["anti_maternal", "random_uniform", "pro_maternal"], default=None)
    ap.add_argument(
        "--align",
        choices=["nanmean", "intersection"],
        default="nanmean",
        help="How to align generations across runs (default: nanmean)",
    )
    ap.add_argument("--title", default="", help="Figure title override")
    args = ap.parse_args()

    run_dirs = _discover_run_dirs(args.root)
    filtered: list[str] = []
    for rd in run_dirs:
        base = os.path.basename(rd)
        m = RUN_DIR_RE.match(base)
        if not m:
            continue
        if args.plasticity and m.group("plast") != args.plasticity:
            continue
        if args.init and m.group("init") != args.init:
            continue
        filtered.append(rd)

    if not filtered:
        print(f"[error] no emergence run dirs with checkpoints under {args.root}", file=sys.stderr)
        return 2

    run_maps: list[dict[int, dict[str, float]]] = []
    skipped = 0
    for rd in filtered:
        ck = os.path.join(rd, "checkpoints")
        m = _load_run_checkpoint_map(ck)
        if not m:
            skipped += 1
            continue
        run_maps.append(m)

    if not run_maps:
        print(f"[error] no checkpoint JSONs found (skipped dirs={skipped})", file=sys.stderr)
        return 2

    try:
        gens, mean_mat, cnt_mat = _aggregate(run_maps, align=args.align)
    except ValueError as e:
        print(f"[error] {e}", file=sys.stderr)
        return 2

    out_dir = os.path.abspath(args.out_dir) if args.out_dir else os.path.join(os.path.abspath(args.root), "figures_checkpoint_mean")
    os.makedirs(out_dir, exist_ok=True)
    out_png = os.path.join(out_dir, args.out_png)

    n_runs = len(run_maps)
    title = args.title or (
        f"Mean motivation weights U across checkpoints (n_runs={n_runs}, align={args.align})\n"
        f"{os.path.basename(os.path.abspath(args.root))}"
        + (f" | {args.plasticity}" if args.plasticity else "")
        + (f" | {args.init}" if args.init else "")
    )

    _plot_mean_heatmap(gens, mean_mat, out_path=out_png, title=title, dpi=args.dpi, vmin=args.vmin, vmax=args.vmax)
    print(f"[ok] {out_png}")
    print(f"[info] runs used: {n_runs}, skipped empty: {skipped}, generations: {len(gens)} ({gens[0]}…{gens[-1]})")
    if args.align == "nanmean" and len(gens):
        c0 = cnt_mat[:, 0]
        if np.any(c0 != c0[0]):
            print(f"[info] per-gen coverage min/max: {int(cnt_mat.min())} / {int(cnt_mat.max())}")

    # Tidy CSV: generation + mean per leaf + n_runs column (same for all leaves at that gen)
    csv_path = os.path.join(out_dir, args.out_png.replace(".png", "_matrix.csv"))
    with open(csv_path, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["generation", "n_runs"] + LEAF_KEYS)
        for j, g in enumerate(gens):
            nr = int(np.min(cnt_mat[:, j])) if cnt_mat.shape[1] else 0
            w.writerow([g, nr] + [float(mean_mat[i, j]) for i in range(len(LEAF_KEYS))])
    print(f"[ok] {csv_path}")

    return 0


if __name__ == "__main__":
    sys.exit(main())
