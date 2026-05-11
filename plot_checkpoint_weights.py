#!/usr/bin/env python3
"""
plot_checkpoint_weights.py
==========================
Visualize how evolved motivation weights U change across checkpoint snapshots.

Reads ``checkpoints/best_genXXXX.json`` files (same nested dict as ``run_evolve_lineage`` /
``MotherAgent.motivation_weights_fixed``). Flattens leaves in the **same order** as
``lineage_generations.csv`` columns ``u_forage_*``, ...

Outputs (default: next to checkpoints/ or --out-dir):
  - checkpoint_weights_heatmap.png   — weights × generation (color = value)
  - checkpoint_weights_lines.png     — one subplot per motivation (Forage/Care/Self/Protect)
  - checkpoint_weights_delta.png     — bar chart: Δ weight from first → last checkpoint
  - checkpoint_weights_matrix.csv    — tidy table for spreadsheets

Usage:
  python3 plot_checkpoint_weights.py Emergence_results/normal/P1_anti_maternal_seed42
  python3 plot_checkpoint_weights.py path/to/run --out-dir figures/P1_anti42
  python3 plot_checkpoint_weights.py path/to/run --heatmap-only
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

# Must match run_evolve_lineage._GENOME_LEAVES_ORDER
GENOME_LEAVES_ORDER: list[tuple[str, str]] = [
    ("forage", "child_hunger"),
    ("forage", "energy_deficit"),
    ("forage", "low_fear"),
    ("care", "child_warmth"),
    ("care", "closeness_deficit"),
    ("care", "bonding"),
    ("self", "fatigue"),
    ("self", "fear"),
    ("self", "stress"),
    ("protect", "child_injury"),
    ("protect", "fear"),
    ("protect", "closeness_deficit"),
    ("protect", "bonding"),
]

CHECKPOINT_RE = re.compile(r"best_gen(\d+)\.json$", re.I)


def genome_to_flat(g: dict) -> dict[str, float]:
    out: dict[str, float] = {}
    for cat, leaf in GENOME_LEAVES_ORDER:
        key = f"u_{cat}_{leaf}"
        try:
            out[key] = float(g[cat][leaf])
        except (KeyError, TypeError, ValueError):
            out[key] = float("nan")
    return out


def collect_checkpoints(checkpoints_dir: str) -> tuple[list[int], dict[str, list[float]], list[str]]:
    """Return sorted generations, matrix of leaf -> values per gen, ordered leaf keys."""
    paths = sorted(glob(os.path.join(checkpoints_dir, "best_gen*.json")))
    gens: list[int] = []
    rows: dict[str, list[float]] = {}
    leaf_keys = [f"u_{c}_{k}" for c, k in GENOME_LEAVES_ORDER]

    for path in paths:
        m = CHECKPOINT_RE.search(os.path.basename(path))
        if not m:
            continue
        gen = int(m.group(1))
        try:
            with open(path, encoding="utf-8") as f:
                g = json.load(f)
        except (OSError, json.JSONDecodeError):
            continue
        flat = genome_to_flat(g)
        gens.append(gen)
        for k in leaf_keys:
            rows.setdefault(k, []).append(flat[k])

    order = sorted(range(len(gens)), key=lambda i: gens[i])
    gens = [gens[i] for i in order]
    for k in leaf_keys:
        if k in rows:
            rows[k] = [rows[k][i] for i in order]

    return gens, rows, leaf_keys


def plot_heatmap(
    gens: list[int],
    rows: dict[str, list[float]],
    leaf_keys: list[str],
    out_path: str,
    dpi: int,
) -> None:
    """Rows = weight leaves, cols = checkpoint generation."""
    if not gens:
        return
    mat = np.array([[rows[k][j] for j in range(len(gens))] for k in leaf_keys], dtype=float)
    fig, ax = plt.subplots(figsize=(max(8, len(gens) * 0.35), 8))
    vmin, vmax = 0.0, 1.0
    im = ax.imshow(mat, aspect="auto", cmap="RdYlBu_r", vmin=vmin, vmax=vmax)
    ax.set_xticks(range(len(gens)))
    ax.set_xticklabels([str(g) for g in gens], rotation=45, ha="right", fontsize=8)
    short_labels = [k.replace("u_", "").replace("_", "\n") for k in leaf_keys]
    ax.set_yticks(range(len(leaf_keys)))
    ax.set_yticklabels(short_labels, fontsize=7)
    ax.set_xlabel("Checkpoint generation")
    ax.set_title("Motivation weights U across checkpoints (gene-evolved fixed weights)")
    plt.colorbar(im, ax=ax, fraction=0.02, pad=0.02, label="Weight")
    fig.tight_layout()
    fig.savefig(out_path, dpi=dpi)
    plt.close(fig)


def plot_lines(
    gens: list[int],
    rows: dict[str, list[float]],
    leaf_keys: list[str],
    out_path: str,
    dpi: int,
) -> None:
    """Four subplots: forage / care / self / protect."""
    by_cat: dict[str, list[tuple[str, str]]] = {}
    for cat, leaf in GENOME_LEAVES_ORDER:
        by_cat.setdefault(cat, []).append((cat, leaf))

    fig, axes = plt.subplots(2, 2, figsize=(11, 8), sharex=True)
    axes_flat = axes.flatten()
    cats = ["forage", "care", "self", "protect"]
    colors = plt.cm.tab10(np.linspace(0, 1, 10))

    for ax, cat in zip(axes_flat, cats):
        pairs = by_cat.get(cat, [])
        for idx, (_c, leaf) in enumerate(pairs):
            key = f"u_{cat}_{leaf}"
            if key not in rows:
                continue
            y = np.array(rows[key], dtype=float)
            ax.plot(gens, y, "o-", label=leaf, color=colors[idx % 10], linewidth=1.8, markersize=4)
        ax.set_ylabel("Weight")
        ax.set_title(cat.capitalize())
        ax.set_ylim(-0.05, 1.05)
        ax.grid(True, alpha=0.3)
        ax.legend(loc="best", fontsize=7, ncol=1)
    axes_flat[-2].set_xlabel("Generation")
    axes_flat[-1].set_xlabel("Generation")
    fig.suptitle("Motivation weight trajectories (checkpoints)", fontsize=12, y=1.02)
    fig.tight_layout()
    fig.savefig(out_path, dpi=dpi)
    plt.close(fig)


def plot_delta(
    gens: list[int],
    rows: dict[str, list[float]],
    leaf_keys: list[str],
    out_path: str,
    dpi: int,
) -> None:
    """Horizontal bars: final - initial for each leaf."""
    if len(gens) < 2:
        return
    deltas = []
    labels = []
    for k in leaf_keys:
        vals = rows[k]
        d = float(vals[-1]) - float(vals[0])
        deltas.append(d)
        labels.append(k.replace("u_", "").replace("_", " / "))

    fig, ax = plt.subplots(figsize=(8, max(5, len(labels) * 0.28)))
    y = np.arange(len(labels))
    colors = ["#c62828" if d < 0 else "#2e7d32" if d > 0 else "#757575" for d in deltas]
    ax.barh(y, deltas, color=colors, alpha=0.85)
    ax.set_yticks(y)
    ax.set_yticklabels(labels, fontsize=8)
    ax.axvline(0, color="black", linewidth=0.8)
    ax.set_xlabel(f"Δ weight (gen {gens[-1]} − gen {gens[0]})")
    ax.set_title("Total weight change from first to last checkpoint")
    fig.tight_layout()
    fig.savefig(out_path, dpi=dpi)
    plt.close(fig)


def write_csv(
    gens: list[int],
    rows: dict[str, list[float]],
    leaf_keys: list[str],
    out_path: str,
) -> None:
    with open(out_path, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["generation"] + leaf_keys)
        for j, g in enumerate(gens):
            w.writerow([g] + [rows[k][j] for k in leaf_keys])


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument(
        "run_dir",
        nargs="?",
        default=".",
        help="Evolve-lineage output dir (contains checkpoints/) or path directly to checkpoints/",
    )
    ap.add_argument("--out-dir", default=None, help="Where to write PNGs and CSV (default: run_dir or run_dir/checkpoints)")
    ap.add_argument("--dpi", type=int, default=150)
    ap.add_argument("--heatmap-only", action="store_true")
    ap.add_argument("--no-csv", action="store_true")
    args = ap.parse_args()

    run_dir = os.path.abspath(args.run_dir)
    ck_dir = run_dir if os.path.basename(run_dir).lower() == "checkpoints" else os.path.join(run_dir, "checkpoints")
    if not os.path.isdir(ck_dir):
        print(f"[error] checkpoints not found: {ck_dir}", file=sys.stderr)
        return 2

    gens, rows, leaf_keys = collect_checkpoints(ck_dir)
    if not gens:
        print(f"[error] no best_gen*.json files in {ck_dir}", file=sys.stderr)
        return 2

    out_dir = os.path.abspath(args.out_dir) if args.out_dir else run_dir
    os.makedirs(out_dir, exist_ok=True)

    base = os.path.join(out_dir, "checkpoint_weights")
    plot_heatmap(gens, rows, leaf_keys, base + "_heatmap.png", args.dpi)
    print(f"[ok] {base}_heatmap.png")

    if not args.heatmap_only:
        plot_lines(gens, rows, leaf_keys, base + "_lines.png", args.dpi)
        print(f"[ok] {base}_lines.png")
        plot_delta(gens, rows, leaf_keys, base + "_delta.png", args.dpi)
        print(f"[ok] {base}_delta.png")

    if not args.no_csv:
        write_csv(gens, rows, leaf_keys, base + "_matrix.csv")
        print(f"[ok] {base}_matrix.csv")

    # Summary stats
    if len(gens) >= 2:
        mat = np.array([[rows[k][j] for j in range(len(gens))] for k in leaf_keys], dtype=float)
        delta_mat = mat[:, -1] - mat[:, 0]
        print(f"[info] checkpoints: gen {gens[0]} → {gens[-1]} ({len(gens)} snapshots)")
        print(f"[info] mean |Δ| per leaf (first→last): {np.nanmean(np.abs(delta_mat)):.4f}")
        print(f"[info] max |Δ| leaf: {leaf_keys[int(np.nanargmax(np.abs(delta_mat)))]}")

    return 0


if __name__ == "__main__":
    sys.exit(main())
