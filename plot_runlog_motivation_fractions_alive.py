#!/usr/bin/env python3
"""
plot_runlog_motivation_fractions_alive.py
=========================================
Plot motivation selection fractions from RunLogger-style headless rollout logs,
conditioning on ticks where BOTH mother and child are alive.

Inputs
------
Directory layout like:
  <root>/<run_dir>/run_<seed>.csv

where run_dir matches emergence naming:
  E2_anti_maternal_seed42
  P1_random_uniform_seed44
  P2_pro_maternal_seed46

Each run_*.csv is produced by `headless_rollout_log.py` and includes:
  - m0_sel_Forage/Care/Self/Protect (one-hot selected motivation)
  - m0_energy (non-finite after mother death)
  - c0_hunger (non-finite after child death)

This script computes, for each run log:
  frac_<Motivation>_alive = mean(m0_sel_<Motivation>) over ticks where
                           isfinite(m0_energy) AND isfinite(c0_hunger)

and then plots box/violin distributions across seeds in an "emergence-style" layout.

Usage
-----
  python3 plot_runlog_motivation_fractions_alive.py --root FinalGenomeRollouts_normal
  python3 plot_runlog_motivation_fractions_alive.py --root FinalGenomeRollouts_normal --layout split_plasticity
"""

from __future__ import annotations

import argparse
import glob
import os
import re
import sys

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
MOTS = ["Forage", "Care", "Self", "Protect"]

COLORS = {"E2": "#4C78A8", "P1": "#E45756", "P2": "#54A24B"}
MOT_COLORS = {
    "Forage": "#4C78A8",
    "Care": "#F28E2B",
    "Self": "#9D9D9D",
    "Protect": "#54A24B",
}
PAIR_COLORS = {"maternal": "#E45756", "base": "#4C78A8"}

RUN_RE = re.compile(
    r"^(?P<plast>E2|P1|P2)_(?P<init>anti_maternal|random_uniform|pro_maternal)_seed(?P<seed>\d+)$"
)


def _alive_mask(df: "pd.DataFrame") -> np.ndarray:
    m = pd.to_numeric(df.get("m0_energy"), errors="coerce").to_numpy(dtype=float)
    c = pd.to_numeric(df.get("c0_hunger"), errors="coerce").to_numpy(dtype=float)
    return np.isfinite(m) & np.isfinite(c)


def _load_one_run(csv_path: str) -> dict | None:
    try:
        df = pd.read_csv(csv_path)
    except Exception:
        return None
    if "m0_energy" not in df.columns or "c0_hunger" not in df.columns:
        return None
    mask = _alive_mask(df)
    n_alive = int(mask.sum())
    if n_alive <= 0:
        return None
    row: dict = {"path": csv_path, "n_alive_ticks": n_alive}
    for mot in MOTS:
        col = f"m0_sel_{mot}"
        if col not in df.columns:
            return None
        s = pd.to_numeric(df[col], errors="coerce").fillna(0.0).to_numpy(dtype=float)
        row[f"frac_{mot}_alive"] = float(np.nanmean(s[mask])) if n_alive else float("nan")
    row["maternal_fraction_alive"] = float(row["frac_Care_alive"] + row["frac_Protect_alive"])
    return row


def _collect(root: str) -> "pd.DataFrame":
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
        rec = _load_one_run(p)
        if rec is None:
            skipped += 1
            continue
        rec["run_dir"] = run_dir
        rec["plasticity"] = m.group("plast")
        rec["init"] = m.group("init")
        rec["evolve_seed"] = int(m.group("seed"))
        rows.append(rec)
    if not rows:
        raise SystemExit(f"No usable run logs. (skipped={skipped})")
    df = pd.DataFrame(rows)
    print(f"[ok] loaded {len(df)} run logs (skipped={skipped})")
    return df


def _panel_box_or_violin(ax, data: list[np.ndarray], labels: list[str], color: str, style: str) -> None:
    style = style.lower()
    if style == "box":
        bp = ax.boxplot(data, labels=labels, showmeans=True, patch_artist=True)
        for patch in bp.get("boxes", []):
            patch.set_facecolor(color)
            patch.set_alpha(0.16)
            patch.set_edgecolor(color)
    else:
        pos = np.arange(1, len(labels) + 1, dtype=float)
        for i, vals in enumerate(data):
            if len(vals) == 0:
                continue
            parts = ax.violinplot([vals], positions=[pos[i]], widths=0.78, showmeans=True, showmedians=True)
            for pc in parts.get("bodies", []):
                pc.set_facecolor(color)
                pc.set_edgecolor(color)
                pc.set_alpha(0.16)
        ax.set_xticks(pos)
        ax.set_xticklabels(labels)

    rng = np.random.default_rng(0)
    for i, vals in enumerate(data):
        if len(vals) == 0:
            continue
        x = rng.normal(i + 1, 0.04, size=len(vals))
        ax.scatter(x, vals, s=9, alpha=0.14, color=color, edgecolors="none", zorder=3)


def _plot(df: "pd.DataFrame", *, value_col: str, out_png: str, title: str, layout: str, style: str) -> None:
    layout = layout.lower()
    style = style.lower()
    if layout not in ("split_init", "split_plasticity"):
        raise ValueError("layout must be split_init|split_plasticity")
    if style not in ("box", "violin"):
        raise ValueError("style must be box|violin")

    fig, axes = plt.subplots(1, 3, figsize=(12.6, 4.0), sharey=True)

    if layout == "split_init":
        panels = INIT_ORDER
        panel_key = "init"
        within_order = PLAST_ORDER
        within_key = "plasticity"
        within_labels = within_order
        def _col_for(panel_val: str, within_val: str) -> str:  # noqa: ARG001
            return COLORS.get(within_val, "0.4")
    else:
        panels = PLAST_ORDER
        panel_key = "plasticity"
        within_order = INIT_ORDER
        within_key = "init"
        within_labels = [x.replace("_", " ") for x in within_order]
        def _col_for(panel_val: str, within_val: str) -> str:  # noqa: ARG001
            return COLORS.get(panel_val, "0.4")

    for ax, panel_val in zip(axes, panels):
        sub = df[df[panel_key] == panel_val]
        data = []
        for w in within_order:
            vals = pd.to_numeric(sub.loc[sub[within_key] == w, value_col], errors="coerce").dropna().to_numpy()
            data.append(vals)
        if not any(len(d) for d in data):
            ax.set_visible(False)
            continue
        color = _col_for(str(panel_val), str(within_order[0]))
        _panel_box_or_violin(ax, data, within_labels, color=color, style=style)
        ax.set_title(str(panel_val).replace("_", " "), fontsize=12)
        ax.grid(True, axis="y", alpha=0.25)
        ax.grid(False, axis="x")

    axes[0].set_ylabel("Fraction of ticks (mother+child alive)")
    fig.suptitle(title, fontsize=12, y=1.02)
    fig.tight_layout()
    fig.savefig(out_png, dpi=200, bbox_inches="tight")
    plt.close(fig)
    print("Wrote", out_png)


def _plot_stacked_summary(df: "pd.DataFrame", *, out_png: str, title: str) -> None:
    """
    One figure that shows ALL motivation fractions at once:
      x-axis: condition (init × plasticity)
      stacked bar segments: Forage/Care/Self/Protect fractions (mean over run logs)
    Conditioning: only ticks where mother+child are alive (already baked into frac_*_alive).
    """
    # Mean across run logs within each (plasticity, init)
    g = df.groupby(["plasticity", "init"], dropna=False)
    agg = g.agg(
        n=("n_alive_ticks", "count"),
        Forage=("frac_Forage_alive", "mean"),
        Care=("frac_Care_alive", "mean"),
        Self=("frac_Self_alive", "mean"),
        Protect=("frac_Protect_alive", "mean"),
    ).reset_index()

    # Stable order: panels by plasticity, within each init order
    order = []
    for plast in PLAST_ORDER:
        for init in INIT_ORDER:
            order.append((plast, init))
    key_to_row = {(r["plasticity"], r["init"]): r for _, r in agg.iterrows()}

    labels = []
    vals = {m: [] for m in MOTS}
    ns = []
    for plast, init in order:
        r = key_to_row.get((plast, init))
        labels.append(f"{plast}\n{init.replace('_',' ')}")
        if r is None:
            for m in MOTS:
                vals[m].append(np.nan)
            ns.append(0)
        else:
            for m in MOTS:
                vals[m].append(float(r[m]))
            ns.append(int(r["n"]))

    x = np.arange(len(labels), dtype=float)
    fig, ax = plt.subplots(figsize=(13.5, 4.6))

    bottom = np.zeros_like(x, dtype=float)
    for m in MOTS:
        y = np.array(vals[m], dtype=float)
        ax.bar(
            x,
            y,
            bottom=bottom,
            color=MOT_COLORS.get(m, "0.5"),
            edgecolor="white",
            linewidth=0.6,
            label=m,
        )
        bottom = bottom + np.nan_to_num(y, nan=0.0)

    ax.set_ylim(0, 1.0)
    ax.set_ylabel("Mean fraction (mother+child alive)")
    ax.set_xticks(x)
    ax.set_xticklabels(labels, rotation=0)
    ax.grid(True, axis="y", alpha=0.25)
    ax.set_title(title, fontsize=12, pad=10)
    ax.legend(loc="upper center", ncol=4, frameon=False, bbox_to_anchor=(0.5, 1.12))

    # Annotate n above each bar (how many run logs in that cell)
    for i, n in enumerate(ns):
        if n:
            ax.text(i, 1.01, f"n={n}", ha="center", va="bottom", fontsize=8, color="0.35")

    fig.tight_layout()
    fig.savefig(out_png, dpi=200, bbox_inches="tight")
    plt.close(fig)
    print("Wrote", out_png)


def _plot_maternal_vs_base(df: "pd.DataFrame", *, out_png: str, title: str) -> None:
    """
    One figure: Maternal vs Base fractions (means) per condition.
      maternal = Care + Protect
      base = Forage + Self
    """
    g = df.groupby(["plasticity", "init"], dropna=False)
    agg = g.agg(
        n=("n_alive_ticks", "count"),
        maternal=("maternal_fraction_alive", "mean"),
        base=("frac_Forage_alive", "mean"),
        self_frac=("frac_Self_alive", "mean"),
    ).reset_index()
    # base = Forage + Self
    agg["base"] = agg["base"].astype(float) + agg["self_frac"].astype(float)

    order = [(p, i) for p in PLAST_ORDER for i in INIT_ORDER]
    key_to_row = {(r["plasticity"], r["init"]): r for _, r in agg.iterrows()}

    labels, maternal, base, ns = [], [], [], []
    for plast, init in order:
        labels.append(f"{plast}\n{init.replace('_',' ')}")
        r = key_to_row.get((plast, init))
        if r is None:
            maternal.append(np.nan)
            base.append(np.nan)
            ns.append(0)
        else:
            maternal.append(float(r["maternal"]))
            base.append(float(r["base"]))
            ns.append(int(r["n"]))

    x = np.arange(len(labels), dtype=float)
    fig, ax = plt.subplots(figsize=(13.5, 4.3))

    maternal_y = np.array(maternal, dtype=float)
    base_y = np.array(base, dtype=float)

    ax.bar(x, base_y, color=PAIR_COLORS["base"], alpha=0.85, edgecolor="white", linewidth=0.6, label="Base (Forage+Self)")
    ax.bar(
        x,
        maternal_y,
        bottom=np.nan_to_num(base_y, nan=0.0),
        color=PAIR_COLORS["maternal"],
        alpha=0.85,
        edgecolor="white",
        linewidth=0.6,
        label="Maternal (Care+Protect)",
    )

    ax.set_ylim(0, 1.0)
    ax.set_ylabel("Mean fraction (mother+child alive)")
    ax.set_xticks(x)
    ax.set_xticklabels(labels)
    ax.grid(True, axis="y", alpha=0.25)
    ax.set_title(title, fontsize=12, pad=10)
    ax.legend(loc="upper center", ncol=2, frameon=False, bbox_to_anchor=(0.5, 1.12))

    for i, n in enumerate(ns):
        if n:
            ax.text(i, 1.01, f"n={n}", ha="center", va="bottom", fontsize=8, color="0.35")

    fig.tight_layout()
    fig.savefig(out_png, dpi=200, bbox_inches="tight")
    plt.close(fig)
    print("Wrote", out_png)
def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--root", required=True, help="Root with <run_dir>/run_*.csv")
    ap.add_argument("--out-dir", default=None, help="Output dir (default: <root>/figures)")
    ap.add_argument("--layout", choices=["split_init", "split_plasticity"], default="split_plasticity")
    ap.add_argument("--style", choices=["box", "violin"], default="box")
    ap.add_argument("--title-prefix", default="")
    ap.add_argument(
        "--stacked-summary",
        action="store_true",
        help="Also write one stacked-bar summary figure that shows all four motivation fractions at once.",
    )
    ap.add_argument(
        "--maternal-vs-base",
        action="store_true",
        help="Also write one summary figure collapsing to Maternal (Care+Protect) vs Base (Forage+Self).",
    )
    args = ap.parse_args()

    root = os.path.abspath(args.root)
    if not os.path.isdir(root):
        print(f"[error] not a directory: {root}", file=sys.stderr)
        return 2
    out_dir = os.path.abspath(args.out_dir) if args.out_dir else os.path.join(root, "figures")
    os.makedirs(out_dir, exist_ok=True)

    df = _collect(root)
    df.to_csv(os.path.join(out_dir, "motivation_fractions_alive.csv"), index=False)

    prefix = (args.title_prefix + " — ") if args.title_prefix else ""
    if args.stacked_summary:
        _plot_stacked_summary(
            df,
            out_png=os.path.join(out_dir, f"motivation_fractions_alive_stacked.png"),
            title=f"{prefix}Motivation fractions while mother+child alive (stacked mean)",
        )
    if args.maternal_vs_base:
        _plot_maternal_vs_base(
            df,
            out_png=os.path.join(out_dir, "motivation_fractions_alive_maternal_vs_base.png"),
            title=f"{prefix}Maternal vs Base while mother+child alive (stacked mean)",
        )
    for mot in MOTS + ["maternal_fraction"]:
        col = "maternal_fraction_alive" if mot == "maternal_fraction" else f"frac_{mot}_alive"
        title = f"{prefix}{mot.replace('_', ' ').title()} fraction (mother+child alive)"
        out_png = os.path.join(out_dir, f"frac_{mot}_alive_{args.layout}_{args.style}.png")
        _plot(df, value_col=col, out_png=out_png, title=title, layout=args.layout, style=args.style)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

