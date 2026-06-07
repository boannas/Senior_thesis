#!/usr/bin/env python3
"""
plot_e4_death_cause_summary_tiles.py
====================================
Slide-ready *tile* summary for E4 child death causes.

Why tiles?
  Stacked bars are accurate but hard to scan. This plot summarizes each unseen
  condition (world × food) by the dominant outcome in that cell.

Input (produced by plot_e4_child_death_causes_all.py):
  E4_figures/death_causes/E4_child_death_causes_dominant__per_condition.csv

Output:
  E4_figures/death_causes/E4_child_death_causes_summary_tiles.png

Layout:
  - rows: init (anti / random / pro)
  - cols: plasticity (E2 / P1 / P2)
  - each small panel: 3×3 tiles (world rows: easy/normal/hard; food cols: 5/15/25)
    tile color = dominant outcome, text = "<abbr> <pct>%"
"""

from __future__ import annotations

import argparse
import os

try:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import pandas as pd
except ImportError as e:
    raise SystemExit(f"Requires pandas+matplotlib: {e}") from e


WORLDS = ("easy", "normal", "hard")
FOODS = (5, 15, 25)
INIT_ORDER = ("anti_maternal", "random_uniform", "pro_maternal")
INIT_LABELS = {
    "anti_maternal": "anti-maternal",
    "random_uniform": "random",
    "pro_maternal": "pro-maternal",
}
PLAST_ORDER = ("E2", "P1", "P2")

CAUSE_COLORS = {
    "alive": "#7f7f7f",
    "injury": "#9467bd",
    "hunger": "#e15759",
    "cold": "#4e79a7",
    "heat": "#f28e2b",
    "unknown_nonfinite": "#9c755f",
    "unknown_missing": "#bab0ac",
}
CAUSE_ABBR = {
    "alive": "A",
    "injury": "inj",
    "hunger": "hun",
    "cold": "cold",
    "heat": "heat",
    "unknown_nonfinite": "unk",
    "unknown_missing": "miss",
}


def _light_text(bg_hex: str) -> bool:
    """Return True if text should be light on this background."""
    bg_hex = bg_hex.lstrip("#")
    r = int(bg_hex[0:2], 16) / 255.0
    g = int(bg_hex[2:4], 16) / 255.0
    b = int(bg_hex[4:6], 16) / 255.0
    # perceived luminance
    lum = 0.2126 * r + 0.7152 * g + 0.0722 * b
    return lum < 0.55


def _draw_panel(ax: plt.Axes, sub: pd.DataFrame, *, title: str) -> None:
    ax.set_xlim(0, len(FOODS))
    ax.set_ylim(0, len(WORLDS))
    ax.invert_yaxis()
    ax.set_aspect("equal")
    ax.set_xticks([i + 0.5 for i in range(len(FOODS))])
    ax.set_xticklabels([str(f) for f in FOODS], fontsize=8)
    ax.set_yticks([i + 0.5 for i in range(len(WORLDS))])
    ax.set_yticklabels(list(WORLDS), fontsize=8)
    ax.tick_params(length=0)
    ax.set_title(title, fontsize=10, pad=6)

    # grid lines
    for x in range(len(FOODS) + 1):
        ax.axvline(x, color="white", linewidth=1.0)
    for y in range(len(WORLDS) + 1):
        ax.axhline(y, color="white", linewidth=1.0)

    idx = {(r["world"], int(r["food"])): r for _, r in sub.iterrows()}

    for yi, w in enumerate(WORLDS):
        for xi, f in enumerate(FOODS):
            r = idx.get((w, f), None)
            if r is None:
                cause = "unknown_missing"
                frac = 0.0
            else:
                cause = str(r["dominant_cause"])
                frac = float(r["dominant_frac"])
            color = CAUSE_COLORS.get(cause, "#999999")
            rect = plt.Rectangle((xi, yi), 1, 1, facecolor=color, edgecolor="white", linewidth=1.0)
            ax.add_patch(rect)

            ab = CAUSE_ABBR.get(cause, cause[:4])
            txt = f"{ab} {int(round(100 * frac))}%"
            tcolor = "white" if _light_text(color) else "black"
            ax.text(xi + 0.5, yi + 0.52, txt, ha="center", va="center", fontsize=8, color=tcolor)

    # clean spines
    for s in ax.spines.values():
        s.set_visible(False)


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--in-dir", default="E4_figures/death_causes")
    ap.add_argument("--out-dir", default="E4_figures/death_causes")
    ap.add_argument("--title", default="E4 child outcomes (dominant cause per unseen condition)")
    args = ap.parse_args(argv)

    in_dir = os.path.abspath(args.in_dir)
    out_dir = os.path.abspath(args.out_dir)
    os.makedirs(out_dir, exist_ok=True)

    csv_path = os.path.join(in_dir, "E4_child_death_causes_dominant__per_condition.csv")
    if not os.path.isfile(csv_path):
        raise SystemExit(
            f"Missing {csv_path}. Run:\n"
            f"  python3 plot_e4_child_death_causes_all.py --out-dir {in_dir}\n"
        )

    df = pd.read_csv(csv_path)
    for c in ("init", "plasticity", "world", "food", "dominant_cause", "dominant_frac"):
        if c not in df.columns:
            raise SystemExit(f"CSV missing column {c}: {csv_path}")
    df["food"] = pd.to_numeric(df["food"], errors="coerce").astype(int)

    fig, axes = plt.subplots(
        len(INIT_ORDER),
        len(PLAST_ORDER),
        figsize=(13.6, 8.6),
        constrained_layout=False,
    )
    # Manual spacing: add breathing room and reserve space for title+legend.
    fig.subplots_adjust(left=0.06, right=0.985, top=0.90, bottom=0.12, wspace=0.28, hspace=0.32)

    for ri, init in enumerate(INIT_ORDER):
        for ci, plast in enumerate(PLAST_ORDER):
            ax = axes[ri, ci]
            sub = df[(df["init"] == init) & (df["plasticity"] == plast)].copy()
            title = f"{INIT_LABELS.get(init, init)} | {plast}"
            _draw_panel(ax, sub, title=title)
            if ri != len(INIT_ORDER) - 1:
                ax.set_xlabel("")
                ax.set_xticklabels([])
            if ci != 0:
                ax.set_ylabel("")
                ax.set_yticklabels([])

    # legend
    legend_items = ["alive", "injury", "hunger", "cold"]
    handles = [plt.Rectangle((0, 0), 1, 1, facecolor=CAUSE_COLORS[k], edgecolor="none", label=k) for k in legend_items]
    fig.legend(
        handles=handles,
        loc="lower center",
        ncol=len(handles),
        frameon=False,
        fontsize=10,
        bbox_to_anchor=(0.5, 0.03),
    )
    fig.suptitle(args.title, fontsize=14, y=0.965)

    out_png = os.path.join(out_dir, "E4_child_death_causes_summary_tiles.png")
    fig.savefig(out_png, dpi=240, bbox_inches="tight")
    plt.close(fig)
    print("[ok] wrote", out_png)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

