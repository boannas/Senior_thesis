#!/usr/bin/env python3
"""Status check for evolve-lineage run trees.

Walks the given directory trees and classifies every run directory as:
  COMPLETE   final_genome.json exists (run finished cleanly)
  PARTIAL    lineage_generations.csv and/or latest_state.json exist but no
             final_genome.json (interrupted; resumable with --resume)
  EMPTY      run_config.json/cli_args.txt only (no progress yet)

Usage:
  python check_runs.py [<root_dir> ...]
  python check_runs.py Evolved_results Plastic_1_results Plastic_2_results
  python check_runs.py --show-resume-cmd Evolved_results
  python check_runs.py --only PARTIAL --show-resume-cmd .

The --show-resume-cmd flag prints a ready-to-paste resume command for every
PARTIAL run, reusing the original cli_args.txt and appending --resume.
"""

from __future__ import annotations

import argparse
import csv
import json
import os
import sys
from pathlib import Path

# Files that indicate "this directory is a run output" (used to skip
# unrelated subfolders like __pycache__/ or figures/).
_RUN_MARKER_FILES = (
    "run_config.json",
    "cli_args.txt",
    "lineage_generations.csv",
    "final_genome.json",
    "latest_state.json",
)


def _is_run_dir(p: Path) -> bool:
    return any((p / name).exists() for name in _RUN_MARKER_FILES)


def _count_csv_rows(p: Path) -> int:
    if not p.exists():
        return 0
    n = 0
    try:
        with open(p, encoding="utf-8") as f:
            for _ in csv.DictReader(f):
                n += 1
    except (OSError, csv.Error):
        return -1
    return n


def _target_generations(run_dir: Path) -> int | None:
    """Total expected rows = num_generations + 1 (gen 0 included)."""
    cfg = run_dir / "run_config.json"
    if not cfg.exists():
        return None
    try:
        with open(cfg, encoding="utf-8") as f:
            d = json.load(f)
        v = d.get("cfg", {}).get("num_generations")
        if v is not None:
            return int(v) + 1
    except (OSError, json.JSONDecodeError, ValueError, TypeError):
        pass
    return None


def _last_completed_gen(run_dir: Path) -> int | None:
    state = run_dir / "latest_state.json"
    if not state.exists():
        return None
    try:
        with open(state, encoding="utf-8") as f:
            d = json.load(f)
        return int(d.get("last_completed_gen", -1))
    except (OSError, json.JSONDecodeError, ValueError, TypeError):
        return None


def _read_cli_args(run_dir: Path) -> str | None:
    cli = run_dir / "cli_args.txt"
    if not cli.exists():
        return None
    try:
        with open(cli, encoding="utf-8") as f:
            txt = f.read().strip()
        return txt or None
    except OSError:
        return None


def _classify(run_dir: Path) -> str:
    if (run_dir / "final_genome.json").exists():
        return "COMPLETE"
    if (run_dir / "lineage_generations.csv").exists() or (
        run_dir / "latest_state.json"
    ).exists():
        return "PARTIAL"
    return "EMPTY"


def _scan(roots: list[Path]) -> list[dict]:
    seen: set[Path] = set()
    out: list[dict] = []
    for root in roots:
        if not root.exists():
            print(f"[skip] {root} (does not exist)", file=sys.stderr)
            continue
        # Include the root itself as a candidate run dir.
        candidates: list[Path] = [root] + [d for d in root.rglob("*") if d.is_dir()]
        for run_dir in candidates:
            if run_dir in seen:
                continue
            if not _is_run_dir(run_dir):
                continue
            seen.add(run_dir)

            csv_path = run_dir / "lineage_generations.csv"
            csv_rows = _count_csv_rows(csv_path)
            target = _target_generations(run_dir)
            last_state = _last_completed_gen(run_dir)
            status = _classify(run_dir)

            out.append(
                {
                    "status": status,
                    "path": run_dir,
                    "csv_rows": csv_rows,
                    "target": target,
                    "last_state": last_state,
                    "cli": _read_cli_args(run_dir),
                }
            )
    return out


def _fmt(v) -> str:
    if v is None:
        return "-"
    return str(v)


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("roots", nargs="*", default=["."], help="Directory trees to scan (default: current dir).")
    p.add_argument(
        "--only",
        choices=["COMPLETE", "PARTIAL", "EMPTY"],
        default=None,
        help="Show only runs with this status.",
    )
    p.add_argument(
        "--show-resume-cmd",
        action="store_true",
        help="Print a ready-to-paste resume command for every PARTIAL run.",
    )
    p.add_argument(
        "--python",
        default="python",
        help="Python interpreter to put at the front of the resume command (default: python).",
    )
    args = p.parse_args()

    roots = [Path(r).resolve() for r in args.roots]
    rows = _scan(roots)

    if args.only is not None:
        rows = [r for r in rows if r["status"] == args.only]

    if not rows:
        print("No run directories found.")
        return 0

    rows.sort(key=lambda r: (r["status"], str(r["path"])))

    width = max(len(str(r["path"])) for r in rows)
    width = min(max(width, 30), 90)

    print(f"{'STATUS':<10} {'CSV':>6} {'TARGET':>7} {'LAST':>5}  PATH")
    print("-" * (10 + 1 + 6 + 1 + 7 + 1 + 5 + 2 + width))
    for r in rows:
        path_disp = str(r["path"])
        if len(path_disp) > width:
            path_disp = "…" + path_disp[-(width - 1):]
        csv_disp = "-" if r["csv_rows"] == 0 else _fmt(r["csv_rows"])
        print(
            f"{r['status']:<10} {csv_disp:>6} {_fmt(r['target']):>7} "
            f"{_fmt(r['last_state']):>5}  {path_disp}"
        )

    n_complete = sum(1 for r in rows if r["status"] == "COMPLETE")
    n_partial = sum(1 for r in rows if r["status"] == "PARTIAL")
    n_empty = sum(1 for r in rows if r["status"] == "EMPTY")
    print()
    print(
        f"Summary: COMPLETE={n_complete}  PARTIAL={n_partial}  EMPTY={n_empty}  "
        f"(total {len(rows)})"
    )

    if args.show_resume_cmd:
        partials = [r for r in rows if r["status"] == "PARTIAL"]
        if partials:
            print()
            print("# Resume commands (paste into the same kind of shell that started the run):")
            for r in partials:
                cli = r["cli"]
                if cli:
                    print(f"{args.python} {cli} --resume")
                else:
                    print(
                        f"# (no cli_args.txt for {r['path']}; cannot auto-build "
                        f"a resume command)"
                    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
