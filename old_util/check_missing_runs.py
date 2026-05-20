#!/usr/bin/env python3
"""Compare disk to the RUN_GUIDE.md experiment grid (expected dirs only).

check_runs.py only sees folders that already contain run markers. This script
enumerates every output directory that the RUN_GUIDE loops *would* create and
reports:

  COMPLETE     final_genome.json exists
  INCOMPLETE   dir exists, no final_genome.json (never finished / partial / empty)
  MISSING      dir does not exist (never launched)

Design mirrored from RUN_GUIDE.md:
  Emergence: Emergence_results/normal/{E2|P1|P2}_{init}_seed{N}
  Sweep:     Sweep_results/{easy|normal|hard}/{E2|P1|P2}_thr{T}_g15_i15_a{X}_seed{N}

Seeds 42–46; alphas 0.3/0.5/0.7; sweep threats 0/1/2 per world.

After a scan, run pending work with:

  python3 check_missing_runs.py --only PENDING --print-cmds --python python3 > pending.sh
  # review, then from repo root:
  bash pending.sh

Use tmux / multiple terminals and split pending.sh (e.g. GNU split -n l/12/12).
"""

from __future__ import annotations

import argparse
import re
import shlex
import sys
from pathlib import Path


def _atag(alpha: float) -> str:
    return str(alpha).replace(".", "p")


def iter_expected_paths(repo_root: Path) -> list[Path]:
    emergence = repo_root / "Emergence_results" / "normal"
    sweep = repo_root / "Sweep_results"
    inits = ("anti_maternal", "random_uniform", "pro_maternal")
    seeds = (42, 43, 44, 45, 46)
    alphas = (0.3, 0.5, 0.7)
    worlds = (("easy", 0), ("normal", 1), ("hard", 2))

    out: list[Path] = []
    for prefix in ("E2", "P1", "P2"):
        for init in inits:
            for s in seeds:
                out.append(emergence / f"{prefix}_{init}_seed{s}")

    for world, thr in worlds:
        for prefix in ("E2", "P1", "P2"):
            for a in alphas:
                at = _atag(a)
                for s in seeds:
                    tag = f"{prefix}_thr{thr}_g15_i15_a{at}_seed{s}"
                    out.append(sweep / world / tag)
    return out


def classify_all(repo_root: Path) -> list[tuple[str, Path]]:
    rows: list[tuple[str, Path]] = []
    for path in iter_expected_paths(repo_root):
        final = path / "final_genome.json"
        if not path.exists():
            rows.append(("MISSING", path))
        elif final.exists():
            rows.append(("COMPLETE", path))
        else:
            rows.append(("INCOMPLETE", path))
    return rows


_ATAG_TO_ALPHA = {"0p3": "0.3", "0p5": "0.5", "0p7": "0.7"}


def _parse_emergence(name: str) -> tuple[str, str, int] | None:
    m = re.fullmatch(r"(E2|P1|P2)_(.+)_seed(\d+)", name)
    if not m:
        return None
    return m.group(1), m.group(2), int(m.group(3))


def _parse_sweep(name: str) -> tuple[str, int, str, int] | None:
    m = re.fullmatch(r"(E2|P1|P2)_thr(\d+)_g15_i15_a(0p[357])_seed(\d+)", name)
    if not m:
        return None
    at = _ATAG_TO_ALPHA.get(m.group(3))
    if at is None:
        return None
    return m.group(1), int(m.group(2)), at, int(m.group(4))


def _emergence_launch(name: str, python_exe: str) -> str:
    p = _parse_emergence(name)
    if not p:
        raise ValueError(f"cannot parse emergence dir name: {name!r}")
    prefix, init, seed = p
    tag = name
    od = f"Emergence_results/normal/{tag}"
    argv: list[str] = [
        python_exe,
        "run_evolve_lineage.py",
        "--generations",
        "3000",
        "--episodes",
        "10",
        "--max-ticks",
        "1000",
        "--grid-w",
        "15",
        "--grid-h",
        "15",
        "--threats",
        "1",
        "--food-spawn-interval",
        "15",
        "--food-spawn-n",
        "1",
        "--fitness-mode",
        "ttd_overall",
        "--alpha-child",
        "0.5",
        "--seed-master",
        str(seed),
        "--init-seed",
        str(seed),
        "--init-mode",
        init,
        "--init-noise",
        "0.05",
    ]
    if prefix == "E2":
        argv += ["--plasticity", "none", "--checkpoint-every", "500"]
    elif prefix == "P1":
        argv += [
            "--plasticity",
            "outcome_adaptive",
            "--deficit-signal",
            "global",
            "--learn-w",
            "off",
            "--update-mode",
            "per_tick",
            "--checkpoint-every",
            "500",
        ]
    elif prefix == "P2":
        argv += [
            "--plasticity",
            "outcome",
            "--deficit-signal",
            "local",
            "--learn-w",
            "off",
            "--update-mode",
            "per_tick",
            "--checkpoint-every",
            "500",
        ]
    argv += ["--output-dir", od]
    return shlex.join(argv)


def _sweep_launch(world: str, name: str, python_exe: str) -> str:
    p = _parse_sweep(name)
    if not p:
        raise ValueError(f"cannot parse sweep dir name: {name!r}")
    prefix, thr, alpha_str, seed = p
    exp_thr = {"easy": 0, "normal": 1, "hard": 2}[world]
    if thr != exp_thr:
        raise ValueError(f"world {world!r} does not match thr={thr} in {name!r}")
    od = f"Sweep_results/{world}/{name}"
    argv: list[str] = [
        python_exe,
        "run_evolve_lineage.py",
        "--generations",
        "3000",
        "--episodes",
        "10",
        "--max-ticks",
        "1000",
        "--grid-w",
        "15",
        "--grid-h",
        "15",
        "--threats",
        str(thr),
        "--food-spawn-interval",
        "15",
        "--food-spawn-n",
        "1",
        "--fitness-mode",
        "ttd_overall",
        "--alpha-child",
        alpha_str,
        "--seed-master",
        str(seed),
        "--init-seed",
        str(seed),
        "--init-mode",
        "baseline_zero",
    ]
    if prefix == "E2":
        argv += ["--plasticity", "none"]
    elif prefix == "P1":
        argv += [
            "--plasticity",
            "outcome_adaptive",
            "--deficit-signal",
            "global",
            "--learn-w",
            "off",
            "--update-mode",
            "per_tick",
        ]
    elif prefix == "P2":
        argv += [
            "--plasticity",
            "outcome",
            "--deficit-signal",
            "local",
            "--learn-w",
            "off",
            "--update-mode",
            "per_tick",
        ]
    argv += ["--output-dir", od]
    return shlex.join(argv)


def pending_command(repo_root: Path, out_dir: Path, python_exe: str) -> str:
    """One shell-ready line: resume if cli_args exists, else RUN_GUIDE launch."""
    final = out_dir / "final_genome.json"
    if final.exists():
        raise ValueError("run already complete")
    cli = out_dir / "cli_args.txt"
    if out_dir.exists() and cli.is_file():
        body = cli.read_text(encoding="utf-8").strip()
        if body:
            return shlex.join([python_exe, *shlex.split(body), "--resume"])

    name = out_dir.name
    parts = out_dir.parts
    if "Emergence_results" in parts:
        return _emergence_launch(name, python_exe)
    if "Sweep_results" in parts:
        wi = parts.index("Sweep_results")
        world = parts[wi + 1]
        return _sweep_launch(world, name, python_exe)
    raise ValueError(f"unexpected output path: {out_dir}")


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument(
        "--root",
        type=Path,
        default=Path.cwd(),
        help="Repository root (default: current directory).",
    )
    p.add_argument(
        "--only",
        choices=("MISSING", "INCOMPLETE", "COMPLETE", "PENDING"),
        default=None,
        help="PENDING = MISSING or INCOMPLETE (not done yet).",
    )
    p.add_argument(
        "--print-cmds",
        action="store_true",
        help="Print one runnable shell line per selected non-complete run (use with --only PENDING).",
    )
    p.add_argument(
        "--python",
        dest="python_exe",
        default="python3",
        help="Python executable for generated commands (default: python3).",
    )
    args = p.parse_args()
    root = args.root.resolve()

    full = classify_all(root)
    rows = full[:]
    if args.only == "PENDING":
        rows = [r for r in rows if r[0] in ("MISSING", "INCOMPLETE")]
    elif args.only:
        rows = [r for r in rows if r[0] == args.only]

    n_miss_all = sum(1 for s, _ in full if s == "MISSING")
    n_inc_all = sum(1 for s, _ in full if s == "INCOMPLETE")
    n_ok_all = sum(1 for s, _ in full if s == "COMPLETE")

    if args.print_cmds:
        for status, path in sorted(rows, key=lambda x: (x[0], str(x[1]))):
            if status == "COMPLETE":
                continue
            try:
                print(pending_command(root, path, args.python_exe))
            except (OSError, ValueError) as e:
                print(f"# ERROR {path}: {e}", file=sys.stderr)
        print(
            f"# Summary: COMPLETE={n_ok_all}  INCOMPLETE={n_inc_all}  MISSING={n_miss_all}  "
            f"PENDING={n_miss_all + n_inc_all}  (expected {len(full)} dirs)",
            file=sys.stderr,
        )
        return 0

    width = min(max(len(str(r[1])) for r in rows) if rows else 40, 100)
    for status, path in sorted(rows, key=lambda x: (x[0], str(x[1]))):
        try:
            rel = path.relative_to(root)
        except ValueError:
            rel = path
        disp = str(rel)
        if len(disp) > width:
            disp = "…" + disp[-(width - 1) :]
        print(f"{status:<11} {disp}")

    print()
    print(
        f"Summary: COMPLETE={n_ok_all}  INCOMPLETE={n_inc_all}  MISSING={n_miss_all}  "
        f"PENDING={n_miss_all + n_inc_all}  (expected {len(full)} dirs)"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
