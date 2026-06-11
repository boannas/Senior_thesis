"""
Run baseline_passive_lower_bound.py across plasticity settings and rank by child survival.

Default preset matches the paper-style stack: active mother, local deficit, segment_capped,
learn-w off, kmax=20 — sweeps plasticity rule: none + outcome + outcome_adaptive + signed variants.

Usage (from project root):
  python compare_plasticity_rules.py --genome result_experiment/baseline_zero_genome.json \\
      --output-root result_experiment/plasticity_sweep --replicates 32

  # Full factorial with segment_capped at k=20 and k=0 (per_tick/segment run once each)
  python compare_plasticity_rules.py --preset full --segment-kmax 20 0 --genome ... --replicates 16

  # List jobs without running
  python compare_plasticity_rules.py --preset full --segment-kmax 20 0 --dry-run

After runs: reads each folder's survival_experiment_replicates.csv, writes
  plasticity_comparison_summary.csv under --output-root (sorted by mean child_death_tick).

``segment_kmax`` (CLI ``--segment-kmax``) only affects ``segment_capped``:
  - ``k > 0``: force a plasticity update every k ticks if the same motivation stays selected.
  - ``k = 0``: cap is off → updates only when motivation **changes** (same as ``segment``).

Pass multiple caps as ``--segment-kmax 20 0``: only ``segment_capped`` folders are duplicated
per k; ``per_tick`` / ``segment`` run once (CLI still gets the first k, which those modes ignore).
"""

from __future__ import annotations

import argparse
import itertools
import os
import subprocess
import sys
from dataclasses import dataclass

try:
    import pandas as pd
except ImportError as e:
    raise SystemExit("pandas required: pip install pandas") from e


BASELINE_SCRIPT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "baseline_passive_lower_bound.py")


@dataclass(frozen=True)
class Job:
    plasticity: str  # none | outcome | outcome_adaptive | outcome_signed | outcome_adaptive_signed
    deficit_signal: str  # global | local (ignored when plasticity==none)
    update_mode: str  # per_tick | segment | segment_capped
    segment_kmax: int
    learn_w: str  # on | off

    def out_dir_name(self) -> str:
        if self.plasticity == "none":
            return "plast_none_active_fixed"
        lw = "lw1" if self.learn_w == "on" else "lw0"
        if self.update_mode == "segment_capped":
            kpart = f"k{self.segment_kmax}"
        else:
            kpart = "kna"  # segment_kmax unused for per_tick / segment
        return (
            f"plast_{self.plasticity}__ds-{self.deficit_signal}__"
            f"um-{self.update_mode}__{kpart}__{lw}"
        )


def _jobs_rules_local_segcap(learn_w: str, segment_kmax_values: list[int]) -> list[Job]:
    """Paper-style: local + segment_capped, sweep rules + none; one folder per k for each rule."""
    k0 = int(segment_kmax_values[0])
    jobs = [Job("none", "local", "segment_capped", k0, learn_w)]
    for rule in ("outcome", "outcome_adaptive", "outcome_signed", "outcome_adaptive_signed"):
        for k in segment_kmax_values:
            jobs.append(Job(rule, "local", "segment_capped", int(k), learn_w))
    return jobs


def _jobs_full(learn_w: str, segment_kmax_values: list[int]) -> list[Job]:
    """All combinations of plastic rules × deficit × update (+ none).

    ``segment_capped`` is repeated for each k in ``segment_kmax_values``.
    ``per_tick`` / ``segment`` appear once each (k stored as k0 for CLI only).
    """
    rules = ("outcome", "outcome_adaptive", "outcome_signed", "outcome_adaptive_signed")
    deficits = ("global", "local")
    updates = ("per_tick", "segment", "segment_capped")
    k0 = int(segment_kmax_values[0])
    jobs = [Job("none", "local", "segment_capped", k0, learn_w)]
    for rule, d, u in itertools.product(rules, deficits, updates):
        if u == "segment_capped":
            for k in segment_kmax_values:
                jobs.append(Job(rule, d, u, int(k), learn_w))
        else:
            jobs.append(Job(rule, d, u, k0, learn_w))
    return jobs


def _build_cmd(
    job: Job,
    *,
    genome: str | None,
    replicates: int,
    max_ticks: int,
    threats: int,
    seed_base: int,
    grid_w: int,
    grid_h: int,
    log_timeseries: bool,
) -> list[str]:
    cmd = [
        sys.executable,
        BASELINE_SCRIPT,
        "--mode",
        "active",
        "--plasticity",
        job.plasticity,
        "--deficit-signal",
        job.deficit_signal,
        "--learn-w",
        job.learn_w,
        "--update-mode",
        job.update_mode,
        "--segment-kmax",
        str(job.segment_kmax),
        "--replicates",
        str(replicates),
        "--max-ticks",
        str(max_ticks),
        "--threats",
        str(threats),
        "--seed-base",
        str(seed_base),
        "--grid-w",
        str(grid_w),
        "--grid-h",
        str(grid_h),
    ]
    if genome:
        cmd.extend(["--genome", genome])
    if not log_timeseries:
        cmd.append("--no-log-timeseries")
    return cmd


def _summarize_folder(path: str) -> dict | None:
    csv_path = os.path.join(path, "survival_experiment_replicates.csv")
    if not os.path.isfile(csv_path):
        return None
    df = pd.read_csv(csv_path)
    if "child_death_tick" not in df.columns:
        return None
    cdt = pd.to_numeric(df["child_death_tick"], errors="coerce")
    alive_col = "child_alive_end" if "child_alive_end" in df.columns else None
    p_alive = float(pd.to_numeric(df[alive_col], errors="coerce").mean()) if alive_col else float("nan")
    return {
        "path": path,
        "folder": os.path.basename(path.rstrip(os.sep)),
        "n": int(len(df)),
        "mean_child_ttd": float(cdt.mean()),
        "std_child_ttd": float(cdt.std(ddof=0)),
        "median_child_ttd": float(cdt.median()),
        "p_child_alive_end": p_alive,
    }


def main() -> None:
    p = argparse.ArgumentParser(description="Sweep plasticity rules and rank by mean child TTD.")
    p.add_argument(
        "--preset",
        choices=("rules_local_segcap", "full"),
        default="rules_local_segcap",
        help="rules_local_segcap: none + 4 rules × each --segment-kmax value (local segment_capped). "
        "full: per_tick+segment once each; segment_capped for each k; + none (8+8+16k+1 with two k).",
    )
    p.add_argument("--genome", type=str, default=None, help="Motivation JSON (recommended).")
    p.add_argument("--output-root", type=str, default="result_experiment/plasticity_sweep")
    p.add_argument("--replicates", type=int, default=32)
    p.add_argument("--max-ticks", type=int, default=1000)
    p.add_argument("--threats", type=int, default=1)
    p.add_argument("--seed-base", type=int, default=50_000)
    p.add_argument("--grid-w", type=int, default=10)
    p.add_argument("--grid-h", type=int, default=10)
    p.add_argument(
        "--segment-kmax",
        type=int,
        nargs="+",
        default=[20],
        metavar="K",
        help="segment_capped: cap length(s). Use e.g. '20 0' to compare forced updates vs cap off. "
        "0 = cap off (like segment). per_tick/segment jobs use only the first K for CLI (ignored).",
    )
    p.add_argument(
        "--learn-w",
        choices=("off", "on"),
        default="off",
        help="Whether psych weights w update (default off, matches E1b).",
    )
    p.add_argument(
        "--log-timeseries",
        action="store_true",
        help="Enable per-tick CSV logs (slower; off by default for sweeps).",
    )
    p.add_argument("--dry-run", action="store_true", help="Print commands only.")
    p.add_argument(
        "--skip-existing",
        action="store_true",
        help="Skip a job if survival_experiment_replicates.csv already exists.",
    )
    p.add_argument("--summarize-only", action="store_true", help="Only scan output-root and write summary CSV.")
    args = p.parse_args()

    learn_w = args.learn_w
    k_list = [int(x) for x in args.segment_kmax]
    if args.preset == "rules_local_segcap":
        jobs = _jobs_rules_local_segcap(learn_w, k_list)
    else:
        jobs = _jobs_full(learn_w, k_list)

    os.makedirs(args.output_root, exist_ok=True)

    if not args.summarize_only:
        for job in jobs:
            out = os.path.join(args.output_root, job.out_dir_name())
            surv = os.path.join(out, "survival_experiment_replicates.csv")
            if args.skip_existing and os.path.isfile(surv):
                print(f"[skip] {job.out_dir_name()}")
                continue
            os.makedirs(out, exist_ok=True)
            cmd = _build_cmd(
                job,
                genome=args.genome,
                replicates=int(args.replicates),
                max_ticks=int(args.max_ticks),
                threats=int(args.threats),
                seed_base=int(args.seed_base),
                grid_w=int(args.grid_w),
                grid_h=int(args.grid_h),
                log_timeseries=bool(args.log_timeseries),
            )
            cmd.extend(["--output-dir", out])
            print("RUN:", " ".join(cmd))
            if args.dry_run:
                continue
            r = subprocess.run(cmd, cwd=os.path.dirname(BASELINE_SCRIPT) or ".")
            if r.returncode != 0:
                print(f"[error] exit {r.returncode} for {job.out_dir_name()}", file=sys.stderr)

    # Summarize all folders under output-root that look like our runs
    rows = []
    for name in sorted(os.listdir(args.output_root)):
        path = os.path.join(args.output_root, name)
        if not os.path.isdir(path):
            continue
        s = _summarize_folder(path)
        if s:
            rows.append(s)

    if not rows:
        print("No survival_experiment_replicates.csv found under", args.output_root)
        return

    summary = pd.DataFrame(rows)
    summary = summary.sort_values("mean_child_ttd", ascending=False).reset_index(drop=True)
    summary_path = os.path.join(args.output_root, "plasticity_comparison_summary.csv")
    summary.to_csv(summary_path, index=False)
    print("\nWrote", summary_path)
    print("\nRanked by mean child_death_tick (higher = longer survival; censored at max-ticks counts as max):")
    cols = ["folder", "mean_child_ttd", "std_child_ttd", "median_child_ttd", "p_child_alive_end", "n"]
    print(summary[cols].to_string(index=False))


if __name__ == "__main__":
    main()
