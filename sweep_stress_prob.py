#!/usr/bin/env python3
"""
sweep_stress_prob.py — probabilistic stress sweep.

For each compute scale factor k in --factors, run the simulator
--seeds-per-k times with different RNG seeds. Aggregate the per-(k,seed)
fall outcomes into a fall-probability curve P(fall | k), and report
boundary metrics:

    k(P=0.05)   first k where 5% of seeds fall   "safe operating point"
    k(P=0.50)   median boundary
    k(P=0.95)   point past which catastrophic failure dominates

Outputs:
    results/prob_sweep_raw.csv         one row per (k, seed)
    results/prob_sweep_summary.csv     one row per k with P(fall) + CIs
    results/prob_sweep_summary.md      pasteable Markdown summary
    results/prob_sweep.png             plot of P(fall) vs k with CIs

Usage (default = Docker mode, sensor_fusion executable):
    python3 sweep_stress_prob.py
    python3 sweep_stress_prob.py --factors 10 12 14 16 18 20 22 25 \\
                                 --seeds-per-k 10
    python3 sweep_stress_prob.py --no-docker --binary ./build/sensor_fusion
    python3 sweep_stress_prob.py --skip-runs                # re-plot only

Each (k, seed) run takes ~5–10 s wall-clock through Docker. A 14×10 sweep
is ~12 minutes total.
"""

from __future__ import annotations

import argparse
import csv
import math
import os
import shlex
import subprocess
import sys
from pathlib import Path
from typing import Dict, List, Optional, Tuple

try:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    HAVE_MPL = True
except ImportError:
    HAVE_MPL = False


# Default fine-grained sweep around k=20, where the deterministic v3 run
# saw the boundary. ±5 in 1-step increments covers the transition region.
DEFAULT_FACTORS = [10, 12, 14, 15, 16, 17, 18, 19, 20, 22, 25, 30]
DEFAULT_SEEDS = 10


def fmt_k(k: float) -> str:
    return f"{k:g}"


def case_label(k: float, seed: int) -> str:
    return f"prob_k{fmt_k(k)}_s{seed}"


def get_user_spec() -> str:
    if os.name == "nt": return ""
    try: return f"{os.getuid()}:{os.getgid()}"
    except AttributeError: return ""


def build_docker_cmd(exec_name: str, sim_args: List[str],
                     compose_service: str, user_spec: str) -> List[str]:
    inner = "./build-docker/" + exec_name + " " + " ".join(shlex.quote(a) for a in sim_args)
    cmd = ["docker", "compose", "run", "--rm"]
    if user_spec: cmd += ["--user", user_spec]
    cmd += [compose_service, "bash", "-lc", inner]
    return cmd


def run_one(cmd: List[str], cwd: Path, k: float, seed: int,
            timeout_s: int) -> Tuple[bool, str]:
    try:
        proc = subprocess.run(cmd, cwd=cwd, capture_output=True, text=True,
                              timeout=timeout_s)
    except subprocess.TimeoutExpired:
        return False, f"TIMEOUT after {timeout_s} s"
    except FileNotFoundError as e:
        return False, f"command not found: {e}"
    if proc.returncode != 0:
        tail = "\n".join(proc.stderr.strip().splitlines()[-3:])
        return False, f"rc={proc.returncode}: {tail}"
    return True, ""


def parse_summary(case_dir: Path) -> Dict[str, str]:
    out: Dict[str, str] = {}
    p = case_dir / "summary.csv"
    if not p.is_file(): return out
    with p.open(newline="") as f:
        for row in csv.DictReader(f):
            out[row["metric"]] = row["value"]
    return out


def asfloat(s: Optional[str], default: float = float("nan")) -> float:
    if s is None: return default
    try: return float(s)
    except ValueError: return default


def asint(s: Optional[str], default: int = 0) -> int:
    if s is None: return default
    try: return int(s)
    except ValueError: return default


def collect_raw(results_root: Path, factors: List[float],
                seeds: List[int]) -> List[Dict]:
    """One row per (k, seed) — the raw observation."""
    rows = []
    for k in factors:
        for seed in seeds:
            d = results_root / case_label(k, seed)
            s = parse_summary(d)
            if not s:
                continue
            fc_emit = asint(s.get("fc_emitted"))
            fc_miss = asint(s.get("fc_deadline_misses"))
            miss_rate = (100.0 * fc_miss / (fc_emit + fc_miss)) if (fc_emit + fc_miss) > 0 else 0.0
            rows.append({
                "k":                k,
                "seed":             seed,
                "case":             s.get("case_name", case_label(k, seed)),
                "fc_emitted":       fc_emit,
                "fc_deadline_misses": fc_miss,
                "miss_rate_pct":    miss_rate,
                "plant_misses":     asint(s.get("plant_deadline_misses")),
                "fell":             asint(s.get("fell")),
                "fall_time_s":      asfloat(s.get("fall_time_s"), -1.0),
                "theta_max_abs":    asfloat(s.get("theta_max_abs")),
            })
    return rows


def wilson_ci(k_succ: int, n: int, z: float = 1.96) -> Tuple[float, float]:
    """Wilson score 95% CI for a binomial proportion. Avoids the
    pathological behaviour of Normal-approx CIs at p=0 and p=1."""
    if n == 0: return (0.0, 1.0)
    p_hat = k_succ / n
    denom = 1.0 + z*z / n
    centre = (p_hat + z*z / (2*n)) / denom
    half   = z * math.sqrt(p_hat * (1 - p_hat) / n + z*z / (4*n*n)) / denom
    return (max(0.0, centre - half), min(1.0, centre + half))


def summarise(raw: List[Dict], factors: List[float]) -> List[Dict]:
    """One row per k aggregating fall stats and confidence intervals."""
    summary = []
    for k in factors:
        seeds_at_k = [r for r in raw if r["k"] == k]
        n = len(seeds_at_k)
        if n == 0: continue
        n_fell = sum(1 for r in seeds_at_k if r["fell"] == 1)
        p_fell = n_fell / n
        lo, hi = wilson_ci(n_fell, n)

        # Fall times of seeds that did fall (for diagnostic context)
        fall_times = [r["fall_time_s"] for r in seeds_at_k if r["fell"] == 1]
        # Max |theta| only meaningful for non-fallen runs
        theta_max_pre = [r["theta_max_abs"] for r in seeds_at_k if r["fell"] == 0]
        miss_rates    = [r["miss_rate_pct"] for r in seeds_at_k]
        summary.append({
            "k":              k,
            "n_seeds":        n,
            "n_fell":         n_fell,
            "p_fell":         p_fell,
            "ci_lo":          lo,
            "ci_hi":          hi,
            "miss_rate_mean": sum(miss_rates) / n if miss_rates else float("nan"),
            "fall_time_min":  min(fall_times) if fall_times else float("nan"),
            "fall_time_med":  (sorted(fall_times)[len(fall_times)//2]
                               if fall_times else float("nan")),
            "theta_max_pre_med": (sorted(theta_max_pre)[len(theta_max_pre)//2]
                                  if theta_max_pre else float("nan")),
        })
    return summary


def find_threshold(summary: List[Dict], target_p: float) -> Optional[float]:
    """Linear-interp k where P(fall)=target_p between adjacent points.
    Returns None if the observed P(fall) never crosses target_p."""
    sorted_s = sorted(summary, key=lambda r: r["k"])
    for a, b in zip(sorted_s, sorted_s[1:]):
        if a["p_fell"] <= target_p <= b["p_fell"]:
            if b["p_fell"] == a["p_fell"]: return a["k"]
            frac = (target_p - a["p_fell"]) / (b["p_fell"] - a["p_fell"])
            return a["k"] + frac * (b["k"] - a["k"])
        if a["p_fell"] >= target_p >= b["p_fell"]:
            # Shouldn't happen in a monotonic sweep, but handle anyway
            if b["p_fell"] == a["p_fell"]: return a["k"]
            frac = (target_p - a["p_fell"]) / (b["p_fell"] - a["p_fell"])
            return a["k"] + frac * (b["k"] - a["k"])
    return None


def write_raw_csv(raw: List[Dict], path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    if not raw:
        print("  no raw rows"); return
    cols = list(raw[0].keys())
    with path.open("w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=cols); w.writeheader()
        for r in raw: w.writerow(r)
    print(f"  wrote {path}")


def write_summary_csv(summary: List[Dict], path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    if not summary:
        print("  no summary rows"); return
    cols = list(summary[0].keys())
    with path.open("w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=cols); w.writeheader()
        for r in summary: w.writerow(r)
    print(f"  wrote {path}")


def write_summary_md(summary: List[Dict],
                     thresholds: Dict[float, Optional[float]],
                     path: Path) -> None:
    L = ["# Probabilistic stress sweep summary", ""]
    L.append("**Boundary metrics** (linearly interpolated between bracketing k):")
    for tp in (0.05, 0.50, 0.95):
        v = thresholds.get(tp)
        if v is None:
            L.append(f"- k(P={tp:.2f}) = not crossed in swept range")
        else:
            L.append(f"- k(P={tp:.2f}) = **{v:.2f}**")
    L.append("")
    L.append("Per-k results (95% Wilson CI on P(fall)):")
    L.append("")
    L.append("| k | n | fell | P(fall) | 95% CI | miss% | fall t med (s) | max\\|θ\\| pre-fall |")
    L.append("|---:|---:|---:|---:|:---:|---:|---:|---:|")
    for r in summary:
        ci = f"[{r['ci_lo']:.2f}, {r['ci_hi']:.2f}]"
        ftm = f"{r['fall_time_med']:.2f}" if r['fall_time_med'] == r['fall_time_med'] else "—"
        tm  = f"{r['theta_max_pre_med']:.3f}" if r['theta_max_pre_med'] == r['theta_max_pre_med'] else "—"
        L.append(f"| {r['k']:g} | {r['n_seeds']} | {r['n_fell']} "
                 f"| {r['p_fell']:.2f} | {ci} | {r['miss_rate_mean']:.1f} "
                 f"| {ftm} | {tm} |")
    path.write_text("\n".join(L) + "\n", encoding="utf-8")
    print(f"  wrote {path}")


def make_plot(summary: List[Dict], thresholds: Dict[float, Optional[float]],
              path: Path) -> None:
    if not HAVE_MPL or not summary:
        return
    ks = [r["k"] for r in summary]
    ps = [r["p_fell"] for r in summary]
    los = [r["ci_lo"] for r in summary]
    his = [r["ci_hi"] for r in summary]

    fig, ax = plt.subplots(figsize=(7.0, 4.2))
    # Plot CI band
    ax.fill_between(ks, los, his, alpha=0.2, color="tab:red",
                    label="95% Wilson CI")
    ax.plot(ks, ps, "o-", color="tab:red", lw=1.5, label="P(fall)")
    ax.set_xlabel("Compute stress factor k")
    ax.set_ylabel("Fall probability P(fall | k)")
    ax.set_ylim(-0.02, 1.02)
    ax.grid(alpha=0.3)

    for tp, color in ((0.05, "tab:green"), (0.50, "tab:gray"),
                      (0.95, "tab:purple")):
        v = thresholds.get(tp)
        if v is None: continue
        ax.axvline(v, color=color, lw=1.0, alpha=0.7, linestyle="--",
                   label=f"k(P={tp:.2f}) = {v:.2f}")
        ax.axhline(tp, color=color, lw=0.5, alpha=0.3, linestyle=":")

    ax.legend(loc="lower right", fontsize=8, framealpha=0.9)
    ax.set_title("Fall probability vs. compute stress factor")
    fig.tight_layout()
    fig.savefig(path, dpi=130)
    plt.close(fig)
    print(f"  wrote {path}")


def main(argv: Optional[List[str]] = None) -> int:
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--duration", type=int, default=20000,
                    help="simulation duration in ms per run (default: 20000)")
    ap.add_argument("--factors", type=float, nargs="+", default=DEFAULT_FACTORS,
                    help=f"stress factors k to sweep (default: {DEFAULT_FACTORS})")
    ap.add_argument("--seeds-per-k", type=int, default=DEFAULT_SEEDS,
                    help=f"number of RNG seeds per k (default: {DEFAULT_SEEDS})")
    ap.add_argument("--seed-base", type=int, default=1000,
                    help="first seed; subsequent runs use seed_base+1, +2, ... "
                         "(default: 1000, distinct from any internal default)")
    ap.add_argument("--results-dir", default="results",
                    help="directory the simulator writes into (default: results)")
    ap.add_argument("--skip-runs", action="store_true",
                    help="skip simulator invocations; just collate existing")
    ap.add_argument("--timeout", type=int, default=900,
                    help="per-run timeout in seconds (default: 900)")

    ap.add_argument("--no-docker", action="store_true",
                    help="invoke binary directly instead of via docker compose")
    ap.add_argument("--exec-name", default="sensor_fusion",
                    help="executable name in ./build-docker/ (default: sensor_fusion)")
    ap.add_argument("--compose-service", default="systemc",
                    help="docker-compose service name (default: systemc)")
    ap.add_argument("--binary", default="./build-docker/sensor_fusion",
                    help="binary path in --no-docker mode")

    args = ap.parse_args(argv)

    cwd = Path.cwd()
    results_root = (cwd / args.results_dir).resolve()
    results_root.mkdir(parents=True, exist_ok=True)
    user_spec = get_user_spec()

    seeds = list(range(args.seed_base, args.seed_base + args.seeds_per_k))
    n_total = len(args.factors) * len(seeds)

    if not args.skip_runs:
        if args.no_docker:
            binary = Path(args.binary)
            if not binary.is_file():
                print(f"error: binary not found at {binary}", file=sys.stderr)
                return 1
            print(f"Sweeping {len(args.factors)} factors × {len(seeds)} seeds "
                  f"= {n_total} runs (native {binary})…")
        else:
            print(f"Sweeping {len(args.factors)} factors × {len(seeds)} seeds "
                  f"= {n_total} runs via docker compose…")

        completed = 0
        failures: List[Tuple[float, int, str]] = []
        for k in args.factors:
            for seed in seeds:
                completed += 1
                label = case_label(k, seed)
                sim_args = ["stress_k", str(args.duration), str(k), label,
                            str(seed)]
                cmd = ([str(Path(args.binary))] + sim_args if args.no_docker
                       else build_docker_cmd(args.exec_name, sim_args,
                                             args.compose_service, user_spec))
                tag = f"[{completed:>3}/{n_total}] k={k:5.2f} seed={seed}"
                print(f"  {tag}")
                ok, err = run_one(cmd, cwd, k, seed, args.timeout)
                if not ok:
                    print(f"    FAILED: {err}", file=sys.stderr)
                    failures.append((k, seed, err))

        if failures and len(failures) == n_total:
            print("\nAll runs failed; aborting before collation.",
                  file=sys.stderr)
            return 1
        if failures:
            print(f"\nWarning: {len(failures)} of {n_total} runs failed.",
                  file=sys.stderr)

    print("\nCollating raw results…")
    raw = collect_raw(results_root, args.factors, seeds)
    if not raw:
        print("error: no usable summary.csv files found.", file=sys.stderr)
        return 1
    write_raw_csv(raw, results_root / "prob_sweep_raw.csv")

    print("Computing P(fall) per k…")
    summary = summarise(raw, args.factors)
    thresholds = {tp: find_threshold(summary, tp)
                  for tp in (0.05, 0.50, 0.95)}

    write_summary_csv(summary, results_root / "prob_sweep_summary.csv")
    write_summary_md(summary, thresholds,
                     results_root / "prob_sweep_summary.md")
    make_plot(summary, thresholds, results_root / "prob_sweep.png")

    print()
    print("Probability thresholds:")
    for tp in (0.05, 0.50, 0.95):
        v = thresholds[tp]
        s = "not crossed" if v is None else f"k = {v:.2f}"
        print(f"  P(fall) = {tp:.2f}  →  {s}")
    print(f"\nSee results/prob_sweep_summary.md for the table\n"
          f"and results/prob_sweep.png for the curve.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
