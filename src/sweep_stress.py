#!/usr/bin/env python3
"""
sweep_stress.py — drive the SystemC sensor-fusion simulator across a
sweep of compute stress factors and produce the stability-boundary plot.

Designed for the project's standard Docker workflow: each run is invoked
as

    docker compose run --rm --user "<uid>:<gid>" systemc \\
        bash -lc './build-docker/sensor_fusion stress_k <duration_ms> <k> <label>'

Results land in ./results/<label>/ on the host (via the docker-compose
bind mount). After the sweep finishes the script collates every
summary.csv into one wide sweep_results.csv and produces sweep.png.

Run from the project root (the same directory as docker-compose.yml).

Usage:
    python3 sweep_stress.py                                # default sweep, Docker
    python3 sweep_stress.py --duration 20000
    python3 sweep_stress.py --factors 1.0 1.5 2.0 3.0 5.0
    python3 sweep_stress.py --no-docker --binary ./build/sensor_fusion
                                                           # native binary instead
    python3 sweep_stress.py --skip-runs                    # re-plot only

If you've changed the executable name, pass --exec-name (Docker mode) or
--binary (native mode).
"""

from __future__ import annotations

import argparse
import csv
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


DEFAULT_FACTORS = [1.0, 1.25, 1.5, 1.75, 2.0, 2.5, 3.0, 4.0, 5.0, 7.0, 10.0]


def fmt_k(k: float) -> str:
    """Folder-friendly suffix: 1.0 → '1.0', 2.5 → '2.5'."""
    return f"{k:g}"


def case_label(k: float) -> str:
    return f"stress_k{fmt_k(k)}"


def get_user_spec() -> str:
    """Mirror `$(id -u):$(id -g)` on POSIX hosts; on Windows return ''
    so the --user flag is omitted."""
    if os.name == "nt":
        return ""
    try:
        return f"{os.getuid()}:{os.getgid()}"
    except AttributeError:
        return ""


def build_docker_cmd(exec_name: str, sim_args: List[str],
                     compose_service: str, user_spec: str) -> List[str]:
    """Compose the docker-compose run command for one simulator invocation."""
    inner = "./build-docker/" + exec_name + " " + " ".join(shlex.quote(a) for a in sim_args)
    cmd = ["docker", "compose", "run", "--rm"]
    if user_spec:
        cmd += ["--user", user_spec]
    cmd += [compose_service, "bash", "-lc", inner]
    return cmd


def run_one(cmd: List[str], cwd: Path, k: float,
            timeout_s: int) -> Tuple[bool, str]:
    """Run a single simulator invocation. Returns (success, stderr_tail)."""
    print(f"  [k={k:5.2f}] $ {' '.join(shlex.quote(c) for c in cmd)}")
    try:
        proc = subprocess.run(cmd, cwd=cwd, capture_output=True, text=True,
                              timeout=timeout_s)
    except subprocess.TimeoutExpired:
        return False, f"TIMEOUT after {timeout_s} s"
    except FileNotFoundError as e:
        return False, f"command not found: {e}"
    if proc.returncode != 0:
        tail = "\n".join(proc.stderr.strip().splitlines()[-5:])
        return False, f"rc={proc.returncode}\n{tail}"
    return True, ""


def parse_summary(case_dir: Path) -> Dict[str, str]:
    out: Dict[str, str] = {}
    p = case_dir / "summary.csv"
    if not p.is_file():
        return out
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


def collect(results_root: Path, factors: List[float]) -> List[Dict]:
    rows = []
    for k in factors:
        d = results_root / case_label(k)
        if not d.is_dir():
            print(f"  [k={k:5.2f}] no result directory at {d}, skipping",
                  file=sys.stderr)
            continue
        s = parse_summary(d)
        if not s:
            print(f"  [k={k:5.2f}] no summary.csv at {d}", file=sys.stderr)
            continue
        fc_emit  = asint(s.get("fc_emitted"))
        fc_miss  = asint(s.get("fc_deadline_misses"))
        miss_rate = (100.0 * fc_miss / (fc_emit + fc_miss)) if (fc_emit + fc_miss) > 0 else 0.0
        rows.append({
            "k":                k,
            "case":             s.get("case_name", case_label(k)),
            "duration_s":       asfloat(s.get("duration_s")),
            "fc_emitted":       fc_emit,
            "fc_deadline_misses": fc_miss,
            "miss_rate_pct":    miss_rate,
            "plant_misses":     asint(s.get("plant_deadline_misses")),
            "imu_drops":        asint(s.get("imu_drops")),
            "fc_drops":         asint(s.get("fc_drops")),
            "fell":             asint(s.get("fell")),
            "fall_time_s":      asfloat(s.get("fall_time_s"), -1.0),
            "theta_max_abs":    asfloat(s.get("theta_max_abs")),
        })
    return rows


def write_csv(rows: List[Dict], out_path: Path) -> None:
    out_path.parent.mkdir(parents=True, exist_ok=True)
    if not rows:
        print("  no rows to write", file=sys.stderr); return
    cols = ["k", "case", "duration_s", "fc_emitted", "fc_deadline_misses",
            "miss_rate_pct", "plant_misses", "imu_drops", "fc_drops",
            "fell", "fall_time_s", "theta_max_abs"]
    with out_path.open("w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=cols)
        w.writeheader()
        for r in rows: w.writerow(r)
    print(f"  wrote {out_path}")


def find_boundary(rows: List[Dict]) -> Optional[float]:
    """Smallest k for which fell==1. Returns None if nothing fell."""
    fallen = sorted([r["k"] for r in rows if r.get("fell") == 1])
    return fallen[0] if fallen else None


def write_md(rows: List[Dict], boundary: Optional[float],
             out_path: Path) -> None:
    L = ["# Stress sweep summary", ""]
    if boundary is not None:
        L.append(f"**Stability boundary:** the controller first failed at "
                 f"k = {boundary:g} (compute scaled {boundary:g}× the baseline "
                 "stress configuration).")
    else:
        L.append("**Stability boundary:** the controller did not fail across "
                 "the swept range.")
    L.append("")
    L.append("| k | FC misses | miss-rate (%) | plant misses | max \\|θ\\| | fell | fall t (s) |")
    L.append("|---:|---:|---:|---:|---:|:---:|---:|")
    for r in rows:
        fell_mark = "**YES**" if r["fell"] else "no"
        ft = f"{r['fall_time_s']:.2f}" if r["fall_time_s"] >= 0 else "—"
        L.append(f"| {r['k']:g} | {r['fc_deadline_misses']} "
                 f"| {r['miss_rate_pct']:.2f} | {r['plant_misses']} "
                 f"| {r['theta_max_abs']:.3f} | {fell_mark} | {ft} |")
    out_path.write_text("\n".join(L) + "\n", encoding="utf-8")
    print(f"  wrote {out_path}")


def make_plot(rows: List[Dict], boundary: Optional[float],
              out_path: Path) -> None:
    if not HAVE_MPL or not rows:
        return
    ks       = [r["k"] for r in rows]
    miss_pct = [r["miss_rate_pct"] for r in rows]
    th_max   = [r["theta_max_abs"] for r in rows]

    fig, ax_left = plt.subplots(figsize=(7.0, 4.2))
    ax_left.set_xlabel("Compute stress factor k")
    ax_left.set_ylabel("FC deadline-miss rate (%)", color="tab:blue")
    ax_left.plot(ks, miss_pct, "o-", color="tab:blue", lw=1.5,
                 label="FC miss rate")
    ax_left.tick_params(axis="y", labelcolor="tab:blue")
    ax_left.grid(alpha=0.3)

    ax_right = ax_left.twinx()
    ax_right.set_ylabel("max |θ| (rad)", color="tab:red")
    ax_right.plot(ks, th_max, "s--", color="tab:red", lw=1.5,
                  label="max |θ|")
    ax_right.axhline(0.6, color="tab:red", lw=0.7, alpha=0.5,
                     linestyle=":", label="fall threshold (0.6 rad)")
    ax_right.tick_params(axis="y", labelcolor="tab:red")

    if boundary is not None:
        ax_left.axvline(boundary, color="black", lw=1.0, alpha=0.7,
                        linestyle="--", label=f"fall boundary k={boundary:g}")
        ax_left.annotate(f"k={boundary:g}", xy=(boundary, max(miss_pct)),
                         xytext=(5, 0), textcoords="offset points",
                         fontsize=9)

    fig.suptitle("Stress sweep: deadline misses and pendulum excursion vs. k",
                 y=0.99)

    lines, labels = ax_left.get_legend_handles_labels()
    lines2, labels2 = ax_right.get_legend_handles_labels()
    ax_left.legend(lines + lines2, labels + labels2, loc="upper left",
                   fontsize=8, framealpha=0.9)

    fig.tight_layout()
    fig.savefig(out_path, dpi=130)
    plt.close(fig)
    print(f"  wrote {out_path}")


def main(argv: Optional[List[str]] = None) -> int:
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--duration", type=int, default=20000,
                    help="simulation duration in ms per run (default: 20000)")
    ap.add_argument("--factors", type=float, nargs="+", default=DEFAULT_FACTORS,
                    help=f"stress factors k to sweep (default: {DEFAULT_FACTORS})")
    ap.add_argument("--results-dir", default="results",
                    help="directory the simulator writes into (default: results)")
    ap.add_argument("--skip-runs", action="store_true",
                    help="skip simulator invocations; just collate existing results")
    ap.add_argument("--timeout", type=int, default=900,
                    help="per-run timeout in seconds, including Docker startup "
                         "(default: 900)")

    # Docker-mode (default) options:
    ap.add_argument("--no-docker", action="store_true",
                    help="invoke the binary directly instead of through "
                         "`docker compose run`. Use --binary to point at the "
                         "native executable.")
    ap.add_argument("--exec-name", default="sensor_fusion",
                    help="name of the executable inside ./build-docker/ "
                         "(Docker mode only; default: sensor_fusion)")
    ap.add_argument("--compose-service", default="systemc",
                    help="docker-compose service name (default: systemc)")

    # Native-mode options:
    ap.add_argument("--binary", default="./build-docker/sensor_fusion",
                    help="path to the simulator binary in --no-docker mode "
                         "(default: ./build-docker/sensor_fusion)")

    args = ap.parse_args(argv)

    cwd = Path.cwd()
    results_root = (cwd / args.results_dir).resolve()
    results_root.mkdir(parents=True, exist_ok=True)

    user_spec = get_user_spec()

    if not args.skip_runs:
        if args.no_docker:
            binary = Path(args.binary)
            if not binary.is_file():
                print(f"error: binary not found at {binary}\n"
                      "Build the project first or pass --skip-runs to re-plot.",
                      file=sys.stderr)
                return 1
            print(f"Sweeping {len(args.factors)} factors (native binary {binary})…")
        else:
            print(f"Sweeping {len(args.factors)} factors via "
                  f"docker compose ({args.compose_service})…")
            print("(make sure you've already done: docker compose build && "
                  "cmake --build inside the container)")

        failures: List[Tuple[float, str]] = []
        for k in args.factors:
            label = case_label(k)
            sim_args = ["stress_k", str(args.duration), str(k), label]

            if args.no_docker:
                cmd = [str(Path(args.binary))] + sim_args
            else:
                cmd = build_docker_cmd(args.exec_name, sim_args,
                                       args.compose_service, user_spec)

            ok, err = run_one(cmd, cwd, k, args.timeout)
            if not ok:
                print(f"  [k={k:5.2f}] FAILED: {err}", file=sys.stderr)
                failures.append((k, err))
                continue

            case_dir = results_root / label
            if not (case_dir / "summary.csv").is_file():
                msg = (f"no summary.csv at {case_dir} — check that "
                       "Telemetry::end_of_simulation() and sc_stop() are wired "
                       "up in your build, and that the case_name override "
                       "worked (4th argv).")
                print(f"  [k={k:5.2f}] {msg}", file=sys.stderr)
                failures.append((k, msg))

        if failures and len(failures) == len(args.factors):
            print("\nAll runs failed; aborting before collation.",
                  file=sys.stderr)
            return 1

    print("\nCollating results…")
    rows = collect(results_root, args.factors)
    if not rows:
        print("error: no usable summary.csv files found.", file=sys.stderr)
        return 1

    boundary = find_boundary(rows)
    out_csv  = results_root / "sweep_results.csv"
    out_md   = results_root / "sweep_results.md"
    out_png  = results_root / "sweep.png"

    write_csv(rows, out_csv)
    write_md(rows, boundary, out_md)
    make_plot(rows, boundary, out_png)

    print()
    if boundary is not None:
        print(f"Stability boundary: k = {boundary:g}  "
              f"(controller falls at {boundary:g}× nominal stress compute).")
    else:
        print("Stability boundary: not reached in the swept range.")
    print(f"\nSee {out_md} for the table and {out_png} for the plot.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
