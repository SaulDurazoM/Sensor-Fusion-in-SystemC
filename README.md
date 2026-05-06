# SystemC Cart-Pole Sensor-Fusion Pipeline

> Companion code for the paper *SEND-HELP: Sensor-Fusion ENgine for Dynamic
> Hardware Evaluation of Latency in a Pendulum* (Wilson, Durazo Martinez,
> Romero-Lozano, Payan; ECE 576B, University of Arizona). The paper reports
> the analysis; this README covers the build, runtime, and reproduction
> commands.

A SystemC transaction-level model of an inverted-pendulum-on-a-cart stabiliser,
instrumented to study how compute-time pressure on the controller degrades
real-time performance and, past a critical point, causes the pendulum to fall.
The build runs inside Docker; two Python helper scripts drive parametric stress
sweeps that turn the simulator into a tool for finding the controller's
stability boundary.

## What the simulation models

The plant is a non-linear cart-pole integrated by RK4 at 2 kHz. An IMU
(1 kHz, gyro + body-frame accelerometer) and a wheel encoder (100 Hz, quantised
position) sample plant state with sensor noise and hand samples to a single
`FusionControl` block that runs at 1 kHz. `FusionControl` performs:

- a complementary filter for `θ` (gyro integration corrected by accelerometer)
  and a model-based `θ̈` estimate solved from the 2×2 mass-matrix EOM,
- a complementary filter for `z` (IMU integration corrected by encoder finite
  difference), and
- a cascaded PID: an outer loop (`z` → desired tilt `θ_setpoint`) feeding an
  inner loop (`θ` → cart force).

The resulting force command is consumed by the plant via a FIFO. A scheduled
`Disturbance` module injects torque impulses on a parallel FIFO. A `Telemetry`
module snapshots FIFO occupancy at 100 Hz and writes a summary at end-of-sim
covering throughput, drops, deadline misses, peak pendulum angle, and a
fall flag.

Every block's per-cycle compute time is drawn from a Gaussian with a
configurable mean and standard deviation. The `stress_k` configuration scales
the controller's compute distribution by a factor `k` while leaving sensor and
plant times unchanged — controller firmware is the variable under study; sensor
hardware and physics are not. The sweep scripts walk `k` upward and locate the
boundary at which the controller starts missing 1 ms deadlines often enough
that the pendulum falls.

## Architecture and timing

| Module          | Period   | What it does                                                    | FIFOs                          |
|-----------------|----------|------------------------------------------------------------------|---------------------------------|
| `Plant`         | 500 µs   | RK4 integration of non-linear cart-pole; consumes force commands | `control_fifo`, `disturbance_fifo` (in) |
| `IMUSensor`     | 1 ms     | Gyro + body-frame accel with Gaussian noise                      | `imu_fifo` (out)                |
| `EncoderSensor` | 10 ms    | Cart-position quantisation                                        | `enc_fifo` (out)                |
| `FusionControl` | 1 ms     | Comp. filter + cascade PID + force saturation                    | `imu_fifo`/`enc_fifo` (in), `control_fifo` (out) |
| `Disturbance`   | event    | Plays back a CSV schedule of torque impulses                     | `disturbance_fifo` (out)        |
| `Telemetry`     | 10 ms    | FIFO snapshots; final summary at end-of-sim                      | reads all FIFOs + module counters |

All FIFOs default to depth 16. Sensors read plant ground truth through a
`const PlantState*` provided by `Plant::get_state()`, so module construction
order in `Top` is fixed (`Plant` before sensors, `Telemetry` last).

## Repository layout

```text
sensor_fusion_systemc/
├── .devcontainer/             # VS Code Dev Containers config (optional alternate workflow)
├── .gitignore
├── CMakeLists.txt
├── Dockerfile
├── docker-compose.yml
├── README.md
├── sweep_stress.py            # deterministic stability-boundary sweep
├── sweep_stress_prob.py       # probabilistic Monte-Carlo sweep
├── include/
│   ├── physics_config.h       # plant parameters, initial conditions, disturbance schedule
│   ├── sim_config.h           # timing, FIFO depths, gains, compute distributions, case factories
│   ├── top.h                  # all module declarations + Top wiring class
│   └── types.h                # IMUSample, EncoderSample, ControlCommand, DisturbanceTorque, PlantState
├── src/
│   ├── main.cpp               # CLI parsing, disturbance loading, sc_main
│   └── top.cpp                # module implementations (Telemetry lives here too)
└── resources/
    └── disturbances.csv       # input: scheduled torque impulses (see "Disturbance schedule")
```

## Requirements

The recommended workflow uses Docker, so the only host tools required are:

- Docker
- Docker Compose plugin (`docker compose`)
- Python 3.8+ with `matplotlib` (only if you intend to run the sweep scripts on
  the host)

The Docker image installs the toolchain (g++, CMake, Eigen) and builds
SystemC 2.3.4 from source into `/usr/local/systemc`.

## Build and run via Docker

VS Code users can alternatively open the repository through the Dev Containers
extension — the `.devcontainer/` config wraps the same image used by
`docker compose`, so the rest of this section applies inside that environment
too (skip the `docker compose run` prefix on every command).

### 1. Build the image

```bash
docker compose build --no-cache
```

`--no-cache` is recommended after editing the Dockerfile or bumping the
SystemC version.

### 2. Build the project inside the container

```bash
docker compose run --rm --user "$(id -u):$(id -g)" systemc bash -lc '
cmake -S . -B build-docker &&
cmake --build build-docker -j
'
```

The `--user` flag makes any files written into the bind-mounted workspace
(build artefacts, `results/`) owned by your host user rather than root.

### 3. Run the simulation

The executable is `sensor_fusion`. Its full CLI is

```text
sensor_fusion [case] [duration_ms] [stress_k] [output_label] [seed]
```

| Arg            | Meaning                                                                                  | Default                |
|----------------|------------------------------------------------------------------------------------------|------------------------|
| `case`         | one of `normal`, `stress`, `burst`, `stress_k`                                           | `normal`               |
| `duration_ms`  | total simulated time in milliseconds                                                     | 20000 (from `SimConfig`) |
| `stress_k`     | compute-scale factor — only consulted when `case == stress_k`                            | 1.0                    |
| `output_label` | overrides `case_name` and therefore the `results/<label>/` subdirectory                  | empty (uses case name) |
| `seed`         | master RNG seed; modules derive their own seeds as `seed+1`, `seed+2`, …                 | 42                     |

Examples (Docker):

```bash
# Three baseline cases at default duration:
docker compose run --rm --user "$(id -u):$(id -g)" systemc \
    bash -lc './build-docker/sensor_fusion normal'
docker compose run --rm --user "$(id -u):$(id -g)" systemc \
    bash -lc './build-docker/sensor_fusion stress'
docker compose run --rm --user "$(id -u):$(id -g)" systemc \
    bash -lc './build-docker/sensor_fusion burst'

# Stress-by-factor: 2× nominal stress compute, 30 s, custom output dir, seed 7:
docker compose run --rm --user "$(id -u):$(id -g)" systemc \
    bash -lc './build-docker/sensor_fusion stress_k 30000 2.0 stress_k2.0 7'
```

## Simulation cases

| Case        | What it changes                                                                                                                                                                       |
|-------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| `normal`    | Baseline timing and compute distributions. Pendulum stays upright across the full duration.                                                                                           |
| `stress`    | Heavier and higher-variance compute distributions on `FusionControl`, `Plant`, IMU, and encoder. `FusionControl` mean rises to 600 µs (1.1 ms when disturbed) — already pressuring the 1 ms control period. |
| `burst`     | Faster IMU (2 kHz) and encoder (200 Hz) production, FIFO depths shrunk to 4. Compute times stay nominal — this isolates communication stress from CPU stress.                          |
| `stress_k`  | Starts from `stress` then multiplies all four `FusionControl` compute parameters by the CLI factor `k`. Plant and sensor compute are *not* scaled. This is what the sweeps walk over. |

## Output files

Each run writes its CSVs to `results/<case_name>/` (or
`results/<output_label>/` when the 4th arg is supplied). Files written:

| File                  | Schema                                                                                                              |
|-----------------------|---------------------------------------------------------------------------------------------------------------------|
| `imu.csv`             | `time_s,seq,omega,a_x_prime,a_y_prime,disturbed`                                                                    |
| `encoder.csv`         | `time_s,seq,z_quantized`                                                                                            |
| `control.csv`         | per-tick estimator state + outer/inner PID terms + force; full schema in `FusionControl` constructor                |
| `plant_state.csv`     | `time_s,theta,theta_dot,theta_ddot,z,z_dot,z_ddot,force_applied,tau_disturbance`                                    |
| `telemetry_fifo.csv`  | 10 ms snapshots of all four FIFO depths plus running emitted/drop/deadline-miss counters                            |
| `summary.csv`         | end-of-sim aggregate (see below)                                                                                    |

`summary.csv` has rows `metric,value` covering: `case_name`, `duration_s`,
`imu_emitted`, `imu_drops`, `enc_emitted`, `enc_drops`, `fc_emitted`,
`fc_drops`, `fc_deadline_misses`, `plant_cmds_consumed`,
`plant_deadline_misses`, `fell` (0/1), `fall_time_s` (-1 if never fell), and
`theta_max_abs`. The sweep scripts read this file.

## Disturbance schedule

`Disturbance` plays back events from a CSV at the path given by
`SimConfig::disturbance_csv` (default `resources/disturbances.csv`). The format
is a header row plus one event per line:

```csv
time_s,torque_nm,duration_s
2.0,0.5,0.05
5.0,-0.7,0.05
8.0,1.0,0.05
12.0,-0.5,0.10
16.0,0.8,0.05
```

Each event applies `torque_nm` as a constant pivot torque starting at `time_s`
for `duration_s`, then turns it off. While `state_.tau_disturbance` is non-zero
the IMU sets a sticky `disturbed` flag on its samples, which switches
`FusionControl` to its higher-mean `fc_disturbed_*` compute distribution — that
is the path the `stress` and `stress_k` cases stress hardest.

If the CSV is missing the simulator prints a warning and continues with no
disturbances; the controller will look perfectly stable in that case, which is
generally not what you want.

## Stress sweeps

Two Python scripts on the project root drive the simulator across a range of
`stress_k` values and post-process the resulting `summary.csv` files. Both
default to invoking the binary through `docker compose run`, but accept
`--no-docker --binary <path>` to call a native build directly. Both also
support `--skip-runs` to re-collate and re-plot without re-simulating.

### `sweep_stress.py` — deterministic boundary

For each `k` in `--factors` the script runs one simulation at fixed seed,
collects all `summary.csv`s, and produces

```text
results/sweep_results.csv     # one wide row per k
results/sweep_results.md      # markdown table of misses, max|θ|, fell, fall time
results/sweep.png             # FC miss-rate (left axis) and max|θ| (right axis) vs k,
                              # with the fall-threshold line and the boundary k marked
```

The "stability boundary" reported is simply the smallest `k` at which the
deterministic run fell. Useful as a fast first pass before committing to the
much slower probabilistic sweep.

```bash
# Default factors, 20 s per run, Docker:
python3 sweep_stress.py

# Custom range and duration:
python3 sweep_stress.py --duration 30000 --factors 1.0 1.5 2.0 3.0 5.0

# Native binary path (no Docker):
python3 sweep_stress.py --no-docker --binary ./build/sensor_fusion

# Re-plot from existing results/ subdirectories without re-running:
python3 sweep_stress.py --skip-runs
```

### `sweep_stress_prob.py` — probabilistic boundary

For each `k` in `--factors` the script runs `--seeds-per-k` simulations with
distinct seeds, then estimates `P(fall | k)` with a 95 % Wilson confidence
interval per point. Outputs:

```text
results/prob_sweep_raw.csv      # one row per (k, seed)
results/prob_sweep_summary.csv  # one row per k with P(fall) + CI bounds
results/prob_sweep_summary.md   # markdown table + boundary metrics
results/prob_sweep.png          # P(fall) curve with 95% CI band
```

The summary reports three boundary metrics, linearly interpolated between
adjacent `k` points: `k(P=0.05)` (safe operating point — at most 5% of seeds
fall), `k(P=0.50)` (median boundary), and `k(P=0.95)` (catastrophic regime).

```bash
# Default sweep (12 factors × 10 seeds = 120 runs, ~12 min via Docker):
python3 sweep_stress_prob.py

# Targeted sweep around a known boundary:
python3 sweep_stress_prob.py --factors 14 16 18 20 22 24 --seeds-per-k 20

# Re-aggregate from existing results without re-simulating:
python3 sweep_stress_prob.py --skip-runs
```

`--seed-base` (default 1000) sets the first seed; subsequent runs use
`seed_base+1, seed_base+2, …` so the sweep is reproducible run-to-run.

## Reproducing the paper

The fixed-scenario tables (sensing-to-command latency, throughput, closed-loop
performance) come from running each baseline case at its default 20 s
duration:

```bash
docker compose run --rm --user "$(id -u):$(id -g)" systemc \
    bash -lc './build-docker/sensor_fusion normal'
docker compose run --rm --user "$(id -u):$(id -g)" systemc \
    bash -lc './build-docker/sensor_fusion stress'
docker compose run --rm --user "$(id -u):$(id -g)" systemc \
    bash -lc './build-docker/sensor_fusion burst'
```

The probabilistic sweep (Table IV and Fig. 3 in the paper) uses **50 seeds
per `k`** over a denser factor list around the transition region than the
script's default — neither is the default, so both flags must be passed:

```bash
python3 sweep_stress_prob.py --seeds-per-k 50 \
    --factors 8 9 10 12 13 15 17 19 21 23 25
```

That's 550 simulator runs and takes roughly an hour through Docker on a
typical machine. The script writes `results/prob_sweep_raw.csv`,
`results/prob_sweep_summary.csv`, `results/prob_sweep_summary.md`, and
`results/prob_sweep.png`. The paper's reported boundaries
(`k(P=0.05) ≈ 9.13`, `k(P=0.50) = 13.00`, `k(P=0.95) ≈ 20.83`) appear at the
top of `prob_sweep_summary.md`. Exact numbers will vary slightly between
machines because the sweep depends on RNG seeds the simulator derives from
`--seed-base` — re-running with the same `--seed-base` reproduces the same
boundary values.

## Native build (optional)

If you would rather skip Docker, install SystemC 2.3.4 and Eigen 3 on the host
and point CMake at the SystemC install:

```bash
cmake -S . -B build -DCMAKE_PREFIX_PATH=/usr/local/systemc
cmake --build build -j
./build/sensor_fusion normal
```

The sweep scripts pick up a native build with
`--no-docker --binary ./build/sensor_fusion`.

## Example output

A baseline `normal` run (default 20 s) prints, on stdout:

```text
Loaded 5 disturbance events from 'resources/disturbances.csv'
Case:     normal
Duration: 20 s
Seed:     42

=== Telemetry summary (normal) ===
  IMU:   emitted=20000  drops=0
  ENC:   emitted=2000   drops=0
  FC:    emitted=20000  drops=0  deadline_misses=0
  PLANT: consumed=20000 deadline_misses=0
  FALL:  fell=no  fall_time_s=-1  max|theta|=0.041 rad
Simulation finished at 20 s
```

(Exact counts depend on duration, seed, and disturbance schedule. The numbers
above are illustrative for a 20 s run.) The `stress` case typically shows a
non-zero `fc_deadline_misses` count and a larger peak `|θ|` while still
recovering; aggressive `stress_k` runs eventually flip `fell=YES` and report a
finite `fall_time_s`.
