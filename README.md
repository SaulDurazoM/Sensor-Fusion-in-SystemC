# SystemC Sensor Fusion Pipeline

A small SystemC-based sensor-fusion pipeline that simulates IMU and GPS producers, a fusion stage, a control stage, and a plant response block. The project is built with CMake and is intended to be run inside Docker with a local workspace mount.

## Overview

This project models a simple real-time sensor-fusion flow:

- **IMU source** generates inertial samples.
- **GPS source** generates position and velocity samples.
- **Fusion block** consumes the latest available IMU and GPS data and emits a fused state.
- **Control block** consumes fused state data and emits control commands.
- **Plant block** consumes control commands and updates a simple simulated plant state.
- **Telemetry** records runtime events, FIFO occupancy, delay metrics, deadline misses, and summary statistics.

The executable is named `sensor_fusion` and supports three runtime cases:

- `normal`
- `stress`
- `burst`

An optional second argument lets you override the simulation duration in **milliseconds**.

## Repository Layout

```text
sensor_fusion_systemc/
├── CMakeLists.txt
├── Dockerfile
├── docker-compose.yml
├── include/
│   ├── config.h
│   ├── telemetry.h
│   ├── top.h
│   └── types.h
└── src/
    ├── main.cpp
    ├── telemetry.cpp
    └── top.cpp
```

## Requirements

The recommended workflow uses Docker, so the only required host tools are:

- Docker
- Docker Compose plugin (`docker compose`)

The container installs the build toolchain and builds SystemC inside the image.

## Recommended Build and Run Workflow

### 1. Clone the repository

```bash
git clone <your-repo-url>
cd sensor_fusion_systemc
```

### 2. Build the Docker image

```bash
docker compose build --no-cache
```

`--no-cache` is recommended after changing the Dockerfile or SystemC build options.

### 3. Build the project inside the container

```bash
docker compose run --rm --user "$(id -u):$(id -g)" systemc bash -lc '
cmake -S . -B build-docker &&
cmake --build build-docker -j
'
```

### 4. Run the simulation

Run the default case:

```bash
docker compose run --rm --user "$(id -u):$(id -g)" systemc bash -lc './build-docker/sensor_fusion normal'
```

Run the stress and burst cases:

```bash
docker compose run --rm --user "$(id -u):$(id -g)" systemc bash -lc './build-docker/sensor_fusion stress'
docker compose run --rm --user "$(id -u):$(id -g)" systemc bash -lc './build-docker/sensor_fusion burst'
```

Run with a custom duration in milliseconds:

```bash
docker compose run --rm --user "$(id -u):$(id -g)" systemc bash -lc './build-docker/sensor_fusion normal 5000'
```

## Simulation Cases

The project defines three preset configurations:

### `normal`
Baseline timing and default sensor periods.

### `stress`
Adds extra compute time to the fusion and control blocks to create a more demanding timing scenario.

### `burst`
Enables a burst window with faster IMU and GPS production rates to exercise FIFO and latency behavior.

## Output Files

After each run, telemetry is written to:

```text
results/<case_name>/
```

The generated files include:

- `event_log.csv`
- `fifo_usage.csv`
- `control_log.csv`
- `delay_log.csv`
- `fifo_overflows.csv`
- `summary.csv`

These files capture:

- block-level events
- FIFO occupancy over time
- control execution timing and deadline misses
- sensor-to-command and fusion-to-command delay metrics
- FIFO overflow counts
- aggregate summary statistics

## Example Successful Run

A successful `normal` run prints a summary like this:

```text
Starting SystemC sensor-fusion pipeline
Case: normal
Simulation duration: 10 s

=== CASE: normal ===
Control cycles              : 10000
Control deadline misses     : 0
Sensor->command median (ms) : 2.100000
Fusion->command median (ms) : 1.020000
FIFO overflows:
  none
Simulation finished at 10 s
```

## Troubleshooting

### Build fails with old C++ standard errors
If you see errors involving `std::uint64_t`, `enum class`, `std::stod`, or `std::filesystem`, your build is probably not using a modern enough C++ standard. Make sure the project and SystemC are both built with C++17.

### Linker error mentioning `sc_api_version_2_3_4_cxx201703L`
This usually means the application and the SystemC library were built with different C++ standards. Rebuild the Docker image with `--no-cache` after updating the Dockerfile so SystemC is rebuilt with the same standard as the application.

### `I have no name!` inside the container
This is harmless. It happens because the container is run with your host UID/GID, but the container does not have a matching username entry.

### Compose warning about `version`
If Docker Compose warns that the `version` field is obsolete, remove the top-level `version:` line from `docker-compose.yml`.

## Suggested CMake Configuration

For a stable Docker-based workflow, use a fixed C++ standard in `CMakeLists.txt`:

```cmake
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
```

## Suggested Dockerfile Note

When building SystemC inside Docker, it should be compiled with the same C++ standard as the application. For example:

```dockerfile
cmake .. \
  -DCMAKE_INSTALL_PREFIX=${SYSTEMC_HOME} \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_CXX_STANDARD=17 \
  -DCMAKE_CXX_STANDARD_REQUIRED=ON
```

## License

Add the license you intend to use here.

## Author

Add your name and contact information here.
