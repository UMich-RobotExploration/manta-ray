# mantaray/ — C++ simulator

Ray-traced acoustic ranging for multi-agent underwater fleets. Consumes
environmental grids produced by
[`pymantaray/acoustics/`](../pymantaray/acoustics/README.md) and emits
the `.pfg` factor graph consumed by
[`pymantaray/localization/`](../pymantaray/localization/README.md).

## Overview

Three internal libraries plus an orchestrator and an output layer
(full architecture diagrams live in the design docs linked below):

- **[`rb/`](src/rb/rb.md)** — Rigid-body world. `RbWorld` owns `RobotI`
  subclasses (e.g. `CurrentDriftRobot`) each carrying `SensorI`
  subclasses. Motion is integrated on SE(3) using `manif`/Eigen.
- **[`acoustics/`](src/acoustics/acoustics.md)** — `AcousticsBuilder`
  configures a bellhop run; `BhContext` wraps `bhc::run`; `Arrival`
  parses the arrival table; `Grid2D` / `Grid3D` load `.npy` environment
  data.
- **[`sim/`](src/sim/sim.md)** — Orchestrator. `main.cpp` drives the
  event loop; `AcousticPairwiseRangeSystem` (APRS) executes ranging
  events and iteratively refines beam count until a direct-path arrival
  is found; `PfgWriter` (in `utils/`) emits the factor graph.

## Build

### Prerequisites

- CMake ≥ 3.20 (older ROS-bundled cmakes have failed at CUDA-arch
  detection — a fresh 3.31+ is recommended if you hit `nvcc` errors)
- Ninja
- C++20 compiler
- CUDA toolkit if `MANTA_CUDA=ON` (default). Set `MANTA_CUDA=OFF` to
  build against CPU-only `bellhopcxx` instead.
- `libbellhop{cuda,cxx}lib.so` (and `_debug` variants if you want debug
  builds) in `deps/lib/`. Bellhopcuda must be built separately — see
  [`A-New-BellHope/bellhopcuda`](https://github.com/A-New-BellHope/bellhopcuda/blob/main/doc/compilation.md)
  for compilation notes.

### Compile

```bash
cd mantaray
cmake -S . -B cmake-build-release -DCMAKE_BUILD_TYPE=Release -G Ninja
cmake --build cmake-build-release
```

### CPU-only fallback

```bash
cmake -S . -B cmake-build-release -DCMAKE_BUILD_TYPE=Release \
      -DMANTA_CUDA=OFF -G Ninja
```

CUDA runs at ~30–60 ms per source-receiver pair; the CPU path is
~120 ms. The CUDA toolchain is worth the setup only if your workflow
runs many pairings per iteration.

## Run

### Single scenario

Sim binaries must be executed from **inside the build tree** so that
`env_config_file` paths (resolved relative to the working directory at
execution time) find their targets.

```bash
cd cmake-build-release/src
./mantaray_core ../../sim_config/paper_beaufort_fleet_week_sim.json
```

> [!IMPORTANT]
> The `output_dir` field in a sim config is likewise CWD-relative. Runs
> that emit to `results/<name>/` write into
> `cmake-build-release/src/results/<name>/`.

### Batch

```bash
cd mantaray
./sim_config/run_batch.sh sim_config/lbl_sim.json \
                          sim_config/beaufort_floats_sim.json
```

## Config schema

Sim configs are JSON files under [`sim_config/`](sim_config/README.md).
The `"acoustics"` block controls ray tracing:

| Key | Type | Description |
|---|---|---|
| `num_beams` | int | Starting beam count |
| `max_beams` | int | Ceiling for iterative refinement |
| `beam_spread_deg` | float | Angular spread around the source-target bearing |
| `allow_multipath` | bool | Accept non-first-arrival paths |
| `tof_mode` | str | `"first_arrival"` or `"expected"` |

Iterative beam refinement doubles the beam count from `num_beams` up to
`max_beams` looking for a direct-path arrival (zero top/bottom
bounces). See [`sim.md`](src/sim/sim.md) for the full ping-lifecycle
sequence diagram and the invariants around the pre-allocated ray
arrays.

Output `.pfg` files are conventionally named `output.pfg`; sensor CSVs
sit next to them.

## Tests

```bash
cd cmake-build-release
ctest --output-on-failure
# Run a single test binary:
./tests/test_PfgWriter
```

## Documentation

- Full C++ Doxygen documentation:
  [umich-robotexploration.github.io/manta-ray](https://umich-robotexploration.github.io/manta-ray/)
- Local generation: `./create_docs.sh` writes to `docs/html/`; open
  `docs/html/index.html`.
- Architecture design docs (Mermaid diagrams, class overviews):
  - [Rigid-body world](src/rb/rb.md)
  - [Acoustics + environment grids](src/acoustics/acoustics.md)
  - [Simulator orchestrator + iterative beam refinement](src/sim/sim.md)
- Data-layout contract: [`data/README.md`](data/README.md)
- Sim-config directory: [`sim_config/README.md`](sim_config/README.md)

## Conventions

- Serializers and writers live in [`src/utils/`](src/utils/), not
  `src/sim/` (e.g. `PfgWriter.cpp`).
- `deps/fmt_eigen/fmt_eigen.h` is vendored from upstream — do not
  modify it.
- Bellhop's ray writeout segfaults in 3D ray mode; use `writeenv` only.
- Sensor and config objects take their configuration via constructor,
  not via setters after construction.
