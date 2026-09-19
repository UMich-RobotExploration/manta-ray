# Simulation Configurations

JSON configs consumed by `mantaray_core`. Two kinds of file live in
this directory:

- **Sim configs** — top-level runs. Declare robots, sensors, mission
  duration, and point at an env config via `env_config_file`. Passed as
  the sole argument to `mantaray_core` (see the [subproject
  README](../README.md)).
- **Env configs** — environment-only bundles referenced by `env_config_file`
  from a sim config. They name the region's bathymetry, SSP, and current
  grids under `mantaray/data/<region>/`. Not runnable on their own.

Paths inside a sim config are resolved relative to `mantaray_core`'s
working directory at execution time — that is why sim runs are launched
from inside `cmake-build-release/src/`.

## Paper runs

Configs prefixed `paper_` reproduce the runs used in the accompanying
paper (see the [root README](../../README.md)). Do not rename or edit
these in place; copy to a new filename first.

| File | Region | Robots | Notes |
|---|---|---|---|
| `paper_beaufort_fleet_week_sim.json` | Beaufort Sea | 11 | Fleet-week run, cold-cap halocline |
| `paper_fram_strait_fleet_week_sim.json` | Fram Strait | 11 | Fleet-week run, Atlantic-polar exchange |

## Other sim configs

Development and smoke-test configs. Kept in the tree because
`run_batch.sh` and the `mantaray/README.md` quickstart reference some of
them.

| File | Region | Robots | Purpose |
|---|---|---|---|
| `lbl_simple_arctic_sim.json` | Beaufort | 1 | Minimal single-agent smoke test; canonical example in [`CLAUDE.md`](../../CLAUDE.md) |
| `lbl_simple_arctic_diver_sim.json` | Beaufort | 1 | Single diver variant |
| `lbl_simple_sim.json` | Monterey | 1 | Minimal single-agent, warm-water env |
| `lbl_sim.json` | Monterey | 2 | Two-agent LBL geometry |
| `beaufort_floats_sim.json` | Beaufort | 5 | Small-fleet variant |
| `random_sim.json` | Monterey | 4 | Randomized placement scratch |

## Env configs

Referenced by `env_config_file` from the sim configs above; not
runnable directly.

| File | Region |
|---|---|
| `beaufort_summer.json` | Beaufort Sea, summer |
| `fram_strait_summer.json` | Fram Strait, summer |
| `monterey.json` | Monterey Bay |

## Batch runs

`run_batch.sh` accepts one or more sim configs and runs them in
sequence:

```bash
cd mantaray && ./sim_config/run_batch.sh \
    sim_config/lbl_sim.json \
    sim_config/beaufort_floats_sim.json
```
