# pymantaray/acoustics/ — Ocean-data ingest

Pulls HYCOM reanalysis products via `oceanbench` and writes `.npy`
grids in the flat layout the C++ simulator expects. This is the first
stage of the [MantaRay pipeline](../../README.md#data-flow).

## Setup

```bash
cd pymantaray/acoustics
uv sync
```

Python 3.12. This project is fully independent from
[`pymantaray/localization/`](../localization/README.md) (which uses
Python 3.11).

## Modules

Scripts are scratch workbooks — region and season presets live at the
top of each file. Edit and re-run rather than adding CLI flags.

| Module | Purpose |
|---|---|
| [`oceanbench_data.py`](oceanbench_data.py) | Top-level driver. Region/season presets, pulls HYCOM through the oceanbench provider, calls the grid exporters. |
| [`oceanbench_process.py`](oceanbench_process.py) | Region extraction, kriging, SSP processing helpers. Mixed transform + plot module (the `_process` suffix is a repo convention). |
| [`grid_exporters.py`](grid_exporters.py) | Owns the C++ contract. Writes bathymetry / SSP / currents in the flat layout `mantaray_core` reads. Dim contract is validated before writing. |
| [`readers_1_8.py`](readers_1_8.py) | HYCOM v1.8 reanalysis file readers. |
| [`plot_ssp_comparison.py`](plot_ssp_comparison.py) | Produces paper Figure 4 — SSP comparison between Beaufort Sea and Fram Strait. |
| [`debug_raytrace.py`](debug_raytrace.py) | Bellhop ray-trace diagnostics against exported grids. |
| [`shadow_candidates_process.py`](shadow_candidates_process.py) | Scans for acoustic shadow-zone geometries in a given SSP field. |

## C++ contract (grid layout)

The C++ simulator reads these grids as flat float arrays. **This layout
is load-bearing** — a misconfigured xarray must fail here, not silently
scramble axes downstream.

| Grid | Layout | Indexing |
|---|---|---|
| Bathymetry (Grid2D) | `ix * ny + iy` | lon-major |
| SSP, currents (Grid3D) | `ix * ny * nz + iy * nz + iz` | lon-major then depth |

Dim names on the xarray input are validated as `(lat, lon, depth)`
before writing. Output filenames: `bathy.npy`, `ssp.npy`, `current.npy`
under `mantaray/data/<region>/`.

## Data flow

```
oceanbench (HYCOM GLBv0.08)
        │
        ▼
oceanbench_data.py + oceanbench_process.py
        │
        ▼
grid_exporters.py  ──►  mantaray/data/<region>/{bathy,ssp,current}.npy
                                                    │
                                                    ▼
                                          consumed by mantaray_core
```

## Reproducing paper Figure 4

```bash
uv run python plot_ssp_comparison.py
```

The script's `ENV_DIRS` list at the top selects the regions to compare;
change it to plot other environments once their SSP grids are on disk.

## Conventions

- Scripts are not CLI-driven; edit path / region constants at the top.
- Diagnostic PNG dumps land in `arrivalDataDebug*/` — these are
  gitignored and safe to remove between runs.
- New dataset pulls default to a **single timestep and the requested
  region** — never fetch a full year of the global grid by accident.
