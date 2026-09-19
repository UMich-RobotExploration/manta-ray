# Data

Environment grids consumed by `mantaray_core`. Each region is a
self-contained bundle of bathymetry, sound-speed profile (SSP), and
depth-averaged current fields, in the flat `.npy` layout the C++
`Grid2D`/`Grid3D` loaders expect. The layout contract lives in
[`pymantaray/acoustics/grid_exporters.py`](../../pymantaray/acoustics/grid_exporters.py).

## Regions shipped

The three regions below are checked in so the repo can be run
end-to-end without pulling HYCOM first. Total on-disk footprint is
~35 MB.

| Region | Env config | Paper role |
|---|---|---|
| `beaufort_summer/` | [`beaufort_summer.json`](../sim_config/beaufort_summer.json) | Beaufort Sea fleet-week run ([`paper_beaufort_fleet_week_sim.json`](../sim_config/paper_beaufort_fleet_week_sim.json)) |
| `fram_strait_summer/` | [`fram_strait_summer.json`](../sim_config/fram_strait_summer.json) | Fram Strait fleet-week run ([`paper_fram_strait_fleet_week_sim.json`](../sim_config/paper_fram_strait_fleet_week_sim.json)) |
| `monterey/` | [`monterey.json`](../sim_config/monterey.json) | Warm-water dev region for LBL smoke tests |

## Directory layout

Every region follows the same shape:

```
mantaray/data/<region>/
  bathymetry/
    bathymetry.npy         # 2D depth grid (m), lon-major
    x_coords.npy           # longitude axis
    y_coords.npy           # latitude axis
  ssp/
    ssp.npy                # 3D sound-speed grid (m/s), lon-major then depth
    x_coords.npy
    y_coords.npy
    depth_coords.npy
  current/
    u.npy                  # eastward current (m/s), 3D
    v.npy                  # northward current (m/s), 3D
    x_coords.npy
    y_coords.npy
    z_coords.npy
```

Flat memory layouts (as read by C++):

- Grid2D: `ix * ny + iy`
- Grid3D: `ix * ny * nz + iy * nz + iz`

## Regenerating grids

Run the ingest pipeline in `pymantaray/acoustics/`; see that
subproject's [README](../../pymantaray/acoustics/README.md). Fresh
pulls default to a single timestep for the requested region.

## NaN handling

Grids are pre-filled at write time so the C++ side never has to
special-case gaps:

- Currents: NaNs (land / out-of-domain cells) are filled with `0`.
- SSP: below-bathymetry cells are filled with the deepest valid value
  along the water column (zero-order hold), so ray-trace queries into
  those cells return a physically reasonable speed rather than NaN.
