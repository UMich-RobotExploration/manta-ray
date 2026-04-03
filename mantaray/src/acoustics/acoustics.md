# Acoustics

This internal library wraps BellhopCUDA to provide underwater acoustic ray
tracing for the manta-ray simulation. It handles environment setup (bathymetry,
sound speed profiles, source/receiver placement) and arrival extraction.

## Key Classes

- **AcousticsBuilder** — Configures and owns the Bellhop simulation state:
  bathymetry, altimetry, SSP, agent positions, and beam fan geometry. Provides
  `updateSource()` / `updateReceiver()` to reposition agents between runs, and
  `rebuildBeam()` for iterative beam refinement.

- **BhContext** — RAII wrapper around `bhcParams` and `bhcOutputs`. Manages
  the Bellhop init/setup lifecycle so callers don't touch raw bellhop memory.

- **Arrival** — Extracts arrival data from Bellhop output arrays. Supports
  fastest-arrival, largest-amplitude, and full debug dump modes.
  `getFastestArrival(bool directPathOnly)` optionally filters out multipath
  (bounced) arrivals.

- **Grid / Grid2D / Grid3D** — Axis-aligned grids for bathymetry and SSP data
  with bilinear interpolation.

## Bellhop Integration Notes

- Only single-source, single-receiver configurations are supported.
- Ray arrays are pre-allocated for the maximum beam count at construction to
  avoid bellhop memory budget errors during iterative refinement.
- `bhc::writeout()` segfaults in 3D ray mode due to a null pointer in
  bellhop's `Ray::Writeout`. Use `bhc::writeenv()` only for debug output.
